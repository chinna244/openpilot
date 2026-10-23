#!/usr/bin/env python3
"""Independent, offroad Wi-Fi archive uploads. No Konik remote-command connection."""
import os
import errno
import threading
import time

import requests

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.api.konik import KonikApi, register
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.utils import CallbackReader, get_upload_stream
from openpilot.system.loggerd.uploader import listdir_by_creation
from openpilot.system.loggerd.xattr_cache import setxattr

NetworkType = log.DeviceState.NetworkType
UPLOAD_ATTR_NAME = 'user.konik.upload'
ARCHIVE_FILES = ('qlog', 'qlog.zst', 'qcamera.ts', 'rlog', 'rlog.zst', 'fcamera.hevc', 'ecamera.hevc')


def pending_upload_files(root: str):
  """Read markers fresh: the queue RPC runs in a different process from uploads."""
  for segment in listdir_by_creation(root):
    directory = os.path.join(root, segment)
    if os.path.islink(directory):
      continue
    try:
      names = os.listdir(directory)
      if any(name.endswith('.lock') for name in names):
        continue
      for name in ARCHIVE_FILES:
        path = os.path.join(directory, name)
        if name not in names or os.path.islink(path) or not os.path.isfile(path):
          continue
        try:
          if os.getxattr(path, UPLOAD_ATTR_NAME) == b'1':
            continue
        except OSError as e:
          if e.errno not in (errno.ENODATA, getattr(errno, 'ENOATTR', errno.ENODATA)):
            raise
        yield os.path.join(segment, name), path
    except OSError:
      # The normal free-space deleter can remove a segment during a scan.
      continue


class UploadPaused(Exception):
  pass


class KonikUploader:
  def __init__(self, params: Params, root: str, device_state, exit_event: threading.Event | None = None):
    self.params = params
    self.root = root
    self.device_state = device_state
    self.exit_event = exit_event or threading.Event()
    self.retry_at: dict[str, float] = {}
    self.registration_retry_at = 0.0

  def allowed(self) -> bool:
    device = self.device_state()
    return (device is not None and not self.exit_event.is_set() and self.params.get_bool('KonikUploadsEnabled') and self.params.get_bool('IsOffroad')
            and device.networkType == NetworkType.wifi and not device.networkMetered)

  def _check_policy(self, *_):
    if not self.allowed():
      raise UploadPaused()

  def _status(self, status: str):
    if self.params.get('KonikUploadStatus') != status:
      self.params.put('KonikUploadStatus', status, block=True)

  def next_file(self) -> tuple[str, str] | None:
    pending = set()
    candidate = None
    now = time.monotonic()
    for key, path in pending_upload_files(self.root):
      pending.add(path)
      if candidate is None and now >= self.retry_at.get(path, 0):
        candidate = (key, path)
    self.retry_at = {path: retry for path, retry in self.retry_at.items() if path in pending}
    return candidate

  def step(self) -> bool | None:
    if not self.allowed():
      self._status('waiting for parked, unmetered Wi-Fi')
      return None
    if time.monotonic() < self.registration_retry_at:
      return None
    try:
      dongle_id = register(self.params)
      self._check_policy()
    except UploadPaused:
      self._status('paused')
      return None
    except Exception:
      cloudlog.exception('Konik registration failed')
      self.registration_retry_at = time.monotonic() + 60
      self._status('registration failed; will retry')
      return False
    item = self.next_file()
    if item is None:
      self._status('retry pending' if self.retry_at else 'backup up to date')
      return None
    key, path = item
    current = {'path': key, 'progress': 0.0}
    self.params.put('KonikCurrentUpload', current, block=True)
    last_progress = time.monotonic()
    try:
      self._status('uploading to Konik')
      api = KonikApi(dongle_id)
      token = api.get_token()
      self._check_policy()
      if key.endswith(('qlog', 'rlog')):
        key += '.zst'
      response = api.get(f'v1.4/{dongle_id}/upload_url/', timeout=15, path=key, access_token=token)
      self._check_policy()
      if response.status_code != 200:
        raise RuntimeError(f'Konik upload URL returned {response.status_code}')
      signed = response.json()
      stream, stream_size = get_upload_stream(path, key.endswith('.zst') and not path.endswith('.zst'))
      def progress(bytes_read):
        nonlocal last_progress
        self._check_policy()
        now = time.monotonic()
        if now - last_progress >= 1 or bytes_read >= stream_size:
          current['progress'] = min(bytes_read / max(stream_size, 1), 1.0)
          self.params.put('KonikCurrentUpload', current, block=True)
          last_progress = now
      with stream:
        self._check_policy()
        response = requests.put(signed['url'], headers=signed['headers'], data=CallbackReader(stream, progress), timeout=30)
      self._check_policy()
      if response.status_code not in (200, 201, 204):
        raise RuntimeError(f'Konik file upload returned {response.status_code}')
      setxattr(path, UPLOAD_ATTR_NAME, b'1')
      self.retry_at.pop(path, None)
      return True
    except UploadPaused:
      self._status('paused')
      return None
    except Exception:
      cloudlog.exception('Konik backup failed')
      self.retry_at[path] = time.monotonic() + 60
      self._status('backup failed; will retry')
      return False
    finally:
      self.params.put('KonikCurrentUpload', {}, block=True)


def main(exit_event: threading.Event | None = None):
  exit_event = exit_event or threading.Event()
  params = Params()
  params.put('KonikCurrentUpload', {}, block=True)
  sm = messaging.SubMaster(['deviceState'])
  def current_device_state():
    sm.update(0)
    return sm['deviceState'] if sm.alive['deviceState'] and sm.valid['deviceState'] else None

  uploader = KonikUploader(params, Paths.log_root(), current_device_state, exit_event)
  while not exit_event.is_set():
    sm.update(1000)
    # Ignore stale state so a lost network publisher cannot keep transfers active.
    if not sm.alive['deviceState'] or not sm.valid['deviceState']:
      exit_event.wait(1)
      continue
    result = uploader.step()
    exit_event.wait(0.1 if result is True else 5)


if __name__ == '__main__':
  main()
