#!/usr/bin/env python3
"""Independent, offroad Wi-Fi archive uploads. No Konik remote-command connection."""
import os
import threading
import time
from urllib.parse import urlparse

import requests

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.konik_api import KonikApi, register
from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.common.utils import CallbackReader, get_upload_stream
from openpilot.system.loggerd.xattr_cache import setxattr
from openpilot.system.loggerd import konik_queue
from openpilot.system.loggerd.konik_queue import ARCHIVE_FILES as ARCHIVE_FILES, UPLOAD_ATTR_NAME as UPLOAD_ATTR_NAME, pending_upload_files

NetworkType = log.DeviceState.NetworkType


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
    self._request = None
    self._active_key = None
    self._control_checked = 0.0
    self._cancelled = set()

  def allowed(self) -> bool:
    device = self.device_state()
    return (device is not None and not self.exit_event.is_set() and self.params.get_bool('KonikUploadsEnabled') and self.params.get_bool('IsOffroad')
            and device.networkType == NetworkType.wifi and not device.networkMetered)

  def _check_policy(self, *_, force: bool = False):
    if not self.allowed():
      raise UploadPaused()
    if self._active_key is not None:
      now = time.monotonic()
      if force or now - self._control_checked >= 0.5:
        self._cancelled = set(konik_queue.control(self.params)['cancelled'])
        self._control_checked = now
      if self._active_key in self._cancelled:
        raise UploadPaused()

  def _status(self, status: str):
    if self.params.get('KonikUploadStatus') != status:
      self.params.put('KonikUploadStatus', status, block=True)

  def next_file(self) -> tuple[str, str] | None:
    state = konik_queue.control(self.params)
    self._request = None
    now = time.monotonic()
    requests_pending = list(konik_queue.pending_requests(state, self.root))
    requested_keys = {key for _, key, _ in requests_pending}
    pending = set()
    candidate = None
    rank = None
    for job, key, path in requests_pending:
      pending.add(path)
      priority = (0, job['priority'])
      if now >= self.retry_at.get(path, 0) and (rank is None or priority < rank):
        candidate, rank, self._request = (key, path), priority, job
    for key, path in pending_upload_files(self.root):
      if key in requested_keys or key in state['cancelled']:
        continue
      pending.add(path)
      name = os.path.basename(path)
      route = key.split('/')[0].rpartition('--')[0]
      viewed = route in state['viewed']
      preview = name in ('qlog', 'qlog.zst', 'qcamera.ts')
      priority = (1 if viewed and preview else 2 if name.startswith('qlog') else 3 if preview else 4 if viewed else 5, 0)
      if now >= self.retry_at.get(path, 0) and (rank is None or priority < rank):
        candidate, rank = (key, path), priority
        self._request = None
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
      cancelled = konik_queue.control(self.params)['cancelled']
      self._status('retry pending' if self.retry_at else 'files cancelled; request them again to resume' if cancelled else 'backup up to date')
      return None
    key, path = item
    self._active_key = key
    self._control_checked = float('-inf')
    current = {'path': key, 'progress': 0.0, 'id': self._request['id'] if self._request else konik_queue.automatic_id(key)}
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
      queued = False
      if response.status_code == 403 and getattr(response, 'text', '').strip() == 'Queued Worker':
        # Konik's proxy acknowledges a queued upload with this comma-compatible response.
        endpoint = urlparse(response.url)
        queued = endpoint.scheme == 'https' and endpoint.hostname == 'api.konik.ai'
      if response.status_code not in (200, 201, 204) and not queued:
        raise RuntimeError(f'Konik file upload returned {response.status_code}')
      self._check_policy(force=True)
      if self._request is not None:
        setxattr(path, konik_queue.REQUEST_ATTR_NAME, self._request['id'].encode())
      setxattr(path, UPLOAD_ATTR_NAME, b'1')
      self.retry_at.pop(path, None)
      if queued:
        self._status('queued for processing on Konik')
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
      self._active_key = None
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
