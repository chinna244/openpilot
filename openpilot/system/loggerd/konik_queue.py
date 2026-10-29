"""Konik-only recorded-file requests and cancellation state.

Only the RPC process writes control state. Upload completion stays on each file,
so a worker finishing one job cannot overwrite a newly received website request.
"""
import errno
import hashlib
import os
from pathlib import Path
import re
from urllib.parse import quote
import uuid

from openpilot.common.konik_api import API_HOST
from openpilot.system.loggerd.uploader import listdir_by_creation
from openpilot.system.loggerd.xattr_cache import _getxattr

UPLOAD_ATTR_NAME = 'user.konik.upload'
REQUEST_ATTR_NAME = 'user.konik.request'
ARCHIVE_FILES = ('qlog', 'qlog.zst', 'qcamera.ts', 'rlog', 'rlog.zst', 'fcamera.hevc', 'ecamera.hevc')
REQUEST_FILES = (*ARCHIVE_FILES, 'dcamera.hevc')
MAX_REQUESTS = 2000
MAX_CANCELLED = 10000


def marker(path: str, attr: str) -> bytes | None:
  try:
    return _getxattr(path, attr)
  except OSError as e:
    if e.errno in (errno.ENODATA, getattr(errno, 'ENOATTR', errno.ENODATA)):
      return None
    raise


def valid_route(route) -> bool:
  return isinstance(route, str) and len(route) <= 160 and re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*--[A-Za-z0-9_-]+', route) is not None


def resolve_recording(root: str, fn, *, allow_locked: bool = False) -> tuple[str, str] | None:
  if not isinstance(fn, str) or '\\' in fn or len(fn) > 256:
    return None
  parts = fn.split('/')
  if len(parts) != 2 or parts[1] not in REQUEST_FILES:
    return None
  route, sep, segment = parts[0].rpartition('--')
  if not sep or not valid_route(route) or not segment.isdecimal() or len(segment) > 8:
    return None
  directory = Path(root) / parts[0]
  path = directory / parts[1]
  try:
    if directory.is_symlink() or path.is_symlink() or (not allow_locked and any(p.name.endswith('.lock') for p in directory.iterdir())):
      return None
    if not path.exists() and parts[1].endswith('.zst'):
      path = directory / parts[1][:-4]
    if path.is_symlink() or not path.is_file() or not path.resolve().is_relative_to(Path(root).resolve()):
      return None
    return str(path.relative_to(root)), str(path)
  except OSError:
    return None


def pending_upload_files(root: str):
  for folder in listdir_by_creation(root):
    route, separator, segment = folder.rpartition('--')
    directory = Path(root) / folder
    if not separator or not valid_route(route) or not segment.isdecimal() or directory.is_symlink():
      continue
    try:
      names = os.listdir(directory)
      if any(name.endswith('.lock') for name in names):
        continue
      for name in ARCHIVE_FILES:
        path = directory / name
        if name in names and not path.is_symlink() and path.is_file() and marker(str(path), UPLOAD_ATTR_NAME) != b'1':
          yield folder + '/' + name, str(path)
    except OSError:
      continue


def control(params) -> dict:
  value = params.get('KonikUploadControl') or {}
  return {
    'requested': [dict(j) for j in value.get('requested', []) if isinstance(j, dict)],
    'cancelled': list(value.get('cancelled', [])),
    'viewed': list(value.get('viewed', [])),
  }


def pending_requests(state: dict, root: str):
  for job in state['requested']:
    result = resolve_recording(root, job.get('key'))
    if result is None or result[0] in state['cancelled']:
      continue
    key, path = result
    try:
      if marker(path, REQUEST_ATTR_NAME) != job['id'].encode():
        yield job, key, path
    except OSError:
      continue


def automatic_id(key: str) -> str:
  return 'backup-' + hashlib.sha256(key.encode()).hexdigest()


def queue_item(key: str, path: str, dongle_id: str, job: dict | None = None, current: dict | None = None) -> dict:
  folder, name = os.path.split(key)
  route, _, segment = folder.rpartition('--')
  remote_name = name + '.zst' if name in ('qlog', 'rlog') else name
  job_id = job['id'] if job else automatic_id(key)
  active = bool(current and current.get('path') == key and current.get('id', job_id) == job_id)
  url = f'{API_HOST}/queue/{dongle_id}/{quote(route)}/{segment}/{quote(remote_name)}'
  return {'path': key, 'url': url, 'headers': {}, 'created_at': int(os.stat(path).st_mtime * 1000), 'id': job_id,
          'retry_count': 0, 'current': active, 'progress': current.get('progress', 0.0) if active else 0.0,
          'allow_cellular': False, 'priority': job.get('priority', 0) if job else 99}


def queue_items(params, root: str, dongle_id: str) -> list[dict]:
  state = control(params)
  current = params.get('KonikCurrentUpload') or {}
  items = []
  requested = set()
  for job, key, path in pending_requests(state, root):
    try:
      items.append(queue_item(key, path, dongle_id, job, current))
      requested.add(key)
    except OSError:
      continue
  for key, path in pending_upload_files(root):
    if key not in requested and key not in state['cancelled']:
      try:
        items.append(queue_item(key, path, dongle_id, current=current))
      except OSError:
        continue
  return items


def enqueue(params, root: str, files_data: list[dict]) -> dict:
  if not isinstance(files_data, list) or len(files_data) > MAX_REQUESTS:
    raise TypeError('request must contain at most 2000 recorded files')
  state = control(params)
  state['requested'] = [j for j, _, _ in pending_requests(state, root)]
  existing = {j['key'] for j in state['requested']}
  items, failed = [], []
  for file in files_data:
    fn = file.get('fn') if isinstance(file, dict) else None
    result = resolve_recording(root, fn)
    priority = file.get('priority', 0) if isinstance(file, dict) else None
    if result is None or type(priority) is not int or not 0 <= priority <= 99:
      failed.append(fn if isinstance(fn, str) else '')
      continue
    key, path = result
    if key in existing:
      continue
    if len(state['requested']) >= MAX_REQUESTS:
      failed.append(fn)
      continue
    job = {'key': key, 'id': 'request-' + uuid.uuid4().hex, 'priority': priority}
    state['requested'].append(job)
    existing.add(key)
    state['cancelled'] = [k for k in state['cancelled'] if k != key]
    items.append(queue_item(key, path, params.get('KonikDongleId') or '', job))
  params.put('KonikUploadControl', state, block=True)
  return {'enqueued': len(items), 'items': items, **({'failed': failed} if failed else {})}


def cancel(params, root: str, dongle_id: str, upload_id) -> dict:
  ids = [upload_id] if isinstance(upload_id, str) else upload_id
  if not isinstance(ids, list) or len(ids) > MAX_CANCELLED or any(not isinstance(i, str) for i in ids):
    raise TypeError('upload_id must be a string or a list of at most 10000 strings')
  ids = set(ids)
  keys = {i['path'] for i in queue_items(params, root, dongle_id) if i['id'] in ids}
  if not keys:
    return {'success': 0, 'error': 'not found'}
  state = control(params)
  cancelled = {key for key in state['cancelled'] if resolve_recording(root, key, allow_locked=True) is not None} | keys
  if len(cancelled) > MAX_CANCELLED:
    raise RuntimeError('too many cancelled files; request cancelled recordings again to resume them')
  state['cancelled'] = sorted(cancelled)
  state['requested'] = [j for j in state['requested'] if j['key'] not in keys]
  params.put('KonikUploadControl', state, block=True)
  return {'success': 1}


def view_route(params, dongle_id: str, route: str) -> dict:
  if not isinstance(route, str):
    raise TypeError('route must be a string')
  if '|' in route:
    identity, route = route.split('|', 1)
    if identity != dongle_id:
      raise ValueError('route belongs to another device')
  if not valid_route(route):
    raise ValueError('invalid route')
  state = control(params)
  state['viewed'] = ([r for r in state['viewed'] if r != route] + [route])[-10:]
  params.put('KonikUploadControl', state, block=True)
  return {'success': 1}
