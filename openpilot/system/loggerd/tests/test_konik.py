from types import SimpleNamespace

import jwt
import pytest
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric import ec

from openpilot.common import konik_api as api
from openpilot.common.hardware.hw import Paths
from openpilot.system.loggerd import konik
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr


class FakeParams:
  def __init__(self):
    self.values = {'DongleId': 'comma-id', 'KonikUploadsEnabled': True, 'IsOffroad': True}

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.get(key))

  def put(self, key, value, **kwargs):
    self.values[key] = value


@pytest.fixture
def context(tmp_path, mocker):
  params = FakeParams()
  device = SimpleNamespace(networkType=konik.NetworkType.wifi, networkMetered=False)
  client = mocker.Mock(dongle_id='konik-id')
  client.get_token.return_value = 'konik-token'
  client.get.return_value = SimpleNamespace(status_code=200, json=lambda: {'url': 'https://storage.test/file', 'headers': {}})
  mocker.patch.object(konik, 'KonikApi', return_value=client)
  mocker.patch.object(konik, 'register', return_value='konik-id')
  put = mocker.patch.object(konik.requests, 'put', return_value=SimpleNamespace(status_code=200))
  worker = konik.KonikUploader(params, str(tmp_path), lambda: device)
  segment = tmp_path / '00000001--abc--0'
  segment.mkdir()
  return SimpleNamespace(params=params, device=device, worker=worker, segment=segment, client=client, put=put)


def file(context, name='qlog', marker=None):
  path = context.segment / name
  path.write_bytes(b'drive data')
  if marker:
    setxattr(str(path), marker, b'1')
  return path


def test_comma_uploaded_file_still_uploads_to_konik(context):
  path = file(context, marker='user.upload')
  assert context.worker.step() is True
  assert getxattr(str(path), 'user.upload') == b'1'
  assert getxattr(str(path), 'user.konik.upload') == b'1'
  assert context.params.get('DongleId') == 'comma-id'
  assert context.client.get.call_args.args[0] == 'v1.4/konik-id/upload_url/'
  assert context.client.get.call_args.kwargs['path'].endswith('/qlog.zst')
  assert context.worker.step() is None
  assert context.put.call_count == 1


def test_konik_upload_does_not_acknowledge_comma(context):
  path = file(context)
  assert context.worker.step() is True
  assert getxattr(str(path), 'user.upload') is None
  assert getxattr(str(path), 'user.sunny.upload') is None


@pytest.mark.parametrize('status', [401, 403, 412, 429, 500])
def test_rejected_signing_request_remains_pending(context, status):
  path = file(context)
  context.client.get.return_value.status_code = status
  assert context.worker.step() is False
  assert getxattr(str(path), 'user.konik.upload') is None
  context.put.assert_not_called()


@pytest.mark.parametrize('status', [401, 403, 429, 500])
def test_failed_upload_retries_without_marking(context, status):
  path = file(context)
  context.put.return_value.status_code = status
  assert context.worker.step() is False
  assert getxattr(str(path), 'user.konik.upload') is None
  context.put.return_value.status_code = 200
  assert context.worker.step() is None  # backoff is independent of comma uploads
  context.worker.retry_at[str(path)] = 0
  assert context.worker.step() is True
  assert context.put.call_count == 2


def test_konik_queued_worker_acknowledges_file_without_retry(context):
  path = file(context)
  context.client.get.return_value.json = lambda: {'url': 'https://api.konik.ai/upload/file', 'headers': {}}
  context.put.return_value = SimpleNamespace(status_code=403, text='Queued Worker', url='https://api.konik.ai/upload/file')
  assert context.worker.step() is True
  assert getxattr(str(path), 'user.konik.upload') == b'1'
  assert getxattr(str(path), 'user.upload') is None
  assert context.worker.step() is None
  assert context.put.call_count == 1


@pytest.mark.parametrize('url,text', [
  ('https://api.konik.ai/upload/file', 'Forbidden'),
  ('https://api.konik.ai/upload/file', 'Permission denied'),
  ('https://storage.test/file', 'Queued Worker'),
  ('http://api.konik.ai/upload/file', 'Queued Worker'),
])
def test_other_forbidden_responses_are_not_queued_acknowledgements(context, url, text):
  path = file(context)
  context.put.return_value = SimpleNamespace(status_code=403, text=text, url=url)
  assert context.worker.step() is False
  assert getxattr(str(path), 'user.konik.upload') is None


@pytest.mark.parametrize('blocked', ['onroad', 'disabled', 'metered', 'cellular', 'offline'])
def test_upload_requires_enabled_offroad_unmetered_wifi(context, blocked):
  file(context)
  if blocked == 'onroad':
    context.params.values['IsOffroad'] = False
  elif blocked == 'disabled':
    context.params.values['KonikUploadsEnabled'] = False
  elif blocked == 'metered':
    context.device.networkMetered = True
  else:
    context.device.networkType = konik.NetworkType.cell4G if blocked == 'cellular' else konik.NetworkType.none
  assert context.worker.step() is None
  context.put.assert_not_called()
  context.client.get.assert_not_called()


def test_locked_segments_are_skipped(context):
  file(context)
  file(context, 'rlog.lock')
  assert context.worker.step() is None
  context.client.get.assert_not_called()


def test_all_route_logs_and_road_cameras_are_archived(context):
  names = ['qlog.zst', 'qcamera.ts', 'rlog.zst', 'fcamera.hevc', 'ecamera.hevc']
  paths = [file(context, name) for name in names]
  file(context, 'dcamera.hevc')
  file(context, 'arbitrary.txt')
  for _ in names:
    assert context.worker.step() is True
  assert context.worker.step() is None
  assert all(getxattr(str(p), 'user.konik.upload') == b'1' for p in paths)
  assert context.put.call_count == len(names)


def test_disable_while_signing_prevents_transfer(context):
  path = file(context)
  response = context.client.get.return_value
  def disable(*args, **kwargs):
    context.params.values['KonikUploadsEnabled'] = False
    return response
  context.client.get.side_effect = disable
  assert context.worker.step() is None
  context.put.assert_not_called()
  assert getxattr(str(path), 'user.konik.upload') is None


def test_onroad_transition_aborts_stream_and_does_not_mark(context):
  path = file(context)
  def transfer(*args, **kwargs):
    context.params.values['IsOffroad'] = False
    kwargs['data'].read(1)
    raise AssertionError('stream read must abort')
  context.put.side_effect = transfer
  assert context.worker.step() is None
  assert getxattr(str(path), 'user.konik.upload') is None


def test_pending_head_failure_does_not_starve_other_files(context):
  file(context, 'qlog.zst')
  video = file(context, 'qcamera.ts')
  context.client.get.return_value.status_code = 500
  assert context.worker.step() is False
  context.client.get.return_value.status_code = 200
  assert context.worker.step() is True
  assert getxattr(str(video), 'user.konik.upload') == b'1'


def test_existing_konik_id_never_replaces_comma_id(mocker):
  params = FakeParams()
  params.values['KonikDongleId'] = 'konik-existing'
  client = mocker.patch.object(api, 'KonikApi')
  assert api.register(params) == 'konik-existing'
  assert params.get('DongleId') == 'comma-id'
  client.assert_not_called()


def test_registration_uses_fixed_host_and_ecdsa_without_global_identity(tmp_path, mocker):
  params = FakeParams()
  key = ec.generate_private_key(ec.SECP256R1())
  keydir = tmp_path / 'comma'
  keydir.mkdir()
  (keydir / 'id_ecdsa').write_bytes(key.private_bytes(serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8,
                                                    serialization.NoEncryption()))
  public = key.public_key().public_bytes(serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo)
  (keydir / 'id_ecdsa.pub').write_bytes(public)
  mocker.patch.object(Paths, 'persist_root', return_value=str(tmp_path))
  mocker.patch.object(api.HARDWARE, 'get_serial', return_value='serial')
  mocker.patch.object(api.HARDWARE, 'get_imei', return_value='imei')
  request = mocker.patch('requests.request', return_value=SimpleNamespace(status_code=200, json=lambda: {'dongle_id': 'konik-new'}))
  assert api.register(params) == 'konik-new'
  assert params.get('DongleId') == 'comma-id'
  assert params.get('KonikDongleId') == 'konik-new'
  assert request.call_args.args[1] == 'https://api.konik.ai/v2/pilotauth/'
  token = request.call_args.kwargs['params']['register_token']
  assert jwt.decode(token, public, algorithms=['ES256'])['register'] is True
  assert api.pairing_url(params).startswith('https://stable.konik.ai/?pair=')


@pytest.mark.parametrize('status', [403, 500])
def test_registration_rejection_leaves_comma_identity_untouched(mocker, status):
  params = FakeParams()
  client = mocker.Mock()
  client.get_key_pair.return_value = ('RS256', 'private', 'public')
  client.get_token.return_value = 'token'
  client.api_get.return_value = SimpleNamespace(status_code=status)
  mocker.patch.object(api, 'KonikApi', return_value=client)
  mocker.patch.object(api.HARDWARE, 'get_serial', return_value='serial')
  mocker.patch.object(api.HARDWARE, 'get_imei', return_value='imei')
  with pytest.raises(RuntimeError):
    api.register(params)
  assert params.get('DongleId') == 'comma-id'
  assert params.get('KonikDongleId') is None


def test_empty_device_registers_before_first_drive(context, mocker):
  registration = mocker.patch.object(konik, 'register', return_value='konik-id')
  assert context.worker.step() is None
  registration.assert_called_once_with(context.params)
  context.put.assert_not_called()


@pytest.mark.parametrize('first', ['comma', 'konik'])
def test_both_uploaders_complete_same_file_independently(context, monkeypatch, mocker, first):
  from openpilot.system.loggerd import uploader
  path = file(context)
  comma = uploader.Uploader('comma-id', str(context.segment.parent))
  comma.params = context.params
  comma.api = mocker.Mock()
  comma.api.get.return_value = SimpleNamespace(status_code=200, text='{"url":"https://comma.test/file","headers":{}}')
  monkeypatch.setattr(uploader, 'fake_upload', True)
  actions = {'comma': lambda: comma.step(konik.NetworkType.wifi, False), 'konik': context.worker.step}
  second = 'konik' if first == 'comma' else 'comma'
  assert actions[first]() is True
  assert actions[second]() is True
  assert getxattr(str(path), 'user.upload') == b'1'
  assert getxattr(str(path), 'user.konik.upload') == b'1'
  assert context.params.get('DongleId') == 'comma-id'


def test_missing_device_state_pauses(context):
  file(context)
  context.worker.device_state = lambda: None
  assert context.worker.step() is None
  context.put.assert_not_called()


def test_success_marker_survives_worker_restart(context):
  from openpilot.system.loggerd.xattr_cache import _cached_attributes
  file(context)
  assert context.worker.step() is True
  _cached_attributes.clear()
  replacement = konik.KonikUploader(context.params, str(context.segment.parent), lambda: context.device)
  assert replacement.step() is None
  assert context.put.call_count == 1


@pytest.mark.parametrize('change', ['metered', 'offline'])
def test_network_change_aborts_stream(context, change):
  path = file(context)
  def transfer(*args, **kwargs):
    if change == 'metered':
      context.device.networkMetered = True
    else:
      context.device.networkType = konik.NetworkType.none
    kwargs['data'].read(1)
    raise AssertionError('read should have stopped')
  context.put.side_effect = transfer
  assert context.worker.step() is None
  assert getxattr(str(path), 'user.konik.upload') is None


def test_registration_outage_retries_even_with_no_routes(context, mocker):
  registration = mocker.patch.object(konik, 'register', side_effect=RuntimeError('server down'))
  assert context.worker.step() is False
  assert context.worker.step() is None
  assert registration.call_count == 1
  context.worker.registration_retry_at = 0
  registration.side_effect = None
  registration.return_value = 'konik-id'
  assert context.worker.step() is None
  assert registration.call_count == 2
  assert context.params.get('DongleId') == 'comma-id'


@pytest.mark.parametrize('invalid', [None, '', 'UnregisteredDevice', '../comma', 123])
def test_invalid_registration_response_not_persisted(mocker, invalid):
  params = FakeParams()
  client = mocker.Mock()
  client.get_key_pair.return_value = ('RS256', 'private', 'public')
  client.api_get.return_value = SimpleNamespace(status_code=200, json=lambda: {'dongle_id': invalid})
  mocker.patch.object(api, 'KonikApi', return_value=client)
  mocker.patch.object(api.HARDWARE, 'get_serial', return_value='serial')
  mocker.patch.object(api.HARDWARE, 'get_imei', return_value='imei')
  with pytest.raises(RuntimeError):
    api.register(params)
  assert params.get('KonikDongleId') is None
  assert params.get('DongleId') == 'comma-id'


def test_active_upload_progress_is_reported_and_cleared(context):
  file(context, 'qcamera.ts')
  def transfer(*args, **kwargs):
    current = context.params.get('KonikCurrentUpload')
    assert current['path'] == '00000001--abc--0/qcamera.ts'
    assert current['progress'] == 0
    kwargs['data'].read()
    assert context.params.get('KonikCurrentUpload')['progress'] == 1
    return SimpleNamespace(status_code=200)
  context.put.side_effect = transfer
  assert context.worker.step() is True
  assert context.params.get('KonikCurrentUpload') == {}


def test_failed_transfer_clears_active_progress(context):
  file(context)
  context.put.side_effect = OSError('network failure')
  assert context.worker.step() is False
  assert context.params.get('KonikCurrentUpload') == {}


def request_file(context, name='qcamera.ts'):
  from openpilot.system.loggerd import konik_queue
  return konik_queue.enqueue(context.params, str(context.segment.parent), [{'fn': context.segment.name + '/' + name}])['items'][0]


def test_requested_completed_file_reuploads_using_fixed_konik_signing(context):
  path = file(context, 'qcamera.ts', marker='user.konik.upload')
  item = request_file(context)
  assert context.worker.step() is True
  assert getxattr(str(path), 'user.konik.request') == item['id'].encode()
  assert context.client.get.call_count == 1
  assert context.put.call_count == 1
  assert getxattr(str(path), 'user.upload') is None
  assert context.worker.step() is None


def test_cancelled_inflight_upload_never_marks_complete(context):
  from openpilot.system.loggerd import konik_queue
  path = file(context, 'qcamera.ts')
  def transfer(*args, **kwargs):
    items = konik_queue.queue_items(context.params, str(context.segment.parent), 'konik-id')
    konik_queue.cancel(context.params, str(context.segment.parent), 'konik-id', items[0]['id'])
    return SimpleNamespace(status_code=200)
  context.put.side_effect = transfer
  assert context.worker.step() is None
  assert getxattr(str(path), 'user.konik.upload') is None
  assert context.worker.step() is None


def test_manual_request_has_priority_over_automatic_backups(context):
  file(context, 'qlog.zst')
  selected = file(context, 'dcamera.hevc')
  request_file(context, 'dcamera.hevc')
  assert context.worker.next_file()[1] == str(selected)
  assert context.worker.step() is True


def test_viewed_route_preview_precedes_other_full_quality_files(context):
  from openpilot.system.loggerd import konik_queue
  file(context, 'fcamera.hevc')
  folder = context.segment.parent / '00000002--recent--0'
  folder.mkdir()
  preview = folder / 'qcamera.ts'
  preview.write_bytes(b'preview')
  konik_queue.view_route(context.params, 'konik-id', 'konik-id|00000002--recent')
  assert context.worker.next_file()[1] == str(preview)


def test_new_request_during_upload_is_not_lost_when_worker_finishes(context):
  from openpilot.system.loggerd import konik_queue
  file(context, 'qcamera.ts')
  file(context, 'rlog')
  request_file(context)
  def transfer(*args, **kwargs):
    request_file(context, 'rlog.zst')
    return SimpleNamespace(status_code=200)
  context.put.side_effect = transfer
  assert context.worker.step() is True
  items = konik_queue.queue_items(context.params, str(context.segment.parent), 'konik-id')
  assert len(items) == 1
  assert items[0]['path'].endswith('rlog')
