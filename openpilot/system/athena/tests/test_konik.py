import json
import threading
from types import SimpleNamespace

import pytest
from websocket import ABNF, WebSocketTimeoutException

from openpilot.system.athena import konik


class FakeParams:
  def __init__(self):
    self.values = {'KonikUploadsEnabled': True, 'IsOffroad': True, 'KonikDongleId': 'konik-id',
                   'DongleId': 'comma-id', 'LastAthenaPingTime': 12345}

  def get(self, key):
    return self.values.get(key)

  def get_bool(self, key):
    return bool(self.get(key))

  def put(self, key, value, **kwargs):
    self.values[key] = value

  def put_bool(self, key, value, **kwargs):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


@pytest.fixture
def setup(mocker):
  params = FakeParams()
  end = threading.Event()
  ws = mocker.Mock()
  client = mocker.patch.object(konik, 'KonikApi').return_value
  client.get_token.return_value = 'own-token'
  connect = mocker.patch.object(konik, 'create_connection', return_value=ws)
  connection = konik.KonikConnection(params, end)
  return SimpleNamespace(params=params, end=end, ws=ws, client=client, connect=connect, connection=connection)


def frame(setup, opcode, payload):
  def recv(**kwargs):
    setup.end.set()
    return opcode, payload
  setup.ws.recv_data.side_effect = recv


def call(setup, method, params=None):
  return json.loads(konik.rpc.handle({'jsonrpc': '2.0', 'id': 1, 'method': method, 'params': params}, setup.connection.methods()))


def test_connect_uses_only_konik_identity_and_fixed_host(setup):
  frame(setup, ABNF.OPCODE_PING, b'')
  setup.connection.session()
  assert setup.connect.call_args.args == ('wss://athena.konik.ai/ws/v2/konik-id',)
  assert setup.connect.call_args.kwargs['cookie'] == 'jwt=own-token'
  assert setup.connect.call_args.kwargs['timeout'] <= 5
  assert setup.params.get('DongleId') == 'comma-id'
  assert setup.params.get('LastAthenaPingTime') == 12345
  setup.ws.close.assert_called_once()


def test_ping_state_is_separate_and_cleared_on_disconnect(setup):
  states = []
  def recv(**kwargs):
    if not states:
      states.append('ping')
      return ABNF.OPCODE_PING, b''
    assert setup.params.get('KonikLastAthenaPingTime') is not None
    assert setup.params.get('LastAthenaPingTime') == 12345
    setup.end.set()
    return ABNF.OPCODE_PONG, b''
  setup.ws.recv_data.side_effect = recv
  setup.connection.session()
  assert setup.params.get('KonikLastAthenaPingTime') is None
  assert setup.params.get('KonikConnectionStatus') == 'disconnected'


@pytest.mark.parametrize('method', ['startLocalProxy', 'uploadFileToUrl', 'getPublicKey', 'getSshAuthorizedKeys',
                                    'getParams', 'saveParams', 'listDataDirectory', 'reboot', 'ssh'])
def test_all_unapproved_remote_commands_are_rejected(setup, method):
  result = call(setup, method)
  assert result['error']['code'] == -32601


def test_rpc_dispatcher_is_not_comma_dispatcher(setup):
  methods = setup.connection.methods()
  assert methods is not konik.rpc.dispatcher
  assert set(methods) == {'echo', 'getVersion', 'getNetworkType', 'getNetworkMetered', 'startStream', 'listUploadQueue',
                          'cancelUpload', 'uploadFilesToUrls', 'setRouteViewed', 'getMessage'}
  assert call(setup, 'echo', ['hello'])['result'] == 'hello'


def test_live_view_uses_only_road_video_and_read_only_bridges(setup, mocker):
  wait = mocker.patch.object(konik, 'wait_for_webrtcd')
  post = mocker.patch.object(konik, 'post_stream_request', return_value={'sdp': 'answer'})
  result = call(setup, 'startStream', {'sdp': 'offer', 'enabled': True})
  assert result['result'] == {'sdp': 'answer'}
  wait.assert_called_once()
  body = post.call_args.args[0]
  assert body.cameras == ['wideRoad']
  assert body.konik is True
  assert body.bridge_services_in == []
  assert body.bridge_services_out == ['carState', 'deviceState']
  assert setup.params.get_bool('IsLiveStreaming')
  assert setup.params.get('DongleId') == 'comma-id'


@pytest.mark.parametrize('change', ['onroad', 'disabled'])
def test_stream_rejected_when_not_eligible(setup, mocker, change):
  setup.params.values['IsOffroad' if change == 'onroad' else 'KonikUploadsEnabled'] = False
  post = mocker.patch.object(konik, 'post_stream_request')
  result = call(setup, 'startStream', {'sdp': 'offer', 'enabled': True})
  assert 'error' in result
  post.assert_not_called()
  assert not setup.params.get_bool('IsLiveStreaming')


def test_ignition_transition_during_camera_start_blocks_stream(setup, mocker):
  def ignition():
    setup.params.values['IsOffroad'] = False
  mocker.patch.object(konik, 'wait_for_webrtcd', side_effect=ignition)
  post = mocker.patch.object(konik, 'post_stream_request')
  result = call(setup, 'startStream', {'sdp': 'offer', 'enabled': True})
  assert 'error' in result
  post.assert_not_called()


@pytest.mark.parametrize('values', [{'sdp': 2, 'enabled': True}, {'sdp': '', 'enabled': True},
                                    {'sdp': 'offer', 'enabled': 'true'}, {'sdp': 'x'*262145, 'enabled': True},
                                    {'sdp': 'offer', 'enabled': True, 'bridge_services_in': ['sendcan']}])
def test_invalid_stream_requests_cannot_reach_camera_service(setup, mocker, values):
  post = mocker.patch.object(konik, 'post_stream_request')
  assert 'error' in call(setup, 'startStream', values)
  post.assert_not_called()


def test_message_handling_recovers_after_malformed_rpc(setup):
  frames = [(ABNF.OPCODE_TEXT, b'{broken'),
            (ABNF.OPCODE_TEXT, json.dumps({'jsonrpc': '2.0', 'id': 9, 'method': 'echo', 'params': ['ok']}).encode())]
  def recv(**kwargs):
    result = frames.pop(0)
    if not frames:
      setup.end.set()
    return result
  setup.ws.recv_data.side_effect = recv
  setup.connection.session()
  sent = [json.loads(c.args[0]) for c in setup.ws.send.call_args_list]
  assert sent[0]['error']['code'] == -32700
  assert sent[1]['result'] == 'ok'


@pytest.mark.parametrize('change', ['disable', 'identity'])
def test_connection_ends_when_local_configuration_changes(setup, change):
  def recv(**kwargs):
    setup.params.values['KonikUploadsEnabled' if change == 'disable' else 'KonikDongleId'] = False if change == 'disable' else 'different-id'
    return ABNF.OPCODE_PONG, b''
  setup.ws.recv_data.side_effect = recv
  setup.connection.session()
  setup.ws.close.assert_called_once()
  assert setup.ws.recv_data.call_count == 1


def test_silent_connection_times_out_and_closes(setup, mocker):
  clock = iter([0, 71])
  mocker.patch.object(konik.time, 'monotonic', side_effect=lambda: next(clock))
  setup.ws.recv_data.side_effect = WebSocketTimeoutException()
  setup.connection.session()
  setup.ws.close.assert_called_once()


def test_failed_connect_leaves_comma_state_unchanged(setup):
  setup.connect.side_effect = ConnectionError('offline')
  with pytest.raises(ConnectionError):
    setup.connection.session()
  assert setup.params.get('LastAthenaPingTime') == 12345
  assert setup.params.get('KonikConnectionStatus') == 'disconnected'


def test_upload_queue_reflects_pending_konik_files_and_current_progress(setup, tmp_path, mocker):
  from openpilot.system.loggerd import konik as uploader
  root = tmp_path / 'logs'
  segment = root / '00000001--abc--2'
  segment.mkdir(parents=True)
  pending = segment / 'qcamera.ts'
  pending.write_bytes(b'pending video')
  done = segment / 'qlog.zst'
  done.write_bytes(b'uploaded')
  import os
  os.setxattr(done, uploader.UPLOAD_ATTR_NAME, b'1')
  mocker.patch.object(konik.Paths, 'log_root', return_value=str(root))
  setup.params.values['KonikCurrentUpload'] = {'path': '00000001--abc--2/qcamera.ts', 'progress': 0.5}
  result = call(setup, 'listUploadQueue')['result']
  assert len(result) == 1
  item = result[0]
  assert item['path'] == '00000001--abc--2/qcamera.ts'
  assert item['id']  # The website labels empty IDs as Offline and reconciles rows by ID.
  assert call(setup, 'listUploadQueue')['result'][0]['id'] == item['id']
  assert item['current'] is True
  assert item['progress'] == 0.5
  # Konik's queue UI derives route, segment and filename from URL components 3, 4, 5.
  from urllib.parse import urlsplit
  assert urlsplit(item['url']).path.split('/')[3:] == ['00000001--abc', '2', 'qcamera.ts']
  assert item['headers'] == {}
  assert setup.params.get('DongleId') == 'comma-id'
  os.setxattr(pending, uploader.UPLOAD_ATTR_NAME, b'1')
  assert call(setup, 'listUploadQueue')['result'] == []


def test_queue_rpc_cannot_select_arbitrary_directories(setup):
  result = call(setup, 'listUploadQueue', {'root': '/persist'})
  assert result['error']['code'] == -32602


def test_large_queue_response_is_fragmented_without_changing_json(setup):
  payload = konik.rpc.dumps_result(1, ['pending route'] * 2000)
  setup.connection.send_response(setup.ws, payload)
  frames = [entry.args[0] for entry in setup.ws.send_frame.call_args_list]
  assert len(frames) > 1
  assert frames[0].opcode == ABNF.OPCODE_TEXT
  assert all(f.opcode == ABNF.OPCODE_CONT for f in frames[1:])
  assert all(not f.fin for f in frames[:-1]) and frames[-1].fin
  assert b''.join(f.data for f in frames).decode() == payload



def test_unknown_cancel_does_not_modify_automatic_backup_queue(setup):
  setup.params.values['KonikCurrentUpload'] = {'path': 'route/qcamera.ts', 'progress': 0.5}
  result = call(setup, 'cancelUpload', {'upload_id': ['backup-id']})
  assert result['result']['success'] == 0
  assert setup.params.get('KonikCurrentUpload')['progress'] == 0.5
  assert setup.params.get_bool('KonikUploadsEnabled')


def make_recording(root, name='qcamera.ts', folder='00000001--abc--2'):
  path = root / folder / name
  path.parent.mkdir(parents=True, exist_ok=True)
  path.write_bytes(b'recording')
  return path


def test_website_requests_recordings_without_trusting_remote_upload_destination(setup, tmp_path, mocker):
  make_recording(tmp_path)
  mocker.patch.object(konik.Paths, 'log_root', return_value=str(tmp_path))
  response = call(setup, 'uploadFilesToUrls', {'files_data': [
    {'fn': '00000001--abc--2/qcamera.ts', 'url': 'http://localhost/private', 'headers': {'Authorization': 'untrusted'}, 'allow_cellular': True}
  ]})
  assert response['result']['enqueued'] == 1
  job = response['result']['items'][0]
  assert job['id'].startswith('request-')
  assert job['allow_cellular'] is False
  state = setup.params.get('KonikUploadControl')
  assert 'localhost' not in json.dumps(state)
  assert 'untrusted' not in json.dumps(state)
  assert setup.params.get('DongleId') == 'comma-id'
  assert len(call(setup, 'listUploadQueue')['result']) == 1


@pytest.mark.parametrize('name', ['/persist/comma/id_ecdsa', '../id_ecdsa', '00000001--abc--2/../../secret',
                                 '00000001--abc--2/arbitrary.txt', '00000001--abc--2/qcamera.ts/extra', '', None])
def test_website_requests_cannot_read_non_recordings(setup, tmp_path, mocker, name):
  mocker.patch.object(konik.Paths, 'log_root', return_value=str(tmp_path))
  result = call(setup, 'uploadFilesToUrls', {'files_data': [{'fn': name, 'url': 'https://anything.test/file'}]})
  assert result['result']['enqueued'] == 0
  assert result['result']['failed']


def test_request_rejects_symlink_and_locked_recording(setup, tmp_path, mocker):
  file = make_recording(tmp_path, 'rlog')
  outside = tmp_path / 'secret'
  outside.write_bytes(b'private')
  (file.parent / 'qcamera.ts').symlink_to(outside)
  mocker.patch.object(konik.Paths, 'log_root', return_value=str(tmp_path))
  result = call(setup, 'uploadFilesToUrls', {'files_data': [{'fn': '00000001--abc--2/qcamera.ts'}]})
  assert result['result']['enqueued'] == 0
  (file.parent / 'rlog.lock').touch()
  result = call(setup, 'uploadFilesToUrls', {'files_data': [{'fn': '00000001--abc--2/rlog.zst'}]})
  assert result['result']['enqueued'] == 0


def test_cancel_automatic_file_and_request_again_resumes_it(setup, tmp_path, mocker):
  path = make_recording(tmp_path)
  mocker.patch.object(konik.Paths, 'log_root', return_value=str(tmp_path))
  item = call(setup, 'listUploadQueue')['result'][0]
  cancelled = call(setup, 'cancelUpload', {'upload_id': item['id']})
  assert cancelled['result']['success'] == 1
  assert call(setup, 'listUploadQueue')['result'] == []
  import os
  import errno
  with pytest.raises(OSError) as error:
    os.getxattr(path, 'user.konik.upload')
  assert error.value.errno == errno.ENODATA
  assert call(setup, 'uploadFilesToUrls', {'files_data': [{'fn': item['path']}]})['result']['enqueued'] == 1
  assert len(call(setup, 'listUploadQueue')['result']) == 1


def test_request_deduplicates_and_resolves_compressed_log(setup, tmp_path, mocker):
  make_recording(tmp_path, 'rlog')
  mocker.patch.object(konik.Paths, 'log_root', return_value=str(tmp_path))
  payload = {'files_data': [{'fn': '00000001--abc--2/rlog.zst'}]}
  assert call(setup, 'uploadFilesToUrls', payload)['result']['enqueued'] == 1
  assert call(setup, 'uploadFilesToUrls', payload)['result']['enqueued'] == 0
  assert len(call(setup, 'listUploadQueue')['result']) == 1


def test_route_view_notifications_are_konik_only_and_keep_ten_routes(setup):
  for i in range(12):
    result = call(setup, 'setRouteViewed', {'route': f'konik-id|00000001--route{i}'})
    assert result['result']['success'] == 1
  state = setup.params.get('KonikUploadControl')
  assert len(state['viewed']) == 10
  assert state['viewed'][-1] == '00000001--route11'
  assert setup.params.get('AthenadRecentlyViewedRoutes') is None


@pytest.mark.parametrize('route', ['comma-id|00000001--abc', '../route', 'konik-id|/persist', 'konik-id|bad/route'])
def test_route_view_notifications_reject_invalid_routes(setup, route):
  assert 'error' in call(setup, 'setRouteViewed', {'route': route})


def test_deleted_cancellations_do_not_exhaust_capacity(setup, tmp_path, mocker):
  from openpilot.system.loggerd import konik_queue
  setup.params.values['KonikUploadControl'] = {'requested': [], 'cancelled': [f'00000001--deleted--{i}/qcamera.ts' for i in range(10000)], 'viewed': []}
  make_recording(tmp_path)
  mocker.patch.object(konik.Paths, 'log_root', return_value=str(tmp_path))
  item = call(setup, 'listUploadQueue')['result'][0]
  result = call(setup, 'cancelUpload', {'upload_id': item['id']})
  assert result['result']['success'] == 1
  assert len(konik_queue.control(setup.params)['cancelled']) == 1


def test_cancel_pruning_preserves_locked_recordings(setup, tmp_path, mocker):
  make_recording(tmp_path)
  mocker.patch.object(konik.Paths, 'log_root', return_value=str(tmp_path))
  item = call(setup, 'listUploadQueue')['result'][0]
  locked = tmp_path / '00000001--locked--0'
  locked.mkdir()
  (locked / 'qcamera.ts').write_bytes(b'video')
  (locked / 'qcamera.ts.lock').touch()
  key = locked.name + '/qcamera.ts'
  setup.params.values['KonikUploadControl'] = {'requested': [], 'cancelled': [key], 'viewed': []}
  assert call(setup, 'cancelUpload', {'upload_id': item['id']})['result']['success'] == 1
  assert key in setup.params.values['KonikUploadControl']['cancelled']


def test_voltage_badge_reads_peripheral_state(setup, mocker):
  socket = mocker.patch.object(konik.messaging, 'sub_sock')
  message = mocker.Mock()
  message.to_dict.return_value = {'peripheralState': {'voltage': 12400}}
  receive = mocker.patch.object(konik.messaging, 'recv_one', return_value=message)
  result = call(setup, 'getMessage', {'service': 'peripheralState', 'timeout': 5000})
  assert result['result'] == {'peripheralState': {'voltage': 12400}}
  socket.assert_called_once_with('peripheralState', timeout=5000)
  receive.assert_called_once_with(socket.return_value)


@pytest.mark.parametrize('params', [
  {'service': 'sendcan'}, {'service': 'carState'}, {'service': None},
  {'service': 'peripheralState', 'timeout': -1}, {'service': 'peripheralState', 'timeout': 5001},
  {'service': 'peripheralState', 'timeout': True}, {'service': 'peripheralState', 'timeout': '5000'},
])
def test_voltage_rpc_rejects_other_services_and_unbounded_timeouts(setup, mocker, params):
  socket = mocker.patch.object(konik.messaging, 'sub_sock')
  assert 'error' in call(setup, 'getMessage', params)
  socket.assert_not_called()


def test_voltage_read_timeout_returns_error(setup, mocker):
  mocker.patch.object(konik.messaging, 'sub_sock')
  mocker.patch.object(konik.messaging, 'recv_one', return_value=None)
  assert 'error' in call(setup, 'getMessage', {'service': 'peripheralState'})


def test_voltage_read_disabled_with_konik(setup, mocker):
  setup.params.values['KonikUploadsEnabled'] = False
  socket = mocker.patch.object(konik.messaging, 'sub_sock')
  assert 'error' in call(setup, 'getMessage', {'service': 'peripheralState'})
  socket.assert_not_called()


def test_connection_retries_after_network_failure_and_stops_cleanly(mocker):
  end = mocker.Mock()
  end.is_set.return_value = False
  connection = mocker.patch.object(konik, 'KonikConnection').return_value
  connection.session.side_effect = [ConnectionError('network unavailable'), None]
  mocker.patch.object(konik, 'Params')
  def retry_wait(delay):
    if end.wait.call_count == 2:
      end.is_set.return_value = True
  end.wait.side_effect = retry_wait
  konik.main(end)
  assert connection.session.call_count == 2
  assert [c.args[0] for c in end.wait.call_args_list] == [2, 4]
  assert all(c.args[0] <= 60 for c in end.wait.call_args_list)


def test_remote_socket_close_clears_presence_without_touching_comma(setup):
  setup.ws.recv_data.return_value = (ABNF.OPCODE_CLOSE, b'')
  setup.connection.session()
  setup.ws.close.assert_called_once()
  assert setup.params.get('KonikConnectionStatus') == 'disconnected'
  assert setup.params.get('KonikLastAthenaPingTime') is None
  assert setup.params.get('LastAthenaPingTime') == 12345
