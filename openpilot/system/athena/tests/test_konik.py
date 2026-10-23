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


@pytest.mark.parametrize('method', ['startLocalProxy', 'uploadFileToUrl', 'getMessage', 'getPublicKey', 'getSshAuthorizedKeys',
                                    'getParams', 'saveParams', 'listDataDirectory', 'reboot', 'ssh', 'setRouteViewed'])
def test_all_unapproved_remote_commands_are_rejected(setup, method):
  result = call(setup, method)
  assert result['error']['code'] == -32601


def test_rpc_dispatcher_is_not_comma_dispatcher(setup):
  methods = setup.connection.methods()
  assert methods is not konik.rpc.dispatcher
  assert set(methods) == {'echo', 'getVersion', 'getNetworkType', 'getNetworkMetered', 'startStream'}
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
