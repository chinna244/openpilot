import asyncio
import contextlib
import pytest
import json
import time

import capnp
from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal import messaging, log
from teleoprtc.tracks import VIDEO_CLOCK_RATE

from openpilot.system.webrtc.webrtcd import CerealOutgoingMessageProxy, CerealIncomingMessageProxy, ServerState, handle_get_stream
from openpilot.system.webrtc.device.video import LiveStreamVideoStreamTrack


class TestStreamSession(OpenpilotTestCase):
  def setup_method(self):
    self.loop = asyncio.new_event_loop()

  def teardown_method(self):
    self.loop.stop()
    self.loop.close()

  def test_outgoing_proxy(self, mocker):
    test_msg = log.Event.new_message()
    test_msg.logMonoTime = 123
    test_msg.valid = True
    test_msg.customReservedRawData0 = b"test"
    expected_dict = {"type": "customReservedRawData0", "logMonoTime": 123, "valid": True, "data": "test"}
    expected_json = json.dumps(expected_dict).encode()

    channel = mocker.Mock()
    channel.is_open.return_value = True
    proxy = CerealOutgoingMessageProxy(["customReservedRawData0"])
    def mocked_update(t):
      proxy.sm.update_msgs(0, [test_msg])

    mocker.patch.object(messaging.SubMaster, "update", side_effect=mocked_update)
    proxy.add_channel(channel)

    proxy.update()

    channel.send.assert_called_once_with(expected_json)

  def test_incoming_proxy(self, mocker):
    tested_msgs = [
      {"type": "customReservedRawData0", "data": "test"}, # primitive
      {"type": "can", "data": [{"address": 0, "dat": "", "src": 0}]}, # list
      {"type": "testJoystick", "data": {"axes": [0, 0], "buttons": [False]}}, # dict
    ]

    mocked_pubmaster = mocker.MagicMock(spec=messaging.PubMaster)

    proxy = CerealIncomingMessageProxy(mocked_pubmaster)

    for msg in tested_msgs:
      proxy.send(json.dumps(msg).encode())

      mocked_pubmaster.send.assert_called_once()
      mt, md = mocked_pubmaster.send.call_args.args
      msg_type = msg["type"]
      assert isinstance(msg_type, str)
      assert mt == msg_type
      assert isinstance(md, capnp._DynamicStructBuilder)
      assert hasattr(md, msg_type)

      mocked_pubmaster.reset_mock()

  def test_livestream_track(self, mocker):
    fake_msg = messaging.new_message("livestreamCabinEncodeData")

    config = {"receive.return_value": fake_msg.to_bytes()}
    mocker.patch("msgq.SubSocket", spec=True, **config)
    track = LiveStreamVideoStreamTrack("driver")

    assert track.id.startswith("driver")

    for i in range(5):
      packet = self.loop.run_until_complete(track.recv())
      if i == 0:
        start_ns = time.monotonic_ns()
        start_pts = packet.pts
      assert abs(i + packet.pts - (start_pts + (((time.monotonic_ns() - start_ns) * VIDEO_CLOCK_RATE) // 1_000_000_000))) < 450 #5ms
      assert bytes(packet) == b""

  def test_stream_rejects_non_json_content_type(self):
    response = self.loop.run_until_complete(handle_get_stream(ServerState(), b"{}", "text/plain"))

    assert response == (415, b'{"error": "unsupported media type"}', "application/json; charset=utf-8")


class TestCameraSwitchPolicy:
  @pytest.mark.parametrize("camera", ["driver", "road", "wideRoad"])
  def test_konik_session_allows_supported_camera_switch(self, mocker, camera):
    from openpilot.system.webrtc.webrtcd import StreamSession
    session = object.__new__(StreamSession)
    session.konik = True
    track = mocker.Mock()
    session.video_tracks = [track]
    session.message_handler(json.dumps({'type': 'livestreamCameraSwitch', 'data': {'camera': camera}}).encode())
    track.switch_camera.assert_called_once_with(camera)

  def test_existing_sessions_keep_camera_switch(self, mocker):
    from openpilot.system.webrtc.webrtcd import StreamSession
    session = object.__new__(StreamSession)
    session.konik = False
    track = mocker.Mock()
    session.video_tracks = [track]
    session.message_handler(json.dumps({'type': 'livestreamCameraSwitch', 'data': {'camera': 'road'}}).encode())
    track.switch_camera.assert_called_once_with('road')


  @pytest.mark.parametrize("key", ["KonikUploadsEnabled", "IsOffroad"])
  @pytest.mark.parametrize("phase", ["negotiating", "streaming"])
  def test_revoking_konik_stops_only_its_session(self, mocker, key, phase):
    from openpilot.system.webrtc.webrtcd import StreamSession
    async def check():
      flags = {"KonikUploadsEnabled": True, "IsOffroad": True}
      entered = asyncio.Event()
      session = object.__new__(StreamSession)
      session.konik = True
      session.identifier = 'konik-test'
      session.is_body = False
      session.bitrate_controller = None
      session.params = mocker.Mock()
      session.params.get_bool.side_effect = lambda k: flags[k]
      session.stream = mocker.Mock()
      session.stream.has_messaging_channel.return_value = False
      async def block():
        entered.set()
        await asyncio.Event().wait()
      session.stream.wait_for_connection = block if phase == "negotiating" else mocker.AsyncMock()
      session.run_normal_session = block
      session.post_run_cleanup = mocker.AsyncMock()
      session.logger = mocker.Mock()
      unrelated_comma_viewer = asyncio.create_task(asyncio.Event().wait())
      task = asyncio.create_task(session.run())
      try:
        await asyncio.wait_for(entered.wait(), timeout=1)
        flags[key] = False
        with pytest.raises(asyncio.CancelledError):
          await asyncio.wait_for(task, timeout=1)
        session.post_run_cleanup.assert_awaited_once()
        assert not unrelated_comma_viewer.done()
      finally:
        unrelated_comma_viewer.cancel()
        with contextlib.suppress(asyncio.CancelledError):
          await unrelated_comma_viewer
    asyncio.run(check())


  @pytest.mark.parametrize("camera", ["sendcan", "../driver", "unknown", None])
  def test_konik_session_rejects_unknown_camera(self, mocker, camera):
    from openpilot.system.webrtc.webrtcd import StreamSession
    session = object.__new__(StreamSession)
    session.konik = True
    track = mocker.Mock()
    session.video_tracks = [track]
    session.message_handler(json.dumps({'type': 'livestreamCameraSwitch', 'data': {'camera': camera}}).encode())
    track.switch_camera.assert_not_called()


def test_camera_switch_requests_new_keyframe_and_resets_decoder_state(mocker):
  track = object.__new__(LiveStreamVideoStreamTrack)
  track._seen_keyframe = True
  track._make_sock = mocker.Mock(return_value='driver socket')
  track.request_keyframe = mocker.Mock()
  track.switch_camera('driver')
  assert track._sock == 'driver socket'
  assert track._seen_keyframe is False
  track.request_keyframe.assert_called_once()
