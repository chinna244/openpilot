import threading
import time

import pytest

from openpilot.selfdrive.ui.lib import drive_replay as replay


def recording(root, route='00000001--abc', segment=0):
  folder = root / f'{route}--{segment}'
  folder.mkdir(parents=True, exist_ok=True)
  path = folder / 'qcamera.ts'
  path.write_bytes(b'video')
  return path


def test_catalog_groups_and_orders_only_finalized_previews(tmp_path):
  recording(tmp_path, segment=10)
  recording(tmp_path, segment=2)
  locked = recording(tmp_path, route='00000002--def')
  locked.with_suffix('.ts.lock').touch()
  empty = recording(tmp_path, route='00000003--ghi')
  empty.write_bytes(b'')
  (tmp_path / 'symlink--route--0').symlink_to(locked.parent, target_is_directory=True)
  (tmp_path / 'boot').mkdir()
  routes = replay.local_routes(str(tmp_path))
  assert len(routes) == 1
  assert routes[0].name == '00000001--abc'
  assert [clip.number for clip in routes[0].clips] == [2, 10]


def test_catalog_rejects_symlink_preview(tmp_path):
  path = recording(tmp_path)
  outside = tmp_path / 'outside.ts'
  outside.write_bytes(b'private')
  path.unlink()
  path.symlink_to(outside)
  assert replay.local_routes(str(tmp_path)) == []


def test_catalog_missing_root_is_empty(tmp_path):
  assert replay.local_routes(str(tmp_path / 'missing')) == []


def test_decoder_uses_packaged_binary_and_local_only_arguments(tmp_path, mocker):
  path = recording(tmp_path)
  decoder = replay.PreviewDecoder(str(path), lambda: True)
  mocker.patch.object(replay, 'ffmpeg_binary', side_effect=lambda name: '/packaged/' + name)
  command = decoder.command(2.5, 526, 330)
  assert command[0] == '/packaged/ffmpeg'
  assert command[command.index('-i') + 1] == str(path)
  assert command[command.index('-ss') + 1] == '2.5'
  assert command[command.index('-protocol_whitelist') + 1] == 'file,pipe'
  assert '-nostdin' in command and '-an' in command
  decoder.close()


def test_decoder_rejects_onroad_before_spawning(tmp_path, mocker):
  path = recording(tmp_path)
  spawn = mocker.patch.object(replay.subprocess, 'Popen')
  decoder = replay.PreviewDecoder(str(path), lambda: False)
  with pytest.raises(RuntimeError, match='park'):
    list(decoder.frames(0))
  spawn.assert_not_called()


def test_player_refuses_onroad(tmp_path, mocker):
  recording(tmp_path)
  decoder = mocker.patch.object(replay, 'PreviewDecoder')
  player = replay.ReplayPlayer(lambda: False)
  player.play(replay.local_routes(str(tmp_path))[0])
  assert 'park' in player.snapshot()['error'].lower()
  decoder.assert_not_called()


def test_late_worker_result_cannot_replace_new_playback():
  player = replay.ReplayPlayer(lambda: True)
  old = threading.Event()
  player._event = threading.Event()
  player._publish(old, error='stale')
  assert player.snapshot()['error'] == ''


def test_player_closes_decoder_when_policy_is_revoked(tmp_path, mocker):
  recording(tmp_path)
  allowed = [True]
  entered = threading.Event()
  closed = threading.Event()
  class Decoder:
    duration = 30.
    def __init__(self, path, policy):
      self.policy = policy
    def frames(self, offset):
      entered.set()
      while not closed.is_set():
        if not self.policy():
          raise RuntimeError('Playback requires parking')
        time.sleep(.01)
        yield offset, 2, 2, b'\0' * 12
    def close(self):
      closed.set()
  mocker.patch.object(replay, 'PreviewDecoder', Decoder)
  player = replay.ReplayPlayer(lambda: allowed[0])
  player.play(replay.local_routes(str(tmp_path))[0])
  assert entered.wait(1)
  allowed[0] = False
  player._thread.join(1)
  assert not player.snapshot()['playing']
  assert closed.is_set()


def test_replay_layout_closes_on_ignition_and_hiding(mocker):
  from openpilot.selfdrive.ui.layouts import drive_replay as ui
  panel = object.__new__(ui.DriveReplayLayout)
  panel._params = mocker.Mock()
  panel._params.get_bool.return_value = False
  panel._player = mocker.Mock()
  panel._release_texture = mocker.Mock()
  mocker.patch.object(ui.gui_app, 'pop_widget')
  mocker.patch.object(ui, 'ui_state', mocker.Mock(started=True))
  panel._update_state()
  panel._player.close.assert_called_once()
  panel._release_texture.assert_called_once()
  ui.gui_app.pop_widget.assert_called_once()


def test_replay_back_returns_to_browser_before_leaving(mocker):
  from openpilot.selfdrive.ui.layouts import drive_replay as ui
  panel = object.__new__(ui.DriveReplayLayout)
  panel._route = object()
  panel._player = mocker.Mock()
  panel._release_texture = mocker.Mock()
  mocker.patch.object(ui.gui_app, 'pop_widget')
  panel._go_back()
  assert panel._route is None
  panel._player.close.assert_called_once()
  ui.gui_app.pop_widget.assert_not_called()


def test_pause_retains_frame_and_prevents_late_publication():
  player = replay.ReplayPlayer(lambda: True)
  event = player._event
  player._state.update(frame=(2, 2, b'\0' * 12), playing=True, position=3.)
  player.pause()
  player._publish(event, frame=None, position=9.)
  assert player.snapshot()['position'] == 3.
  assert player.snapshot()['frame'] is not None
  assert not player.snapshot()['playing']


@pytest.fixture
def video(tmp_path):
  import subprocess
  path = recording(tmp_path)
  # Two seconds: red for the first second, blue for the second.
  pixels = (b'\xff\0\0' * (64 * 32) * 20) + (b'\0\0\xff' * (64 * 32) * 20)
  subprocess.run([replay.ffmpeg_binary('ffmpeg'), '-v', 'error', '-nostdin', '-threads', '1',
                  '-f', 'rawvideo', '-pixel_format', 'rgb24', '-video_size', '64x32', '-framerate', '20',
                  '-i', 'pipe:0', '-filter_threads', '1', '-frames:v', '40', '-c:v', 'libx264', '-f', 'mpegts', '-y', str(path)],
                 input=pixels, check=True, timeout=10)
  return path


def test_real_preview_decode_and_seek(video):
  decoder = replay.PreviewDecoder(str(video), lambda: True)
  frames = decoder.frames(0)
  position, width, height, pixels = next(frames)
  assert position == 0 and (width, height) == (64, 32)
  assert len(pixels) == width * height * 3
  assert pixels[0] > 200 and pixels[2] < 50
  frames.close()
  decoder = replay.PreviewDecoder(str(video), lambda: True)
  frames = decoder.frames(1.3)
  position, width, height, pixels = next(frames)
  assert position == 1.3
  assert pixels[2] > 200 and pixels[0] < 50
  frames.close()
  assert decoder._process is None


def test_real_decoder_handles_entire_clip_without_truncation(video):
  decoder = replay.PreviewDecoder(str(video), lambda: True)
  frames = list(decoder.frames(0))
  assert 35 <= len(frames) <= 42
  assert frames[-1][0] > 1.7
  assert decoder._process is None


def test_real_decode_stops_when_recording_is_deleted(video):
  decoder = replay.PreviewDecoder(str(video), lambda: True)
  frames = decoder.frames(0)
  next(frames)
  video.unlink()
  with pytest.raises(RuntimeError, match='no longer'):
    next(frames)
  assert decoder._process is None


def test_real_close_interrupts_blocked_decode(video):
  decoder = replay.PreviewDecoder(str(video), lambda: True)
  frames = decoder.frames(0)
  next(frames)
  process = decoder._process
  decoder.close()
  assert process.poll() is not None
  assert decoder._process is None
  frames.close()


def test_player_automatically_advances_retained_clips(tmp_path, mocker):
  recording(tmp_path, segment=2)
  recording(tmp_path, segment=7)
  opened = []
  class Decoder:
    duration = 1.
    def __init__(self, path, policy):
      opened.append(path)
    def frames(self, offset):
      yield 0., 2, 2, b'\0' * 12
    def close(self):
      pass
  mocker.patch.object(replay, 'PreviewDecoder', Decoder)
  player = replay.ReplayPlayer(lambda: True)
  player.play(replay.local_routes(str(tmp_path))[0])
  player._thread.join(1)
  assert [replay.Path(p).parent.name.rsplit('--', 1)[1] for p in opened] == ['2', '7']
  assert player.snapshot()['clip_index'] == 1
  assert not player.snapshot()['playing']


def test_layout_hide_releases_decoder_and_texture(mocker):
  from openpilot.selfdrive.ui.layouts import drive_replay as ui
  panel = object.__new__(ui.DriveReplayLayout)
  panel._children = []
  panel._scan_event = threading.Event()
  panel._player = mocker.Mock()
  panel._release_texture = mocker.Mock()
  panel.hide_event()
  assert panel._scan_event.is_set()
  panel._player.close.assert_called_once()
  panel._release_texture.assert_called_once()


def test_seeking_while_paused_stays_paused(tmp_path, mocker):
  recording(tmp_path)
  player = replay.ReplayPlayer(lambda: True)
  player.route = replay.local_routes(str(tmp_path))[0]
  player._state.update(playing=False, position=3., duration=30.)
  play = mocker.patch.object(player, 'play')
  player.seek(10)
  play.assert_called_once_with(player.route, 0, 13., preview_only=True)


def test_repeated_seeks_remain_paused_while_loading(tmp_path, mocker):
  recording(tmp_path)
  player = replay.ReplayPlayer(lambda: True)
  player.route = replay.local_routes(str(tmp_path))[0]
  player._state.update(playing=False, position=3., duration=30.)
  mocker.patch.object(replay.threading.Thread, 'start')
  player.seek(10)
  assert not player.snapshot()['playing']
  player.seek(10)
  assert not player.snapshot()['playing']
  player.close()


def test_render_after_ignition_does_not_draw_or_recreate_resources(mocker):
  from openpilot.selfdrive.ui.layouts import drive_replay as ui
  panel = object.__new__(ui.DriveReplayLayout)
  panel._parked = lambda: False
  draw = mocker.patch.object(ui.rl, 'draw_rectangle_rec', side_effect=AssertionError('rendered after going onroad'))
  panel._render(ui.rl.Rectangle(0, 0, 536, 240))
  draw.assert_not_called()


def test_consecutive_video_frames_use_pyray_compatible_pixel_pointer(mocker):
  from types import SimpleNamespace
  from openpilot.selfdrive.ui.layouts import drive_replay as ui

  # Exercise pyray's actual void-pointer conversion without a GPU context.
  uploaded = []
  @ui.rl.ffi.callback('void(int, const void *)')
  def upload(texture_id, pixels):
    uploaded.append((texture_id, bytes(ui.rl.ffi.buffer(pixels, 12))))
  wrapped_upload = ui.rl._wrap_function(upload)
  mocker.patch.object(ui.rl, 'update_texture', side_effect=lambda texture, pixels: wrapped_upload(texture.id, pixels))
  for name in ('draw_rectangle_rec', 'draw_texture_pro'):
    mocker.patch.object(ui.rl, name)

  panel = object.__new__(ui.DriveReplayLayout)
  panel._parked = lambda: True
  panel._route = replay.LocalRoute('route', (replay.PreviewClip('qcamera.ts', 0, 0),))
  panel._texture = SimpleNamespace(id=7, width=2, height=2)
  panel._last_frame = None
  panel._back = mocker.Mock()
  panel._controls = [mocker.Mock() for _ in range(5)]
  panel._text = mocker.Mock()
  panel._player = mocker.Mock()
  for pixels in (b'\xff\x00\x00' * 4, b'\x00\x00\xff' * 4):
    panel._player.snapshot.return_value = {'clip_index': 0, 'position': 1., 'duration': 30.,
                                          'frame': (2, 2, pixels), 'error': '', 'loading': False, 'playing': True}
    panel._render(ui.rl.Rectangle(0, 0, 536, 240))
  assert uploaded == [(7, b'\xff\x00\x00' * 4), (7, b'\x00\x00\xff' * 4)]
