"""Parked replay of retained road previews, independent of cloud upload state."""
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import select
import subprocess
import threading
import time
from collections.abc import Callable

FPS = 20


@dataclass(frozen=True)
class PreviewClip:
  path: str
  number: int
  modified: float


@dataclass(frozen=True)
class LocalRoute:
  name: str
  clips: tuple[PreviewClip, ...]


def valid_preview(path: Path) -> bool:
  try:
    return (path.name == 'qcamera.ts' and not path.is_symlink() and not path.parent.is_symlink()
            and path.is_file() and path.stat().st_size > 0 and not any(p.name.endswith('.lock') for p in path.parent.iterdir()))
  except OSError:
    return False


def local_routes(root: str) -> list[LocalRoute]:
  grouped: dict[str, list[PreviewClip]] = {}
  try:
    folders = list(Path(root).iterdir())
  except OSError:
    return []
  for folder in folders:
    match = re.fullmatch(r'([A-Za-z0-9][A-Za-z0-9_-]*--[A-Za-z0-9_-]+)--([0-9]{1,8})', folder.name)
    path = folder / 'qcamera.ts'
    if match and valid_preview(path):
      try:
        grouped.setdefault(match[1], []).append(PreviewClip(str(path), int(match[2]), path.stat().st_mtime))
      except OSError:
        continue
  routes = [LocalRoute(name, tuple(sorted(clips, key=lambda c: c.number))) for name, clips in grouped.items()]
  return sorted(routes, key=lambda r: max(c.modified for c in r.clips), reverse=True)


def ffmpeg_binary(name: str) -> str:
  # Device services may omit the venv's bin directory from PATH.
  import ffmpeg
  binary = Path(ffmpeg.__file__).parent / 'install' / 'bin' / name
  if not binary.is_file() or not os.access(binary, os.X_OK):
    raise RuntimeError('The installed video decoder is unavailable')
  return str(binary)


class PreviewDecoder:
  def __init__(self, path: str, parked: Callable[[], bool]):
    self.path = path
    self.parked = parked
    self.duration = 0.
    self._lock = threading.Lock()
    self._closed = False
    self._process: subprocess.Popen | None = None

  def _check(self):
    if self._closed:
      raise RuntimeError('Playback closed')
    if not self.parked():
      raise RuntimeError('Drive replay requires parking')
    if not valid_preview(Path(self.path)):
      raise RuntimeError('This recording is no longer available')

  def _spawn(self, command):
    with self._lock:
      self._check()
      self._process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
      return self._process

  def command(self, seconds: float, width: int, height: int) -> list[str]:
    return [ffmpeg_binary('ffmpeg'), '-v', 'error', '-nostdin', '-threads', '2',
            '-protocol_whitelist', 'file,pipe', '-i', self.path, '-ss', str(seconds),
            '-an', '-sn', '-dn', '-vf', f'scale={width}:{height}', '-filter_threads', '1',
            '-r', str(FPS), '-fps_mode', 'cfr', '-t', str(max(self.duration - seconds, .05)),
            '-f', 'rawvideo', '-pix_fmt', 'rgb24', 'pipe:1']

  def frames(self, seconds: float):
    try:
      self._check()
      probe = self._spawn([ffmpeg_binary('ffprobe'), '-v', 'error', '-protocol_whitelist', 'file,pipe',
                           '-show_entries', 'stream=codec_name,width,height:format=duration', '-of', 'json', self.path])
      data, _ = probe.communicate(timeout=5)
      self._check()
      if probe.returncode:
        raise RuntimeError('Unable to read this recording')
      info = json.loads(data)
      stream = info['streams'][0]
      self.duration = float(info['format']['duration'])
      width, height = int(stream['width']), int(stream['height'])
      if (stream['codec_name'] != 'h264' or not 0 < width <= 4096 or not 0 < height <= 2160
          or not math.isfinite(self.duration) or not 0 < self.duration <= 180):
        raise RuntimeError('Unsupported road preview')
      scale = min(1., 640 / width, 400 / height)
      width, height = max(2, int(width * scale) // 2 * 2), max(2, int(height * scale) // 2 * 2)
      seconds = max(0., min(float(seconds), max(0., self.duration - 1 / FPS)))
      process = self._spawn(self.command(seconds, width, height))
      size = width * height * 3
      buffer = bytearray()
      last_data = time.monotonic()
      index = 0
      first_frame_at = None
      while True:
        self._check()
        ready, _, _ = select.select([process.stdout], [], [], .1)
        if not ready:
          if time.monotonic() - last_data > (20 if index == 0 else 5):
            raise RuntimeError('Video decoder stopped responding')
          continue
        data = os.read(process.stdout.fileno(), size - len(buffer))
        if not data:
          if buffer or process.wait(timeout=1) != 0 or index == 0:
            raise RuntimeError('This recording could not be decoded completely')
          break
        buffer.extend(data)
        last_data = time.monotonic()
        if len(buffer) == size:
          if first_frame_at is None:
            first_frame_at = time.monotonic()
          # Pace decoded frames here: -re would also delay frames discarded during output seeking.
          while (delay := first_frame_at + index / FPS - time.monotonic()) > 0:
            self._check()
            time.sleep(min(delay, .025))
          yield min(seconds + index / FPS, self.duration), width, height, bytes(buffer)
          buffer.clear()
          index += 1
    finally:
      self.close()

  def close(self):
    with self._lock:
      self._closed = True
      process = self._process
      self._process = None
    if process is not None:
      if process.poll() is None:
        process.terminate()
        try:
          process.wait(timeout=.2)
        except subprocess.TimeoutExpired:
          process.kill()
          process.wait(timeout=1)
      if process.stdout is not None:
        process.stdout.close()


class ReplayPlayer:
  def __init__(self, parked: Callable[[], bool]):
    self.parked = parked
    self.route: LocalRoute | None = None
    self._lock = threading.Lock()
    self._event = threading.Event()
    self._thread: threading.Thread | None = None
    self._decoder: PreviewDecoder | None = None
    self._state = {'frame': None, 'position': 0., 'duration': 0., 'clip_index': 0, 'playing': False, 'error': '', 'loading': False}

  def snapshot(self) -> dict:
    with self._lock:
      return dict(self._state)

  def _publish(self, event, **values):
    with self._lock:
      if event is self._event and not event.is_set():
        self._state.update(values)

  def play(self, route: LocalRoute, clip_index: int = 0, seconds: float = 0., *, preview_only: bool = False):
    self.close()
    self.route = route
    if not self.parked():
      with self._lock:
        self._state.update(error='Drive replay requires parking')
      return
    event = threading.Event()
    with self._lock:
      self._event = event
      self._state.update(playing=not preview_only, loading=True, error='', clip_index=clip_index, position=seconds, frame=None)
    self._thread = threading.Thread(target=self._run, args=(event, clip_index, seconds, preview_only), daemon=True)
    self._thread.start()

  def _run(self, event, clip_index, seconds, preview_only):
    try:
      route = self.route
      for i in range(clip_index, len(route.clips)):
        decoder = PreviewDecoder(route.clips[i].path, lambda: not event.is_set() and self.parked())
        with self._lock:
          if event.is_set():
            return
          self._decoder = decoder
        self._publish(event, clip_index=i, position=seconds, loading=True)
        try:
          for position, width, height, pixels in decoder.frames(seconds):
            self._publish(event, position=position, duration=decoder.duration, frame=(width, height, pixels), loading=False)
            if preview_only:
              self._publish(event, playing=False)
              return
        finally:
          decoder.close()
        seconds = 0.
      self._publish(event, playing=False, loading=False)
    except Exception as e:
      self._publish(event, playing=False, loading=False, error=str(e) if isinstance(e, RuntimeError) else 'Unable to play this recording')

  def pause(self):
    self.close()

  def resume(self):
    state = self.snapshot()
    if self.route is not None:
      self.play(self.route, state['clip_index'], state['position'])

  def seek(self, delta: float):
    state = self.snapshot()
    if self.route is not None:
      self.play(self.route, state['clip_index'], max(0., min(state['position'] + delta, max(0., state['duration'] - .05))),
                preview_only=not state['playing'])

  def close(self):
    with self._lock:
      self._event.set()
      decoder = self._decoder
      self._decoder = None
      self._state.update(playing=False, loading=False)
    if decoder is not None:
      decoder.close()
