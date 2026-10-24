"""Local road-preview browser and parked-only player for both display sizes."""
from datetime import datetime
import threading

import pyray as rl

from openpilot.common.hardware.hw import Paths
from openpilot.common.params import Params
from openpilot.selfdrive.ui.lib.drive_replay import LocalRoute, ReplayPlayer, local_routes
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.button import Button


class DriveReplayLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._player = ReplayPlayer(self._parked)
    self._routes: list[LocalRoute] = []
    self._route: LocalRoute | None = None
    self._page = 0
    self._scanning = False
    self._scan_result = None
    self._scan_event = threading.Event()
    self._texture = None
    self._last_frame = None
    self._scale = 4 if gui_app.big_ui() else 1
    self._font = gui_app.font(FontWeight.NORMAL)
    self._back = self._child(self._button('Back', self._go_back))
    self._refresh = self._child(self._button('Refresh', self._scan))
    self._previous = self._child(self._button('Earlier', lambda: self._change_page(1)))
    self._next = self._child(self._button('Newer', lambda: self._change_page(-1)))
    self._rows = [self._child(self._button('', lambda i=i: self._open_row(i))) for i in range(3)]
    self._controls = [self._child(self._button(label, callback)) for label, callback in [
      ('Prev', lambda: self._change_clip(-1)), ('-10s', lambda: self._player.seek(-10)),
      ('Pause', self._toggle), ('+10s', lambda: self._player.seek(10)), ('Next', lambda: self._change_clip(1)),
    ]]

  def _button(self, text, callback):
    return Button(text, callback, font_size=18 * self._scale, border_radius=7 * self._scale, text_padding=4 * self._scale)

  def _parked(self):
    return not ui_state.started and self._params.get_bool('IsOffroad')

  def show_event(self):
    super().show_event()
    self._route = None
    self._page = 0
    self._scan()

  def _scan(self):
    self._scan_event.set()
    event = threading.Event()
    self._scan_event = event
    self._scanning = True
    self._scan_result = None
    root = Paths.log_root()
    def scan():
      result = local_routes(root)
      if not event.is_set():
        self._scan_result = result
    threading.Thread(target=scan, daemon=True).start()

  def _change_page(self, delta):
    self._page = max(0, min(self._page + delta, max(0, (len(self._routes) - 1) // 3)))

  def _open_row(self, index):
    index += self._page * 3
    if self._parked() and index < len(self._routes):
      self._route = self._routes[index]
      self._player.play(self._route)

  def _change_clip(self, delta):
    state = self._player.snapshot()
    if self._route is not None:
      index = state['clip_index'] + delta
      if 0 <= index < len(self._route.clips):
        self._player.play(self._route, index)

  def _toggle(self):
    state = self._player.snapshot()
    if state['playing']:
      self._player.pause()
    elif self._route is not None:
      if state['position'] >= state['duration'] - .1 and state['clip_index'] == len(self._route.clips) - 1:
        self._player.play(self._route)
      else:
        self._player.resume()

  def _release_texture(self):
    if self._texture is not None:
      rl.unload_texture(self._texture)
    self._texture = None
    self._last_frame = None

  def _go_back(self):
    self._player.close()
    self._release_texture()
    if self._route is not None:
      self._route = None
    else:
      gui_app.pop_widget()

  def hide_event(self):
    self._scan_event.set()
    self._player.close()
    self._release_texture()
    super().hide_event()

  def _update_state(self):
    super()._update_state()
    if not self._parked():
      self._player.close()
      self._release_texture()
      gui_app.pop_widget()
      return
    if self._scan_result is not None:
      self._routes = self._scan_result
      self._scan_result = None
      self._scanning = False
      self._change_page(0)

  def _text(self, text, x, y, size=18, color=rl.WHITE):
    rl.draw_text_ex(self._font, text, rl.Vector2(x, y), size * self._scale, 0, color)

  def _render(self, rect):
    if not self._parked():
      return
    scale = rect.height / 240
    pad, bar = 8 * scale, 32 * scale
    rl.draw_rectangle_rec(rect, rl.BLACK)
    self._back.render(rl.Rectangle(rect.x + pad, rect.y + pad, 66 * scale, bar))
    if self._route is None:
      self._text('Drive replay', rect.x + 84 * scale, rect.y + 14 * scale)
      self._refresh.render(rl.Rectangle(rect.x + rect.width - 100 * scale, rect.y + pad, 92 * scale, bar))
      if self._scanning or not self._routes:
        self._text('Finding local drives...' if self._scanning else 'No completed road previews on this device.',
                   rect.x + pad, rect.y + 75 * scale, 17, rl.LIGHTGRAY)
      for i, row in enumerate(self._rows):
        index = self._page * 3 + i
        if index < len(self._routes):
          route = self._routes[index]
          date = datetime.fromtimestamp(route.clips[0].modified).strftime('%b %d %H:%M')
          row.set_text(f'{date}   |   {len(route.clips)} clips   |   {route.name.split("--")[0]}')
          row.render(rl.Rectangle(rect.x + pad, rect.y + (46 + 46 * i) * scale, rect.width - 2 * pad, 40 * scale))
      self._previous.set_enabled((self._page + 1) * 3 < len(self._routes))
      self._next.set_enabled(self._page > 0)
      self._previous.render(rl.Rectangle(rect.x + pad, rect.y + 195 * scale, 90 * scale, bar))
      self._next.render(rl.Rectangle(rect.x + rect.width - 98 * scale, rect.y + 195 * scale, 90 * scale, bar))
      self._text('Local road previews | parked only', rect.x + 110 * scale, rect.y + 205 * scale, 15, rl.LIGHTGRAY)
      return

    state = self._player.snapshot()
    clip = self._route.clips[state['clip_index']]
    self._text(f'Clip {clip.number}  |  {state["position"]:.0f} / {state["duration"]:.0f}s',
               rect.x + 84 * scale, rect.y + 14 * scale, 17)
    frame = state['frame']
    if frame is not None and frame is not self._last_frame:
      width, height, pixels = frame
      if self._texture is None or self._texture.width != width or self._texture.height != height:
        self._release_texture()
        image = rl.Image(rl.ffi.from_buffer(pixels), width, height, 1, rl.PixelFormat.PIXELFORMAT_UNCOMPRESSED_R8G8B8)
        self._texture = rl.load_texture_from_image(image)
        rl.set_texture_filter(self._texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
      else:
        rl.update_texture(self._texture, rl.ffi.cast('void *', rl.ffi.from_buffer(pixels)))
      self._last_frame = frame
    if self._texture is not None:
      area = rl.Rectangle(rect.x + pad, rect.y + 44 * scale, rect.width - 2 * pad, 145 * scale)
      factor = min(area.width / self._texture.width, area.height / self._texture.height)
      width, height = self._texture.width * factor, self._texture.height * factor
      rl.draw_texture_pro(self._texture, rl.Rectangle(0, 0, self._texture.width, self._texture.height),
                          rl.Rectangle(area.x + (area.width - width) / 2, area.y + (area.height - height) / 2, width, height),
                          rl.Vector2(0, 0), 0, rl.WHITE)
    if state['error'] or state['loading']:
      rl.draw_rectangle(int(rect.x), int(rect.y + 86 * scale), int(rect.width), int(52 * scale), rl.Color(0, 0, 0, 220))
      self._text(state['error'] or 'Loading preview...', rect.x + pad, rect.y + 100 * scale, 16,
                 rl.RED if state['error'] else rl.WHITE)
    self._controls[0].set_enabled(state['clip_index'] > 0)
    self._controls[2].set_text('Pause' if state['playing'] else 'Play')
    self._controls[4].set_enabled(state['clip_index'] + 1 < len(self._route.clips))
    width = (rect.width - 6 * pad) / 5
    for i, button in enumerate(self._controls):
      button.render(rl.Rectangle(rect.x + pad + i * (width + pad), rect.y + 198 * scale, width, bar))
