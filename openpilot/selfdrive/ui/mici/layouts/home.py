import datetime
import time

from openpilot.cereal import log
import pyray as rl
from collections.abc import Callable
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.layouts import HBoxLayout
from openpilot.system.ui.widgets.icon_widget import IconWidget
from openpilot.system.ui.widgets.label import UnifiedLabel, gui_label
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
from openpilot.common.hardware import HARDWARE
from openpilot.common.time_helpers import system_time_valid
from openpilot.common.version import RELEASE_BRANCHES
from openpilot.system.ui.lib.text_measure import measure_text_cached

HEAD_BUTTON_FONT_SIZE = 40
HOME_PADDING = 8
ALERTS_ZONE_WIDTH = 180
STATUS_BAR_SPACING = 18
CLOCK_FONT_SIZE = 36
CLOCK_POLL_S = 1.0
NITZ_POLL_S = 5.0
# 3GPP/Quectel NITZ offset is in quarters of an hour.
TZ_OFFSET_QUARTERS_MIN = -48
TZ_OFFSET_QUARTERS_MAX = 56
CLOCK_SAMPLE_TEXT = "00/00 00:00"
CLOCK_FALLBACK_WIDTH = 180

NetworkType = log.DeviceState.NetworkType

NETWORK_TYPES = {
  NetworkType.none: "Offline",
  NetworkType.wifi: "WiFi",
  NetworkType.cell2G: "2G",
  NetworkType.cell3G: "3G",
  NetworkType.cell4G: "LTE",
  NetworkType.cell5G: "5G",
  NetworkType.ethernet: "Ethernet",
}


class AlertsPill(Widget):
  ICON_OFFSET = 12
  COUNT_OFFSET = 40

  def __init__(self):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, 104, 52))

    self._pill_bg_txt = gui_app.texture("icons_mici/alerts_pill.png", 104, 52)
    self._icon_red = gui_app.texture("icons_mici/offroad_alerts/red_warning.png", 36, 36)
    self._icon_orange = gui_app.texture("icons_mici/offroad_alerts/orange_warning.png", 36, 36)
    self._icon_green = gui_app.texture("icons_mici/offroad_alerts/green_wheel.png", 36, 36)
    self._alert_count_callback: Callable[[], int] | None = None
    self._max_severity_callback: Callable[[], int | None] | None = None

  def set_alert_count_callback(self, callback: Callable[[], int] | None,
                               severity_callback: Callable[[], int | None] | None = None):
    self._alert_count_callback = callback
    self._max_severity_callback = severity_callback

  def _render(self, _):
    alert_count = self._alert_count_callback() if self._alert_count_callback else 0
    if alert_count > 0:
      pill_w, pill_h = self._pill_bg_txt.width, self._pill_bg_txt.height
      rl.draw_texture_ex(self._pill_bg_txt, rl.Vector2(self.rect.x, self.rect.y), 0.0, 1.0, rl.WHITE)

      severity = self._max_severity_callback() if self._max_severity_callback else None
      if severity == -1:
        warning_txt = self._icon_green
      elif severity is not None and severity > 0:
        warning_txt = self._icon_red
      else:
        warning_txt = self._icon_orange

      warn_x = self.rect.x + self.ICON_OFFSET
      warn_y = self.rect.y + (pill_h - warning_txt.height) / 2
      rl.draw_texture_ex(warning_txt, rl.Vector2(warn_x, warn_y), 0.0, 1.0, rl.WHITE)

      count_rect = rl.Rectangle(self.rect.x + self.COUNT_OFFSET, self.rect.y, pill_w - self.COUNT_OFFSET, pill_h)
      gui_label(count_rect, str(alert_count), font_size=36,
                alignment=rl.GuiTextAlignment.TEXT_ALIGN_CENTER,
                alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_MIDDLE)


class NetworkIcon(Widget):
  def __init__(self):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, 54, 44))  # max size of all icons
    self._net_type = NetworkType.none
    self._net_strength = 0

    self._wifi_slash_txt = gui_app.texture("icons_mici/settings/network/wifi_strength_slash.png", 50, 44)
    self._wifi_none_txt = gui_app.texture("icons_mici/settings/network/wifi_strength_none.png", 50, 37)
    self._wifi_low_txt = gui_app.texture("icons_mici/settings/network/wifi_strength_low.png", 50, 37)
    self._wifi_medium_txt = gui_app.texture("icons_mici/settings/network/wifi_strength_medium.png", 50, 37)
    self._wifi_full_txt = gui_app.texture("icons_mici/settings/network/wifi_strength_full.png", 50, 37)

    self._cell_none_txt = gui_app.texture("icons_mici/settings/network/cell_strength_none.png", 54, 36)
    self._cell_low_txt = gui_app.texture("icons_mici/settings/network/cell_strength_low.png", 54, 36)
    self._cell_medium_txt = gui_app.texture("icons_mici/settings/network/cell_strength_medium.png", 54, 36)
    self._cell_high_txt = gui_app.texture("icons_mici/settings/network/cell_strength_high.png", 54, 36)
    self._cell_full_txt = gui_app.texture("icons_mici/settings/network/cell_strength_full.png", 54, 36)

  def _update_state(self):
    device_state = ui_state.sm['deviceState']
    self._net_type = device_state.networkType
    strength = device_state.networkStrength
    self._net_strength = max(0, min(5, strength.raw + 1)) if strength.raw > 0 else 0

  def _render(self, _):
    if self._net_type == NetworkType.wifi:
      # There is no 1
      draw_net_txt = {0: self._wifi_none_txt,
                      2: self._wifi_low_txt,
                      3: self._wifi_medium_txt,
                      4: self._wifi_full_txt,
                      5: self._wifi_full_txt}.get(self._net_strength, self._wifi_low_txt)
    elif self._net_type in (NetworkType.cell2G, NetworkType.cell3G, NetworkType.cell4G, NetworkType.cell5G):
      draw_net_txt = {0: self._cell_none_txt,
                      2: self._cell_low_txt,
                      3: self._cell_medium_txt,
                      4: self._cell_high_txt,
                      5: self._cell_full_txt}.get(self._net_strength, self._cell_none_txt)
    else:
      draw_net_txt = self._wifi_slash_txt

    draw_x = self._rect.x + (self._rect.width - draw_net_txt.width) / 2
    draw_y = self._rect.y + (self._rect.height - draw_net_txt.height) / 2

    if draw_net_txt == self._wifi_slash_txt:
      # Offset by difference in height between slashless and slash icons to make center align match
      draw_y -= (self._wifi_slash_txt.height - self._wifi_none_txt.height) / 2

    rl.draw_texture_ex(draw_net_txt, rl.Vector2(draw_x, draw_y), 0.0, 1.0, rl.Color(255, 255, 255, int(255 * 0.9)))


def _coerce_offset_quarters(value) -> int | None:
  if isinstance(value, bool) or value is None:
    return None
  try:
    if isinstance(value, float):
      if not value.is_integer():
        return None
      offset = int(value)
    else:
      offset = int(value)
  except (TypeError, ValueError):
    return None
  if TZ_OFFSET_QUARTERS_MIN <= offset <= TZ_OFFSET_QUARTERS_MAX:
    return offset
  return None


class LocalClock(Widget):
  def __init__(self):
    super().__init__()
    self._params = ui_state.params
    self._offset_quarters = self._read_persisted_offset()
    self._persisted_offset = self._offset_quarters
    self._text = ""
    self._last_clock_mono = 0.0
    self._last_nitz_mono = 0.0
    self.set_enabled(False)
    self.set_rect(rl.Rectangle(0, 0, self._measure_width(), 44))
    self.set_visible(False)

  def _measure_width(self) -> float:
    try:
      return float(measure_text_cached(gui_app.font(FontWeight.MEDIUM), CLOCK_SAMPLE_TEXT, CLOCK_FONT_SIZE).x)
    except Exception:
      return float(CLOCK_FALLBACK_WIDTH)

  def _read_persisted_offset(self) -> int | None:
    try:
      return _coerce_offset_quarters(self._params.get("LastNetworkTimeZoneOffsetQuarters"))
    except Exception:
      return None

  def _poll_nitz(self):
    live = None
    try:
      get_modem_state = getattr(HARDWARE, "get_modem_state", None)
      if callable(get_modem_state):
        state = get_modem_state()
        if isinstance(state, dict):
          live = _coerce_offset_quarters(state.get("network_timezone_offset_quarters"))
    except Exception:
      live = None

    if live is None:
      return

    self._offset_quarters = live
    if live != self._persisted_offset:
      try:
        self._params.put("LastNetworkTimeZoneOffsetQuarters", live)
        self._persisted_offset = live
      except Exception:
        pass

  @property
  def has_time(self) -> bool:
    return bool(self._text)

  def _refresh_text(self):
    if self._offset_quarters is None or not system_time_valid():
      self._text = ""
      self.set_visible(False)
      return
    try:
      utc_now = datetime.datetime.now(datetime.timezone.utc)
      local_now = utc_now + datetime.timedelta(minutes=self._offset_quarters * 15)
      self._text = local_now.strftime("%m/%d %H:%M")
    except Exception:
      self._text = ""

  def _update_state(self):
    now = time.monotonic()
    if self._last_nitz_mono == 0.0 or (now - self._last_nitz_mono) >= NITZ_POLL_S:
      self._last_nitz_mono = now
      self._poll_nitz()
    if self._last_clock_mono == 0.0 or (now - self._last_clock_mono) >= CLOCK_POLL_S:
      self._last_clock_mono = now
      self._refresh_text()

  def _render(self, _):
    if not self._text:
      return
    gui_label(self._rect, self._text, font_size=CLOCK_FONT_SIZE, color=rl.WHITE,
              font_weight=FontWeight.MEDIUM,
              alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
              alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_MIDDLE,
              elide_right=False)


class MiciHomeLayout(Widget):
  def __init__(self):
    super().__init__()
    self._on_settings_click: Callable | None = None
    self._on_alerts_click: Callable | None = None
    self._alert_count_callback: Callable[[], int] | None = None

    self._mouse_down_t: None | float = None
    self._did_long_press = False
    self._is_pressed_prev = False

    self._version_text = self._get_version_text()

    self._experimental_icon = IconWidget("icons_mici/experimental_mode.png", (48, 48))
    self._chestnut_icon = IconWidget("icons_mici/chestnut_green.png", (68, 40))
    self._chestnut_failed_icon = IconWidget("icons_mici/chestnut_orange.png", (68, 40))
    self._mic_icon = IconWidget("icons_mici/microphone.png", (32, 46))
    self._body_icon = IconWidget("icons_mici/body.png", (54, 37))

    self._alerts_pill = AlertsPill()
    self._local_clock = LocalClock()

    self._status_bar_layout = HBoxLayout([
      IconWidget("icons_mici/settings.png", (48, 48), opacity=0.9),
      NetworkIcon(),
      self._local_clock,
      self._experimental_icon,
      self._chestnut_icon,
      self._chestnut_failed_icon,
      self._body_icon,
      self._mic_icon,
    ], spacing=STATUS_BAR_SPACING)

    self._openpilot_label = UnifiedLabel("zoompilot", font_size=96, font_weight=FontWeight.DISPLAY, max_width=480, wrap_text=False)
    self._version_label = UnifiedLabel("", font_size=36, font_weight=FontWeight.ROMAN, max_width=480, wrap_text=False)
    self._large_version_label = UnifiedLabel("", font_size=64, text_color=rl.GRAY, font_weight=FontWeight.ROMAN, max_width=480, wrap_text=False)
    self._date_label = UnifiedLabel("", font_size=36, text_color=rl.GRAY, font_weight=FontWeight.ROMAN, max_width=480, wrap_text=False)
    self._branch_label = UnifiedLabel("", font_size=36, text_color=rl.GRAY, font_weight=FontWeight.ROMAN, scroll=True)
    self._version_commit_label = UnifiedLabel("", font_size=36, text_color=rl.GRAY, font_weight=FontWeight.ROMAN, max_width=480, wrap_text=False)

  def _update_state(self):
    if self.is_pressed and not self._is_pressed_prev:
      self._mouse_down_t = time.monotonic()
    elif not self.is_pressed and self._is_pressed_prev:
      self._mouse_down_t = None
      self._did_long_press = False
    self._is_pressed_prev = self.is_pressed

    if self._mouse_down_t is not None:
      if time.monotonic() - self._mouse_down_t > 0.5:
        # long gating for experimental mode - only allow toggle if longitudinal control is available
        if ui_state.has_longitudinal_control and ui_state.experimental_mode_confirmed:
          ui_state.experimental_mode = not ui_state.experimental_mode
          ui_state.params.put("ExperimentalMode", ui_state.experimental_mode, block=True)
        self._mouse_down_t = None
        self._did_long_press = True

  def set_callbacks(self, on_settings: Callable | None = None, on_alerts: Callable | None = None,
                    alert_count_callback: Callable[[], int] | None = None,
                    max_severity_callback: Callable[[], int | None] | None = None):
    self._on_settings_click = on_settings
    self._on_alerts_click = on_alerts
    self._alert_count_callback = alert_count_callback
    self._alerts_pill.set_alert_count_callback(alert_count_callback, max_severity_callback)

  def _handle_mouse_release(self, mouse_pos: MousePos):
    if not self._did_long_press:
      relative_x = mouse_pos.x - self.rect.x
      has_alerts = self._alert_count_callback and self._alert_count_callback() > 0
      if has_alerts and relative_x > self.rect.width - ALERTS_ZONE_WIDTH:
        if self._on_alerts_click:
          self._on_alerts_click()
      elif self._on_settings_click:
        self._on_settings_click()
    self._did_long_press = False

  def _get_version_text(self) -> tuple[str, str, str, str] | None:
    version = ui_state.params.get("Version")
    branch = ui_state.params.get("GitBranch")
    commit = ui_state.params.get("GitCommit")

    if not all((version, branch, commit)):
      return None

    commit_date_raw = ui_state.params.get("GitCommitDate")
    try:
      # GitCommitDate format from get_commit_date(): '%ct %ci' e.g. "'1708012345 2024-02-15 ...'"
      unix_ts = int(commit_date_raw.strip("'").split()[0])
      date_str = datetime.datetime.fromtimestamp(unix_ts).strftime("%b %d")
    except (ValueError, IndexError, TypeError, AttributeError):
      date_str = ""

    return version, branch, commit[:7], date_str

  def _alerts_present(self) -> bool:
    count = self._alert_count_callback() if self._alert_count_callback else 0
    return bool(count)

  def _footer_available_width(self) -> float:
    if self._alerts_present():
      return max(0.0, self.rect.width - self._alerts_pill.rect.width - 2 * HOME_PADDING)
    return max(0.0, self.rect.width - HOME_PADDING)

  def _status_bar_width_with_clock(self) -> float:
    widths = [w.rect.width for w in self._status_bar_layout.widgets
              if w is self._local_clock or w.is_visible]
    if not widths:
      return 0.0
    return sum(widths) + STATUS_BAR_SPACING * (len(widths) - 1)

  def _update_local_clock_visibility(self):
    # HBoxLayout skips hidden children, so LocalClock never gets render()/_update_state()
    # while hidden. Keep polling NITZ/time here, then show only if the current visible
    # footer widgets plus the clock fit in the remaining footer/alerts-pill space.
    self._local_clock._update_state()
    self._local_clock.set_visible(
      self._local_clock.has_time and self._status_bar_width_with_clock() <= self._footer_available_width())

  def _render(self, _):
    # TODO: why is there extra space here to get it to be flush?
    text_pos = rl.Vector2(self.rect.x - 2 + HOME_PADDING, self.rect.y - 16)
    self._openpilot_label.set_position(text_pos.x, text_pos.y)
    self._openpilot_label.render()

    if self._version_text is not None:
      # release branch
      release_branch = self._version_text[1] in RELEASE_BRANCHES
      version_pos = rl.Rectangle(text_pos.x, text_pos.y + self._openpilot_label.font_size + 16, 100, 44)
      self._version_label.set_text(self._version_text[0])
      self._version_label.set_position(version_pos.x, version_pos.y)
      self._version_label.render()

      self._date_label.set_text(" " + self._version_text[3])
      self._date_label.set_position(version_pos.x + self._version_label.text_width + 10, version_pos.y)
      self._date_label.render()

      self._branch_label.set_max_width(gui_app.width - self._version_label.text_width - self._date_label.text_width - 32)
      self._branch_label.set_text(" " + ("release" if release_branch else self._version_text[1]))
      self._branch_label.set_position(version_pos.x + self._version_label.text_width + self._date_label.text_width + 20, version_pos.y)
      self._branch_label.render()

      if not release_branch:
        # 2nd line
        self._version_commit_label.set_text(self._version_text[2])
        self._version_commit_label.set_position(version_pos.x, version_pos.y + self._date_label.font_size + 7)
        self._version_commit_label.render()

    # ***** Center-aligned bottom section icons *****
    self._experimental_icon.set_visible(ui_state.experimental_mode)
    if gui_app.sunnypilot_ui():
      self._set_chestnut_visibility()
    else:
      self._chestnut_icon.set_visible(ui_state.chestnut_state in (ChestnutState.READY, ChestnutState.LOADING, ChestnutState.ACTIVE))
      self._chestnut_failed_icon.set_visible(ui_state.chestnut_state in (ChestnutState.UNCOMPILED, ChestnutState.FAILED))
    self._mic_icon.set_visible(ui_state.recording_audio)
    self._body_icon.set_visible(bool(ui_state.is_body))
    self._update_local_clock_visibility()

    footer_rect = rl.Rectangle(self.rect.x + HOME_PADDING, self.rect.y + self.rect.height - 48, self.rect.width - HOME_PADDING, 48)
    self._status_bar_layout.render(footer_rect)

    # TODO: add alignment to hboxlayout and add to there
    self._alerts_pill.set_position(self.rect.x + self.rect.width - self._alerts_pill.rect.width - HOME_PADDING,
                                   self.rect.y + self.rect.height - self._alerts_pill.rect.height)
    self._alerts_pill.render()
