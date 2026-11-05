"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_assist_badge import GAUGE_ASPECT, draw_gauge_centered
from openpilot.selfdrive.ui.ui_state import UIStatus, ui_state
from openpilot.system.ui.lib.application import gui_app

# zoompilot's Set Speed Assist gauge mirrors the steering wheel in the bottom-right corner
SPEED_ASSIST_HEIGHT = 44  # the wheel is 50, and round; the gauge's swoosh carries it to the same weight
SPEED_ASSIST_ALPHA = 0.9
# the AEB-off marker beside the gauge: 46x38 of its 60x50 fits between the gauge and the torque arc's end (x 415)
AEB_OFF_BESIDE = (46, 38)
AEB_OFF_GAP = 4  # px


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    # the wheel's filters, so the two corners come and go together
    self._speed_assist_alpha_filter = FirstOrderFilter(0.0, 0.05, 1 / gui_app.target_fps)
    self._speed_assist_y_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    self._draw_speed_assist(rect)

  def _aeb_off_rect(self, rect: rl.Rectangle) -> rl.Rectangle:
    # beside the gauge, shrunk to fit between it and the torque arc's end and centred on it; slides with the
    # gauge's fade (last frame's: it draws after the marker)
    corner = super()._aeb_off_rect(rect)
    s = self._speed_assist_alpha_filter.x / SPEED_ASSIST_ALPHA
    if s < 1e-2:
      return corner
    w, h = AEB_OFF_BESIDE
    cx, cy = rect.x + rect.width - 21 - self._txt_wheel.width / 2, rect.y + rect.height - 14 - self._txt_wheel.height / 2
    beside = (cx - GAUGE_ASPECT * SPEED_ASSIST_HEIGHT / 2 - AEB_OFF_GAP - w, cy - h / 2, w, h)
    return rl.Rectangle(*(c + s * (b - c) for c, b in zip((corner.x, corner.y, corner.width, corner.height), beside, strict=True)))

  def _draw_speed_assist(self, rect: rl.Rectangle) -> None:
    # comes and goes with the wheel
    wheel = self._txt_wheel
    shown = (ui_state.speed_assist_active and ui_state.status != UIStatus.DISENGAGED and
             not self.blind_spot_indicators.right_detected)
    alpha = self._speed_assist_alpha_filter.update(SPEED_ASSIST_ALPHA if shown else 0.0)
    y_offset = self._speed_assist_y_filter.update(0.0 if shown else wheel.height / 2)
    if alpha < 1e-2:
      return
    draw_gauge_centered(rect.x + rect.width - 21 - wheel.width / 2, rect.y + rect.height - 14 - wheel.height / 2 + y_offset,
                        SPEED_ASSIST_HEIGHT, alpha)

  def left_blind_spot_detected(self) -> bool:
    return self.blind_spot_indicators.left_detected

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
