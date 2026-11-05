import math
import pyray as rl
from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

PlanSource = log.LongitudinalPlan.LongitudinalPlanSource
PlanSourceSP = custom.LongitudinalPlanSP.LongitudinalPlanSource

ACCEL_ON = -0.5  # m/s^2, openpilot braking harder than this ...
ON_TIME = 1.0  # s ... for this long shows the banner
OFF_TIME = 2.5  # s without it hides it
HOLD_TIME = 2.0  # s a shown reason stays before another one replaces it
V_MIN = 2.0  # m/s, not while stopped or creeping
FONT_SIZE = 32
LEFT_MARGIN = 84  # px, clear of the driver monitoring icon
LEAD_TTC = 8.  # s, a slower radar lead closer than this in time is the reason
MODEL_SLOW_V = 4.  # m/s, model speed 4 s out below this: a stop or a turn ahead
TURN_V_MAX = 15.  # m/s, a blinker below this speed is a turn
SMALL_ALERT_HEIGHT = 0.583  # of the screen height, as alert_renderer draws a small alert

SP_REASONS = {PlanSourceSP.sccVision: 'curve', PlanSourceSP.sccMap: 'map', PlanSourceSP.speedLimitAssist: 'limit'}


def slow_reason(plan_source: int, sp_source: int, lead_present: bool, d_rel: float, v_lead: float, v_ego: float,
                blinker: bool = False, model_v4: float = 1e3, model_v_end: float = 1e3) -> str:
  """Which planner candidate is braking: the MPC on a lead, the model, or a cruise target (SCC-V, SCC-M, SLA, set speed)."""
  if plan_source in (PlanSource.lead0, PlanSource.lead1, PlanSource.lead2):
    return 'lead'
  if plan_source == PlanSource.e2e:
    # the model slows for a close, slower radar lead too, or one it closes on within 8 s
    if lead_present and v_lead < v_ego + 1. and (d_rel < max(40., 3. * v_ego) or d_rel < LEAD_TTC * (v_ego - v_lead)):
      return 'model_lead'
    # a blinker at town speed; above it more likely a lane change
    if blinker and v_ego < TURN_V_MAX:
      return 'turn'
    # a plan down to a crawl within 4 s, or to a stop: stop sign, light, corner
    if model_v_end < 1. or model_v4 < MODEL_SLOW_V:
      return 'stop'
    return 'model'
  return SP_REASONS.get(sp_source, 'cruise')


class SlowReasonFilter:
  """Debounces the braking reason for display: None while the banner is hidden."""
  def __init__(self):
    self.reason: str | None = None
    self._on_since: float | None = None
    self._last_braking = 0.
    self._shown_at = 0.

  def update(self, now: float, active: bool, braking: bool, reason: str) -> str | None:
    braking = active and braking
    if not braking:
      self._on_since = None
    elif self._on_since is None:
      self._on_since = now

    on_since = self._on_since
    if self.reason is None:
      if on_since is not None and now - on_since >= ON_TIME:
        self.reason, self._shown_at, self._last_braking = reason, now, now
    elif not active or now - self._last_braking >= OFF_TIME:
      self.reason = None
    elif braking:
      self._last_braking = now
      if reason != self.reason and now - self._shown_at >= HOLD_TIME:
        self.reason, self._shown_at = reason, now
    return self.reason


class SlowBanner(Widget):
  """Display only: says why openpilot is slowing the car."""
  def __init__(self):
    super().__init__()
    self._filter = SlowReasonFilter()
    self._alpha_filter = FirstOrderFilter(0.0, 0.05, 1 / gui_app.target_fps)
    self._font = gui_app.font(FontWeight.BOLD)
    self._should_draw = False
    self._text = ''
    self._top_shown = 0.
    self._alert_bottom = 0.

  def set_should_draw(self, should_draw: bool):
    self._should_draw = should_draw

  def set_alert_bottom(self, alert_bottom: float):
    self._alert_bottom = alert_bottom

  @staticmethod
  def _box_x(rect: rl.Rectangle, text_w: float) -> float:
    """Centered on the screen, pushed right only as far as the driver monitoring icon needs."""
    return max(rect.x + LEFT_MARGIN, rect.x + (rect.width - text_w) / 2 - 14)

  @staticmethod
  def _top(alert_size: int, rect: rl.Rectangle, alert_bottom: float = 0.) -> float | None:
    """Banner top edge: just under a small alert's text (or its 140 px band before it is drawn); hidden under bigger ones."""
    if alert_size == log.SelfdriveState.AlertSize.none:
      return rect.y + 12
    if alert_size == log.SelfdriveState.AlertSize.small:
      if alert_bottom > rect.y:
        return min(alert_bottom + 6, rect.y + round(rect.height * SMALL_ALERT_HEIGHT) + 4)
      return rect.y + round(rect.height * SMALL_ALERT_HEIGHT) + 4
    return None

  @staticmethod
  def _text_for(reason: str, lead_present: bool, d_rel: float, speed_limit: float, metric: bool) -> str:
    """The system that brakes first (Lead = radar MPC, Model = e2e, SCC-V, SCC-M, SLA), then what for."""
    if reason in ('lead', 'model_lead'):
      system = 'Lead' if reason == 'lead' else 'Model: car'
      if not lead_present or not math.isfinite(d_rel) or d_rel <= 0:
        return system
      return f'{system} {round(d_rel)} m' if metric else f'{system} {round(d_rel * CV.METER_TO_FOOT / 10) * 10} ft'
    if reason == 'limit':
      limit = speed_limit * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH)
      return f'SLA {round(limit)}' if limit > 1 else 'SLA'
    return {'model': 'Model', 'stop': 'Model: stop/turn', 'turn': 'Model: turning', 'curve': 'SCC-V curve', 'map': 'SCC-M map curve',
            'cruise': 'Set speed'}[reason]

  def _update_state(self) -> None:
    sm = ui_state.sm
    if sm.recv_frame['carControl'] < ui_state.started_frame:
      self._filter = SlowReasonFilter()
      self._alpha_filter.x = 0.0
      self._text = ''
      return
    cc, cs, lead = sm['carControl'], sm['carState'], sm['radarState'].leadOne
    active = cc.longActive and not cs.gasPressed and cs.vEgo > V_MIN
    v_plan = sm['modelV2'].velocity.x
    reason = slow_reason(sm['longitudinalPlan'].longitudinalPlanSource.raw, sm['longitudinalPlanSP'].longitudinalPlanSource.raw,
                         lead.present, lead.dRel, lead.vLead, cs.vEgo, cs.leftBlinker or cs.rightBlinker,
                         v_plan[min(len(v_plan) - 1, 20)] if len(v_plan) else 1e3, v_plan[-1] if len(v_plan) else 1e3)
    if self._filter.update(rl.get_time(), active, cc.actuators.accel < ACCEL_ON, reason) is not None:
      # the MPC's second lead is radard's leadTwo
      shown = sm['radarState'].leadTwo if sm['longitudinalPlan'].longitudinalPlanSource == PlanSource.lead1 else lead
      self._text = self._text_for(self._filter.reason, shown.present, shown.dRel,
                                  sm['longitudinalPlanSP'].speedLimit.resolver.speedLimit, ui_state.is_metric)

  def _render(self, rect: rl.Rectangle) -> None:
    top = self._top(ui_state.sm['selfdriveState'].alertSize, rect, self._alert_bottom)
    if top is not None:
      self._top_shown = top
    alpha = self._alpha_filter.update(self._should_draw and self._filter.reason is not None and top is not None)
    if alpha < 1e-2 or not self._text:
      return
    top = self._top_shown

    # right of the driver monitoring icon (x 16 to 76)
    w = rect.width - LEFT_MARGIN - 8
    sz = measure_text_cached(self._font, self._text, FONT_SIZE)
    font_size = min(FONT_SIZE, FONT_SIZE * (w - 28) / max(sz.x, 1.))
    sz = measure_text_cached(self._font, self._text, round(font_size))
    box = rl.Rectangle(self._box_x(rect, sz.x), top, sz.x + 28, sz.y + 10)
    rl.draw_rectangle_rounded(box, 0.5, 10, rl.Color(0, 0, 0, int(255 * 0.6 * alpha)))
    rl.draw_text_ex(self._font, self._text, rl.Vector2(box.x + 14, box.y + 5), round(font_size), 0, rl.Color(255, 255, 255, int(255 * 0.9 * alpha)))
