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

SP_REASONS = {PlanSourceSP.sccVision: 'curve', PlanSourceSP.sccMap: 'map', PlanSourceSP.speedLimitAssist: 'limit'}


def slow_reason(plan_source: int, sp_source: int, lead_present: bool, d_rel: float, v_lead: float, v_ego: float) -> str:
  """Which planner candidate is braking: the MPC on a lead, the model, or a cruise target (SCC-V, SCC-M, SLA, set speed)."""
  if plan_source in (PlanSource.lead0, PlanSource.lead1, PlanSource.lead2):
    return 'lead'
  if plan_source == PlanSource.e2e:
    # the model slows for a close, slower radar lead too
    return 'lead' if lead_present and d_rel < max(40., 3. * v_ego) and v_lead < v_ego + 1. else 'model'
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

  def set_should_draw(self, should_draw: bool):
    self._should_draw = should_draw

  @staticmethod
  def _text_for(reason: str, lead_present: bool, d_rel: float, speed_limit: float, metric: bool) -> str:
    if reason == 'lead':
      if not lead_present or not math.isfinite(d_rel) or d_rel <= 0:
        return 'Car ahead'
      return f'Car ahead {round(d_rel)} m' if metric else f'Car ahead {round(d_rel * CV.METER_TO_FOOT / 10) * 10} ft'
    if reason == 'limit':
      limit = speed_limit * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH)
      return f'Speed limit {round(limit)}' if limit > 1 else 'Speed limit'
    return {'model': 'Slowing for road', 'curve': 'Curve ahead', 'map': 'Map curve', 'cruise': 'Slowing'}[reason]

  def _update_state(self) -> None:
    sm = ui_state.sm
    if sm.recv_frame['carControl'] < ui_state.started_frame:
      self._filter = SlowReasonFilter()
      self._alpha_filter.x = 0.0
      self._text = ''
      return
    cc, cs, lead = sm['carControl'], sm['carState'], sm['radarState'].leadOne
    active = cc.longActive and not cs.gasPressed and cs.vEgo > V_MIN
    reason = slow_reason(sm['longitudinalPlan'].longitudinalPlanSource.raw, sm['longitudinalPlanSP'].longitudinalPlanSource.raw,
                         lead.present, lead.dRel, lead.vLead, cs.vEgo)
    if self._filter.update(rl.get_time(), active, cc.actuators.accel < ACCEL_ON, reason) is not None:
      # the MPC's second lead is radard's leadTwo
      shown = sm['radarState'].leadTwo if sm['longitudinalPlan'].longitudinalPlanSource == PlanSource.lead1 else lead
      self._text = self._text_for(self._filter.reason, shown.present, shown.dRel,
                                  sm['longitudinalPlanSP'].speedLimit.resolver.speedLimit, ui_state.is_metric)

  def _render(self, rect: rl.Rectangle) -> None:
    visible = (self._should_draw and self._filter.reason is not None and
               ui_state.sm['selfdriveState'].alertSize == log.SelfdriveState.AlertSize.none)
    alpha = self._alpha_filter.update(visible)
    if alpha < 1e-2 or not self._text:
      return

    # right of the driver monitoring icon (x 16 to 76)
    x0, w = rect.x + LEFT_MARGIN, rect.width - LEFT_MARGIN - 8
    sz = measure_text_cached(self._font, self._text, FONT_SIZE)
    font_size = min(FONT_SIZE, FONT_SIZE * (w - 28) / max(sz.x, 1.))
    sz = measure_text_cached(self._font, self._text, round(font_size))
    box = rl.Rectangle(x0 + (w - sz.x) / 2 - 14, rect.y + 12, sz.x + 28, sz.y + 10)
    rl.draw_rectangle_rounded(box, 0.5, 10, rl.Color(0, 0, 0, int(255 * 0.6 * alpha)))
    rl.draw_text_ex(self._font, self._text, rl.Vector2(box.x + 14, box.y + 5), round(font_size), 0, rl.Color(255, 255, 255, int(255 * 0.9 * alpha)))
