"""The Mazda's model curvature settings: Lane Center Assist (Blendful) and big model smoothing,
from zoompilot develop 197598c0f6 (sunnypilot/modeld_v2/model_settings.py and its calls in
selfdrive/modeld/modeld.py).

develop applies both in modeld, to the action the model just produced and before it becomes
prev_action and modelV2.action. The port's modeld.py is pinned (StarPilot's Auto Learn hashes its
bytes), so the same step runs from the common model join it already calls once per published
frame, the Subaru filter_action (starpilot/car/subaru/smoothing.py), at the same point: after the
model's action, before prev_action and the message. modeld's own frame supplies what develop
passes in (the parsed outputs, the model, its desire and the lane change state).
"""
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value
from openpilot.starpilot.car.mazda.lateral.blendful import Blendful

PARAMS_FRAMES = 60
MIN_LAT_CONTROL_SPEED = 0.3  # modeld's; Blendful's correction is only added above it, as develop's with_blendful

# the small model bundles' `lat` override. A big model runs comma's stock modeld constants, which smooth
# nothing since openpilot #36987; on straights that leaves a ~1 Hz wiggle in its curvature (11-11 drives)
BIG_MODEL_LAT_SMOOTH_SECONDS = 0.1

LANE_CHANGE_OFF = log.LaneChangeState.off
DESIRE_NONE = log.Desire.none


class ModelSettings:
  """Blendful and big model smoothing, as develop's ModelSettings runs them (camera offset and steer
  delay stay StarPilot's)."""

  def __init__(self, params=None):
    self.params = params
    self.blendful = Blendful()
    self.blendful_enabled = False
    self.big_model_smoothing = False
    self.frame = 0
    self.failed = False

  def refresh(self) -> bool:
    """Reads the params every PARAMS_FRAMES frames; True on the frames it read them."""
    frame, self.frame = self.frame, self.frame + 1
    if frame % PARAMS_FRAMES != 0:
      return False
    if self.params is None:
      self.params = Params()
    self.blendful_enabled = self.params.get_bool("Blendful")
    self.big_model_smoothing = self.params.get_bool("BigModelSmoothing")
    return True

  def big_lat_smooth(self, model) -> float:
    """Seconds of curvature smoothing this frame adds on top of the model's own: the small models'
    amount for a big model that smooths none itself, with the toggle on; 0 otherwise. Checked per
    frame, as jetlink can hand the drive between models at any frame. Not added to the delay the model
    plans for (develop: telling the model the car is slower feeds the ~1 Hz loop it damps)."""
    if not (self.big_model_smoothing and getattr(model, 'chestnut', False)) or getattr(model, 'LAT_SMOOTH_SECONDS', 0.0) > 0.0:
      return 0.0
    return BIG_MODEL_LAT_SMOOTH_SECONDS

  @staticmethod
  def smooth_curvature(action, prev_action, seconds: float) -> None:
    if seconds > 0.0:
      action.desiredCurvature = float(smooth_value(action.desiredCurvature, prev_action.desiredCurvature, seconds))

  def apply(self, action, prev_action, model_output, model, v_ego: float, desire, lane_change_state) -> None:
    """develop's modeld order: Blendful's correction on the model's curvature (above
    MIN_LAT_CONTROL_SPEED), then big model smoothing against prev_action."""
    self.refresh()
    big_lat_smooth = self.big_lat_smooth(model)
    active = desire == DESIRE_NONE and lane_change_state == LANE_CHANGE_OFF
    correction = self.blendful.update(model_output, v_ego, active, action.desiredCurvature, enabled=self.blendful_enabled)
    if v_ego > MIN_LAT_CONTROL_SPEED:
      action.desiredCurvature = float(action.desiredCurvature + correction)
    self.smooth_curvature(action, prev_action, big_lat_smooth)


_settings: ModelSettings | None = None


def model_join(action, prev_action, frame) -> None:
  """The hook filter_action calls with modeld's main-loop frame. Mazda only; any other car, or a
  frame without the names develop's step reads (replay tools calling filter_action), is left alone."""
  global _settings
  f = frame.f_locals
  CP = f.get('CP')
  if CP is None or getattr(CP, 'brand', None) != 'mazda':
    return
  if _settings is None:
    _settings = ModelSettings()
  model_output, model, dh = f.get('model_output'), f.get('model'), f.get('DH')
  if model_output is None or model is None or dh is None or 'desire' not in f or 'v_ego' not in f:
    if not _settings.failed:
      # a Mazda caller without modeld's names: say so once rather than drop Blendful silently
      _settings.failed = True
      cloudlog.error("mazda model settings: caller lacks modeld's names; Blendful and big model smoothing off")
    return
  curvature = action.desiredCurvature
  try:
    _settings.apply(action, prev_action, model_output, model, f['v_ego'], f['desire'], dh.lane_change_state)
  except Exception:
    # never take modeld down: a model whose outputs lack a head Blendful reads drives unassisted
    action.desiredCurvature = curvature
    if not _settings.failed:
      _settings.failed = True
      cloudlog.exception("mazda model settings failed; curvature left as the model gave it")
