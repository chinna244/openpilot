"""Vision Force Stop: StarPilot's ForceStopRuntime (starpilot/longitudinal/force_stop_runtime.py, Dom 1d30dc3), sign path only.

A camera STOP sign confirmed by the reader feeds StarPilot's ForceStop policy unchanged: it acts below 40 mph when the
model's own plan ends within 50 m, with no curve, turn or slower lead, caps the speed toward the sign's camera range (box
height, 2a854471f) and holds the stop until the gas or RES; stopped behind a car at a sign, it stops again at the line once
that car leaves. StarPilot's red-light path (StopLightDetector), the dashboard TSR sign and the distance offset are not
part of this port.
"""

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.vision_force_stop.curve import frozen_raw_curve
from openpilot.sunnypilot.selfdrive.controls.lib.vision_force_stop.force_stop import ForceStop, StopFrame, StopPlan, vehicle_tune
from openpilot.sunnypilot.selfdrive.controls.lib.vision_force_stop.lead_tracking import LeadDetector
from openpilot.sunnypilot.selfdrive.controls.lib.vision_force_stop.model_geometry import normalized_origin, road_curvature, _finite
from openpilot.sunnypilot.selfdrive.controls.lib.vision_force_stop.stop_confirm import (CONTROL_STOP_MAX_AGE_NS, StopConfirmation,
                                                                                         read_stop_evidence)
from openpilot.sunnypilot.selfdrive.controls.lib.vision_force_stop.stop_resume import StopResume

KEY = 'VisionForceStop'
SOURCE_AGE_NS = 150_000_000
SERVICES = ('modelV2', 'carState', 'carControl', 'controlsState', 'selfdriveState', 'radarState')


def model_horizon(model) -> float | None:
  """StarPilot's conditional_mode.projection._horizon: the end of the model's 10 s path, validated."""
  try:
    distances = normalized_origin(tuple(model.position.x))
    if len(distances) != 33:
      return None
    parsed = tuple(_finite(value, low=0.0, high=500.0) for value in distances)
    if any(value is None for value in parsed):
      return None
    if any(b < a for a, b in zip(parsed, parsed[1:], strict=False)):
      return None
    return parsed[-1]
  except (AttributeError, TypeError, ValueError, OverflowError):
    return None


class VisionForceStop:
  def __init__(self, params: Params | None = None):
    self.params = params
    self.policy = ForceStop()
    self.resume = StopResume()
    self.vision_stop = StopConfirmation()
    self.vision_ranged_ns = 0
    self.lead = LeadDetector()
    self.plan = StopPlan()
    self.enabled = False
    self.next_refresh_ns = 0

  def reset(self):
    self.vision_stop.reset()
    self.vision_ranged_ns = 0
    self.resume.reset()
    self.policy.reset()
    self.lead.reset()
    self.plan = StopPlan()

  def _refresh(self, now_ns: int):
    if self.params is None or now_ns < self.next_refresh_ns:
      return
    self.next_refresh_ns = now_ns + 1_000_000_000
    enabled = self.params.get_bool(KEY) and self.params.get_bool('SpeedLimitVision')
    if enabled != self.enabled:
      self.reset()
    self.enabled = enabled

  def sample(self, sm, cp, *, now_ns: int, drive_id: int, follow_seconds: float | None) -> StopPlan:
    self._refresh(now_ns)
    try:
      current = all(sm.valid[name] and sm.alive[name] and drive_id < sm.logMonoTime[name] <= now_ns and
                    0 <= now_ns - sm.logMonoTime[name] <= SOURCE_AGE_NS for name in SERVICES)
      car, control, state, model = (sm[name] for name in ('carState', 'carControl', 'selfdriveState', 'modelV2'))
      if (not self.enabled or not cp.openpilotLongitudinalControl or cp.passive or cp.dashcamOnly or cp.notCar or
          drive_id <= 0 or not current or not car.canValid or car.canTimeout or not (control.longActive or control.latActive or car.gasPressed) or
          sm['controlsState'].forceDecel):
        self.reset()
        return self.plan
      stamp = int(sm.logMonoTime['modelV2'])
      horizon = model_horizon(model)
      curvature = road_curvature(model, car.vEgo)
      raw_curve = frozen_raw_curve(model, car.vEgo, sm['controlsState'].curvature, car.leftBlinker, car.rightBlinker)
      lead = sm['radarState'].leadOne
      perception_available = horizon is not None and curvature is not None and raw_curve is not None
      tracked = None
      if perception_available:
        tracked = self.lead.step(lead, model, speed_mps=float(car.vEgo), t_follow_s=follow_seconds,
                                 standstill=bool(car.standstill), observed_mono_s=stamp / 1e9, now_mono_s=now_ns / 1e9,
                                 radar_fresh=True, model_fresh=True, car_fresh=True, headway_fresh=follow_seconds is not None)
        perception_available = tracked.tracked is not None
      else:
        self.lead.reset()
      resume_requested = self.resume.consume(now_ns=now_ns, drive_id=drive_id, car_ns=int(sm.logMonoTime['carState']))
      # Only actual longitudinal ownership may produce a stop target or hold. fork: a gas override (longActive off, still
      # engaged) is not a reset: StarPilot's reset there wiped the 10 s release window, so a car leaving a sign stop on
      # the gas could re-arm on the same sign; the planner ignores the plan while the gas is held (reset_state)
      if not state.enabled or car.brakePressed or not (control.longActive or car.gasPressed):
        self.resume.reset()
        self.policy.reset()
        self.vision_ranged_ns = 0  # fork: a confirmation still fresh after the reset is ranged again, not seeded at 27 m
        self.plan = StopPlan()
        return self.plan
      vision_sign = self.vision_stop.observe(read_stop_evidence(sm, now_ns=now_ns, drive_id=drive_id), now_ns=now_ns)
      vision_fresh = bool(vision_sign and 0 <= now_ns - vision_sign <= CONTROL_STOP_MAX_AGE_NS)
      vision_distance = None
      if vision_fresh and vision_sign != self.vision_ranged_ns and self.vision_stop.distance_m is not None:
        self.vision_ranged_ns = vision_sign
        vision_distance = self.vision_stop.distance_m - float(car.vEgo) * (stamp - vision_sign) / 1e9
      frame = StopFrame(stamp, float(car.vEgo), horizon, False, horizon is not None and horizon < 50,
                        bool(model.action.shouldStop), bool(car.standstill), True, bool(car.gasPressed),
                        tracked is not None and tracked.tracked is True, bool(lead.present), float(lead.dRel), float(lead.vLead),
                        raw_curve[1] if raw_curve is not None else False, curvature[0] if curvature is not None else None,
                        bool(car.leftBlinker or car.rightBlinker), float(car.steeringAngleDeg), resume_requested=resume_requested,
                        perception_available=perception_available, sign=vision_fresh, vision_distance_m=vision_distance)
      self.plan = self.policy.step(frame, vehicle_tune(str(cp.carFingerprint)), 0)
      return self.plan
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError, RuntimeError):
      self.reset()
      return self.plan
