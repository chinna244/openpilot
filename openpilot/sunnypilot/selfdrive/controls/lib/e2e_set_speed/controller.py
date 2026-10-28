"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Set Speed Assist for experimental mode.

The driving model has no set-speed input and keeps its own following distance. With no lead it
cruises at its own pace, a median 5.5 mph under the set speed in our experimental-mode logs; behind
a lead it followed a median 57 m further back than long_mpc's gap on aggressive. The planner's
min() keeps the car at the model's pace either way.

This lifts the e2e candidate toward what chill mode would do and never lowers it:

  e2e = a_model + lift,  lift -> gain * fade * max(0, floor - a_model)

floor is chill mode's answer, the lower of the planner's own lead MPC and cruise candidates
(upstream's cruise law with SCC and SLA's targets), so with nothing to slow for experimental mode
drives and follows as chill does. gain fades the lift out as the model eases off (its own
acceleration and its throttle probability) and below walking pace, and fade as the road ahead bends
at the current speed. Every slowdown is the model's: the lift only lets go of it, and a lead that
brakes binds through the MPC candidate at once.

A lift that could go negative made the car pump on a straight road: the plan's far end flickers
by several mph from one frame to the next, and handing back added speed on each flicker
alternated throttle and brake every 15-20 s. Read only through the model's own acceleration and
probability, and the road's geometry, the lift has no such input to chase. Design and data:
docs/zoompilot/e2e-set-speed.md.
"""
import math

import numpy as np

from opendbc.car import rate_limit
import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

Inhibit = custom.LongitudinalPlanZP.E2ESetSpeed.Inhibit
LaneChangeState = log.LaneChangeState

PARAM = "ExperimentalModeSetSpeed"

# The lift fades in above walking pace: stop-and-go stays the model's.
MIN_SPEED = 5.0  # m/s
FULL_SPEED = 8.0  # m/s

# The model's acceleration wanders +-0.15 m/s^2 below 0.3 Hz in cruise and settles near -0.05..-0.1
# at the set speed, where it pushes back toward its own pace; the gain ramps below both.
GAIN_BP = [-0.3, -0.1]  # m/s^2
# No lift where the road ahead bends harder than this at the current speed.
FADE_LAT_BP = [0.6, 1.0]  # m/s^2
LAT_HORIZON = 10.0  # s

# The lift rises at upstream's cruise jerk, as the cruise candidate it stands in for, and falls fast
# so the model's own braking is never held off.
LIFT_FALL = 2.4  # m/s^3

# Local tuning (this fork, Mazda CX-5), carried onto the lift from the earlier set-speed floor.
# The floor approaches the cruise target the way stock cruise does, over TAU, and never past the
# ceiling for the speed: a full floor at Mazda's ceiling surged the car on top of the model's own pull
# (route 00000159), so the ceiling and the lift's rise ease from the town values to the highway ones
# across FLOOR_SPEED_BP.
TAU = 4.0  # s
FLOOR_MAX = 0.8  # m/s^2
FLOOR_SPEED_BP = [15., 25.]  # m/s
FLOOR_MAX_HIGHWAY = 0.5  # m/s^2
LIFT_RISE = 0.5  # m/s^3
LIFT_RISE_HIGHWAY = 0.25  # m/s^3

# Town: the lift over the model is held to TOWN_LIFT_MAX below TOWN_LIFT_BP[0], easing back to FLOOR_MAX by
# TOWN_LIFT_BP[1]. A hesitant model under the full floor pushed 0.6 for 7 s into stops it saw late (routes
# 0000016a t=312-318, 0000016b t=387-394); 0.6 with FLOOR_MAX 0.8 kept the same pickup gain as 0.8 in replay
# (median 0.40 s on 20 pickups, 00000174-00000177).
TOWN_LIFT_BP = [12.5, 15.6]  # m/s, 28 to 35 mph
TOWN_LIFT_MAX = 0.6  # m/s^2

# Plan-lead braking. Before a stop the model's action lags its own plan: route 0000016a t=153-158 the plan slowed
# to 5 mph within 100-130 m (a constant -0.7 to -1.0 m/s^2) while the action stayed at -0.04 to -0.26, then it
# braked at -2.3 and the driver took over. Without a lead, once the plan has needed PLB_ON or more for PLB_ARM,
# the e2e candidate brakes at PLB_GAIN of what the plan needs until it needs less than PLB_OFF. Replayed on routes
# 00000166-0000016a: 27 episodes, 22 ahead of a real slowdown, 5 ahead of a partial one, none on a clear road.
PLB_ON = -0.6  # m/s^2
PLB_OFF = -0.3  # m/s^2
PLB_ARM = 0.5  # s
PLB_GAIN = 0.8
PLB_MIN = -1.5  # m/s^2, past this the model's own braking leads
PLB_DROP = 0.5  # m/s, plan speeds this far under the start count as slowing
PLB_JERK = 1.0  # m/s^3, into the brake
PLB_RELEASE = 1.0  # m/s^3, out of it

T_IDXS = np.asarray(ModelConstants.T_IDXS)
LAT_MASK = T_IDXS <= LAT_HORIZON


def floor_max(v_ego: float) -> float:
  return float(np.interp(v_ego, FLOOR_SPEED_BP, [FLOOR_MAX, FLOOR_MAX_HIGHWAY]))


def lift_rise(v_ego: float) -> float:
  return float(np.interp(v_ego, FLOOR_SPEED_BP, [LIFT_RISE, LIFT_RISE_HIGHWAY]))


def lift_max(v_ego: float) -> float:
  return float(np.interp(v_ego, TOWN_LIFT_BP, [TOWN_LIFT_MAX, FLOOR_MAX]))


def plan_decel(vel: np.ndarray, pos: np.ndarray) -> float:
  """Constant acceleration (<= 0) that meets the slowest-to-reach point of the model's plan."""
  v0 = float(vel[0])
  slow = vel < v0 - PLB_DROP
  if not slow.any():
    return 0.
  return float(np.min((vel[slow] ** 2 - v0 ** 2) / (2. * np.maximum(pos[slow] - pos[0], 1.))))


class E2ESetSpeedController:
  def __init__(self, params=None, dt: float = DT_MDL):
    self.params = params or Params()
    self.dt = dt
    self.params_frames = int(PARAMS_UPDATE_PERIOD / dt)
    self.plb_arm_frames = int(round(PLB_ARM / dt))
    self.frame = -1
    self.enabled = False
    self._reset(Inhibit.disabled)

  def _reset(self, reason) -> None:
    self.gain = 0.
    self.fade = 0.
    self.floor = 0.
    self.lift = 0.
    self.plb_count = 0
    self.plb_armed = False
    self.slowing_count = 0
    # an invalid model frame lets an active plan-lead brake ease out instead of dropping it
    if reason != Inhibit.invalid:
      self.plb = 0.
      self.plb_active = False
    self.inhibit = reason

  def _update_params(self) -> None:
    if self.frame % self.params_frames == 0:
      self.enabled = self.params.get_bool(PARAM)

  def fill(self, msg) -> None:
    """Report into LongitudinalPlanZP.e2eSetSpeed."""
    msg.gain = float(self.gain)
    msg.fade = float(self.fade)
    msg.floor = float(self.floor)
    msg.boost = float(self.lift)
    msg.inhibit = self.inhibit

  def update(self, sm: messaging.SubMaster, a_model: float, a_chill: float, is_e2e: bool, reset_state: bool,
             steer_lat_accel: float = 0., v_cruise: float = math.inf) -> float:
    """Return the e2e candidate; a_model unchanged whenever the feature is off or idle.
    a_chill: chill mode's acceleration, the lower of the planner's lead MPC and cruise candidates.
    steer_lat_accel: lateral acceleration from the measured steering angle.
    v_cruise: this frame's cruise target after SCC and SLA, which the floor approaches over TAU."""
    self.frame += 1
    self._update_params()

    # switched off with a lift on: it goes out at LIFT_FALL below, then the controller idles; so does a
    # plan-lead brake, at PLB_RELEASE
    if not self.enabled and self.lift == 0. and not self.plb_active:
      self._reset(Inhibit.disabled)
      return a_model
    if not is_e2e or reset_state:
      self._reset(Inhibit.inactive)
      return a_model

    md = sm['modelV2']
    v_ego = sm['carState'].vEgo
    vel = np.asarray(md.velocity.x, dtype=float)
    yaw_rate = np.asarray(md.orientationRate.z, dtype=float)
    pos = np.asarray(md.position.x, dtype=float)
    curvature = md.action.desiredCurvature
    throttle_probs = md.meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.
    if (any(len(x) != ModelConstants.IDX_N for x in (vel, yaw_rate, pos)) or
        not (np.isfinite(vel).all() and np.isfinite(yaw_rate).all() and np.isfinite(pos).all()) or
        not all(math.isfinite(x) for x in (v_ego, a_model, a_chill, throttle_prob, curvature)) or math.isnan(v_cruise)):
      self._reset(Inhibit.invalid)
      return self._plan_lead(a_model, None) if math.isfinite(a_model) else a_model

    # the road's geometry ahead at the current speed, not the plan's speeds, which flicker
    lat_now = max(abs(curvature) * v_ego ** 2, abs(steer_lat_accel))
    kappa_ahead = float(np.max((np.abs(yaw_rate) / np.maximum(vel, 1.))[LAT_MASK]))
    lat_ahead = max(lat_now, kappa_ahead * v_ego ** 2)

    approach = float(np.clip((v_cruise - v_ego) / TAU, 0., floor_max(v_ego))) if math.isfinite(v_cruise) else floor_max(v_ego)
    self.floor = min(a_chill, approach)
    # stop-and-go stays the model's: the gain fades in above walking pace
    self.gain = (float(np.interp(a_model, GAIN_BP, [0., 1.])) * float(np.clip(throttle_prob, 0., 1.)) *
                 float(np.interp(v_ego, [MIN_SPEED, FULL_SPEED], [0., 1.])))
    self.fade = float(np.interp(lat_ahead, FADE_LAT_BP, [1., 0.]))

    if not self.enabled:
      self.inhibit = Inhibit.disabled
    elif md.meta.laneChangeState != LaneChangeState.off:
      self.inhibit = Inhibit.laneChange
    else:
      self.inhibit = Inhibit.none
    # No lift once the model's plan has slowed by PLB_OFF or more for PLB_ARM: the old envelope held it off
    # there, and a hesitant model under the full floor pushed into stops it saw late (routes 0000016a,
    # 0000016b). A plan whose far end flickers frame to frame never holds that long.
    req = plan_decel(vel, pos)
    self.slowing_count = self.slowing_count + 1 if req <= PLB_OFF else 0
    slowing = self.slowing_count >= self.plb_arm_frames
    target = min(self.gain * self.fade * max(0., self.floor - a_model), lift_max(v_ego)) if self.inhibit == Inhibit.none and not slowing else 0.
    self.lift = rate_limit(target, self.lift, -LIFT_FALL * self.dt, lift_rise(v_ego) * self.dt)

    # Arms on PLB_ARM of consecutive need, only at speed; once armed it stays through the final approach under
    # MIN_SPEED, where the model's lag would otherwise leave it the hardest braking.
    if not self.enabled or sm['radarState'].leadOne.present or req > PLB_OFF:
      self.plb_count, self.plb_armed = 0, False
    elif not self.plb_armed:
      self.plb_count = self.plb_count + 1 if req <= PLB_ON and v_ego >= MIN_SPEED else 0
      self.plb_armed = self.plb_count >= self.plb_arm_frames
    return self._plan_lead(a_model + self.lift, max(PLB_GAIN * req, PLB_MIN) if self.plb_armed else None)

  def _plan_lead(self, candidate: float, want: float | None) -> float:
    """The e2e candidate under plan-lead braking. want: the brake it asks for, None when disarmed. It eases from
    the candidate down to want at PLB_JERK and back up to the candidate at PLB_RELEASE, so neither end steps."""
    if not self.plb_active:
      if want is None:
        return candidate
      self.plb_active, self.plb = True, candidate
    target = candidate if want is None else min(candidate, want)
    self.plb = float(np.clip(target, self.plb - PLB_JERK * self.dt, self.plb + PLB_RELEASE * self.dt))
    if want is None and self.plb >= candidate:
      self.plb_active = False
      return candidate
    return min(candidate, self.plb)
