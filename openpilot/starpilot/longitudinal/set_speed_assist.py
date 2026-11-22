"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Set Speed Assist for experimental mode, ported from chinna244/openpilot develop
(openpilot/sunnypilot/selfdrive/controls/lib/e2e_set_speed/controller.py, 23cfaa66b7 era) for the Mazda port.
Same law and constants; the toggle is StarPilot's ExperimentalSpeedNudge, passed in per frame, and the
develop log struct (LongitudinalPlanZP.e2eSetSpeed) is not published.

The driving model has no set-speed input and keeps its own following distance. With no lead it
cruises at its own pace, a median 5.5 mph under the set speed in our experimental-mode logs; behind
a lead it followed a median 57 m further back than long_mpc's gap on aggressive. The planner's
min() keeps the car at the model's pace either way.

This lifts the e2e candidate toward what chill mode would do and never lowers it:

  e2e = a_model + lift,  lift -> authority * max(0, floor - a_model),  authority -> gain * fade

floor is chill mode's answer, the lower of the planner's own lead MPC and cruise candidates
(upstream's cruise law with SCC and SLA's targets), so with nothing to slow for experimental mode
drives and follows as chill does. gain fades the lift out as the model eases off (its own
acceleration and its throttle probability), now or over its plan (a stop, a light, a closing
lead), and below walking pace, and fade as the road ahead bends at the current speed. The
authority moves on a jerk budget, except to let go for the model (REENGAGE_JERK). Every
slowdown is the model's: the lift only lets go of it, and a lead that brakes binds through the
MPC candidate at once.

A lift that could go negative made the car pump on a straight road: the plan's far end flickers
by several mph from one frame to the next, and handing back added speed on each flicker
alternated throttle and brake every 15-20 s. The lift reads the plan's speeds only for a slowdown
well past that flicker, and only to let go, so it has no such input to chase. Design and data:
docs/zoompilot/e2e-set-speed.md.
"""
import math
from enum import IntEnum

import numpy as np

from opendbc.car import rate_limit
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants


class Inhibit(IntEnum):
  # develop's custom.LongitudinalPlanZP.E2ESetSpeed.Inhibit values still in use
  disabled = 0
  none = 1
  inactive = 2
  invalid = 4
  laneChange = 16


LaneChangeState = log.LaneChangeState

# develop's firm_stop.plan_stop_distance, the only part of firm stop the lift reads
FIRM_STOP_PLAN_V = 0.3  # m/s, plan speed that counts as stopped
FIRM_STOP_GO_V = 0.5  # m/s, a plan this fast within FIRM_STOP_STAY of its stop point stops and goes: not a stop
FIRM_STOP_STAY = 2.0  # s

# The lift fades in above walking pace: stop-and-go stays the model's.
MIN_SPEED = 5.0  # m/s
FULL_SPEED = 8.0  # m/s

# The model's acceleration wanders +-0.15 m/s^2 below 0.3 Hz in cruise and settles near -0.05..-0.1
# at the set speed, where it pushes back toward its own pace; the gain ramps below both.
GAIN_BP = [-0.3, -0.1]  # m/s^2
# No lift where the road ahead bends harder than this at the current speed.
FADE_LAT_BP = [0.6, 1.0]  # m/s^2
HORIZON = 10.0  # s
# The planned slowdown, read through GAIN_BP as its mean over HORIZON: the plan ending 1-3 m/s under
# the current speed (its far end flickers by about 1 m/s on a straight road), or a lead closing this
# fast. A lead counts in full under 15 s to contact and not at all past 25 s.
LEAD_TTC_BP = [15., 25.]  # s

# The lift rises at upstream's cruise jerk, as the cruise candidate it stands in for. This is the
# slowest it eases off, e.g. on a lane change or a bend.
LIFT_FALL = 2.4  # m/s^3

# Local tuning (this fork, Mazda CX-5), carried onto the lift from the earlier set-speed floor.
# The floor approaches the cruise target the way stock cruise does, over TAU, and never past the
# ceiling for the speed: a full floor at Mazda's ceiling surged the car on top of the model's own pull
# (route 00000159), so the ceiling and the lift's rise ease from the town values to the highway ones
# across FLOOR_SPEED_BP. Raised from 0.8, where free-road town pickups sat at the floor under a model asking 0.4-0.6:
# replaying 396 min of engaged driving (routes 00000147-00000036), 15-25 / 20-30 mph pickups 5.7 / 6.6 s -> 5.5 / 6.3 s
# against this driver's 5.4 / 6.3 s, 3 more catch-up kickdowns (76 before), at most 0.19 m/s^2 more in the 3 s before
# 4 of 964 brakes.
TAU = 4.0  # s
FLOOR_MAX = 0.9  # m/s^2
FLOOR_SPEED_BP = [15., 25.]  # m/s
FLOOR_MAX_HIGHWAY = 0.5  # m/s^2
LIFT_RISE = 0.5  # m/s^3
LIFT_RISE_HIGHWAY = 0.25  # m/s^3

# Town: the lift over the model is held to TOWN_LIFT_MAX below TOWN_LIFT_BP[0], easing back to FLOOR_MAX by
# TOWN_LIFT_BP[1]. A hesitant model under the full floor pushed 0.6 for 7 s into stops it saw late (routes
# 0000016a t=312-318, 0000016b t=387-394); 0.6 with FLOOR_MAX then 0.8 kept the same pickup gain as 0.8 in replay
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

# Departure. After an engaged stop the model can crawl at 2-5 mph for seconds across an empty intersection: route
# 00000066 t=251 a stop-sign departure held 3.7-4.7 mph for 3 s with the plan at -0.1..0.0, 00000037 t=257 2 mph for
# 7 s, both until the driver pressed again. When the driver presses the gas within DEPART_STOP_T of an engaged stop
# and lets go under DEPART_MAX_V, straight, no blinker and no lead within DEPART_LEAD_D, the lift works below walking
# pace and without the throttle probability for DEPART_T or until FULL_SPEED; the model's own braking still lets go
# at once. A brake, a lead, a blinker, a turn or a planned stop ends it at once, otherwise it eases out at LIFT_FALL. Over 231
# engaged releases under 8 mph in 195 drives it armed 33 times and acted 11, the driver braking in none; with
# blinkers allowed two waits to turn (00000060 t=946, 00000180 t=135) pushed and the driver braked.
DEPART_STOP_V = 0.1  # m/s
DEPART_STOP_T = 5.0  # s
DEPART_MAX_V = 3.6  # m/s, 8 mph
DEPART_T = 4.0  # s
DEPART_REENGAGE_T = 1.0  # s, longitudinal control back after the press
DEPART_LEAD_D = 30.  # m
DEPART_STEER_ARM = 30.  # deg
DEPART_STEER_END = 45.  # deg
# With k = gain * fade, e2e = a_model + k * gap = (1 - k) * a_model + k * floor (gap = floor - a_model):
# a blend of two smooth signals, plus k' * gap. That term is all the jerk the lift adds, and k is read
# each frame from flickering inputs (the path's bends over 10 s, the model's wander), so k moves on a
# jerk budget: it re-engages at most REENGAGE_JERK / gap a second, and the fade, a comfort gate (curve
# speed is SCC's, through the floor), moves at most FADE_JERK / gap either way. Letting go for the
# model (the gain, a lane change) is still at once. docs/zoompilot/e2e-set-speed.md, "Authority".
REENGAGE_JERK = 0.3  # m/s^3
FADE_JERK = 0.3  # m/s^3
MIN_GAP = 0.05  # m/s^2, only to bound the step at a closed gap

T_IDXS = np.asarray(ModelConstants.T_IDXS)
HORIZON_MASK = T_IDXS <= HORIZON


def plan_stop_distance(vel, pos) -> float:
  """Distance to the first point of the model's plan slower than FIRM_STOP_PLAN_V that it stays stopped after, inf if it
  plans none. A plan that starts that slow (pulling away) plans no stop ahead."""
  vel, pos = np.asarray(vel, dtype=float), np.asarray(pos, dtype=float)
  if len(vel) < 2 or len(vel) != len(pos) or not (np.isfinite(vel).all() and np.isfinite(pos).all()) or vel[0] < FIRM_STOP_PLAN_V:
    return math.inf
  stop = np.flatnonzero(vel < FIRM_STOP_PLAN_V)
  if not len(stop):
    return math.inf
  i = stop[0]
  if len(vel) == len(T_IDXS) and (vel[i:][T_IDXS[i:] <= T_IDXS[i] + FIRM_STOP_STAY] >= FIRM_STOP_GO_V).any():
    return math.inf
  return float(pos[i] - pos[0])


def planned_slowdown(v_ego: float, plan_v: np.ndarray, lead) -> float:
  """m/s the car is set to shed over the horizon: down to the plan's lowest speed, or a closing
  lead's closing speed as it nears contact."""
  drop = v_ego - float(np.min(plan_v[HORIZON_MASK]))
  if lead.present and math.isfinite(lead.vRel) and math.isfinite(lead.dRel) and lead.vRel < 0.:
    ttc = max(lead.dRel, 0.) / -lead.vRel
    drop = max(drop, -lead.vRel * float(np.interp(ttc, LEAD_TTC_BP, [1., 0.])))
  return drop


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
  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    self.plb_arm_frames = int(round(PLB_ARM / dt))
    self.frame = -1
    self.enabled = False
    # departure tracking runs through resets: a gas override resets the controller every frame
    self.since_stop = math.inf
    self.gas_prev = False
    self.prev_reset = True
    self.press_from_stop = False
    self.depart_t: float | None = None  # s into a departure window
    self.depart_ease = False  # a window that ran out, its lift easing off
    self.reengage_jerk = REENGAGE_JERK
    self.fade_jerk = FADE_JERK
    self._reset(Inhibit.disabled)

  def _reset(self, reason) -> None:
    self.gain = 0.
    self.fade = 0.
    self.plan_fade = 0.
    self.floor = 0.
    self.authority = 0.
    self.lift = 0.
    self.plb_count = 0
    self.plb_armed = False
    self.slowing_count = 0
    self.depart_ease = False
    # an invalid model frame lets an active plan-lead brake ease out instead of dropping it
    if reason != Inhibit.invalid:
      self.plb = 0.
      self.plb_active = False
    self.inhibit = reason
    self._fresh = True  # the fade starts from the road as it is, with nothing to smooth yet

  def _update_departure(self, sm, reset_state: bool) -> None:
    cs = sm['carState']
    lead = sm['radarState'].leadOne
    # only an engaged stop counts
    self.since_stop = 0. if cs.vEgo < DEPART_STOP_V and not reset_state else self.since_stop + self.dt
    lead_near = lead.present and not lead.dRel >= DEPART_LEAD_D  # a NaN distance counts as near
    blinker = cs.leftBlinker or cs.rightBlinker
    turning = abs(cs.steeringAngleDeg) > DEPART_STEER_END
    if cs.gasPressed and not self.gas_prev:
      # a press from an engaged stop: the frame before it was engaged
      self.press_from_stop = self.since_stop <= DEPART_STOP_T and not self.prev_reset
      self.depart_t, self.depart_ease = None, False
    elif self.gas_prev and not cs.gasPressed:
      if (self.press_from_stop and cs.vEgo < DEPART_MAX_V and not lead_near and not blinker and
          abs(cs.steeringAngleDeg) < DEPART_STEER_ARM):
        self.depart_t = 0.
      self.press_from_stop = False
    elif self.depart_t is not None:
      self.depart_t += self.dt
      if cs.brakePressed or lead_near or blinker or turning or (reset_state and self.depart_t > DEPART_REENGAGE_T):
        self.depart_t, self.depart_ease = None, False
      elif self.depart_t > DEPART_T or cs.vEgo >= FULL_SPEED:
        self.depart_t, self.depart_ease = None, True
    if cs.brakePressed or lead_near or blinker or turning:
      self.depart_ease = False
    self.gas_prev = cs.gasPressed
    self.prev_reset = reset_state

  def update(self, sm, a_model: float, a_chill: float, is_e2e: bool, reset_state: bool,
             steer_lat_accel: float = 0., v_cruise: float = math.inf, *, enabled: bool) -> float:
    """Return the e2e candidate; a_model unchanged whenever the feature is off or idle.
    a_chill: chill mode's acceleration, the lower of the planner's lead MPC and cruise candidates.
    steer_lat_accel: lateral acceleration from the measured steering angle.
    v_cruise: this frame's cruise target after SCC and SLA, which the floor approaches over TAU."""
    self.frame += 1
    self.enabled = enabled
    self._update_departure(sm, reset_state)

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
        not all(math.isfinite(x) for x in (v_ego, a_model, a_chill, throttle_prob, curvature, steer_lat_accel)) or
        math.isnan(v_cruise)):
      self._reset(Inhibit.invalid)
      return self._plan_lead(a_model, None) if math.isfinite(a_model) else a_model

    # a departure ends at once when the model plans a stop: at a crawl a planned stop sheds too little speed for
    # plan_fade, and too gently for the slowing gate, to let go of it
    if self.depart_t is not None and math.isfinite(plan_stop_distance(vel, pos)):
      self.depart_t, self.depart_ease = None, False

    # the road's geometry ahead at the current speed, not the plan's speeds, which flicker
    lat_now = max(abs(curvature) * v_ego ** 2, abs(steer_lat_accel))
    kappa_ahead = float(np.max((np.abs(yaw_rate) / np.maximum(vel, 1.))[HORIZON_MASK]))
    lat_ahead = max(lat_now, kappa_ahead * v_ego ** 2)

    approach = float(np.clip((v_cruise - v_ego) / TAU, 0., floor_max(v_ego))) if math.isfinite(v_cruise) else floor_max(v_ego)
    self.floor = min(a_chill, approach)
    # the model's decision, now and over its plan: a gentle coast toward a stop still lets go
    self.plan_fade = float(np.interp(-planned_slowdown(v_ego, vel, sm['radarState'].leadOne) / HORIZON, GAIN_BP, [0., 1.]))
    # stop-and-go stays the model's: the gain fades in above walking pace, except in a departure window
    model_term = min(float(np.interp(a_model, GAIN_BP, [0., 1.])), self.plan_fade)
    speed_fade = float(np.interp(v_ego, [MIN_SPEED, FULL_SPEED], [0., 1.]))
    window = self.depart_t is not None
    model_gain = model_term * (1. if window else speed_fade)
    self.gain = model_gain * (1. if window else float(np.clip(throttle_prob, 0., 1.)))
    gap = max(0., self.floor - a_model)
    fade = float(np.interp(lat_ahead, FADE_LAT_BP, [1., 0.]))
    fade_step = self.fade_jerk * self.dt / max(gap, MIN_GAP)
    self.fade = fade if self._fresh else float(np.clip(fade, self.fade - fade_step, self.fade + fade_step))
    self._fresh = False

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
    authority = self.gain * self.fade if self.inhibit == Inhibit.none and not slowing else 0.
    self.authority = min(authority, self.authority + self.reengage_jerk * self.dt / max(gap, MIN_GAP))
    target = min(self.authority * gap, lift_max(v_ego))
    # Normally the lift eases off at LIFT_FALL. When the model starts to brake the gain drops, and
    # the lift may drop with it in one frame, so at gain 0 the model's braking is not delayed. Only the
    # model's own easing off counts here: a flickering throttle probability eases out at LIFT_FALL.
    # A departure window that ran out eases off at LIFT_FALL instead of dropping with the speed fade.
    fall_gain = model_term * (1. if window or self.depart_ease else speed_fade)
    fall = max(LIFT_FALL * self.dt, (1. - fall_gain) * self.lift)
    self.lift = rate_limit(target, self.lift, -fall, lift_rise(v_ego) * self.dt)
    if self.lift == 0.:
      self.depart_ease = False

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
