"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Vision-based curve speed planning over the model path.

Geometry-derived curvature remains independent of planned speed. A backward pass applies
the platform deceleration budget to produce the speed profile. The update loop and params
refresh are sunnypilot's; the solver and state machine are ours. See
docs/zoompilot/scc-curve-planning.md for tuning data and design details.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.starpilot.longitudinal.scc import MIN_V
from openpilot.starpilot.longitudinal.scc.limits import (
  A_PUB_MIN, COMMIT_FRAC, get_planning_limits, publish_ramp)
from openpilot.starpilot.longitudinal.scc.speed_profile import (
  allowed_speed, backward_pass, lead_distance, min_profile_speed, required_decel)
from openpilot.starpilot.longitudinal.scc.vision_controller import (
  ACTIVE_STATES, ENABLED_STATES, VisionState, SmartCruiseControlVision as UpstreamVision)
from openpilot.starpilot.longitudinal.scc.zoompilot.curve_speed_profile import VisionProfile

__all__ = ['ACTIVE_STATES', 'ENABLED_STATES', 'SmartCruiseControlVision', 'VisionState']

# Curves are taken at or below this lateral acceleration (v^2 * curvature), m/s2, keyed on the
# curve's own speed, not the set speed. Tolerance falls with speed, but banked highway curves
# (2.5-4 deg) hide 0.4-0.65 m/s2, so the ceiling dips mid-range and recovers (doc).
_A_LAT_CURVE_BP = [15.6, 24.6, 31.3]  # m/s curve speed; 35, 55, 70 mph
# Local tuning (zoompilot 2.2, 2.1, 2.2): this driver's own turns peak at a median 2.4 m/s2 at 15-24 mph (31 turns,
# routes 0000014e-00000163), and the model's own plan took a 210 m bend at about 2.6 (route 00000160 830 s) where
# 2.0 held the car 7 mph under it.
# This driver's own curves (gyro yaw rate x speed, routes 00000160-0000018f, 107 curves): taken without slowing, apex
# 2.5 to 3.1 m/s2 median and 3.6 to 3.8 p90 at 20-40 mph, 1.6 to 1.7 median and 2.0 to 2.4 p90 from 50 mph, and up to
# 2.9 at p90 where curve control slowed the car and the driver pressed the gas. Replayed on 41 drives (closed loop):
# town curves braked 47 -> 38 (101 -> 73 s), highway 15 -> 12 (50 -> 32 s). The town value stops at 2.8: with the
# actuator lag a hairpin entered from a stop peaks 0.9 over it (3.8 at r = 15 m, about this driver's own p90), and 3.5
# would put loops at 4.0-4.5.
_A_LAT_CURVE = [2.8, 2.8, 2.6]
# Reserve margin for actuation delay at the apex.
_PLAN_MARGIN = 0.95
# The same table by curvature (ascending), so each point's ceiling needs no speed solve.
_A_LAT_KAPPA_BP = [a * _PLAN_MARGIN / v ** 2 for v, a in zip(_A_LAT_CURVE_BP[::-1], _A_LAT_CURVE[::-1], strict=True)]
_A_LAT_KAPPA = [a * _PLAN_MARGIN for a in _A_LAT_CURVE[::-1]]

# Bank. What the car feels is v^2 * kappa - g * roll (signed, left positive; against the IMU on
# the 10-12/13 drives, 0.97 and 0.91). The table above was tuned on curves carrying the typical
# bank for their speed (median g * roll over sustained curves, side-signed): +0.48 m/s2 from
# 35 mph (designed roads, hardly ever adverse), +0.3 at 25-35, about zero at intersection speed.
# Only a departure from that moves a point's ceiling, so a typical curve plans as it did, a flat
# or adverse one slows and a steep one is let through. Roll along the path is the road roll now
# plus the model's predicted change (modelV2.orientation.x, which tracks the realised change
# with slope 1.0, corr 0.86 at 2-4 s; doc).
_BANK_TYPICAL_BP = [7.8, 12.3, 16.8]  # m/s curve speed; 17.5, 27.5, 37.5 mph
_BANK_TYPICAL = [0.05, 0.3, 0.48]  # m/s2
_BANK_SHIFT_MIN, _BANK_SHIFT_MAX = -1.0, 0.4  # m/s2, before the planning margin
_BANK_FLOOR = 0.5  # a shifted ceiling keeps at least this share of the table's
_ROLL_MAX = 0.14  # rad, 8 deg, paramsd's lowered validity bound

# Compensate the measured distance-dependent curvature bias before solving. The gain returns
# to one in the near field, changing brake timing without changing apex speed.
_KAPPA_BIAS_D = [0., 30., 50., 70., 90., 110.]  # m along the path
# Reciprocal of the measured ratio, capped at the validated range.
_KAPPA_BIAS_GAIN = [1.0, 1.06, 1.14, 1.22, 1.42, 1.5]
# Fade the correction above its 30-50 mph fit range.
_KAPPA_BIAS_V_BP = [22.4, 26.8]  # m/s; full correction to 50 mph, none from 60 mph
_KAPPA_BIAS_V_FADE = [1.0, 0.0]
# Both models share the table: the big one under-reads less at range, and its own fit by
# seconds ahead entered curves hotter (doc).

# On stock ACC the profile minimum is the dash target, so the gain sets depth as well as timing.
# Braking for a highway bend drops vEgo under the fade band and restores the full gain, which
# overshot the target (doc). From a 55 mph set speed the profile uses the raw read.
_KAPPA_DEPTH_SET_BP = [20.1, 24.6]  # m/s set speed; 45 -> 55 mph

# Below the bias band the far read also flickers: a bend seen at 150 m can vanish from the
# path at 100 m and come back at 50 m, and the planner released 100 m short of the apex (the
# old servo only arrived slow because its gap decayed slowly after release). On stock ACC a
# committed bend is therefore held, with its speed and remaining distance, until the near
# window reaches where it was seen. A phantom binds for a frame or two, a real bend for
# seconds before the read drops it, so the latch waits 2 s; the route-sim sweep is in the doc.
_HOLD_V_MAX = _KAPPA_BIAS_V_BP[0]  # m/s
_HOLD_SEEN_T = 2.0  # s

# Only the measured near field may ask past the budget, and only where arriving hot costs
# more than braking hard: cruising below 50 mph. From a 60 mph set speed the ECU's range past
# the budget is the braking that was reported as too hard, for one hot curve in 13 (route
# sim). Keyed on the set speed so the ceiling does not creep back up as the car slows.
_ESCALATION_V_BP = _KAPPA_BIAS_V_BP

# Use hysteresis below the commit threshold.
_RELEASE_FRAC = 0.3
# The car is still in a curve while the near field limits it below vEgo plus this; past it the
# plan hands back and the ICBM restore stays capped at sccVAheadMin.
_IN_CURVE_MARGIN = 1.0  # m/s

# The near field is measured rather than predicted: it holds the planner through a curve and is
# the only part of the path allowed to ask past the budget, so what bounds it is the model
# flagging a bend that is not there. The small model's read holds to 3 s
# and a 4 s window quadruples its past-budget phantoms above 50 mph; the big model (modelV2.big)
# reads and flags at 3-4 s as the small one does at 2-3 s (model_reach.py; numbers in the doc).
_NEAR_T = 3.0  # s
_NEAR_T_BIG = 4.0  # s

_V_FLOOR = 0.5  # m/s; model velocity floor when converting yaw rate to curvature
# Where the plan has all but stopped and the car has not, yaw-rate noise over that speed reads as a sharp bend
# (route 00000163 909 s: 0.023 rad/s at a planned 8 mph, a 156 m "curve" on a straight approach to a stop). Those
# points carry no curvature; a plan that slow is already under any bend that matters.
_KAPPA_V_MIN = 4.5  # m/s
_KAPPA_V_FRAC = 0.5  # of vEgo

# Floor turns. Pulling away from a signal on openpilot long, the car crosses MIN_V mid-turn in a
# corner tighter than the floor allows, and a cycle started there can only hold MIN_V: the pull-away
# pauses at the exit (routes 0000014b, 00000151-153). A corner the car is already turning through
# near the floor is left to the plan until the solver releases it, unless the car would reach
# _FLOOR_TURN_LAT_CAP on the curvature past the actuation lead _FLOOR_TURN_PROJ_T on (the turn goes
# on: loops, hairpins, a tightening corner), or gets more than the in-curve margin over the curve.
_FLOOR_TURN_LAT_ACC = 1.6  # m/s2, already turning
_FLOOR_TURN_LAT_CAP = 2.8  # m/s2; at this cap synthetic R15/R20 loops from a stop peaked as before under the 2.4 ceiling
_FLOOR_TURN_PROJ_T = 1.0  # s

# In-turn braking (openpilot long, town speed). Through an intersection corner the near field still reads the bend
# the car is in, and the solver held -1.2 until the exit, where this driver was already back on the gas (13 of 22
# episodes, routes 00000166-0000016e). Once the car is turning, braking stops at _IN_TURN_A_MIN unless the turn past
# the actuation lead would put the car over _IN_TURN_ESCAPE_LAT. Closed-loop replay through the host's speed filter,
# cruise candidate and actuator lag over 35 episodes (28 drives): town in-turn braking under -0.55 12.7 s -> 4.3 s,
# +0.1 mph mean, worst corner 4.26 m/s2 as without the cap (route 00000167 318 s, a turn the escape hands back).
_IN_TURN_V_MAX = 15.6  # m/s, 35 mph
_IN_TURN_LAT_ACC = 1.6  # m/s2, current lateral acceleration that counts as turning
_IN_TURN_A_MIN = -0.5  # m/s2
_IN_TURN_ESCAPE_LAT = 3.5  # m/s2, lateral acceleration past the actuation lead that restores full braking

# Turn exit (openpilot long, town speed). Once the corner opens, the near field still holds the bend the car is leaving
# and the profile held ~0 or braked while the wheel unwound and the model asked to pull away: route 00000166 t=395 braked
# -0.3 at 14.5 mph with the model at +1.44 and the lateral acceleration falling 2.9 -> 2.5. The turn is handed back
# to the plan once the car has turned, the lateral acceleration has fallen _EXIT_DROP below the turn's peak, the turn
# past the actuation lead stays under _EXIT_LAT_AHEAD, and the model itself asks for more than _EXIT_MODEL_A.
# Full-planner replay of 160 op turns (75 drives): 23 changed, +0.6 mph median out of the turn, no turn's lateral
# acceleration over 2.8, no frame with less acceleration, 5 short changes elsewhere (curve exits, model +1.1..1.3).
# The wheel must be unwinding (curvature _EXIT_UNWIND below its peak), not just the car slower in a steady curve; no
# bend ahead may need committed braking (a second corner past the near window); once handed back it stays so until
# the solver lets go, a bend ahead tightens past _EXIT_LAT_AHEAD or needs committed braking, or the model brakes.
_EXIT_DROP = 0.3  # m/s2 below the turn's peak lateral acceleration
_EXIT_UNWIND = 0.9  # of the turn's peak curvature
_EXIT_LAT_AHEAD = 2.8  # m/s2
_EXIT_MODEL_A = 0.1  # m/s2

# UI state thresholds do not affect control output.
_TURNING_LAT_ACC_TH = 1.6  # current lat acc above this displays as turning
_LEAVING_LAT_ACC_TH = 1.3  # turning displays as leaving below this
_FINISH_LAT_ACC_TH = 1.1  # leaving ends below this


def curve_table(scale: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
  """The ceiling by curvature at a Curve Speed profile's scale: the ceiling and its curvature keys
  both scale."""
  return np.array(_A_LAT_KAPPA_BP) * scale, np.array(_A_LAT_KAPPA) * scale


_NORMAL_TABLE = curve_table()


def curve_speed(kappa, table: tuple[np.ndarray, np.ndarray] = _NORMAL_TABLE, shift=0.) -> np.ndarray:
  """Allowed speed at each path point under the curve-speed ceiling, planning margin included,
  with each point's ceiling moved by shift (bank_shift)."""
  a_lat = np.interp(kappa, *table)
  return allowed_speed(kappa, np.maximum(a_lat + shift, _BANK_FLOOR * a_lat))


def bank_shift(kappa, side, roll, table: tuple[np.ndarray, np.ndarray] = _NORMAL_TABLE) -> np.ndarray:
  """Ceiling shift at each path point, m/s2 with the planning margin: how far the bank carries
  more (+) or less (-) of the lateral load than the table's typical curve at that speed.
  side is the sign of the curvature (left +; 0 without a roll reading), roll the road roll there, rad."""
  side = np.asarray(side, dtype=float)
  credit = ACCELERATION_DUE_TO_GRAVITY * np.clip(roll, -_ROLL_MAX, _ROLL_MAX) * side
  typical = np.interp(curve_speed(kappa, table), _BANK_TYPICAL_BP, _BANK_TYPICAL)
  # side 0 is no bank reading (or a straight point): no shift
  return np.where(side != 0., np.clip(credit - typical, _BANK_SHIFT_MIN, _BANK_SHIFT_MAX) * _PLAN_MARGIN, 0.)


def path_roll(road_roll: float, model) -> np.ndarray | float:
  """Road roll along the model path, rad: now, plus the change the model predicts."""
  predicted = np.asarray(model.orientation.x, dtype=float)
  n = len(model.orientationRate.z)
  return road_roll + predicted if len(predicted) == n and np.all(np.isfinite(predicted)) else road_roll


class SmartCruiseControlVision(VisionProfile, UpstreamVision):
  def __init__(self, CP):
    super().__init__()
    self.limits = get_planning_limits(CP)
    self._reset_solver()
    self.a_out = 0.
    self.v_ego_car = 0.

  def _set_scale(self, scale: float) -> None:
    super()._set_scale(scale)
    self._curve_table = curve_table(scale)

  def _reset_solver(self) -> None:
    self.solver_valid = False
    self.solver_active = False
    self.a_required = 0.
    self.a_needed = 0.
    self.v_profile_now = float('inf')
    self.v_dip_ahead = float('inf')
    self.v_near_min = float('inf')
    self.v_dip_held = float('inf')  # committed bend below the bias band: its speed
    self.d_held = 0.  # and its remaining distance
    self.seen_frames = 0  # consecutive frames a bend has bound
    self.max_pred_lat_acc = 0.
    self.lat_ahead = 0.  # lateral acceleration past the actuation lead, _FLOOR_TURN_PROJ_T on
    self.floor_turn = False  # the curve is a floor turn left to the plan
    self.turn_peak_lat = 0.  # highest lateral acceleration of this cycle
    self.turn_peak_curv = 0.  # and highest curvature
    self.current_curv = 0.
    self.turn_exit = False  # the turn is opening and the model pulls away: handed back to the plan
    self.a_required_far = 0.
    self.model_accel = 0.

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    if not self.long_enabled or not self.enabled:
      self._reset_solver()
      return

    model = sm['modelV2']
    rate_z_signed = np.asarray(model.orientationRate.z, dtype=float)
    rate_z = np.abs(rate_z_signed)
    vel = np.asarray(model.velocity.x, dtype=float)
    x = np.asarray(model.position.x, dtype=float)
    y = np.asarray(model.position.y, dtype=float)
    if len(rate_z) < 2 or not (len(rate_z) == len(vel) == len(x) == len(y)):
      self._reset_solver()
      return

    self.current_curv = abs(sm['controlsState'].curvature)
    self.current_lat_acc = self.v_ego ** 2 * self.current_curv
    self.model_accel = model.action.desiredAcceleration
    # the planner hands this controller its filtered speed but builds the cruise candidate on carState.vEgo
    self.v_ego_car = sm['carState'].vEgo

    # Derive speed-independent curvature from model geometry.
    kappa = rate_z / np.maximum(vel, _V_FLOOR)
    kappa = np.where((vel < _KAPPA_V_MIN) & (vel < _KAPPA_V_FRAC * self.v_ego), 0., kappa)
    dist = np.empty_like(x)
    dist[0] = 0.
    dist[1:] = np.cumsum(np.hypot(np.diff(x), np.diff(y)))

    lim = self.limits
    table = self._curve_table
    # Without a valid road roll (paramsd still converging) every point plans on the typical bank.
    vp = sm['vehicleParameters']
    side = np.sign(rate_z_signed) if vp.valid else np.zeros_like(rate_z_signed)
    roll = path_roll(vp.roll, model)
    # The raw near field says whether the car is in a curve.
    near_d = max(self.v_ego, MIN_V) * (_NEAR_T_BIG if model.big else _NEAR_T)
    near = dist <= near_d  # dist[0] is 0, so never empty
    k_near = float(np.max(kappa[near]))
    self.v_near_min = float(np.min(curve_speed(kappa, table, bank_shift(kappa, side, roll, table))[near]))
    # Publish near-path lateral acceleration for UI state.
    self.max_pred_lat_acc = k_near * self.v_ego ** 2

    # Corrected curvature times the brake, and sets the profile below a 45-55 mph set speed.
    fade = np.interp(self.v_ego, _KAPPA_BIAS_V_BP, _KAPPA_BIAS_V_FADE)
    depth_fade = min(fade, np.interp(self.v_cruise_setpoint, _KAPPA_DEPTH_SET_BP, [1., 0.]))
    bias = np.interp(dist, _KAPPA_BIAS_D, _KAPPA_BIAS_GAIN) - 1.
    k_allowed = kappa * (1. + bias * fade)
    k_profile = kappa * (1. + bias * depth_fade)
    v_allowed = curve_speed(k_allowed, table, bank_shift(k_allowed, side, roll, table))
    v_profile = curve_speed(k_profile, table, bank_shift(k_profile, side, roll, table))

    # Stock ACC lead time includes the set-speed traversal to the lowest target.
    t_lead = lim.t_lead
    if not lim.op_long:
      v_dip = float(np.min(v_allowed))
      if np.isfinite(v_dip):
        t_lead += lim.dash_traversal_time(max(self.v_ego - max(v_dip, MIN_V), 0.), self.is_metric)
    d_lead = lead_distance(self.v_ego, t_lead, lim.a_budget, lim.jerk(self.v_ego))
    # kappa is a magnitude, so left and right turns read alike
    past_lead = near & (dist >= d_lead)
    v_soon = self.v_ego + max(self.a_ego, 0.) * _FLOOR_TURN_PROJ_T
    self.lat_ahead = v_soon ** 2 * float(np.max(kappa[past_lead])) if np.any(past_lead) else 0.

    self.a_required = required_decel(self.v_ego, v_allowed, dist, d_lead)
    # bends past the near window only: the turn the car is in or leaving does not count (turn exit)
    self.a_required_far = required_decel(self.v_ego, np.where(near, np.inf, v_allowed), dist, d_lead)
    # What the car must still shed from here. The commit gate budgets the stock dash walk in
    # d_lead, but once committed the servo is already walking, so only the ECU's response is
    # still ahead. Past the budget it means the car is late, which only measured geometry may
    # claim: a predicted constraint beyond the near window asks for the budget at most.
    d_resp = lead_distance(self.v_ego, lim.t_lead)
    if lim.op_long:
      self.a_needed = self.a_required
    else:
      a_near = required_decel(self.v_ego, np.where(near, v_allowed, np.inf), dist, d_resp)
      a_far = required_decel(self.v_ego, np.where(near, np.inf, v_allowed), dist, d_resp)
      a_ceiling = float(np.interp(self.v_cruise_setpoint, _ESCALATION_V_BP, [a_near, lim.a_budget]))
      self.a_needed = max(min(a_near, a_ceiling), min(a_far, lim.a_budget))
    v_max = backward_pass(v_profile, dist, lim.a_budget)
    self.v_profile_now = float(v_max[0])
    self.v_dip_ahead = min_profile_speed(v_max, dist, float(dist[-1]))

    # A held bend keeps its speed and distance on the plan until the near window reaches it.
    self.d_held -= self.v_ego * DT_MDL
    held = np.isfinite(self.v_dip_held) and self.d_held > near_d
    if not held:
      self.v_dip_held = float('inf')
    else:
      # a prediction like the far field: it asks for the budget at most
      a_held = required_decel(self.v_ego, [self.v_dip_held], [self.d_held], d_resp)
      self.a_needed = max(self.a_needed, min(a_held, lim.a_budget))
      self.v_dip_ahead = min(self.v_dip_ahead, self.v_dip_held)

    # Commit near the platform budget and retain control through the curve with hysteresis.
    commit = self.a_required >= COMMIT_FRAC * lim.a_budget
    seen = self.a_required >= _RELEASE_FRAC * lim.a_budget
    in_curve = np.isfinite(self.v_near_min) and self.v_near_min < self.v_ego + _IN_CURVE_MARGIN
    self.solver_active = commit or (self.solver_active and (seen or in_curve or held))
    self.solver_valid = True

    # Latch the bend that binds, or a deeper one, while committed below the bias band, once
    # it has bound long enough not to be a single-frame flicker.
    self.seen_frames = self.seen_frames + 1 if seen else 0
    i_dip = int(np.argmin(v_max))
    if lim.op_long or not self.solver_active or self.v_ego >= _HOLD_V_MAX:
      self.v_dip_held = float('inf')
    elif self.seen_frames * DT_MDL >= _HOLD_SEEN_T and v_max[i_dip] <= self.v_dip_held:
      self.v_dip_held = float(v_max[i_dip])
      self.d_held = float(dist[i_dip])

  def _update_floor_turn(self) -> None:
    band = MIN_V + _IN_CURVE_MARGIN
    escape = self.v_ego > max(self.v_near_min, MIN_V) + _IN_CURVE_MARGIN or self.lat_ahead >= _FLOOR_TURN_LAT_CAP
    if not self.solver_active or escape:
      self.floor_turn = False
    elif (self.limits.op_long and self.v_dip_ahead <= band and self.v_ego <= band and
          self.current_lat_acc >= _FLOOR_TURN_LAT_ACC):
      self.floor_turn = True

  def _update_state_machine(self) -> tuple[bool, bool]:
    self._update_floor_turn()
    if self.state != VisionState.disabled:
      # Longitudinal and feature disable take priority over active states.
      if not self.long_enabled or not self.enabled:
        self.state = VisionState.disabled
      elif self.long_override:
        self.state = VisionState.overriding

      else:
        if self.state == VisionState.enabled:
          # Do not start a turn-control cycle below the operating speed.
          if self.v_ego <= MIN_V:
            pass
          elif self.solver_active and not self.floor_turn:
            self.state = VisionState.entering

        elif self.state == VisionState.overriding:
          if not self.long_override:
            self.state = VisionState.enabled

        elif self.state == VisionState.entering:
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          elif not self.solver_active:
            self.state = VisionState.enabled

        elif self.state == VisionState.turning:
          if self.current_lat_acc <= _LEAVING_LAT_ACC_TH:
            self.state = VisionState.leaving

        elif self.state == VisionState.leaving:
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          elif self.current_lat_acc < _FINISH_LAT_ACC_TH and not self.solver_active:
            self.state = VisionState.enabled

    elif self.state == VisionState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = VisionState.overriding
        else:
          self.state = VisionState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES
    self._update_turn_exit()

    return enabled, active

  def _update_turn_exit(self) -> None:
    if not self.solver_active:
      self.turn_peak_lat, self.turn_peak_curv, self.turn_exit = 0., 0., False
      return
    self.turn_peak_lat = max(self.turn_peak_lat, self.current_lat_acc)
    self.turn_peak_curv = max(self.turn_peak_curv, self.current_curv)
    committed = self.a_required_far >= COMMIT_FRAC * self.limits.a_budget  # a further bend that needs braking
    keep = (self.limits.op_long and self.v_ego < _IN_TURN_V_MAX and self.lat_ahead < _EXIT_LAT_AHEAD and not committed)
    if self.turn_exit:
      self.turn_exit = keep and self.model_accel >= 0.
    else:
      opening = (self.turn_peak_lat >= _IN_TURN_LAT_ACC and self.current_lat_acc <= self.turn_peak_lat - _EXIT_DROP and
                 self.current_curv <= _EXIT_UNWIND * self.turn_peak_curv)
      self.turn_exit = keep and opening and self.model_accel > _EXIT_MODEL_A

  @property
  def _controlling(self) -> bool:
    return self.is_active and self.solver_active and not self.turn_exit

  @property
  def v_ahead_min(self) -> float:
    """Lowest planned speed on the horizon for the ICBM restore gate, m/s.

    0 means no lookahead (feature off or no fresh profile) and the servo falls back to
    its stillness heuristic; a clear road caps at 255 (V_CRUISE_UNSET convention).
    """
    if not (self.enabled and self.solver_valid):
      return 0.
    return float(min(self.v_dip_ahead, 255.))

  def get_a_target_from_control(self) -> float:
    if not self._controlling:
      # Seed the activation ramp from the idle acceleration wire.
      self.a_out = self.a_ego
      return self.a_out

    # Bound near-distance required deceleration by the speed error to the profile minimum.
    a_need = self.a_needed
    if np.isfinite(self.v_dip_ahead):
      a_need = min(a_need, max(self.v_ego - self.v_dip_ahead, 0.))
    # Measured geometry may use the ECU's range past the budget; jerk-limit the published command.
    self.a_out = publish_ramp(-a_need, self.a_out, self.limits, self.v_ego, a_floor=A_PUB_MIN)
    if self._in_turn_capped:
      self.a_out = max(self.a_out, _IN_TURN_A_MIN)
    return self.a_out

  @property
  def _in_turn_capped(self) -> bool:
    return (self.limits.op_long and self.v_ego < _IN_TURN_V_MAX and self.current_lat_acc >= _IN_TURN_LAT_ACC and
            self.lat_ahead < _IN_TURN_ESCAPE_LAT)

  def get_v_target_from_control(self) -> float:
    if not self._controlling:
      return V_CRUISE_UNSET

    v_lead = self.v_ego + max(-self.a_required, A_PUB_MIN)
    if self.limits.op_long:
      # Lead vEgo by the required decel for the unit-gain cruise candidate, within the profile,
      # and not below the plan minimum once the P candidate reaches its budget.
      v = min(self.v_profile_now, v_lead)
      if np.isfinite(self.v_dip_ahead):
        v = max(v, self.v_dip_ahead)
      if self._in_turn_capped:
        # the unit-gain cruise candidate brakes no harder than the in-turn cap either, on the measured speed it
        # is built from as well as the filtered speed the MPC starts from
        v = max(v, max(self.v_ego, self.v_ego_car) + _IN_TURN_A_MIN)
    else:
      # Pre-position the discrete stock-ACC setpoint at the horizon minimum, and lead vEgo down
      # only while braking: with nothing to brake for the dash follows the profile up.
      v = min(self.v_profile_now, self.v_dip_ahead)
      if self.a_required > 0.:
        v = min(v, v_lead)
    return max(v, MIN_V)

  def _update_solution(self) -> float:
    # the decel comes from the speed profile (get_a_target_from_control), not the state
    return 0.
