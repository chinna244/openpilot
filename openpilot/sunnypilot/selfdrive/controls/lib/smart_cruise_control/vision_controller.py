"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Smart Cruise Control - Vision: curve speed from a profile over the model path.

Curvature comes from geometry (orientationRate.z / velocity.x), so slowing down does not
lower the prediction and talk the controller out of the slowdown it just started. A
backward pass at the budget the platform can deliver turns the path into a speed profile;
the car holds the set speed until the profile binds, then brakes once, at the budget,
arriving at the curve at the allowed speed. The state machine remains for UI and alerts.

Measurements, design and rejected alternatives: docs/zoompilot/scc-curve-planning.md.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import (
  A_PUB_MIN, COMMIT_FRAC, get_planning_limits, publish_ramp)
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.speed_profile import (
  allowed_speed, backward_pass, lead_distance, min_profile_speed, required_decel)

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState

ACTIVE_STATES = (VisionState.entering, VisionState.turning, VisionState.leaving)
ENABLED_STATES = (VisionState.enabled, VisionState.overriding, *ACTIVE_STATES)

_A_LAT_REG_MAX = 2.  # m/s2; curves are taken at or below this lateral acceleration
# plan to 95% of the ceiling so actuation lag lands the apex on it instead of over it
_PLAN_MARGIN = 0.95

# The model under-reads path curvature with range (a distance effect, not a horizon
# fraction), so the profile binds late. The gain undoes the measured ratio before the
# solve; it returns to 1.0 as the curve closes, so it moves WHEN the car brakes, not how hard.
_KAPPA_BIAS_D = [0., 30., 50., 70., 90., 110.]  # m along the path
# reciprocal of the measured ratio, capped where the replay stops buying apexes
_KAPPA_BIAS_GAIN = [1.0, 1.06, 1.14, 1.22, 1.42, 1.5]
# the table was fitted on a 30-50 mph road; above that band the raw path decides
_KAPPA_BIAS_V_BP = [22.4, 26.8]  # m/s; full correction to 50 mph, none from 60 mph
_KAPPA_BIAS_V_FADE = [1.0, 0.0]

# Above the band the far field is not trusted to plan on either: at highway speed the
# curvature that binds is tiny and the model's range error is a tail of over-reads, so
# only the near window (the part of the path the car can already measure) enters the profile.
_PLAN_HORIZON_V_BP = _KAPPA_BIAS_V_BP
_PLAN_HORIZON_D = [300., 0.]  # m; the near window is the floor

# how much tighter than the near field the raw path ahead has to read before the near
# floor lets go; curvature noise must not look like a corner worth braking past it for
_NEAR_FLOOR_FRAC = 0.98

# release well below COMMIT_FRAC so the gate does not chatter on noise
_RELEASE_FRAC = 0.3

_NEAR_T = 3.0  # s; "the curve is here" window for the in-curve speed hold

_V_FLOOR = 0.5  # m/s; model velocity floor when converting yaw rate to curvature

# display thresholds only; not load-bearing for control
_TURNING_LAT_ACC_TH = 1.6  # current lat acc above this displays as turning
_LEAVING_LAT_ACC_TH = 1.3  # turning displays as leaving below this
_FINISH_LAT_ACC_TH = 1.1  # leaving ends below this


class SmartCruiseControlVision:
  def __init__(self, CP):
    self.params = Params()
    self.limits = get_planning_limits(CP)
    self.frame = -1
    self.long_enabled = False
    self.long_override = False
    self.is_enabled = False
    self.is_active = False
    self.enabled = self.params.get_bool("SmartCruiseControlVision")
    self.v_cruise_setpoint = 0.

    self.state = VisionState.disabled
    self.v_ego = 0.
    self.a_ego = 0.

    # solver
    self.solver_valid = False
    self.solver_active = False
    self.a_required = 0.
    self.v_profile_now = float('inf')
    self.v_dip_ahead = float('inf')
    self.v_near_min = float('inf')
    self.v_raw_min = float('inf')

    # published
    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.
    self.a_out = 0.
    self.current_lat_acc = 0.
    self.max_pred_lat_acc = 0.

  def _reset_solver(self) -> None:
    self.solver_valid = False
    self.solver_active = False
    self.a_required = 0.
    self.v_profile_now = float('inf')
    self.v_dip_ahead = float('inf')
    self.v_near_min = float('inf')
    self.v_raw_min = float('inf')
    self.max_pred_lat_acc = 0.

  def _update_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.enabled = self.params.get_bool("SmartCruiseControlVision")

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    if not self.long_enabled:
      self._reset_solver()
      return

    model = sm['modelV2']
    rate_z = np.abs(np.asarray(model.orientationRate.z, dtype=float))
    vel = np.asarray(model.velocity.x, dtype=float)
    x = np.asarray(model.position.x, dtype=float)
    y = np.asarray(model.position.y, dtype=float)
    if len(rate_z) < 2 or not (len(rate_z) == len(vel) == len(x) == len(y)):
      self._reset_solver()
      return

    self.current_lat_acc = self.v_ego ** 2 * abs(sm['controlsState'].curvature)

    # geometry, not lateral acceleration: kappa is speed-independent
    kappa = rate_z / np.maximum(vel, _V_FLOOR)
    dist = np.empty_like(x)
    dist[0] = 0.
    dist[1:] = np.cumsum(np.hypot(np.diff(x), np.diff(y)))

    lim = self.limits
    # the near field is measured, not predicted, so it keeps raw geometry: the bias
    # correction is never allowed to outvote it
    near_d = max(self.v_ego, MIN_V) * _NEAR_T
    near = dist <= near_d
    # the predicted far field the planner may act on shrinks with speed, down to the near window
    far = dist > max(float(np.interp(self.v_ego, _PLAN_HORIZON_V_BP, _PLAN_HORIZON_D)), near_d)
    v_raw = allowed_speed(kappa, _A_LAT_REG_MAX * _PLAN_MARGIN)
    self.v_near_min = float(np.min(v_raw[near])) if np.any(near) else float('inf')
    self.v_raw_min = float(np.min(v_raw[~far]))
    # UI wire: the lateral acceleration the near path would produce at the current speed
    self.max_pred_lat_acc = float(np.max(kappa[near]) * self.v_ego ** 2) if np.any(near) else 0.

    # everything that decides WHEN to brake plans on the bias-corrected curve
    fade = np.interp(self.v_ego, _KAPPA_BIAS_V_BP, _KAPPA_BIAS_V_FADE)
    kappa = kappa * (1. + (np.interp(dist, _KAPPA_BIAS_D, _KAPPA_BIAS_GAIN) - 1.) * fade)
    v_allowed = allowed_speed(kappa, _A_LAT_REG_MAX * _PLAN_MARGIN)
    v_allowed[far] = np.inf

    # the stock path's lead includes walking the dash down to the deepest dip ahead
    t_lead = lim.t_lead
    if not lim.op_long:
      v_dip = float(np.min(v_allowed))
      if np.isfinite(v_dip):
        t_lead += lim.dash_traversal_time(max(self.v_ego - max(v_dip, MIN_V), 0.))
    d_lead = lead_distance(self.v_ego, t_lead, lim.a_budget, lim.jerk(self.v_ego))

    self.a_required = required_decel(self.v_ego, v_allowed, dist, d_lead)
    v_max = backward_pass(v_allowed, dist, lim.a_budget)
    self.v_profile_now = float(v_max[0])
    self.v_dip_ahead = min_profile_speed(v_max, dist, float(dist[-1]))

    # commit when the required decel approaches the budget; once braking, hold through the
    # curve (the near path stays below the setpoint) and release on real relaxation
    commit = self.a_required >= COMMIT_FRAC * lim.a_budget
    in_curve = np.isfinite(self.v_near_min) and self.v_near_min < self.v_cruise_setpoint
    hold = self.a_required >= _RELEASE_FRAC * lim.a_budget or in_curve
    self.solver_active = commit or (self.solver_active and hold)
    self.solver_valid = True

  def _update_state_machine(self) -> tuple[bool, bool]:
    # ENABLED, ENTERING, TURNING, LEAVING, OVERRIDING
    if self.state != VisionState.disabled:
      # longitudinal and feature disable always have priority in a non-disabled state
      if not self.long_enabled or not self.enabled:
        self.state = VisionState.disabled
      elif self.long_override:
        self.state = VisionState.overriding

      else:
        # ENABLED
        if self.state == VisionState.enabled:
          # do not enter a turn control cycle if the speed is low
          if self.v_ego <= MIN_V:
            pass
          elif self.solver_active:
            self.state = VisionState.entering

        # OVERRIDING
        elif self.state == VisionState.overriding:
          if not self.long_override:
            self.state = VisionState.enabled

        # ENTERING
        elif self.state == VisionState.entering:
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          elif not self.solver_active:
            self.state = VisionState.enabled

        # TURNING
        elif self.state == VisionState.turning:
          if self.current_lat_acc <= _LEAVING_LAT_ACC_TH:
            self.state = VisionState.leaving

        # LEAVING
        elif self.state == VisionState.leaving:
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          elif self.current_lat_acc < _FINISH_LAT_ACC_TH and not self.solver_active:
            self.state = VisionState.enabled

    # DISABLED
    elif self.state == VisionState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = VisionState.overriding
        else:
          self.state = VisionState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  @property
  def _controlling(self) -> bool:
    return self.is_active and self.solver_active

  @property
  def _near_floor(self) -> float:
    """Speed the road the car can already resolve requires, m/s; -inf when it does not bind.

    The correction is a claim about what the model cannot resolve yet, so it must not be the
    sole reason to command below what it can: going under the near requirement is allowed
    only when the raw path genuinely reports something tighter further out. Without this a
    constant-radius curve inflates its own far half and the car settles below the speed the
    road requires for the length of the curve.
    """
    if np.isfinite(self.v_near_min) and self.v_raw_min >= self.v_near_min * _NEAR_FLOOR_FRAC:
      return self.v_near_min
    return -float('inf')

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
      # parity with the idle wire; also the ramp's starting point on activation
      self.a_out = self.a_ego
      return self.a_out

    # near convergence a bumper-distance constraint makes required_decel scream through its
    # distance floor; nothing beyond the unit-gain pull to the lowest profile speed is useful
    a_need = self.a_required
    if np.isfinite(self.v_dip_ahead):
      a_need = min(a_need, max(self.v_ego - self.v_dip_ahead, 0.))
    # keep the two channels consistent: a decel the target floor forbids must not reach
    # the actuator (or pull the dash under the floor through the overshoot lever)
    a_need = min(a_need, max(self.v_ego - self._near_floor, 0.))
    # clipped to the path and jerk-ramped, so a one-frame step never reaches the actuators as a snap
    self.a_out = publish_ramp(-a_need, self.a_out, self.limits, self.v_ego)
    return self.a_out

  def get_v_target_from_control(self) -> float:
    if not self._controlling:
      return V_CRUISE_UNSET

    # openpilot long's cruise candidate is a unit-gain P controller on (v_cruise - v_ego),
    # so commanding decel means leading v_ego by the decel required; the profile value
    # caps it so the car settles at the allowed speed inside the curve
    v_lead = self.v_ego + max(-self.a_required, A_PUB_MIN)
    v = min(self.v_profile_now, v_lead)
    if np.isfinite(self.v_dip_ahead):
      if self.limits.op_long:
        # never command below the slowest point of the plan: past that the P candidate is
        # already railed at the budget, and lower targets only distort the wire
        v = max(v, self.v_dip_ahead)
      else:
        # a dash servo cannot track a continuous profile in 1 mph taps; pre-position it
        # at the deepest dip on the horizon and let the decel gap do the shaping
        v = min(v, self.v_dip_ahead)
    return max(v, self._near_floor, MIN_V)

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float,
             v_cruise_setpoint: float) -> None:
    self.long_enabled = long_enabled
    self.long_override = long_override
    self.v_ego = v_ego
    self.a_ego = a_ego
    self.v_cruise_setpoint = v_cruise_setpoint

    self._update_params()
    self._update_calculations(sm)

    self.is_enabled, self.is_active = self._update_state_machine()

    self.output_a_target = self.get_a_target_from_control()
    self.output_v_target = self.get_v_target_from_control()

    self.frame += 1
