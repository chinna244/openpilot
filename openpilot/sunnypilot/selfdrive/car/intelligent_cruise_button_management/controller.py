"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from opendbc.sunnypilot.car.icbm_actuation_profile import get_actuation_profile
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

INACTIVE_TIMER = 0.4
# Reaction deadband in display units (mph/kph), applied only while a limiter (SCC/SLA)
# drives the plan: those targets jitter by ±1-2 units frame-to-frame and with no deadband
# the servo saturates its button pacing ping-ponging SET+/SET- around the noise. When the
# plan source is cruise the target IS the driver's setpoint — a stable integer — so the
# servo tracks it exactly (deadband 1): any residual dash error, e.g. from a press the ECU
# dropped, self-heals instead of stranding the dash 1 unit low forever.
REACT_DEADBAND = 2
# The error must persist this long before acting, so a single-frame target glitch
# (e.g. a bad map sample) or a momentary dip can't trigger a button burst.
REACT_TIMER = 0.3
# Moves DOWN act after REACT_TIMER; moves UP wait for the target to hold still this long
# first. Two reasons: transient limiter dips arrive in trains (back-to-back curves), and
# restoring between them both churns the dash and delays the next deceleration on ECUs
# that won't commit to decel while the set speed is moving. It also gives the driver-
# setpoint reconciliation (card) time to adopt the dash after a driver press before the
# servo could chase a stale target — settled-press re-anchoring is race-free because of
# this. Decel-overshoot release is exempt: its slewed rise is the one sanctioned
# continuous up-tracking (measured; the ECU tolerates slow monotonic rises).
RESTORE_QUIET_TIME = 3.0

# Deceleration overshoot: a stock ACC's deceleration scales with the gap between the dash
# set speed and the ACTUAL speed, not the target — commanding dash = target produces almost
# nothing until the car is already several mph over it, so it arrives at curves hot. When the
# planner demands deceleration, command the dash below vEgo by the gap that yields the
# requested decel, capped at the planner target from above (down-only: a stale command
# fail-safes to the car slowing). The command tracks vEgo down through the maneuver and rises
# back to the target on its own as the car converges and aTarget relaxes.
# The mechanism is brand-agnostic; the response curve is not. To enable a brand, measure its
# achieved decel vs (dash - vEgo) gap from logs and add an inverse map entry here.
DECEL_OVERSHOOT_PARAMS = {
  # Mazda CX-5 2022, 422k hands-off cruise samples across 447 rlog segments:
  # ~0.09 m/s^2 per mph of gap, dead below ~2 mph, saturating near -0.75 m/s^2 by ~9 mph
  'mazda': {
    'decel_bp': [0.02, 0.09, 0.26, 0.44, 0.73],  # desired decel magnitude, m/s^2
    'gap_v': [1.5, 2.5, 4.0, 6.0, 8.5],  # required gap below vEgo, mph
    'max_gap': 10.,  # mph; the response saturates, going deeper buys nothing
    'min_decel': 0.15,  # m/s^2; leave gentle coast-downs to the stock behavior
  },
}
# Apply fast (the curve is coming), release slowly so the command doesn't pump between the
# ECU's discrete coast/downshift/brake stages.
DECEL_OVERSHOOT_RISE = 10.  # mph/s
DECEL_OVERSHOOT_RELEASE = 3.  # mph/s
DECEL_OVERSHOOT_SOURCES = (LongitudinalPlanSource.sccVision, LongitudinalPlanSource.sccMap,
                           LongitudinalPlanSource.speedLimitAssist)

# A hold is abandoned (and long-press disabled for the drive) if the dash hasn't moved at
# all this long after the profile says the first step should have landed. Synthesized holds
# interleave with the wheel's genuine button-up frames on the bus, so an ECU may refuse to
# integrate them as a hold — taps are the proven fallback.
HOLD_FIRST_STEP_MARGIN = 0.5  # s

TAP_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}
HOLD_BUTTONS = {
  State.increasing: SendButtonState.increaseHold,
  State.decreasing: SendButtonState.decreaseHold,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.profile = get_actuation_profile(CP.brand)

    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0
    self.restore_quiet_timer = 0
    self.v_target_prev = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.is_metric = False
    self.decel_overshoot_enabled = False
    self.overshoot_mph = 0.0
    self.overshoot_params = DECEL_OVERSHOOT_PARAMS.get(CP.brand)
    self.limiter_active = False

    # Long-press (hold) execution
    self.hold_active = False
    self.hold_frames = 0
    self.hold_start_cluster = 0
    self.longpress_faulted = False  # set for the drive when a synthesized hold doesn't land

    self.frame = 0
    self.cruise_button_timers = dict(CRUISE_BUTTON_TIMER)

  @property
  def react_deadband(self) -> int:
    # Exact tracking against the (stable) driver setpoint; jitter band against limiters.
    # Overshoot keeps the limiter band: its command moves by design.
    return REACT_DEADBAND if self.limiter_active or self.overshoot_mph > 0 else 1

  def update_decel_overshoot(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> float:
    if self.overshoot_params is None:
      return 0.0

    p = self.overshoot_params
    want = 0.0
    if (self.decel_overshoot_enabled and self.is_ready
        and LP_SP.longitudinalPlanSource in DECEL_OVERSHOOT_SOURCES
        and LP_SP.aTarget < -p['min_decel'] and CS.vEgo > LP_SP.vTarget):
      want = min(float(np.interp(-LP_SP.aTarget, p['decel_bp'], p['gap_v'])), p['max_gap'])

    if want > self.overshoot_mph:
      self.overshoot_mph = min(want, self.overshoot_mph + DECEL_OVERSHOOT_RISE * DT_CTRL)
    else:
      self.overshoot_mph = max(want, self.overshoot_mph - DECEL_OVERSHOOT_RELEASE * DT_CTRL)

    return self.overshoot_mph

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

    self.limiter_active = LP_SP.longitudinalPlanSource != LongitudinalPlanSource.cruise

    v_target_ms = LP_SP.vTarget
    overshoot_ms = self.update_decel_overshoot(CS, LP_SP) * CV.MPH_TO_MS
    if overshoot_ms > 0:
      # command relative to actual speed so the ECU sees the gap that produces the requested
      # decel; never above the planner target, and never more than the gap below it
      v_target_ms = min(v_target_ms, max(CS.vEgo, LP_SP.vTarget) - overshoot_ms)

    self.v_target_prev = self.v_target
    self.v_target = round(v_target_ms * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

  def update_restore_quiet_timer(self) -> None:
    # Counts how long an up-error has persisted against a still target; any target motion
    # or the error closing resets it. Bypassed entirely while overshoot is releasing.
    up_error = self.v_target - self.v_cruise_cluster
    if up_error >= self.react_deadband and self.v_target == self.v_target_prev:
      self.restore_quiet_timer += 1
    else:
      self.restore_quiet_timer = 0

  def plan_hold(self) -> None:
    # Start a hold when the remaining error spans at least one ECU snap step; run taps for
    # the remainder. Release is evaluated every frame against the live dash, so a snap that
    # lands mid-grid (the ECU aligns first) just shrinks the remainder.
    remaining = abs(self.v_target - self.v_cruise_cluster)
    use_hold = (self.profile.supports_longpress(self.is_metric) and not self.longpress_faulted
                and remaining >= self.profile.longpress_step)

    if use_hold and not self.hold_active:
      self.hold_active = True
      self.hold_frames = 0
      self.hold_start_cluster = self.v_cruise_cluster
    elif self.hold_active:
      self.hold_frames += 1
      if remaining < self.profile.longpress_step:
        self.hold_active = False
      elif self.v_cruise_cluster == self.hold_start_cluster:
        # the ECU should have snapped by now; if the dash never moved, this ECU does not
        # integrate synthesized holds — fall back to taps for the rest of the drive
        timeout = self.profile.longpress_first_step_s + HOLD_FIRST_STEP_MARGIN
        if self.hold_frames * DT_CTRL > timeout:
          self.longpress_faulted = True
          self.hold_active = False
          cloudlog.event("icbm_longpress_fallback", brand=self.CP.brand)
      else:
        # a step landed; re-arm the watchdog from the new dash value
        self.hold_start_cluster = self.v_cruise_cluster
        self.hold_frames = 0

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)
    self.update_restore_quiet_timer()

    # HOLDING, ACCELERATING, DECELERATING, PRE_ACTIVE
    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_target - self.v_cruise_cluster >= self.react_deadband:
              self.state = State.increasing

            elif self.v_cruise_cluster - self.v_target >= self.react_deadband and self.v_cruise_cluster > self.v_cruise_min:
              self.state = State.decreasing

            else:
              self.state = State.holding

        # HOLDING
        elif self.state == State.holding:
          down_pending = self.v_cruise_cluster - self.v_target >= self.react_deadband
          up_pending = self.v_target - self.v_cruise_cluster >= self.react_deadband
          up_allowed = self.overshoot_mph > 0 or self.restore_quiet_timer >= int(RESTORE_QUIET_TIME / DT_CTRL)
          if down_pending or (up_pending and up_allowed):
            self.pre_active_timer = int(REACT_TIMER / DT_CTRL)
            self.state = State.preActive

        # ACCELERATING
        elif self.state == State.increasing:
          if self.v_target <= self.v_cruise_cluster:
            self.state = State.holding

        # DECELERATING
        elif self.state == State.decreasing:
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    # INACTIVE
    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    if self.state in TAP_BUTTONS:
      self.plan_hold()
      send_button = HOLD_BUTTONS[self.state] if self.hold_active else TAP_BUTTONS[self.state]
    else:
      self.hold_active = False
      send_button = SendButtonState.none

    return send_button

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    self.is_ready = ready and not button_pressed

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool,
          decel_overshoot_enabled: bool = False) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      return

    self.is_metric = is_metric
    self.decel_overshoot_enabled = decel_overshoot_enabled

    self.update_calculations(CS, LP_SP)
    self.update_readiness(CS, CC)

    self.cruise_button = self.update_state_machine()

    self.is_ready_prev = self.is_ready
    self.frame += 1
