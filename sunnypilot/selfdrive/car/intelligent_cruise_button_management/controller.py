"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from cereal import car, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

INACTIVE_TIMER = 0.4
# Reaction deadband, in display units (mph/kph). The planner target (sccVision especially)
# jitters by ±1-2 units frame-to-frame; with no deadband ICBM saturates its 0.2s button pacing
# ping-ponging SET+/SET- around the noise. Don't leave HOLDING until the error is at least
# this large; increasing/decreasing still run to the exact target once started. This is the
# single anti-jitter mechanism: it filters target-vs-cluster error, which subsumes filtering
# the target's own motion (the former apply_hysteresis/HYST_GAP seam).
REACT_DEADBAND = 2
# The error must persist this long before acting, so a single-frame target glitch
# (e.g. a bad map sample) or a momentary dip can't trigger a button burst.
REACT_TIMER = 0.3

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


SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.is_metric = False
    self.decel_overshoot_enabled = False
    self.overshoot_mph = 0.0
    self.overshoot_params = DECEL_OVERSHOOT_PARAMS.get(CP.brand)

    self.cruise_button_timers = dict(CRUISE_BUTTON_TIMER)

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

    v_target_ms = LP_SP.vTarget
    overshoot_ms = self.update_decel_overshoot(CS, LP_SP) * CV.MPH_TO_MS
    if overshoot_ms > 0:
      # command relative to actual speed so the ECU sees the gap that produces the requested
      # decel; never above the planner target, and never more than the gap below it
      v_target_ms = min(v_target_ms, max(CS.vEgo, LP_SP.vTarget) - overshoot_ms)

    self.v_target = round(v_target_ms * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    # HOLDING, ACCELERATING, DECELERATING, PRE_ACTIVE
    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_target - self.v_cruise_cluster >= REACT_DEADBAND:
              self.state = State.increasing

            elif self.v_cruise_cluster - self.v_target >= REACT_DEADBAND and self.v_cruise_cluster > self.v_cruise_min:
              self.state = State.decreasing

            else:
              self.state = State.holding

        # HOLDING
        elif self.state == State.holding:
          if abs(self.v_target - self.v_cruise_cluster) >= REACT_DEADBAND:
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

    send_button = SEND_BUTTONS.get(self.state, SendButtonState.none)

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
