"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Intelligent Cruise Button Management: a servo that walks a stock ACC's dash set speed
onto the plan target with synthesized cruise button presses (non-pcmCruiseSpeed cars).
Measurements behind the constants and the rejected alternatives: docs/zoompilot/icbm.md.
"""
import numpy as np

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from opendbc.sunnypilot.car.icbm_actuation_profile import get_actuation_profile
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers

ButtonType = car.CarState.ButtonEvent.Type
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

INACTIVE_TIMER = 0.4
# selfdrived refreshes its toggles from a 0.1 s params thread; the servo's own read keeps that cadence
PARAMS_UPDATE_FRAMES = int(0.1 / DT_CTRL)
# after a genuine driver press the servo yields in the opposing direction: SET+ parks
# down-moves, SET- parks up-moves, and a press the other way cancels the other grace
DRIVER_PRESS_GRACE_T = 3.0
DRIVER_PRESS_GRACE_FRAMES = int(DRIVER_PRESS_GRACE_T / DT_CTRL)
# display units; applied only while a limiter drives the plan, whose targets jitter 1-2
# units frame to frame. A cruise-source target is the driver setpoint and is tracked exactly.
REACT_DEADBAND = 2
# an error must persist this long before acting, so a one-frame target glitch cannot
# trigger a button burst
REACT_TIMER = 0.3
# up-moves on decel_needs_stable_setpoint cars wait for the plan target to hold still this
# long: limiter dips arrive in trains, and the ECU will not decel while the set speed moves
RESTORE_QUIET_TIME = 1.0
RESTORE_QUIET_FRAMES = int(RESTORE_QUIET_TIME / DT_CTRL)

# Deceleration overshoot: a stock ACC's decel scales with the gap between the dash set
# speed and the ACTUAL speed, so when the planner asks for decel the dash is commanded below
# vEgo by the gap that yields it (down-only; capped at the target from above). The response
# curve is per brand: the inverse map is measured from logs before a brand is enabled.
DECEL_OVERSHOOT_PARAMS = {
  'mazda': {
    'decel_bp': [0.02, 0.09, 0.26, 0.44, 0.73],  # desired decel magnitude, m/s^2
    # gap below vEgo, mph; leads the steady-state inverse to pay back the dash walk
    'gap_v': [2.0, 4.0, 6.0, 8.5, 10.0],
    'max_gap': 10.,  # mph; the response saturates, going deeper buys nothing
    'min_decel': 0.15,  # m/s^2; leave gentle coast-downs to the stock behavior
  },
}
# apply fast, release slowly so the command does not pump between the ECU's decel stages
DECEL_OVERSHOOT_RISE = 10.  # mph/s
DECEL_OVERSHOOT_RELEASE = 3.  # mph/s
DECEL_OVERSHOOT_SOURCES = (LongitudinalPlanSource.sccVision, LongitudinalPlanSource.sccMap,
                           LongitudinalPlanSource.speedLimitAssist)

# The 10 Hz hold stream registers on the ECU as paced 1-unit presses, never as a held
# button, but it is still the fastest walk available; taps take the small remainder where
# the stream's in-flight frames would overshoot.
FAST_MODE_MIN = 3  # display units of remaining error to run the stream
FAST_STALL_T = 1.5  # s; a dash that never moves under the stream means taps for the rest of the drive

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
    self.params = Params()
    self.profile = get_actuation_profile(CP.brand)
    self.frame = 0

    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0
    self.restore_quiet_timer = 0
    self.v_target_prev = 0
    self.v_target_raw = 0
    self.v_target_raw_prev = 0
    self.react_deadband = REACT_DEADBAND
    self.lookahead_valid = False
    self.dip_ahead = False
    self.down_grace_timer = 0
    self.up_grace_timer = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.is_metric = False
    # a pending SLA confirm prompt parks the servo. The state is read off the plan the
    # servo tracks (plannerd mirrors the card session into longitudinalPlanSP), so the
    # freeze and the frozen target arrive together; card vetoes emission with same-frame
    # session state, since this view is two message hops stale
    self.prompt_frozen = False
    self.decel_overshoot_enabled = self.params.get_bool("SmartCruiseDecelOvershoot")
    self.overshoot_mph = 0.0
    self.overshoot_params = DECEL_OVERSHOOT_PARAMS.get(CP.brand)
    self.limiter_active = False

    # fast-walk stream execution
    self.fast_active = False
    self.fast_stall_frames = 0
    self.fast_last_cluster = 0
    self.fast_faulted = False  # set for the drive when the stream never moves the dash

    self.cruise_button_timers = dict(CRUISE_BUTTON_TIMER)

  def update_decel_overshoot(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> float:
    if self.overshoot_params is None:
      return 0.0

    p = self.overshoot_params
    want = 0.0
    # never integrate a command the servo cannot emit (driver press, confirm prompt, SET+
    # grace): winding up behind a block only banks a stale gap to dump when it lifts
    if (self.decel_overshoot_enabled and self.is_ready and not self.prompt_frozen
        and self.down_grace_timer <= 0
        and LP_SP.longitudinalPlanSource in DECEL_OVERSHOOT_SOURCES
        and LP_SP.aTarget < -p['min_decel'] and CS.vEgo > LP_SP.vTarget):
      want = min(float(np.interp(-LP_SP.aTarget, p['decel_bp'], p['gap_v'])), p['max_gap'])

    if want > self.overshoot_mph:
      self.overshoot_mph = min(want, self.overshoot_mph + DECEL_OVERSHOOT_RISE * DT_CTRL)
    else:
      # release gently only while the limiter is live; back on cruise the residual only
      # holds the dash down and stalls the restore, so drop it at the build rate
      release = DECEL_OVERSHOOT_RELEASE if self.limiter_active else DECEL_OVERSHOOT_RISE
      self.overshoot_mph = max(want, self.overshoot_mph - release * DT_CTRL)

    return self.overshoot_mph

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

    self.limiter_active = LP_SP.longitudinalPlanSource != LongitudinalPlanSource.cruise

    v_target_ms = LP_SP.vTarget
    overshoot_ms = self.update_decel_overshoot(CS, LP_SP) * CV.MPH_TO_MS
    if overshoot_ms > 0:
      # command relative to actual speed so the ECU sees the gap; never above the planner
      # target, never more than the gap below it
      v_target_ms = min(v_target_ms, max(CS.vEgo, LP_SP.vTarget) - overshoot_ms)

    self.v_target_prev = self.v_target
    self.v_target = round(v_target_ms * speed_conv)
    # the plan's own target before the overshoot lever: restore intent is judged against
    # this, since the lever's decay is self-inflicted motion
    self.v_target_raw_prev = self.v_target_raw
    self.v_target_raw = round(LP_SP.vTarget * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)

    # exact tracking against the driver setpoint; jitter band against limiters and the
    # overshoot command, which moves by design
    self.react_deadband = REACT_DEADBAND if self.limiter_active or self.overshoot_mph > 0 else 1

    # vision lookahead for the restore gate; 0 means no lookahead and the servo falls
    # back to the stillness heuristic
    v_ahead_min = LP_SP.smartCruiseControl.vision.vAheadMin
    self.lookahead_valid = v_ahead_min > 0.
    self.dip_ahead = self.lookahead_valid and v_ahead_min * speed_conv < self.v_target_raw - self.react_deadband

  def update_restore_quiet_timer(self) -> None:
    # how long an up-error has persisted against a still plan target. Keyed on the raw
    # target so the overshoot lever's release does not read as plan motion; held at zero
    # through a prompt so a decline still waits out a full quiet window.
    up_error = self.v_target_raw - self.v_cruise_cluster
    if self.prompt_frozen:
      self.restore_quiet_timer = 0
    elif up_error >= self.react_deadband and self.v_target_raw == self.v_target_raw_prev:
      self.restore_quiet_timer += 1
    else:
      self.restore_quiet_timer = 0

  def plan_fast_mode(self) -> None:
    # run the stream while the remaining error is worth it; taps take the remainder. The
    # stream is just presses, so it is valid wherever taps are (no grid or metric assumption).
    remaining = abs(self.v_target - self.v_cruise_cluster)
    use_fast = not self.fast_faulted and remaining >= FAST_MODE_MIN

    if use_fast and not self.fast_active:
      self.fast_active = True
      self.fast_stall_frames = 0
      self.fast_last_cluster = self.v_cruise_cluster
    elif self.fast_active:
      if remaining < FAST_MODE_MIN:
        self.fast_active = False
      elif self.v_cruise_cluster != self.fast_last_cluster:
        self.fast_last_cluster = self.v_cruise_cluster
        self.fast_stall_frames = 0
      else:
        self.fast_stall_frames += 1
        if self.fast_stall_frames * DT_CTRL > FAST_STALL_T:
          self.fast_faulted = True
          self.fast_active = False
          cloudlog.event("icbm_fast_mode_fallback", brand=self.CP.brand)

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)
    self.update_restore_quiet_timer()

    # a pending confirm prompt parks any move; transitions out of holding are gated below
    if self.prompt_frozen and self.state in (State.preActive, State.increasing, State.decreasing):
      self.state = State.holding

    # HOLDING, ACCELERATING, DECELERATING, PRE_ACTIVE
    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # Up-moves: with a valid vision lookahead the profile is the oracle (restore when
        # nothing ahead binds, hold while a dip is coming). Without it, the quiet window
        # applies on every entry path for decel_needs_stable_setpoint cars; only the
        # overshoot lever's own release while its limiter is live is exempt.
        if self.lookahead_valid:
          up_allowed = not self.dip_ahead
        else:
          up_allowed = ((self.overshoot_mph > 0 and self.limiter_active)
                        or not self.profile.decel_needs_stable_setpoint
                        or self.restore_quiet_timer >= RESTORE_QUIET_FRAMES)
        up_allowed = up_allowed and self.up_grace_timer <= 0

        # Down-moves skip the quiet window because a live limiter's decel is urgent. A
        # residual overshoot gap after the source flips back to cruise must not start a
        # fresh descent; a plain setpoint correction (no overshoot in play) stays
        # unconditional, and a fresh driver SET+ parks all of them for the grace window.
        down_allowed = (self.limiter_active or self.overshoot_mph <= 0) and self.down_grace_timer <= 0

        # PRE_ACTIVE
        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_target - self.v_cruise_cluster >= self.react_deadband and up_allowed:
              self.state = State.increasing

            elif self.v_cruise_cluster - self.v_target >= self.react_deadband \
                 and self.v_cruise_cluster > self.v_cruise_min and down_allowed:
              self.state = State.decreasing

            else:
              self.state = State.holding

        # HOLDING
        elif self.state == State.holding and not self.prompt_frozen:
          down_pending = self.v_cruise_cluster - self.v_target >= self.react_deadband and down_allowed
          up_pending = self.v_target - self.v_cruise_cluster >= self.react_deadband
          if down_pending or (up_pending and up_allowed):
            self.pre_active_timer = int(REACT_TIMER / DT_CTRL)
            self.state = State.preActive

        # ACCELERATING
        elif self.state == State.increasing:
          # a dip appearing mid-restore aborts it: stepping up until the limiter takes
          # the source feeds the next apex
          if self.v_target <= self.v_cruise_cluster or self.dip_ahead:
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
      self.plan_fast_mode()
      send_button = HOLD_BUTTONS[self.state] if self.fast_active else TAP_BUTTONS[self.state]
    else:
      self.fast_active = False
      send_button = SendButtonState.none

    return send_button

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    # buttonEvents carry only the wheel's own presses (forged frames never reach
    # carState), so the grace cannot latch on the servo's own sends
    if self.cruise_button_timers[ButtonType.accelCruise] > 0:
      self.down_grace_timer = DRIVER_PRESS_GRACE_FRAMES
      self.up_grace_timer = 0
    elif self.cruise_button_timers[ButtonType.decelCruise] > 0:
      self.up_grace_timer = DRIVER_PRESS_GRACE_FRAMES
      self.down_grace_timer = 0
    else:
      self.down_grace_timer = max(0, self.down_grace_timer - 1)
      self.up_grace_timer = max(0, self.up_grace_timer - 1)

    self.is_ready = ready and not button_pressed

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      return

    if self.frame % PARAMS_UPDATE_FRAMES == 0:
      self.decel_overshoot_enabled = self.params.get_bool("SmartCruiseDecelOvershoot")
    self.frame += 1

    self.is_metric = is_metric
    self.prompt_frozen = LP_SP.speedLimit.assist.state == SessionState.preActive

    self.update_calculations(CS, LP_SP)
    self.update_readiness(CS, CC)

    self.cruise_button = self.update_state_machine()

    self.is_ready_prev = self.is_ready
