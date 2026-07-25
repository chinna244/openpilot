"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import ACTIVE_STATES as SLA_ACTIVE_STATES
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import compare_cluster_target

ButtonType = car.CarState.ButtonEvent.Type
SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
IcbmState = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState

CRUISE_BUTTON_TIMER = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0,
                       ButtonType.setCruise: 0, ButtonType.resumeCruise: 0,
                       ButtonType.cancel: 0, ButtonType.mainCruise: 0}

V_CRUISE_MIN = 8
V_CRUISE_MAX = 145
V_CRUISE_UNSET = 255

# Setpoint reconciliation for non-pcmCruiseSpeed (ICBM) cars. The car's own ECU keeps the
# real set speed and steps it on wheel-button presses, while openpilot integrates the same
# presses independently — the two drift whenever their stepping cadences differ (long-press
# snaps to a grid, gas-override presses, the ECU applying a trailing long-press increment
# right after release). The dash is the ECU's truth of the driver's setpoint, but only when
# nothing is deliberately holding it away from v_cruise: while a limiter (SCC/SLA) is the
# plan source or ICBM is mid-move, dash != v_cruise by design and adopting it would destroy
# the driver's baseline. So: while the driver is pressing +/- and briefly after, adopt the
# dash as the source of truth iff the plan source is cruise and ICBM is not actively stepping.
# The settle time absorbs the ECU's trailing long-press increment (lands well inside 1 s on
# a Mazda CX-5 2022; revalidate for other ICBM brands whose ECUs step differently).
RECONCILE_SETTLE_TIME = 1.0  # s after the last press
RECONCILE_SETTLE_FRAMES = int(RECONCILE_SETTLE_TIME / DT_CTRL)
RECONCILE_BUTTONS = (ButtonType.accelCruise, ButtonType.decelCruise)
# The dash must have been at an intended resting value when the press started: the driver
# setpoint (normal cruising, incl. small drift from dropped presses) or an active SLA
# session's target (a settled re-anchor press). A dash in transit matches neither — e.g.
# a press that aborts an SLA move mid-walk: the servo gets knocked idle and the plan
# source snaps back to cruise on that same press, so without this latch the settle window
# would adopt the half-walked dash and destroy the baseline the servo is about to restore.
RECONCILE_AGREE_KPH = 2 * CV.MPH_TO_KPH


def update_manual_button_timers(CS: car.CarState, button_timers: dict[car.CarState.ButtonEvent.Type, int]) -> None:
  # increment timer for buttons still pressed
  for k in button_timers:
    if button_timers[k] > 0:
      button_timers[k] += 1

  for b in CS.buttonEvents:
    if b.type.raw in button_timers:
      # Start/end timer and store current state on change of button pressed
      button_timers[b.type.raw] = 1 if b.pressed else 0


class VCruiseHelperSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self.CP = CP
    self.CP_SP = CP_SP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.params = Params()
    self.v_cruise_min = 0
    self.enabled_prev = False

    self.custom_acc_enabled = self.params.get_bool("CustomAccIncrementsEnabled")
    self.short_increment = self.params.get("CustomAccShortPressIncrement", return_default=True)
    self.long_increment = self.params.get("CustomAccLongPressIncrement", return_default=True)

    self.enable_button_timers = dict(CRUISE_BUTTON_TIMER)

    # Setpoint reconciliation (non-pcmCruiseSpeed cars)
    self.reconcile_frames = 0
    self.reconcile_allowed = False
    self._press_owned_by_sla = False

    # Plan/actuation regime, updated from longitudinalPlanSP + carControlSP each frame
    self.lp_source = LongitudinalPlanSource.cruise
    self.icbm_state = IcbmState.inactive

    # Speed Limit Assist
    self.sla_state = SpeedLimitAssistState.disabled
    self.prev_sla_state = SpeedLimitAssistState.disabled
    self.has_speed_limit = False
    self.speed_limit_final_last = 0.
    self.speed_limit_final_last_kph = 0.
    self.req_plus = False
    self.req_minus = False

  def read_custom_set_speed_params(self) -> None:
    self.custom_acc_enabled = self.params.get_bool("CustomAccIncrementsEnabled")
    self.short_increment = self.params.get("CustomAccShortPressIncrement", return_default=True)
    self.long_increment = self.params.get("CustomAccLongPressIncrement", return_default=True)

  def update_v_cruise_delta(self, long_press: bool, v_cruise_delta: float) -> tuple[bool, float]:
    if not self.custom_acc_enabled:
      v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
      return long_press, v_cruise_delta

    # Apply user-specified multipliers to the base increment
    short_increment = np.clip(self.short_increment, 1, 10)
    long_increment = np.clip(self.long_increment, 1, 10)

    actual_increment = long_increment if long_press else short_increment
    round_to_nearest = actual_increment in (5, 10)
    v_cruise_delta = v_cruise_delta * actual_increment

    return round_to_nearest, v_cruise_delta

  def get_minimum_set_speed(self, is_metric: bool) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      self.v_cruise_min = V_CRUISE_MIN
      return

    self.v_cruise_min = get_minimum_set_speed(is_metric)

  def update_enabled_state(self, CS: car.CarState, enabled: bool) -> bool:
    # special enabled state for non pcmCruiseSpeed, unchanged for non pcmCruise
    if not self.CP_SP.pcmCruiseSpeed:
      update_manual_button_timers(CS, self.enable_button_timers)
      button_pressed = any(self.enable_button_timers[k] > 0 for k in self.enable_button_timers)

      # Ownership is decided at the press edge and holds for the whole press. The increment
      # applies on the release edge, and by then SLA has usually already consumed the press
      # and gone inactive — deciding at release would let an SLA-owned press (confirm,
      # abort, re-anchor) leak through and bump the setpoint it was supposed to leave alone.
      for b in (ButtonType.accelCruise, ButtonType.decelCruise):
        if self.enable_button_timers[b] == 1:
          self._press_owned_by_sla = self.sla_state in SLA_ACTIVE_STATES

      if enabled and not self.enabled_prev:
        self.enabled_prev = not button_pressed
        enabled = False
      elif not enabled:
        self.enabled_prev = enabled

      return enabled and self.enabled_prev

    return enabled

  def reconcile_setpoint_with_dash(self, CS: car.CarState) -> None:
    if self.CP_SP.pcmCruiseSpeed or not self.CP.pcmCruise:
      return

    if not CS.cruiseState.available or self.v_cruise_kph in (V_CRUISE_UNSET, -1):
      self.reconcile_frames = 0
      return

    dash_kph = CS.cruiseState.speed * CV.MS_TO_KPH

    pressed = any(self.enable_button_timers[b] > 0 for b in RECONCILE_BUTTONS)
    if pressed:
      if self.reconcile_frames <= 0:
        # evaluated once at press start, before the press's own ECU effect lands
        agree_setpoint = abs(dash_kph - self.v_cruise_kph) <= RECONCILE_AGREE_KPH
        sla_session = self.sla_state in SLA_ACTIVE_STATES or self.prev_sla_state in SLA_ACTIVE_STATES
        agree_sla = sla_session and abs(dash_kph - self.speed_limit_final_last_kph) <= RECONCILE_AGREE_KPH
        self.reconcile_allowed = agree_setpoint or agree_sla
      self.reconcile_frames = RECONCILE_SETTLE_FRAMES
    elif self.reconcile_frames > 0:
      self.reconcile_frames -= 1
    else:
      return

    if not self.reconcile_allowed:
      return

    # Per-frame regime gates: even a legitimate window must not adopt while a limiter is
    # driving the plan or ICBM is stepping the dash — those are the moments the dash is
    # deliberately somewhere else.
    if self.lp_source != LongitudinalPlanSource.cruise:
      return
    if self.icbm_state in (IcbmState.increasing, IcbmState.decreasing):
      return

    if dash_kph > 1:
      self.v_cruise_kph = float(np.clip(round(dash_kph, 1), self.v_cruise_min, V_CRUISE_MAX))
      self.v_cruise_cluster_kph = self.v_cruise_kph

  def update_speed_limit_assist(self, is_metric, LP_SP: custom.LongitudinalPlanSP,
                                CC_SP: custom.CarControlSP) -> None:
    resolver = LP_SP.speedLimit.resolver
    self.has_speed_limit = resolver.speedLimitValid or resolver.speedLimitLastValid
    self.speed_limit_final_last = LP_SP.speedLimit.resolver.speedLimitFinalLast
    self.speed_limit_final_last_kph = self.speed_limit_final_last * CV.MS_TO_KPH
    self.prev_sla_state = self.sla_state
    self.sla_state = LP_SP.speedLimit.assist.state
    self.lp_source = LP_SP.longitudinalPlanSource
    self.icbm_state = CC_SP.intelligentCruiseButtonManagement.state
    self.req_plus, self.req_minus = compare_cluster_target(self.v_cruise_cluster_kph * CV.KPH_TO_MS,
                                                           self.speed_limit_final_last, is_metric)

  def update_speed_limit_assist_pre_active_confirmed(self, button_type: car.CarState.ButtonEvent.Type) -> bool:
    if self.sla_state == SpeedLimitAssistState.preActive or self.prev_sla_state == SpeedLimitAssistState.preActive:
      if button_type == ButtonType.decelCruise and self.req_minus:
        return True
      if button_type == ButtonType.accelCruise and self.req_plus:
        return True

    return False

  @property
  def speed_limit_assist_owns_buttons(self) -> bool:
    # A press that started while SLA was active carries SLA semantics (abort a move in
    # flight, or re-anchor to the dash once settled) — never a v_cruise increment. The dash
    # keeps the ECU's response to the press; reconcile_setpoint_with_dash adopts it once
    # the plan source is back to cruise and the servo is idle. Incrementing here as well
    # would double-count the press against the ECU's own step.
    return self._press_owned_by_sla
