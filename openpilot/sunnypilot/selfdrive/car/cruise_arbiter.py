"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Cruise arbiter: single owner of button meaning and the SLA session on non-pcm cars
(everything that is not pcm-op-long).

Runs at 100 Hz inside card, in the same frame as the button events and the setpoint
writer. Every +/- press is classified exactly once, into one intent, from the pre-frame
session snapshot; everything downstream (the v_cruise increment path, the reconciler,
the plannerd mirror, the ICBM servo) consumes the classification or the published
session (carStateSP.zoompilot.cruiseSession) instead of re-interpreting buttons.

A pending confirm prompt freezes speed at three altitudes: the session cap holds the
old target in the plan min(), the ICBM servo parks (prompt_frozen), and card vetoes
button emission with same-frame state (gate_send_button). Setpoint ownership, the
session model and the dismiss semantics per car class: docs/zoompilot/cruise-arbiter.md.
"""
from dataclasses import dataclass

import numpy as np

from openpilot.cereal import custom
from opendbc.car import structs
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import ACTIVE_STATES, V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import compare_cluster_target, confirm_needed_for_change

ButtonType = car.CarState.ButtonEvent.Type
SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
CruiseIntent = custom.CarStateZP.CruiseSession.CruiseIntent

# All timers are 100 Hz frame counts (DT_CTRL).
DISABLED_GUARD_PERIOD = 0.5   # s after engagement before the session may form
PRE_ACTIVE_GUARD_PERIOD = 5.  # s a confirm prompt stays open
# a prompt press resolves at long-press duration instead of release; one frame before
# cruise.py's first repeat tick (CRUISE_LONG_PRESS = 50, not importable here without a
# cycle) so the tick is already owned when it fires
LONG_PRESS_FRAMES = 50 - 1

PLUS_BUTTONS = (ButtonType.accelCruise, ButtonType.resumeCruise)
MINUS_BUTTONS = (ButtonType.decelCruise, ButtonType.setCruise)

# per-press classification, decided at the press edge from the pre-frame snapshot
_PRESS_NORMAL = 0   # plain increment/decrement press
_PRESS_DISMISS = 1  # started while the session was active: owned, ends the session
_PRESS_PROMPT = 2   # started while a confirm prompt was open: resolves at release/tick


@dataclass
class _Press:
  cls: int
  frames: int = 0
  resolved: bool = False   # prompt press answered (confirm); owned from then on
  released: bool = False   # kept through the release frame for press_owned, swept next


class CruiseArbiter:
  def __init__(self, CP, CP_SP):
    # stock-ACC button cars (ICBM) and op-long ports without pcmCruise; only pcm-op-long
    # cars keep the plannerd machine
    self.applicable = not (CP.openpilotLongitudinalControl and CP.pcmCruise)
    # who holds the setpoint: the ECU steps the dash on ICBM cars and the reconciler adopts
    # it; on op-long ports without pcmCruise any re-anchor has to be written here
    self.op_owns_setpoint = not CP.pcmCruise

    # session
    self.state = SessionState.disabled
    self.state_prev_frame = SessionState.disabled  # snapshot from before this frame's step
    self.v_cap = V_CRUISE_UNSET  # m/s; session target while active, frozen hold while prompting
    self.last_intent = CruiseIntent.none
    self.announce_counter = 0

    # params, refreshed off the RT path (card params thread)
    self.enabled = False   # SpeedLimitMode == assist
    self.is_metric = False

    # resolver inputs (from longitudinalPlanSP, updated at plan rate)
    self._speed_limit = 0.
    self._speed_limit_prev = 0.
    self._slf = 0.  # speedLimitFinalLast, m/s
    self._has_limit = False

    # machine state
    self.long_enabled_prev = False
    self.long_engaged_timer = 0
    self.pre_active_timer = 0
    self._driver_dismissed = False
    self._cluster_conv = 0
    self._cluster_conv_prev = 0

    # press tracking, keyed by raw enumerant int (capnp _DynamicEnum instances do not
    # hash-match the raw ints cruise.py passes into press_owned)
    self._press: dict[int, _Press] = {}
    # set for the frame the arbiter writes the setpoint; the helper consumes it and kills
    # the reconcile window before the reconciler runs
    self.adopted_this_frame = False

  # ---- params (called from card's params thread, never the 100 Hz path) -------------
  def read_params(self, params):
    if not self.applicable:
      return
    self.enabled = params.get("SpeedLimitMode", return_default=True) == Mode.assist
    self.is_metric = params.get_bool("IsMetric")

  # ---- resolver inputs (card, on longitudinalPlanSP updates) ------------------------
  def update_limit(self, LP_SP):
    if not self.applicable:
      return
    resolver = LP_SP.speedLimit.resolver
    self._speed_limit = float(resolver.speedLimit)
    self._slf = float(resolver.speedLimitFinalLast)
    self._has_limit = bool(resolver.speedLimitValid or resolver.speedLimitLastValid)

  # ---- helpers ----------------------------------------------------------------------
  @property
  def _conv(self) -> float:
    return CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

  @property
  def prompting(self) -> bool:
    return self.state == SessionState.preActive

  @property
  def session_active(self) -> bool:
    return self.state in ACTIVE_STATES

  def _target_conv(self) -> int:
    return round(self._slf * self._conv)

  @property
  def target_kph(self) -> float:
    # the limit rounded to a display integer, expressed in kph (the setpoint's unit)
    conv = 1. if self.is_metric else CV.KPH_TO_MPH
    return round(self._slf * CV.MS_TO_KPH * conv) / conv

  @property
  def slf_kph(self) -> float:
    return self._slf * CV.MS_TO_KPH

  @property
  def _limit_changed(self) -> bool:
    return self._has_limit and self._speed_limit != self._speed_limit_prev

  def _set_state(self, state, announce=False):
    self.state = state
    if announce:
      self.announce_counter += 1

  def _enter_prompt(self):
    # Out of an active session the hold is the session's last cap, so the dash cannot
    # restore un-confirmed. From idle publish no cap at all: a cap equal to the baseline
    # lands mm/s under v_cruise after the display-unit round trip and relabels the plan
    # source as a limiter, which arms ICBM's overshoot against a plain cruise convergence.
    was_session = self.state in ACTIVE_STATES or self.v_cap < V_CRUISE_UNSET
    hold = self.v_cap if was_session else V_CRUISE_UNSET
    self._set_state(SessionState.preActive)
    self.v_cap = float(hold)
    self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD / DT_CTRL)

  def _activate(self, from_prompt: bool):
    # announce a resolved prompt or an upcoming walk; activating because the setpoint
    # already matches the limit is silent
    announce = from_prompt or self._target_conv() != self._cluster_conv
    self._set_state(SessionState.active, announce=announce)

  # ---- press classification ----------------------------------------------------------
  def _classify_presses(self, CS, v_cruise_kph: float) -> float:
    """Consume button edges; decide intents from the pre-frame session snapshot.

    Returns v_cruise_kph, possibly raised by an upward confirm adoption."""
    self.adopted_this_frame = False
    if self._press:
      # releases stayed through their frame for press_owned; sweep them now
      self._press = {btn: p for btn, p in self._press.items() if not p.released}

    for b in CS.buttonEvents:
      if b.type not in PLUS_BUTTONS and b.type not in MINUS_BUTTONS:
        continue
      btn = b.type.raw

      if b.pressed:
        if self.state_prev_frame in ACTIVE_STATES:
          # A press on an active session dismisses it at the press edge. On ICBM cars the
          # press is owned (the ECU steps the dash, the reconciler adopts it); where
          # openpilot holds the setpoint, v_cruise is re-anchored to what the plan was
          # running (the cap, never above the baseline) and the press steps from there.
          if self.op_owns_setpoint and self.v_cap < V_CRUISE_UNSET:
            anchor = min(v_cruise_kph, self.target_kph)
            v_cruise_kph = float(np.clip(round(anchor, 1), get_minimum_set_speed(self.is_metric), V_CRUISE_MAX))
            self.adopted_this_frame = True
            self._press[btn] = _Press(_PRESS_NORMAL)
          else:
            self._press[btn] = _Press(_PRESS_DISMISS)
          self._set_state(SessionState.inactive)
          self._driver_dismissed = True
          self.last_intent = CruiseIntent.dismiss
        elif self.state_prev_frame == SessionState.preActive:
          self._press[btn] = _Press(_PRESS_PROMPT)
        else:
          self._press[btn] = _Press(_PRESS_NORMAL)

      else:  # release
        press = self._press.get(btn)
        if press is None:
          continue
        if press.cls == _PRESS_PROMPT and not press.resolved:
          v_cruise_kph = self._resolve_prompt_press(btn, press, v_cruise_kph)
        if press.cls == _PRESS_NORMAL and self.last_intent == CruiseIntent.none:
          self.last_intent = CruiseIntent.increment if btn in PLUS_BUTTONS else CruiseIntent.decrement
        press.released = True

    # a prompt press that reaches long-press duration resolves at the first tick instead
    # of waiting for release (the ECU is already grid-stepping)
    for btn, press in self._press.items():
      if press.released:
        continue
      press.frames += 1
      if press.cls == _PRESS_PROMPT and not press.resolved and press.frames >= LONG_PRESS_FRAMES:
        v_cruise_kph = self._resolve_prompt_press(btn, press, v_cruise_kph)

    return v_cruise_kph

  def _resolve_prompt_press(self, button: int, press: _Press, v_cruise_kph: float) -> float:
    if self.state != SessionState.preActive:
      # the prompt resolved some other way (timeout, dial-to-target) while the press was
      # in flight; treat as a plain press
      press.cls = _PRESS_NORMAL
      return v_cruise_kph

    req_plus, req_minus = compare_cluster_target(self._cluster_conv / self._conv, self._slf, self.is_metric)
    is_plus = button in PLUS_BUTTONS

    if (req_plus and is_plus) or (req_minus and not is_plus):
      # confirm; an upward confirm raises the setpoint to the limit (never lowers it: a
      # baseline above the limit stays and the active session caps the plan instead)
      press.resolved = True
      self.last_intent = CruiseIntent.confirm
      if is_plus and self.target_kph > v_cruise_kph:
        v_cruise_kph = float(np.clip(round(self.target_kph, 1), get_minimum_set_speed(self.is_metric), V_CRUISE_MAX))
        self.adopted_this_frame = True
      self._activate(from_prompt=True)
    else:
      # a press against the confirm direction declines: the session ends at once so the
      # frozen hold releases; the press still counts as a normal increment
      press.cls = _PRESS_NORMAL
      self.last_intent = CruiseIntent.decline
      self._set_state(SessionState.inactive)

    return v_cruise_kph

  def press_owned(self, button_type: int) -> bool:
    """True when this press must not increment v_cruise (confirm- or dismiss-owned).
    Takes the raw enumerant int (as cruise.py's button paths carry); valid for release
    events and long-press repeat ticks in the same frame."""
    press = self._press.get(button_type)
    if press is None:
      return False
    if press.cls == _PRESS_DISMISS:
      return True
    return press.cls == _PRESS_PROMPT and press.resolved

  # ---- main step (card 100 Hz) -------------------------------------------------------
  def step(self, CS, long_enabled: bool, v_cruise_kph: float, v_cruise_cluster_kph: float) -> float:
    """Pinned sub-step order: snapshot the pre-frame session state; classify press
    edges/ticks from it (an upward confirm or an op-long dismiss may write v_cruise and
    flag the reconcile kill); step the session machine; refresh the published cap. The
    caller then runs the increment path (press_owned) and the reconciler."""
    if not self.applicable:
      return v_cruise_kph

    self.state_prev_frame = self.state
    conv = CV.KPH_TO_MS * self._conv
    self._cluster_conv_prev = self._cluster_conv
    self._cluster_conv = round(v_cruise_cluster_kph * conv) if v_cruise_cluster_kph not in (V_CRUISE_UNSET, -1) else 0
    self.last_intent = CruiseIntent.none

    v_cruise_kph = self._classify_presses(CS, v_cruise_kph)

    self.long_engaged_timer = max(0, self.long_engaged_timer - 1)
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    if self.state != SessionState.disabled:
      if not long_enabled or not self.enabled:
        self._set_state(SessionState.disabled)
        self._driver_dismissed = False

      elif self.state in ACTIVE_STATES:
        # dismiss is handled at the press edge in classification
        if self._limit_changed and confirm_needed_for_change(self._cluster_conv, self._target_conv(), self.is_metric):
          self._enter_prompt()
        elif self._limit_changed and self._target_conv() != self._cluster_conv:
          # CST auto-apply: the new target takes over without confirmation
          self.announce_counter += 1

      elif self.state == SessionState.preActive:
        # confirm/decline are handled at release/tick in classification
        if self._target_conv() == self._cluster_conv:
          self._activate(from_prompt=True)  # dialing onto the target answers the prompt
        elif self.pre_active_timer <= 0:
          self._set_state(SessionState.inactive)

      elif self.state == SessionState.inactive:
        if self._limit_changed:
          self._driver_dismissed = False
          self._enter_prompt()
        elif not self._driver_dismissed and self._has_limit and self._target_conv() == self._cluster_conv \
             and not self._press:
          # dial-to-target latches only once the press is over: latching mid-hold would
          # cap a driver who is dialing past the limit
          self._activate(from_prompt=False)

    else:  # DISABLED
      if long_enabled and self.enabled:
        if not self.long_enabled_prev or self._cluster_conv != self._cluster_conv_prev:
          self.long_engaged_timer = int(DISABLED_GUARD_PERIOD / DT_CTRL)
        elif self.long_engaged_timer <= 0:
          if self._has_limit and self._target_conv() == self._cluster_conv:
            self._activate(from_prompt=False)
          elif self._has_limit:
            self._enter_prompt()
          else:
            self._set_state(SessionState.inactive)

    # published cap: session target while active, the frozen hold while prompting
    if self.state in ACTIVE_STATES:
      self.v_cap = float(self._slf) if self._has_limit else V_CRUISE_UNSET
    elif self.state != SessionState.preActive:  # a prompt keeps its frozen hold
      self.v_cap = V_CRUISE_UNSET

    self._speed_limit_prev = self._speed_limit
    self.long_enabled_prev = long_enabled
    return v_cruise_kph

  # ---- publishing / output gating ----------------------------------------------------
  def fill_msg(self, cs_sp) -> None:
    if not self.applicable:
      return
    session = cs_sp.zoompilot.cruiseSession
    session.state = self.state
    session.vCap = float(self.v_cap)
    session.lastIntent = self.last_intent
    session.announceCounter = self.announce_counter

  def gate_send_button(self, CC_SP) -> None:
    """Authoritative emission gate, called by card just before CI.apply: the servo's own
    prompt freeze is one message hop stale, so a button frame could otherwise escape at
    prompt onset."""
    if self.applicable and self.prompting:
      CC_SP.intelligentCruiseButtonManagement.sendButton = structs.IntelligentCruiseButtonManagement.SendButtonState.none
