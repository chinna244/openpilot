"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Cruise-button intent and speed-limit session arbitration for non-pcm cars.

The arbiter classifies each press once from the pre-frame session state and publishes
the result for the setpoint, planner, and ICBM paths. Confirmation prompts freeze the
session cap and block synthesized button output. See docs/zoompilot/cruise-arbiter.md.
"""
from dataclasses import dataclass

from openpilot.cereal import custom
from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import ACTIVE_STATES, V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import compare_cluster_target, confirm_needed_for_change, pcm_machine_owns_sla, \
  settle_conv

ButtonType = car.CarState.ButtonEvent.Type
SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
CruiseIntent = custom.CarStateZP.CruiseSession.CruiseIntent

# Timers use 100 Hz control frames.
DISABLED_GUARD_PERIOD = 0.5   # s after engagement before the session may form
PRE_ACTIVE_GUARD_PERIOD = 5.  # s a confirm prompt stays open
# A prompt nobody answered asks again while the set speed and the car are still over the limit: routes 00000160 (476 s,
# 28 s at 61-63 mph in a 58 mph target) and 00000163 (987 s, set speed 63 in a 35 mph target, 29 minus taps) went quiet
# after the one prompt. Only an unanswered prompt re-asks (any press is the driver's answer), twice at most per limit.
REPROMPT_MARGIN = 3     # display units the set speed must stand above the limit's target
REPROMPT_NEAR = 5       # display units: the car must be above target minus this, or the set speed is not yet a problem
REPROMPT_HOLD = 2.      # s the condition must hold
REPROMPT_COOLDOWN = 25.  # s after a prompt closes unanswered, about 30 s from one prompt to the next
REPROMPT_MAX = 2
# A new limit counts only once the resolver has held it this long: in route 00000167 the map came back on the road
# just left (30 for 1.0 s inside a 25 the driver had confirmed) and flicked 30-25-30 for 0.9 s, the camera read 25 for
# 1.8 s in a map gap, and each asked the driver again; in route 00000166 the camera flipped 5-35 in under 0.5 s.
LIMIT_SETTLE = 2.  # s
DRIVER_DIAL_PERIOD = 1.  # s after a wheel press edge in which a cluster match counts as the driver's dial
# Resolve a held prompt before cruise.py emits its first long-press repeat tick.
LONG_PRESS_FRAMES = 50 - 1

PLUS_BUTTONS = (ButtonType.accelCruise, ButtonType.resumeCruise)
MINUS_BUTTONS = (ButtonType.decelCruise, ButtonType.setCruise)

# Press classes are assigned at the press edge from the pre-frame state.
_PRESS_NORMAL = 0   # plain increment/decrement press
_PRESS_DISMISS = 1  # started while the session was active: owned, ends the session
_PRESS_PROMPT = 2   # started while a confirm prompt was open: resolves at release/tick


@dataclass
class _Press:
  cls: int
  frames: int = 0
  resolved: bool = False   # a resolved prompt press remains owned
  released: bool = False   # retain through the release frame for press_owned


class CruiseArbiter:
  def __init__(self, CP, CP_SP):
    # The planner-side machine keeps only the cars whose setpoint nobody but the driver can
    # move; a reachable setpoint (openpilot's own, or the ICBM buttons) is arbitrated here.
    self.CP = CP
    self.applicable = self._applicable_for(icbm_active=not CP_SP.pcmCruiseSpeed)
    # ICBM platforms adopt the ECU setpoint; non-pcm openpilot longitudinal writes it here.
    self.op_owns_setpoint = not CP.pcmCruise

    self.state = SessionState.disabled
    self.state_prev_frame = SessionState.disabled  # snapshot from before this frame's step
    self.v_cap = V_CRUISE_UNSET  # m/s; session target while active, frozen hold while prompting
    self.last_intent = CruiseIntent.none
    self.announce_counter = 0

    # The card params thread refreshes these outside the real-time path.
    self.enabled = False   # SpeedLimitMode == assist
    self.is_metric = False

    # longitudinalPlanSP updates these at the planning rate.
    self._speed_limit = 0.
    self._speed_limit_prev = 0.
    self._slf = 0.  # speedLimitFinalLast, m/s
    self._has_limit = False
    self._limit_seen = (0., 0., False)  # the resolver's latest (speedLimit, speedLimitFinalLast, valid)
    self._limit_seen_frames = 0

    self.long_enabled_prev = False
    self.long_engaged_timer = 0
    self.pre_active_timer = 0
    self._driver_dismissed = False
    self._ignored_prompt = False  # the last prompt closed unanswered; any press clears it
    self._reprompts_left = 0
    self._reprompt_cooldown = 0
    self._reprompt_hold = 0
    self._cluster_conv = 0
    self._cluster_conv_prev = 0
    self._driver_dial_timer = 0  # frames since a wheel press edge: a cluster match is the driver's

    # Key by raw integers because capnp enums do not hash-match cruise.py's integers.
    self._press: dict[int, _Press] = {}
    # The cruise helper uses this to close reconciliation before it runs.
    self.adopted_this_frame = False

  def set_icbm_active(self, icbm_active: bool) -> None:
    """card's ICBM latch moved (only while disengaged): on Mazda alpha long the session owner
    moves with it between this arbiter and the plannerd machine."""
    applicable = self._applicable_for(icbm_active)
    if applicable != self.applicable:
      self.applicable = applicable
      self._set_state(SessionState.disabled)
      self.v_cap = V_CRUISE_UNSET
      self._press.clear()

  def _applicable_for(self, icbm_active: bool) -> bool:
    return not pcm_machine_owns_sla(self.CP, icbm_active)

  def read_params(self, params):
    # read even while the plannerd machine owns the session: ICBM can hand it back here
    self.enabled = params.get("SpeedLimitMode", return_default=True) == Mode.assist
    self.is_metric = params.get_bool("IsMetric")

  def update_limit(self, LP_SP):
    if not self.applicable:
      return
    resolver = LP_SP.speedLimit.resolver
    seen = (float(resolver.speedLimit), float(resolver.speedLimitFinalLast), bool(resolver.speedLimitValid or resolver.speedLimitLastValid))
    if seen != self._limit_seen:
      self._limit_seen = seen
      self._limit_seen_frames = 0
    self._limit_seen_frames += 1
    if self._limit_seen_frames >= int(LIMIT_SETTLE / DT_CTRL):
      self._speed_limit, self._slf, self._has_limit = seen

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
    # Convert the display-rounded limit back to the setpoint's kph domain.
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

  def _enter_prompt(self, rearm: bool = True):
    if rearm:
      self._reprompts_left = REPROMPT_MAX
      self._ignored_prompt = False
    if self._target_conv() == self._cluster_conv:
      self._activate(from_prompt=False)  # the dash already shows the limit: nothing to confirm
      return
    # Preserve an active session's cap while prompting. From idle, leave the cap unset to
    # avoid round-trip error that would incorrectly classify cruise as a limiter.
    was_session = self.state in ACTIVE_STATES or self.v_cap < V_CRUISE_UNSET
    hold = self.v_cap if was_session else V_CRUISE_UNSET
    self._set_state(SessionState.preActive)
    self.v_cap = float(hold)
    self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD / DT_CTRL)

  @property
  def _speed_moves(self) -> bool:
    # the current target against the cap published last frame
    return settle_conv(self._slf, self._cluster_conv, self.is_metric) != settle_conv(self.v_cap, self._cluster_conv, self.is_metric)

  def _activate(self, from_prompt: bool):
    # Announce confirmations and sessions that move the speed, not an existing match.
    self._set_state(SessionState.active, announce=from_prompt or self._speed_moves)

  def _classify_presses(self, CS, v_cruise_kph: float) -> float:
    """Consume button edges; decide intents from the pre-frame session snapshot.

    Returns v_cruise_kph, possibly raised by an upward confirm adoption. Unclipped: the cruise
    helper applies its floor to whatever the arbiter adopts."""
    self.adopted_this_frame = False
    if self._press:
      # Release ownership lasts through the release frame.
      self._press = {btn: p for btn, p in self._press.items() if not p.released}

    for b in CS.buttonEvents:
      if b.type not in PLUS_BUTTONS and b.type not in MINUS_BUTTONS:
        continue
      btn = b.type.raw
      self._driver_dial_timer = int(DRIVER_DIAL_PERIOD / DT_CTRL)
      if b.pressed:
        self._ignored_prompt = False  # a press is an answer, even a no

      if b.pressed:
        if self.state_prev_frame in ACTIVE_STATES:
          # A press dismisses an active session. Re-anchor openpilot-owned setpoints to the
          # active cap before applying the press; ICBM platforms adopt the ECU's result.
          if self.op_owns_setpoint and self.v_cap < V_CRUISE_UNSET:
            v_cruise_kph = min(v_cruise_kph, self.target_kph)
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

    # Resolve a held prompt before the first repeated setpoint step.
    for btn, press in self._press.items():
      if press.released:
        continue
      press.frames += 1
      if press.cls == _PRESS_PROMPT and not press.resolved and press.frames >= LONG_PRESS_FRAMES:
        v_cruise_kph = self._resolve_prompt_press(btn, press, v_cruise_kph)

    return v_cruise_kph

  def _resolve_prompt_press(self, button: int, press: _Press, v_cruise_kph: float) -> float:
    if self.state != SessionState.preActive:
      # Treat an in-flight press as normal if another event resolved the prompt.
      press.cls = _PRESS_NORMAL
      return v_cruise_kph

    req_plus, req_minus = compare_cluster_target(self._cluster_conv / self._conv, self._slf, self.is_metric)
    is_plus = button in PLUS_BUTTONS

    if (req_plus and is_plus) or (req_minus and not is_plus):
      # An upward confirmation may raise the baseline; the active cap handles lower limits.
      press.resolved = True
      self.last_intent = CruiseIntent.confirm
      if is_plus and self.target_kph > v_cruise_kph:
        v_cruise_kph = self.target_kph
        self.adopted_this_frame = True
      self._activate(from_prompt=True)
    else:
      # A press in the opposite direction declines and resumes normal setpoint handling.
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

  def _reprompt_due(self, CS) -> bool:
    """An unanswered prompt, the set speed still over the limit's target, and the car at or near it."""
    target = self._target_conv()
    over = (self._ignored_prompt and self._reprompts_left > 0 and self._has_limit and target > 0
            and self._cluster_conv > target + REPROMPT_MARGIN and round(CS.vEgo * self._conv) > target - REPROMPT_NEAR
            and not CS.gasPressed and not CS.standstill)
    self._reprompt_hold = self._reprompt_hold + 1 if over else 0
    return self._reprompt_cooldown <= 0 and self._reprompt_hold >= int(REPROMPT_HOLD / DT_CTRL)

  def step(self, CS, long_enabled: bool, v_cruise_kph: float, v_cruise_cluster_kph: float) -> float:
    """Classify presses, advance the session, and publish its cap in a fixed order."""
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
    self._driver_dial_timer = max(0, self._driver_dial_timer - 1)
    self._reprompt_cooldown = max(0, self._reprompt_cooldown - 1)

    if self.state != SessionState.disabled:
      if not long_enabled or not self.enabled:
        self._set_state(SessionState.disabled)
        self._driver_dismissed = False
        self._ignored_prompt = False

      elif self.state in ACTIVE_STATES:
        if self._limit_changed and confirm_needed_for_change(self._cluster_conv, self._target_conv(), self.is_metric):
          self._enter_prompt()
        elif self._limit_changed and self._speed_moves:
          # Auto-apply target changes that do not require confirmation; a limit above the
          # setpoint the car already holds changes nothing to announce.
          self.announce_counter += 1

      elif self.state == SessionState.preActive:
        # a press resolves the prompt (above); the ICBM servo may walk the dash through the
        # limit for a curve meanwhile, which confirms nothing
        if self.pre_active_timer <= 0:
          self._set_state(SessionState.inactive)
          self._ignored_prompt = True
          self._reprompt_cooldown = int(REPROMPT_COOLDOWN / DT_CTRL)
          self._reprompt_hold = 0

      elif self.state == SessionState.inactive:
        if self._limit_changed:
          self._driver_dismissed = False
          self._enter_prompt()
        elif not self._driver_dismissed and self._has_limit and self._target_conv() == self._cluster_conv \
             and not self._press and self._driver_dial_timer > 0:
          # The driver dialed onto the limit: wait for release so they can continue through
          # it. A dash the servo walked onto it is not a dial.
          self._activate(from_prompt=False)
        elif self._reprompt_due(CS):
          self._reprompts_left -= 1
          self._reprompt_hold = 0
          self._enter_prompt(rearm=False)

    else:
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

    # Prompts retain the previous cap; active sessions publish the current target.
    if self.state in ACTIVE_STATES:
      self.v_cap = float(self._slf) if self._has_limit else V_CRUISE_UNSET
    elif self.state != SessionState.preActive:  # a prompt keeps its frozen hold
      self.v_cap = V_CRUISE_UNSET

    self._speed_limit_prev = self._speed_limit
    self.long_enabled_prev = long_enabled
    return v_cruise_kph

  def fill_msg(self, cs_sp) -> None:
    if not self.applicable:
      return
    session = cs_sp.zoompilot.cruiseSession
    session.state = self.state
    session.vCap = float(self.v_cap)
    session.lastIntent = self.last_intent
    session.announceCounter = self.announce_counter

  def gate_send_button(self, CC_SP) -> None:
    """Block synthesized up moves at prompt onset before CI.apply: the servo's view of the
    session is a hop stale, and a dash raised past the limit would be adopted on an upward
    confirm. Down moves (a curve) go through; nothing confirms on a dash the servo moved."""
    Send = structs.IntelligentCruiseButtonManagement.SendButtonState
    icbm = CC_SP.intelligentCruiseButtonManagement
    if self.applicable and self.prompting and icbm.sendButton == Send.increase:
      icbm.sendButton = Send.none
