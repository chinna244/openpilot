"""Card-side owner of the Mazda SLA session: filter-dev's cruise_ext wiring around the cruise arbiter and the servo.

Per card frame (filter-dev card.py: update_speed_limit_assist, update_v_cruise -> update_enabled_state, the dash
reconciliation, then ICBM and gate_send_button before CI.apply):
  1. update_limit() with the resolver fields plannerd published (slcState.mazdaSla)
  2. the non-pcmCruiseSpeed engage gate: enabled waits for the engaging button's release
  3. arbiter.step(): classify presses, advance the session, adopt the target into the setpoint
  4. reconcile the setpoint to the dash around a driver press (cruise_ext.reconcile_setpoint_with_dash)
  5. the servo walks the factory SET to min(setpoint, plan cap); no up move while a prompt is open
"""
import math

import numpy as np

from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.starpilot.car.mazda.sla import ACTIVE_STATES, V_CRUISE_UNSET, AssistState, CruiseIntent
from openpilot.starpilot.car.mazda.sla.arbiter import LIMIT_SETTLE, CruiseArbiter
from openpilot.starpilot.car.mazda.sla.servo import MazdaSetServo

V_CRUISE_MAX = 145
MAZDA_MIN_SET_KPH = 30.  # Mazda interface zoompilot.minimumSetSpeed (cruise_ext.get_minimum_set_speed)
IMPERIAL_INCREMENT = round(CV.MPH_TO_KPH, 1)
CRUISE_LONG_PRESS = 50
BUTTON_RAW = {n: int(getattr(car.CarState.ButtonEvent.Type, n)) for n in ('accelCruise', 'decelCruise')}
RECONCILE_SETTLE_FRAMES = int(1.0 / DT_CTRL)
LIMIT_SETTLE_FRAMES = int(LIMIT_SETTLE / DT_CTRL)
RECONCILE_AGREE_KPH = 2 * CV.MPH_TO_KPH
ARBITER_BUTTONS = ('accelCruise', 'decelCruise', 'setCruise', 'resumeCruise')
ENABLE_BUTTONS = ('decelCruise', 'accelCruise', 'setCruise', 'resumeCruise', 'cancel', 'mainCruise')
# Route 08: the SET walk tapped SET+ to the factory ECU while the car stood at a held stop, and Mazda's ECU can take SET+ at a
# standstill as a resume. No synthetic SET press while the car is stopped, held or crawling; queued ones are discarded and
# the walk starts afresh only after the car has driven off for DEPARTURE_FRAMES. A driver's own SET+ still answers an offer.
SET_WALK_MIN_SPEED = 1.0  # m/s
DEPARTURE_FRAMES = int(1.0 / DT_CTRL)


def _button_name(event) -> str:
  t = event.type
  try:
    return str(t)
  except Exception:
    return ''


class MazdaSlaSession:
  def __init__(self, CP):
    self.CP = CP
    self.arbiter = CruiseArbiter(CP)
    self.servo = MazdaSetServo()
    self.v_cruise_kph = V_CRUISE_UNSET  # the openpilot setpoint the session owns (filter-dev v_cruise_helper.v_cruise_kph)
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.v_cruise_kph_last = V_CRUISE_UNSET
    self.button_timers = {'decelCruise': 0, 'accelCruise': 0}
    self.button_change_states = {'decelCruise': (False, False), 'accelCruise': (False, False)}
    self.prompting_published = False
    self._fresh_frames = 0
    self._moving_frames = 0
    self._road_change_ns = 0
    self.enabled_prev = False
    self.enable_timers = dict.fromkeys(ENABLE_BUTTONS, 0)
    self.reconcile_frames = 0
    self.reconcile_allowed = False
    self.reconcile_floor = False
    self.send_button = 'none'
    self.hold = False
    self._next_button = 'none'  # the servo's decision this frame, applied next frame
    self._next_hold = False
    self.long_enabled = False

  @property
  def is_metric(self) -> bool:
    return self.arbiter.is_metric

  def _clip(self, kph: float) -> float:
    return float(np.clip(round(kph, 1), MAZDA_MIN_SET_KPH, V_CRUISE_MAX))

  def update(self, CS, *, cc_enabled: bool, override: bool, cancel: bool, resume: bool, resolver, servo_v_target: float | None,
             limiter_active: bool, enabled_setting: bool, is_metric: bool) -> str:
    """One card frame. resolver: slcState.mazdaSla or None; servo_v_target / limiter_active: plannerd's plan target and
    whether the session's cap drives the plan (filter-dev LP_SP.vTarget / longitudinalPlanSource), as of its last tick.
    Returns the factory SET button for CI.apply ('none'/'increase'/'decrease'); self.hold asks for the 10 Hz stream."""
    arb = self.arbiter
    arb.is_metric = is_metric
    arb.enabled = enabled_setting
    names = [(_button_name(b), bool(b.pressed)) for b in CS.buttonEvents]
    events = [car.CarState.ButtonEvent.new_message(type=n, pressed=p) for n, p in names if n in ARBITER_BUTTONS]
    CSa = _ArbiterCarState(events, CS)
    # card: update_speed_limit_assist(LP_SP)
    # StarPilot: decisions only on fresh plannerd data. Expired -> close the offer, presses are manual; recovered -> the limit
    # must hold LIMIT_SETTLE on fresh data again before anything is offered or answered
    road_changed = False
    if resolver is not None:
      road_change_ns = int(getattr(resolver, 'roadChangeMonoTime', 0))
      road_changed = road_change_ns != self._road_change_ns
      self._road_change_ns = road_change_ns
      if road_changed:
        arb.expire_inputs()
        self._fresh_frames = 0
    if resolver is None:
      if self._fresh_frames:
        arb.expire_inputs()
      self._fresh_frames = 0
    else:
      self._fresh_frames += 1
      arb.update_limit(resolver)
    arb.inputs_fresh = self._fresh_frames >= LIMIT_SETTLE_FRAMES

    # cruise.update_v_cruise -> cruise_ext.update_enabled_state (the engage gate, then the arbiter)
    self.v_cruise_kph_last = self.v_cruise_kph
    for k in self.enable_timers:
      if self.enable_timers[k] > 0:
        self.enable_timers[k] += 1
    for n, p in names:
      if n in self.enable_timers:
        self.enable_timers[n] = 1 if p else 0
    enabled = cc_enabled
    if enabled and not self.enabled_prev:
      self.enabled_prev = not any(v > 0 for v in self.enable_timers.values())
      enabled = False
    elif not enabled:
      self.enabled_prev = enabled
    enabled = enabled and self.enabled_prev
    self.long_enabled = enabled
    v_cruise_kph = arb.step(CSa, enabled, self.v_cruise_kph, self.v_cruise_cluster_kph)
    if arb.adopted_this_frame:
      self.v_cruise_kph = self.v_cruise_cluster_kph = self._clip(v_cruise_kph)
      self._clear_reconcile()

    if CS.cruiseState.available:
      if enabled:
        self._integrate_press(CS, enabled)
        self.v_cruise_cluster_kph = self.v_cruise_kph
      else:
        self.v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
        self.v_cruise_cluster_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
        if CS.cruiseState.speed == 0:
          self.v_cruise_kph = self.v_cruise_cluster_kph = V_CRUISE_UNSET
    else:
      self.v_cruise_kph = self.v_cruise_cluster_kph = V_CRUISE_UNSET
    # cruise.update_button_timers
    for k in self.button_timers:
      if self.button_timers[k] > 0:
        self.button_timers[k] += 1
    for n, p in names:
      if n in self.button_timers:
        self.button_timers[n] = 1 if p else 0
        self.button_change_states[n] = (bool(CS.cruiseState.standstill), cc_enabled)

    # card_ext.update_v_cruise_post: the plan regime plannerd reported, then reconcile
    self._reconcile(CS, limiter_active)

    stopped = bool(CS.standstill or CS.cruiseState.standstill or not CS.vEgo >= SET_WALK_MIN_SPEED)
    self._moving_frames = 0 if stopped else self._moving_frames + 1
    if road_changed or servo_v_target is None or not servo_v_target > 0. or self._moving_frames < DEPARTURE_FRAMES:
      # Discard queued SET presses at a road change, when the planner target expires, or while the car is stopped or has
      # not yet driven off. The walk starts from a fresh decision once the planner supplies a target again.
      self._next_button, self._next_hold = 'none', False
      self.send_button, self.hold = 'none', False
      self.servo.reset()
      return self.send_button

    # filter-dev runs ICBM in selfdrived on this frame's carState and card applies its button on the next frame, gated
    # by that frame's session: the button for CI.apply now is the one decided last frame
    self.send_button = arb.gate_send_button(self._next_button)
    self.hold = self._next_hold and self.send_button != 'none'

    # ICBM (op long): walk the dash to the plan target, the setpoint under the session's ramped cap
    conv = CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH
    cluster = CS.cruiseState.speedCluster
    self._next_button = self.servo.update(buttons=names, enabled=cc_enabled, override=override, cancel=cancel, resume=resume,
                                          v_target_conv=round(servo_v_target * conv), v_target_raw_conv=round(servo_v_target * conv),
                                          cluster_conv=round(cluster * conv), setpoint_unset=self.v_cruise_kph >= V_CRUISE_UNSET,
                                          limiter_active=limiter_active, prompting=self.prompting_published,
                                          v_cruise_min_conv=round(MAZDA_MIN_SET_KPH * CV.KPH_TO_MS * conv))
    self._next_hold = self.servo.fast_active
    return self.send_button

  def _integrate_press(self, CS, enabled: bool) -> None:
    """cruise._update_v_cruise_non_pcm on an ICBM car: openpilot steps its setpoint with the driver's press as the ECU
    steps the dash; presses the arbiter owns (a confirm, a dismiss) do not step it."""
    if self.v_cruise_kph == V_CRUISE_UNSET:
      return
    button, long_press = None, False
    for b in CS.buttonEvents:
      n = _button_name(b)
      if n in self.button_timers and not b.pressed:
        if self.button_timers[n] > CRUISE_LONG_PRESS:
          return
        button = n
        break
    else:
      for k, timer in self.button_timers.items():
        if timer and timer % CRUISE_LONG_PRESS == 0:
          button, long_press = k, True
          break
    if button is None:
      return
    standstill, was_enabled = self.button_change_states[button]
    if button == 'accelCruise' and (standstill or CS.cruiseState.standstill):
      return
    if not was_enabled:
      return
    if self.arbiter.press_owned(BUTTON_RAW[button]):
      return
    delta = (1. if self.is_metric else IMPERIAL_INCREMENT) * (5 if long_press else 1)
    sign = 1 if button == 'accelCruise' else -1
    if long_press and self.v_cruise_kph % delta != 0:
      self.v_cruise_kph = (math.ceil if sign > 0 else math.floor)(self.v_cruise_kph / delta) * delta
    else:
      self.v_cruise_kph += delta * sign
    self.v_cruise_kph = self._clip(self.v_cruise_kph)

  def _clear_reconcile(self) -> None:
    self.reconcile_frames = 0
    self.reconcile_allowed = False
    self.reconcile_floor = False

  def _reconcile(self, CS, limiter_drives_plan: bool) -> None:
    """cruise_ext.reconcile_setpoint_with_dash: around a driver press the dash is the truth of the setpoint."""
    if self.v_cruise_kph == V_CRUISE_UNSET and CS.cruiseState.available and CS.cruiseState.speed > 0 and CS.cruiseState.speedCluster > 0:
      self.v_cruise_kph = self.v_cruise_cluster_kph = self._clip(CS.cruiseState.speedCluster * CV.MS_TO_KPH)
    if not CS.cruiseState.available or self.v_cruise_kph in (V_CRUISE_UNSET, -1):
      self.reconcile_frames = 0
      self.reconcile_floor = False
      return
    pressed = self.enable_timers['accelCruise'] > 0 or self.enable_timers['decelCruise'] > 0
    if not pressed and self.reconcile_frames <= 0:
      return
    arb = self.arbiter
    dash_kph = CS.cruiseState.speedCluster * CV.MS_TO_KPH
    if pressed:
      if self.enable_timers['accelCruise'] == 1 or self.enable_timers['decelCruise'] == 1:
        agree_setpoint = abs(dash_kph - self.v_cruise_kph) <= RECONCILE_AGREE_KPH
        agree_sla = arb.state_prev_frame in ACTIVE_STATES and abs(dash_kph - arb.slf_kph) <= RECONCILE_AGREE_KPH
        self.reconcile_allowed = agree_setpoint or agree_sla
        if arb.last_intent == CruiseIntent.dismiss and self.enable_timers['decelCruise'] == 1:
          self.reconcile_floor = True
      self.reconcile_frames = RECONCILE_SETTLE_FRAMES
    else:
      self.reconcile_frames -= 1
    if self.reconcile_allowed:
      regime_ok = not limiter_drives_plan and self.servo.state not in ('increasing', 'decreasing') and not arb.prompting
      if regime_ok and dash_kph > 1:
        self.v_cruise_kph = self.v_cruise_cluster_kph = self._clip(dash_kph)
    if self.reconcile_floor and not pressed and self.reconcile_frames <= 0:
      self.reconcile_floor = False
      if not arb.prompting and dash_kph > 1:
        self.v_cruise_kph = self.v_cruise_cluster_kph = min(self.v_cruise_kph, self._clip(dash_kph))

  def fill(self, msg) -> None:
    """starpilotCarState.mazdaSlaSession (filter-dev carStateSP.zoompilot.cruiseSession, plus the setpoint)."""
    state, v_cap, ramp_speed, intent, announce = self.arbiter.session()
    msg.state = state
    msg.vCap = v_cap
    msg.lastIntent = intent
    msg.announceCounter = announce
    msg.setpointKph = float(self.v_cruise_kph)
    msg.longEnabled = self.long_enabled
    msg.target = self.arbiter.target


class _ArbiterCarState:
  """The CarState fields the arbiter reads, with capnp button events (the reference keys presses by enum raw)."""
  __slots__ = ('buttonEvents', 'vEgo', 'gasPressed', 'standstill', 'steeringAngleDeg')

  def __init__(self, events, CS):
    self.buttonEvents = events
    self.vEgo = CS.vEgo
    self.gasPressed = CS.gasPressed
    self.standstill = CS.standstill
    self.steeringAngleDeg = CS.steeringAngleDeg


__all__ = ['MazdaSlaSession', 'AssistState']
