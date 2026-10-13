"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A SET/RES press before openpilot owns the stock ECU it stands in for is answered by an alert
in the shape of upstream's no-entry conditions, keyed on the published state, whatever
selfdrived and MADS are doing. Route 0000021b (2026-09-11): three minutes of presses under
parkToTakeOver with lateral off, nothing shown.
"""
import copy

import pytest

from openpilot.cereal import custom, messaging
from opendbc.car import DT_CTRL, structs
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET, AudibleAlert

EventNameSP = custom.OnroadEventSP.EventName
ButtonType = structs.CarState.ButtonEvent.Type


def _car_events(brand: str = 'mazda') -> CarSpecificEventsSP:
  CP = structs.CarParams()
  CP.brand = brand
  return CarSpecificEventsSP(CP, structs.CarParamsSP())


def _update(car_events: CarSpecificEventsSP, state: str, button=None, pressed: bool = True, standstill: bool = False):
  CS = structs.CarState()
  CS.standstill = standstill
  if button is not None:
    CS.buttonEvents = [structs.CarState.ButtonEvent(type=button, pressed=pressed)]
  CS_SP = messaging.new_message('carStateSP').carStateSP
  CS_SP.zoompilot.stockEcu = state
  return car_events.update(CS, Events(), CS_SP)


def _events_for(state: str, button=ButtonType.accelCruise, pressed: bool = True, brand: str = 'mazda'):
  return _update(_car_events(brand), state, button, pressed)


def _car_state_sp(state: str):
  CS_SP = messaging.new_message('carStateSP').carStateSP
  CS_SP.zoompilot.stockEcu = state
  return CS_SP


def _alert(state: str, standstill: bool):
  CS = structs.CarState()
  CS.standstill = standstill
  alert = EVENTS_SP[EventNameSP.stockEcuNotReady][ET.PERMANENT](structs.CarParams(), CS, {'carStateSP': _car_state_sp(state)}, False, 0, None)
  return alert, {alert.alert_text_1, alert.alert_text_2}  # mici swaps the two lines


class TestStockEcuNotReady:
  @pytest.mark.parametrize("state", ["starting", "parkToTakeOver", "restoring", "failed"])
  def test_a_set_press_before_ownership_raises_it(self, state):
    assert _events_for(state).has(EventNameSP.stockEcuNotReady)

  @pytest.mark.parametrize("state", ["notNeeded", "ready", "restored", "stockCruiseOn"])
  def test_an_owned_or_stock_body_engages_without_comment(self, state):
    assert not _events_for(state).has(EventNameSP.stockEcuNotReady)

  def test_only_set_and_resume_presses_count(self):
    assert not _events_for("parkToTakeOver", button=ButtonType.mainCruise).has(EventNameSP.stockEcuNotReady)
    assert not _events_for("parkToTakeOver", button=ButtonType.cancel).has(EventNameSP.stockEcuNotReady)
    assert _events_for("parkToTakeOver", button=ButtonType.resumeCruise).has(EventNameSP.stockEcuNotReady)
    assert not _events_for("parkToTakeOver", pressed=False).has(EventNameSP.stockEcuNotReady)

  def test_brand_independent(self):
    assert _events_for("starting", brand='honda').has(EventNameSP.stockEcuNotReady)

  def test_shows_with_nothing_engaged(self):
    # selfdrived disabled, MADS not active: the state a driver presses SET in before the
    # takeover, and the one the *AlertOnly WARNING convention never reaches
    events_sp = _events_for("parkToTakeOver")
    machine = StateMachine()
    machine.update(Events())
    args = [structs.CarParams(), structs.CarState(), {'carStateSP': _car_state_sp("parkToTakeOver")}, False, 0, None]
    alerts = events_sp.create_alerts(machine.current_alert_types, args)
    assert [a.alert_type for a in alerts] == ["stockEcuNotReady/permanent"]

  def test_the_no_entry_shape(self):
    alert, _ = _alert("parkToTakeOver", standstill=False)
    assert alert.audible_alert == AudibleAlert.refuse
    assert alert.duration == int(3. / DT_CTRL)  # frames

  @pytest.mark.parametrize("state, standstill, expected", [
    ("starting", True, {"AEB turning off", "Stay still"}),
    ("starting", False, {"AEB turning off", "Stop to finish"}),
    ("parkToTakeOver", False, {"openpilot cruise at stop", "Stop first"}),
    ("stockCruiseOn", False, {"car cruise is on", "Cancel first"}),
    ("restoring", False, {"car cruise restarting", "Wait ~12 s"}),
    ("failed", True, {"radar switch failed", "Restart car"}),
  ])
  def test_text_names_what_the_driver_has_to_do(self, state, standstill, expected):
    assert _alert(state, standstill)[1] == expected


class TestStockEcuUnprompted:
  """Two alerts need no press: parked with the takeover still starting (waiting helps; route
  0000021b pulled away 5 s short of it), and the edge into ready."""

  def test_parked_and_starting_shows_the_line_every_frame(self):
    car_events = _car_events()
    for _ in range(3):
      events_sp = _update(car_events, "starting", standstill=True)
      assert events_sp.has(EventNameSP.stockEcuInitializing)
      assert not events_sp.has(EventNameSP.stockEcuNotReady)  # no press, no press alert

  def test_moving_clears_it(self):
    car_events = _car_events()
    assert _update(car_events, "starting", standstill=True).has(EventNameSP.stockEcuInitializing)
    assert not _update(car_events, "starting", standstill=False).has(EventNameSP.stockEcuInitializing)

  @pytest.mark.parametrize("state", ["notNeeded", "parkToTakeOver", "stockCruiseOn", "ready", "restoring", "restored", "failed"])
  def test_only_the_starting_state(self, state):
    assert not _update(_car_events(), state, standstill=True).has(EventNameSP.stockEcuInitializing)

  def test_ready_edge_fires_once(self):
    car_events = _car_events()
    assert not _update(car_events, "starting", standstill=True).has(EventNameSP.stockEcuReady)
    assert _update(car_events, "ready").has(EventNameSP.stockEcuReady)
    assert not _update(car_events, "ready").has(EventNameSP.stockEcuReady)
    # a returned radar (accFaulted) and a fresh ownership: told again
    _update(car_events, "starting")
    assert _update(car_events, "ready").has(EventNameSP.stockEcuReady)

  def test_boot_straight_into_ready_is_still_told(self):
    # first frame of a session already owned (comma restart onroad adopting a quiet radar)
    assert _update(_car_events(), "ready").has(EventNameSP.stockEcuReady)

  def test_shape(self):
    initializing = EVENTS_SP[EventNameSP.stockEcuInitializing][ET.PERMANENT]
    ready = EVENTS_SP[EventNameSP.stockEcuReady][ET.PERMANENT]
    assert initializing.audible_alert == AudibleAlert.none
    assert ready.audible_alert == AudibleAlert.prompt
    assert ready.priority >= EVENTS_SP[EventNameSP.stockEcuNotReady][ET.PERMANENT](
      structs.CarParams(), structs.CarState(), {'carStateSP': _car_state_sp("starting")}, False, 0, None).priority
    assert (initializing.alert_text_1, initializing.alert_text_2) == ("Stay still", "AEB turning off")
    assert (ready.alert_text_1, ready.alert_text_2) == ("AEB off", "SET+ for alpha long")
    assert ready.duration == int(2. / DT_CTRL)

  def test_both_show_with_nothing_engaged(self):
    car_events = _car_events()
    _update(car_events, "starting", standstill=True)
    machine = StateMachine()
    machine.update(Events())
    args = [structs.CarParams(), structs.CarState(), {'carStateSP': _car_state_sp("ready")}, False, 0, None]
    shown = lambda events_sp: [a.alert_type for a in events_sp.create_alerts(machine.current_alert_types, args)]  # noqa: E731
    assert shown(_update(car_events, "starting", standstill=True)) == ["stockEcuInitializing/permanent"]
    assert shown(_update(car_events, "ready")) == ["stockEcuReady/permanent"]


class TestStockCruiseChimes:
  """The car's own cruise on the stock radar plays nothing itself (route 0000012e): openpilot
  mirrors its engagement with the sound-only engage and disengage chimes, once per edge."""

  def test_engage_and_disengage_chime_once(self):
    car_events = _car_events()
    _update(car_events, "notNeeded")
    assert _update(car_events, "stockCruiseOn").has(EventNameSP.longitudinalEnableChime)
    held = _update(car_events, "stockCruiseOn")
    assert not held.has(EventNameSP.longitudinalEnableChime) and not held.has(EventNameSP.longitudinalDisableChime)
    assert _update(car_events, "notNeeded").has(EventNameSP.longitudinalDisableChime)
    assert not _update(car_events, "notNeeded").has(EventNameSP.longitudinalDisableChime)

  def test_end_of_drive_hand_back_under_the_car_cruise_is_silent(self):
    car_events = _car_events()
    _update(car_events, "stockCruiseOn")
    assert not _update(car_events, "restored").has(EventNameSP.longitudinalDisableChime)

  @pytest.mark.parametrize("state", ["notNeeded", "parkToTakeOver", "starting", "ready", "restoring", "restored", "failed"])
  def test_other_states_are_silent(self, state):
    car_events = _car_events()
    _update(car_events, "notNeeded")
    events_sp = _update(car_events, state)
    assert not events_sp.has(EventNameSP.longitudinalEnableChime) and not events_sp.has(EventNameSP.longitudinalDisableChime)


def _update_silenced(car_events: CarSpecificEventsSP, silenced: bool, state: str = "notNeeded", setting_off: bool = False):
  CS_SP = messaging.new_message('carStateSP').carStateSP
  CS_SP.zoompilot.stockEcu = state
  CS_SP.zoompilot.stockEcuOff = silenced
  CS_SP.zoompilot.aebSettingOff = setting_off
  return car_events.update(structs.CarState(), Events(), CS_SP)


class TestStockEcuHandedBack:
  """The stock ECU back in its default session (Mazda AEB on again) is said once, with a chime."""

  def test_said_once_on_the_edge(self):
    car_events = _car_events()
    assert not _update_silenced(car_events, False).has(EventNameSP.stockEcuHandedBack)
    assert not _update_silenced(car_events, True, "ready").has(EventNameSP.stockEcuHandedBack)
    assert _update_silenced(car_events, False, "restoring").has(EventNameSP.stockEcuHandedBack)
    assert not _update_silenced(car_events, False, "restoring").has(EventNameSP.stockEcuHandedBack)

  def test_not_said_while_the_driver_has_aeb_off(self):
    car_events = _car_events()
    _update_silenced(car_events, True, "ready", setting_off=True)
    assert not _update_silenced(car_events, False, "restoring", setting_off=True).has(EventNameSP.stockEcuHandedBack)

  def test_shape(self):
    alert = EVENTS_SP[EventNameSP.stockEcuHandedBack][ET.PERMANENT]
    assert (alert.alert_text_1, alert.alert_text_2) == ("AEB on", "radar back to car")
    assert alert.audible_alert == AudibleAlert.prompt


class TestAlphaLongNextStop:
  """RES registered as a request for alpha long is said once, with a soft chime, while driving."""

  def update(self, car_events, requested, standstill=False):
    CS_SP = messaging.new_message('carStateSP').carStateSP
    CS_SP.zoompilot.alphaLongRequested = requested
    CS = structs.CarState()
    CS.standstill = standstill
    return car_events.update(CS, Events(), CS_SP)

  def test_said_once_per_request(self):
    car_events = _car_events()
    assert not self.update(car_events, False).has(EventNameSP.alphaLongNextStop)
    assert self.update(car_events, True).has(EventNameSP.alphaLongNextStop)
    assert not self.update(car_events, True).has(EventNameSP.alphaLongNextStop)
    assert not self.update(car_events, True, standstill=True).has(EventNameSP.alphaLongNextStop)
    assert not self.update(car_events, True).has(EventNameSP.alphaLongNextStop)  # same request
    self.update(car_events, False)
    assert self.update(car_events, True).has(EventNameSP.alphaLongNextStop)  # a new one

  def test_pressed_at_a_stop_is_said_on_pulling_away(self):
    car_events = _car_events()
    assert not self.update(car_events, True, standstill=True).has(EventNameSP.alphaLongNextStop)
    assert self.update(car_events, True).has(EventNameSP.alphaLongNextStop)

  def test_shape(self):
    alert = EVENTS_SP[EventNameSP.alphaLongNextStop][ET.PERMANENT]
    assert (alert.alert_text_1, alert.alert_text_2) == ("Alpha long", "next stop, 2 s")
    assert alert.audible_alert == AudibleAlert.prompt


class TestSoundOnce:
  """An AEB message interrupted by another alert (the car's cruise engage chime, a gear change)
  comes back silent, not with a second chime (route 00000133, 163.3 s and 271.2 s)."""

  @staticmethod
  def entry(event, name):
    alert = copy.copy(EVENTS_SP[event][ET.PERMANENT])
    alert.alert_type = f"{name}/permanent"
    return alert

  def run(self, ours):
    from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
    chime = self.entry(EventNameSP.longitudinalEnableChime, "longitudinalEnableChime")
    am, sounds = AlertManager(), []
    for frame in range(ours.duration - 5):
      am.add_many(frame, [ours] if frame == 0 else [chime] if frame == 50 else [])
      am.process_alerts(frame, set())
      if am.current_alert.alert_text_1 == ours.alert_text_1:
        sounds.append(am.current_alert.audible_alert)
    return sounds

  @pytest.mark.parametrize("event", ["stockEcuReady", "stockEcuHandedBack", "alphaLongNextStop"])
  def test_back_silent_after_an_interruption(self, event):
    sounds = self.run(self.entry(getattr(EventNameSP, event), event))
    assert sounds[0] == AudibleAlert.prompt and sounds[-1] == AudibleAlert.none
    assert len(sounds) < self.entry(getattr(EventNameSP, event), event).duration - 5  # it was interrupted

  def test_other_alerts_keep_their_sound(self):
    ours = self.entry(EventNameSP.stockEcuHandedBack, "notSoundOnce")
    ours.sound_once = False
    sounds = self.run(ours)
    assert sounds[-1] == AudibleAlert.prompt


class TestStockEcuTextFits:
  """mici sizes the first line by length: 12 characters or fewer is one line at its largest font.
  The no-entry's big line there is the action (mici swaps the two), so it must stay that short."""

  def test_actions_are_one_big_line(self):
    from openpilot.sunnypilot.selfdrive.selfdrived.events import STOCK_ECU_ALERT_TEXT
    for _title, line in STOCK_ECU_ALERT_TEXT.values():
      assert len(line) <= 12, line
    for event in (EventNameSP.stockEcuInitializing, EventNameSP.stockEcuReady, EventNameSP.stockEcuHandedBack,
                  EventNameSP.alphaLongNextStop):
      assert len(EVENTS_SP[event][ET.PERMANENT].alert_text_1) <= 12
