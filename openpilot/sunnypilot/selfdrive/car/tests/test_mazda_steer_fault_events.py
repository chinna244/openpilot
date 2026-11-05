"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import custom, log, messaging
from opendbc.car import structs
from opendbc.car.mazda.values import MazdaFlags
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET

EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
EventNameZP = custom.OnroadEventZP.EventName


def _car_events(brand: str, flags: int = 0, sp_flags: int = 0) -> CarSpecificEventsSP:
  CP = structs.CarParams()
  CP.brand = brand
  CP.flags = int(flags)
  CP_SP = structs.CarParamsSP()
  CP_SP.flags = int(sp_flags)
  return CarSpecificEventsSP(CP, CP_SP)


def _car_state_sp():
  return messaging.new_message('carStateSP').carStateSP


def _events(*names: int) -> Events:
  events = Events()
  for name in names:
    events.add(name)
  return events


class TestMazdaSteerFaultEvents:
  """The 2022 EPS non-delivery latch reports through steerFaultTemporary. The latch already
  zeroes the command, so the report is a warning and must not escalate to a soft disable."""

  def test_steer_to_zero_eps_downgrades_to_the_silent_warning(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS)
    events = _events(EventName.steerTempUnavailable)
    car_events.update(structs.CarState(), events, _car_state_sp())
    assert not events.has(EventName.steerTempUnavailable)
    assert events.has(EventName.steerTempUnavailableSilent)
    assert events.contains(ET.WARNING)
    assert not events.contains(ET.SOFT_DISABLE)
    assert not events.contains(ET.NO_ENTRY)

  def test_older_eps_keeps_upstream_escalation(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1)
    events = _events(EventName.steerTempUnavailable)
    car_events.update(structs.CarState(), events, _car_state_sp())
    assert events.has(EventName.steerTempUnavailable)
    assert not events.has(EventName.steerTempUnavailableSilent)
    assert events.contains(ET.SOFT_DISABLE)

  def test_other_brands_are_untouched(self):
    car_events = _car_events('honda', MazdaFlags.STEER_TO_ZERO_EPS)
    events = _events(EventName.steerTempUnavailable)
    car_events.update(structs.CarState(), events, _car_state_sp())
    assert events.has(EventName.steerTempUnavailable)
    assert not events.has(EventName.steerTempUnavailableSilent)

  def test_other_events_survive_the_swap(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS)
    events = _events(EventName.steerTempUnavailable, EventName.steerUnavailable, EventName.steerSaturated)
    car_events.update(structs.CarState(), events, _car_state_sp())
    assert events.has(EventName.steerUnavailable)
    assert events.has(EventName.steerSaturated)
    assert events.has(EventName.steerTempUnavailableSilent)
    assert not events.has(EventName.steerTempUnavailable)

  def test_nothing_to_downgrade_is_a_no_op(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS)
    events = _events(EventName.steerUnavailable)
    car_events.update(structs.CarState(), events, _car_state_sp())
    assert events.names == [EventName.steerUnavailable]


class TestMazdaStockLkasOff:
  """Mazda swaps invalidLkasSetting for the SP stockLkasOff when MADS is on: the selfdrive
  machine still engages, the MADS machine alone refuses lateral. Otherwise nothing is swapped."""

  @staticmethod
  def _cs(invalid: bool) -> structs.CarState:
    CS = structs.CarState()
    CS.invalidLkasSetting = invalid
    return CS

  def test_mads_on_swaps_the_no_entry_for_the_sp_event(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS)
    events = _events(EventName.invalidLkasSetting)
    events_sp = car_events.update(self._cs(True), events, _car_state_sp(), True)
    assert not events.has(EventName.invalidLkasSetting)
    assert not events.contains(ET.NO_ENTRY), "the selfdrive machine must still engage"
    assert not events.contains(ET.PERMANENT), "the banner belongs to the MADS-off path"
    assert events_sp.zp.has(EventNameZP.stockLkasOff)
    assert events_sp.contains(ET.NO_ENTRY)
    assert not events_sp.contains(ET.USER_DISABLE), "MADS pauses on it; a disable would drop the panda's lateral"

  def test_mads_off_keeps_the_whole_system_no_entry(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1)
    events = _events(EventName.invalidLkasSetting)
    events_sp = car_events.update(self._cs(True), events, _car_state_sp(), False)
    assert events.has(EventName.invalidLkasSetting)
    assert events.contains(ET.NO_ENTRY)
    assert events.contains(ET.PERMANENT), "the driver needs the banner to explain the refusal"
    assert not events_sp.zp.has(EventNameZP.stockLkasOff)

  def test_lka_on_swaps_nothing(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1)
    events = _events()
    events_sp = car_events.update(self._cs(False), events, _car_state_sp(), True)
    assert events.names == []
    assert not events_sp.zp.has(EventNameZP.stockLkasOff)


  def test_other_brands_are_untouched(self):
    car_events = _car_events('nissan')
    events = _events(EventName.invalidLkasSetting)
    events_sp = car_events.update(self._cs(True), events, _car_state_sp(), True)
    assert events.has(EventName.invalidLkasSetting)
    assert not events_sp.zp.has(EventNameZP.stockLkasOff)

  def test_eps_arming_raises_the_sp_event_while_mads_is_enabled(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS)
    cs_sp = _car_state_sp()
    cs_sp.zoompilot.lkasArming = True
    events_sp = car_events.update(self._cs(False), _events(), cs_sp, True, True)
    assert events_sp.zp.has(EventNameZP.stockLkasArming)
    assert not events_sp.contains(ET.NO_ENTRY), "lateral has to engage for the EPS to take torque"
    assert not events_sp.contains(ET.USER_DISABLE)
    assert not car_events.update(self._cs(False), _events(), cs_sp, True, False).zp.has(EventNameZP.stockLkasArming)


class TestMazdaCameraFault:
  """Mazda swaps steerUnavailable (the camera's fault) for the ZP cameraLkasFault when MADS is on:
  cruise stays engaged, the MADS machine alone pauses lateral. Otherwise upstream's disable stands."""

  @staticmethod
  def _cs(faulted: bool) -> structs.CarState:
    CS = structs.CarState()
    CS.steerFaultPermanent = faulted
    return CS

  def test_mads_on_keeps_cruise_and_raises_the_zp_event(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS)
    events = _events(EventName.steerUnavailable)
    events_sp = car_events.update(self._cs(True), events, _car_state_sp(), True)
    assert not events.has(EventName.steerUnavailable)
    assert not events.contains(ET.IMMEDIATE_DISABLE), "cruise must not let go on a camera fault"
    assert not events.contains(ET.NO_ENTRY), "cruise must still engage"
    assert events_sp.zp.has(EventNameZP.cameraLkasFault)
    assert events_sp.contains(ET.NO_ENTRY)
    assert not events_sp.contains(ET.IMMEDIATE_DISABLE)
    assert not events_sp.contains(ET.USER_DISABLE), "MADS pauses on it; a disable would not resume"

  def test_mads_off_keeps_upstream_immediate_disable(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1)
    events = _events(EventName.steerUnavailable)
    events_sp = car_events.update(self._cs(True), events, _car_state_sp(), False)
    assert events.has(EventName.steerUnavailable)
    assert events.contains(ET.IMMEDIATE_DISABLE)
    assert not events_sp.zp.has(EventNameZP.cameraLkasFault)

  def test_no_fault_swaps_nothing(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1)
    events = _events()
    events_sp = car_events.update(self._cs(False), events, _car_state_sp(), True)
    assert events.names == []
    assert not events_sp.zp.has(EventNameZP.cameraLkasFault)

  def test_other_brands_are_untouched(self):
    car_events = _car_events('honda')
    events = _events(EventName.steerUnavailable)
    events_sp = car_events.update(self._cs(True), events, _car_state_sp(), True)
    assert events.has(EventName.steerUnavailable)
    assert not events_sp.zp.has(EventNameZP.cameraLkasFault)

  def test_the_event_holds_one_second_past_the_clear(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1)
    clear = car_events.CAMERA_FAULT_CLEAR_FRAMES
    assert car_events.update(self._cs(True), _events(EventName.steerUnavailable), _car_state_sp(), True).zp.has(EventNameZP.cameraLkasFault)
    for _ in range(clear - 1):
      assert car_events.update(self._cs(False), _events(), _car_state_sp(), True).zp.has(EventNameZP.cameraLkasFault)
    assert not car_events.update(self._cs(False), _events(), _car_state_sp(), True).zp.has(EventNameZP.cameraLkasFault)

  def test_a_flicker_restarts_the_hold(self):
    car_events = _car_events('mazda', MazdaFlags.GEN1)
    clear = car_events.CAMERA_FAULT_CLEAR_FRAMES
    car_events.update(self._cs(True), _events(EventName.steerUnavailable), _car_state_sp(), True)
    for _ in range(clear // 2):
      car_events.update(self._cs(False), _events(), _car_state_sp(), True)
    car_events.update(self._cs(True), _events(EventName.steerUnavailable), _car_state_sp(), True)
    for _ in range(clear - 1):
      assert car_events.update(self._cs(False), _events(), _car_state_sp(), True).zp.has(EventNameZP.cameraLkasFault)
