"""Mazda TJA-button MADS/MRCC chime behavior on current upstream ownership.

Scoped through MazdaFlagsSP.TJA_BUTTON / button_owns_lateral — not the old TJA_MADS stack.
"""

from opendbc.car import structs
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP
from openpilot.cereal import custom, log
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import ET
from openpilot.sunnypilot.mads.state import State
from openpilot.sunnypilot.mads.tests.test_mads_main_cruise_off_switch import make_mads
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP, EngagementAlert, AudibleAlert

EventNameSP = custom.OnroadEventSP.EventName
ButtonType = structs.CarState.ButtonEvent.Type
LogAudible = log.SelfdriveState.AudibleAlert


def _restore_sp_event_maps():
  # test_mads_state_machine.make_event mutates EVENTS_SP[0] (lkasEnable). Restore before chime asserts.
  EVENTS_SP[EventNameSP.lkasEnable] = {ET.ENABLE: EngagementAlert(AudibleAlert.engage)}
  EVENTS_SP[EventNameSP.manualSteeringRequired] = {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  }
  EVENTS_SP[EventNameSP.longitudinalEnableChime] = {
    ET.PERMANENT: EngagementAlert(AudibleAlert.engage),
  }
  EVENTS_SP[EventNameSP.longitudinalDisableChime] = {
    ET.PERMANENT: EngagementAlert(AudibleAlert.disengage),
  }


def car_state(available=False, lkas_pressed=False):
  cs = structs.CarState()
  cs.cruiseState.available = available
  if lkas_pressed:
    be = structs.CarState.ButtonEvent()
    be.type = ButtonType.lkas
    be.pressed = True
    cs.buttonEvents = [be]
  return cs


class TestMazdaTjaMadsChimes(OpenpilotTestCase):
  def _mads(self, tja=True, alpha_long=False):
    _restore_sp_event_maps()
    mads, sd = make_mads(self._fixture("mocker"), "mazda", True,
                         sp_flags=MazdaFlagsSP.TJA_BUTTON if tja else 0)
    if alpha_long:
      # Current Mazda Alpha Long mechanism: CarParams.openpilotLongitudinalControl.
      sd.CP.alphaLongitudinalAvailable = True
      sd.CP.openpilotLongitudinalControl = True
    return mads, sd

  def test_event_ordinals(self):
    assert int(EventNameSP.longitudinalEnableChime) == 27
    assert int(EventNameSP.longitudinalDisableChime) == 28

  def test_mads_enable_chime_while_longitudinal_active(self):
    mads, sd = self._mads()
    sd.enabled = True
    mads.state_machine.state = State.disabled
    sd.state_machine.current_alert_types = []
    sd.events_sp.add(EventNameSP.lkasEnable)

    mads.state_machine.update()

    assert mads.state_machine.state == State.enabled
    assert ET.ENABLE in sd.state_machine.current_alert_types
    enable_alerts = sd.events_sp.create_alerts(sd.state_machine.current_alert_types)
    assert any(a.alert_type == "lkasEnable/enable" and a.audible_alert == LogAudible.engage
               for a in enable_alerts)

  def test_mads_disable_chime_while_longitudinal_active(self):
    mads, sd = self._mads()
    sd.enabled = True
    mads.state_machine.state = State.enabled
    mads.enabled = True
    sd.state_machine.current_alert_types = []
    sd.events_sp.add(EventNameSP.manualSteeringRequired)

    mads.state_machine.update()

    assert mads.state_machine.state == State.disabled
    assert ET.USER_DISABLE in sd.state_machine.current_alert_types
    disable_alerts = sd.events_sp.create_alerts(sd.state_machine.current_alert_types)
    assert any(a.alert_type == "manualSteeringRequired/userDisable" and a.audible_alert == LogAudible.disengage
               for a in disable_alerts)

  def test_longitudinal_chimes_do_not_toggle_mads(self):
    mads, sd = self._mads()
    mads.enabled = False
    assert mads.button_owns_lateral

    sd.enabled_prev = False
    sd.enabled = True
    mads.update_events(car_state())
    assert not mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalEnableChime)
    enable_alerts = sd.events_sp.create_alerts([ET.PERMANENT])
    assert any(a.alert_type == "longitudinalEnableChime/permanent" and a.audible_alert == LogAudible.engage
               for a in enable_alerts)

    sd.events_sp.clear()
    sd.enabled_prev = True
    sd.enabled = False
    mads.update_events(car_state())
    assert not mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalDisableChime)
    disable_alerts = sd.events_sp.create_alerts([ET.PERMANENT])
    assert any(a.alert_type == "longitudinalDisableChime/permanent" and a.audible_alert == LogAudible.disengage
               for a in disable_alerts)

  def test_longitudinal_chimes_preserve_enabled_mads(self):
    mads, sd = self._mads()
    mads.enabled = True

    sd.enabled_prev = False
    sd.enabled = True
    mads.update_events(car_state())
    assert mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalEnableChime)

    sd.events_sp.clear()
    sd.enabled_prev = True
    sd.enabled = False
    mads.update_events(car_state())
    assert mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalDisableChime)

  def test_undeclared_skips_longitudinal_chimes(self):
    mads, sd = self._mads(tja=False)
    assert not mads.button_owns_lateral
    sd.enabled_prev = False
    sd.enabled = True
    mads.update_events(car_state(available=True))
    assert not sd.events_sp.has(EventNameSP.longitudinalEnableChime)

  def test_set_res_cancel_do_not_toggle_mads_when_button_owns(self):
    mads, sd = self._mads()
    mads.enabled = True
    for btn in (ButtonType.setCruise, ButtonType.resumeCruise, ButtonType.cancel,
                ButtonType.accelCruise, ButtonType.decelCruise):
      cs = car_state(available=True)
      be = structs.CarState.ButtonEvent()
      be.type = btn
      be.pressed = True
      cs.buttonEvents = [be]
      sd.events_sp.clear()
      mads.update_events(cs)
      assert mads.enabled
      assert not sd.events_sp.has(EventNameSP.lkasDisable)
      assert not sd.events_sp.has(EventNameSP.lkasEnable)

  def test_alpha_long_chimes_do_not_toggle_mads(self):
    # A1: Alpha Long + TJA_BUTTON, MADS OFF — long OFF->ON / ON->OFF via mads.update().
    mads, sd = self._mads(alpha_long=True)
    assert mads.CP.alphaLongitudinalAvailable
    assert mads.CP.openpilotLongitudinalControl
    assert mads.button_owns_lateral
    mads.enabled = False
    mads.state_machine.state = State.disabled

    sd.enabled_prev = False
    sd.enabled = True
    mads.update(car_state())
    assert not mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalEnableChime)
    enable_alerts = sd.events_sp.create_alerts([ET.PERMANENT])
    assert any(a.alert_type == "longitudinalEnableChime/permanent" and a.audible_alert == LogAudible.engage
               for a in enable_alerts)

    # Level hold: already active longitudinal must not re-chime.
    sd.events_sp.clear()
    mads.update(car_state())
    assert not mads.enabled
    assert not sd.events_sp.has(EventNameSP.longitudinalEnableChime)
    assert not sd.events_sp.has(EventNameSP.longitudinalDisableChime)

    sd.events_sp.clear()
    sd.enabled = False
    mads.update(car_state())
    assert not mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalDisableChime)
    disable_alerts = sd.events_sp.create_alerts([ET.PERMANENT])
    assert any(a.alert_type == "longitudinalDisableChime/permanent" and a.audible_alert == LogAudible.disengage
               for a in disable_alerts)

  def test_alpha_long_chimes_preserve_enabled_mads(self):
    # A2: Alpha Long + TJA_BUTTON, MADS ON — long OFF->ON / ON->OFF via mads.update().
    mads, sd = self._mads(alpha_long=True)
    assert mads.CP.alphaLongitudinalAvailable
    assert mads.CP.openpilotLongitudinalControl
    assert mads.button_owns_lateral
    mads.enabled = True
    mads.state_machine.state = State.enabled

    sd.enabled_prev = False
    sd.enabled = True
    mads.update(car_state())
    assert mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalEnableChime)

    sd.events_sp.clear()
    mads.update(car_state())
    assert mads.enabled
    assert not sd.events_sp.has(EventNameSP.longitudinalEnableChime)
    assert not sd.events_sp.has(EventNameSP.longitudinalDisableChime)

    sd.events_sp.clear()
    sd.enabled = False
    mads.update(car_state())
    assert mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalDisableChime)

  def test_mrcc_available_only_no_longitudinal_chime(self):
    # Chimes key off selfdrive.enabled edges, not cruiseState.available / MRCC armed.
    mads, sd = self._mads()
    mads.enabled = False
    mads.state_machine.state = State.disabled
    sd.enabled = False
    sd.enabled_prev = False
    sd.CS_prev = car_state(available=False)

    mads.update(car_state(available=True))
    assert not mads.enabled
    assert not sd.events_sp.has(EventNameSP.longitudinalEnableChime)
    assert not sd.events_sp.has(EventNameSP.longitudinalDisableChime)

    sd.events_sp.clear()
    sd.CS_prev = car_state(available=True)
    mads.update(car_state(available=True))
    assert not mads.enabled
    assert not sd.events_sp.has(EventNameSP.longitudinalEnableChime)
    assert not sd.events_sp.has(EventNameSP.longitudinalDisableChime)

  def test_set_res_while_long_active_no_extra_chime(self):
    # ON->ON: SET/RES/speed adjust must not emit another longitudinal engage chime.
    mads, sd = self._mads()
    mads.enabled = True
    sd.enabled = True
    sd.enabled_prev = True
    for btn in (ButtonType.setCruise, ButtonType.resumeCruise,
                ButtonType.accelCruise, ButtonType.decelCruise):
      cs = car_state(available=True)
      be = structs.CarState.ButtonEvent()
      be.type = btn
      be.pressed = True
      cs.buttonEvents = [be]
      sd.events_sp.clear()
      mads.update_events(cs)
      assert sd.enabled
      assert mads.enabled
      assert not sd.events_sp.has(EventNameSP.longitudinalEnableChime)
      assert not sd.events_sp.has(EventNameSP.longitudinalDisableChime)
