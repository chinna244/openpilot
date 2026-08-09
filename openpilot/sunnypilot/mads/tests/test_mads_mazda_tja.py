# Mazda dedicated TJA-button MADS independence regression tests.

from openpilot.cereal import log, custom
from opendbc.car import structs
from opendbc.car.vin import VIN_UNKNOWN
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem

ButtonType = structs.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName

TJA_VIN = "JM3KFBDM0N0123456"
OTHER_VIN = "JM3KFBCM0N0654321"


def make_car_state(available: bool, button_type=None, pressed=False):
  cs = structs.CarState()
  cs.cruiseState.available = available
  if button_type is not None:
    be = structs.CarState.ButtonEvent()
    be.type = button_type
    be.pressed = pressed
    cs.buttonEvents = [be]
  return cs


def make_mads(mocker, brand="mazda", vin=TJA_VIN, stored_tja_vin=None, prev_available=False):
  sd = mocker.MagicMock()
  sd.CP = structs.CarParams()
  sd.CP.brand = brand
  sd.CP.carVin = vin
  sd.CP_SP = structs.CarParamsSP()

  params = mocker.MagicMock()
  params.get_bool = mocker.MagicMock(side_effect=lambda key: {
    "Mads": True,
    "MadsMainCruiseAllowed": True,
    "DisengageOnAccelerator": True,
    "MadsUnifiedEngagementMode": False,
  }.get(key, False))
  params.get = mocker.MagicMock(side_effect=lambda key, **_: {
    "MazdaTjaButtonVin": stored_tja_vin,
    "MadsSteeringMode": 0,
  }.get(key))
  params.put = mocker.MagicMock()
  sd.params = params

  mocker.patch("openpilot.sunnypilot.mads.mads.Params", return_value=params)

  sd.events = Events()
  sd.events_sp = EventsSP()
  sd.enabled = False
  sd.enabled_prev = False
  sd.initialized = True
  sd.CS_prev = make_car_state(prev_available)
  sd.sm = {"pandaStates": []}

  mads = ModularAssistiveDrivingSystem(sd)
  mads.enabled_toggle = True
  return mads, sd, params


def test_unlearned_mazda_mrcc_on_keeps_existing_mads_behavior(mocker):
  mads, sd, _ = make_mads(mocker, prev_available=False)
  mads.update_events(make_car_state(True))
  assert sd.events_sp.has(EventNameSP.lkasEnable)
  assert not mads.mazda_tja_button_detected


def test_unlearned_mazda_mrcc_off_keeps_existing_mads_behavior(mocker):
  mads, sd, _ = make_mads(mocker, prev_available=True)
  mads.update_events(make_car_state(False))
  assert sd.events_sp.has(EventNameSP.lkasDisable)
  assert not mads.mazda_tja_button_detected


def test_first_tja_press_learns_capability_and_works_with_mrcc_off(mocker):
  mads, sd, params = make_mads(mocker, prev_available=False)
  mads.update_events(make_car_state(False, ButtonType.lkas, True))
  assert mads.mazda_tja_button_detected
  assert sd.events_sp.has(EventNameSP.lkasEnable)
  params.put.assert_called_once_with("MazdaTjaButtonVin", TJA_VIN)



def test_first_tja_press_after_mrcc_legacy_enable_keeps_lateral_enabled(mocker):
  # Before the dedicated TJA button is learned, MRCC availability still uses
  # the legacy behavior and can leave MADS enabled. The first real TJA press
  # must adopt that ON state rather than immediately toggling lateral back off.
  mads, sd, params = make_mads(mocker, prev_available=True)
  mads.enabled = True
  sd.enabled = False

  mads.update_events(make_car_state(True, ButtonType.lkas, True))

  assert mads.mazda_tja_button_detected
  assert not sd.events_sp.has(EventNameSP.lkasDisable)
  assert not sd.events_sp.has(EventNameSP.lkasEnable)
  assert not sd.events_sp.has(EventNameSP.manualSteeringRequired)
  params.put.assert_called_once_with("MazdaTjaButtonVin", TJA_VIN)

def test_persisted_tja_vin_is_detected_at_startup(mocker):
  mads, _, params = make_mads(mocker, stored_tja_vin=TJA_VIN)
  assert mads.mazda_tja_button_detected
  assert mads.mazda_tja_button_vin == TJA_VIN
  params.put.assert_not_called()


def test_different_persisted_vin_does_not_mark_current_car_tja(mocker):
  mads, _, _ = make_mads(mocker, stored_tja_vin=OTHER_VIN)
  assert not mads.mazda_tja_button_detected


def test_unknown_vin_learns_for_session_but_is_not_persisted(mocker):
  mads, sd, params = make_mads(mocker, vin=VIN_UNKNOWN)
  mads.update_events(make_car_state(False, ButtonType.lkas, True))
  assert mads.mazda_tja_button_detected
  assert mads.mazda_tja_button_vin is None
  assert sd.events_sp.has(EventNameSP.lkasEnable)
  params.put.assert_not_called()


def test_detected_tja_mrcc_on_does_not_enable_lateral(mocker):
  mads, sd, _ = make_mads(mocker, stored_tja_vin=TJA_VIN, prev_available=False)
  mads.update_events(make_car_state(True))
  assert not sd.events_sp.has(EventNameSP.lkasEnable)
  assert not sd.events_sp.has(EventNameSP.lkasDisable)


def test_detected_tja_mrcc_off_does_not_disable_lateral_but_cleans_button_enable(mocker):
  mads, sd, _ = make_mads(mocker, stored_tja_vin=TJA_VIN, prev_available=True)
  sd.events.add(EventName.buttonEnable)
  mads.update_events(make_car_state(False))
  assert not sd.events.has(EventName.buttonEnable)
  assert not sd.events_sp.has(EventNameSP.lkasDisable)


def test_detected_tja_button_disables_lateral_with_mrcc_off(mocker):
  mads, sd, _ = make_mads(mocker, stored_tja_vin=TJA_VIN)
  mads.enabled = True
  sd.enabled = False
  mads.update_events(make_car_state(False, ButtonType.lkas, True))
  assert sd.events_sp.has(EventNameSP.lkasDisable)


def test_non_mazda_availability_behavior_is_unchanged(mocker):
  mads, sd, params = make_mads(mocker, brand="toyota", vin="", prev_available=False)
  mads.update_events(make_car_state(True))
  assert sd.events_sp.has(EventNameSP.lkasEnable)
  assert not mads.mazda_tja_button_detected
  assert all(call.args[0] != "MazdaTjaButtonVin" for call in params.get.call_args_list)
