# Mazda single-TJA MADS independence and fail-closed auth handshake.

import sys
import time
from types import ModuleType
from unittest.mock import MagicMock, patch

if "msgq" not in sys.modules:
  _msgq = ModuleType("msgq")
  _msgq.MultiplePublishersError = type("MultiplePublishersError", (Exception,), {})
  _msgq.IpcError = type("IpcError", (Exception,), {})
  for _name in (
    "fake_event_handle", "drain_sock_raw", "Context", "Poller", "SubSocket", "PubSocket",
    "SocketEventHandle", "toggle_fake_events", "set_fake_prefix", "get_fake_prefix",
    "delete_fake_prefix", "wait_for_one_event",
  ):
    setattr(_msgq, _name, MagicMock())
  sys.modules["msgq"] = _msgq

if "openpilot.common.params_pyx" not in sys.modules:
  _params_pyx = ModuleType("openpilot.common.params_pyx")
  class Params:
    def get_bool(self, key):
      return False
    def get(self, key, **kwargs):
      return None
    def put(self, *args, **kwargs):
      return None
    def check_key(self, key):
      return True
  _params_pyx.Params = Params
  _params_pyx.ParamKeyFlag = type("ParamKeyFlag", (), {})
  _params_pyx.ParamKeyType = type("ParamKeyType", (), {})
  _params_pyx.UnknownKeyName = type("UnknownKeyName", (Exception,), {})
  sys.modules["openpilot.common.params_pyx"] = _params_pyx

from openpilot.cereal import log, custom
from opendbc.car import structs
from opendbc.car.mazda.values import MazdaSafetyFlags
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.mads.mads import MAZDA_TJA_FRESH_AUTH_TIMEOUT_S, MAZDA_TJA_PANDA_PRED_MISMATCH_HOLD, ModularAssistiveDrivingSystem

ButtonType = structs.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
SafetyModel = structs.CarParams.SafetyModel
GearShifter = structs.CarState.GearShifter
State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState


def make_car_state(available=False, enabled=False, button_type=None, pressed=False):
  cs = structs.CarState()
  cs.cruiseState.available = available
  cs.cruiseState.enabled = enabled
  if button_type is not None:
    be = structs.CarState.ButtonEvent()
    be.type = button_type
    be.pressed = pressed
    cs.buttonEvents = [be]
  return cs


def make_panda_state(controls_allowed_lateral=True, safety_model=SafetyModel.mazda):
  ps = MagicMock()
  ps.controlsAllowedLateral = controls_allowed_lateral
  ps.safetyModel = safety_model
  return ps


def make_mads(brand="mazda", tja=True, unified=False, prev_available=False,
              panda_lat=False, recv_frame=5):
  sd = MagicMock()
  sd.CP = structs.CarParams()
  sd.CP.brand = brand
  safety_config = structs.CarParams.SafetyConfig()
  safety_config.safetyParam = int(MazdaSafetyFlags.TJA) if tja and brand == "mazda" else 0
  sd.CP.safetyConfigs = [safety_config]
  sd.CP_SP = structs.CarParamsSP()
  sd.CP.passive = False

  params = MagicMock()
  params.get_bool = MagicMock(side_effect=lambda key: {
    "Mads": True,
    "MadsMainCruiseAllowed": True,
    "DisengageOnAccelerator": True,
    "MadsUnifiedEngagementMode": unified,
  }.get(key, False))
  params.get = MagicMock(return_value=0)
  sd.params = params

  sd.events = Events()
  sd.events_sp = EventsSP()
  sd.enabled = False
  sd.enabled_prev = False
  sd.initialized = True
  sd.CS_prev = make_car_state(prev_available)
  sd.sm = MagicMock()
  sd.sm.__getitem__ = MagicMock(return_value=[make_panda_state(panda_lat)])
  sd.sm.recv_frame = {"pandaStates": recv_frame}

  with patch("openpilot.sunnypilot.mads.mads.Params", return_value=params):
    mads = ModularAssistiveDrivingSystem(sd)
  mads.enabled_toggle = True
  return mads, sd


def test_tja_mrcc_on_does_not_enable_mads():
  mads, sd = make_mads(prev_available=False)
  mads.update_events(make_car_state(True))
  assert not sd.events_sp.has(EventNameSP.lkasEnable)


def test_tja_mrcc_off_does_not_disable_mads():
  mads, sd = make_mads(prev_available=True)
  mads.enabled = True
  mads.update_events(make_car_state(False))
  assert not sd.events_sp.has(EventNameSP.lkasDisable)


def test_tja_press_enables_with_mrcc_off():
  mads, sd = make_mads()
  mads.update_events(make_car_state(False, button_type=ButtonType.lkas, pressed=True))
  assert sd.events_sp.has(EventNameSP.lkasEnable)


def test_tja_press_disables_when_enabled():
  mads, sd = make_mads(panda_lat=True)
  mads.enabled = True
  mads.update_events(make_car_state(False, button_type=ButtonType.lkas, pressed=True))
  assert sd.events_sp.has(EventNameSP.lkasDisable)


def test_tja_on_with_mrcc_armed_or_active_enables_mads_only():
  for available, enabled in ((True, False), (True, True)):
    mads, sd = make_mads()
    mads.update_events(make_car_state(available, enabled=enabled, button_type=ButtonType.lkas, pressed=True))
    assert sd.events_sp.has(EventNameSP.lkasEnable)
    assert not sd.events_sp.has(EventNameSP.lkasDisable)


def test_tja_off_with_mrcc_armed_or_active_still_disables_mads():
  for available, enabled in ((True, False), (True, True)):
    mads, sd = make_mads(panda_lat=True)
    mads.enabled = True
    mads.update_events(make_car_state(available, enabled=enabled, button_type=ButtonType.lkas, pressed=True))
    assert sd.events_sp.has(EventNameSP.lkasDisable)


def test_set_res_cancel_do_not_toggle_mads():
  mads, sd = make_mads()
  mads.enabled = True
  for btn in (ButtonType.accelCruise, ButtonType.decelCruise, ButtonType.resumeCruise, ButtonType.cancel):
    sd.events_sp.clear()
    mads.update_events(make_car_state(True, enabled=True, button_type=btn, pressed=True))
    assert not sd.events_sp.has(EventNameSP.lkasEnable)
    assert not sd.events_sp.has(EventNameSP.lkasDisable)


def test_non_tja_mazda_keeps_cruise_coupling():
  mads, sd = make_mads(tja=False, prev_available=False)
  mads.update_events(make_car_state(True))
  assert sd.events_sp.has(EventNameSP.lkasEnable)


def test_uem_does_not_enable_mads_from_longitudinal():
  mads, sd = make_mads(unified=True)
  sd.events.add(EventName.pcmEnable)
  assert mads.block_unified_engagement_mode() is True
  mads.update_events(make_car_state(True, enabled=True))
  assert not sd.events.has(EventName.pcmEnable)


def test_late_fresh_true_after_timeout_does_not_cancel_disable():
  mads, sd = make_mads(panda_lat=False, recv_frame=5)
  mads.enabled = True
  mads._arm_lat_auth_freshness()
  mads._lat_auth_enable_mono = time.monotonic() - (MAZDA_TJA_FRESH_AUTH_TIMEOUT_S + 0.05)
  mads.data_sample()
  assert mads.lateral_auth_lost
  sd.sm.__getitem__ = MagicMock(return_value=[make_panda_state(True)])
  sd.sm.recv_frame = {"pandaStates": 99}
  mads.data_sample()
  assert mads.lateral_auth_lost
  mads.update_events(make_car_state(False))
  assert sd.events_sp.has(EventNameSP.lkasDisable)


def test_stale_panda_false_during_grace_does_not_disable():
  mads, sd = make_mads(panda_lat=False, recv_frame=5)
  mads.enabled = True
  mads._arm_lat_auth_freshness()
  mads.data_sample()
  assert not mads.lateral_auth_lost


def test_fresh_panda_false_after_enable_disables():
  mads, sd = make_mads(panda_lat=False, recv_frame=5)
  mads.enabled = True
  mads._arm_lat_auth_freshness()
  sd.sm.recv_frame = {"pandaStates": 6}
  mads.data_sample()
  assert mads.lateral_auth_lost


def test_two_tja_presses_in_one_update_net_unchanged():
  mads, sd = make_mads()
  cs = make_car_state(False)
  be1 = structs.CarState.ButtonEvent()
  be1.type = ButtonType.lkas
  be1.pressed = True
  be2 = structs.CarState.ButtonEvent()
  be2.type = ButtonType.lkas
  be2.pressed = True
  cs.buttonEvents = [be1, be2]
  mads.update_events(cs)
  assert not sd.events_sp.has(EventNameSP.lkasEnable)
  assert not sd.events_sp.has(EventNameSP.lkasDisable)


def test_three_tja_presses_in_one_update_enables():
  mads, sd = make_mads()
  cs = make_car_state(False)
  presses = []
  for _ in range(3):
    be = structs.CarState.ButtonEvent()
    be.type = ButtonType.lkas
    be.pressed = True
    presses.append(be)
  cs.buttonEvents = presses
  mads.update_events(cs)
  assert sd.events_sp.has(EventNameSP.lkasEnable)


def _press(n=1):
  cs = make_car_state(False)
  presses = []
  for _ in range(n):
    be = structs.CarState.ButtonEvent()
    be.type = ButtonType.lkas
    be.pressed = True
    presses.append(be)
  cs.buttonEvents = presses
  return cs


def test_synced_off_tja_enables():
  mads, sd = make_mads(panda_lat=False)
  mads.enabled = False
  mads.update_events(_press())
  assert sd.events_sp.has(EventNameSP.lkasEnable)
  assert not sd.events_sp.has(EventNameSP.lkasDisable)


def test_synced_on_tja_disables():
  mads, sd = make_mads(panda_lat=True)
  mads.enabled = True
  mads.update_events(_press())
  assert sd.events_sp.has(EventNameSP.lkasDisable)


def test_diverge_us_off_panda_on_tja_does_not_enable():
  # selfdrived restart: MADS OFF, Panda still ON. One physical press turns
  # Panda OFF; userspace must not XOR to ON or the latches invert forever.
  mads, sd = make_mads(panda_lat=True)
  mads.enabled = False
  mads.update_events(_press())
  assert not sd.events_sp.has(EventNameSP.lkasEnable)
  assert not sd.events_sp.has(EventNameSP.lkasDisable)
  assert mads._panda_lat_pred is False


def test_diverge_us_on_panda_off_tja_does_not_disable():
  # Panda restart before MADS observes it: MADS ON, Panda OFF.
  mads, sd = make_mads(panda_lat=False)
  mads.enabled = True
  mads.update_events(_press())
  assert not sd.events_sp.has(EventNameSP.lkasEnable)
  assert not sd.events_sp.has(EventNameSP.lkasDisable)
  assert mads._panda_lat_pred is True


def test_diverge_then_second_press_is_in_sync():
  mads, sd = make_mads(panda_lat=True)
  mads.enabled = False
  mads.update_events(_press())
  sd.events_sp.clear()
  # Predicted panda is now OFF and matches MADS. Next press enables both.
  mads.update_events(_press())
  assert sd.events_sp.has(EventNameSP.lkasEnable)


def test_panda_reconnect_without_tja_disables_mads():
  mads, sd = make_mads(panda_lat=False)
  mads.enabled = True
  sd.sm.alive = {"pandaStates": False}
  mads.data_sample()
  sd.sm.alive = {"pandaStates": True}
  mads.data_sample()
  assert mads._tja_panda_resync
  mads.update_events(make_car_state(False))
  assert sd.events_sp.has(EventNameSP.lkasDisable)
  assert not mads._tja_panda_resync


def test_panda_reconnect_with_tja_stays_enabled():
  # Held-not: a fresh rise after panda reset toggles Panda 0→1. Userspace was ON;
  # skip-disable keeps it ON so both end ON instead of inverted.
  mads, sd = make_mads(panda_lat=False)
  mads.enabled = True
  sd.sm.alive = {"pandaStates": False}
  mads.data_sample()
  sd.sm.alive = {"pandaStates": True}
  mads.data_sample()
  mads.update_events(_press())
  assert not sd.events_sp.has(EventNameSP.lkasDisable)
  assert not sd.events_sp.has(EventNameSP.lkasEnable)
  assert mads._panda_lat_pred is True


def test_controlsd_restart_does_not_manufacture_press():
  mads, sd = make_mads(panda_lat=True)
  mads.enabled = True
  mads.update_events(make_car_state(False))
  assert not sd.events_sp.has(EventNameSP.lkasEnable)
  assert not sd.events_sp.has(EventNameSP.lkasDisable)


def test_panda_health_before_button_event_still_enables():
  # Route 0000001a event 1: pandaStates latched ON ~11 ms before CarState
  # buttonEvents. That health sample must not poison pred into a diverge skip.
  mads, sd = make_mads(panda_lat=False)
  mads.update_events(make_car_state(False))
  sd.sm.__getitem__ = MagicMock(return_value=[make_panda_state(True)])
  mads.update_events(make_car_state(False))
  sd.events_sp.clear()
  mads.update_events(_press())
  assert sd.events_sp.has(EventNameSP.lkasEnable)
  assert not sd.events_sp.has(EventNameSP.lkasDisable)


def test_stable_panda_leftover_without_tja_still_diverges():
  mads, sd = make_mads(panda_lat=True)
  mads.enabled = False
  for _ in range(MAZDA_TJA_PANDA_PRED_MISMATCH_HOLD):
    mads.update_events(make_car_state(False))
  sd.events_sp.clear()
  mads.update_events(_press())
  assert not sd.events_sp.has(EventNameSP.lkasEnable)
  assert not sd.events_sp.has(EventNameSP.lkasDisable)


def _park_cs(pressed=True):
  cs = make_car_state(False, button_type=ButtonType.lkas if pressed else None, pressed=pressed)
  cs.gearShifter = GearShifter.park
  cs.standstill = True
  cs.vEgo = 0.0
  return cs


def test_park_tja_toggles_enabled_paused_not_active():
  mads, sd = make_mads(panda_lat=False)
  sd.events.add(EventName.wrongGear)
  mads.update(_park_cs(True))
  assert mads.enabled is True
  assert mads.active is False
  assert mads.state_machine.state == State.paused


def test_park_second_tja_disables_enabled():
  mads, sd = make_mads(panda_lat=False)
  sd.events.add(EventName.wrongGear)
  mads.update(_park_cs(True))
  assert mads.enabled is True
  sd.events_sp.clear()
  sd.events.add(EventName.wrongGear)
  mads.update(_park_cs(True))
  assert mads.enabled is False
  assert mads.active is False
  assert mads.state_machine.state == State.disabled


def test_tja_with_non_gear_no_entry_still_enables_paused():
  mads, sd = make_mads(panda_lat=False)
  sd.events.add(EventName.invalidLkasSetting)
  mads.update(_press())
  assert mads.enabled is True
  assert mads.active is False
  assert mads.state_machine.state == State.paused


def test_park_to_drive_stays_paused_until_no_entry_clears():
  mads, sd = make_mads(panda_lat=False)
  sd.events.add(EventName.wrongGear)
  mads.update(_park_cs(True))
  assert mads.state_machine.state == State.paused
  sd.events_sp.clear()
  sd.events.clear()
  drive = make_car_state(False)
  drive.gearShifter = GearShifter.drive
  mads.update(drive)
  assert mads.enabled is True
  assert mads.active is True
  assert mads.state_machine.state == State.enabled


def test_route_0000001a_event1_park_panda_race_enables_paused():
  # Recorded 0000001a t=16.706: Park, MRCC OFF, MADS OFF, physical TJA.
  # pandaStates latched ~11 ms before CarState buttonEvents.
  # Logical enabled must toggle; active/latActive stay false.
  mads, sd = make_mads(panda_lat=False)
  sd.events.add(EventName.wrongGear)
  mads.update_events(make_car_state(False))
  sd.sm.__getitem__ = MagicMock(return_value=[make_panda_state(True)])
  mads.update(_park_cs(False))
  sd.events.add(EventName.wrongGear)
  sd.events_sp.clear()
  mads.update(_park_cs(True))
  assert mads.enabled is True
  assert mads.active is False
  assert mads.state_machine.state == State.paused


def test_generated_gear_brake_tja_enabled_toggle():
  gears = (GearShifter.park, GearShifter.reverse, GearShifter.neutral, GearShifter.drive)
  for gear in gears:
    for standstill in (True, False):
      for brake in (True, False):
        for parking_brake in (True, False):
          mads, sd = make_mads(panda_lat=False)
          if gear != GearShifter.drive:
            sd.events.add(EventName.wrongGear)
          if gear == GearShifter.reverse:
            sd.events.add(EventName.reverseGear)
          if parking_brake:
            sd.events.add(EventName.parkBrake)
          cs = _press()
          cs.gearShifter = gear
          cs.standstill = standstill
          cs.brakePressed = brake
          mads.update(cs)
          assert mads.enabled is True, (gear, standstill, brake, parking_brake)
          can_actuate = gear == GearShifter.drive and not parking_brake
          if can_actuate:
            assert mads.active is True, (gear, standstill, brake, parking_brake)
          else:
            assert mads.active is False, (gear, standstill, brake, parking_brake)
          sd.events_sp.clear()
          if gear != GearShifter.drive:
            sd.events.add(EventName.wrongGear)
          if gear == GearShifter.reverse:
            sd.events.add(EventName.reverseGear)
          if parking_brake:
            sd.events.add(EventName.parkBrake)
          mads.update(cs)
          assert mads.enabled is False, (gear, standstill, brake, parking_brake)
          assert mads.active is False, (gear, standstill, brake, parking_brake)
