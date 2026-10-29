from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from openpilot.common.params import Params
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.car.mazda.aol import create_intent, policy_for
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences


def params_cp(alpha=False):
  return CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], alpha, False, False)


def test_tja_preference_is_bound_before_controller_construction():
  cp = params_cp(True)
  configured = VehicleStartupPreferences(mazda_tja=True).prepare(cp)
  assert configured.safetyConfigs[0].safetyParam == 11
  ci = CarInterface(configured)
  assert ci.CP_SP.flags & 1
  assert policy_for(configured).runtime_supported
  assert not policy_for(configured).cruise_main_required


def test_tja_switch_owns_lateral_without_arming_mrcc():
  cp = VehicleStartupPreferences(mazda_tja=True).prepare(params_cp())
  owner = create_intent(cp, AolSettings(True, 0., 0, 0, (0, 0, 0), (0, 0, 0)))
  cs = structs.CarState(canValid=True, vEgo=15., gearShifter=structs.CarState.GearShifter.drive)
  owner.update(cs, now_ns=1_000_000_000, fault_active=False)
  cs.buttonEvents = [structs.CarState.ButtonEvent(type=structs.CarState.ButtonEvent.Type.lkas, pressed=True)]
  owner.update(cs, now_ns=1_010_000_000, fault_active=False)
  assert owner.output(cs)[0]
  assert not cs.cruiseState.available
  cs.buttonEvents[0].pressed = False
  owner.update(cs, now_ns=1_020_000_000, fault_active=False)
  cs.buttonEvents[0].pressed = True
  owner.update(cs, now_ns=1_030_000_000, fault_active=False)
  assert not owner.output(cs)[0]


def test_galaxy_and_native_share_parked_tja_setting(tmp_path):
  from openpilot.starpilot.galaxy.settings import AuthorityContext, ContextSource, SettingsChanged, SettingsGateway
  params = Params(str(tmp_path/'params'))
  state = SimpleNamespace(parked=True, cp=params_cp())
  state.raw = state.cp.to_bytes()
  gateway = SettingsGateway(params, Mock(spec=ContextSource, sample=lambda: AuthorityContext(state.parked, state.cp, state.raw)), clock=lambda: 10)
  try:
    page = gateway.page('vehicle', 'session', b'generation')
    index = next(i for i, row in enumerate(page['rows']) if row['label'] == 'Mazda TJA Button')
    intent = gateway.preview(page['view'], index, 0, 'session', b'generation', value='On')
    assert gateway.confirm(intent['intent'], 'session', b'generation')
    assert params.get_bool('MazdaTjaButton')
    assert params.get_bool('OnroadCycleRequested')
    page = gateway.page('vehicle', 'session', b'generation')
    index = next(i for i, row in enumerate(page['rows']) if row['label'] == 'Mazda TJA Button')
    intent = gateway.preview(page['view'], index, 0, 'session', b'generation', value='Off')
    state.parked = False
    with pytest.raises(SettingsChanged):
      gateway.confirm(intent['intent'], 'session', b'generation')
    assert params.get_bool('MazdaTjaButton')
  finally:
    gateway.close()


def test_unmapped_on_rearms_lateral_without_toggling_an_active_latch():
  cp = params_cp()
  owner = create_intent(cp, AolSettings(True, 0., 0, 0, (0, 0, 0), (0, 0, 0)))
  cs = structs.CarState(canValid=True, vEgo=15., gearShifter=structs.CarState.GearShifter.drive)
  cs.cruiseState.available = True
  owner.update(cs, now_ns=1_000_000_000, fault_active=False)
  cs.buttonEvents = [structs.CarState.ButtonEvent(type=structs.CarState.ButtonEvent.Type.mainCruise, pressed=True)]
  owner.update(cs, now_ns=1_010_000_000, fault_active=False)
  assert owner.allowed_latch
  cs.buttonEvents[0].pressed = False
  owner.update(cs, now_ns=1_020_000_000, fault_active=False)
  cs.buttonEvents[0].pressed = True
  owner.update(cs, now_ns=1_030_000_000, fault_active=False)
  assert owner.allowed_latch
