"""Mazda Smart CAN Filter on the StarPilot port (develop mazda-smart-can-filter-dev): the driver's choice reaches the
panda's safety word and the radar, and selfdrived says what develop's car_specific / events_zp said."""
from types import SimpleNamespace

import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.stock_ecu import StockEcuState
from opendbc.car.mazda.values import CAR, MazdaFlags, MazdaSafetyFlags
from openpilot.selfdrive.selfdrived.events import ET, MAZDA_EVENTS, Events, MazdaEvents
from openpilot.selfdrive.selfdrived.selfdrived import MAZDA_LOST_ENGAGED_FRAMES, SelfdriveD
from openpilot.starpilot.car.mazda.radar_status import stock_ecu_events
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences

FILTER = int(MazdaSafetyFlags.CAN_FILTER)
NOW = 10_000_000_000


def cp(enabled=True, candidate=CAR.MAZDA_CX5_2022, alpha=True):
  return VehicleStartupPreferences(mazda_can_filter=enabled).prepare(
    CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha, False, False))


@pytest.mark.parametrize('enabled,candidate,alpha,on', [
  (True, CAR.MAZDA_CX5_2022, True, True),
  (False, CAR.MAZDA_CX5_2022, True, False),
  (True, CAR.MAZDA_CX5_2022, False, False),
  (True, CAR.MAZDA_CX9_2021, True, False),
])
def test_choice_reaches_the_safety_word_and_the_radar(enabled, candidate, alpha, on):
  p = cp(enabled, candidate, alpha)
  assert bool(p.safetyConfigs[0].safetyParam & FILTER) == on
  if on:
    assert not p.radarUnavailable
    assert CarInterface.get_params_sp(p, candidate, gen_empty_fingerprint(), [], alpha_long=alpha).flags & 4  # MazdaFlagsSP.CAN_FILTER
  # a second prepare with the choice off clears the bit (a branch switch keeps CarParams)
  assert not VehicleStartupPreferences().prepare(p).safetyConfigs[0].safetyParam & FILTER


def test_g46l_radar_stays_vision_only():
  base = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], True, False, False)
  base.flags |= MazdaFlags.G46L_RADAR.value
  base.radarUnavailable = True
  p = VehicleStartupPreferences(mazda_can_filter=True).prepare(base)
  assert p.safetyConfigs[0].safetyParam & FILTER and p.radarUnavailable


@pytest.mark.parametrize('name', ['cameraTjaStuck', 'stockEcuLost', 'stockEcuLostIdle', 'radarLostCameraOnly'])
def test_filter_alerts_are_display_only(name):
  assert set(MAZDA_EVENTS[name]) == {ET.PERMANENT}


def selfdrived(enabled=True):
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.CP = cp(enabled)
  sd.events = Events()
  sd.mazda_events = MazdaEvents()
  sd.mazda_stock_state_prev, sd.mazda_radar_off_prev, sd.mazda_alpha_announced = None, False, False
  return sd


def step(sd, state, standstill=False, cruise=False, radar_off=False, lost=False, stuck=False, monkeypatch=None):
  sd.mazda_events.clear()
  sd.mazda_stock_ecu = {'fingerprint': str(sd.CP.carFingerprint), 'state': str(state), 'radarOff': radar_off,
                        'alphaRequested': False, 'aebSettingOff': False, 'stockEcuLost': lost, 'cameraTjaStuck': stuck,
                        'monoTime': NOW}
  cs = SimpleNamespace(standstill=standstill, cruiseState=SimpleNamespace(enabled=cruise))
  sd.add_mazda_takeover_events(cs)
  return set(sd.mazda_events.names)


@pytest.fixture(autouse=True)
def clock(monkeypatch):
  import openpilot.selfdrive.selfdrived.selfdrived as module
  monkeypatch.setattr(module.time, 'monotonic_ns', lambda: NOW)


def test_online_radar_needs_no_stop_and_is_not_aeb_off():
  sd = selfdrived()
  assert 'stockEcuInitializing' not in step(sd, StockEcuState.STARTING, standstill=True)
  # ready with the radar online: AEB is on, so no "AEB off"
  assert 'stockEcuReady' not in step(sd, StockEcuState.READY)
  # without the filter the takeover still says both
  sd = selfdrived(enabled=False)
  assert 'stockEcuInitializing' in step(sd, StockEcuState.STARTING, standstill=True)
  assert 'stockEcuReady' in step(sd, StockEcuState.READY, radar_off=True)


def test_loss_is_loud_when_engaged_and_said_once():
  sd = selfdrived()
  step(sd, StockEcuState.READY, cruise=True)
  assert 'stockEcuLost' in step(sd, StockEcuState.READY, cruise=False, lost=True)
  assert not {'stockEcuLost', 'stockEcuLostIdle'} & step(sd, StockEcuState.READY, lost=True)


def test_loss_while_not_engaged_is_the_quiet_line():
  sd = selfdrived()
  for _ in range(MAZDA_LOST_ENGAGED_FRAMES + 1):
    step(sd, StockEcuState.STARTING)
  assert step(sd, StockEcuState.STARTING, lost=True) & {'stockEcuLost', 'stockEcuLostIdle'} == {'stockEcuLostIdle'}


def test_camera_tja_stuck_is_said_while_it_lasts():
  sd = selfdrived()
  assert 'cameraTjaStuck' in step(sd, StockEcuState.READY, stuck=True)
  assert 'cameraTjaStuck' not in step(sd, StockEcuState.READY)


def test_set_press_before_the_board_confirms_is_silent():
  press = [structs.CarState.ButtonEvent(type=structs.CarState.ButtonEvent.Type.setCruise, pressed=True)]
  record = {'fingerprint': str(CAR.MAZDA_CX5_2022), 'state': str(StockEcuState.STARTING), 'radarOff': False,
            'alphaRequested': False, 'monoTime': NOW}
  assert stock_ecu_events(cp(True), record, False, press, NOW) == (False, None)
  assert stock_ecu_events(cp(False), record, False, press, NOW)[1] == StockEcuState.STARTING


@pytest.mark.parametrize("tja", [False, True])
def test_filter_keeps_mads_lateral(tja):
  """develop keeps MADS under the filter: the filter bit must not cost AOL its admission, the native latch or the
  panda's alternative experience."""
  from openpilot.starpilot.car.mazda.aol import native_accepts_cp, policy_for
  results = {}
  for enabled in (False, True):
    p = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], True, False, False)
    p.fingerprintSource = structs.CarParams.FingerprintSource.fw
    p = VehicleStartupPreferences(mazda_can_filter=enabled, mazda_tja=tja).prepare(p)
    word = p.safetyConfigs[0].safetyParam
    assert bool(word & MazdaSafetyFlags.CAN_FILTER) == enabled
    policy = policy_for(p)
    p.alternativeExperience = policy.alternative_experience_addition
    results[enabled] = (policy.intent_supported, policy.explicit_latch, policy.alternative_experience_addition,
                        policy.cruise_main_required, native_accepts_cp(p, int(structs.CarParams.SafetyModel.mazda), word))
  assert results[True] == results[False] == (True, True, 32, not tja, True)


def test_filter_bit_without_long_is_not_admitted():
  from opendbc.car.mazda.stock_aol import qualified
  p = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], False, False, False)
  p.fingerprintSource = structs.CarParams.FingerprintSource.fw
  assert qualified(p)
  p.safetyConfigs[0].safetyParam |= int(MazdaSafetyFlags.CAN_FILTER)
  assert not qualified(p)


def test_stock_cruise_edges_chime_sound_only():
  sd = selfdrived()
  step(sd, StockEcuState.READY)
  assert 'longitudinalEnableChime' in step(sd, StockEcuState.STOCK_CRUISE_ON)
  assert not {'longitudinalEnableChime', 'longitudinalDisableChime'} & step(sd, StockEcuState.STOCK_CRUISE_ON)
  assert 'longitudinalDisableChime' in step(sd, StockEcuState.READY)
  step(sd, StockEcuState.STOCK_CRUISE_ON)
  assert 'longitudinalDisableChime' not in step(sd, StockEcuState.RESTORED)  # the end-of-drive hand-back
  for name in ('longitudinalEnableChime', 'longitudinalDisableChime'):
    assert set(MAZDA_EVENTS[name]) == {ET.PERMANENT}


@pytest.mark.parametrize('arming,latched,shown', [(True, True, True), (True, False, False), (False, True, False)])
def test_lkas_arming_while_lateral_latched(arming, latched, shown):
  sd = selfdrived()
  sd.aol_last_intent = SimpleNamespace(allowedLatch=latched)
  sd.mazda_events.clear()
  sd.mazda_stock_ecu = {'fingerprint': str(sd.CP.carFingerprint), 'state': str(StockEcuState.READY), 'radarOff': False,
                        'alphaRequested': False, 'aebSettingOff': False, 'stockEcuLost': False, 'cameraTjaStuck': False,
                        'lkasArming': arming, 'monoTime': NOW}
  sd.add_mazda_takeover_events(SimpleNamespace(standstill=False, cruiseState=SimpleNamespace(enabled=False)))
  assert ('stockLkasArming' in sd.mazda_events) == shown


def test_radar_messages_chime_once_after_being_covered():
  from openpilot.cereal import log
  from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
  from openpilot.selfdrive.selfdrived.events import EngagementAlert
  none = log.SelfdriveState.AudibleAlert.none
  ready = MazdaEvents()
  ready.add('stockEcuReady')
  ready_alert = ready.create_alerts([ET.PERMANENT])[0]
  ready_alert.duration = 300
  cover = EngagementAlert(log.SelfdriveState.AudibleAlert.engage)
  cover.alert_type, cover.event_type, cover.priority = 'cover/permanent', ET.PERMANENT, ready_alert.priority + 1
  am = AlertManager()
  am.add_many(0, [ready_alert])
  am.process_alerts(1, set())
  assert am.current_alert.audible_alert != none
  am.add_many(1, [cover])
  am.process_alerts(2, set())
  assert am.current_alert is cover
  am.process_alerts(250, set())  # the cover is gone; the radar message comes back silent
  assert am.current_alert.alert_type == ready_alert.alert_type and am.current_alert.audible_alert == none
  assert ready_alert.audible_alert != none  # the shared alert keeps its chime for the next time
