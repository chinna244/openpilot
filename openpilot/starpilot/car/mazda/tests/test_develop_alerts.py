"""Develop's Mazda alerts under StarPilot: the camera's fault and the car's lane keep switched off pause TJA steering
and keep cruise, the radar takeover is said once per edge, and the MADS banners follow TJA and cancel."""
from types import SimpleNamespace

import pytest
from opendbc.car import structs
from opendbc.car.mazda.stock_ecu import StockEcuState
from openpilot.cereal import log
from openpilot.selfdrive.car.car_events import CarEvents
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, Events
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.starpilot.aol.runtime import AxisDecision
from openpilot.starpilot.car.mazda.aol import create_intent
from openpilot.starpilot.car.mazda.stock_intent import CAMERA_FAULT_CLEAR_FRAMES
from openpilot.starpilot.car.mazda.tests.test_tja_only_lateral import SETTINGS, driving, press, tja_cp

EventName = log.OnroadEvent.EventName
ButtonType = structs.CarState.ButtonEvent.Type
NEW = (EventName.cameraLkasFault, EventName.cameraLkasFaultOnset, EventName.cameraLkasResumed, EventName.stockLkasOff,
       EventName.stockLkasOffPress, EventName.stockEcuInitializing, EventName.stockEcuReady, EventName.stockEcuHandedBack,
       EventName.alphaLongNextStop, EventName.manualSteeringRequired, EventName.manualLongitudinalRequired,
       EventName.cameraLkasFaultPress)


@pytest.mark.parametrize('event', NEW)
def test_new_alerts_are_display_only(event):
  # a NO_ENTRY would also withdraw AOL's longitudinal axis (aol_no_entry); none may disable anything
  assert set(EVENTS[event]) == {ET.PERMANENT}


def car_events(cs, cp=None):
  ce = CarEvents(cp or tja_cp())
  return ce, ce.update(cs, structs.CarState(), SimpleNamespace())


def test_lkas_off_pauses_lateral_without_blocking_cruise():
  cs = driving()
  cs.invalidLkasSetting = True
  _, events = car_events(cs)
  assert EventName.stockLkasOff in events.names and EventName.invalidLkasSetting not in events.names
  assert not events.contains(ET.NO_ENTRY)


def test_camera_fault_pauses_and_holds_one_second_past_its_clear():
  cs = driving()
  cs.steerFaultPermanent = True
  ce, events = car_events(cs)
  assert EventName.cameraLkasFault in events.names and EventName.steerUnavailable not in events.names
  assert not events.contains(ET.IMMEDIATE_DISABLE)
  cs.steerFaultPermanent = False
  for frame in range(1, CAMERA_FAULT_CLEAR_FRAMES + 1):
    events = ce.update(cs, cs, SimpleNamespace())
    assert (EventName.cameraLkasFault in events.names) == (frame < CAMERA_FAULT_CLEAR_FRAMES)


def test_without_the_tja_button_upstream_faults_stand():
  cs = driving()
  cs.steerFaultPermanent = cs.invalidLkasSetting = True
  _, events = car_events(cs, tja_cp(alpha=False, tja=False))
  assert EventName.steerUnavailable in events.names and EventName.invalidLkasSetting in events.names
  assert EventName.cameraLkasFault not in events.names


def test_sustained_non_delivery_is_silent_on_the_steer_to_zero_eps():
  ce = CarEvents(tja_cp())
  cs = driving()
  cs.steerFaultTemporary = True
  ce.silent_steer_warning, ce.steering_unpressed = False, 1000
  events = ce.update(cs, cs, SimpleNamespace())
  assert EventName.steerTempUnavailableSilent in events.names and EventName.steerTempUnavailable not in events.names


def test_intent_keeps_the_latch_through_camera_fault_and_lkas_off():
  owner = create_intent(tja_cp(), SETTINGS)
  cs = driving()
  owner.update(owner.observe_lateral_holds(cs), now_ns=1_000_000_000, fault_active=False)
  press(owner, cs, ButtonType.lkas, 1_100_000_000)
  assert owner.allowed_latch and not owner.pause_lateral
  cs.steerFaultPermanent = True
  view = owner.observe_lateral_holds(cs)
  assert not view.steerFaultPermanent and cs.steerFaultPermanent  # the AOL owners see a pause, the wire keeps it
  owner.update(view, now_ns=1_300_000_000, fault_active=False)
  assert owner.allowed_latch and owner.pause_lateral
  cs.steerFaultPermanent = False
  for i in range(CAMERA_FAULT_CLEAR_FRAMES):
    owner.update(owner.observe_lateral_holds(cs), now_ns=1_400_000_000 + i * 10_000_000, fault_active=False)
    assert owner.pause_lateral == (i < CAMERA_FAULT_CLEAR_FRAMES - 1)
  assert owner.allowed_latch
  cs.invalidLkasSetting = True
  owner.update(owner.observe_lateral_holds(cs), now_ns=3_000_000_000, fault_active=False)
  assert owner.allowed_latch and owner.pause_lateral
  # TJA still turns it off while paused
  cs.buttonEvents = [structs.CarState.ButtonEvent(type=ButtonType.lkas, pressed=True)]
  owner.update(owner.observe_lateral_holds(cs), now_ns=3_100_000_000, fault_active=False)
  assert not owner.allowed_latch and owner.pause_lateral


def selfdrived(latched=True, enabled=True):
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.CP = tja_cp()
  sd.events = Events()
  sd.enabled = enabled
  sd.aol_last_intent = SimpleNamespace(allowedLatch=latched)
  sd.aol_axis_decision = AxisDecision()
  sd.mazda_camera_fault_prev = sd.mazda_camera_paused = False
  sd.mazda_stock_state_prev, sd.mazda_radar_off_prev, sd.mazda_alpha_announced = None, False, False
  return sd


def buttons(*types):
  return [structs.CarState.ButtonEvent(type=t, pressed=True) for t in types]


def test_tja_and_cancel_banners():
  sd = selfdrived()
  sd.add_mazda_tja_events(SimpleNamespace(buttonEvents=buttons(ButtonType.lkas), invalidLkasSetting=False))
  assert EventName.manualSteeringRequired in sd.events.names
  sd = selfdrived(enabled=False)  # TJA off without cruise: the AOL chime alone
  sd.add_mazda_tja_events(SimpleNamespace(buttonEvents=buttons(ButtonType.lkas), invalidLkasSetting=False))
  assert EventName.manualSteeringRequired not in sd.events.names
  sd = selfdrived(latched=False)
  sd.add_mazda_tja_events(SimpleNamespace(buttonEvents=buttons(ButtonType.lkas), invalidLkasSetting=True))
  assert EventName.stockLkasOffPress in sd.events.names
  sd = selfdrived()
  sd.add_mazda_tja_events(SimpleNamespace(buttonEvents=buttons(ButtonType.cancel), invalidLkasSetting=False))
  assert EventName.manualLongitudinalRequired in sd.events.names


def test_camera_fault_onset_and_resumed():
  sd = selfdrived()
  cs = SimpleNamespace(buttonEvents=[], invalidLkasSetting=False)
  sd.events.add(EventName.cameraLkasFault)
  sd.add_mazda_tja_events(cs)
  assert EventName.cameraLkasFaultOnset in sd.events.names
  sd.events.clear()
  sd.events.add(EventName.cameraLkasFault)
  sd.add_mazda_tja_events(cs)
  assert EventName.cameraLkasFaultOnset not in sd.events.names  # once
  sd.events.clear()
  sd.add_mazda_tja_events(cs)  # cleared, lateral not back yet
  assert EventName.cameraLkasResumed not in sd.events.names
  sd.aol_axis_decision = AxisDecision(desired_lateral=True, lateral_active=True)
  sd.add_mazda_tja_events(cs)
  assert EventName.cameraLkasResumed in sd.events.names
  sd.events.clear()
  sd.add_mazda_tja_events(cs)
  assert EventName.cameraLkasResumed not in sd.events.names


def test_takeover_lines_are_said_once_per_edge(monkeypatch):
  import openpilot.selfdrive.selfdrived.selfdrived as module
  monkeypatch.setattr(module.time, 'monotonic_ns', lambda: 10_000_000_000)
  sd = selfdrived()
  def step(state, standstill, radar_off=False, requested=False, aeb_setting_off=False):
    sd.events.clear()
    sd.mazda_stock_ecu = dict(fingerprint=str(sd.CP.carFingerprint), state=str(state), radarOff=radar_off,
                              alphaRequested=requested, aebSettingOff=aeb_setting_off, monoTime=10_000_000_000)
    sd.add_mazda_takeover_events(SimpleNamespace(standstill=standstill))
    return set(sd.events.names)
  assert EventName.alphaLongNextStop in step(StockEcuState.PARK_TO_TAKE_OVER, False, requested=True)
  assert EventName.alphaLongNextStop not in step(StockEcuState.PARK_TO_TAKE_OVER, False, requested=True)
  assert EventName.stockEcuInitializing in step(StockEcuState.STARTING, True, requested=True)
  assert EventName.stockEcuReady in step(StockEcuState.READY, True, radar_off=True)
  assert EventName.stockEcuReady not in step(StockEcuState.READY, False, radar_off=True)
  assert EventName.stockEcuHandedBack in step(StockEcuState.RESTORED, False)
  step(StockEcuState.READY, False, radar_off=True)
  assert EventName.stockEcuHandedBack not in step(StockEcuState.RESTORED, False, aeb_setting_off=True)


def test_tja_cannot_turn_steering_on_through_the_camera_fault():
  owner = create_intent(tja_cp(), SETTINGS)
  cs = driving()
  cs.steerFaultPermanent = True
  owner.update(owner.observe_lateral_holds(cs), now_ns=1_000_000_000, fault_active=False)
  for t in (1_100_000_000, 1_300_000_000):
    cs.buttonEvents = [structs.CarState.ButtonEvent(type=ButtonType.lkas, pressed=True)]
    owner.update(owner.observe_lateral_holds(cs), now_ns=t, fault_active=False)
    cs.buttonEvents = [structs.CarState.ButtonEvent(type=ButtonType.lkas, pressed=False)]
    owner.update(owner.observe_lateral_holds(cs), now_ns=t + 10_000_000, fault_active=False)
    assert not owner.allowed_latch
  sd = selfdrived(latched=False)
  sd.events.add(EventName.cameraLkasFault)
  sd.add_mazda_tja_events(SimpleNamespace(buttonEvents=buttons(ButtonType.lkas), invalidLkasSetting=False))
  assert EventName.cameraLkasFaultPress in sd.events.names


def test_lkas_off_refuse_only_when_turning_on():
  sd = selfdrived(latched=True, enabled=False)  # turning it off is allowed and silent here (AOL chime)
  sd.add_mazda_tja_events(SimpleNamespace(buttonEvents=buttons(ButtonType.lkas), invalidLkasSetting=True))
  assert EventName.stockLkasOffPress not in sd.events.names


def test_konik_backup_stands_down_when_konik_is_the_active_provider(monkeypatch):
  from openpilot.system.manager import process_config as pc
  import openpilot.starpilot.connect.provider as provider
  params = SimpleNamespace(get_bool=lambda k: k == 'KonikUploadsEnabled', get=lambda k: 'abc123')
  monkeypatch.setattr(provider, 'active_provider', lambda: provider.PROVIDERS['comma'])
  assert pc.konik_uploads_ready(False, params, None) and pc.konik_connect_ready(False, params, None)
  assert not pc.konik_uploads_ready(True, params, None)
  monkeypatch.setattr(provider, 'active_provider', lambda: provider.PROVIDERS['konik'])
  assert not pc.konik_uploads_ready(False, params, None) and not pc.konik_connect_ready(False, params, None)
