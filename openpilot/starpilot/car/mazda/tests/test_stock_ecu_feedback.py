"""Mazda alpha long: a failed radar return and a SET/RES press the car will not engage on reach the driver
through selfdrived's own events, and a failed radar is never described as openpilot's."""
from types import SimpleNamespace

import pytest

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.stock_ecu import StockEcuState
from opendbc.car.mazda.values import CAR
from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, Events, STOCK_ECU_PRESS_STATE
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.starpilot.car.mazda.radar_status import notice, stock_ecu_events
from openpilot.starpilot.car.stock_ecu_handback import (FAILED_KEY, HANDBACK_WAIT_T, StockEcuHandBackGate,
                                                        clear_handback_failure_on_ignition_off)
from openpilot.starpilot.car.tests.handback_fakes import FakeClock, FakeParams
from openpilot.starpilot.ui.onroad_state import OnroadAlert

EventName = log.OnroadEvent.EventName
ButtonType = structs.CarState.ButtonEvent.Type
NOW = 10_000_000_000


def cp(alpha_long=True, candidate=CAR.MAZDA_CX5_2022):
  return CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha_long, False, False)


def record(state, radar_off=False, stamp=NOW, fingerprint=str(CAR.MAZDA_CX5_2022)):
  return dict(fingerprint=fingerprint, state=str(state), radarOff=radar_off, alphaRequested=True, monoTime=stamp)


def press(button=ButtonType.setCruise, pressed=True):
  return [structs.CarState.ButtonEvent(pressed=pressed, type=button)]


def test_failed_radar_is_never_shown_as_openpilot_ownership():
  alert = notice(OnroadAlert(), cp(), record(StockEcuState.FAILED, radar_off=True), False, NOW)
  assert 'failed' in alert.text1.lower() and 'restart' in alert.text2.lower()
  assert 'control' not in (alert.text1 + alert.text2).lower()


@pytest.mark.parametrize("state,named", [
  (StockEcuState.STARTING, StockEcuState.STARTING), (StockEcuState.PARK_TO_TAKE_OVER, StockEcuState.PARK_TO_TAKE_OVER),
  (StockEcuState.RESTORING, StockEcuState.RESTORING), (StockEcuState.FAILED, StockEcuState.FAILED),
  (StockEcuState.READY, None), (StockEcuState.RESTORED, None), (StockEcuState.NOT_NEEDED, None),
  (StockEcuState.STOCK_CRUISE_ON, None),
])
def test_set_press_names_why_the_car_will_not_engage(state, named):
  assert stock_ecu_events(cp(), record(state), False, press(), NOW)[1] == named
  # no press, no press alert; a release is not a press
  assert stock_ecu_events(cp(), record(state), False, [], NOW)[1] is None
  assert stock_ecu_events(cp(), record(state), False, press(pressed=False), NOW)[1] is None


@pytest.mark.parametrize("data", [None, record(StockEcuState.READY, stamp=NOW - 3_000_000_001),
                                  record(StockEcuState.READY, fingerprint=str(CAR.MAZDA_CX9_2021))])
def test_stale_or_foreign_status_is_unknown_not_ready(data):
  assert stock_ecu_events(cp(), data, False, press(ButtonType.resumeCruise), NOW) == (False, 'unknown')


def test_only_mazda_alpha_long_gets_stock_ecu_events():
  assert stock_ecu_events(cp(alpha_long=False), record(StockEcuState.FAILED), True, press(), NOW) == (False, None)


def test_selfdrived_adds_the_display_events_from_its_params_view():
  sd = SimpleNamespace(CP=cp(), events=Events(), mazda_stock_ecu=None, mazda_handback_failed=True)
  sd.mazda_stock_ecu = record(StockEcuState.STARTING)
  sd.mazda_stock_ecu['monoTime'] = 1  # stale on purpose: still unknown, failure flag still shown
  SelfdriveD.add_mazda_stock_ecu_events(sd, SimpleNamespace(buttonEvents=press()))
  assert EventName.stockEcuFailed in sd.events.names and EventName.stockEcuNotReady in sd.events.names
  assert STOCK_ECU_PRESS_STATE[0] == 'unknown'


@pytest.mark.parametrize("event", [EventName.stockEcuFailed, EventName.stockEcuNotReady])
def test_alerts_are_display_only(event):
  # PERMANENT: shown to a driver with nothing engaged, and no enable/disable/no-entry authority
  assert set(EVENTS[event]) == {ET.PERMANENT}
  STOCK_ECU_PRESS_STATE[0] = StockEcuState.PARK_TO_TAKE_OVER
  alert = EVENTS[event][ET.PERMANENT]
  alert = alert(cp(), None, None, False, 0, None) if callable(alert) else alert
  assert alert.alert_text_1


def _timed_out_gate(**values):
  params, clock = FakeParams(**values), FakeClock()
  gate = StockEcuHandBackGate(params, now=clock)
  assert not gate.ready(True)
  clock.t = HANDBACK_WAIT_T + 1
  assert gate.ready(True)
  return params


def test_unanswered_stop_with_the_radar_off_leaves_failure_evidence():
  params = _timed_out_gate(MazdaStockEcuStatus=record(StockEcuState.READY, radar_off=True))
  assert params.get_bool(FAILED_KEY)
  # card's last state alone, its status record gone
  assert _timed_out_gate(MazdaStockEcuState=str(StockEcuState.READY)).get_bool(FAILED_KEY)


def test_unanswered_stop_before_card_ran_control_is_not_a_failure():
  # no status at all: card never ran controls_update, the only place a takeover is commanded
  assert not _timed_out_gate().get_bool(FAILED_KEY)


@pytest.mark.parametrize("state", [StockEcuState.PARK_TO_TAKE_OVER, StockEcuState.RESTORED, StockEcuState.STARTING])
def test_unanswered_stop_with_the_radar_on_leaves_none(state):
  assert not _timed_out_gate(MazdaStockEcuStatus=record(state, radar_off=False)).get_bool(FAILED_KEY)


def _panda(type_=log.PandaState.PandaType.redPanda, ignition=False):
  return SimpleNamespace(pandaType=type_, ignitionLine=ignition, ignitionCan=False)


def test_failure_evidence_lasts_until_a_known_panda_sees_ignition_off():
  params = FakeParams(**{FAILED_KEY: True})
  clear_handback_failure_on_ignition_off(params, [])  # boot, no panda yet
  clear_handback_failure_on_ignition_off(params, [_panda(log.PandaState.PandaType.unknown)])
  clear_handback_failure_on_ignition_off(params, [_panda(ignition=True)])  # reboot with the ignition on
  assert params.get_bool(FAILED_KEY)
  clear_handback_failure_on_ignition_off(params, [_panda(ignition=False)])
  assert not params.get_bool(FAILED_KEY)
