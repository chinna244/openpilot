"""Mazda radar hand-back across a stop, through the real Card: its hand-back server and params polling,
the CarInterface on a closed-loop simulated bus (the radar answers its UDS sessions), the consumer gate
hardwared and manager use, and manager's own offroad/onroad Params clears."""
import gc
from unittest.mock import patch

import pytest

from opendbc.car import DT_CTRL, car_helpers, gen_empty_fingerprint, structs
from opendbc.car.mazda.carcontroller import RADAR_RESTART_FRAMES
from opendbc.car.mazda.stock_ecu import StockEcuState
from opendbc.car.mazda.tests.conftest import RADAR_UDS, SESSION_PROG_DAT, car_control, split_inputs
from opendbc.car.mazda.tests.test_mazda_golden_tx import PARKED, STACK_CC, SimCar, _takeover
from openpilot.cereal import messaging
from openpilot.common.params import Params, ParamKeyFlag
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as card_module
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.car.mazda.tests.test_mazda_card_native_pause import cx5_2022_fw
from openpilot.starpilot.car.stock_ecu_handback import REQUEST_KEY, StockEcuHandBackGate, mazda_handback_required, read_record

PARAMS_THREAD_FRAMES = 10  # card's params_thread runs at 10 Hz


@pytest.fixture
def alpha_card(monkeypatch):
  monkeypatch.setenv("SIMULATION", "1")
  monkeypatch.setenv("FINGERPRINT", "")
  monkeypatch.setenv("SKIP_FW_QUERY", "")
  monkeypatch.setenv("DISABLE_FW_CACHE", "1")
  card = None
  with OpenpilotPrefix():
    params = Params()
    for key, value in (("OpenpilotEnabledToggle", True), ("AlwaysOnLateral", False),
                       ("AlphaLongitudinalEnabled", True), ("IsReleaseBranch", False)):
      params.put_bool(key, value, block=True)
    try:
      with patch.object(card_module.messaging, "recv_one_retry", return_value=messaging.new_message("can", 1)), \
           patch.object(card_module, "can_comm_callbacks", return_value=(lambda *a, **k: [], lambda frames: None)), \
           patch.object(card_module, "obd_callback", return_value=lambda enabled: None), \
           patch.object(car_helpers, "get_vin", return_value=(-1, -1, "0" * 17)), \
           patch.object(car_helpers, "get_present_ecus", return_value=set()), \
           patch.object(car_helpers, "get_fw_versions_ordered", return_value=cx5_2022_fw()), \
           patch.object(car_helpers, "can_fingerprint", return_value=(None, gen_empty_fingerprint())):
        card = Car()
      assert card.CP.openpilotLongitudinalControl and mazda_handback_required(params)
      yield card, params
    finally:
      del card
      gc.collect()


class Drive:
  def __init__(self, card):
    self.card, self.sim, self.programming = card, SimCar(), 0

  def step(self, frames, base=None, per_frame=None):
    for i in range(frames):
      self.sim.state.update({k: v for k, v in (base or {}).items() if k in self.sim.state})
      kwargs = dict(STACK_CC, **{k: v for k, v in (base or {}).items() if k not in self.sim.state})
      for k, v in (per_frame(i, self.sim) if per_frame is not None else {}).items():
        if k in self.sim.state:
          self.sim.state[k] = v
        else:
          kwargs[k] = v
      cc_kw, _, _ = split_inputs(kwargs)
      t = int(self.sim.frame * DT_CTRL * 1e9)
      if self.sim.frame % PARAMS_THREAD_FRAMES == 0:
        self.card.mazda_handback_server().update_params()
      self.card.CI.update([(t, self.sim.can())])
      cc = car_control(**cc_kw)
      self.card.mazda_handback_update(cc)
      _, sends = self.card.CI.apply(cc, t)
      self.programming += sum(addr == RADAR_UDS and bytes(dat) == SESSION_PROG_DAT for addr, dat, _ in sends)
      self.sim.react(sends, [True] * len(sends))

  def take_radar(self):
    for _, n, base, per_frame in _takeover(150)[:-1]:
      self.step(n, base, per_frame)
    assert self.card.CI.CC.stock_ecu_off and self.programming

  def stop_until_restored(self, params, gate):
    for _ in range(2000):
      self.step(1, PARKED)
      if gate.ready(True):
        break
    assert self.card.CI.CC.stock_ecu_state == StockEcuState.RESTORED  # stock_ecu_off still counts the restart window


def test_restored_radar_stays_restored_through_the_offroad_clear(alpha_card):
  card, params = alpha_card
  drive = Drive(card)
  drive.take_radar()
  drive.stop_until_restored(params, StockEcuHandBackGate(params))

  # manager's offroad transition runs while card is still alive (it is stopped later in the same loop)
  params.clear_all(ParamKeyFlag.CLEAR_ON_OFFROAD_TRANSITION)
  assert read_record(params, REQUEST_KEY) is not None
  programming = drive.programming
  drive.step(RADAR_RESTART_FRAMES + 100, PARKED)  # past the restored radar's restart window
  assert drive.programming == programming, "a restored radar was taken over again during the stop"
  assert card.CI.CC.stock_ecu_state == StockEcuState.RESTORED and not card.CI.CC.stock_ecu_off

  # the next onroad transition starts the following session fresh
  params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
  assert read_record(params, REQUEST_KEY) is None


def test_withdrawn_stop_releases_the_radar_to_the_driver(alpha_card):
  # A stop withdrawn while onroad (not forced offroad, no reboot/cycle) removes the request: card stops
  # asserting and the parked, main-on, cruise-requested car may take the radar again.
  card, params = alpha_card
  drive = Drive(card)
  drive.take_radar()
  gate = StockEcuHandBackGate(params)
  drive.stop_until_restored(params, gate)
  gate.ready(True)  # a second stop opens a new request on the settled one
  assert gate.pending
  gate.reset()
  assert read_record(params, REQUEST_KEY) is None
  programming = drive.programming
  drive.step(PARAMS_THREAD_FRAMES * 2, PARKED)
  for _, n, base, per_frame in _takeover(150)[2:5]:
    drive.step(n, base, per_frame)
  assert drive.programming > programming
