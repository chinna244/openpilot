"""Mazda through the real Card: the native owner's raw LKAS_BLOCK pause reaches the published CarState only
while AOL (the native lateral owner) is active; without it the source's delivery semantics stand."""
import gc
from unittest.mock import patch

import pytest

from opendbc.car import DT_CTRL, car_helpers, gen_empty_fingerprint, structs
from opendbc.car.mazda.fingerprints import FW_VERSIONS
from opendbc.car.mazda.tests.test_mazda_golden_tx import D
from opendbc.car.mazda.tests.test_mazda_production_eps import DeliveringSimCar
from opendbc.car.mazda.values import CAR, STEER_TO_ZERO_EPS_FW
from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.car import card as card_module
from openpilot.selfdrive.car.card import Car

Ecu = structs.CarParams.Ecu


def cx5_2022_fw():
  fw = []
  for (ecu, address, sub), versions in FW_VERSIONS[CAR.MAZDA_CX5_2022].items():
    version = next(v for v in versions if v in STEER_TO_ZERO_EPS_FW) if ecu == Ecu.eps else versions[0]
    fw.append(structs.CarParams.CarFw(ecu=ecu, address=address, subAddress=sub or 0, fwVersion=version, brand="mazda"))
  return fw


@pytest.fixture(params=[True, False], ids=["aol", "no_aol"])
def mazda_card(request, monkeypatch):
  monkeypatch.setenv("SIMULATION", "1")
  monkeypatch.setenv("FINGERPRINT", "")
  monkeypatch.setenv("SKIP_FW_QUERY", "")
  monkeypatch.setenv("DISABLE_FW_CACHE", "1")
  fw = cx5_2022_fw()
  card = None
  with OpenpilotPrefix():
    saved = Params()
    for key, value in (("OpenpilotEnabledToggle", True), ("AlwaysOnLateral", request.param),
                       ("AlphaLongitudinalEnabled", False), ("IsReleaseBranch", False)):
      saved.put_bool(key, value, block=True)
    try:
      with patch.object(card_module.messaging, "recv_one_retry", return_value=messaging.new_message("can", 1)), \
           patch.object(card_module, "can_comm_callbacks", return_value=(lambda *a, **k: [], lambda frames: None)), \
           patch.object(card_module, "obd_callback", return_value=lambda enabled: None), \
           patch.object(car_helpers, "get_vin", return_value=(-1, -1, "0" * 17)), \
           patch.object(car_helpers, "get_present_ecus", return_value=set()), \
           patch.object(car_helpers, "get_fw_versions_ordered", return_value=fw), \
           patch.object(car_helpers, "can_fingerprint", return_value=(None, gen_empty_fingerprint())):
        card = Car()
      yield card
    finally:
      del card
      gc.collect()


def bus_frames(sim, frames, **state):
  sim.state.update(dict(v=15., gear=D, brake=False), **state)
  for _ in range(frames):
    yield int(sim.frame * DT_CTRL * 1e9), sim.can()
    sim.frame += 1


def test_raw_block_reaches_carstate_only_with_the_native_owner(mazda_card):
  card = mazda_card
  aol = card.CP.alternativeExperience == 32
  assert card.CP.carFingerprint == CAR.MAZDA_CX5_2022
  assert card.aol_qualified == aol and (card.aol_card_intent is not None) == aol

  sim = DeliveringSimCar(delivered=0.4)
  for t, frames in bus_frames(sim, 100):
    state = card.CI.update([(t, frames)])
  assert not card.CI.CS.lkas_blocked and not state.steerFaultTemporary
  for t, frames in bus_frames(sim, 50, lkas_block=True):
    state = card.CI.update([(t, frames)])
    assert state.steerFaultTemporary == aol
  for t, frames in bus_frames(sim, 2, lkas_block=False):
    state = card.CI.update([(t, frames)])
  assert not card.CI.CS.lkas_blocked and not state.steerFaultTemporary


@pytest.mark.parametrize("word", [2, 3, 10, 11])
def test_native_owner_withholds_lateral_on_the_frame_card_reads(word):
  # The STEER_RATE frame card's view reads: mazda_aol_permission drops lateral on it and restores it after.
  from opendbc.safety.tests.test_mazda_stock_aol import TestMazdaStockAol
  native = TestMazdaStockAol()
  native.configure(word=word)
  native.safety.set_aol_test_heartbeat(False)
  for _ in range(20):
    native.time += 10_000
    native.safety.set_timer(native.time)
    native.feed()
    native.safety.aol_set_host_request(0)
  if word & 1:
    native.safety.safety_rx_hook(native.packet('CRZ_BTNS', {'RES': 1}))
  name, fields = ('PEDALS', {'ACC_ACTIVE': 1}) if word & 1 else ('CRZ_CTRL', {'CRZ_AVAILABLE': 1, 'CRZ_ACTIVE': 1})
  native.safety.safety_rx_hook(native.packet(name, fields))
  native.time += 10_000
  native.safety.set_timer(native.time)
  native.safety.set_aol_test_heartbeat(True)
  native.safety.aol_set_host_request(1)
  assert native.safety.aol_get_permission_mask() & 1
  native.safety.safety_rx_hook(native.packet('STEER_RATE', {'LKAS_BLOCK': 1}))
  assert not native.safety.aol_get_permission_mask() & 1
  native.safety.safety_rx_hook(native.packet('STEER_RATE', {'LKAS_BLOCK': 0}))
  assert native.safety.aol_get_permission_mask() & 1
