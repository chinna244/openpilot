"""Mazda TJA-only MADS toggle: physical TJA rising edge toggles MADS; MRCC does not."""

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem

ButtonType = structs.CarState.ButtonEvent.Type
SafetyModel = structs.CarParams.SafetyModel


def _car_interface():
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=True,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, fingerprint, [],
                                     alpha_long=True, is_release_sp=False, docs=False)
  return CarInterface(CP, CP_SP)


def _make_mads(mocker, CP, CP_SP):
  sd = mocker.MagicMock()
  sd.CP = CP
  sd.CP_SP = CP_SP
  sd.params = mocker.MagicMock()
  sd.params.get_bool = mocker.MagicMock(side_effect=lambda k: {
    "Mads": True,
    "MadsMainCruiseAllowed": True,  # default ON; Mazda must ignore it
    "DisengageOnAccelerator": True,
    "MadsUnifiedEngagementMode": False,
  }.get(k, False))
  sd.params.get = mocker.MagicMock(return_value=MadsSteeringModeOnBrake.REMAIN_ACTIVE)
  sd.events = Events()
  sd.events_sp = EventsSP()
  sd.enabled = False
  sd.enabled_prev = False
  sd.initialized = True
  sd.passive = False
  sd.CP.passive = False
  sd.CS_prev = structs.CarState()
  ps = mocker.MagicMock()
  ps.controlsAllowedLateral = True
  ps.safetyModel = SafetyModel.mazda
  sd.sm = {"pandaStates": [ps]}
  sd.state_machine = mocker.MagicMock()
  sd.state_machine.current_alert_types = []
  mads = ModularAssistiveDrivingSystem(sd)
  mads.enabled_toggle = True
  return mads, sd


class TjaMadsHarness:
  def __init__(self, mocker):
    self.ci = _car_interface()
    self.mads, self.sd = _make_mads(mocker, self.ci.CP, self.ci.CP_SP)
    self.packer = CANPacker("mazda_2017")
    self.t = 0

  def step(self, *, tja=0, mrcc=0, acc_off=0, acc_active=0, set_p=0, set_m=0,
           res=0, can_off=0):
    self.t += 10_000_000
    addr, dat, bus = self.packer.make_can_msg("CRZ_BTNS", 0, {
      "TJA_BUTTON": tja,
      "SET_P": set_p,
      "SET_M": set_m,
      "RES": res,
      "CAN_OFF": can_off,
      "BIT1": 1,
      "BIT2": 1,
      "BIT3": 1,
    })
    if mrcc:
      dat = bytes((dat[0], dat[1] | 0x80, *dat[2:]))
    crz = (addr, dat, bus)
    pedals = self.packer.make_can_msg("PEDALS", 0, {
      "ACC_OFF": acc_off,
      "ACC_ACTIVE": acc_active,
      "BRAKE_ON": 0,
    })
    cs, _ = self.ci.update([(self.t, [crz, pedals])])
    self.sd.events.clear()
    self.sd.events_sp.clear()
    self.mads.update(cs)
    self.sd.CS_prev = cs
    return cs


class TestMazdaTjaMads:
  def test_1_tja_rising_edge_enables_mads(self, mocker):
    h = TjaMadsHarness(mocker)
    h.step()
    assert not h.mads.enabled
    h.step(tja=1)
    assert h.mads.enabled

  def test_2_tja_held_is_single_toggle(self, mocker):
    h = TjaMadsHarness(mocker)
    h.step()
    h.step(tja=1)
    assert h.mads.enabled
    for _ in range(5):
      h.step(tja=1)
      assert h.mads.enabled

  def test_3_tja_release_does_not_toggle(self, mocker):
    h = TjaMadsHarness(mocker)
    h.step()
    h.step(tja=1)
    h.step(tja=0)
    assert h.mads.enabled

  def test_4_second_tja_rising_edge_disables_mads(self, mocker):
    h = TjaMadsHarness(mocker)
    h.step()
    h.step(tja=1)
    h.step(tja=0)
    h.step(tja=1)
    assert not h.mads.enabled

  def test_5_one_mads_transition_per_rising_edge(self, mocker):
    h = TjaMadsHarness(mocker)
    h.step()
    expected = False
    transitions = 0
    for tja in (1, 0, 1, 0, 1, 0, 1, 0):
      prev = h.mads.enabled
      h.step(tja=tja)
      if tja == 1:
        expected = not expected
        transitions += 1
      assert h.mads.enabled is expected
      if tja == 0:
        assert h.mads.enabled is prev
    assert transitions == 4

  def test_6_mrcc_with_mads_off_stays_off(self, mocker):
    h = TjaMadsHarness(mocker)
    h.step()
    h.step(mrcc=1, acc_off=1)
    assert not h.mads.enabled
    h.step(acc_off=1)
    assert not h.mads.enabled

  def test_7_mrcc_with_mads_on_stays_on(self, mocker):
    h = TjaMadsHarness(mocker)
    h.step()
    h.step(tja=1)
    h.step(tja=0)
    assert h.mads.enabled
    h.step(mrcc=1, acc_off=1)
    assert h.mads.enabled
    h.step(acc_off=0)
    assert h.mads.enabled

  def test_8_to_11_set_res_cancel_do_not_toggle_mads(self, mocker):
    h = TjaMadsHarness(mocker)
    h.step()
    h.step(set_p=1)
    h.step()
    h.step(set_m=1)
    h.step()
    h.step(res=1)
    h.step()
    h.step(can_off=1)
    h.step()
    assert not h.mads.enabled

    h.step(tja=1)
    h.step(tja=0)
    assert h.mads.enabled
    h.step(set_p=1, acc_off=1)
    h.step(acc_off=1)
    h.step(set_m=1, acc_off=1)
    h.step(acc_off=1)
    h.step(res=1, acc_off=1)
    h.step(acc_off=1)
    h.step(can_off=1, acc_off=1)
    h.step(acc_off=1)
    assert h.mads.enabled

  def test_12_13_tja_does_not_fabricate_cruise_state(self, mocker):
    h = TjaMadsHarness(mocker)
    h.step()
    cs = h.step(tja=1)
    assert h.mads.enabled
    assert not cs.cruiseState.available
    assert not cs.cruiseState.enabled
    h.step(tja=0)
    armed = h.step(acc_off=1)
    assert armed.cruiseState.available
    assert not armed.cruiseState.enabled
    tja_armed = h.step(tja=1, acc_off=1)
    assert tja_armed.cruiseState.available
    assert not tja_armed.cruiseState.enabled
    assert not h.mads.enabled

  def test_14_mrcc_oem_cruise_unchanged(self, mocker):
    h = TjaMadsHarness(mocker)
    h.step()
    armed = h.step(mrcc=1, acc_off=1)
    assert armed.cruiseState.available
    assert not armed.cruiseState.enabled
    assert not h.mads.enabled
    active = h.step(acc_off=1, acc_active=1)
    assert active.cruiseState.available
    assert active.cruiseState.enabled
    assert not h.mads.enabled
