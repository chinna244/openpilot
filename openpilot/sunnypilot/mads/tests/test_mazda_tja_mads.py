"""Mazda TJA-only MADS toggle: physical TJA rising edge toggles MADS; MRCC does not."""

import random

import pytest

pytestmark = pytest.mark.xdist_group("mazda_tja_mads")

from opendbc.can import CANPacker
from opendbc.car import DT_CTRL, gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, CarControllerParams
from openpilot.cereal import custom, log
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake, set_car_specific_params
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem

ButtonType = structs.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
SafetyModel = structs.CarParams.SafetyModel

# Same CRZ_BTNS TJA run-length as opendbc test_mazda_tja_button (route
# ff7df7d6f9c3403b|00000033--7b0201ce40). One physical pulse = one MADS toggle.
ROUTE_TJA_RLE = (
  (0, 217), (1, 3), (0, 115), (1, 3), (0, 4), (1, 3), (0, 50), (1, 2), (0, 4), (1, 3),
  (0, 18), (1, 2), (0, 30), (1, 3), (0, 102), (1, 3), (0, 8), (1, 3), (0, 12), (1, 3),
  (0, 19), (1, 3), (0, 5), (1, 3), (0, 10), (1, 3), (0, 286), (1, 3), (0, 5), (1, 2),
  (0, 18), (1, 3), (0, 12), (1, 3), (0, 33), (1, 3), (0, 6), (1, 3), (0, 6), (1, 3),
  (0, 6), (1, 3), (0, 10), (1, 4), (0, 13), (1, 3), (0, 16), (1, 3), (0, 23), (1, 3),
  (0, 14), (1, 3), (0, 8), (1, 4), (0, 22),
)
ROUTE_TJA_RISING_EDGES = 27


def _car_interface(*, alpha_long=True):
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=alpha_long,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, fingerprint, [],
                                     alpha_long=alpha_long, is_release_sp=False, docs=False)
  assert CP.openpilotLongitudinalControl == alpha_long
  return CarInterface(CP, CP_SP)


def _make_mads(mocker, CP, CP_SP, *, uem=True):
  sd = mocker.MagicMock()
  sd.CP = CP
  sd.CP_SP = CP_SP
  sd.params = mocker.MagicMock()
  sd.params.get_bool = mocker.MagicMock(side_effect=lambda k: {
    "Mads": True,
    "MadsMainCruiseAllowed": True,  # default ON; Mazda must ignore it
    "DisengageOnAccelerator": True,
    "MadsUnifiedEngagementMode": uem,  # default ON in params; Mazda must force off
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
  def __init__(self, mocker, *, alpha_long=True):
    self.ci = _car_interface(alpha_long=alpha_long)
    self.mads, self.sd = _make_mads(mocker, self.ci.CP, self.ci.CP_SP)
    self.packer = CANPacker("mazda_2017")
    self.t = 0
    self._tja = 0
    if alpha_long:
      for _ in range(int(CarControllerParams.STOCK_RADAR_GUARD_T / DT_CTRL) + 1):
        self.step()

  def _cam_msgs(self):
    lkas = self.packer.make_can_msg("CAM_LKAS", 0, {
      "ERR_BIT_1": 0, "ERR_BIT_2": 0, "LINE_NOT_VISIBLE": 0, "BIT_1": 1,
    })
    lane = self.packer.make_can_msg("CAM_LANEINFO", 0, {
      "LANE_LINES": 1, "NO_ERR_BIT": 0, "ERR_BIT": 0, "BIT2": 0,
    })
    return [(lkas[0], lkas[1], 2), (lane[0], lane[1], 2)]

  def step(self, *, tja=0, acc_off=0, acc_active=0, set_p=0, set_m=0,
           res=0, can_off=0, crz_available=0, crz_active=0, mode_x=0, mode_y=0):
    self.t += 10_000_000
    addr, dat, bus = self.packer.make_can_msg("CRZ_BTNS", 0, {
      "TJA_BUTTON": tja,
      "SET_P": set_p,
      "SET_M": set_m,
      "RES": res,
      "CAN_OFF": can_off,
      "MODE_X": mode_x,
      "MODE_Y": mode_y,
      "BIT1": 1,
      "BIT2": 1,
      "BIT3": 1,
    })
    msgs = [(addr, dat, bus), self.packer.make_can_msg("PEDALS", 0, {
      "ACC_OFF": acc_off,
      "ACC_ACTIVE": acc_active,
      "BRAKE_ON": 0,
    })]
    msgs.extend(self._cam_msgs())
    if not self.ci.CP.openpilotLongitudinalControl:
      msgs.append(self.packer.make_can_msg("CRZ_CTRL", 0, {
        "CRZ_AVAILABLE": crz_available,
        "CRZ_ACTIVE": crz_active,
      }))
    cs, _ = self.ci.update([(self.t, msgs)])
    # CI DBC packing can drop TJA_BUTTON; MADS only cares about ButtonType.lkas edges.
    prev_tja = self._tja
    self._tja = tja
    if tja != prev_tja:
      already = any(be.type == ButtonType.lkas and bool(be.pressed) == bool(tja) for be in cs.buttonEvents)
      if not already:
        be = structs.CarState.ButtonEvent(type=ButtonType.lkas, pressed=bool(tja))
        cs.buttonEvents = [*cs.buttonEvents, be]
    self.sd.events.clear()
    self.sd.events_sp.clear()
    self.mads.update(cs)
    self.sd.CS_prev = cs
    return cs


@pytest.mark.parametrize("alpha_long", [False, True])
class TestMazdaTjaMads:
  def test_1_tja_rising_edge_enables_mads(self, mocker, alpha_long):
    h = TjaMadsHarness(mocker, alpha_long=alpha_long)
    h.step()
    assert not h.mads.enabled
    h.step(tja=1)
    assert h.mads.enabled

  def test_2_tja_held_is_single_toggle(self, mocker, alpha_long):
    h = TjaMadsHarness(mocker, alpha_long=alpha_long)
    h.step()
    h.step(tja=1)
    assert h.mads.enabled
    for _ in range(5):
      h.step(tja=1)
      assert h.mads.enabled

  def test_3_tja_release_does_not_toggle(self, mocker, alpha_long):
    h = TjaMadsHarness(mocker, alpha_long=alpha_long)
    h.step()
    h.step(tja=1)
    h.step(tja=0)
    assert h.mads.enabled

  def test_4_second_tja_rising_edge_disables_mads(self, mocker, alpha_long):
    h = TjaMadsHarness(mocker, alpha_long=alpha_long)
    h.step()
    h.step(tja=1)
    h.step(tja=0)
    h.step(tja=1)
    assert not h.mads.enabled

  def test_5_one_mads_transition_per_rising_edge(self, mocker, alpha_long):
    h = TjaMadsHarness(mocker, alpha_long=alpha_long)
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

  def test_6_mrcc_with_mads_off_stays_off(self, mocker, alpha_long):
    h = TjaMadsHarness(mocker, alpha_long=alpha_long)
    h.step()
    if alpha_long:
      h.step(mode_x=1, mode_y=1, acc_off=1)
      h.step(acc_off=1)
    else:
      h.step(mode_x=1, mode_y=1, crz_available=1)
      h.step(crz_available=1)
    assert not h.mads.enabled

  def test_7_mrcc_with_mads_on_stays_on(self, mocker, alpha_long):
    h = TjaMadsHarness(mocker, alpha_long=alpha_long)
    h.step()
    h.step(tja=1)
    h.step(tja=0)
    assert h.mads.enabled
    if alpha_long:
      h.step(mode_x=1, mode_y=1, acc_off=1)
      h.step(acc_off=0)
    else:
      h.step(mode_x=1, mode_y=1, crz_available=1)
      h.step(crz_available=0)
    assert h.mads.enabled

  def test_mode_x_y_do_not_toggle_mads(self, mocker, alpha_long):
    h = TjaMadsHarness(mocker, alpha_long=alpha_long)
    h.step()
    h.step(mode_x=1)
    h.step()
    h.step(mode_y=1)
    h.step()
    h.step(mode_x=1, mode_y=1)
    h.step()
    assert not h.mads.enabled
    h.step(tja=1)
    h.step(tja=0)
    assert h.mads.enabled
    h.step(mode_x=1, mode_y=1)
    h.step()
    assert h.mads.enabled

  def test_8_to_11_set_res_cancel_do_not_toggle_mads(self, mocker, alpha_long):
    h = TjaMadsHarness(mocker, alpha_long=alpha_long)
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
    cruise = {"acc_off": 1} if alpha_long else {"crz_available": 1}
    h.step(set_p=1, **cruise)
    h.step(**cruise)
    h.step(set_m=1, **cruise)
    h.step(**cruise)
    h.step(res=1, **cruise)
    h.step(**cruise)
    h.step(can_off=1, **cruise)
    h.step(**cruise)
    assert h.mads.enabled

  def test_route_tja_pulses_toggle_mads_once_each(self, mocker, alpha_long):
    h = TjaMadsHarness(mocker, alpha_long=alpha_long)
    toggles = 0
    prev = False
    for tja, n in ROUTE_TJA_RLE:
      for _ in range(n):
        h.step(tja=tja)
        if h.mads.enabled != prev:
          toggles += 1
          prev = h.mads.enabled
    assert toggles == ROUTE_TJA_RISING_EDGES
    assert h.mads.enabled is True  # odd pulse count, started off

  def test_randomized_button_fuzz(self, mocker, alpha_long):
    rng = random.Random(1)
    h = TjaMadsHarness(mocker, alpha_long=alpha_long)
    h.step()
    prev_tja = 0
    expected = False
    expected_toggles = 0
    for _ in range(400):
      tja = rng.choice((0, 0, 0, 1))
      kwargs = {
        "tja": tja,
        "mode_x": rng.choice((0, 0, 1)),
        "mode_y": rng.choice((0, 0, 1)),
        "set_p": rng.choice((0, 0, 1)),
        "set_m": rng.choice((0, 0, 1)),
        "res": rng.choice((0, 0, 1)),
        "can_off": rng.choice((0, 0, 1)),
      }
      if alpha_long:
        kwargs["acc_off"] = rng.choice((0, 1))
        kwargs["acc_active"] = rng.choice((0, 1)) if kwargs["acc_off"] else 0
      else:
        kwargs["crz_available"] = rng.choice((0, 1))
        kwargs["crz_active"] = rng.choice((0, 1)) if kwargs["crz_available"] else 0
      h.step(**kwargs)
      if tja == 1 and prev_tja == 0:
        expected = not expected
        expected_toggles += 1
      assert h.mads.enabled is expected
      prev_tja = tja
    assert expected_toggles > 0


class TestMazdaTjaMadsCruiseState:
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

  def test_stock_long_tja_keeps_oem_crz_ctrl(self, mocker):
    h = TjaMadsHarness(mocker, alpha_long=False)
    h.step()
    cs = h.step(tja=1)
    assert h.mads.enabled
    assert not cs.cruiseState.available
    assert not cs.cruiseState.enabled
    h.step(tja=0)
    armed = h.step(crz_available=1)
    assert armed.cruiseState.available
    assert not armed.cruiseState.enabled
    assert h.mads.enabled
    tja_armed = h.step(tja=1, crz_available=1)
    assert tja_armed.cruiseState.available
    assert not tja_armed.cruiseState.enabled
    assert not h.mads.enabled

  def test_14_mrcc_oem_cruise_unchanged(self, mocker):
    h = TjaMadsHarness(mocker)
    h.step()
    armed = h.step(mode_x=1, mode_y=1, acc_off=1)
    assert armed.cruiseState.available
    assert not armed.cruiseState.enabled
    assert not h.mads.enabled
    active = h.step(acc_off=1, acc_active=1)
    assert active.cruiseState.available
    assert active.cruiseState.enabled
    assert not h.mads.enabled


class TestMazdaPandaAuthPause:
  def test_auth_loss_counts_after_grace_but_keeps_mads(self, mocker):
    from openpilot.sunnypilot.mads.mads import MAZDA_LATERAL_AUTH_GRACE, MAZDA_LATERAL_MISMATCH_LIMIT

    h = TjaMadsHarness(mocker, alpha_long=False)
    h.step()
    h.step(tja=1)
    assert h.mads.enabled
    h.mads.active = True
    h.sd.sm["pandaStates"][0].controlsAllowedLateral = False
    h.sd.sm["pandaStates"][0].safetyModel = SafetyModel.mazda
    for _ in range(MAZDA_LATERAL_AUTH_GRACE - 1):
      h.mads.data_sample()
    assert h.mads.lateral_mismatch_counter == MAZDA_LATERAL_AUTH_GRACE - 1
    assert h.mads.enabled
    for _ in range(MAZDA_LATERAL_MISMATCH_LIMIT):
      h.mads.data_sample()
    assert h.mads.lateral_mismatch_counter == MAZDA_LATERAL_AUTH_GRACE + MAZDA_LATERAL_MISMATCH_LIMIT - 1
    assert h.mads.enabled
    h.mads.data_sample()
    assert h.mads.lateral_mismatch_counter == MAZDA_LATERAL_AUTH_GRACE + MAZDA_LATERAL_MISMATCH_LIMIT
    h.mads.update_events(structs.CarState())
    assert h.sd.events_sp.has(EventNameSP.controlsMismatchLateral)
    assert h.mads.enabled

  def test_missing_panda_counts_as_mismatch(self, mocker):
    from openpilot.sunnypilot.mads.mads import MAZDA_LATERAL_AUTH_GRACE, MAZDA_LATERAL_MISMATCH_LIMIT

    h = TjaMadsHarness(mocker, alpha_long=False)
    h.step()
    h.step(tja=1)
    assert h.mads.enabled
    h.mads.active = True
    h.sd.sm["pandaStates"] = []
    for _ in range(MAZDA_LATERAL_AUTH_GRACE + MAZDA_LATERAL_MISMATCH_LIMIT):
      h.mads.data_sample()
    h.mads.update_events(structs.CarState())
    assert h.sd.events_sp.has(EventNameSP.controlsMismatchLateral)
    assert h.mads.enabled

  def test_all_ignored_pandas_count_as_mismatch(self, mocker):
    from openpilot.sunnypilot.mads.mads import MAZDA_LATERAL_AUTH_GRACE, MAZDA_LATERAL_MISMATCH_LIMIT

    h = TjaMadsHarness(mocker, alpha_long=False)
    h.step()
    h.step(tja=1)
    h.mads.active = True
    h.sd.sm["pandaStates"][0].safetyModel = SafetyModel.elm327
    h.sd.sm["pandaStates"][0].controlsAllowedLateral = True
    for _ in range(MAZDA_LATERAL_AUTH_GRACE + MAZDA_LATERAL_MISMATCH_LIMIT):
      h.mads.data_sample()
    h.mads.update_events(structs.CarState())
    assert h.sd.events_sp.has(EventNameSP.controlsMismatchLateral)
    assert h.mads.enabled

  def test_uem_param_on_is_forced_off(self, mocker):
    h = TjaMadsHarness(mocker)
    assert h.mads.unified_engagement_mode is False
    assert h.mads.main_enabled_toggle is False
    h.mads.read_params()
    assert h.mads.unified_engagement_mode is False
    assert h.mads.main_enabled_toggle is False

  def test_pcm_enable_does_not_enable_mads(self, mocker):
    h = TjaMadsHarness(mocker)
    h.step()
    assert not h.mads.enabled
    h.sd.events.add(EventName.pcmEnable)
    h.mads.update(structs.CarState())
    assert not h.mads.enabled
    assert not h.sd.events.has(EventName.pcmEnable)

  def test_button_enable_does_not_enable_mads(self, mocker):
    h = TjaMadsHarness(mocker)
    h.step()
    h.sd.events.add(EventName.buttonEnable)
    h.mads.update(structs.CarState())
    assert not h.mads.enabled
    assert not h.sd.events.has(EventName.buttonEnable)

  def test_set_car_specific_params_disables_mazda_uem(self, mocker):
    params = mocker.MagicMock()
    CP = mocker.MagicMock()
    CP.brand = "mazda"
    set_car_specific_params(CP, mocker.MagicMock(), params)
    params.put_bool.assert_any_call("MadsUnifiedEngagementMode", False, block=True)
    params.remove.assert_any_call("MadsMainCruiseAllowed")

  def test_panda_lateral_allowed_helper(self):
    from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
    from openpilot.cereal import custom, log

    class PS:
      def __init__(self, model, cal=False, ca=False):
        self.safetyModel = model
        self.controlsAllowedLateral = cal
        self.controlsAllowed = ca

    def sm(pandas, *, mads_active=True, selfdrive_active=False):
      ss = log.SelfdriveState()
      ss.active = selfdrive_active
      ss_sp = custom.SelfdriveStateSP()
      ss_sp.mads.active = mads_active
      return {
        "pandaStates": pandas,
        "selfdriveState": ss,
        "selfdriveStateSP": ss_sp,
      }

    fn = ControlsExt._panda_lateral_allowed
    assert fn(sm([PS("elm327")])) is False
    assert fn(sm([PS("mazda", cal=False, ca=False)])) is False
    assert fn(sm([PS("mazda", cal=True)])) is True
    assert fn(sm([PS("mazda", ca=True)])) is False
    assert fn(sm([PS("mazda", ca=True)], mads_active=False, selfdrive_active=True)) is True
    assert fn(sm([PS("mazda", cal=False), PS("mazda", cal=True)])) is False
    assert fn(sm([])) is False
