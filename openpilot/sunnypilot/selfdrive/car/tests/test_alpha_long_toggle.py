"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.car.alpha_long_toggle import AlphaLongToggleMonitor, HANDBACK_TIMEOUT_FRAMES
from openpilot.sunnypilot.system.offroad_request import STANDSTILL_V

MOVING_V = 12.0


class FakeParams:
  def __init__(self, **bools):
    self.bools = dict(bools)

  def get_bool(self, key):
    return self.bools.get(key, False)

  def put_bool(self, key, value, **kwargs):
    self.bools[key] = value


def _monitor(toggle: bool, brand="mazda", op_long=True, alpha_avail=True, offroad_requested=False, cycle_attempted=False,
             parked=True):
  cp = structs.CarParams()
  cp.brand = brand
  cp.openpilotLongitudinalControl = op_long
  cp.alphaLongitudinalAvailable = alpha_avail
  params = FakeParams(AlphaLongitudinalEnabled=toggle, OffroadModeRequested=offroad_requested,
                      AlphaLongCycleAttempted=cycle_attempted)
  m = AlphaLongToggleMonitor(cp, params)
  m.update_params()
  if parked:
    # the car has been sitting still since card started; the standstill debounce is already satisfied
    m.standstill.stopped_frames = m.standstill.frames_needed
  return m, params


def _step(monitor, acc_faulted=False, enabled=False, v_ego=0.0):
  cs = structs.CarState()
  cs.accFaulted = acc_faulted
  cs.vEgo = v_ego
  cc = structs.CarControl()
  cc.enabled = enabled
  cc_sp = structs.CarControlSP()
  monitor.update(cs, cc, cc_sp)
  return cc_sp


class TestAlphaLongToggleMonitor:
  def test_no_mismatch_no_action(self):
    m, params = _monitor(toggle=True, op_long=True)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
    assert not params.get_bool("OnroadCycleRequested")

  def test_enable_direction_cycles_immediately(self):
    m, params = _monitor(toggle=True, op_long=False)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
    assert params.get_bool("OnroadCycleRequested")

  def test_disable_runs_handback_until_radar_returns(self):
    m, params = _monitor(toggle=False, op_long=True)
    # radar still silent: hand-back asserted, no cycle yet
    for _ in range(50):
      cc_sp = _step(m, acc_faulted=False)
      assert cc_sp.stockEcuHandBack
      assert not params.get_bool("OnroadCycleRequested")
    # stock radar heard again: cycle requested
    cc_sp = _step(m, acc_faulted=True)
    assert cc_sp.stockEcuHandBack
    assert params.get_bool("OnroadCycleRequested")

  def test_disable_times_out_to_cycle(self):
    m, params = _monitor(toggle=False, op_long=True)
    for _ in range(HANDBACK_TIMEOUT_FRAMES):
      _step(m, acc_faulted=False)
    assert params.get_bool("OnroadCycleRequested")

  def test_waits_for_disengagement(self):
    m, params = _monitor(toggle=False, op_long=True)
    cc_sp = _step(m, enabled=True)
    assert not cc_sp.stockEcuHandBack
    # once started, engagement no longer pauses the sequence
    _step(m, enabled=False)
    cc_sp = _step(m, enabled=True)
    assert cc_sp.stockEcuHandBack

  def test_handback_stays_asserted_after_done(self):
    # CC_SP is rebuilt each frame; dropping the assert once done latched made the session
    # manager read a withdrawal and re-silence the radar it had just handed back, right
    # before shutdown
    m, params = _monitor(toggle=False, op_long=True)
    _step(m)
    _step(m, acc_faulted=True)
    assert params.get_bool("OnroadCycleRequested")
    for _ in range(10):
      cc_sp = _step(m, acc_faulted=True)
      assert cc_sp.stockEcuHandBack

  def test_no_assert_after_done_when_nothing_was_handed_back(self):
    # the enable direction never starts a hand-back, so there is nothing to keep asserting
    m, params = _monitor(toggle=True, op_long=False)
    _step(m)
    assert params.get_bool("OnroadCycleRequested")
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack

  def test_non_mazda_disable_cycles_immediately(self):
    m, params = _monitor(toggle=False, brand="toyota", op_long=True)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
    assert params.get_bool("OnroadCycleRequested")

  def test_unavailable_never_acts(self):
    m, params = _monitor(toggle=True, op_long=False, alpha_avail=False)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
    assert not params.get_bool("OnroadCycleRequested")

  def test_cycle_requested_only_once(self):
    m, params = _monitor(toggle=True, op_long=False)
    _step(m)
    params.put_bool("OnroadCycleRequested", False)  # hardwared consumed it
    _step(m)
    assert not params.get_bool("OnroadCycleRequested")


class TestEngagedDefersFinish:
  # the UIs block both actions while engaged, but the params can flip from anywhere

  def test_enable_direction_waits_for_disengage(self):
    m, params = _monitor(toggle=True, op_long=False)
    for _ in range(50):
      _step(m, enabled=True)
      assert not params.get_bool("OnroadCycleRequested")
    _step(m, enabled=False)
    assert params.get_bool("OnroadCycleRequested")

  def test_non_mazda_disable_waits_for_disengage(self):
    m, params = _monitor(toggle=False, brand="toyota", op_long=True)
    _step(m, enabled=True)
    assert not params.get_bool("OnroadCycleRequested")
    _step(m, enabled=False)
    assert params.get_bool("OnroadCycleRequested")

  def test_mazda_radar_return_while_engaged_holds_cycle_not_handback(self):
    m, params = _monitor(toggle=False, op_long=True)
    _step(m)  # hand-back starts disengaged
    # engaged when the radar comes back: keep asserting, do not cycle yet
    for _ in range(50):
      cc_sp = _step(m, acc_faulted=True, enabled=True)
      assert cc_sp.stockEcuHandBack
      assert not params.get_bool("OnroadCycleRequested")
    cc_sp = _step(m, acc_faulted=True, enabled=False)
    assert cc_sp.stockEcuHandBack
    assert params.get_bool("OnroadCycleRequested")

  def test_mazda_timeout_while_engaged_holds_cycle(self):
    m, params = _monitor(toggle=False, op_long=True)
    _step(m)
    for _ in range(HANDBACK_TIMEOUT_FRAMES + 10):
      cc_sp = _step(m, enabled=True)
      assert cc_sp.stockEcuHandBack
    assert not params.get_bool("OnroadCycleRequested")
    _step(m, enabled=False)
    assert params.get_bool("OnroadCycleRequested")

  def test_non_mazda_offroad_grant_waits_for_disengage(self):
    m, params = _monitor(toggle=True, brand="toyota", op_long=True, offroad_requested=True)
    _step(m, enabled=True)
    assert not params.get_bool("OffroadMode")
    _step(m, enabled=False)
    assert params.get_bool("OffroadMode")

  def test_mazda_offroad_grant_waits_for_disengage(self):
    m, params = _monitor(toggle=True, op_long=True, offroad_requested=True)
    _step(m)
    cc_sp = _step(m, acc_faulted=True, enabled=True)
    assert cc_sp.stockEcuHandBack
    assert not params.get_bool("OffroadMode")
    _step(m, acc_faulted=True, enabled=False)
    assert params.get_bool("OffroadMode")


class TestOneCyclePerIgnition:
  def test_cycle_marks_the_ignition(self):
    m, params = _monitor(toggle=True, op_long=False)
    _step(m)
    assert params.get_bool("AlphaLongCycleAttempted")

  def test_persisting_mismatch_does_not_cycle_again(self):
    # card restarted after the cycle and the fingerprint still does not satisfy the toggle
    m, params = _monitor(toggle=True, op_long=False, cycle_attempted=True)
    for _ in range(50):
      _step(m)
    assert not params.get_bool("OnroadCycleRequested")
    assert params.get_bool("AlphaLongCycleAttempted")

  def test_persisting_mismatch_mazda_disable_does_not_hand_back(self):
    m, params = _monitor(toggle=False, op_long=True, cycle_attempted=True)
    cc_sp = _step(m, acc_faulted=True)
    assert not cc_sp.stockEcuHandBack
    assert not params.get_bool("OnroadCycleRequested")

  def test_satisfied_toggle_clears_the_marker(self):
    # the cycle took: a later flip this ignition is a fresh request
    m, params = _monitor(toggle=True, op_long=True, cycle_attempted=True)
    assert not params.get_bool("AlphaLongCycleAttempted")
    params.put_bool("AlphaLongitudinalEnabled", False)
    m.update_params()
    _step(m, acc_faulted=True)
    assert params.get_bool("OnroadCycleRequested")

  def test_offroad_request_still_granted_after_a_cycle(self):
    m, params = _monitor(toggle=True, op_long=False, offroad_requested=True, cycle_attempted=True)
    _step(m)
    assert params.get_bool("OffroadMode")
    assert not params.get_bool("OnroadCycleRequested")


class TestOffroadRequest:
  def test_mazda_op_long_hands_back_before_granting(self):
    m, params = _monitor(toggle=True, op_long=True, offroad_requested=True)
    for _ in range(50):
      cc_sp = _step(m)
      assert cc_sp.stockEcuHandBack
      assert not params.get_bool("OffroadMode")
    _step(m, acc_faulted=True)
    assert params.get_bool("OffroadMode")
    assert not params.get_bool("OffroadModeRequested")
    assert not params.get_bool("OnroadCycleRequested")

  def test_handback_timeout_still_grants(self):
    m, params = _monitor(toggle=True, op_long=True, offroad_requested=True)
    for _ in range(HANDBACK_TIMEOUT_FRAMES):
      _step(m)
    assert params.get_bool("OffroadMode")

  def test_non_mazda_grants_immediately(self):
    m, params = _monitor(toggle=True, brand="toyota", op_long=True, offroad_requested=True)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
    assert params.get_bool("OffroadMode")

  def test_stock_long_grants_immediately(self):
    m, params = _monitor(toggle=False, op_long=False, offroad_requested=True)
    cc_sp = _step(m)
    assert not cc_sp.stockEcuHandBack
    assert params.get_bool("OffroadMode")

  def test_offroad_wins_over_pending_toggle_cycle(self):
    # toggle-off and force-offroad both pending: go offroad, skip the cycle
    m, params = _monitor(toggle=False, op_long=True, offroad_requested=True)
    _step(m, acc_faulted=True)
    assert params.get_bool("OffroadMode")
    assert not params.get_bool("OnroadCycleRequested")

  def test_grant_works_when_alpha_unavailable(self):
    m, params = _monitor(toggle=False, op_long=False, alpha_avail=False, offroad_requested=True)
    _step(m)
    assert params.get_bool("OffroadMode")


class TestStandstillGate:
  # Neither finish may end the session under a moving car: the UIs only refuse the toggle
  # and the offroad button while engaged, and on file both were pressed at up to 19 m/s

  def _stop(self, m, **kw):
    for _ in range(m.standstill.frames_needed - 1):
      _step(m, v_ego=0.0, **kw)
      assert not m.done
    return _step(m, v_ego=0.0, **kw)

  def test_toggle_cycle_waits_for_standstill(self):
    m, params = _monitor(toggle=False, op_long=True, parked=False)
    # the hand-back itself runs at speed and the radar answers; only the cycle is held
    for _ in range(100):
      cc_sp = _step(m, acc_faulted=True, v_ego=MOVING_V)
      assert cc_sp.stockEcuHandBack
      assert not params.get_bool("OnroadCycleRequested")
    self._stop(m, acc_faulted=True)
    assert params.get_bool("OnroadCycleRequested")

  def test_offroad_grant_waits_for_standstill(self):
    m, params = _monitor(toggle=True, op_long=True, offroad_requested=True, parked=False)
    for _ in range(100):
      cc_sp = _step(m, acc_faulted=True, v_ego=MOVING_V)
      assert cc_sp.stockEcuHandBack
      assert not params.get_bool("OffroadMode")
    self._stop(m, acc_faulted=True)
    assert params.get_bool("OffroadMode")
    assert not params.get_bool("OffroadModeRequested")

  def test_handback_timeout_still_waits_for_standstill(self):
    m, params = _monitor(toggle=True, op_long=True, offroad_requested=True, parked=False)
    for _ in range(HANDBACK_TIMEOUT_FRAMES + 100):
      cc_sp = _step(m, v_ego=MOVING_V)
      assert cc_sp.stockEcuHandBack
    assert not params.get_bool("OffroadMode")
    self._stop(m)
    assert params.get_bool("OffroadMode")

  def test_enable_direction_waits_for_standstill(self):
    m, params = _monitor(toggle=True, op_long=False, parked=False)
    for _ in range(100):
      _step(m, v_ego=MOVING_V)
    assert not params.get_bool("OnroadCycleRequested")
    self._stop(m)
    assert params.get_bool("OnroadCycleRequested")

  def test_non_mazda_offroad_grant_waits_for_standstill(self):
    m, params = _monitor(toggle=True, brand="toyota", op_long=True, offroad_requested=True, parked=False)
    for _ in range(100):
      _step(m, v_ego=MOVING_V)
    assert not params.get_bool("OffroadMode")
    self._stop(m)
    assert params.get_bool("OffroadMode")

  def test_creep_below_threshold_counts_as_stopped(self):
    m, params = _monitor(toggle=True, op_long=False, parked=False)
    for _ in range(m.standstill.frames_needed):
      _step(m, v_ego=STANDSTILL_V / 2)
    assert params.get_bool("OnroadCycleRequested")

  def test_brief_stop_does_not_count(self):
    m, params = _monitor(toggle=True, op_long=False, parked=False)
    for _ in range(m.standstill.frames_needed - 1):
      _step(m, v_ego=0.0)
    _step(m, v_ego=MOVING_V)
    for _ in range(m.standstill.frames_needed - 1):
      _step(m, v_ego=0.0)
    assert not params.get_bool("OnroadCycleRequested")

  def test_engaged_at_standstill_still_holds(self):
    # op-long holding the car at a light: stopped but engaged, so nothing fires until disengaged
    m, params = _monitor(toggle=True, op_long=True, offroad_requested=True, parked=False)
    _step(m, v_ego=MOVING_V)
    for _ in range(m.standstill.frames_needed + 10):
      _step(m, acc_faulted=True, enabled=True, v_ego=0.0)
    assert not params.get_bool("OffroadMode")
    _step(m, acc_faulted=True, enabled=False, v_ego=0.0)
    assert params.get_bool("OffroadMode")
