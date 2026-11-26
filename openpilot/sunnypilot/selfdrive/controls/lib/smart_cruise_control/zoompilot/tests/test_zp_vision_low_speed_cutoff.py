"""Low-speed cutoff (route 08 t=309.6): at 13.4 mph mid-turn a cycle started just over MIN_V clamped the cruise target to
the car's speed and braked while the model asked +1.2. Under the cutoff no cycle starts and an active one is handed back
to the plan unless the turn past the actuation lead would reach _FLOOR_TURN_LAT_CAP; above it nothing changes."""
from unittest import mock

import pytest

from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import VisionState
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import vision_controller
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.tests.vision_harness import VisionCase, make_cp

MPH = 0.44704
CUTOFFS = pytest.mark.parametrize("cutoff_mph", [20., 25.])


def cutoff(mph):
  return mock.patch.object(vision_controller, 'LOW_SPEED_CUTOFF', mph * MPH)


class TestLowSpeedCutoff(VisionCase):
  def step(self, v, k, cur, n=5, a_ego=0., scc=None):
    scc = scc or self.scc_v
    sm = self.make_sm(v, lambda s: k(s) if callable(k) else k, cur)
    for _ in range(n):
      scc.update(sm, True, False, v, a_ego, 30.)
    return scc

  def new_scc(self):
    scc = SmartCruiseControlVision(make_cp())
    scc.enabled = self.scc_v.enabled
    return scc

  @CUTOFFS
  def test_turn_under_the_cutoff_starts_no_cycle(self, cutoff_mph):
    # 2.9 m/s2 projected ahead starts a cycle today; with the cutoff only a turn past the 2.8 cap may, so the same
    # corner read a little gentler (2.6, route 08 t=309.6 was 2.5-2.8) stays with the plan under the cutoff
    v = 7.0
    for lat, starts_today, starts_cut in ((2.94, True, True), (2.6, False, False)):
      today, cut = self.new_scc(), self.new_scc()
      road = (lambda k: lambda s: k if s > 3. else 0.01)(lat / v ** 2)
      with cutoff(MIN_V / MPH):  # today: no cutoff
        self.step(v, road, 0.01, scc=today)
      with cutoff(cutoff_mph):
        self.step(v, road, 0.01, scc=cut)
      assert today.is_active == starts_today
      assert cut.is_active == starts_cut
      if not starts_cut:
        assert cut.output_v_target == V_CRUISE_UNSET

  @CUTOFFS
  def test_active_cycle_is_handed_back_under_the_cutoff(self, cutoff_mph):
    with cutoff(cutoff_mph):
      self.step(13.5, 0.02, 0.02)  # 30 mph into a 50 m curve: a cycle starts
      assert self.scc_v.is_active
      self.step((cutoff_mph - 3) * MPH, 0.02, 0.02)  # slowed under the cutoff, the rest of the curve is gentle
      assert self.scc_v.state == VisionState.enabled
      assert self.scc_v.output_v_target == V_CRUISE_UNSET

  @CUTOFFS
  def test_tight_turn_ahead_keeps_the_cycle(self, cutoff_mph):
    with cutoff(cutoff_mph):
      self.step(13.5, 0.02, 0.02)
      assert self.scc_v.is_active
      v = (cutoff_mph - 3) * MPH
      self.step(v, 3.2 / v ** 2, 0.02)  # 3.2 m/s2 past the lead: over the 2.8 cap
      assert self.scc_v.is_active

  @CUTOFFS
  def test_tightening_turn_under_the_cutoff_still_starts(self, cutoff_mph):
    v = 6.0
    with cutoff(cutoff_mph):
      self.step(v, lambda s: 3.4 / v ** 2 if s > 3. else 0.01, 0.01)  # 3.4 m/s2 ahead: over the 2.8 cap
    assert self.scc_v.is_active

  @CUTOFFS
  def test_above_the_cutoff_nothing_changes(self, cutoff_mph):
    other = self.new_scc()
    for v, k in ((13.5, 0.02), (12.5, 0.025), (11.5, 0.03), ((cutoff_mph + 1) * MPH, 0.03)):
      with cutoff(MIN_V / MPH):  # today: no cutoff
        self.step(v, k, k)
      with cutoff(cutoff_mph):
        self.step(v, k, k, scc=other)
      assert other.state == self.scc_v.state
      assert other.output_v_target == self.scc_v.output_v_target

  @CUTOFFS
  def test_car_at_the_cap_mid_corner_keeps_the_cycle(self, cutoff_mph):
    # a 3.0 m/s2 corner now, gentler past the lead: the plan does not take over a car at the cap
    with cutoff(cutoff_mph):
      v, tight = 6.5, 0.07
      self.step(v, tight, tight, n=20)
      assert self.scc_v.is_active
      self.step(v, lambda s: tight if s < 4. else 0.03, tight, n=10)
      assert self.scc_v.is_active
      self.step(v, 0.03, 0.03, n=10)  # out of the peak and gentle ahead: handed back
      assert self.scc_v.state == VisionState.enabled

  @CUTOFFS
  def test_corner_read_near_the_cap_does_not_flap(self, cutoff_mph):
    # one threshold switched the cycle on and off every frame for a corner read 2.75/2.85 and its braking never built
    v = 8.0
    with cutoff(cutoff_mph):
      self.step(v, lambda s: 3.2 / v ** 2 if s > 3. else 0.01, 0.01, n=10)
      assert self.scc_v.is_active
      for i in range(16):
        lat = 2.85 if i % 2 else 2.75
        self.step(v, (lambda k: lambda s: k if s > 3. else 0.01)(lat / v ** 2), 0.01, n=1)
        assert self.scc_v.is_active
        assert self.scc_v.output_v_target != V_CRUISE_UNSET

  @CUTOFFS
  def test_cycle_is_handed_back_once_the_turn_reads_under_the_release(self, cutoff_mph):
    v = 8.0
    with cutoff(cutoff_mph):
      self.step(v, lambda s: 3.2 / v ** 2 if s > 3. else 0.01, 0.01, n=10)
      assert self.scc_v.is_active
      self.step(v, lambda s: 2.4 / v ** 2 if s > 3. else 0.01, 0.01)
      assert self.scc_v.state == VisionState.enabled
      assert self.scc_v.output_v_target == V_CRUISE_UNSET

  @CUTOFFS
  def test_stock_acc_is_unchanged(self, cutoff_mph):
    # no openpilot plan to take the turn over on stock ACC: the dash setpoint stays
    today, cut = SmartCruiseControlVision(make_cp(False)), SmartCruiseControlVision(make_cp(False))
    today.enabled = cut.enabled = self.scc_v.enabled
    for v, k in ((13.5, 0.02), (9.0, 0.025), (7.0, 0.04), (6.0, 0.05)):
      with cutoff(MIN_V / MPH):
        self.step(v, k, k, scc=today)
      with cutoff(cutoff_mph):
        self.step(v, k, k, scc=cut)
      assert cut.state == today.state
      assert cut.output_v_target == today.output_v_target

  @CUTOFFS
  def test_override_keeps_priority_under_the_cutoff(self, cutoff_mph):
    v = 8.0
    with cutoff(cutoff_mph):
      self.step(v, lambda s: 3.2 / v ** 2 if s > 3. else 0.01, 0.01, n=10)
      sm = self.make_sm(v, lambda s: 2.0 / v ** 2, 0.01)
      self.scc_v.update(sm, True, True, v, 0., 30.)
      assert self.scc_v.state == VisionState.overriding
      assert self.scc_v.output_v_target == V_CRUISE_UNSET
