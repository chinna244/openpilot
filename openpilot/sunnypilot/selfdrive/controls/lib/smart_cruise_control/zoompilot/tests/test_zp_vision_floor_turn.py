"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Floor turns: a curve under the 20 km/h operating floor that the car is already turning through near
the floor. Pulling away from a signal, the car crosses MIN_V mid-turn and a cycle started there could
only hold MIN_V, which at the exit is a pause in the pull-away (routes 0000014b, 00000151-153). The
start is left to the plan until the solver releases the curve, unless the curvature past the
actuation lead would put the car over _FLOOR_TURN_LAT_CAP a second on, or the car gets more than the
in-curve margin over the curve. Long tight curves (loops, hairpins) and every curve met above the
floor start exactly as before.
"""
from unittest import mock

import numpy as np
import pytest

from openpilot.cereal import custom
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import vision_controller
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.tests.vision_harness import VisionCase, make_cp

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState

V_EXIT = 5.7  # m/s, 12.8 mph: just over MIN_V, where the pauses began
K_TURN = 0.08  # 1/m, a 12.5 m intersection corner (2.6 m/s2 at V_EXIT)
SIDES = pytest.mark.parametrize("side", [1., -1.], ids=["left", "right"])


def corner(k_near: float, k_after: float, side: float, d_split: float = 4.0):
  """The turn at the car, then what follows past d_split (inside the actuation lead at V_EXIT is ~4.5 m)."""
  return lambda s: side * (k_near if s < d_split else k_after)


def floor_turns_never_latch():
  """The controller as it was before the latch: every floor turn escapes at once."""
  return mock.patch.object(vision_controller, '_FLOOR_TURN_LAT_CAP', 0.)


class TestFloorTurn(VisionCase):

  def step(self, v, kappa_fn, cur_curvature, a_ego=0., n=3, scc=None):
    scc = scc or self.scc_v
    sm = self.make_sm(v, kappa_fn, cur_curvature)
    for _ in range(n):
      scc.update(sm, True, False, v, a_ego, 20.)
    return scc

  @SIDES
  def test_turn_exit_at_the_floor_starts_no_cycle(self, side):
    # the corner is behind the actuation lead and the road opens out after it
    road = corner(K_TURN, 0.03, side)
    self.step(V_EXIT, road, side * K_TURN)
    assert self.scc_v.solver_active  # the solver still reads the corner the car is in
    assert self.scc_v.state == VisionState.enabled
    assert self.scc_v.output_v_target == V_CRUISE_UNSET

  @SIDES
  def test_same_turn_started_a_cycle_before_the_latch(self, side):
    with floor_turns_never_latch():
      self.step(V_EXIT, corner(K_TURN, 0.03, side), side * K_TURN)
    assert self.scc_v.is_active
    assert self.scc_v.output_v_target == MIN_V

  @SIDES
  def test_tightening_floor_turn_escapes(self, side):
    # past the lead the turn tightens: 0.10 at V_EXIT is 3.2 m/s2, over the cap
    self.step(V_EXIT, corner(K_TURN, 0.10, side), side * K_TURN)
    assert self.scc_v.is_active

  @SIDES
  def test_accelerating_into_the_rest_of_the_turn_escapes(self, side):
    # 0.075 past the lead is 2.4 m/s2 at V_EXIT, but 3.0 a second on at +0.6 m/s2
    road = corner(K_TURN, 0.075, side)
    self.step(V_EXIT, road, side * K_TURN, a_ego=0.)
    assert self.scc_v.state == VisionState.enabled
    self.step(V_EXIT, road, side * K_TURN, a_ego=0.6)
    assert self.scc_v.is_active

  @SIDES
  def test_latched_turn_hands_back_once_the_car_is_over_it(self, side):
    road = corner(K_TURN, 0.03, side)
    self.step(V_EXIT, road, side * K_TURN)
    assert self.scc_v.state == VisionState.enabled
    # still in the same corner, now more than the in-curve margin over its speed
    v_over = max(self.scc_v.v_near_min, MIN_V) + vision_controller._IN_CURVE_MARGIN + 0.1
    self.step(v_over, road, side * K_TURN)
    assert self.scc_v.is_active

  @SIDES
  def test_latch_clears_with_the_curve(self, side):
    self.step(V_EXIT, corner(K_TURN, 0.03, side), side * K_TURN)
    self.step(V_EXIT, lambda s: 0., 0.)
    assert not self.scc_v.solver_active
    assert not self.scc_v.floor_turn

  def test_stock_acc_is_unchanged(self):
    stock = SmartCruiseControlVision(make_cp(op_long=False))
    self.step(V_EXIT, corner(K_TURN, 0.03, 1.), K_TURN, scc=stock)
    assert not stock.floor_turn


def pull_away_peak_lat(scc, kappa, side, a_plan=1.0, v_set=25., t_max=20., dt=0.05):
  """Pull away from 2 m/s through a constant curve with a plan that never slows for it, the planner taking
  min(plan, the cruise candidate on the cycle's target) as on openpilot long; the peak lateral acceleration."""
  v, a_cruise, a_prev, peak = 2., a_plan, 0., 0.
  for _ in range(int(t_max / dt)):
    sm = VisionCase.make_sm(None, v, lambda s: side * kappa, side * kappa)
    scc.update(sm, True, False, v, a_prev, v_set)
    v_cruise = min(v_set, scc.output_v_target) if scc.is_active else v_set
    j = float(np.interp(v, [0., 10.], [1.6, 1.2])) * dt
    a_cruise = float(np.clip(np.clip(v_cruise - v, -1.2, 2.), a_cruise - j, a_cruise + j))
    a_prev = min(a_plan, a_cruise)
    v = max(v + a_prev * dt, 0.)
    peak = max(peak, v * v * kappa)
  return peak


class TestLongCurvesKeepTheirProtection(VisionCase):

  @SIDES
  @pytest.mark.parametrize("radius", [15., 20., 40.], ids=["hairpin_r15", "loop_r20", "loop_r40"])
  def test_loop_entered_from_a_stop_peaks_as_before(self, radius, side):
    peak_now = pull_away_peak_lat(self.scc_v, 1. / radius, side)
    with floor_turns_never_latch():
      peak_before = pull_away_peak_lat(SmartCruiseControlVision(make_cp()), 1. / radius, side)
    assert peak_before < 3.5  # the curve is protected at all
    assert peak_now <= peak_before + 0.01

  @SIDES
  def test_highway_bend_starts_as_before(self, side):
    def run(scc):
      for d in np.arange(300., 0., -5.):
        sm = self.make_sm(29., lambda s, d=d: side * 0.004 if s >= d else 0.)
        scc.update(sm, True, False, 29., 0., 31.)
        if scc.is_active:
          return d
      return None
    d_now = run(self.scc_v)
    with floor_turns_never_latch():
      d_before = run(SmartCruiseControlVision(make_cp()))
    assert d_before is not None
    assert d_now == d_before
