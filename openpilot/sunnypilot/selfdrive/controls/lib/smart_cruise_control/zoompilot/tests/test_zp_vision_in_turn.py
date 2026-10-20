"""
In-turn braking: once the car is turning through a town corner on openpilot long, the vision
controller brakes no harder than _IN_TURN_A_MIN, unless the turn past the actuation lead would put
the car over _IN_TURN_ESCAPE_LAT. The approach, faster curves, stock ACC and tightening turns brake
as before.
"""
from unittest import mock

import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import vision_controller
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.tests.vision_harness import VisionCase, make_cp

A_MIN = vision_controller._IN_TURN_A_MIN
SIDES = pytest.mark.parametrize("side", [1., -1.], ids=["left", "right"])


def corner(k_near: float, k_after: float, side: float, d_split: float = 4.0):
  """The turn at the car, then what follows past d_split (inside the actuation lead)."""
  return lambda s: side * (k_near if s < d_split else k_after)


def uncapped():
  """The controller as it was before the in-turn cap."""
  return mock.patch.object(vision_controller, '_IN_TURN_A_MIN', -99.)


class TestInTurnBraking(VisionCase):

  def drive(self, v, kappa_fn, cur_curvature, op_long=True):
    """Hardest command over 2 s on a fresh controller."""
    scc = SmartCruiseControlVision(make_cp(op_long))
    sm = self.make_sm(v, kappa_fn, cur_curvature)
    a_min = 0.
    for _ in range(40):
      scc.update(sm, True, False, v, 0., 20.)
      a_min = min(a_min, scc.output_a_target)
    return a_min

  def same_as_before(self, *args, **kwargs):
    a = self.drive(*args, **kwargs)
    with uncapped():
      return a == pytest.approx(self.drive(*args, **kwargs))

  @SIDES
  def test_turning_through_a_town_corner_brakes_gently(self, side):
    # 8 m/s (18 mph) in a 20 m corner: 3.2 m/s2 now and past the lead, over the 2.3 ceiling
    assert self.drive(8., corner(0.05, 0.05, side), side * 0.05) == pytest.approx(A_MIN)

  @SIDES
  def test_same_corner_braked_hard_before_the_cap(self, side):
    with uncapped():
      assert self.drive(8., corner(0.05, 0.05, side), side * 0.05) < A_MIN - 0.5

  @SIDES
  def test_tightening_turn_brakes_as_before(self, side):
    # 0.06 past the lead is 3.84 m/s2 at 8 m/s, over the escape
    road = corner(0.05, 0.06, side)
    assert self.drive(8., road, side * 0.05) < A_MIN - 0.5
    assert self.same_as_before(8., road, side * 0.05)

  @SIDES
  def test_approach_before_the_turn_brakes_as_before(self, side):
    # not turning yet: the car's own curvature is zero
    assert self.same_as_before(8., corner(0.05, 0.05, side), 0.)

  def test_faster_curve_brakes_as_before(self):
    # 17 m/s (38 mph) in an 83 m curve: 3.47 m/s2, under the escape, but above town speed
    assert self.drive(17., corner(0.012, 0.012, 1.), 0.012) < A_MIN
    assert self.same_as_before(17., corner(0.012, 0.012, 1.), 0.012)

  def test_stock_acc_brakes_as_before(self):
    assert self.same_as_before(8., corner(0.05, 0.05, 1.), 0.05, op_long=False)
