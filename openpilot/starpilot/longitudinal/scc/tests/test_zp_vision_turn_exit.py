"""
Turn exit: once a town corner opens and the model asks to pull away, the vision controller hands the turn back to
the plan instead of holding the bend it is leaving (route 00000166 t=395). A tight turn, a corner that tightens
again, a model that is not pulling away, faster curves and stock ACC are held as before.
"""
import numpy as np
import pytest

from openpilot.starpilot.longitudinal.scc.zoompilot.vision_controller import SmartCruiseControlVision
from openpilot.starpilot.longitudinal.scc.tests.vision_harness import VisionCase, make_cp

SIDES = pytest.mark.parametrize("side", [1., -1.], ids=["left", "right"])
V_CRUISE = 20.
TIGHT = 0.07  # 1/m: 3.0 m/s2 at 6.5 m/s


def corner(k_near: float, k_after: float, side: float, d_split: float = 4.0):
  return lambda s: side * (k_near if s < d_split else k_after)


class TestTurnExit(VisionCase):

  def exit_turn(self, side, *, v=6.5, k_exit=0.045, k_ahead=0., model_a=0.8, op_long=True, k_leaving=0.06):
    """1 s through the tight part of the corner, then 1 s with the wheel unwinding: the near field still holds the
    bend the car is leaving (inside the actuation lead), past it the road is k_ahead. Returns the controller."""
    scc = SmartCruiseControlVision(make_cp(op_long))
    tight = self.make_sm(v, corner(TIGHT, TIGHT, side), side * TIGHT)
    tight['modelV2'].action.desiredAcceleration = model_a
    for _ in range(20):
      scc.update(tight, True, False, v, 0., V_CRUISE)
    assert scc._controlling and scc.turn_peak_lat > 2.5
    opening = self.make_sm(v, corner(k_leaving, k_ahead, side), side * k_exit)
    opening['modelV2'].action.desiredAcceleration = model_a
    for _ in range(20):
      scc.update(opening, True, False, v, 0., V_CRUISE)
    return scc

  @SIDES
  def test_an_opening_turn_is_handed_back_when_the_model_pulls_away(self, side):
    scc = self.exit_turn(side)
    assert scc.turn_exit and not scc._controlling

  @SIDES
  def test_the_model_not_pulling_away_keeps_the_hold(self, side):
    scc = self.exit_turn(side, model_a=0.)
    assert not scc.turn_exit and scc._controlling

  @SIDES
  def test_a_turn_still_at_its_peak_keeps_the_hold(self, side):
    scc = self.exit_turn(side, k_exit=TIGHT)
    assert not scc.turn_exit and scc._controlling

  @SIDES
  def test_a_corner_that_tightens_again_keeps_the_hold(self, side):
    scc = self.exit_turn(side, k_ahead=TIGHT)
    assert not scc.turn_exit and scc._controlling

  @SIDES
  def test_stock_acc_is_unchanged(self, side):
    assert not self.exit_turn(side, op_long=False).turn_exit

  @SIDES
  def test_faster_than_town_speed_is_unchanged(self, side):
    # 16 m/s (36 mph) in a curve that opens: above _IN_TURN_V_MAX the profile holds as before
    assert not self.exit_turn(side, v=16., k_exit=0.006).turn_exit

  @SIDES
  def test_a_second_bend_past_the_near_window_keeps_the_hold(self, side):
    # review: out of a 3.0 m/s2 turn at 10 m/s, a sharp bend (k 0.1) 33 m on still needs committed braking
    scc = SmartCruiseControlVision(make_cp())
    turn = self.make_sm(10., corner(0.03, 0.03, side), side * 0.03)
    turn['modelV2'].action.desiredAcceleration = 0.5
    for _ in range(20):
      scc.update(turn, True, False, 10., 0., V_CRUISE)
    bend = self.make_sm(10., lambda s: side * (0.1 if s >= 33. else 0.), side * 0.003)
    bend['modelV2'].action.desiredAcceleration = 0.5
    for _ in range(5):
      scc.update(bend, True, False, 10., 0., V_CRUISE)
      assert not scc.turn_exit

  @SIDES
  def test_slowing_in_a_steady_curve_is_not_an_opening_turn(self, side):
    # review: a constant 25 m curve, the car slowed 9.0 -> 8.3 m/s: the lateral acceleration drops, the wheel does not
    scc = SmartCruiseControlVision(make_cp())
    for v in np.linspace(9.0, 8.3, 30):
      sm = self.make_sm(float(v), corner(0.04, 0.04, side), side * 0.04)
      sm['modelV2'].action.desiredAcceleration = 0.3
      scc.update(sm, True, False, float(v), 0., V_CRUISE)
      assert not scc.turn_exit

  @SIDES
  def test_once_handed_back_a_noisy_model_does_not_flip_it(self, side):
    scc = self.exit_turn(side)
    assert scc.turn_exit
    opening = self.make_sm(6.5, corner(0.06, 0., side), side * 0.045)
    for i in range(20):
      opening['modelV2'].action.desiredAcceleration = 0.05 if i % 2 else 0.15
      scc.update(opening, True, False, 6.5, 0., V_CRUISE)
      assert scc.turn_exit
    opening['modelV2'].action.desiredAcceleration = -0.3  # the model brakes: the hold returns
    scc.update(opening, True, False, 6.5, 0., V_CRUISE)
    assert not scc.turn_exit and scc._controlling

  @SIDES
  def test_the_bend_being_left_does_not_count_as_one_ahead(self, side):
    # route 00000166 t=395: under the car the near field still reads the corner, tighter than the speed allows; only
    # bends past the near window block the hand-back
    scc = self.exit_turn(side, v=7.0, k_leaving=TIGHT)
    assert scc.a_required > scc.a_required_far
    assert scc.turn_exit and not scc._controlling
