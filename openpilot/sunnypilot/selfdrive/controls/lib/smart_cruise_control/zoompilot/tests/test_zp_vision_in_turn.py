"""
In-turn braking: once the car is turning through a town corner on openpilot long, the vision
controller brakes no harder than _IN_TURN_A_MIN, unless the turn past the actuation lead would put
the car over _IN_TURN_ESCAPE_LAT. The approach, faster curves, stock ACC and tightening turns brake
as before.
"""
from unittest import mock

import numpy as np
import pytest

from opendbc.car import structs
from openpilot.common.realtime import DT_MDL
from opendbc.car.interfaces import ACCEL_MAX
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import OP_LONG_A_BUDGET, op_long_cruise_jerk

from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import vision_controller
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.tests.vision_harness import VisionCase, make_cp

A_MIN = vision_controller._IN_TURN_A_MIN
SIDES = pytest.mark.parametrize("side", [1., -1.], ids=["left", "right"])
V_CRUISE = 20.


def host_cruise(v_target, v_car, a_prev):
  """The host's cruise candidate on the SCC target: upstream's, then rebuilt on SCC's budget while SCC is the target."""
  a = get_cruise_accel(v_target, v_car, a_prev, DT_MDL)
  if v_target >= V_CRUISE:
    return a
  j = op_long_cruise_jerk(v_car) * DT_MDL
  return min(a, float(np.clip(np.clip(v_target - v_car, -OP_LONG_A_BUDGET, ACCEL_MAX), a_prev - j, a_prev + j)))


def corner(k_near: float, k_after: float, side: float, d_split: float = 4.0):
  """The turn at the car, then what follows past d_split (inside the actuation lead)."""
  return lambda s: side * (k_near if s < d_split else k_after)


def uncapped():
  """The controller as it was before the in-turn cap."""
  return mock.patch.object(vision_controller, '_IN_TURN_A_MIN', -99.)


class TestInTurnBraking(VisionCase):

  def drive(self, v, kappa_fn, cur_curvature, op_long=True, v_car=None):
    """Hardest braking over 2 s on a fresh controller: its acceleration seed, or the host's cruise
    candidate built from its speed target, whichever is lower (the planner takes the minimum).
    v is the planner's filtered speed the controller gets, v_car the carState.vEgo the host's cruise
    candidate is built on."""
    v_car = v if v_car is None else v_car
    scc = SmartCruiseControlVision(make_cp(op_long))
    sm = self.make_sm(v, kappa_fn, cur_curvature, v_car=v_car)
    a_min = a_cruise = 0.
    for _ in range(40):
      scc.update(sm, True, False, v, 0., V_CRUISE)
      a_cruise = host_cruise(scc.output_v_target, v_car, a_cruise)
      a_min = min(a_min, scc.output_a_target, a_cruise)
    return a_min

  def same_as_before(self, *args, **kwargs):
    a = self.drive(*args, **kwargs)
    with uncapped():
      return a == pytest.approx(self.drive(*args, **kwargs))

  @SIDES
  def test_turning_through_a_town_corner_brakes_gently(self, side):
    # 8 m/s (18 mph) in a 20 m corner: 3.2 m/s2 now and past the lead, over the 2.8 ceiling
    assert self.drive(8., corner(0.05, 0.05, side), side * 0.05) == pytest.approx(A_MIN)

  @SIDES
  @pytest.mark.parametrize("v_car", [7.7, 8.3], ids=["filter_ahead", "filter_behind"])
  def test_cap_holds_when_the_filtered_speed_lags_the_car(self, side, v_car):
    # the planner's speed filter trails carState.vEgo by 0.3 m/s through a corner (route-like actuator lag)
    assert self.drive(8., corner(0.05, 0.05, side), side * 0.05, v_car=v_car) >= A_MIN - 1e-6

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


def test_planner_cruise_candidate_holds_the_cap_with_actuator_lag():
  """The full planner in a 20 m corner at 18 mph, the car lagging the request by its 0.36 s delay and a
  0.5 s response: the filtered speed handed to the controller falls behind carState.vEgo, and the cruise
  candidate built on vEgo used to reach -0.81."""
  from collections import deque
  from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
  from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams
  from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_experimental_gate import build_planner, build_sm
  from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import make_smart_cruise_control
  from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.tests.vision_harness import model_for_road

  kappa = 0.05
  p = build_planner(False)
  p.CP.brand = 'mazda'
  p.CP.openpilotLongitudinalControl = True
  p.CP.longitudinalActuatorDelay = 0.36
  p.scc = make_smart_cruise_control(p.CP)
  p.scc.vision.params = FakeParams(SmartCruiseControlVision=True)
  p.scc.map.params = FakeParams(SmartCruiseControlMap=False)
  p.sla_machine = None
  p.e2e_set_speed.params = FakeParams(ExperimentalModeSetSpeed=False)
  p.v_desired_filter.x = 8.
  sm = build_sm(False)
  cs_ctrl = sm['controlsState'].as_builder()
  cs_ctrl.longControlState, cs_ctrl.curvature = LongCtrlState.pid, kappa
  sm['controlsState'] = cs_ctrl.as_reader()

  v, a, delayed = 8., 0., deque([0.] * 7)
  lagged = capped = 0
  for _ in range(120):
    cs = sm['carState'].as_builder()
    cs.vEgo, cs.aEgo, cs.vCruise, cs.vCruiseCluster = v, a, 72., 72.
    sm['carState'] = cs.as_reader()
    model = model_for_road(v, lambda s: kappa).modelV2
    model.action.desiredAcceleration = 0.
    sm['modelV2'] = model
    p.update(sm)
    vc = p.scc.vision
    if vc._in_turn_capped and vc.is_active:
      capped += 1
      lagged += vc.v_ego < v - 0.2
      assert p.a_cruise >= A_MIN - 1e-6
      assert p.output_a_target >= A_MIN - 1e-6
    delayed.append(float(p.output_a_target))
    a += DT_MDL / 0.5 * (delayed.popleft() - a)
    v += a * DT_MDL
  assert capped > 100 and lagged > 0
