"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The controller is exercised with synthetic roads: kappa(s) profiles rendered into the
time-indexed model arrays the way the model would report them at a given speed. The
activation distances asserted here follow from the platform limits in limits.py; if
those constants move, the geometry in these tests moves with them.
"""
from typing import Any

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.modeld.constants import ModelConstants
from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision
from openpilot.common.test import OpenpilotTestCase

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState

V_EGO = 20.
SETPOINT = 20.
CURVE_KAPPA = 0.02  # r = 50 m -> allowed 10 m/s at the 2.0 ceiling
CURVE_V = 10.


def make_cp(op_long: bool = True) -> structs.CarParams:
  return structs.CarParams(brand="mazda", openpilotLongitudinalControl=op_long,
                           longitudinalActuatorDelay=0.36)


def model_for_road(v: float, kappa_fn, v_model: float | None = None):
  """Render kappa(s) into model arrays as the model would report driving it at speed v.

  v_model lets the model's own velocity plan differ from v (a planned slowdown); the yaw
  rate follows the planned velocity, exactly as the model reports it.
  """
  t = np.array(ModelConstants.T_IDXS)
  s = v * t
  vm = v if v_model is None else v_model

  model = messaging.new_message('modelV2')
  position = log.XYZTData.new_message()
  position.x = [float(si) for si in s]
  position.y = [0.0] * len(t)
  model.modelV2.position = position
  velocity = log.XYZTData.new_message()
  velocity.x = [float(vm)] * len(t)
  model.modelV2.velocity = velocity
  orientation_rate = log.XYZTData.new_message()
  orientation_rate.z = [float(kappa_fn(si) * vm) for si in s]
  model.modelV2.orientationRate = orientation_rate
  return model


def curve_at(d_curve: float, kappa: float = CURVE_KAPPA):
  return lambda s: kappa if s >= d_curve else 0.


class TestSmartCruiseControlVision(OpenpilotTestCase):

  def setup_method(self):
    self.params = Params()
    self.params.put_bool("SmartCruiseControlVision", True, block=True)
    self.scc_v = SmartCruiseControlVision(make_cp())

  def make_sm(self, v: float, kappa_fn, cur_curvature: float = 0., v_model: float | None = None) -> Any:
    controls_state = messaging.new_message('controlsState')
    controls_state.controlsState.curvature = float(cur_curvature)
    return {'modelV2': model_for_road(v, kappa_fn, v_model).modelV2,
            'controlsState': controls_state.controlsState}

  def run_road(self, v: float, kappa_fn, n: int = 3, cur_curvature: float = 0.,
               v_model: float | None = None, setpoint: float = SETPOINT,
               enabled: bool = True, override: bool = False, scc=None):
    scc = scc or self.scc_v
    sm = self.make_sm(v, kappa_fn, cur_curvature, v_model)
    for _ in range(n):
      scc.update(sm, enabled, override, v, 0., setpoint)
    return scc

  # -- lifecycle -------------------------------------------------------------

  def test_initial_state(self):
    assert self.scc_v.state == VisionState.disabled
    assert not self.scc_v.is_active
    assert self.scc_v.output_v_target == V_CRUISE_UNSET
    assert self.scc_v.output_a_target == 0.

  def test_param_disable(self):
    self.params.put_bool("SmartCruiseControlVision", False, block=True)
    self.scc_v.enabled = False
    self.run_road(V_EGO, curve_at(50.))
    assert self.scc_v.state == VisionState.disabled

  def test_long_disabled(self):
    self.run_road(V_EGO, curve_at(50.), enabled=False)
    assert self.scc_v.state == VisionState.disabled
    assert self.scc_v.output_v_target == V_CRUISE_UNSET

  def test_override_suspends_control(self):
    self.run_road(V_EGO, curve_at(50.), override=True)
    assert self.scc_v.state == VisionState.overriding
    assert self.scc_v.output_v_target == V_CRUISE_UNSET

  # -- hold the set speed ----------------------------------------------------

  def test_straight_road_never_acts(self):
    self.run_road(V_EGO, lambda s: 0., n=10)
    assert self.scc_v.state == VisionState.enabled
    assert self.scc_v.output_v_target == V_CRUISE_UNSET
    assert self.scc_v.a_required == 0.

  def test_distant_curve_holds_set_speed(self):
    # a 12.6 m/s curve 185 m out needs only ~0.73 m/s2 from here; below the 0.7 * 1.2 commit
    self.run_road(V_EGO, curve_at(185., kappa=0.012))
    assert self.scc_v.state == VisionState.enabled
    assert self.scc_v.output_v_target == V_CRUISE_UNSET
    assert 0. < self.scc_v.a_required < 0.84

  # -- brake at the budget ---------------------------------------------------

  def test_curve_inside_braking_distance_engages(self):
    self.run_road(V_EGO, curve_at(100.))
    assert self.scc_v.state == VisionState.entering
    assert self.scc_v.is_active
    # target leads v_ego by the required decel, capped by the profile
    assert MIN_V < self.scc_v.output_v_target < V_EGO - 0.5
    assert self.scc_v.output_a_target < 0.

  def test_a_target_is_jerk_ramped(self):
    sm = self.make_sm(V_EGO, curve_at(100.))
    prev = 0.
    j = self.scc_v.limits.jerk(V_EGO)
    for i in range(45):
      self.scc_v.update(sm, True, False, V_EGO, 0., SETPOINT)
      a = self.scc_v.output_a_target
      if i:
        assert a <= prev + 1e-9
        assert prev - a <= j * DT_MDL + 1e-6
      prev = a
    assert prev < -1.0  # converged to a real decel request, not the old smear

  def test_planned_slowdown_does_not_lower_the_estimate(self):
    # The old lat-acc form used the model's velocity plan, so a planned slowdown lowered
    # the prediction below the abort threshold mid-braking. Geometry divides it back out.
    self.run_road(V_EGO, curve_at(100.), v_model=0.7 * V_EGO)
    assert self.scc_v.is_active

  def test_slowing_toward_the_curve_stays_committed(self):
    self.run_road(V_EGO, curve_at(100.))
    assert self.scc_v.is_active
    self.run_road(14., curve_at(40.), n=1)
    assert self.scc_v.is_active
    assert self.scc_v.solver_active

  # -- arrive at the correct speed -------------------------------------------

  def test_holds_allowed_speed_inside_the_curve(self):
    # approach a touch fast, curve at the bumper
    self.run_road(12., curve_at(0.), cur_curvature=CURVE_KAPPA)
    assert self.scc_v.is_active
    # settled at the allowed speed: hold it, do not re-accelerate toward the setpoint
    self.run_road(CURVE_V, curve_at(0.), cur_curvature=CURVE_KAPPA, n=2)
    assert self.scc_v.state == VisionState.turning
    assert abs(self.scc_v.output_v_target - CURVE_V) < 1.0

  def test_releases_when_the_road_straightens(self):
    self.run_road(12., curve_at(0.), cur_curvature=CURVE_KAPPA)
    assert self.scc_v.is_active
    self.run_road(CURVE_V, lambda s: 0., cur_curvature=0., n=3)
    assert self.scc_v.state == VisionState.enabled
    assert self.scc_v.output_v_target == V_CRUISE_UNSET

  def test_hairpin_floors_at_min_v(self):
    # kappa 0.12 allows 4.1 m/s, below the 20 km/h operating floor
    self.run_road(6., curve_at(0., kappa=0.12), cur_curvature=0.12)
    assert self.scc_v.is_active
    assert self.scc_v.output_v_target == MIN_V

  # -- per-path budgets ------------------------------------------------------

  def test_stock_path_commits_earlier_and_prepositions_the_dip(self):
    # same road: op-long (1.2 budget, 0.36 s lead) still holds; stock ACC (0.75 budget,
    # response + dash traversal lead) is already inside its braking distance
    road = curve_at(185., kappa=0.012)
    self.run_road(V_EGO, road)
    assert not self.scc_v.is_active

    stock = SmartCruiseControlVision(make_cp(op_long=False))
    self.run_road(V_EGO, road, scc=stock)
    assert stock.is_active
    # the dash cannot track a profile; it gets sent to the dip itself
    v_dip = (2.0 * 0.95 / 0.012) ** 0.5
    assert abs(stock.output_v_target - v_dip) < 1.0


class TestLookaheadWire(OpenpilotTestCase):
  """v_ahead_min feeds the ICBM restore gate: 0 must mean exactly "no lookahead"."""

  def setup_method(self):
    self.params = Params()
    self.params.put_bool("SmartCruiseControlVision", True, block=True)
    self.scc_v = SmartCruiseControlVision(make_cp())

  def step(self, v=V_EGO, kappa_fn=lambda s: 0., long_enabled=True):
    sm = {'modelV2': model_for_road(v, kappa_fn).modelV2,
          'controlsState': messaging.new_message('controlsState').controlsState}
    self.scc_v.update(sm, long_enabled, False, v, 0., SETPOINT)

  def test_clear_road_caps_at_unset(self):
    self.step()
    assert self.scc_v.v_ahead_min == 255.

  def test_dip_passes_through(self):
    self.step(kappa_fn=curve_at(60.))
    assert 0. < self.scc_v.v_ahead_min < SETPOINT

  def test_long_disabled_reports_no_lookahead(self):
    self.step(kappa_fn=curve_at(60.))
    self.step(long_enabled=False)
    assert self.scc_v.v_ahead_min == 0.

  def test_toggle_off_reports_no_lookahead(self):
    self.params.put_bool("SmartCruiseControlVision", False, block=True)
    scc = SmartCruiseControlVision(make_cp())
    sm = {'modelV2': model_for_road(V_EGO, curve_at(60.)).modelV2,
          'controlsState': messaging.new_message('controlsState').controlsState}
    scc.update(sm, True, False, V_EGO, 0., SETPOINT)
    assert scc.v_ahead_min == 0.
