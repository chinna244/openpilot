"""Mazda: develop's Set Speed Assist owns StarPilot's experimental speed nudge inside the native planner."""
import numpy as np
import pytest

from opendbc.car.honda.interface import CarInterface as HondaInterface
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.mazda.interface import CarInterface as MazdaInterface
from opendbc.car.mazda.values import CAR as MAZDA
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.starpilot.longitudinal.set_speed_assist import Inhibit
from openpilot.starpilot.longitudinal.tests.test_encouragement import frame

T_IDXS = np.asarray(ModelConstants.T_IDXS)
V_EGO = 20.


def mazda_planner():
  params = MazdaInterface.get_non_essential_params(MAZDA.MAZDA_CX5_2022)
  params.openpilotLongitudinalControl = True
  return LongitudinalPlanner(params, init_v=V_EGO)


def mazda_frame(tick, *, model_accel=0., plan_v=V_EGO, lane_change=False):
  sm, now = frame(tick)
  sm['carState'].vEgo = V_EGO
  sm['carState'].aEgo = model_accel
  sm['carState'].vCruise = 27. * 3.6
  md = sm['modelV2']
  md.action.desiredAcceleration = model_accel
  vel = np.broadcast_to(np.asarray(plan_v, dtype=float), T_IDXS.shape)
  md.velocity.x = vel.tolist()
  md.position.x = np.concatenate(([0.], np.cumsum(np.diff(T_IDXS) * (vel[1:] + vel[:-1]) / 2.))).tolist()
  md.orientationRate.z = [0.] * len(T_IDXS)
  md.meta.disengagePredictions.gasPressProbs = [1., 1., 1.]
  if lane_change:
    md.meta.laneChangeState = 'laneChangeStarting'
  return sm, now


def drive(planner, ticks, start=0, **kwargs):
  nudge = kwargs.pop('nudge', True)
  for tick in range(start, start + ticks):
    sm, now = mazda_frame(tick, **kwargs)
    planner.update(sm, now_ns=now, experimental_speed_nudge=nudge)
    assert planner.mpc.solution_status == 0
    assert planner.output_a_target <= planner.a_cruise + 1e-6
  return planner


def test_only_mazda_gets_set_speed_assist():
  assert mazda_planner().set_speed_assist is not None
  params = HondaInterface.get_non_essential_params(HONDA.HONDA_CIVIC)
  assert LongitudinalPlanner(params).set_speed_assist is None


def test_hesitant_model_is_lifted_toward_the_set_speed():
  planner = drive(mazda_planner(), 120)
  ssa = planner.set_speed_assist
  assert ssa.inhibit == Inhibit.none
  assert ssa.lift > 0.3
  assert planner.output_a_target == pytest.approx(ssa.lift, abs=0.05)
  # never past chill mode's answer
  assert ssa.lift <= ssa.floor + 1e-6


def test_nudge_off_leaves_the_model():
  planner = drive(mazda_planner(), 120, nudge=False)
  assert planner.set_speed_assist.inhibit == Inhibit.disabled
  assert planner.set_speed_assist.lift == 0.
  assert planner.output_a_target == pytest.approx(0., abs=1e-6)


def test_planned_slowdown_holds_the_lift_off():
  # the plan sheds 6 m/s over its horizon: the model is slowing for something, no lift
  planner = drive(mazda_planner(), 120, plan_v=np.maximum(V_EGO - 6. * np.minimum(T_IDXS, 10.) / 10., 0.))
  assert planner.set_speed_assist.lift == 0.


def test_lane_change_lets_go():
  planner = drive(mazda_planner(), 120)
  lift = planner.set_speed_assist.lift
  drive(planner, 1, start=120, lane_change=True)
  assert planner.set_speed_assist.inhibit == Inhibit.laneChange
  assert planner.set_speed_assist.lift < lift
