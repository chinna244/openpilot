"""Mazda: develop's cruise-candidate fixes in StarPilot's planner (gas-override handover, engage settle, gas-release
coast). Only the cruise candidate is lifted; leads, the model and other targets brake as before."""
import numpy as np
import pytest

from opendbc.car.honda.interface import CarInterface as HondaInterface
from opendbc.car.honda.values import CAR as HONDA
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_planner import (ENGAGE_SETTLE_ACCEL, GAS_COAST_ACCEL, GAS_COAST_JERK,
                                                                   LongitudinalPlanner)
from openpilot.starpilot.longitudinal.tests.test_set_speed_assist_planner import mazda_frame, mazda_planner

from openpilot.selfdrive.controls.lib.longitudinal_planner import LongCtrlState
SET = 27.  # m/s


def frame(tick, *, v, a_ego=0., gas=False, brake=False, engaged=True, long_off=False, set_mps=SET, model_accel=0.):
  sm, now = mazda_frame(tick, model_accel=model_accel, plan_v=v)
  sm['carState'].vEgo, sm['carState'].aEgo = v, a_ego
  sm['carState'].vCruise = set_mps * 3.6
  sm['carState'].gasPressed, sm['carState'].brakePressed = gas, brake
  sm['selfdriveState'].enabled = engaged
  sm['selfdriveState'].experimentalMode = False  # the cruise candidate alone, no e2e
  sm['controlsState'].longControlState = LongCtrlState.off if long_off else LongCtrlState.pid
  return sm, now


def run(planner, ticks, start=0, **kwargs):
  out = []
  for tick in range(start, start + ticks):
    sm, now = frame(tick, **kwargs)
    planner.update(sm, now_ns=now, experimental_speed_nudge=False)
    out.append((planner.a_cruise, planner.output_a_target, planner.a_target_published))
  return out


def lifted(planner):
  # the candidate the planner took (output with no lead and no e2e is the cruise candidate)
  return planner.output_a_target


def test_gas_release_over_the_set_speed_coasts_instead_of_braking():
  planner = mazda_planner()
  v = SET + 3.  # 3 m/s over the set speed
  run(planner, 40, v=SET)
  run(planner, 20, start=40, v=v, gas=True, long_off=True)  # gas override up to 3 over
  outs = [o for _, o, _ in run(planner, 40, start=60, v=v)]  # released, still over
  assert planner.gas_coast
  assert min(outs) >= GAS_COAST_ACCEL - 1e-3
  # coasting in at its own jerk from where the state was
  assert max(abs(b - a) for a, b in zip(outs, outs[1:], strict=False)) <= GAS_COAST_JERK * DT_MDL + 0.02
  assert outs[-1] == pytest.approx(GAS_COAST_ACCEL, abs=0.02)


def test_engaging_just_over_the_set_speed_settles_instead_of_braking():
  planner = mazda_planner()
  run(planner, 20, v=SET + 0.6, engaged=False, long_off=True)
  outs = [o for _, o, _ in run(planner, 60, start=20, v=SET + 0.6)]
  assert planner.engage_settle_frames > 0
  assert min(outs) >= ENGAGE_SETTLE_ACCEL - 1e-3


def test_gas_override_seed_does_not_brake_under_the_set_speed():
  # a gas override in a turn reads -0.6 measured accel; the cruise state restarts there
  planner = mazda_planner()
  run(planner, 40, v=SET - 5.)
  published = [p for _, _, p in run(planner, 10, start=40, v=SET - 5., a_ego=-0.6, gas=True, long_off=True)]
  # the published plan during the override asks what the target asks (never braking under the set speed)
  assert min(published) >= -1e-3
  outs = [o for _, o, _ in run(planner, 5, start=50, v=SET - 5.)]
  assert min(outs) >= -1e-3


def test_other_targets_brake_as_they_ask():
  # a lowered set speed (the driver's own) ends the coast; the candidate then follows the state, not the coast floor
  planner = mazda_planner()
  run(planner, 40, v=SET)
  run(planner, 20, start=40, v=SET + 3., gas=True, long_off=True)
  run(planner, 5, start=60, v=SET + 3.)
  assert planner.gas_coast
  run(planner, 1, start=65, v=SET + 3., set_mps=SET - 3.)
  assert not planner.gas_coast


def test_other_brands_keep_starpilot_behaviour():
  params = HondaInterface.get_non_essential_params(HONDA.HONDA_CIVIC)
  params.openpilotLongitudinalControl = True
  planner = LongitudinalPlanner(params, init_v=SET)
  assert not planner.cruise_handover
  run(planner, 40, v=SET)
  run(planner, 20, start=40, v=SET + 3., gas=True, long_off=True)
  outs = [o for _, o, _ in run(planner, 40, start=60, v=SET + 3.)]
  assert not planner.gas_coast and min(outs) < GAS_COAST_ACCEL - 0.1
  assert np.isfinite(outs).all()
