"""Mazda: develop's planner around the MPC (chinna244/openpilot 197598c0f6). The model candidate (curve offset, Set Speed
Assist, firm stop) is always in the minimum and the cruise candidate follows develop's law, whatever experimentalMode
says; the MPC follows at develop's town gap; StarPilot's AccelBoost is not used. Other brands keep StarPilot's planner."""
import numpy as np
import pytest

from opendbc.car.honda.interface import CarInterface as HondaInterface
from opendbc.car.honda.values import CAR as HONDA
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
from openpilot.selfdrive.controls.lib.curve_speed import MAX_ACCEL_OFFSET
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import CITY_T_FOLLOW_FACTOR, get_T_FOLLOW
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, LongitudinalPlanSource
from openpilot.selfdrive.controls.tests import test_cruise_handover as H
from openpilot.starpilot.longitudinal.tests.test_set_speed_assist_planner import T_IDXS, mazda_frame, mazda_planner


def honda():
  params = HondaInterface.get_non_essential_params(HONDA.HONDA_CIVIC)
  params.openpilotLongitudinalControl = True
  return LongitudinalPlanner(params, init_v=H.V_EGO)


def run(p, n=40, experimental=False, **kwargs):
  sm = None
  for _ in range(n):
    sm = H.build_sm(**kwargs)
    sm['selfdriveState'] = sm['selfdriveState'].as_builder()
    sm['selfdriveState'].experimentalMode = experimental
    sm['selfdriveState'] = sm['selfdriveState'].as_reader()
    p.update(sm)
  return sm


@pytest.mark.parametrize("experimental", [False, True])
def test_the_model_brakes_whatever_experimental_mode_says(experimental):
  p = H.planner()
  run(p, experimental=experimental, model_a=-1.0)
  assert p.mpc.source == LongitudinalPlanSource.e2e
  assert p.output_a_target == pytest.approx(-1.0, abs=0.05)


def test_other_brands_take_the_model_only_in_experimental_mode():
  p = honda()
  p.update = lambda sm, update=p.update: update(sm, now_ns=1)
  run(p, model_a=-1.0)
  assert p.output_a_target > -0.6
  assert p.firm_stop is None and p.curve_speed is None and p.scc is None and not p.mazda


def test_set_speed_assist_lifts_without_experimental_mode():
  planner = mazda_planner()
  for tick in range(120):
    sm, now = mazda_frame(tick)
    sm['selfdriveState'].experimentalMode = False
    planner.update(sm, now_ns=now, experimental_speed_nudge=True)
  assert planner.set_speed_assist.lift > 0.3
  assert planner.output_a_target == pytest.approx(planner.set_speed_assist.lift, abs=0.05)


def test_cruise_law_is_develops():
  # 3 m/s under the set speed from rest: develop smooths the error over 1 s and the jerk limit binds, no lateral or
  # response-time shaping, the same in and out of experimental mode
  for experimental in (False, True):
    p = H.planner()
    p.a_cruise = p.output_a_target = 0.
    run(p, n=1, experimental=experimental, v_cruise_kph=(H.V_EGO + 3.) * 3.6, a_ego=0., model_a=2.)
    expected = lp.get_mazda_cruise_accel(H.V_EGO + 3., H.V_EGO, 0., 0., DT_MDL)
    assert p.a_cruise == pytest.approx(expected)
  # the law itself: develop's get_cruise_accel
  a = lp.get_mazda_cruise_accel(30., 20., 0.3, 0.1, DT_MDL)
  alpha = 1 - np.exp(-DT_MDL)
  j = np.interp(20., lp.A_CRUISE_MAX_BP, lp.J_CRUISE_VALS) * DT_MDL
  assert a == pytest.approx(float(np.clip(np.clip(alpha * 10. + (1 - alpha) * 0.3, -0.5, lp.ACCEL_MAX), 0.1 - j, 0.3 + j)))


def test_accel_boost_is_not_used():
  p = H.planner()
  p.accel_boost.total_boost = 0.2
  run(p, model_a=0.)
  assert p.output_a_target <= p.set_speed_assist.lift + 1e-6


@pytest.mark.parametrize("personality,boost", [(log.LongitudinalPersonality.standard, 0.),
                                               (log.LongitudinalPersonality.aggressive, lp.AGGRESSIVE_ACCEL_BOOST)])
def test_develops_aggressive_boost_only_with_set_speed_assist_off(personality, boost):
  p = H.planner()
  for nudge in (False,):
    for _ in range(3):
      sm = H.build_sm(model_a=-0.2, v_cruise_kph=H.V_EGO * 3.6 + 50.)
      sm['selfdriveState'] = sm['selfdriveState'].as_builder()
      sm['selfdriveState'].personality = personality
      sm['selfdriveState'] = sm['selfdriveState'].as_reader()
      p.update(sm, experimental_speed_nudge=nudge)
  # 6 m/s: the boost is 0.2 of the way in
  assert p.output_a_target == pytest.approx(-0.2 + boost * np.interp(H.V_EGO, [5., 10.], [0., 1.]), abs=1e-6)


def test_curve_offset_brakes_the_model_ahead_of_a_tight_bend():
  p = H.planner()
  sm = H.build_sm(model_a=0.)
  md = sm['modelV2'].as_builder()
  md.orientationRate.z = [0.6] * 33  # 6 m/s * 0.6 rad/s = 3.6 m/s^2 lateral
  md.position.t = [float(t) for t in T_IDXS]
  sm['modelV2'] = md.as_reader()
  for _ in range(40):
    p.update(sm)
  assert p.curve_speed.accel_offset == pytest.approx(-MAX_ACCEL_OFFSET)
  assert p.output_a_target <= -MAX_ACCEL_OFFSET + 1e-6


@pytest.mark.parametrize("v,factor", [(6., CITY_T_FOLLOW_FACTOR), (20., 1.)])
def test_town_following_gap(v, factor):
  p = H.planner()
  p.v_desired_filter.x = v
  p.update(H.build_sm(lead=60.))
  personality_gap = get_T_FOLLOW(log.LongitudinalPersonality.standard)
  # the MPC starts from the filtered speed (here between the measured 6 m/s and v)
  assert float(p.mpc.params[0, 4]) == pytest.approx(lp.get_T_FOLLOW(log.LongitudinalPersonality.standard) *
                                                     float(np.interp(p.mpc.x0[1], [11.2, 15.6], [CITY_T_FOLLOW_FACTOR, 1.])))
  assert float(p.mpc.params[0, 4]) <= personality_gap
  q = honda()
  q.update(H.build_sm(lead=60.), now_ns=1)
  assert float(q.mpc.params[0, 4]) == pytest.approx(personality_gap)
