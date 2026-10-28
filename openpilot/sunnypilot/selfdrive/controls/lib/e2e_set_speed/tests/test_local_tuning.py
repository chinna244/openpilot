"""
This fork's tuning on Set Speed Assist (Mazda CX-5): the floor's approach and ceiling, the lift's rise
and town cap, and plan-lead braking. Values are pinned as literals.
"""
import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL as DT
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed import controller as c
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import E2ESetSpeedController, Inhibit
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.tests.helpers import ENGAGED, T_IDXS, build_sm

V_EGO = 20.
SETTLE = 400  # frames, 20 s


def run(ctl, n, sm, a_model=0., a_chill=2., v_cruise=V_EGO + 10.):  # 10 m/s under at V_EGO
  out = a_model
  for _ in range(n):
    out = ctl.update(sm, a_model, a_chill, **ENGAGED, v_cruise=v_cruise)
  return out


def new_controller():
  return E2ESetSpeedController(params=FakeParams(ExperimentalModeSetSpeed=True), dt=DT)


def _plan(f):
  return np.asarray([f(t) for t in T_IDXS])


# town values to 15 m/s, highway from 25 m/s; below 15.6 m/s the town cap (0.6 to 12.5 m/s) holds the lift under the floor
@pytest.mark.parametrize("v, rise, ceiling, lift", [(8., 0.5, 0.8, 0.6), (15., 0.5, 0.8, 0.76129), (20., 0.375, 0.65, 0.65),
                                                    (25., 0.25, 0.5, 0.5), (30., 0.25, 0.5, 0.5)])
def test_floor_ceiling_and_rise_by_speed(v, rise, ceiling, lift):
  ctl = new_controller()
  lifts = [run(ctl, 1, build_sm(v), v_cruise=v + 10.) for _ in range(SETTLE)]
  assert lifts[0] == pytest.approx(rise * DT)
  assert max(np.diff([0.] + lifts)) <= rise * DT + 1e-9
  assert ctl.floor == pytest.approx(ceiling)
  assert lifts[-1] == pytest.approx(lift, abs=1e-5)


@pytest.mark.parametrize("gap, floor", [(1., 0.25), (2., 0.5), (0., 0.), (-3., 0.)])
def test_floor_approaches_the_cruise_target_over_four_seconds(gap, floor):
  ctl = new_controller()
  run(ctl, SETTLE, build_sm(V_EGO), v_cruise=V_EGO + gap)
  assert ctl.floor == pytest.approx(floor)


def test_floor_never_above_chill():
  ctl = new_controller()
  run(ctl, SETTLE, build_sm(V_EGO), a_chill=0.3)
  assert ctl.floor == pytest.approx(0.3)


@pytest.mark.parametrize("v, lift", [(8., 0.6), (12.5, 0.6), (14.05, 0.7)])
def test_town_lift_held_over_a_hesitant_model(v, lift):
  ctl = new_controller()
  out = run(ctl, SETTLE, build_sm(v), a_model=0.1, v_cruise=v + 10.)
  assert ctl.lift == pytest.approx(lift)
  assert out == pytest.approx(0.1 + lift)


# Plan-lead braking (route 0000016a t=153-158)
SLOWING_TO_5MPH = _plan(lambda t: max(V_EGO - 1.0 * t, 2.2))  # a constant -1.0 m/s^2 down to 5 mph
ARM = 10  # frames, 0.5 s


def test_brakes_ahead_of_the_model_once_the_plan_holds():
  ctl = new_controller()
  sm = build_sm(V_EGO, plan_v=SLOWING_TO_5MPH)
  # a model already easing (-0.4) takes no lift, so only plan-lead braking moves the candidate
  assert run(ctl, ARM - 1, sm, a_model=-0.4, v_cruise=V_EGO) == pytest.approx(-0.4)  # not before 0.5 s
  outs = [run(ctl, 1, sm, a_model=-0.4, v_cruise=V_EGO) for _ in range(100)]
  assert np.diff(outs).min() >= -1.0 * DT - 1e-9  # eased in at 1 m/s^3
  assert outs[-1] == pytest.approx(0.8 * -1.0, abs=0.05)


def test_never_brakes_less_than_the_model():
  ctl = new_controller()
  assert run(ctl, ARM + 100, build_sm(V_EGO, plan_v=SLOWING_TO_5MPH), a_model=-2.0, v_cruise=V_EGO) == pytest.approx(-2.0)


@pytest.mark.parametrize("lead, v_ego", [(True, V_EGO), (False, 4.5)])
def test_a_lead_or_walking_pace_leaves_the_stop_to_the_model(lead, v_ego):
  ctl = new_controller()
  plan = _plan(lambda t: max(v_ego - 1.0 * t, 0.))
  assert run(ctl, ARM + 100, build_sm(v_ego, plan_v=plan, lead=lead), a_model=-0.1, v_cruise=V_EGO) >= -0.1


def test_lets_go_at_the_release_rate_when_the_plan_clears():
  ctl = new_controller()
  run(ctl, ARM + 100, build_sm(V_EGO, plan_v=SLOWING_TO_5MPH), a_model=0.3, v_cruise=V_EGO)
  assert ctl.plb_active
  outs = [run(ctl, 1, build_sm(V_EGO), a_model=0.3, v_cruise=V_EGO) for _ in range(100)]
  assert outs[0] < -0.5
  assert np.diff(outs).max() <= 1.0 * DT + 1e-9
  assert not ctl.plb_active


def test_arming_needs_a_consecutive_half_second():
  ctl = new_controller()
  for _ in range(4 * ARM):  # the need flickers in and out of -0.6
    run(ctl, 1, build_sm(V_EGO, plan_v=SLOWING_TO_5MPH), a_model=0., v_cruise=V_EGO)
    run(ctl, 1, build_sm(V_EGO, plan_v=_plan(lambda t: max(V_EGO - 0.4 * t, 2.2))), a_model=0., v_cruise=V_EGO)
  assert not ctl.plb_armed


def test_an_invalid_model_frame_eases_out_instead_of_dropping():
  ctl = new_controller()
  run(ctl, ARM + 100, build_sm(V_EGO, plan_v=SLOWING_TO_5MPH), a_model=-0.1, v_cruise=V_EGO)
  before = run(ctl, 1, build_sm(V_EGO, plan_v=SLOWING_TO_5MPH), a_model=-0.1, v_cruise=V_EGO)
  out = run(ctl, 1, build_sm(V_EGO, plan_v=np.nan), a_model=-0.1, v_cruise=V_EGO)
  assert ctl.inhibit == Inhibit.invalid
  assert out == pytest.approx(before + 1.0 * DT, abs=1e-6)


def test_plan_decel_reads_the_plan():
  pos = np.concatenate(([0.], np.cumsum(np.diff(T_IDXS) * (SLOWING_TO_5MPH[1:] + SLOWING_TO_5MPH[:-1]) / 2.)))
  assert c.plan_decel(SLOWING_TO_5MPH, pos) == pytest.approx(-1.0, abs=0.05)


# review: a hesitant model under a gently slowing plan (-0.45 m/s^2 to 10 m/s) was lifted +0.6 into the slowdown
EASING = _plan(lambda t: max(V_EGO - 0.45 * t, 10.))


def test_no_lift_once_the_plan_holds_a_slowdown():
  ctl = new_controller()
  run(ctl, SETTLE, build_sm(V_EGO))  # settled on the floor
  assert ctl.lift == pytest.approx(0.65)
  sm = build_sm(V_EGO, plan_v=EASING)
  run(ctl, ARM - 1, sm, a_model=-0.15)
  assert ctl.lift > 0.5  # not before 0.5 s (gain 0.75 at -0.15 eases it toward 0.6)
  run(ctl, 20, sm, a_model=-0.15)
  assert ctl.lift == 0.
  assert run(ctl, 1, sm, a_model=-0.15) == pytest.approx(-0.15)


def test_switching_off_mid_brake_eases_out():
  ctl = new_controller()
  sm = build_sm(V_EGO, plan_v=SLOWING_TO_5MPH)
  before = run(ctl, ARM + 100, sm, a_model=-0.4, v_cruise=V_EGO)
  ctl.params = FakeParams(ExperimentalModeSetSpeed=False)
  ctl.frame = -1  # the next frame reads the param
  outs = [run(ctl, 1, sm, a_model=-0.4, v_cruise=V_EGO) for _ in range(40)]
  assert outs[0] == pytest.approx(before + 1.0 * DT, abs=1e-6)
  assert np.diff(outs).max() <= 1.0 * DT + 1e-9
  assert outs[-1] == pytest.approx(-0.4) and not ctl.plb_active
