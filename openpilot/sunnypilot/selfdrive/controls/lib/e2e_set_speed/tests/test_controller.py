"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import numpy as np
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import DT_MDL as DT
from openpilot.selfdrive.controls.lib import longitudinal_planner as upstream
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed import controller as c
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import E2ESetSpeedController, Inhibit
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.tests.helpers import ENGAGED, T_IDXS, build_sm, plan_pos

V_EGO = 20.
V_CRUISE = 23.  # floor (23 - 20) / 4 = 0.75, under FLOOR_MAX
HOLD_FRAMES = round(c.HOLD_TIME / DT)
SETTLE_FRAMES = round(c.FLOOR_MAX / (c.BOOST_RATE * DT)) + 10


def run(ctl, n, sm, a_model=0., v_cruise=V_CRUISE, deliver=False, **kwargs):
  """deliver: the planner drives the e2e candidate, so the boost reaches the car."""
  args = {**ENGAGED, **kwargs}
  out = a_model
  for _ in range(n):
    out = ctl.update(sm, a_model, v_cruise, **args)
    if deliver:
      ctl.delivered(out, out)
  return out


def new_controller(enabled=True):
  return E2ESetSpeedController(params=FakeParams(ExperimentalModeSetSpeed=enabled), dt=DT)


def settled(v_cruise=V_CRUISE, **kwargs):
  ctl = new_controller()
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO), v_cruise=v_cruise, **kwargs)
  assert ctl.boost == pytest.approx(ctl.floor)
  return ctl


def _plan(f):
  return np.asarray([f(t) for t in T_IDXS])


def bound(vel, yaw=None, v_target=V_CRUISE, added=0.):
  vel = np.asarray(vel, dtype=float)
  yaw = np.zeros(len(T_IDXS)) if yaw is None else yaw
  return c.envelope_bound(vel, yaw, plan_pos(vel), v_target, added)


STOPPING = _plan(lambda t: max(V_EGO - 2. * t, 0.))
BEND_AHEAD = _plan(lambda t: 1.5 / V_EGO if 5. <= t <= 8. else 0.)  # 1.5 m/s^2 at today's speed


def test_friction_circle_matches_upstream():
  assert c.A_TOTAL_MAX_BP == upstream._A_TOTAL_MAX_BP
  assert c.A_TOTAL_MAX_V == upstream._A_TOTAL_MAX_V


def test_disabled_passes_model_through():
  ctl = new_controller(enabled=False)
  assert run(ctl, 200, build_sm(V_EGO), a_model=0.1) == 0.1
  assert ctl.inhibit == Inhibit.disabled


@pytest.mark.parametrize("kwargs, reason", [
  ({"is_e2e": False}, Inhibit.inactive),
  ({"reset_state": True}, Inhibit.inactive),
  ({"dec_active": True}, Inhibit.decActive),
])
def test_idle_resets_at_once(kwargs, reason):
  ctl = settled(deliver=True)
  assert run(ctl, 1, build_sm(V_EGO), a_model=0.05, **kwargs) == 0.05
  assert (ctl.inhibit, ctl.authority, ctl.boost, ctl.added) == (reason, 0., 0., 0.)


def test_boosts_from_the_first_frame_at_the_rate_limit():
  ctl = new_controller()
  boosts = [run(ctl, 1, build_sm(V_EGO), v_cruise=V_EGO + 10.) for _ in range(SETTLE_FRAMES)]
  assert boosts[0] == pytest.approx(c.boost_rise(V_EGO) * DT)
  assert max(np.diff([0.] + boosts)) <= c.boost_rise(V_EGO) * DT + 1e-9
  assert boosts[-1] == pytest.approx(c.floor_max(V_EGO))
  assert ctl.inhibit == Inhibit.none


def test_floor_closes_the_gap():
  assert settled().boost == pytest.approx(min((V_CRUISE - V_EGO) / c.TAU, c.floor_max(V_EGO)))


@pytest.mark.parametrize("v, floor", [
  ((c.MIN_SPEED + c.FULL_SPEED) / 2, c.FLOOR_MAX / 2),
  (c.MIN_SPEED - 1., 0.),
])
def test_floor_fades_in_above_walking_pace(v, floor):
  ctl = new_controller()
  run(ctl, 1, build_sm(v), v_cruise=v + 10.)
  assert ctl.floor == pytest.approx(floor)


def test_gain_fades_the_boost_as_the_model_slows():
  ctl = new_controller()
  floor = min((V_CRUISE - V_EGO) / c.TAU, c.floor_max(V_EGO))
  a_model = -0.1
  gain = (a_model - c.GAIN_BP[0]) / (c.GAIN_BP[1] - c.GAIN_BP[0])
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO), a_model=a_model)
  assert ctl.boost == pytest.approx(gain * (floor - a_model))
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO), a_model=c.GAIN_BP[0])
  assert (ctl.boost, ctl.inhibit) == (0., Inhibit.modelBraking)


def test_never_adds_throttle_at_set_speed():
  ctl = new_controller()
  assert run(ctl, 5, build_sm(V_EGO), a_model=0., v_cruise=V_EGO) == 0.
  assert run(ctl, 5, build_sm(V_EGO), a_model=0.1, v_cruise=V_EGO - 1.) == pytest.approx(0.1)


@pytest.mark.parametrize("sm_kwargs, update_kwargs, reason", [
  ({"lead": True}, {}, Inhibit.lead),
  ({"gas": True}, {}, Inhibit.driver),
  ({"brake": True}, {}, Inhibit.driver),
  ({"hard_brake": True}, {}, Inhibit.hardBrake),
  ({"force_decel": True}, {}, Inhibit.forceDecel),
  ({"lane_change": True}, {}, Inhibit.laneChange),
  ({}, {"fcw": True}, Inhibit.fcw),
])
def test_hazards_cut_the_boost_and_hold(sm_kwargs, update_kwargs, reason):
  ctl = settled()
  start = ctl.boost
  run(ctl, 1, build_sm(V_EGO, **sm_kwargs), **update_kwargs)
  assert (ctl.inhibit, ctl.authority) == (reason, 0.)
  assert ctl.boost == pytest.approx(start - c.HAZARD_FALL * DT)
  run(ctl, math.ceil(start / (c.HAZARD_FALL * DT)), build_sm(V_EGO, **sm_kwargs), **update_kwargs)
  assert ctl.boost == 0.
  # clear: nothing until the hold has run out
  assert run(ctl, HOLD_FRAMES, build_sm(V_EGO)) == 0.
  assert ctl.inhibit == Inhibit.hold
  assert run(ctl, 1, build_sm(V_EGO)) > 0.


@pytest.mark.parametrize("kwargs, hold", [
  ({"dec_active": True}, HOLD_FRAMES),
  ({"is_e2e": False}, 0),
])
def test_leaving_dec_holds_like_a_hazard(kwargs, hold):
  ctl = settled()
  run(ctl, 1, build_sm(V_EGO), **kwargs)
  assert run(ctl, hold, build_sm(V_EGO)) == 0.
  assert run(ctl, 1, build_sm(V_EGO)) > 0.


def test_an_empty_road_does_not_limit_the_floor():
  assert bound(np.full(len(T_IDXS), V_EGO)) > c.FLOOR_MAX


def test_the_model_wobbling_does_not_limit_the_floor():
  # a 0.4 m/s dip and back, well under the target: not a slowdown
  assert bound(_plan(lambda t: V_EGO - 0.4 * math.sin(math.pi * min(t, 4.) / 4.))) > c.FLOOR_MAX


def test_a_stop_ahead_closes_the_envelope():
  assert bound(STOPPING) < 0.


def test_a_planned_slowdown_is_met_at_the_models_speed():
  # the plan slows 3 m/s over 6 s: the boost may not take the car past it
  slowing = _plan(lambda t: V_EGO - 0.5 * min(t, 6.))
  assert bound(slowing) <= min(float(np.min((slowing[c.SLOWING_MASK] - V_EGO) / T_IDXS[c.SLOWING_MASK])), 0.) + 1e-9


def test_a_bend_the_model_takes_slower_than_allowed_limits_the_boost():
  straight = bound(np.full(len(T_IDXS), V_EGO))
  assert bound(np.full(len(T_IDXS), V_EGO), BEND_AHEAD) < straight - 0.75
  # a gentle one leaves the floor alone
  assert bound(np.full(len(T_IDXS), V_EGO), BEND_AHEAD * 0.4 / 1.5) > c.FLOOR_MAX


@pytest.mark.parametrize("offset", [1.0, -0.6])
def test_envelope_is_relative_to_the_plan(offset):
  # the model reads vEgo a few percent off; a flat plan offset from it is not a slowdown
  ctl = settled()
  run(ctl, 1, build_sm(V_EGO, plan_v=V_EGO + offset))
  assert ctl.inhibit == Inhibit.none


def test_envelope_drops_at_once_and_recovers_through_the_filter():
  ctl = settled(v_cruise=V_EGO + 10.)
  run(ctl, 1, build_sm(V_EGO, plan_v=STOPPING), v_cruise=V_EGO + 10.)
  closed = ctl.bound
  assert closed < 0.
  run(ctl, round(c.BOUND_TAU / DT), build_sm(V_EGO), v_cruise=V_EGO + 10.)
  assert closed < ctl.bound < bound(np.full(len(T_IDXS), V_EGO), v_target=V_EGO + 10.)


def test_added_speed_counts_what_reached_the_car():
  assert settled().added == 0.
  ctl = settled(deliver=True)
  assert ctl.added > 0.
  # the gap assist's lift on top of ours reached the car too
  ctl.update(build_sm(V_EGO), 0., V_CRUISE, **ENGAGED)
  shed = ctl.added
  ctl.delivered(ctl.boost + 0.3, ctl.boost + 0.3)
  assert ctl.added == pytest.approx(shed + (ctl.boost + 0.3) * DT)
  # a lower candidate holds the car: none of it reached the car, and what we added converges out
  ctl.update(build_sm(V_EGO), 0., V_CRUISE, **ENGAGED)
  shed = ctl.added
  ctl.delivered(-0.5, ctl.boost)
  assert ctl.added == pytest.approx(shed * (1. - DT / c.CONVERGE_T))


def test_following_a_lead_clears_the_speed_we_added():
  ctl = settled(deliver=True)
  run(ctl, 100, build_sm(V_EGO), deliver=True)
  before = ctl.added
  assert before > 0.5
  # behind a lead the MPC holds the car below the e2e candidate
  for _ in range(round(5 * c.CONVERGE_T / DT)):
    out = ctl.update(build_sm(V_EGO, lead=True), 0., V_CRUISE, **ENGAGED)
    ctl.delivered(-0.3, out)
  assert ctl.added < 0.01 * before


def test_added_speed_is_shed_like_the_model_sheds_it():
  ctl = settled(deliver=True)
  added = ctl.added
  run(ctl, 40, build_sm(V_EGO, lead=True), deliver=True)  # boost cut, nothing more added
  before = ctl.added
  run(ctl, 1, build_sm(V_EGO, lead=True), deliver=True)
  assert ctl.added == pytest.approx(before * (1. - c.SHED_RATE * DT))
  assert before < added + 1.


def test_gives_back_what_it_added_before_a_stop():
  ctl = settled(deliver=True)
  run(ctl, 100, build_sm(V_EGO), deliver=True)
  added = ctl.added
  assert added > 0.5
  boosts = []
  for _ in range(200):
    run(ctl, 1, build_sm(V_EGO, plan_v=STOPPING), deliver=True)
    boosts.append(ctl.boost)
  boosts = np.asarray(boosts)
  # under the model, through the rate limit, never past what it put on
  assert boosts.min() < 0.
  assert np.abs(np.diff(boosts)).max() <= c.BOOST_RATE * DT + 1e-9
  assert boosts.min() >= -c.GIVE_MAX - 1e-9
  assert -boosts.sum() * DT <= added + 1e-6


def test_gives_nothing_back_when_it_added_nothing():
  ctl = settled()
  run(ctl, 200, build_sm(V_EGO, plan_v=STOPPING))
  assert ctl.boost == 0.


def test_friction_circle_caps_the_floor():
  ctl = new_controller()
  lat = 1.65
  run(ctl, 1, build_sm(V_EGO, curvature=lat / V_EGO ** 2), v_cruise=V_EGO + 10.)
  assert ctl.floor == pytest.approx(math.sqrt(1.7 ** 2 - lat ** 2))


@pytest.mark.parametrize("accel_coast", [0.2, -0.1])
def test_coast_caps_the_floor(accel_coast):
  ctl = new_controller()
  assert run(ctl, SETTLE_FRAMES, build_sm(V_EGO), v_cruise=V_EGO + 10., allow_throttle=False, accel_coast=accel_coast) == \
    pytest.approx(max(accel_coast, 0.))
  assert ctl.floor == pytest.approx(accel_coast)


def test_invalid_model_passes_through():
  ctl = settled()
  sm = build_sm(V_EGO, plan_v=_plan(lambda t: float('nan') if t > 5. else V_EGO))
  assert run(ctl, 1, sm, a_model=0.05) == 0.05
  assert (ctl.inhibit, ctl.authority) == (Inhibit.invalid, 0.)

  md = messaging.new_message('modelV2')
  md.modelV2.velocity.x = [V_EGO] * 10
  md.modelV2.orientationRate.z = [0.] * 10
  md.modelV2.position.x = [0.] * 10
  sm = {**build_sm(V_EGO), 'modelV2': md.modelV2.as_reader()}
  assert run(ctl, 1, sm, a_model=0.05) == 0.05
  assert ctl.inhibit == Inhibit.invalid


def test_no_give_back_behind_a_lead():
  ctl = settled(deliver=True)
  run(ctl, 100, build_sm(V_EGO), deliver=True)
  run(ctl, 200, build_sm(V_EGO, plan_v=STOPPING, lead=True), deliver=True)
  assert ctl.boost == 0.


def test_a_lead_lets_a_give_back_go_at_the_rate_limit():
  ctl = settled(deliver=True)
  run(ctl, 100, build_sm(V_EGO), deliver=True)
  run(ctl, 60, build_sm(V_EGO, plan_v=STOPPING), deliver=True)
  giving = ctl.boost
  assert giving < -0.1
  boosts = [giving]
  for _ in range(100):
    run(ctl, 1, build_sm(V_EGO, plan_v=STOPPING, lead=True), deliver=True)
    boosts.append(ctl.boost)
  assert np.abs(np.diff(boosts)).max() <= c.BOOST_RATE * DT + 1e-9
  assert boosts[-1] == 0.


def _dip(depth):
  """A plan that dips depth m/s between 2 and 6 s and comes back: the model wavering, not slowing."""
  return _plan(lambda t: V_EGO - depth if 2. <= t <= 6. else V_EGO)


def _with_speed_added():
  ctl = settled(deliver=True)
  run(ctl, 100, build_sm(V_EGO), deliver=True)
  assert ctl.added > 0.5
  return ctl


def test_a_dip_under_taper_drop_gives_nothing_back():
  # routes 00000150/00000152: a ~1 mph wobble in the plan handed back up to GIVE_MAX on a clear road
  ctl = _with_speed_added()
  boosts = []
  for _ in range(60):
    run(ctl, 1, build_sm(V_EGO, plan_v=_dip(c.TAPER_DROP - 0.3)), deliver=True)
    boosts.append(ctl.boost)
  assert min(boosts) >= 0.


def test_a_dip_under_taper_drop_still_stops_adding():
  ctl = _with_speed_added()
  boost = ctl.boost
  run(ctl, 60, build_sm(V_EGO, plan_v=_dip(c.TAPER_DROP - 0.3)), deliver=True)
  assert ctl.boost < boost


def test_a_dip_the_model_means_still_gives_back():
  ctl = _with_speed_added()
  boosts = []
  for _ in range(60):
    run(ctl, 1, build_sm(V_EGO, plan_v=_dip(c.TAPER_DROP + 1.)), deliver=True)
    boosts.append(ctl.boost)
  assert min(boosts) < 0.
  assert min(boosts) >= -c.GIVE_MAX - 1e-9


# Highway: the floor's ceiling and its rise ease from the city values over FLOOR_SPEED_BP (route 00000159)
@pytest.mark.parametrize("v", [c.FULL_SPEED, 12., c.FLOOR_SPEED_BP[0]])
def test_city_floor_and_rise_are_unchanged(v):
  ctl = new_controller()
  assert run(ctl, 1, build_sm(v), v_cruise=v + 10.) == pytest.approx(c.BOOST_RATE * DT)
  assert ctl.floor == pytest.approx(c.FLOOR_MAX)


@pytest.mark.parametrize("v", [c.FLOOR_SPEED_BP[1], 30.])
def test_highway_floor_and_rise(v):
  ctl = new_controller()
  boosts = [run(ctl, 1, build_sm(v), v_cruise=v + 10.) for _ in range(200)]
  assert boosts[0] == pytest.approx(c.BOOST_RISE_HIGHWAY * DT)
  assert max(np.diff([0.] + boosts)) <= c.BOOST_RISE_HIGHWAY * DT + 1e-9
  assert ctl.floor == pytest.approx(c.FLOOR_MAX_HIGHWAY)
  assert boosts[-1] == pytest.approx(c.FLOOR_MAX_HIGHWAY)


def test_floor_and_rise_interpolate_across_the_band():
  v = sum(c.FLOOR_SPEED_BP) / 2
  ctl = new_controller()
  assert run(ctl, 1, build_sm(v), v_cruise=v + 10.) == pytest.approx((c.BOOST_RATE + c.BOOST_RISE_HIGHWAY) / 2 * DT)
  assert ctl.floor == pytest.approx((c.FLOOR_MAX + c.FLOOR_MAX_HIGHWAY) / 2)


def _settled_on_highway():
  ctl = new_controller()
  v = c.FLOOR_SPEED_BP[1]
  run(ctl, 200, build_sm(v), v_cruise=v + 10., deliver=True)
  assert ctl.boost == pytest.approx(c.FLOOR_MAX_HIGHWAY)
  return ctl, v


def test_highway_boost_still_falls_at_boost_rate():
  ctl, v = _settled_on_highway()
  before = ctl.boost
  run(ctl, 1, build_sm(v), a_model=c.GAIN_BP[0], v_cruise=v + 10.)  # the model braking: gain 0, target 0
  assert before - ctl.boost == pytest.approx(c.BOOST_RATE * DT)


def test_highway_hazard_still_cuts_at_hazard_fall():
  ctl, v = _settled_on_highway()
  before = ctl.boost
  run(ctl, 1, build_sm(v, lead=True), v_cruise=v + 10.)
  assert before - ctl.boost == pytest.approx(c.HAZARD_FALL * DT)


def test_highway_give_back_keeps_its_rate():
  ctl, v = _settled_on_highway()
  run(ctl, 100, build_sm(v), v_cruise=v + 10., deliver=True)
  stopping = _plan(lambda t: max(v - 2. * t, 0.))
  boosts = [ctl.boost]
  for _ in range(200):
    run(ctl, 1, build_sm(v, plan_v=stopping), v_cruise=v + 10., deliver=True)
    boosts.append(ctl.boost)
  assert min(boosts) < 0.
  assert np.abs(np.diff(boosts)).max() == pytest.approx(c.BOOST_RATE * DT)


# A lead far enough ahead and not closing leaves the floor alone: the MPC candidate already holds the
# lead's gap and the planner takes the min of the candidates, so the floor can only ask for what the
# MPC allows. The lead stands the floor down only when it is near, closing or braking.
FAR = {"lead": True, "lead_d": (c.FAR_LEAD_T + 1.) * V_EGO, "lead_vrel": 1.}  # 6 s ahead, pulling away


def test_a_far_lead_pulling_away_leaves_the_floor_on():
  ctl = new_controller()
  boosts = [run(ctl, 1, build_sm(V_EGO, **FAR)) for _ in range(SETTLE_FRAMES)]
  assert ctl.inhibit == Inhibit.none and ctl.authority == 1.
  assert boosts[0] == pytest.approx(c.boost_rise(V_EGO) * DT)
  assert boosts[-1] == pytest.approx(min((V_CRUISE - V_EGO) / c.TAU, c.floor_max(V_EGO)))


def test_a_steady_far_lead_leaves_the_floor_on():
  ctl = new_controller()
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO, **{**FAR, "lead_vrel": c.FAR_LEAD_CLOSING}))
  assert ctl.inhibit == Inhibit.none and ctl.boost > 0.


@pytest.mark.parametrize("v, sm_kwargs", [
  (V_EGO, {**FAR, "lead_d": (c.FAR_LEAD_T - 0.5) * V_EGO}),  # inside the headway
  (V_EGO, {**FAR, "lead_vrel": c.FAR_LEAD_CLOSING - 0.1}),  # closing
  (V_EGO, {**FAR, "lead_a": c.FAR_LEAD_BRAKING - 0.1, "lead_vrel": c.FAR_LEAD_CLOSING - 0.1}),  # braking and closing
  (V_EGO, {**FAR, "lead_d": float('nan')}),  # a lead that is not a number is not clearly far
  (V_EGO, {"lead": True}),  # on the bumper
  (V_EGO, {**FAR, "lead2": (c.FAR_LEAD_T * V_EGO - 10., 1., 0.)}),  # a second lead that is near
  (V_EGO, {**FAR, "lead2": (FAR["lead_d"] + 20., c.FAR_LEAD_CLOSING - 0.1, 0.)}),  # a second lead that is closing
  (6., {"lead": True, "lead_d": c.FAR_LEAD_MIN - 5., "lead_vrel": 1.}),  # slow: 5 s is under the minimum distance
])
def test_a_near_or_closing_lead_still_stands_the_floor_down_at_once(v, sm_kwargs):
  ctl = new_controller()
  run(ctl, SETTLE_FRAMES, build_sm(v), v_cruise=v + 3.)
  start = ctl.boost
  assert start > 0.
  run(ctl, 1, build_sm(v, **sm_kwargs), v_cruise=v + 3.)
  assert (ctl.inhibit, ctl.authority) == (Inhibit.lead, 0.)
  assert ctl.boost == pytest.approx(start - c.HAZARD_FALL * DT)


def test_a_far_lead_that_closes_cuts_the_boost_and_holds():
  ctl = new_controller()
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO, **FAR))
  start = ctl.boost
  assert start > 0.
  # a cut-in or a braking lead: closing now
  run(ctl, math.ceil(start / (c.HAZARD_FALL * DT)), build_sm(V_EGO, **{**FAR, "lead_vrel": -3.}))
  assert ctl.boost == 0. and ctl.inhibit == Inhibit.lead
  # far again: the hold runs out first, as after any other hazard
  assert run(ctl, HOLD_FRAMES, build_sm(V_EGO, **FAR)) == 0.
  assert ctl.inhibit == Inhibit.hold
  assert run(ctl, 1, build_sm(V_EGO, **FAR)) > 0.


def test_a_far_lead_does_not_end_the_hold_after_a_driver_press():
  ctl = settled()
  run(ctl, 1, build_sm(V_EGO, gas=True))
  assert ctl.inhibit == Inhibit.driver
  assert run(ctl, HOLD_FRAMES, build_sm(V_EGO, **FAR)) == 0.
  assert run(ctl, 1, build_sm(V_EGO, **FAR)) > 0.


# A far lead that brakes without closing: the MPC candidate is not binding, so a hazard-rate cut would only be a jerk.
SOFT = {**FAR, "lead_a": c.FAR_LEAD_BRAKING - 0.5}


def _boosted_behind_a_far_lead():
  ctl = new_controller()
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO, **FAR))
  assert ctl.boost > 0.
  return ctl


def test_a_braking_far_lead_winds_the_boost_down_at_the_normal_rate():
  ctl = _boosted_behind_a_far_lead()
  start = ctl.boost
  run(ctl, 1, build_sm(V_EGO, **SOFT))
  assert (ctl.inhibit, ctl.authority) == (Inhibit.lead, 0.)
  assert ctl.boost == pytest.approx(start - c.BOOST_RATE * DT)
  run(ctl, math.ceil(start / (c.BOOST_RATE * DT)), build_sm(V_EGO, **SOFT))
  assert ctl.boost == 0.
  assert run(ctl, 20, build_sm(V_EGO, **SOFT)) == 0.  # and it does not grow back while the lead brakes


def test_a_braking_far_lead_that_starts_closing_cuts_at_the_hazard_rate():
  ctl = _boosted_behind_a_far_lead()
  run(ctl, 1, build_sm(V_EGO, **SOFT))
  before = ctl.boost
  run(ctl, 1, build_sm(V_EGO, **{**SOFT, "lead_vrel": c.FAR_LEAD_CLOSING - 0.5}))
  assert ctl.boost == pytest.approx(before - c.HAZARD_FALL * DT)


def test_the_hold_follows_a_soft_cut_like_any_other():
  ctl = _boosted_behind_a_far_lead()
  run(ctl, math.ceil(ctl.boost / (c.BOOST_RATE * DT)) + 1, build_sm(V_EGO, **SOFT))
  assert run(ctl, HOLD_FRAMES, build_sm(V_EGO, **FAR)) == 0.
  assert ctl.inhibit == Inhibit.hold
  assert run(ctl, 1, build_sm(V_EGO, **FAR)) > 0.


@pytest.mark.parametrize("sm_kwargs", [
  {"lead_d": (c.FAR_LEAD_T - 0.5) * V_EGO},
  {"lead_vrel": c.FAR_LEAD_CLOSING - 0.1},
])
def test_a_soft_lead_never_softens_a_near_or_closing_one(sm_kwargs):
  ctl = _boosted_behind_a_far_lead()
  start = ctl.boost
  run(ctl, 1, build_sm(V_EGO, **{**SOFT, **sm_kwargs}))
  assert ctl.boost == pytest.approx(start - c.HAZARD_FALL * DT)


# Coming back to far from near asks for a little more than staying far does.
EDGE = {**FAR, "lead_d": (c.FAR_LEAD_T + 0.5) * V_EGO}  # far enough to stay far, not to become far again
NEAR = {**FAR, "lead_d": (c.FAR_LEAD_T - 0.5) * V_EGO}


def test_a_lead_at_the_edge_of_far_is_far_until_it_has_been_near():
  ctl = new_controller()
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO, **EDGE))
  assert ctl.inhibit == Inhibit.none and ctl.boost > 0.


def test_a_lead_has_to_be_a_little_further_to_count_as_far_again():
  ctl = new_controller()
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO, **EDGE))
  run(ctl, 1, build_sm(V_EGO, **NEAR))
  assert ctl.inhibit == Inhibit.lead
  run(ctl, HOLD_FRAMES + SETTLE_FRAMES, build_sm(V_EGO, **EDGE))
  assert (ctl.inhibit, ctl.boost) == (Inhibit.lead, 0.)  # still near at the edge
  run(ctl, 1, build_sm(V_EGO, **FAR))  # a full re-arm margin ahead
  assert ctl.inhibit == Inhibit.hold
  run(ctl, HOLD_FRAMES, build_sm(V_EGO, **EDGE))  # and far again at the edge, since it has been far since
  assert run(ctl, 1, build_sm(V_EGO, **EDGE)) > 0.


@pytest.mark.parametrize("clear", ["no_lead", "idle"])
def test_the_edge_rule_is_forgotten_when_the_lead_is_gone(clear):
  ctl = new_controller()
  run(ctl, 1, build_sm(V_EGO, **NEAR))
  assert ctl.inhibit == Inhibit.lead
  if clear == "no_lead":
    run(ctl, 1, build_sm(V_EGO))
  else:
    run(ctl, 1, build_sm(V_EGO), is_e2e=False)
  run(ctl, HOLD_FRAMES + SETTLE_FRAMES, build_sm(V_EGO, **EDGE))
  assert ctl.inhibit == Inhibit.none and ctl.boost > 0.


def test_the_edge_rule_never_delays_a_hazard():
  ctl = new_controller()
  run(ctl, 1, build_sm(V_EGO, **NEAR))
  run(ctl, HOLD_FRAMES, build_sm(V_EGO, **FAR))
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO, **FAR))
  start = ctl.boost
  assert start > 0.
  run(ctl, 1, build_sm(V_EGO, **{**FAR, "lead_vrel": c.FAR_LEAD_CLOSING - 0.1}))
  assert (ctl.inhibit, ctl.authority) == (Inhibit.lead, 0.)
  assert ctl.boost == pytest.approx(start - c.HAZARD_FALL * DT)

