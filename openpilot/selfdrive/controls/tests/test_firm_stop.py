"""Under 5 mph a stop the model plans finishes at a steady decel short of its stop point; it only ever adds braking."""
import math

import numpy as np
import pytest

from openpilot.cereal import messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.firm_stop import (FirmStop, plan_stop_distance, T_IDXS, FIRM_STOP_SPEED, FIRM_STOP_RELEASE_SPEED,
                                                        FIRM_STOP_MARGIN, FIRM_STOP_MIN_DIST, FIRM_STOP_LEAD_GAP, FIRM_STOP_PLAN_V, FIRM_STOP_ARM_FRAMES,
                                                        FIRM_STOP_SETTLE_FRAMES, FIRM_STOP_JERK, FIRM_STOP_DECEL_BP, FIRM_STOP_DECEL_MAX,
                                                        FIRM_STOP_CREEP_A, FIRM_STOP_CREEP_SPEED)
from openpilot.selfdrive.controls.tests import test_cruise_handover as H

WARM = FIRM_STOP_SETTLE_FRAMES + FIRM_STOP_ARM_FRAMES + 40  # past the settle, the arming and the ramp


def stop_plan(v0, dist):
  """A plan from v0 slowing evenly to a stop dist ahead, then standing."""
  a = v0 ** 2 / (2. * dist)
  vel = np.maximum(v0 - a * T_IDXS, 0.)
  pos = np.where(vel > 0., v0 * T_IDXS - 0.5 * a * T_IDXS ** 2, dist)
  return vel, pos


def cruise_plan(v0):
  return np.full(len(T_IDXS), v0), v0 * T_IDXS


def rolling_stop_plan(v0=0.6):
  # A near stop followed immediately by driving onward, as at route 57's STOP.
  vel = np.interp(T_IDXS, [0., 1., 2., 10.], [v0, 0.1, 1., 4.])
  pos = np.cumsum(np.diff(T_IDXS, prepend=0.) * vel)
  return vel, pos


def run(fs, v, vel, pos, n=WARM, active=True, a_candidate=0., lead_d=math.inf):
  for _ in range(n):
    a = fs.update(v, vel, pos, active, a_candidate, lead_d)
  return a


class TestPlanStopDistance:
  def test_no_stop_planned(self):
    assert plan_stop_distance(*cruise_plan(2.)) == math.inf

  def test_stop_point(self):
    vel, pos = stop_plan(2., 3.)
    assert plan_stop_distance(vel, pos) == pytest.approx(3., abs=0.15)

  @pytest.mark.parametrize("vel,pos", [([], []), ([1.], [0.]), ([1., 0.], [0.]), ([1., float('nan')], [0., 1.]), ([1., 0.], [0., float('inf')])])
  def test_bad_plan(self, vel, pos):
    assert plan_stop_distance(vel, pos) == math.inf

  def test_pulling_away_plans_no_stop(self):
    # the first point is the current speed: from a standstill it is under the stop speed with nothing stopping ahead
    vel, pos = cruise_plan(2.)
    vel[0] = 0.1
    assert plan_stop_distance(vel, pos) == math.inf

  def test_slow_pull_away_plans_no_stop(self):
    # creeping off under FIRM_STOP_GO_V: the first point alone is under the stop speed, nothing stops ahead
    vel, pos = cruise_plan(0.4)
    vel[0] = 0.1
    assert plan_stop_distance(vel, pos) == math.inf

  def test_stop_and_go_plan_is_no_stop(self):
    # a creeping queue: the plan dips under the stop speed, then rolls again within FIRM_STOP_STAY
    vel, pos = stop_plan(1.5, 2.)
    i = np.flatnonzero(vel < FIRM_STOP_PLAN_V)[0]
    vel[i + 2:] = 1.0
    assert plan_stop_distance(vel, pos) == math.inf


class TestFirmStop:
  def test_steady_decel_to_short_of_stop(self):
    v, dist = 1.5, 3.
    assert run(FirmStop(), v, *stop_plan(v, dist)) == pytest.approx(-v ** 2 / (2 * (dist - FIRM_STOP_MARGIN)), rel=0.05)

  def test_stops_behind_the_lead_not_short_of_it(self):
    # route 0000004b 827 s: model stop 3.2 m ahead, lead 7.4 m: aim FIRM_STOP_LEAD_GAP behind the lead
    v, dist, lead_d = 1.5, 3.2, 9.0
    assert run(FirmStop(), v, *stop_plan(v, dist), lead_d=lead_d) == pytest.approx(-v ** 2 / (2 * (lead_d - FIRM_STOP_LEAD_GAP)), rel=0.05)

  def test_lead_never_brings_the_stop_closer(self):
    v, dist = 1.5, 3.
    want = -v ** 2 / (2 * (dist - FIRM_STOP_MARGIN))
    assert run(FirmStop(), v, *stop_plan(v, dist), lead_d=FIRM_STOP_LEAD_GAP + 0.5) == pytest.approx(want, rel=0.05)

  def test_needs_consecutive_frames(self):
    fs = FirmStop()
    vel, pos = stop_plan(1.5, 2.)
    run(fs, 1.5, *cruise_plan(1.5), n=FIRM_STOP_SETTLE_FRAMES)
    for _ in range(FIRM_STOP_ARM_FRAMES - 1):
      assert fs.update(1.5, vel, pos, True, 0.) == math.inf
    fs.update(1.5, *cruise_plan(1.5), True, 0.)  # one frame without a stop starts over
    assert fs.update(1.5, vel, pos, True, 0.) == math.inf

  def test_settles_after_a_reset(self):
    # a gas release under 5 mph: no firm stop for FIRM_STOP_SETTLE_FRAMES of control
    fs = FirmStop()
    vel, pos = stop_plan(1.5, 2.)
    run(fs, 1.5, vel, pos, n=FIRM_STOP_SETTLE_FRAMES - 1)
    assert fs.a_target == math.inf
    assert run(fs, 1.5, vel, pos, n=2) < 0.

  def test_ramps_in_from_the_candidate(self):
    fs = FirmStop()
    vel, pos = stop_plan(1.5, FIRM_STOP_MIN_DIST)
    a = run(fs, 1.5, vel, pos, n=FIRM_STOP_SETTLE_FRAMES, a_candidate=-0.2)  # arms on the last of these
    assert a == pytest.approx(-0.2 - FIRM_STOP_JERK * DT_MDL)
    prev = a
    for _ in range(5):
      a = fs.update(1.5, vel, pos, True, -0.2)
      assert a == pytest.approx(prev - FIRM_STOP_JERK * DT_MDL) or a == pytest.approx(-1.5 * 1.0)
      prev = a

  def test_never_above_zero_on_arming(self):
    fs = FirmStop()
    vel, pos = stop_plan(1.5, 6.)
    assert run(fs, 1.5, vel, pos, n=FIRM_STOP_SETTLE_FRAMES, a_candidate=0.4) <= 0.

  def test_hysteresis(self):
    fs = FirmStop()
    vel, pos = stop_plan(2.0, 3.)
    assert run(fs, 2.0, vel, pos) < 0.
    v = (FIRM_STOP_SPEED + FIRM_STOP_RELEASE_SPEED) / 2
    assert run(fs, v, *stop_plan(v, 3.), n=3) < 0.  # keeps finishing between the two speeds
    assert run(fs, FIRM_STOP_RELEASE_SPEED + 0.1, *stop_plan(3., 3.), n=1) == math.inf

  def test_does_not_arm_between_speeds(self):
    v = (FIRM_STOP_SPEED + FIRM_STOP_RELEASE_SPEED) / 2
    assert run(FirmStop(), v, *stop_plan(v, 3.)) == math.inf

  def test_lets_go_when_the_plan_goes_on(self):
    fs = FirmStop()
    assert run(fs, 1.5, *stop_plan(1.5, 2.)) < 0.
    assert run(fs, 1.5, *cruise_plan(1.5), n=1) == math.inf

  def test_finishes_into_the_stopping_state(self):
    fs = FirmStop()
    a = run(fs, 1.0, *stop_plan(1.0, 1.))
    vel, pos = np.zeros(len(T_IDXS)), np.zeros(len(T_IDXS))
    vel[0] = 0.2
    assert run(fs, 0.2, vel, pos, n=5) == pytest.approx(a)  # holds its ask
    assert fs.should_stop
    vel, pos = cruise_plan(1.0)  # the plan goes: let go
    vel[0] = 0.1
    assert run(fs, 0.1, vel, pos, n=1) == math.inf and not fs.should_stop

  def test_off_above_speed(self):
    v = FIRM_STOP_SPEED + 0.1
    assert run(FirmStop(), v, *stop_plan(v, 3.)) == math.inf

  def test_off_when_inactive(self):
    assert run(FirmStop(), 1.5, *stop_plan(1.5, 2.), active=False) == math.inf

  def test_off_without_planned_stop(self):
    assert run(FirmStop(), 1.5, *cruise_plan(1.5)) == math.inf

  @pytest.mark.parametrize("v", [0., 0.2])
  def test_does_not_arm_under_stopping_speed(self, v):
    fs = FirmStop()
    assert run(fs, v, *stop_plan(0.5, 1.)) == math.inf and not fs.should_stop

  @pytest.mark.parametrize("v", [0.3, 0.6, 1.0, 1.8, 2.2])
  def test_capped_and_eased(self, v):
    # a plan stopping right here asks the most it may: eased toward standstill, never past FIRM_STOP_DECEL_MAX
    a = run(FirmStop(), v, *stop_plan(v, FIRM_STOP_MIN_DIST))
    assert a == pytest.approx(-min(v ** 2 / (2 * FIRM_STOP_MIN_DIST), float(np.interp(v, FIRM_STOP_DECEL_BP, FIRM_STOP_DECEL_MAX))))
    assert -max(FIRM_STOP_DECEL_MAX) <= a < 0.

  def test_far_stop_asks_little(self):
    assert -0.15 < run(FirmStop(), 1.0, *stop_plan(1.0, 4.8)) < 0.


def low_speed_sm(v, vel, pos, gas=False, model_a=0., lead=None):
  sm = H.build_sm(v_cruise_kph=40., a_ego=-0.3, gas=gas, model_a=model_a, lead=lead)
  cs = messaging.new_message('carState')
  cs.carState.vEgo, cs.carState.aEgo, cs.carState.vCruise, cs.carState.vCruiseCluster = v, -0.3, 40., 40.
  cs.carState.gasPressed = gas
  sm['carState'] = cs.carState.as_reader()
  md = messaging.new_message('modelV2')
  md.modelV2.orientationRate.z = [0.01] * 33
  md.modelV2.velocity.x, md.modelV2.position.x = [float(x) for x in vel], [float(x) for x in pos]
  md.modelV2.action.desiredAcceleration = model_a
  md.modelV2.meta.disengagePredictions.gasPressProbs = [0., 0.]
  sm['modelV2'] = md.modelV2.as_reader()
  return sm


class TestPlanner:
  def run(self, sm, n=WARM):
    p = H.planner()
    for _ in range(n):
      p.update(sm)
    return p

  def test_brakes_firmly_into_planned_stop(self):
    v, dist = 1.5, 2.5
    p = self.run(low_speed_sm(v, *stop_plan(v, dist)))
    assert p.output_a_target <= -v ** 2 / (2 * (dist - FIRM_STOP_MARGIN)) + 1e-3

  def test_lead_sets_the_stop(self):
    v, lead_d = 1.5, 9.
    p = self.run(low_speed_sm(v, *stop_plan(v, 3.), lead=lead_d))
    assert p.firm_stop.a_target == pytest.approx(-v ** 2 / (2 * (lead_d - FIRM_STOP_LEAD_GAP)), rel=0.05)

  def test_stopping_state_under_stopping_speed(self):
    p = H.planner()
    sm = low_speed_sm(1.0, *stop_plan(1.0, 1.))
    for _ in range(WARM):
      p.update(sm)
    vel = np.zeros(len(T_IDXS))
    vel[0] = 0.2
    sm = low_speed_sm(0.2, vel, np.zeros(len(T_IDXS)), model_a=0.05)
    p.update(sm)
    assert p.output_should_stop

  def test_no_effect_without_planned_stop(self):
    v = 1.5
    assert self.run(low_speed_sm(v, *cruise_plan(v))).firm_stop.a_target == math.inf

  def test_no_effect_pulling_away(self):
    vel, pos = cruise_plan(2.)
    vel[0] = 0.4
    p = self.run(low_speed_sm(0.4, vel, pos, model_a=0.5))
    assert p.firm_stop.a_target == math.inf and not p.output_should_stop

  def test_no_effect_under_gas(self):
    v = 1.5
    sm = low_speed_sm(v, *stop_plan(v, 2.5), gas=True)
    sm['controlsState'] = H.build_sm(override=True)['controlsState']
    assert self.run(sm).firm_stop.a_target == math.inf

  def test_committed_stop_finishes_even_when_the_model_plans_to_roll_on(self, monkeypatch):
    from openpilot.sunnypilot.selfdrive.controls.lib.vision_force_stop.force_stop import StopPlan

    p = H.planner()
    sm = low_speed_sm(0.6, *rolling_stop_plan(), model_a=-0.2)
    monkeypatch.setattr(p.vision_force_stop, 'sample', lambda *args, **kwargs: StopPlan(model_ns=0, forcing=True))
    for _ in range(WARM):
      p.update(sm)
    assert p.firm_stop.finishing
    assert p.output_a_target < -0.2

  @pytest.mark.parametrize('gas,stale', [(True, False), (False, True)])
  def test_committed_stop_cannot_finish_during_override_or_on_a_stale_plan(self, monkeypatch, gas, stale):
    from openpilot.sunnypilot.selfdrive.controls.lib.vision_force_stop.force_stop import StopPlan

    p = H.planner()
    sm = low_speed_sm(0.6, *rolling_stop_plan(), model_a=-0.2, gas=gas)
    if gas:
      sm['controlsState'] = H.build_sm(override=True)['controlsState']
    monkeypatch.setattr(p.vision_force_stop, 'sample', lambda *args, **kwargs: StopPlan(model_ns=1 if stale else 0, forcing=True))
    for _ in range(WARM):
      p.update(sm)
    assert not p.firm_stop.finishing


  def test_a_committed_creep_with_a_far_lead_still_stops_for_the_stop(self, monkeypatch):
    # route 55: a lead appeared 27.6 m ahead during the STOP; the STOP is ahead of it, so the lead cannot soften the finish
    from openpilot.sunnypilot.selfdrive.controls.lib.vision_force_stop.force_stop import StopPlan

    p = H.planner()
    sm = low_speed_sm(0.6, *cruise_plan(0.55), model_a=0.1, lead=27.6)
    monkeypatch.setattr(p.vision_force_stop, 'sample', lambda *args, **kwargs: StopPlan(model_ns=0, forcing=True))
    for _ in range(WARM):
      p.update(sm)
    assert p.firm_stop.finishing
    assert p.firm_stop.a_target == pytest.approx(-np.interp(0.6, FIRM_STOP_DECEL_BP, FIRM_STOP_DECEL_MAX))

  @pytest.mark.parametrize('brand,v,expected', [('mazda', 0.64, False), ('mazda', 0.2, True), ('toyota', 0.64, True), ('hyundai', 0.64, True)])
  def test_force_stop_asks_for_the_stopping_state_only_under_the_stopping_speed(self, monkeypatch, brand, v, expected):
    # route 59 t=227.8: ForceStop's stop at 0.64 m/s put Mazda in STOPPING and the -1.0 command barely slowed the car;
    # other brands keep ForceStop's stop at any speed
    from openpilot.sunnypilot.selfdrive.controls.lib.vision_force_stop.force_stop import StopPlan

    p = H.planner()
    p.CP.brand = brand
    sm = low_speed_sm(v, *cruise_plan(v), model_a=0.1)
    monkeypatch.setattr(p.vision_force_stop, 'sample', lambda *args, **kwargs: StopPlan(model_ns=0, forcing=True, should_stop=True,
                                                                                         speed_ceiling_mps=0.))
    for _ in range(WARM):
      p.update(sm)
    assert p.output_should_stop == expected
    assert p.output_a_target < 0.


class TestCommittedStop:
  def test_near_stop_without_commitment_is_still_allowed_to_roll(self):
    assert run(FirmStop(), 0.6, *rolling_stop_plan()) == math.inf

  def test_committed_stop_lets_a_braking_model_brake(self):
    # no stop point yet, but the model is still braking into the STOP: it keeps the stop, nothing is added
    fs = FirmStop()
    for _ in range(WARM):
      assert fs.update(1.5, *cruise_plan(1.5), True, -FIRM_STOP_CREEP_A - 0.1, committed_stop=True) == math.inf

  @pytest.mark.parametrize('a_model', [0.1, -0.1])
  def test_committed_stop_finishes_when_the_model_only_creeps(self, a_model):
    # routes 57, 59, 4d, 58: slowed under 5 mph at a STOP, the model plans to creep on and never plans a stop
    fs = FirmStop()
    for _ in range(WARM):
      a = fs.update(0.6, *cruise_plan(0.55), True, a_model, committed_stop=True)
    assert fs.finishing and a == pytest.approx(-np.interp(0.6, FIRM_STOP_DECEL_BP, FIRM_STOP_DECEL_MAX))
    assert not fs.should_stop  # pid brakes it down: the stopping state only under FIRM_STOP_PLAN_V
    fs.update(0.2, *cruise_plan(0.2), True, a_model, committed_stop=True)
    assert fs.should_stop

  def test_a_creep_finish_ramps_in(self):
    fs = FirmStop()
    for _ in range(FIRM_STOP_SETTLE_FRAMES):
      fs.update(1.0, *cruise_plan(1.0), True, 0., committed_stop=True)
    a = [fs.update(1.0, *cruise_plan(1.0), True, 0., committed_stop=True) for _ in range(30)]
    steps = np.diff([x for x in a if math.isfinite(x)])
    assert len(steps) and steps.min() >= -FIRM_STOP_JERK * DT_MDL - 1e-9

  @pytest.mark.parametrize('lead_d', [27.6, 12., 4.5])
  def test_a_creep_finish_is_not_softened_by_a_lead(self, lead_d):
    # a car pulled in ahead during the STOP: the STOP still finishes at the cap, a lead never relaxes it
    fs = FirmStop()
    for _ in range(WARM):
      a = fs.update(0.6, *cruise_plan(0.55), True, 0., lead_d, committed_stop=True)
    assert a == pytest.approx(-np.interp(0.6, FIRM_STOP_DECEL_BP, FIRM_STOP_DECEL_MAX))

  def test_a_lead_appearing_as_the_stop_point_goes_does_not_relax_the_braking(self):
    # braking into a planned stop, then the model's stop point disappears just as a lead appears 27.6 m ahead
    fs = FirmStop()
    for _ in range(WARM):
      a = fs.update(1.0, *stop_plan(1.0, 1.5), True, -0.6, committed_stop=True)
    assert a < -0.5
    held = [fs.update(0.9 - 0.02 * k, *cruise_plan(0.9 - 0.02 * k), True, 0., 27.6, committed_stop=True) for k in range(20)]
    assert max(held) <= a + 1e-9

  def test_a_committed_roll_faster_than_a_crawl_is_left_to_the_model(self):
    # route 55 t=102.8: 4.5 mph past the sign into the junction, the model easing at -0.25: not a crawl, nothing added
    fs = FirmStop()
    for _ in range(WARM):
      assert fs.update(FIRM_STOP_CREEP_SPEED + 0.5, *cruise_plan(FIRM_STOP_CREEP_SPEED + 0.5), True, -0.25, committed_stop=True) == math.inf

  def test_a_planned_finish_losing_its_stop_point_above_a_crawl_only_holds(self):
    # a planned finish at 4.47 mph asks -0.27; its stop point goes at 4.03 mph: hold -0.27 as before, the cap only at a crawl
    fs = FirmStop()
    for _ in range(WARM):
      a = fs.update(2.0, *stop_plan(2.0, 7.9), True, -0.27, committed_stop=True)
    assert fs.finishing and a == pytest.approx(-0.27, abs=0.01)
    for _ in range(20):
      assert fs.update(1.8, *cruise_plan(1.8), True, 0., committed_stop=True) == pytest.approx(a)
    for _ in range(40):
      b = fs.update(1.0, *cruise_plan(1.0), True, 0., committed_stop=True)
    assert b == pytest.approx(-np.interp(1.0, FIRM_STOP_DECEL_BP, FIRM_STOP_DECEL_MAX))  # in the crawl band it finishes

  def test_a_creep_without_commitment_is_left_alone(self):
    assert run(FirmStop(), 0.6, *cruise_plan(0.55), a_candidate=0.1) == math.inf

  def test_committed_stop_holds_the_finish_until_stopped(self):
    fs = FirmStop()
    for _ in range(WARM):
      fs.update(0.6, *rolling_stop_plan(), True, -0.2, committed_stop=True)
    assert fs.finishing
    # The model's tiny dip disappears during the crawl, but STOP ownership remains.
    assert fs.update(0.4, *cruise_plan(0.4), True, -0.1, committed_stop=True) < 0.
    fs.update(0.2, *cruise_plan(1.), True, 0.1, committed_stop=True)
    assert fs.should_stop
    # Withdrawing the commitment permits the model's departure immediately.
    assert fs.update(0.2, *cruise_plan(1.), True, 0.1, committed_stop=False) == math.inf
    assert not fs.should_stop

  def test_res_at_standstill_releases_a_committed_finish_on_a_slow_creep(self):
    # the STOP ends (RES) and the model plans a gentle 0.4 m/s creep: under GO_V, but the finish only the STOP made goes too
    fs = FirmStop()
    for _ in range(WARM):
      fs.update(0.6, *rolling_stop_plan(), True, -0.2, committed_stop=True)
    standing = np.zeros(len(T_IDXS)), np.zeros(len(T_IDXS))
    fs.update(0., *standing, True, 0., committed_stop=True)
    assert fs.should_stop
    creep = np.interp(T_IDXS, [0., 2., 10.], [0., 0.4, 0.4])
    assert run(FirmStop(), 0., creep, np.cumsum(np.diff(T_IDXS, prepend=0.) * creep)) == math.inf  # baseline never holds it
    assert fs.update(0., creep, np.cumsum(np.diff(T_IDXS, prepend=0.) * creep), True, 0.1, committed_stop=False) == math.inf
    assert not fs.finishing and not fs.should_stop

  def test_a_committed_finish_the_model_still_plans_stays_when_the_stop_ends(self):
    # moving, the STOP ends while the model itself plans to stay stopped: an ordinary finish, kept as without the STOP
    fs = FirmStop()
    for _ in range(WARM):
      fs.update(0.6, *stop_plan(0.6, 1.5), True, -0.2, committed_stop=True)
    assert fs.update(0.6, *stop_plan(0.6, 1.5), True, -0.2, committed_stop=False) < 0.
    assert fs.finishing

  def test_a_committed_finish_the_model_owns_stays_at_standstill_when_the_stop_ends(self):
    # an ordinary sustained stop under a STOP, standing with the model still planning zero: ending the STOP changes nothing
    standing = np.zeros(len(T_IDXS)), np.zeros(len(T_IDXS))
    fs, base = FirmStop(), FirmStop()
    for _ in range(WARM):
      fs.update(0.6, *stop_plan(0.6, 1.5), True, -0.2, committed_stop=True)
      base.update(0.6, *stop_plan(0.6, 1.5), True, -0.2)
    for committed in (True, True, False, False):
      a = fs.update(0., *standing, True, 0.1, committed_stop=committed)
      b = base.update(0., *standing, True, 0.1)
      assert (a, fs.finishing, fs.should_stop) == (b, base.finishing, base.should_stop)
    assert fs.finishing and fs.should_stop

  @pytest.mark.parametrize('history', ['diverged target', 'invalid geometry'])
  def test_once_the_stop_ends_the_finish_is_exactly_the_uncommitted_one(self, history):
    # whatever the STOP's waiver did to the finish's history, without the STOP every output is the unwaived finish's
    standing = np.zeros(len(T_IDXS)), np.zeros(len(T_IDXS))
    backwards = stop_plan(0.6, 1.5)[0], stop_plan(0.6, 1.5)[1].copy()
    backwards[1][1] = -1.  # one frame whose approach runs backwards: only the committed finish rejects it
    frames = [(1.2, stop_plan(1.2, 1.5), -0.3)] * WARM
    if history == 'diverged target':
      # the model plans to roll on for a moment: the waiver holds its target while the unwaived finish lets go and re-arms
      frames += [(1.1, rolling_stop_plan(1.1), -0.3)] * 3 + [(1.0, stop_plan(1.0, 0.6), -0.3)] * 4
    else:
      frames += [(0.6, stop_plan(0.6, 1.5), -0.2)] * WARM + [(0.6, backwards, -0.2), (0., standing, 0.1)]
    fs, base = FirmStop(), FirmStop()
    for v, plan, a_cand in frames:
      fs.update(v, *plan, True, a_cand, committed_stop=True)
      base.update(v, *plan, True, a_cand)
    assert (fs.a_target, fs.finishing) != (base.a_target, base.finishing)  # the histories did diverge
    after = [(1.0, stop_plan(1.0, 0.6), -0.3)] * 3 if history == 'diverged target' else [(0., standing, 0.1)] * 3
    for v, plan, a_cand in after:
      assert fs.update(v, *plan, True, a_cand, committed_stop=False) == base.update(v, *plan, True, a_cand)
      assert (fs.finishing, fs.should_stop) == (base.finishing, base.should_stop)

  def test_a_held_committed_target_keeps_the_speed_cap(self):
    fs = FirmStop()
    for _ in range(WARM):
      a = fs.update(1.2, *stop_plan(1.2, 0.5), True, -1.5, committed_stop=True)
    assert a == pytest.approx(-FIRM_STOP_DECEL_MAX[1])
    # the model's dip disappears with the STOP still owning it: held, but no harder than 0.4 m/s allows
    a = fs.update(0.4, *cruise_plan(0.4), True, -0.1, committed_stop=True)
    assert a == pytest.approx(-np.interp(0.4, FIRM_STOP_DECEL_BP, FIRM_STOP_DECEL_MAX))

  def test_override_clears_a_committed_finish(self):
    fs = FirmStop()
    for _ in range(WARM):
      fs.update(0.6, *rolling_stop_plan(), True, -0.2, committed_stop=True)
    assert fs.update(0.6, *rolling_stop_plan(), False, -0.2, committed_stop=True) == math.inf
    assert not fs.finishing and not fs.should_stop

  @pytest.mark.parametrize('v', [0.4, 0.2])
  def test_invalid_geometry_cannot_retain_a_committed_finish(self, v):
    fs = FirmStop()
    for _ in range(WARM):
      fs.update(0.6, *rolling_stop_plan(), True, -0.2, committed_stop=True)
    vel, pos = cruise_plan(v)
    pos[-1] = math.nan
    assert fs.update(v, vel, pos, True, -0.1, committed_stop=True) == math.inf
    assert not fs.finishing and not fs.should_stop

  def test_backwards_geometry_cannot_retain_a_committed_finish(self):
    fs = FirmStop()
    for _ in range(WARM):
      fs.update(0.6, *rolling_stop_plan(), True, -0.2, committed_stop=True)
    vel, pos = rolling_stop_plan(0.4)
    pos[1] = -1.  # the approach to the stop point runs backwards
    assert fs.update(0.4, vel, pos, True, -0.1, committed_stop=True) == math.inf
    assert not fs.finishing

  @pytest.mark.parametrize('tail', ['stopped', 'horizon'])
  def test_a_drifting_tail_does_not_drop_a_committed_finish(self, tail):
    # real plans drift back by cm past the stop point and at the horizon's far end (every planned stop, routes 55-57)
    fs = FirmStop()
    vel, pos = stop_plan(0.6, 1.5) if tail == 'stopped' else rolling_stop_plan()
    pos = pos.copy()
    pos[-3:] -= [0.02, 0.05, 0.1]
    for _ in range(WARM):
      a = fs.update(0.6, vel, pos, True, -0.2, committed_stop=True)
    assert fs.finishing and a < 0.
    assert run(FirmStop(), 0.6, vel, pos) == (math.inf if tail == 'horizon' else run(FirmStop(), 0.6, *stop_plan(0.6, 1.5)))

  @pytest.mark.parametrize('v', [0., 0.2, FIRM_STOP_RELEASE_SPEED])
  def test_commitment_does_not_arm_a_standstill_or_above_the_finishing_speed(self, v):
    fs = FirmStop()
    for _ in range(WARM):
      assert fs.update(v, *rolling_stop_plan(), True, -0.2, committed_stop=True) == math.inf

  def test_cancellation_while_moving_releases_the_finish(self):
    fs = FirmStop()
    for _ in range(WARM):
      fs.update(0.6, *rolling_stop_plan(), True, -0.2, committed_stop=True)
    assert fs.update(0.4, *cruise_plan(0.4), True, 0.2, committed_stop=False) == math.inf

  @pytest.mark.parametrize('vel,pos', [([], []), ([0.6, float('nan')], [0., 1.]), ([0.6, 0.], [0., float('inf')])])
  def test_commitment_cannot_arm_on_invalid_geometry(self, vel, pos):
    fs = FirmStop()
    for _ in range(WARM):
      assert fs.update(0.6, vel, pos, True, -0.2, committed_stop=True) == math.inf
