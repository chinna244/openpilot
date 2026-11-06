"""Under 5 mph a stop the model plans finishes at a steady decel short of its stop point; it only ever adds braking."""
import math

import numpy as np
import pytest

from openpilot.cereal import messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.firm_stop import (FirmStop, plan_stop_distance, T_IDXS, FIRM_STOP_SPEED, FIRM_STOP_RELEASE_SPEED,
                                                        FIRM_STOP_MARGIN, FIRM_STOP_MIN_DIST, FIRM_STOP_PLAN_V, FIRM_STOP_ARM_FRAMES,
                                                        FIRM_STOP_SETTLE_FRAMES, FIRM_STOP_JERK, FIRM_STOP_DECEL_BP, FIRM_STOP_DECEL_MAX)
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


def run(fs, v, vel, pos, n=WARM, active=True, a_candidate=0.):
  for _ in range(n):
    a = fs.update(v, vel, pos, active, a_candidate)
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


def low_speed_sm(v, vel, pos, gas=False, model_a=0.):
  sm = H.build_sm(v_cruise_kph=40., a_ego=-0.3, gas=gas, model_a=model_a)
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
