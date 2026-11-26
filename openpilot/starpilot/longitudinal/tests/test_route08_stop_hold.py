"""Route 08 (00000008--80148a3eeb): no-lead stop hold, light vs RES, and the stop-sign RES launch."""
from dataclasses import replace
from types import SimpleNamespace as NS
import unittest

from openpilot.starpilot.longitudinal.force_stop import ForceStop, StopFrame, StopPlan, StopTune, RESUME_LAUNCH_MAX_MPS

BASE, TICK = 10_000_000_000, 50_000_000


def frame(tick, **changes):
  # stopped, enabled, no lead, model stop line 30 m (light/sign set per test)
  return replace(StopFrame(BASE + tick * TICK, 0., 30., False, True, True, True, True, False,
                           False, False, 200., 0., False, 0., False, 0.), **changes)


class StopHoldTests(unittest.TestCase):
  def setUp(self):
    self.owner, self.tune = ForceStop(), StopTune()

  def step(self, tick, **changes):
    return self.owner.step(frame(tick, **changes), self.tune)

  def test_far_lead_is_not_a_queue_close_lead_is(self):
    for lead_m, held in ((34., True), (6., False)):
      with self.subTest(lead_m=lead_m):
        self.owner = ForceStop()
        for tick in range(60):  # behind a queue: upstream's engaged-at-standstill hold still expires (2 s)
          plan = self.step(tick, lead_present=True, lead_distance_m=lead_m)
        self.assertEqual(plan.manual_hold, held)

  def test_light_is_gas_only_and_survives_a_reset_at_the_same_stop(self):
    self.assertTrue(self.step(0, light=True).manual_hold)
    self.assertEqual(self.owner.standstill_reason, 'light')
    self.owner.reset()  # a brake tap resets the policy
    self.assertTrue(self.step(1).manual_hold)
    self.assertEqual(self.owner.standstill_reason, 'light')
    self.assertTrue(self.step(2, resume_requested=True, res_pressed=True).manual_hold)
    self.assertFalse(self.step(3, resume_requested=True, res_pressed=True).resume_override)
    self.assertFalse(self.step(4, pedal_override=True).manual_hold)

  def test_light_memory_ends_once_the_car_moves_on(self):
    self.assertTrue(self.step(0, light=True).manual_hold)
    self.assertFalse(self.step(1, pedal_override=True).manual_hold)
    for tick in range(2, 20):
      self.step(tick, speed_mps=2., standstill=False, horizon_m=190., raw_model_stopped=False)
    self.assertTrue(self.step(20).manual_hold)
    self.assertEqual(self.owner.standstill_reason, 'unclassified')  # the next stop is not the old light

  def test_unclassified_ignores_res_gas_releases_and_not_rearmed_until_moving(self):
    self.assertTrue(self.step(0).manual_hold)
    self.assertEqual(self.owner.standstill_reason, 'unclassified')
    plan = self.step(1, resume_requested=True, res_pressed=True)
    self.assertTrue(plan.manual_hold)
    self.assertFalse(plan.resume_override)
    self.assertFalse(self.step(2, pedal_override=True).manual_hold)
    for tick in range(3, 30):  # still at the line: no new hold
      self.assertFalse(self.step(tick, speed_mps=.3, standstill=False).manual_hold)
    self.step(30, speed_mps=1.5, standstill=False, raw_model_stopped=False, horizon_m=190.)
    self.assertTrue(self.step(31, horizon_m=190.).manual_hold)

  def test_res_still_overrides_a_moving_approach(self):
    for tick in range(15):
      plan = self.step(tick, speed_mps=10., standstill=False, light=True, horizon_m=30.)
    self.assertTrue(plan.forcing)
    self.assertFalse(self.step(15, speed_mps=10., standstill=False, light=True, horizon_m=30., resume_requested=True, res_pressed=True).forcing)

  def test_sign_launch_only_from_a_sign_hold(self):
    for case in ('sign', 'light', 'unclassified'):
      with self.subTest(case=case):
        self.owner = ForceStop()
        kw = {'sign': True, 'dashboard_sign': True} if case == 'sign' else {'light': case == 'light'}
        for tick in range(4):  # approach then stop
          self.step(tick, speed_mps=3., standstill=False, horizon_m=8., **kw)
        self.step(4, **kw)
        plan = self.step(5, resume_requested=True, res_pressed=True, **kw)
        self.assertEqual(plan.resume_launch, case == 'sign', self.owner.standstill_reason)

  def test_set_plus_never_releases_or_launches_a_standstill_hold(self):
    for case in ('sign', 'light', 'unclassified'):
      with self.subTest(case=case):
        self.owner = ForceStop()
        kw = {'sign': True, 'dashboard_sign': True} if case == 'sign' else {'light': case == 'light'}
        for tick in range(4):
          self.step(tick, speed_mps=3., standstill=False, horizon_m=8., **kw)
        self.step(4, **kw)
        for tick in range(5, 40):  # SET+ pressed repeatedly (SLA answers)
          plan = self.step(tick, resume_requested=tick % 5 == 0, **kw)
          self.assertTrue(plan.manual_hold)
          self.assertFalse(plan.resume_launch)
          self.assertFalse(plan.resume_override)
        plan = self.step(40, resume_requested=True, res_pressed=True, **kw)  # then a separate RES
        self.assertEqual(plan.resume_launch, case == 'sign')
        self.assertEqual(plan.manual_hold, case != 'sign')

  def test_set_plus_still_overrides_a_moving_approach(self):
    for tick in range(15):
      plan = self.step(tick, speed_mps=10., standstill=False, light=True, horizon_m=30.)
    self.assertTrue(plan.forcing)
    self.assertFalse(self.step(15, speed_mps=10., standstill=False, light=True, horizon_m=30., resume_requested=True).forcing)

  def test_sign_launch_ends_on_each_hazard(self):
    for change in ({'lead_present': True, 'lead_distance_m': 20.}, {'tracking_lead': True}, {'pedal_override': True},
                   {'speed_mps': RESUME_LAUNCH_MAX_MPS, 'standstill': False}, {'enabled': False},
                   {'light': True, 'sign': False, 'dashboard_sign': False}):
      with self.subTest(change=change):
        self.owner = ForceStop()
        for tick in range(4):
          self.step(tick, speed_mps=3., standstill=False, horizon_m=8., sign=True, dashboard_sign=True)
        self.step(4, sign=True, dashboard_sign=True)
        self.assertTrue(self.step(5, resume_requested=True, res_pressed=True, sign=True, dashboard_sign=True).resume_launch)
        self.assertTrue(self.step(6, speed_mps=1., standstill=False, sign=True, dashboard_sign=True).resume_launch)
        self.assertFalse(self.step(7, **{'sign': True, 'dashboard_sign': True, 'speed_mps': 1., 'standstill': False,
                                         **change}).resume_launch)
        self.assertFalse(self.step(8, speed_mps=1., standstill=False, sign=True, dashboard_sign=True).resume_launch)

  def test_sign_launch_times_out(self):
    for tick in range(4):
      self.step(tick, speed_mps=3., standstill=False, horizon_m=8., sign=True, dashboard_sign=True)
    self.step(4, sign=True, dashboard_sign=True)
    self.assertTrue(self.step(5, resume_requested=True, res_pressed=True, sign=True, dashboard_sign=True).resume_launch)
    for tick in range(6, 125):  # 5.95 s after RES
      self.assertTrue(self.step(tick, speed_mps=1., standstill=False, sign=True, dashboard_sign=True).resume_launch)
    self.step(125, speed_mps=1., standstill=False, sign=True, dashboard_sign=True)
    self.assertFalse(self.step(126, speed_mps=1., standstill=False, sign=True, dashboard_sign=True).resume_launch)


class PlannerLaunchTests(unittest.TestCase):
  """LongitudinalPlanner._resume_launch on a stub planner."""
  def setUp(self):
    from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
    self.fn = LongitudinalPlanner._resume_launch
    self.p = NS(force_stop_plan=StopPlan(resume_launch=True), a_cruise=1.2, dt=.05, resume_launch_state=None,
                resume_launch_floor=None, resume_launch_model_released=False, resume_launch_model_go=False,
                resume_launch_holding=False, resume_launch_go_s=0.)

  def run_(self, v=0., model=-.3, stop=False, mpc=3., clear=True):
    return self.fn(self.p, v, model, stop, mpc, clear, NS(desiredAcceleration=model, shouldStop=stop))

  def test_floor_is_cruise_launch_while_model_holds_and_hands_back(self):
    self.assertEqual(self.run_(), (1.2, False))
    self.assertEqual(self.run_(mpc=.8), (.8, False))
    self.assertEqual(self.run_(model=1.3), (1.3, False))  # the model asks as much: it owns the rest
    self.assertEqual(self.p.resume_launch_state, 'done')
    self.assertEqual(self.run_(model=.1), (.1, False))  # handed back for good this window

  def test_hazards_end_at_once(self):
    for kw in ({'clear': False}, {'model': -1.0}):
      with self.subTest(kw=kw):
        self.setUp()
        self.run_()
        self.assertEqual(self.run_(**kw)[0], kw.get('model', -.3))
    self.setUp()
    self.run_(stop=True)
    self.run_(stop=False)  # the model let go of the line ...
    self.assertEqual(self.run_(stop=True), (-.3, True))  # ... and a new stop ends the launch

  def test_cap_end_fades_other_ends_do_not(self):
    self.run_(v=1., model=.6)  # the model asks to go: the plain launch runs to the cap
    self.run_(v=4.4, model=.2)
    self.p.force_stop_plan = StopPlan()
    a, _ = self.run_(v=4.5, model=.2)
    self.assertAlmostEqual(a, 1.2 - .05)
    self.setUp()
    self.run_(v=1.)
    self.p.force_stop_plan = StopPlan()
    self.assertEqual(self.run_(v=1., model=.2)[0], .2)

  def test_holds_6_mph_while_the_model_holds_back(self):  # routes 10 241.7, 0b 199.4
    self.assertEqual(self.run_(v=2.), (1.2, False))  # below the hold speed: the plain launch
    self.assertEqual(self.run_(v=2.7), (0., False))
    self.assertTrue(self.p.resume_launch_holding)
    self.assertAlmostEqual(self.run_(v=3.4, model=-.6)[0], -.3)  # never brakes harder than -0.3
    self.assertAlmostEqual(self.run_(v=2.5)[0], .3)  # latched: no plain push again below it
    self.assertEqual(self.p.resume_launch_state, 'active')

  def test_hold_never_below_the_model_never_above_the_launch(self):
    self.run_(v=2.7)
    self.assertEqual(self.run_(v=3., model=.4), (.4, False))  # the model asks more: it gets it
    self.assertEqual(self.p.resume_launch_state, 'active')
    self.setUp()
    self.run_(v=2.7, mpc=.1)
    self.assertAlmostEqual(self.run_(v=2., mpc=.1)[0], .2)  # capped by the plain launch floor (0.2)

  def test_model_go_resumes_the_plain_launch(self):  # route 0b 127.2: same as before the hold existed
    self.run_(v=1., model=.6)
    self.assertEqual(self.run_(v=3.5, model=.8), (1.2, False))
    self.assertFalse(self.p.resume_launch_holding)
    self.run_(v=2.7)
    self.assertEqual(self.run_(v=3.5, model=.9), (1.2, False))  # go from the hold: plain launch again
    self.assertEqual(self.run_(v=3.5, model=1.3), (1.3, False))
    self.assertEqual(self.p.resume_launch_state, 'done')

  def test_gentle_go_releases_after_a_second_a_blip_does_not(self):
    self.run_(v=2.7)
    for _ in range(11):  # route 10 244.0: 0.55 s at +0.15
      self.assertEqual(self.run_(v=2.7, model=.15), (.15, False))
    self.run_(v=2.7)
    self.assertFalse(self.p.resume_launch_model_go)
    for _ in range(19):
      self.run_(v=2.8, model=.4)
    self.assertTrue(self.p.resume_launch_holding)
    self.assertEqual(self.run_(v=2.8, model=.4), (1.2, False))  # 1.0 s at +0.4: the plain launch runs on
    self.assertFalse(self.p.resume_launch_holding)

  def test_hazards_end_the_hold_at_once(self):
    for kw in ({'clear': False}, {'model': -1.0}):
      with self.subTest(kw=kw):
        self.setUp()
        self.run_(v=2.8)
        self.assertEqual(self.run_(v=2.8, **kw)[0], kw.get('model', -.3))
        self.assertEqual(self.p.resume_launch_state, 'done')

  def test_hold_window_end_fades(self):
    self.run_(v=2.7)
    self.assertEqual(self.run_(v=2.6)[0], RESUME_HOLD_AT_2_6)
    self.p.force_stop_plan = StopPlan()
    self.assertAlmostEqual(self.run_(v=2.6, model=-.5)[0], RESUME_HOLD_AT_2_6 - .05)
    self.assertEqual(self.p.resume_launch_state, 'fade')


class RecentSignLaunchTests(unittest.TestCase):
  """Route 11 148.8: the sign beside the car at the line, the model's stop still up at the RES."""
  def setUp(self):
    self.owner, self.tune = ForceStop(), StopTune()

  def step(self, tick, **changes):
    return self.owner.step(frame(tick, **changes), self.tune)

  def hold(self, roll_m):
    for tick in range(20):  # the sign confirmed on the approach
      self.step(tick, speed_mps=3., standstill=False, horizon_m=8., sign=True, dashboard_sign=True)
    ticks = round(roll_m / (1. * .05))
    for tick in range(20, 20 + ticks):  # sign out of view for the last metres
      self.step(tick, speed_mps=1., standstill=False, horizon_m=4.)
    tick = 20 + ticks
    self.assertTrue(self.step(tick, light=True).manual_hold)
    self.assertEqual(self.owner.standstill_reason, 'sign')
    return tick + 1

  def test_recent_sign_arms_and_survives_the_stop_flag(self):
    t = self.hold(1.)
    self.assertTrue(self.step(t, light=True, resume_requested=True, res_pressed=True).resume_launch)
    for tick in range(t + 1, t + 30):  # 1.5 s: the stop flag of the stop itself
      self.assertTrue(self.step(tick, speed_mps=1., standstill=False, light=True).resume_launch)
    self.assertTrue(self.step(t + 30, speed_mps=1.5, standstill=False, model_should_stop=False).resume_launch)
    self.assertFalse(self.step(t + 31, speed_mps=1.5, standstill=False, light=True, model_should_stop=False).resume_launch)

  def test_stop_flag_grace_is_bounded(self):
    t = self.hold(1.)
    self.assertTrue(self.step(t, light=True, resume_requested=True, res_pressed=True).resume_launch)
    for tick in range(t + 1, t + 50):  # 2.45 s
      self.assertTrue(self.step(tick, speed_mps=1., standstill=False, light=True).resume_launch)
    self.assertFalse(self.step(t + 50, speed_mps=1., standstill=False, light=True).resume_launch)

  def test_sign_seen_further_back_does_not_arm(self):
    t = self.hold(6.)  # route 11: 0.7 m; 6 m is not the sign at this line
    plan = self.step(t, light=True, resume_requested=True, res_pressed=True)
    self.assertTrue(plan.resume_override)  # the RES still releases the hold (0.2 crawl), as before
    self.assertFalse(plan.resume_launch)

  def test_visible_sign_launch_keeps_its_light_end(self):
    t = self.hold(1.)
    self.assertTrue(self.step(t, light=True, sign=True, resume_requested=True, res_pressed=True).resume_launch)
    self.assertFalse(self.step(t + 1, speed_mps=1., standstill=False, light=True).resume_launch)  # as before


RESUME_HOLD_AT_2_6 = min(1.2, max(-.3, 1.5 * (2.7 - 2.6)))


if __name__ == '__main__':
  unittest.main()
