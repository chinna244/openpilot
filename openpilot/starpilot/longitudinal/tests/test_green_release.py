"""A held no-lead stop ends on a verified green; a dropped stop, a timer, a blinker, a turn desire or a stop sign never ends it.

Model values from route 08 (exact-model replay of Fairy Tale on the recorded clips) and route 0a: the green at 08 302.25 s
(plan end 23 mph, accel 0.05 -> 0.51, desire none), the slower green at 0a 257.5 s (plan end 16-17 mph, accel 0.04 -> 0.12,
the driver's gas at 258.5 s), and the red creep before 08's green (plan end 8.6 mph, accel -0.02)."""
from dataclasses import replace
import unittest

from openpilot.starpilot.longitudinal.force_stop import (ForceStop, StopFrame, StopTune, GREEN_CONFIRM_NS, GREEN_MIN_HOLD_NS,
                                                         GREEN_SIGN_CLEAR_M, update_sign_clear)

BASE, TICK = 10_000_000_000, 50_000_000
HOLD_TICKS = GREEN_MIN_HOLD_NS // TICK
CONFIRM_TICKS = GREEN_CONFIRM_NS // TICK
APPROACH_MPS = 10.
APPROACH_TICKS = int(GREEN_SIGN_CLEAR_M / APPROACH_MPS / 0.05) + 20  # a block of driving with no sign
RED = dict(light=True, horizon_m=12., raw_model_stopped=True, model_should_stop=True, model_accel_mps2=-0.02,
           model_plan_end_mps=0.2, model_desire_none=True)
CREEP = dict(RED, model_plan_end_mps=8.6 * 0.44704)  # 08 298-302 s: the plan creeps while the light is red
GREEN = dict(light=False, horizon_m=190., raw_model_stopped=False, model_should_stop=False, model_accel_mps2=0.51,
             model_plan_end_mps=23. * 0.44704, model_desire_none=True)
SLOW_GREEN = dict(GREEN, horizon_m=40., raw_model_stopped=True, model_should_stop=True, model_accel_mps2=0.04,
                  model_plan_end_mps=16.6 * 0.44704)  # 0a 257.5 s


def frame(tick, **changes):
  return replace(StopFrame(BASE + tick * TICK, 0., 30., False, True, True, True, True, False,
                           False, False, 200., 0., False, 0., False, 0.), **changes)


class GreenReleaseTests(unittest.TestCase):
  def setUp(self):
    self.owner, self.tune = ForceStop(), StopTune()
    self.tick = 0
    self.clear_m = 0.

  def step(self, **changes):
    # the runtime's stop-sign watch, as force_stop_runtime feeds it
    seen = changes.get('sign', False) or changes.get('dashboard_sign', False)
    self.clear_m = update_sign_clear(self.clear_m, seen, changes.get('speed_mps', 0.), 0.05)
    plan = self.owner.step(frame(self.tick, sign_clear_m=self.clear_m, **changes), self.tune)
    self.tick += 1
    return plan

  def approach(self, ticks=APPROACH_TICKS, **extra):
    for _ in range(ticks):
      self.step(speed_mps=APPROACH_MPS, standstill=False, horizon_m=190., raw_model_stopped=False, **extra)

  def hold(self, cue=RED, ticks=2 * HOLD_TICKS, reason='light', **extra):
    for _ in range(ticks):
      plan = self.step(**cue, **extra)
    self.assertTrue(plan.manual_hold)
    self.assertEqual(self.owner.standstill_reason, reason)

  def release_after(self, cue, n=200, **extra):
    for i in range(n):
      if not self.step(**cue, **extra).manual_hold:
        return i
    return None

  def test_verified_green_releases_after_the_confirm_time(self):
    for name, green in (('08 green', GREEN), ('0a slow green', SLOW_GREEN)):
      with self.subTest(name):
        self.setUp()
        self.approach()
        self.hold()
        self.assertEqual(self.release_after(green), CONFIRM_TICKS)
        self.assertIsNone(self.owner.standstill_reason)
        for _ in range(40):  # no new hold at the same line
          self.assertFalse(self.step(**RED).manual_hold)

  def test_unclassified_stop_releases_on_green_too(self):
    # route 0a 250.1: the stop never read as a light
    self.approach()
    self.hold(cue={}, reason='unclassified')
    self.assertEqual(self.release_after(SLOW_GREEN), CONFIRM_TICKS)

  def test_green_needs_the_stop_to_settle_first(self):
    self.approach()
    for _ in range(2):
      self.step(**RED)
    self.assertEqual(self.release_after(GREEN), HOLD_TICKS - 2 + CONFIRM_TICKS)

  def test_single_cues_never_release(self):
    cues = {
      'dropped stop prediction': dict(GREEN, model_accel_mps2=-0.02, model_plan_end_mps=0.2),
      'red creep plan': CREEP,
      'go plan under a turn desire (route 08 375.5)': dict(GREEN, model_desire_none=False, model_accel_mps2=0.17),
      'go plan, still braking': dict(GREEN, model_accel_mps2=-0.1),
      'no braking, plan still short': dict(GREEN, model_plan_end_mps=3.),
      'blinker on': dict(RED, blinker=True),
      'timer': RED,
    }
    for name, cue in cues.items():
      with self.subTest(name):
        self.setUp()
        self.approach()
        self.hold()
        self.assertIsNone(self.release_after(cue, n=60 * 20), name)  # a minute of it

  def test_interrupted_evidence_starts_over(self):
    self.approach()
    self.hold()
    for _ in range(CONFIRM_TICKS - 1):
      self.assertTrue(self.step(**GREEN).manual_hold)
    self.assertTrue(self.step(**dict(GREEN, model_desire_none=False)).manual_hold)
    self.assertEqual(self.release_after(GREEN), CONFIRM_TICKS)

  def test_no_green_release_near_a_stop_sign(self):
    with self.subTest('sign lost before the line (route 0a 218.9): stop reads as a light'):
      self.approach()
      self.approach(ticks=10, sign=True)  # seen ~5 m worth
      self.approach(ticks=7)  # then lost, 3.5 m before the line
      self.hold()
      self.assertIsNone(self.release_after(GREEN, n=60 * 20))
    with self.subTest('the memory survives a policy reset (it is the runtime\'s)'):
      self.setUp()
      self.approach()
      self.approach(ticks=4, sign=True)
      self.owner.reset()
      self.hold()
      self.assertIsNone(self.release_after(GREEN, n=200))
    with self.subTest('dashboard stop sign counts'):
      self.setUp()
      self.approach()
      self.approach(ticks=4, dashboard_sign=True)
      self.hold()
      self.assertIsNone(self.release_after(GREEN, n=200))
    with self.subTest('a sign a block back does not block the next light'):
      self.setUp()
      self.approach(ticks=4, sign=True)
      self.approach()
      self.hold()
      self.assertEqual(self.release_after(GREEN), CONFIRM_TICKS)
    with self.subTest('no driving history yet (start of drive)'):
      self.setUp()
      self.hold()
      self.assertIsNone(self.release_after(GREEN, n=200))

  def test_always_hold_and_sign_hold_never_release_on_green(self):
    with self.subTest('always hold'):
      self.approach()
      self.hold(always_hold=True)  # the light memory names it, Always Hold still keeps it
      self.assertIsNone(self.release_after(GREEN, always_hold=True))
    with self.subTest('stop sign'):
      self.setUp()
      self.approach()
      for _ in range(4):
        self.step(speed_mps=3., standstill=False, horizon_m=8., sign=True, dashboard_sign=True)
      self.hold(cue=dict(sign=True, dashboard_sign=True), reason='sign')
      self.assertIsNone(self.release_after(GREEN))

  def test_gas_banner_marks_exactly_the_holds_green_cannot_release(self):
    def held(**extra):
      plan = None
      for _ in range(2 * HOLD_TICKS):
        plan = self.step(**RED, **extra)
      return plan
    cases = (('sign lost before the line', lambda: (self.approach(), self.approach(ticks=10, sign=True), self.approach(ticks=7)), True),
             ('no driving history yet', lambda: None, True),
             ('a sign a block back', lambda: (self.approach(ticks=4, sign=True), self.approach()), False),
             ('plain light', lambda: self.approach(), False))
    for name, setup, gas in cases:
      with self.subTest(name):
        self.setUp()
        setup()
        plan = held()
        self.assertTrue(plan.manual_hold)
        self.assertEqual((plan.gas_hold, plan.sign_hold), (gas, False))
        self.assertEqual(self.release_after(GREEN, n=200) is None, gas)
    with self.subTest('always hold: no banner (user choice), even where green is blocked'):
      self.setUp()
      plan = held(always_hold=True)  # no driving history: without Always Hold this is a gas hold
      self.assertEqual((plan.manual_hold, plan.gas_hold, plan.sign_hold), (True, False, False))
    with self.subTest('confirmed stop sign: RES banner, not the gas one'):
      self.setUp()
      self.approach()
      for _ in range(4):
        self.step(speed_mps=3., standstill=False, horizon_m=8., sign=True, dashboard_sign=True)
      for _ in range(2 * HOLD_TICKS):
        plan = self.step(sign=True, dashboard_sign=True)
      self.assertEqual((plan.manual_hold, plan.sign_hold, plan.gas_hold), (True, True, False))

  def test_no_gas_banner_on_a_hold_a_close_lead_expires(self):
    # an unclassified hold behind a lead within HOLD_LEAD_M ends by itself (hold_expired): it must not ask for the gas
    for _ in range(40):
      self.step(speed_mps=5., standstill=False, **dict(RED, horizon_m=20.))
    for _ in range(3):
      self.step(speed_mps=1., standstill=False, **dict(RED, horizon_m=3.))
    lead = dict(light=False, horizon_m=6., raw_model_stopped=True, model_should_stop=True, lead_present=True, lead_distance_m=6.)
    held = 0
    for _ in range(120):
      plan = self.step(**lead)
      held += plan.manual_hold
      self.assertFalse(plan.gas_hold)
    self.assertGreater(held, 10)  # the scenario really holds first

  def test_gas_banner_invariants_fuzzed(self):
    import random
    rng = random.Random(7)
    gas_frames = 0
    near, far = dict(lead_present=True, lead_distance_m=6.), dict(lead_present=True, lead_distance_m=15.)
    cues = (RED, CREEP, GREEN, SLOW_GREEN, {}, dict(RED, sign=True), dict(GREEN, sign=True), dict(RED, dashboard_sign=True),
            dict(RED, **near), dict(GREEN, **near), dict(RED, light=False, **near), dict(RED, **far), dict(GREEN, **far))
    for trial in range(300):
      self.setUp()
      prev = None
      for _ in range(rng.randrange(20, 200)):
        moving = rng.random() < 0.3
        extra = dict(speed_mps=rng.choice((3., 10.)), standstill=False, horizon_m=190., raw_model_stopped=False) if moving else {}
        cue = dict(rng.choice(cues), **extra)
        if rng.random() < 0.03:
          cue['pedal_override'] = True
        if rng.random() < 0.05:
          cue['always_hold'] = True
        plan = self.step(**cue)
        self.assertFalse(plan.gas_hold and plan.sign_hold, trial)
        self.assertLessEqual(plan.gas_hold, plan.manual_hold, trial)
        self.assertLessEqual(plan.sign_hold, plan.manual_hold, trial)
        # the gas banner is never shown on a hold that then goes by itself
        if prev is not None and prev.gas_hold and not cue.get('pedal_override') and not moving:
          self.assertTrue(plan.manual_hold, (trial, 'gas hold released without the gas'))
          gas_frames += 1
        prev = plan
    self.assertGreater(gas_frames, 1000)  # the fuzz really exercises gas holds

  def test_gas_still_releases(self):
    self.approach()
    self.hold()
    self.assertFalse(self.step(**RED, pedal_override=True).manual_hold)


if __name__ == '__main__':
  unittest.main()


class SignClearTests(unittest.TestCase):
  def test_counts_driving_and_resets_on_a_sign(self):
    m = 0.
    for _ in range(100):
      m = update_sign_clear(m, False, 10., 0.05)
    self.assertAlmostEqual(m, 50.)
    self.assertEqual(update_sign_clear(m, True, 10., 0.05), 0.)

  def test_a_gap_never_counts_as_driving(self):
    self.assertAlmostEqual(update_sign_clear(0., False, 10., 30.), 1.)  # at most two model frames
    self.assertEqual(update_sign_clear(5., False, float('nan'), 0.05), 0.)
    self.assertEqual(update_sign_clear(5., False, 10., -1.), 5.)


class SignReaderTests(unittest.TestCase):
  def test_only_a_publishing_reader_counts(self):
    from types import SimpleNamespace as NS
    from openpilot.starpilot.longitudinal.force_stop_runtime import SIGN_READER_MAX_AGE_NS, sign_reader_watching
    now = 100 * 10**9
    sm = lambda age, seen=True, valid=True: NS(seen={'slcVisionObservation': seen}, valid={'slcVisionObservation': valid},
                                              logMonoTime={'slcVisionObservation': now - age})
    self.assertTrue(sign_reader_watching(sm(200_000_000), now))
    self.assertTrue(sign_reader_watching(sm(SIGN_READER_MAX_AGE_NS), now))
    self.assertFalse(sign_reader_watching(sm(SIGN_READER_MAX_AGE_NS + 1), now))  # silent: not watching
    self.assertFalse(sign_reader_watching(sm(0, valid=False), now))
    self.assertFalse(sign_reader_watching(sm(0, seen=False), now))
    self.assertFalse(sign_reader_watching(sm(-10**9), now))  # from the future
    self.assertFalse(sign_reader_watching(NS(), now))  # no such service at all
