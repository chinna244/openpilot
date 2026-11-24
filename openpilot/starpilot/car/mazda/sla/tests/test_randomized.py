"""Randomized invariants for the Mazda SLA port: source conflicts, flickers and dropouts, GPS loss, rapid and held
presses, presses around the timeout, and disengagements (20 seeds x 60 s, the same rig as the route tests)."""
import random

import pytest

from openpilot.starpilot.car.mazda.sla import AssistState
from openpilot.starpilot.car.mazda.sla.arbiter import LIMIT_SETTLE, PRE_ACTIVE_GUARD_PERIOD, REPROMPT_MAX
from openpilot.starpilot.car.mazda.sla.tests.rig import MPH, Rig, frame

LIMITS = [15, 20, 25, 30, 35, 40, 45, 50]
SEEDS = range(20)


def run(seed, n=6000):
  rng = random.Random(seed)
  rig = Rig()
  base = rng.choice(LIMITS)
  map_l, vis_l = base, base
  dash, v, enabled = rng.choice([25, 30, 35, 40]), rng.uniform(10, 35), True
  plan, frames = {}, []
  for i in range(n):
    if rng.random() < 1 / 900:
      base = rng.choice(LIMITS)
      map_l, vis_l = base, base if rng.random() < .6 else rng.choice(LIMITS)
    if rng.random() < 1 / 2500:
      enabled = not enabled
    v = min(max(v + rng.gauss(0, .05), 0.), 45.)
    m = map_l if rng.random() > (.4 if rng.random() < .2 else .02) else 0
    vi = vis_l if rng.random() > .2 else 0
    if rng.random() < 1 / 300:
      vi = rng.choice(LIMITS)
    btn = plan.pop(i, [])
    if not btn and rng.random() < 1 / 400:
      name = rng.choice(['accelCruise', 'decelCruise', 'decelCruise', 'resumeCruise'])
      hold = rng.choice([1, 3, 8, 15, 60, 120])
      btn = [(name, True)]
      plan.setdefault(i + hold, []).append((name, False))
      if rng.random() < .3:
        gap = rng.randint(5, 40)
        plan.setdefault(i + hold + gap, []).append((name, True))
        plan.setdefault(i + hold + gap + rng.randint(1, 10), []).append((name, False))
    for name, pressed in btn:
      if pressed and name == 'accelCruise':
        dash += 1
      elif pressed and name == 'decelCruise':
        dash = max(19, dash - 1)
    f = frame(i, v_mph=v, dash=dash, map_mph=m, vision_mph=vi, enabled=enabled, btn=btn, gps=rng.random() > .01,
              gas=rng.random() < .003)
    frames.append(f)
    rig.step(f)
  return frames, rig.rows


@pytest.fixture(scope='module', params=SEEDS)
def case(request):
  return run(request.param)


def test_prompt_names_the_offer_the_press_is_judged_against(case):
  """The direction and speed shown are the session's settled offer against the setpoint, never a flicker."""
  _, rows = case
  for i, r in enumerate(rows):
    if not (r.text.startswith('Press') and r.state == AssistState.preActive):
      continue
    # selfdrived shows what plannerd published at its last tick: at most 5 frames behind the session's offer
    named = set()
    for q in rows[max(0, i - 5):i + 1]:
      offer, setpoint = round(q.offer * MPH), round(q.setpoint / 1.609344)
      named.add(f"{'Press + for' if setpoint < offer else 'Press - for'} {offer} mph")
    assert r.text in named, (i, r.ts)


def test_prompt_never_outlasts_its_window(case):
  _, rows = case
  start = None
  for r in rows:
    if r.state == AssistState.preActive:
      start = r.ts if start is None else start
      assert r.ts - start <= PRE_ACTIVE_GUARD_PERIOD
    else:
      start = None


def test_no_up_press_while_prompting(case):
  _, rows = case
  assert not any(r.send == 'increase' and r.prompting for r in rows)


def test_disengaged_means_no_session_no_alert_no_button(case):
  _, rows = case
  for prev, r in zip(rows, rows[1:], strict=False):
    if not prev.enabled and not r.enabled:
      assert r.state == AssistState.disabled and r.cap == 255.
      assert not r.text.startswith(('Press', 'Adjusting'))
      assert r.send == 'none'


def test_offer_changes_only_after_the_limit_held_settle(case):
  """The session's target moves only to a resolver value held LIMIT_SETTLE: a shorter flicker never reaches it."""
  _, rows = case
  hold = int(LIMIT_SETTLE / 0.01)
  for i in range(1, len(rows)):
    if rows[i].offer != rows[i - 1].offer and rows[i].offer > 0:
      window = rows[max(0, i - hold + 1):i + 1]
      assert len({round(r.target, 3) for r in window}) == 1 and round(window[0].target, 3) == round(rows[i].offer, 3), rows[i].ts


def test_reasks_per_offer_are_bounded(case):
  _, rows = case
  per_offer, prev_state, prev_offer = 0, None, None
  for r in rows:
    if r.offer != prev_offer or not r.enabled:
      per_offer, prev_offer = 0, r.offer
    if r.state == AssistState.preActive and prev_state != AssistState.preActive:
      per_offer += 1
      assert per_offer <= 1 + REPROMPT_MAX
    prev_state = r.state


def test_cap_only_while_the_session_holds_one(case):
  _, rows = case
  for r in rows:
    if r.cap < 255.:
      assert r.state in (AssistState.active, AssistState.adapting, AssistState.preActive)
