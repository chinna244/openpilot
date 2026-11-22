"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Departure window: a driver's gas press from an engaged stop lets the lift work below walking pace.
"""
import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL as DT
from openpilot.starpilot.longitudinal import set_speed_assist as c
from openpilot.starpilot.longitudinal.tests.set_speed_assist.helpers import FakeParams, controller_class, T_IDXS, build_sm

E2ESetSpeedController = controller_class()

A_CRUISE = 1.5  # chill's answer well over the town cap
CRAWL_V = 1.8  # m/s, 4 mph
A_STALL = -0.05  # the model holding a crawl (route 00000066 t=252-255)


def frames(t):
  return round(t / DT)


def step(ctl, n, v, a_model=A_STALL, reset_state=False, **sm_kwargs):
  out = a_model
  sm = build_sm(v, **sm_kwargs)
  for _ in range(n):
    out = ctl.update(sm, a_model, A_CRUISE, True, reset_state)
  return out


def depart(stop_engaged=True, rolling_s=0., release_v=CRAWL_V, **release_kwargs):
  """Engaged stop, a gas press that overrides (resetting the controller), release, 0.3 s more reset.
  release_kwargs hold on the release frame only, so they test the arming, not the window's exits."""
  ctl = E2ESetSpeedController(params=FakeParams(ExperimentalModeSetSpeed=True), dt=DT)
  step(ctl, frames(2.), 0., reset_state=not stop_engaged)
  if rolling_s:
    step(ctl, frames(rolling_s), 3., reset_state=not stop_engaged)
  step(ctl, frames(1.), 1., reset_state=True, gas=True)
  step(ctl, 1, release_v, reset_state=True, **release_kwargs)
  step(ctl, frames(0.3), min(release_v, CRAWL_V), reset_state=True)
  return ctl


def test_lifts_a_stalled_pull_away():
  ctl = depart()
  assert ctl.depart_t is not None
  out = step(ctl, frames(2.), CRAWL_V)
  assert ctl.lift == pytest.approx(c.TOWN_LIFT_MAX)
  assert out == pytest.approx(A_STALL + c.TOWN_LIFT_MAX)


def test_the_drivers_press_stands_in_for_the_throttle_probability():
  # route 00000066 t=252-255: the model's own throttle probability sat at 0.1-0.5 during the crawl
  ctl = depart()
  step(ctl, frames(2.), CRAWL_V, throttle_prob=0.1)
  assert ctl.lift == pytest.approx(c.TOWN_LIFT_MAX)


def test_without_the_window_stop_and_go_stays_the_models():
  # the same crawl with no press from a stop: nothing below walking pace
  ctl = E2ESetSpeedController(params=FakeParams(ExperimentalModeSetSpeed=True), dt=DT)
  assert step(ctl, frames(3.), CRAWL_V) == A_STALL
  assert ctl.lift == 0.


@pytest.mark.parametrize("kwargs", [
  {"rolling_s": c.DEPART_STOP_T + 1.},  # the stop was too long before the press
  {"stop_engaged": False},  # the press began disengaged
  {"release_v": c.DEPART_MAX_V + 0.2},  # let go at speed
  {"blinker": True},  # waiting to turn
  {"lead": (20., 0.)},  # a car ahead
  {"steer_deg": c.DEPART_STEER_ARM + 5.},  # turning
])
def test_does_not_arm(kwargs):
  sm_kwargs = {k: v for k, v in kwargs.items() if k in ("blinker", "lead", "steer_deg")}
  ctl = depart(**{k: v for k, v in kwargs.items() if k not in sm_kwargs}, **sm_kwargs)
  assert ctl.depart_t is None
  # whatever blocked it at the release clears: still no window
  step(ctl, frames(2.), CRAWL_V)
  assert ctl.lift == 0.


@pytest.mark.parametrize("kwargs", [{"brake": True}, {"lead": (20., 0.)}, {"blinker": True},
                                    {"steer_deg": c.DEPART_STEER_END + 5.}])
def test_ends_at_once(kwargs):
  ctl = depart()
  step(ctl, frames(2.), CRAWL_V)
  assert ctl.lift > 0.
  step(ctl, 1, CRAWL_V, **kwargs)
  assert ctl.depart_t is None
  assert ctl.lift == 0.


def test_model_braking_still_lets_go_at_once():
  ctl = depart()
  step(ctl, frames(2.), CRAWL_V)
  assert step(ctl, 1, CRAWL_V, a_model=c.GAIN_BP[0]) == c.GAIN_BP[0]
  assert ctl.lift == 0.


def test_a_planned_stop_ends_it_at_once():
  # creep forward at a stop line and let go: the model plans to stop again 10 m on
  ctl = depart()
  step(ctl, frames(1.), CRAWL_V)
  assert ctl.lift > 0.
  stop_plan = np.maximum(CRAWL_V * (1. - T_IDXS / 3.), 0.)
  assert step(ctl, 1, CRAWL_V, a_model=-0.16, plan_v=stop_plan) == -0.16
  assert ctl.depart_t is None
  assert ctl.lift == 0.


def test_a_stop_made_disengaged_does_not_count():
  # stopped disengaged, engaged rolling, then the press: not a departure from an engaged stop
  ctl = E2ESetSpeedController(params=FakeParams(ExperimentalModeSetSpeed=True), dt=DT)
  step(ctl, frames(2.), 0., reset_state=True)
  step(ctl, frames(0.5), 2., reset_state=False)
  step(ctl, frames(1.), 2., reset_state=True, gas=True)
  step(ctl, frames(0.3), CRAWL_V, reset_state=True)
  assert ctl.depart_t is None
  step(ctl, frames(2.), CRAWL_V)
  assert ctl.lift == 0.


def test_a_nan_lead_distance_counts_as_near():
  ctl = depart()
  step(ctl, frames(1.), CRAWL_V)
  step(ctl, 1, CRAWL_V, lead=(float('nan'), 0.))
  assert ctl.depart_t is None
  assert ctl.lift == 0.


def test_runs_out_and_eases_off():
  ctl = depart()
  lifts = []
  for _ in range(frames(c.DEPART_T + 2.)):
    step(ctl, 1, CRAWL_V)
    lifts.append(ctl.lift)
  assert ctl.depart_t is None
  assert max(lifts) == pytest.approx(c.TOWN_LIFT_MAX)
  drops = [a - b for a, b in zip(lifts, lifts[1:], strict=False)]
  assert max(drops) <= c.LIFT_FALL * DT + 1e-9
  assert lifts[-1] == 0.


def test_ends_once_at_full_speed():
  ctl = depart()
  step(ctl, frames(1.), CRAWL_V)
  step(ctl, 1, c.FULL_SPEED)
  assert ctl.depart_t is None


def test_cancelled_cruise_ends_the_window():
  # control never came back after the press: re-engaging later in the window does not lift
  ctl = depart()
  step(ctl, frames(c.DEPART_REENGAGE_T), CRAWL_V, reset_state=True)
  assert ctl.depart_t is None
  step(ctl, frames(1.), CRAWL_V)
  assert ctl.lift == 0.


def test_a_second_press_ends_the_window():
  ctl = depart()
  step(ctl, frames(1.), CRAWL_V)
  step(ctl, 1, CRAWL_V, reset_state=True, gas=True)
  assert ctl.depart_t is None
