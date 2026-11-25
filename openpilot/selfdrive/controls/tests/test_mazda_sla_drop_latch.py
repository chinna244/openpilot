"""Route 00000010--bc43783821 t=136: "Press -" adopted the SLA's 25 (setpoint 57.9 -> 40.2 kph in one frame, the session's
target 11.18 m/s), no SLA ceiling stayed up, and the set-speed drop braked at the driver's profile (-1.20) where filter-dev
coasts at A_CRUISE_MIN. MazdaSlaDrop marks that drop as the SLA's until the car is down to the set speed."""
import inspect
from types import SimpleNamespace as NS

from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
from openpilot.selfdrive.controls.lib.longitudinal_planner import (A_CRUISE_MIN, MazdaSlaDrop, get_mazda_cruise_accel,
                                                                    mazda_slc_cruise_floor)

MPH = 0.44704


def session(setpoint_kph, target_mps, state=5):
  return NS(setpointKph=setpoint_kph, target=target_mps, state=state)


def run(drop, frames, v_ego=35.6 * MPH, driver_v_cruise=25 * MPH, long_active=True):
  return [drop.update(s, long_active, v_ego, driver_v_cruise) for s in frames]


def test_route10_adoption_latches():
  d = MazdaSlaDrop()
  assert run(d, [session(57.9, 11.18, 2), session(40.2, 11.18, 5), session(40.2, 11.18, 1)]) == [False, True, True]


def test_driver_press_down_is_not_the_slas():
  d = MazdaSlaDrop()
  # one SET- step to 56.3 kph while the SLA's target is 25 mph: the driver's own drop
  assert run(d, [session(57.9, 11.18), session(56.3, 11.18)]) == [False, False]


def test_no_latch_outside_adapting_or_active():
  for state in (0, 1, 2, 3):
    d = MazdaSlaDrop()
    assert run(d, [session(57.9, 11.18, state), session(40.2, 11.18, state)]) == [False, False]


def test_driver_raise_keeps_it_driver_lower_ends_it():
  d = MazdaSlaDrop()
  assert run(d, [session(57.9, 11.18), session(40.2, 11.18), session(41.8, 11.18, 1)]) == [False, True, True]  # route 10 t=138
  d = MazdaSlaDrop()
  assert run(d, [session(57.9, 11.18), session(40.2, 11.18), session(38.6, 11.18, 1)]) == [False, True, False]


def test_ends_at_the_set_speed_and_without_long_or_session():
  d = MazdaSlaDrop()
  run(d, [session(57.9, 11.18), session(40.2, 11.18)])
  assert d.update(session(40.2, 11.18), True, 25 * MPH, 25 * MPH) is False
  d = MazdaSlaDrop()
  run(d, [session(57.9, 11.18), session(40.2, 11.18)])
  assert d.update(session(40.2, 11.18), False, 30 * MPH, 25 * MPH) is False
  d = MazdaSlaDrop()
  run(d, [session(57.9, 11.18), session(40.2, 11.18)])
  assert d.update(None, True, 30 * MPH, 25 * MPH) is False
  # a lost session starts over: no latch from a drop across the gap
  assert d.update(session(40.2, 11.18), True, 30 * MPH, 25 * MPH) is False


def test_unset_setpoint_is_no_drop():
  d = MazdaSlaDrop()
  assert run(d, [session(255., 0.), session(40.2, 11.18)]) == [False, False]


def test_route10_drop_brakes_no_harder_than_filter_dev():
  v, a_old, a_new, lo_old, lo_new = 35.6 * MPH, 0., 0., 0., 0.
  for _ in range(60):  # 3 s at 20 Hz toward 25 mph
    a_old = get_mazda_cruise_accel(25 * MPH, v, a_old, a_old, 0.05, cruise_min=-1.2)
    a_new = get_mazda_cruise_accel(25 * MPH, v, a_new, a_new, 0.05, cruise_min=mazda_slc_cruise_floor(-1.2, True))
    lo_old, lo_new = min(lo_old, a_old), min(lo_new, a_new)
  assert lo_old < -1.0 and lo_new >= A_CRUISE_MIN - 1e-6


def test_planner_wires_the_drop_with_the_same_exclusions():
  source = inspect.getsource(lp.LongitudinalPlanner)
  call = source.index('mazda_min = mazda_slc_cruise_floor(')
  wiring = source[call:call + 400]
  for need in ('sla_drop and v_cruise == driver_v_cruise', 'not relevant_lead', 'curve.effective_mps is None',
               'navigation_target is None', 'self.force_stop_plan.speed_ceiling_mps is None'):
    assert need in wiring, need
  # the set speed before any ceiling: Force Stop, SCC, curves, navigation, coast and the SLA ceiling all lower v_cruise after it
  assert 'driver_v_cruise = v_cruise\n    ceiling = select_cruise_ceiling(' in source
