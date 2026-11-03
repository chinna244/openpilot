"""The planner takes a learned stop spot's candidate (stop_spots.py) into its min(), only on a fresh, good fix."""
import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.tests.test_cruise_handover import build_sm, planner
from openpilot.sunnypilot.selfdrive.controls.lib.stop_spots import StopSpots
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_stop_spots import SPOT, at


def approach(p, along=-15., has_fix=True, accuracy=3., fresh=True, n=10, steer=0., **kwargs):
  for _ in range(n):
    sm = build_sm(**kwargs)
    cs = sm['carState'].as_builder()
    cs.steeringAngleDeg = steer
    sm['carState'] = cs.as_reader()
    gps = messaging.new_message(p.gps_location_service)
    g = getattr(gps, p.gps_location_service)
    g.latitude, g.longitude = at(along)
    g.bearingDeg, g.hasFix, g.horizontalAccuracy = SPOT["bearing"], has_fix, accuracy
    sm[p.gps_location_service] = g.as_reader()
    sm.updated[p.gps_location_service] = fresh
    p.update(sm)
  return p.output_a_target


def with_spot():
  p = planner()
  p.stop_spots = StopSpots([SPOT])
  return p


def test_a_spot_ahead_brakes_the_plan():
  base = approach(planner())
  a = approach(with_spot())
  assert a < base and a < -0.1
  # 6 m/s, 13 m to the aim: the stop needs 1.38 m/s2, built at START_JERK (2 m/s3) from the third agreeing fix
  assert a == pytest.approx(-0.8, abs=0.1)


@pytest.mark.parametrize("kwargs", [{"has_fix": False}, {"accuracy": 8.}, {"fresh": False}, {"gas": True, "override": True}])
def test_no_fix_a_poor_or_stale_fix_or_the_gas_leave_the_plan_alone(kwargs):
  assert approach(with_spot(), **kwargs) == pytest.approx(approach(planner(), **kwargs))


def test_a_gas_tap_on_the_car_pauses_the_stop_and_it_resumes():
  """On the car the gas is an override: longControlState off, so reset_state. The stop pauses, then carries on."""
  p = with_spot()
  approach(p, along=-19.)
  assert p.stop_spots.braking
  approach(p, along=-17., gas=True, override=True, n=5)
  assert p.stop_spots.spot is SPOT and p.stop_spots.paused and not p.stop_spots.braking
  a = approach(p, along=-15., n=10)
  assert p.stop_spots.braking and a < -0.1


def test_the_brake_pedal_ends_the_pass():
  p = with_spot()
  approach(p, along=-19.)
  approach(p, along=-17., brake=True, override=True, n=2)
  assert p.stop_spots.spot is None and p.stop_spots.done_spot is SPOT


def test_a_lead_short_of_the_spot_binds_through_its_own_candidate():
  """A lead is not a reason to drop the stop: behind it the lead MPC is lower, and the min() takes it."""
  p = with_spot()
  assert approach(p, lead=8.) == pytest.approx(approach(planner(), lead=8.))
  assert p.stop_spots.spot is SPOT and p.stop_spots.braking, "the stop is still pending behind the lead"


def test_turning_the_wheel_off_short_of_the_spot_ends_the_pass():
  p = with_spot()
  approach(p)
  assert p.stop_spots.committed
  assert approach(p, steer=250., n=1) == pytest.approx(approach(planner(), steer=250., n=11))
  assert p.stop_spots.spot is None
