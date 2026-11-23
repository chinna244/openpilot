"""Route 00000003--941020f467 t=126: accepting a lower SLA limit (35->30) braked -1.10 m/s^2 under a firmer StarPilot
braking profile; filter-dev's set-speed drop law brakes at A_CRUISE_MIN (-0.52 measured for 39->35)."""
import inspect

from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MIN, get_mazda_cruise_accel, mazda_slc_cruise_floor


def test_slc_drop_uses_the_gentler_floor():
  assert mazda_slc_cruise_floor(-1.2, True) == A_CRUISE_MIN
  assert mazda_slc_cruise_floor(-0.3, True) == -0.3  # a softer profile stays softer


def test_everything_else_keeps_the_profile_floor():
  assert mazda_slc_cruise_floor(-1.2, False) == -1.2


def test_route_126s_drop_brakes_no_harder_than_filter_dev():
  v_ego, target = 33.9 * 0.44704, 30 * 0.44704
  a, a_min_old, a_min_new = 0., 0., 0.
  a_old = a_new = 0.
  for _ in range(60):  # 3 s at 20 Hz
    a_old = get_mazda_cruise_accel(target, v_ego, a_old, a_old, 0.05, cruise_min=-1.2)
    a_new = get_mazda_cruise_accel(target, v_ego, a_new, a_new, 0.05, cruise_min=mazda_slc_cruise_floor(-1.2, True))
    a_min_old, a_min_new = min(a_min_old, a_old), min(a_min_new, a_new)
  assert a_min_old < -1.0 and a_min_new >= A_CRUISE_MIN - 1e-6


def test_planner_wires_slc_ownership_strictly():
  source = inspect.getsource(lp.LongitudinalPlanner)
  call = source.index('mazda_min = mazda_slc_cruise_floor(')
  wiring = source[call:call + 400]
  for need in ('ceiling.effective_mps is not None', 'v_cruise == ceiling.effective_mps', 'not relevant_lead',
               'curve.effective_mps is None', 'navigation_target is None', 'self.force_stop_plan.speed_ceiling_mps is None'):
    assert need in wiring, need
