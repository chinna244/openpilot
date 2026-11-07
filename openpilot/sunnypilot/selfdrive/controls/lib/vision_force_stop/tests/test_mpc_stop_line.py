from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource, STOP_DISTANCE


def solve(stop_line_m, v_ego=10.):
  mpc = LongitudinalMpc()
  mpc.set_weights(True)
  radar = messaging.new_message('radarState').radarState
  for _ in range(20):
    mpc.set_cur_state(v_ego, 0.)
    mpc.update(radar, stop_line_m=stop_line_m)
  return mpc


def test_stop_line_brakes_to_a_stop_before_it():
  mpc = solve(20.)
  assert mpc.source == LongitudinalPlanSource.cruise
  assert mpc.a_solution[:5].min() < -1.0
  assert mpc.v_solution[-1] < 0.5
  assert mpc.x_sol[-1, 0] < 20. + STOP_DISTANCE  # short of the standing obstacle, as for a stopped car


def test_no_stop_line_keeps_speed():
  mpc = solve(None)
  assert mpc.a_solution[:5].min() > -0.1
