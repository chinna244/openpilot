"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

limits.py mirrors the op-long cruise candidate's jerk table from
selfdrive/controls/lib/longitudinal_planner.py, which it cannot import: the upstream
planner imports the SP overlay, which imports this package (and it pulls in acados).
Read the values out of the upstream source instead, so a sync cannot move them silently.
The decel budget is the limiters' own; the SP planner lets it past upstream's A_CRUISE_MIN.
"""
import ast
import pathlib



from openpilot.common.basedir import BASEDIR
from openpilot.starpilot.longitudinal.scc import limits

UPSTREAM_PLANNER = pathlib.Path(BASEDIR) / "openpilot/selfdrive/controls/lib/longitudinal_planner.py"


def _module_constants(path: pathlib.Path) -> dict:
  tree = ast.parse(path.read_text())
  values = {}
  for node in tree.body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
      try:
        values[node.targets[0].id] = ast.literal_eval(node.value)
      except ValueError:
        pass
  return values


class TestOpLongMirror:
  def test_jerk_table_matches_upstream(self):
    up = _module_constants(UPSTREAM_PLANNER)
    # StarPilot port: the Mazda cruise law (get_mazda_cruise_accel) keys the jerk on A_CRUISE_MAX_BP, develop's J_CRUISE_BP
    assert limits._OP_LONG_J_BP == up["A_CRUISE_MAX_BP"]
    assert limits._OP_LONG_J_VALS == up["J_CRUISE_VALS"]

  def test_upstream_still_interpolates_jerk_over_its_breakpoints(self):
    # the mirror assumes j_cruise = interp(v_ego, J_CRUISE_BP, J_CRUISE_VALS)
    src = UPSTREAM_PLANNER.read_text()
    assert "np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)" in src

  def test_upstream_still_smooths_the_gap_over_a_second(self):
    # limits.py and Set Speed Assist's approach pace (e2e_set_speed/approach.py) are written against
    # #39266's 1 s smoothing; a change there needs both looked at again
    assert "smooth_value(v_cruise - v_ego, a_cruise_prev, tau=1.0, dt=dt)" in UPSTREAM_PLANNER.read_text()

  def test_limiter_budget_is_past_upstreams_cruise_floor(self):
    # SCC keeps its 1.2 m/s^2 (LongitudinalPlannerSP.update_cruise_accel); a set-speed drop and SLA
    # coast at upstream's gentler floor
    up = _module_constants(UPSTREAM_PLANNER)
    assert limits.OP_LONG_A_BUDGET == 1.2 > -up["A_CRUISE_MIN"]
    assert limits.OP_LONG_COAST_A == -up["A_CRUISE_MIN"]
    assert "self.scc.update_cruise_accel(a, unlimited=unlimited)" in UPSTREAM_PLANNER.read_text()
