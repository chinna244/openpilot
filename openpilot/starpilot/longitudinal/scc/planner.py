"""
The Smart Cruise Control part of develop's LongitudinalPlannerSP (openpilot/sunnypilot/selfdrive/controls/lib/
longitudinal_planner.py at 197598c0f6) for the Mazda port: update_targets with its SCC targets only (StarPilot's own
speed limit ceiling stands in for develop's SLA as the cruise target it is given), and update_cruise_accel. Same law;
there is no ICBM, so SCC acts on openpilot long only, and longitudinalPlanSP is not published.
"""
import math
from enum import IntEnum

import numpy as np

from opendbc.car.interfaces import ACCEL_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.longitudinal.scc.limits import OP_LONG_A_BUDGET, OP_LONG_RELEASE_JERK_X, op_long_cruise_jerk
from openpilot.starpilot.longitudinal.scc.mapd_targets import MAPD_SERVICE, MapdTargets
from openpilot.starpilot.longitudinal.scc.zoompilot import make_smart_cruise_control


class SccSource(IntEnum):
  # develop's custom.LongitudinalPlanSP.LongitudinalPlanSource values these targets use
  cruise = 0
  sccVision = 1
  sccMap = 2


SCC_SOURCES = (SccSource.sccVision, SccSource.sccMap)


class SmartCruiseTargets:
  def __init__(self, CP):
    self.CP = CP
    self.scc = make_smart_cruise_control(CP)
    self.mapd = MapdTargets()
    self.scc.map.read_targets = self.mapd.targets
    self.source = SccSource.cruise
    self.output_v_target = 0.
    self.output_a_target = 0.
    self.seed_fault_logged = False
    self.a_cruise_prev = 0.
    self.v_ego = 0.

  def update_targets(self, sm, v_ego: float, a_ego: float, v_cruise: float, v_cruise_setpoint: float,
                     a_cruise: float) -> tuple[float, float]:
    """develop's update_targets: the lowest of the cruise target (v_cruise: the set speed, under StarPilot's speed limit
    ceiling) and the SCC targets, and the MPC seed it hands back. v_cruise_setpoint: what SCC is given as the set speed.
    a_cruise: the host's cruise candidate from last frame, before it builds this frame's."""
    self.mapd.update(sm)
    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override
    long_enabled = long_enabled and bool(self.CP.openpilotLongitudinalControl)
    # develop's SmartCruiseControl.update, map then vision; the map needs plannerd's GPS and mapd subscriptions
    services = getattr(sm, 'services', sm)
    if MAPD_SERVICE in services and self.scc.map.gps_location_service in services:
      self.scc.map.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise_setpoint)
    self.scc.vision.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise_setpoint)

    targets = {
      SccSource.cruise: (v_cruise, a_ego),
      SccSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      SccSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
    }
    self.source = min(targets, key=lambda k: targets[k][0])
    v_target, a_target = targets[self.source]

    # a single NaN in the MPC seed poisons the solver: fall back to what the cruise path would have given
    if not (math.isfinite(v_target) and math.isfinite(a_target)):
      if not self.seed_fault_logged:
        self.seed_fault_logged = True
        cloudlog.error(f"longitudinal_planner: non-finite target from {self.source}: v={v_target} a={a_target}")
      v_target = v_cruise if math.isfinite(v_cruise) else v_ego
      a_target = a_ego if math.isfinite(a_ego) else 0.
      if not math.isfinite(v_target):
        v_target = 0.
      self.source = SccSource.cruise

    # the seed may be lowered below the plan's last output, never raised
    if self.CP.openpilotLongitudinalControl and math.isfinite(a_ego):
      a_target = min(a_target, a_ego)

    self.output_v_target, self.output_a_target = v_target, a_target
    self.a_cruise_prev, self.v_ego = a_cruise, sm['carState'].vEgo
    return self.output_v_target, self.output_a_target

  def update_cruise_accel(self, a_cruise: float, unlimited: bool = False) -> float:
    """develop's update_cruise_accel: while SCC is the target, the cruise candidate is rebuilt from it on SCC's budget.
    Positive accel is released faster than brake is built, and never past one cruise step below zero.
    unlimited: what the target asks this cycle, without the jerk limit (the gas handover's ask)."""
    if self.source not in SCC_SOURCES:
      return a_cruise
    j = op_long_cruise_jerk(self.v_ego) * DT_MDL
    a_limiter = np.clip(self.output_v_target - self.v_ego, -OP_LONG_A_BUDGET, ACCEL_MAX)
    if unlimited:
      return min(a_cruise, float(a_limiter))
    a_low = self.a_cruise_prev - j
    if self.a_cruise_prev > 0.:
      a_low = min(a_low, max(self.a_cruise_prev - OP_LONG_RELEASE_JERK_X * j, -j))
    return min(a_cruise, float(np.clip(a_limiter, a_low, self.a_cruise_prev + j)))
