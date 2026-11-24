"""
The Smart Cruise Control part of develop's LongitudinalPlannerSP (openpilot/sunnypilot/selfdrive/controls/lib/
longitudinal_planner.py at 197598c0f6) for the Mazda port: update_targets with its SCC targets only (StarPilot's own
speed limit ceiling stands in for develop's SLA as the cruise target it is given), and update_cruise_accel. Same law;
there is no ICBM, so SCC acts on openpilot long only, and longitudinalPlanSP is not published.
"""
import math
import threading
from enum import IntEnum

import numpy as np

from opendbc.car.interfaces import ACCEL_MAX
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.curve_speed import corrections as curve_corrections
from openpilot.starpilot.saved_source import read_saved
from openpilot.starpilot.longitudinal.scc.limits import OP_LONG_A_BUDGET, OP_LONG_RELEASE_JERK_X, op_long_cruise_jerk
from openpilot.starpilot.longitudinal.scc.zoompilot import make_smart_cruise_control
from openpilot.starpilot.longitudinal.scc.zoompilot.vision_controller import table_lat

CORRECTIONS_KEY = curve_corrections.KEY


def close_lead(sm, v_ego: float) -> bool:
  """A brake press is a curve correction only with fresh radar showing no lead inside max(3 s, 15 m), on either
  track; missing, invalid, stale or unreadable radar counts as a lead. SubMaster supports keyed access only."""
  try:
    rs = sm['radarState']
    if not getattr(sm, 'valid', {}).get('radarState', False) or not getattr(sm, 'alive', {}).get('radarState', False):
      return True
    limit = max(curve_corrections.LEAD_HEADWAY_S * v_ego, 15.)
    for lead in (rs.leadOne, rs.leadTwo):
      if lead.present and not (math.isfinite(lead.dRel) and lead.dRel >= limit):
        return True
    return False
  except (AttributeError, KeyError, TypeError, ValueError):
    return True


class SccSource(IntEnum):
  # develop's custom.LongitudinalPlanSP.LongitudinalPlanSource values these targets use
  cruise = 0
  sccVision = 1


SCC_SOURCES = (SccSource.sccVision,)


class SmartCruiseTargets:
  def __init__(self, CP):
    self.CP = CP
    self.scc = make_smart_cruise_control(CP)
    self.source = SccSource.cruise
    self.output_v_target = 0.
    self.output_a_target = 0.
    self.seed_fault_logged = False
    self.a_cruise_prev = 0.
    self.v_ego = 0.
    self.params = Params()
    self.corrections = curve_corrections.Corrections()
    self.corrections_save_frames = 0
    self.corrections_writer: threading.Thread | None = None
    # plannerd sets it when it holds the curve learning lease (the parked reset's exclusion); without it nothing is learned
    self.corrections_owned = False
    # the planner's verdict for the last published plan: the vision target was the one it followed
    self.binding = False
    self._apply_corrections()

  def take_ownership(self, owned: bool) -> None:
    """plannerd, once it holds (or failed to get) the curve learning lease. The saved corrections are read only now,
    under the lease, so a parked reset that finished before it cannot be undone by values loaded earlier."""
    self.corrections_owned = owned
    self.corrections = curve_corrections.Corrections(self._read_corrections() if owned else None)
    self.corrections_save_frames = 0
    self._apply_corrections()

  def _read_corrections(self) -> tuple[float, ...]:
    """The same bounded read the settings page uses: an oversized or unreadable document is no correction."""
    try:
      raw, readable = read_saved(self.params, CORRECTIONS_KEY, curve_corrections.DOC_LIMIT)
    except Exception:
      return curve_corrections.UNIT
    return curve_corrections.load(raw) if readable else curve_corrections.UNIT

  def _apply_corrections(self) -> None:
    set_corrections = getattr(self.scc.vision, 'set_corrections', None)
    if set_corrections is not None:
      set_corrections(self.corrections.scales)

  def _update_corrections(self, sm, long_enabled: bool) -> None:
    """The driver's gas and brake against the curve controller nudge its table, within +-10% (curve_speed/corrections)."""
    vision = self.scc.vision
    cs = sm['carState']
    if not vision.enabled or not self.corrections_owned:
      self.corrections.idle(gas=bool(cs.gasPressed), brake=bool(cs.brakePressed))
      return
    v = cs.vEgo
    self.corrections.step(dt=DT_MDL, v_ego=float(v), lat_accel=float(abs(sm['controlsState'].curvature) * v ** 2),
                          gas=bool(cs.gasPressed), brake=bool(cs.brakePressed), long_active=bool(sm['carControl'].longActive),
                          engaged=bool(sm['carControl'].enabled),
                          slowing=self.binding, close_lead=close_lead(sm, v),
                          table_lat=table_lat(v) * vision.lat_scale)
    self._apply_corrections()
    self.corrections_save_frames += 1
    if (self.corrections.dirty and self.corrections_save_frames * DT_MDL >= curve_corrections.SAVE_INTERVAL_S and
        (self.corrections_writer is None or not self.corrections_writer.is_alive())):
      self.corrections_save_frames = 0
      self.corrections_writer = threading.Thread(target=self._write_corrections, daemon=True, name='curve-corrections',
                                                 args=(self.corrections.document(), self.corrections.revision))
      self.corrections_writer.start()

  def _write_corrections(self, document: dict, revision: int) -> None:
    """Off the planner thread; the revision counts as saved only once the write is on disk."""
    try:
      self.params.put(CORRECTIONS_KEY, document, block=True)
      self.corrections.saved_revision = max(self.corrections.saved_revision, revision)
    except Exception:
      cloudlog.exception("curve corrections: save failed")

  def close_corrections(self, timeout: float = 2.0) -> bool:
    """plannerd, before it releases the learning lease: stop learning and finish the write in flight, so a parked reset
    that takes the lease next cannot be overwritten by it. True when nothing is left writing."""
    self.corrections_owned = False
    writer = self.corrections_writer
    if writer is not None:
      writer.join(timeout)
    return writer is None or not writer.is_alive()

  def update_targets(self, sm, v_ego: float, a_ego: float, v_cruise: float, v_cruise_setpoint: float,
                     a_cruise: float, lead_pause: bool = False) -> tuple[float, float]:
    """develop's update_targets: the lowest of the cruise target (v_cruise: the set speed, under StarPilot's speed limit
    ceiling) and the SCC targets, and the MPC seed it hands back. v_cruise_setpoint: what SCC is given as the set speed.
    a_cruise: the host's cruise candidate from last frame, before it builds this frame's."""
    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override
    long_enabled = long_enabled and bool(self.CP.openpilotLongitudinalControl)
    # StarPilot port: one curve controller, the vision solver under the Curve Speed Controller toggle (no map controller)
    self.scc.vision.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise_setpoint)

    targets = {SccSource.cruise: (v_cruise, a_ego)}
    if not lead_pause:  # "Pause while following a lead": the vision target sits out while a lead is (or may be) followed
      targets[SccSource.sccVision] = (self.scc.vision.output_v_target, self.scc.vision.output_a_target)
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

    self._update_corrections(sm, long_enabled)

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
