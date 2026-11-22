"""controlsd's Mazda lateral owner: develop's ControlsLateralZP (one torque tune per model size, the
steer-limit classifier, the speed-binned torque dispatch) plus the per-frame torque-parameter,
modelV2 and lag feeds develop's controlsd gives a torque controller.

Ported from zoompilot develop 197598c0f6 (sunnypilot/selfdrive/controls/controls_lateral_zp.py,
controlsd_ext.py, selfdrive/controls/controlsd.py). Lane-change smoothing and blinker pause stay
StarPilot's (not part of this port).
"""
import time

import numpy as np

from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.starpilot.car.mazda.lateral.lateral_tune import get_steer_slew_schedule
from openpilot.starpilot.car.mazda.lateral.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
from openpilot.starpilot.car.mazda.lateral.latcontrol_torque_v1 import LatControlTorque as LatControlTorqueV1
from openpilot.starpilot.car.mazda.lateral.latcontrol_torque_v2 import LatControlTorque as LatControlTorqueV2
from openpilot.starpilot.car.mazda.lateral.speed_bin_learner import bins_from_points
from openpilot.starpilot.car.mazda.lateral.steer_limit import classify
from openpilot.starpilot.car.mazda.lateral.torque_tune import resolved_tune_versions

# how long lateral control has to be inactive, continuously, before the torque tune follows
# a model change: a one-frame drop (a steer fault flicker) would otherwise swap and resume
# steering on a controller no inactive frame had primed
TUNE_SWAP_INACTIVE_FRAMES = round(0.5 / DT_CTRL)
PARAMS_UPDATE_PERIOD = 3  # s, develop's sunnypilot PARAMS_UPDATE_PERIOD
LAGD_VALUE_CACHE_DEFAULT = 0.2  # develop's LagdValueCache default, the lag before the first refresh


class MazdaLateral:
  """Builds develop's torque controllers and runs develop's per-frame lateral bookkeeping around
  them. controlsd keeps the selected controller in self.LaC; this object swaps it."""

  def __init__(self, CP, CI, params, dt=DT_CTRL):
    self.CP = CP
    self.params = params
    # the extension's lag (develop's ModelStateBase.lat_delay, refreshed every PARAMS_UPDATE_PERIOD
    # through get_lat_delay with LagdToggle on: lagd's lateralDelay)
    self.lat_delay = LAGD_VALUE_CACHE_DEFAULT
    self._param_update_time = 0.0

    # steer-limit classifier (steer_limit.py); Mazda is torque-steered, so the schedule exists
    self._steer_slew_schedule = get_steer_slew_schedule(CP)
    self._lat_active_last = False
    # frames in a row with CC.latActive false; the torque tune swap waits for TUNE_SWAP_INACTIVE_FRAMES of them
    self._inactive_frames = 0
    self._applied_torque_prev: float | None = None

    # One controller per model size, built once; every drive starts on the small model's.
    versions = resolved_tune_versions(params, CP.lateralTuning.which() == 'torque')

    def build(version):
      if version == 0.0:
        return LatControlTorqueV0(CP, CI, dt)
      if version == 2.0:
        return LatControlTorqueV2(CP, CI, dt)
      return LatControlTorqueV1(CP, CI, dt)

    built = {version: build(version) for version in dict.fromkeys(versions.values())}
    self._lac_by_size = {big: built[version] for big, version in versions.items()}
    self._lacs = tuple(built.values())
    self.LaC = self._lac_by_size[False]
    cloudlog.warning("controlsd: Mazda torque tunes small v%s, big v%s", versions[False], versions[True])

  # ---- develop's state_control feeds, before the controller runs ----

  def refresh_params(self, sm) -> None:
    """develop's ControlsExt.get_params_sp lag refresh (LagdToggle on: lagd's value)."""
    if time.monotonic() - self._param_update_time > PARAMS_UPDATE_PERIOD:
      self.lat_delay = sm['lateralDelay'].lateralDelay
      self._param_update_time = time.monotonic()

  def update_inputs(self, sm) -> None:
    """develop's controlsd torque block: live torque params, then the extension's modelV2 and lag."""
    torque_params = sm['lateralTorqueParameters']
    if sm.all_checks(['lateralTorqueParameters']) and torque_params.useParams:
      self.LaC.update_torque_parameters(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                        torque_params.frictionCoefficientFiltered)
      self.LaC.extension.update_limits()
    self.LaC.extension.update_model_v2(sm['modelV2'])
    self.LaC.extension.update_lateral_lag(self.lat_delay)

  def note_lat_active(self, lat_active: bool) -> None:
    self._lat_active_last = bool(lat_active)
    self._inactive_frames = 0 if lat_active else self._inactive_frames + 1

  # ---- develop's run_lateral_zp, after publish ----

  def reclassify_steer_limit(self, sm, steer_limited_by_safety: bool) -> bool:
    """Replaces controlsd's raw torque-mismatch flag with the classifier's directional,
    rail-aware one before the next frame's LaC.update reads it. While lateral is inactive the
    flag is returned as controlsd set it."""
    ext = getattr(self.LaC, 'extension', None)
    if ext is None or self._steer_slew_schedule is None:
      return steer_limited_by_safety
    if not self._lat_active_last:
      self._applied_torque_prev = None
      return steer_limited_by_safety
    v_ego = sm['carState'].vEgo
    applied = float(sm['carOutput'].actuatorsOutput.torque)
    bp, up, down = self._steer_slew_schedule
    rail_scale = ext.rail_scale_at(v_ego)
    limit = classify(ext.commanded_torque, applied, self._applied_torque_prev,
                     float(np.interp(v_ego, bp, up)), float(np.interp(v_ego, bp, down)),
                     rail_scale, steer_limited_by_safety, ext.last_error, ext.integrator)
    self._applied_torque_prev = applied
    return limit.limited

  def dispatch_speed_bins(self, sm) -> None:
    """Speed-dependent torque: apply torqued's per-bin values to both sizes' controllers, so the
    idle one holds current bins when it takes over. Handles activation and deactivation."""
    if (self.CP.lateralTuning.which() == 'torque'
        and sm.updated.get('lateralTorqueParameters', False)
        and sm.all_checks(['lateralTorqueParameters'])):
      tp = sm['lateralTorqueParameters']
      # develop publishes the bins on their own service beside each lateralTorqueParameters; here
      # they ride on it (speed_bin_learner.bins_from_points), and none means "no bins"
      tp_sp = bins_from_points(tp.points)
      for lac in self._lacs:
        lac.extension.update_speed_dep_torque(tp, tp_sp)

  def select_lateral_control(self, sm):
    """modelV2.big says which model produced the frame; a change swaps to the controller tuned
    for it, reset, once lateral has been inactive for TUNE_SWAP_INACTIVE_FRAMES in a row.
    Returns the controller controlsd runs from the next frame."""
    if len(self._lacs) == 1 or self._inactive_frames < TUNE_SWAP_INACTIVE_FRAMES:
      return self.LaC
    big = bool(sm['modelV2'].big)
    lac = self._lac_by_size[big]
    if lac is self.LaC:
      return self.LaC
    self.LaC = lac
    lac.reset()
    cloudlog.warning("controlsd: %s model, swapped to its torque tune", "big" if big else "small")
    return self.LaC

  def run(self, sm, steer_limited_by_safety: bool) -> tuple[object, bool]:
    """run_lateral_zp: the steer-limit flag for the next frame, the speed bins, then the tune
    swap, after develop's 3 s params refresh. Returns (the controller to run next frame, the flag)."""
    self.refresh_params(sm)
    steer_limited_by_safety = self.reclassify_steer_limit(sm, steer_limited_by_safety)
    self.dispatch_speed_bins(sm)
    return self.select_lateral_control(sm), steer_limited_by_safety
