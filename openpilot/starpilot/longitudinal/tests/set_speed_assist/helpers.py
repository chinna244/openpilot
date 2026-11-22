"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.starpilot.longitudinal.set_speed_assist import T_IDXS

# update() arguments for an engaged, lead-free e2e frame
ENGAGED = {'is_e2e': True, 'reset_state': False}


def plan_ramp(v_ego: float, drop: float) -> np.ndarray:
  """The model's plan easing from v_ego down by drop over the first 10 s of the horizon."""
  return v_ego - drop * np.minimum(T_IDXS, 10.) / 10.


def build_sm(v_ego: float, plan_v=None, yaw_rate=0., curvature=0., gas=False, brake=False, should_stop=False,
             hard_brake=False, force_decel=False, lane_change=False, throttle_prob=1., lead=None, steer_deg=0.,
             blinker=False) -> dict:
  """Readers for the four services the controller reads. plan_v and yaw_rate take a scalar or a
  33-point trajectory on the model's T_IDXS. lead: (dRel, vRel) of a present leadOne, or True for a
  present one at rest relative to us."""
  cs = messaging.new_message('carState')
  cs.carState.vEgo = v_ego
  cs.carState.gasPressed = gas
  cs.carState.brakePressed = brake
  cs.carState.steeringAngleDeg = steer_deg
  cs.carState.leftBlinker = blinker

  md = messaging.new_message('modelV2')
  vel = np.full(len(T_IDXS), v_ego) if plan_v is None else np.broadcast_to(np.asarray(plan_v, dtype=float), T_IDXS.shape)
  md.modelV2.velocity.x = vel.tolist()
  # positions integrate the plan's speeds, as the model's own do
  md.modelV2.position.x = np.concatenate(([0.], np.cumsum(np.diff(T_IDXS) * (vel[1:] + vel[:-1]) / 2.))).tolist()
  md.modelV2.orientationRate.z = np.broadcast_to(np.asarray(yaw_rate, dtype=float), T_IDXS.shape).tolist()
  md.modelV2.action.desiredCurvature = curvature
  md.modelV2.action.shouldStop = should_stop
  md.modelV2.meta.hardBrakePredicted = hard_brake
  md.modelV2.meta.disengagePredictions.gasPressProbs = [1., throttle_prob, 1.]
  md.modelV2.meta.laneChangeState = log.LaneChangeState.laneChangeStarting if lane_change else log.LaneChangeState.off

  rs = messaging.new_message('radarState')
  if lead is not None and lead is not False:
    rs.radarState.leadOne.present = True
    if lead is not True:
      rs.radarState.leadOne.dRel, rs.radarState.leadOne.vRel = (float(x) for x in lead)

  ctrl = messaging.new_message('controlsState')
  ctrl.controlsState.forceDecel = force_decel

  return {'carState': cs.carState.as_reader(), 'modelV2': md.modelV2.as_reader(),
          'radarState': rs.radarState.as_reader(), 'controlsState': ctrl.controlsState.as_reader()}


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get_bool(self, key):
    return bool(self.values.get(key, False))


def controller_class():
  from openpilot.starpilot.longitudinal.set_speed_assist import E2ESetSpeedController as Base

  class E2ESetSpeedController(Base):
    """develop's constructor: the toggle comes from params, read every PARAMS_UPDATE_PERIOD (3 s) as develop does."""
    def __init__(self, params=None, dt=0.05):
      super().__init__(dt=dt)
      self.params = params or FakeParams()
      self._period = int(3 / dt)

    def update(self, *args, **kwargs):
      if (self.frame + 1) % self._period == 0:
        self.enabled = self.params.get_bool("ExperimentalModeSetSpeed")
      return super().update(*args, enabled=self.enabled, **kwargs)
  return E2ESetSpeedController


def op_long_cruise_jerk(v_ego: float) -> float:
  """develop's smart_cruise_control.limits.op_long_cruise_jerk, for the jerk assertions."""
  return float(np.interp(v_ego, [0., 10., 25., 40.], [1.6, 1.2, 0.8, 0.6]))
