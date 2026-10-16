"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import T_IDXS

# update() arguments for an engaged, lead-free e2e frame
ENGAGED = {'is_e2e': True, 'reset_state': False, 'dec_active': False, 'allow_throttle': True, 'fcw': False, 'accel_coast': 0.}


def plan_pos(vel: np.ndarray) -> np.ndarray:
  """Positions along a plan, integrated from its speeds."""
  return np.r_[0., np.cumsum(0.5 * (vel[1:] + vel[:-1]) * np.diff(T_IDXS))]


def build_sm(v_ego: float, plan_v=None, yaw_rate=0., curvature=0., lead=False, gas=False, brake=False,
             should_stop=False, hard_brake=False, force_decel=False, lane_change=False,
             lead_d=0., lead_vrel=0., lead_a=0., lead2=None) -> dict:
  """Readers for the four services the controller reads. plan_v and yaw_rate take a scalar or a
  33-point trajectory on the model's T_IDXS. A lead is where lead_d, lead_vrel and lead_a put it: left
  at zero it is on the bumper, a hazard at any speed."""
  cs = messaging.new_message('carState')
  cs.carState.vEgo = v_ego
  cs.carState.gasPressed = gas
  cs.carState.brakePressed = brake

  md = messaging.new_message('modelV2')
  vel = np.full(len(T_IDXS), v_ego) if plan_v is None else np.broadcast_to(np.asarray(plan_v, dtype=float), T_IDXS.shape)
  md.modelV2.velocity.x = vel.tolist()
  md.modelV2.position.x = plan_pos(vel).tolist()
  md.modelV2.orientationRate.z = np.broadcast_to(np.asarray(yaw_rate, dtype=float), T_IDXS.shape).tolist()
  md.modelV2.action.desiredCurvature = curvature
  md.modelV2.action.shouldStop = should_stop
  md.modelV2.meta.hardBrakePredicted = hard_brake
  md.modelV2.meta.laneChangeState = log.LaneChangeState.laneChangeStarting if lane_change else log.LaneChangeState.off

  rs = messaging.new_message('radarState')
  rs.radarState.leadOne.present = lead
  rs.radarState.leadOne.dRel = lead_d
  rs.radarState.leadOne.vRel = lead_vrel
  rs.radarState.leadOne.aLeadK = lead_a
  if lead2 is not None:  # (dRel, vRel, aLeadK)
    rs.radarState.leadTwo.present = True
    rs.radarState.leadTwo.dRel, rs.radarState.leadTwo.vRel, rs.radarState.leadTwo.aLeadK = lead2

  ctrl = messaging.new_message('controlsState')
  ctrl.controlsState.forceDecel = force_decel

  return {'carState': cs.carState.as_reader(), 'modelV2': md.modelV2.as_reader(),
          'radarState': rs.radarState.as_reader(), 'controlsState': ctrl.controlsState.as_reader()}
