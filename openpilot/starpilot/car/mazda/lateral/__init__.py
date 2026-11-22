"""Mazda lateral control: zoompilot develop's torque tunes, steer-limit classifier, EPS rail,
speed-binned torque learning and model curvature settings, ported to run on StarPilot for
Mazdas only (source: zoompilot develop 197598c0f6, opendbc 73b97439b2).

Every other brand keeps StarPilot's lateral stack. Measurements and design notes live in the
source's docs/zoompilot/lateral-tune.md.
"""
from typing import NamedTuple

from opendbc.car import structs


def qualified(CP) -> bool:
  """The cars that run develop's torque stack: every torque-steered Mazda, as on develop, where
  these controllers are built for any torque car and the Mazda seed picks the tunes."""
  return (CP.brand == 'mazda' and CP.steerControlType == structs.CarParams.SteerControlType.torque
          and CP.lateralTuning.which() == 'torque')


class LatControlInputs(NamedTuple):
  lateral_acceleration: float
  roll_compensation: float
  vego: float
  aego: float


def torque_from_lateral_accel_in_torque_space(latcontrol_inputs: LatControlInputs, torque_params, gravity_adjusted: bool) -> float:
  """sunnypilot's CarInterfaceBaseSP.torque_from_lateral_accel_linear_in_torque_space, which the
  Mazda interface keeps: linear in latAccelFactor."""
  return latcontrol_inputs.lateral_acceleration / float(torque_params.latAccelFactor)


def get_friction_in_torque_space(lateral_accel_error: float, lateral_accel_deadzone: float, friction_threshold: float, torque_params) -> float:
  """sunnypilot's opendbc lateral_ext.get_friction: friction in torque space (no latAccelFactor)."""
  import numpy as np
  from opendbc.car.lateral import apply_center_deadzone
  return float(np.interp(apply_center_deadzone(lateral_accel_error, lateral_accel_deadzone),
                         [-friction_threshold, friction_threshold], [-torque_params.friction, torque_params.friction]))
