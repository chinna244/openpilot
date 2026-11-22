"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

develop's test_latcontrol_torque_ext.py on the port's v1 controller and the CX-5 2022. NNLC is not
ported, so its cases are dropped.
"""
import numpy as np

from openpilot.cereal import log, messaging
from opendbc.car.structs import car
from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.starpilot.car.mazda.lateral.latcontrol_torque_v1 import LatControlTorque
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.test import OpenpilotTestCase


def _make_controller(enhanced=False):
  params = Params()
  params.put_bool("EnforceTorqueControl", True, block=True)
  params.put_bool("LateralJerkTorqueController", enhanced, block=True)

  car_name = "MAZDA_CX5_2022"
  CarInterface = interfaces[car_name]
  CP = CarInterface.get_non_essential_params(car_name)
  CI = CarInterface(CP)
  VM = VehicleModel(CP)
  controller = LatControlTorque(CP.as_reader(), CI, DT_CTRL)
  return controller, VM, CP


def _make_model_v2():
  model = messaging.new_message('modelV2')
  position = log.XYZTData.new_message()
  position.x = [float(x) for x in 30.0 * np.array(ModelConstants.T_IDXS)]
  model.modelV2.position = position
  orientation = log.XYZTData.new_message()
  orientation.x = [0.0 for _ in ModelConstants.T_IDXS]
  orientation.y = [0.0 for _ in ModelConstants.T_IDXS]
  model.modelV2.orientation = orientation
  velocity = log.XYZTData.new_message()
  velocity.x = [30.0 for _ in ModelConstants.T_IDXS]
  model.modelV2.velocity = velocity
  acceleration = log.XYZTData.new_message()
  acceleration.x = [0.0 for _ in ModelConstants.T_IDXS]
  acceleration.y = [0.0 for _ in ModelConstants.T_IDXS]
  model.modelV2.acceleration = acceleration
  return model


def _run_update(controller, VM):
  CS = car.CarState.new_message()
  CS.vEgo = 30
  CS.steeringPressed = False
  params = log.VehicleParameters.new_message()
  model_v2 = _make_model_v2().modelV2
  controller.extension.update_model_v2(model_v2)
  controller.extension.update_lateral_lag(0.2)
  return controller.update(True, CS, VM, params, False, 0.5, False, 0.2)


class TestLatControlTorqueExt(OpenpilotTestCase):
  def test_init_enhanced_only(self):
    controller, VM, _ = _make_controller(enhanced=True)
    assert controller.extension._jerk_aware_enabled

  def test_init_neither(self):
    controller, VM, _ = _make_controller(enhanced=False)
    assert not controller.extension._jerk_aware_enabled

  def test_update_enhanced_only(self):
    controller, VM, _ = _make_controller(enhanced=True)
    output_torque, _, pid_log = _run_update(controller, VM)
    assert pid_log.active

  def test_update_neither(self):
    controller, VM, _ = _make_controller(enhanced=False)
    output_torque, _, pid_log = _run_update(controller, VM)
    assert pid_log.active

  def test_jerk_aware_pid_limits_survive_param_resets(self):
    """Every update_limits path (live params, per-frame speed-dep override) must re-assert the
    torque-space +-steer_max limits, or the torque-space PID runs with lat-accel-space bounds
    (~latAccelFactor times too wide) and the integrator can wind up past full-scale torque."""
    controller, VM, _ = _make_controller(enhanced=True)
    assert controller.pid.pos_limit == controller.steer_max

    controller.update_torque_parameters(2.5, 0.1, 0.15)
    assert controller.pid.pos_limit == controller.steer_max

    ext = controller.extension
    ext._speed_dep_active = True
    ext._speed_dep_speed_bp = [5.0, 30.0]
    ext._speed_dep_lat_accel_factor_bp = [2.0, 1.2]
    ext._speed_dep_friction_bp = [0.15, 0.1]
    _run_update(controller, VM)
    assert controller.pid.pos_limit == controller.steer_max
    assert controller.pid.neg_limit == -controller.steer_max

  def test_stock_limits_when_extension_inactive(self):
    controller, VM, _ = _make_controller(enhanced=False)
    expected = controller.lateral_accel_from_torque(controller.steer_max, controller.torque_params)
    assert controller.pid.pos_limit == expected
    _run_update(controller, VM)
    # the Mazda's EPS rail moves steer_max (develop pinned this on a Honda, which has none); still lat-accel space
    assert controller.steer_max < 1.0
    assert controller.pid.pos_limit == controller.lateral_accel_from_torque(controller.steer_max, controller.torque_params)

  def test_single_pid_update_per_frame(self):
    """When the extension overrides the output, the stock lat-accel-space pid.update must be
    skipped: both updates hit the same integrator, and the lat-accel error is latAccelFactor
    times the torque-space error, so double updating scales effective ki by (1 + latAccelFactor)."""
    def counting_update(calls, orig):
      def wrapped(*a, **kw):
        calls.append(1)
        return orig(*a, **kw)
      return wrapped

    for enhanced, expected_calls in [(True, 1), (False, 1)]:
      controller, VM, _ = _make_controller(enhanced=enhanced)
      calls: list[int] = []
      controller.pid.update = counting_update(calls, controller.pid.update)
      _run_update(controller, VM)
      assert len(calls) == expected_calls, f"enhanced={enhanced}: {len(calls)} pid updates"

  def test_jerk_aware_ff_subtracts_lat_accel_offset(self):
    """torqued fits lat_accel = latAccelFactor * torque + latAccelOffset; the torque-space
    feedforward must invert that fit like the stock controller's `ff -= latAccelOffset`."""
    controller, VM, _ = _make_controller(enhanced=True)
    _run_update(controller, VM)
    ff_no_offset = controller.extension._ff

    offset = 0.5
    controller.torque_params.latAccelOffset = offset
    _run_update(controller, VM)
    ff_with_offset = controller.extension._ff

    expected_delta = -offset / controller.torque_params.latAccelFactor
    assert abs((ff_with_offset - ff_no_offset) - expected_delta) < 1e-6
