"""Smoke: real Card startup + Controls() on a CX-5 2022 (EPS_HW) runs develop's Mazda torque stack."""
import math
import os
from types import SimpleNamespace
from unittest import mock

from openpilot.cereal import messaging
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from opendbc.car.car_helpers import interfaces
from opendbc.car.mazda.values import CAR, MazdaFlags
from opendbc.car.mazda.tests.conftest import car_params, eps_fw
from openpilot.starpilot.lateral.tests.test_lane_runtime import feed


def publish_mazda_params(params):
  from openpilot.selfdrive.car.card import Car
  params.put_bool('OpenpilotEnabledToggle', True, block=True)
  params.put_bool('SafeMode', False, block=True)
  cp = car_params(CAR.MAZDA_CX5_2022, car_fw=eps_fw(b'KSD5-3210X-C-00' + b'\x00' * 9))
  assert cp.flags & MazdaFlags.EPS_HW

  def discover(*args, pre_create_hook, **kwargs):
    return interfaces[cp.carFingerprint](pre_create_hook(cp.as_reader().as_builder(), cp.carFingerprint, {}, []))

  with mock.patch.dict(os.environ, {'SIMULATION': '1'}), \
       mock.patch('openpilot.selfdrive.car.card.messaging.recv_one_retry', return_value=SimpleNamespace(can=[1])), \
       mock.patch('openpilot.selfdrive.car.card.get_car', side_effect=discover):
    return Car().CP


def test_mazda_controls_run_develop_stack():
  from openpilot.selfdrive.controls.controlsd import Controls
  from openpilot.starpilot.car.mazda.lateral.latcontrol_torque_v1 import LatControlTorque as V1
  from openpilot.starpilot.car.mazda.lateral.latcontrol_torque_v2 import LatControlTorque as V2
  with OpenpilotPrefix(), mock.patch.dict(os.environ, {'AOL_REPLAY_RUNTIME': '0', 'REPLAY': '1'}), \
       mock.patch('openpilot.selfdrive.controls.controlsd.messaging.PubMaster'):
    params = Params()
    publish_mazda_params(params)
    # card's seed ran
    assert params.get('TorqueControlTune') == 2.0 and params.get_bool('EnforceTorqueControl') and params.get_bool('LiveTorqueParamsToggle')
    assert params.get('LateralJerkTorqueController', return_default=True) and params.get('LiveTorqueParamsRelaxedToggle', return_default=True)
    controls = Controls()
    assert controls.mazda_lateral is not None and controls.torque_host is None and controls.global_learning is None
    assert type(controls.LaC) is V2
    # the AOL axis acknowledgment gate has its own tests (fbda5a10f); this test is about steering
    controls.aol_replay = False
    controls.ordinary_axis_ack_required = False
    torques = []
    for tick in range(300):
      now = 1_000_000_000 + tick * 10_000_000
      feed(controls, now, tick, speed=20.)
      if tick % 25 == 0:
        ev = messaging.new_message('lateralTorqueParameters', valid=True, logMonoTime=now)
        tp = ev.lateralTorqueParameters
        tp.useParams = tp.valid = True
        tp.latAccelFactorFiltered, tp.latAccelOffsetFiltered, tp.frictionCoefficientFiltered = 2.0, 0.0, 0.1
        controls.sm.update_msgs(now / 1e9, [ev.as_reader()])
      CC, lac_log = controls.state_control()
      controls.mazda_lateral.note_lat_active(CC.latActive)
      controls.LaC, controls.steer_limited_by_safety = controls.mazda_lateral.run(controls.sm, controls.steer_limited_by_safety)
      torques.append(CC.actuators.torque)
    assert all(math.isfinite(t) for t in torques)
    assert any(abs(t) > 1e-4 for t in torques), torques[-5:]
    # big model frames while disengaged -> swap to v1 after 0.5 s inactive
    from openpilot.starpilot.lateral.tests import test_lane_runtime as lr
    orig = lr.model
    def big_model(*a, **k):
      m = orig(*a, **k)
      m.big = True
      return m
    with mock.patch.object(lr, 'model', big_model):
      for tick in range(300, 420):
        now = 1_000_000_000 + tick * 10_000_000
        feed(controls, now, tick, active=False, enabled=False)
        CC, _ = controls.state_control()
        controls.mazda_lateral.note_lat_active(CC.latActive)
        controls.LaC, controls.steer_limited_by_safety = controls.mazda_lateral.run(controls.sm, controls.steer_limited_by_safety)
    assert type(controls.LaC) is V1
