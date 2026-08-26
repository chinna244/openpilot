"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Behavior tests for the v2 torque tune, run against the real controllers with a toy
# steering geometry (curvature proportional to steering angle) and a mocked car interface
# (torque == lat_accel / latAccelFactor). The load-bearing property is the first test:
# with its mechanisms quiescent (constant speed, friction 0, predictive turn-in off), v2
# must be frame-for-frame identical to v0 — everything else is a deliberate, tested delta.

import math
from collections import deque
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from opendbc.car.structs import car
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v2 import (
  LatControlTorque as LatControlTorqueV2,
  get_center_chatter_jerk_deadzone,
)

DT = 0.01
LAT_DELAY = 0.3
DELAY_FRAMES = int(LAT_DELAY / DT)
LAF = 2.5
CURV_PER_DEG = 2e-4  # toy geometry: curvature = -steeringAngleDeg * CURV_PER_DEG

VM = SimpleNamespace(calc_curvature=lambda angle_rad, v_ego, roll: math.degrees(angle_rad) * CURV_PER_DEG)
LP = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)


def make_cp(friction=0.0):
  CP = car.CarParams.new_message(steerControlType="torque", steerLimitTimer=0.4)
  CP.lateralTuning.init('torque')
  CP.lateralTuning.torque.latAccelFactor = LAF
  CP.lateralTuning.torque.friction = friction
  return CP.as_reader()


def make_ci():
  CI = MagicMock()
  CI.torque_from_lateral_accel.return_value = lambda lataccel, tp: lataccel / tp.latAccelFactor
  CI.lateral_accel_from_torque.return_value = lambda torque, tp: torque * tp.latAccelFactor
  return CI


def make_lac(cls, friction=0.0):
  return cls(make_cp(friction=friction), custom.CarParamsSP.new_message().as_reader(), make_ci(), DT)


def make_pair(friction=0.0):
  return make_lac(LatControlTorqueV0, friction=friction), make_lac(LatControlTorqueV2, friction=friction)


def make_cs(v_ego=15.0, lat_accel=0.0, pressed=False):
  """CarState whose measured lateral accel equals lat_accel at v_ego."""
  angle = -lat_accel / (CURV_PER_DEG * v_ego ** 2)
  return SimpleNamespace(vEgo=v_ego, aEgo=0.0, steeringAngleDeg=angle, steeringRateDeg=0.0, steeringPressed=pressed)


def step(lac, cs, desired_curvature, active=True, lp=LP, lat_delay=LAT_DELAY):
  _, _, pid_log = lac.update(active, cs, VM, lp, False, desired_curvature, None, False, lat_delay)
  return pid_log


@pytest.fixture
def params():
  with OpenpilotPrefix():
    yield Params()


class TestLatControlTorqueV2:
  def test_constant_speed_matches_v0(self, params):
    """At constant speed with friction 0 and predictive turn-in off, the curvature-domain
    buffer is provably identical to v0's lat-accel buffer and the setpoint algebra collapses
    to v0's — outputs must match frame for frame."""
    v0, v2 = make_pair()
    v_ego = 15.0
    # slow sine: setpoint rate stays under the unwind threshold, so v2's integrator
    # policy is quiescent and the equivalence is exact
    for i in range(300):
      desired = 2e-3 * math.sin(i / 80)
      measured = 1.5e-3 * math.sin((i - 20) / 80)
      out0, _, log0 = v0.update(True, make_cs(v_ego, measured * v_ego ** 2), VM, LP, False, desired, None, False, LAT_DELAY)
      out2, _, log2 = v2.update(True, make_cs(v_ego, measured * v_ego ** 2), VM, LP, False, desired, None, False, LAT_DELAY)
      assert out2 == pytest.approx(out0, abs=1e-9), f"frame {i}"
    assert log0.version == 0
    assert log2.version == 2

  def test_no_phantom_jerk_when_decelerating_through_constant_curvature(self, params):
    """Mechanism 1: braking through a constant-curvature arc, the delayed request rescaled by
    the current v^2 equals the live request, so v2 sees zero desired jerk. v0's lat-accel
    buffer replays the higher old-speed values and reads a phantom jerk."""
    v0, v2 = make_pair()
    desired = 2e-3
    v_ego = 25.0
    for _ in range(150):  # settle in the curve at constant speed first
      step(v0, make_cs(v_ego, desired * v_ego ** 2), desired)
      step(v2, make_cs(v_ego, desired * v_ego ** 2), desired)
    v0_jerks, v2_jerks = [], []
    for _ in range(200):
      v_ego = max(v_ego - 0.04, 10.0)  # -4 m/s^2
      log0 = step(v0, make_cs(v_ego, desired * v_ego ** 2), desired)
      log2 = step(v2, make_cs(v_ego, desired * v_ego ** 2), desired)
      v0_jerks.append(abs(log0.desiredLateralJerk))
      v2_jerks.append(abs(log2.desiredLateralJerk))
    assert max(v2_jerks) < 1e-3
    assert max(v0_jerks) > 0.2

  def test_predictive_turn_in_flag(self, params):
    """Off (default): the setpoint is exactly the live request, v0's algebra. On: the setpoint
    is the delayed request plus one lat_delay of filtered jerk — different through the
    transient, converging back to the request in steady state."""
    plain = make_lac(LatControlTorqueV2)
    assert not plain.predictive_turn_in
    params.put_bool("TorqueTuneV2PredictiveTurnIn", True, block=True)
    predictive = make_lac(LatControlTorqueV2)
    assert predictive.predictive_turn_in

    v_ego = 15.0
    transient_diff = 0.0
    for i in range(400):
      desired = 0.0 if i < 100 else 2e-3  # step turn-in
      log_plain = step(plain, make_cs(v_ego), desired)
      log_pred = step(predictive, make_cs(v_ego), desired)
      request = desired * v_ego ** 2
      assert log_plain.desiredLateralAccel == pytest.approx(request, abs=1e-6)  # float32 log field
      if 100 <= i < 120:
        transient_diff = max(transient_diff, abs(log_pred.desiredLateralAccel - request))
    assert transient_diff > 0.05  # the lead reshapes the transient
    assert log_pred.desiredLateralAccel == pytest.approx(2e-3 * v_ego ** 2, abs=1e-3)  # steady state converges

  def test_extension_output_overrides_disabled(self, params):
    """v2 owns the friction shaping and integrator policy, so extension controllers that
    override the shared PID (jerk-aware, NNLC, any future sibling) are disabled no matter
    what the params say, and the PID limits stay in lat-accel space. v0 constructed with
    the same params keeps jerk-aware on (torque-space limits) — pinning that the disable
    is v2's, not a side effect."""
    params.put_bool("LateralJerkTorqueController", True, block=True)
    params.put_bool("NeuralNetworkLateralControl", True, block=True)
    v2 = make_lac(LatControlTorqueV2)
    assert not v2.extension.overrides_output
    assert v2.pid.pos_limit == pytest.approx(LAF)
    # update_limits must stay a no-op on the extension, or the per-frame speed-dep
    # override path would restore torque-space limits
    v2.update_limits()
    assert v2.pid.pos_limit == pytest.approx(LAF)

    v0 = make_lac(LatControlTorqueV0)
    assert v0.extension.overrides_output
    assert v0.pid.pos_limit == pytest.approx(v0.steer_max)

  def test_release_decay_is_one_shot(self, params):
    """Handing back the wheel decays the integrator once (x0.8), not every frame."""
    v2 = make_lac(LatControlTorqueV2)
    v2.pid.i = 1.0
    step(v2, make_cs(pressed=True), 0.0)  # frozen while pressed
    assert v2.pid.i == pytest.approx(1.0)
    step(v2, make_cs(pressed=False), 0.0)  # falling edge: one-shot decay, error is 0
    assert v2.pid.i == pytest.approx(0.8)
    step(v2, make_cs(pressed=False), 0.0)
    assert v2.pid.i == pytest.approx(0.8)

    v0 = make_lac(LatControlTorqueV0)
    v0.pid.i = 1.0
    step(v0, make_cs(pressed=True), 0.0)
    step(v0, make_cs(pressed=False), 0.0)
    assert v0.pid.i == pytest.approx(1.0)  # v0 has no release decay

  def test_unwind_freezes_integrator(self, params):
    """While the plan unwinds through center (setpoint rate < -1 m/s^3, |setpoint| < 0.3),
    the integrator holds instead of integrating the transient error."""
    v2 = make_lac(LatControlTorqueV2)
    v_ego = 15.0
    hold = 0.25 / v_ego ** 2  # steady setpoint 0.25 m/s^2, inside the near-zero band
    for _ in range(50):
      step(v2, make_cs(v_ego), hold)  # measurement 0 -> error 0.25, integrator winds up
    i_before = v2.pid.i
    assert i_before > 0.0
    step(v2, make_cs(v_ego), hold * 0.8)  # setpoint drops 0.05 in one frame: rate -5 m/s^3
    assert v2.pid.i == pytest.approx(i_before)  # frozen through the unwind
    for _ in range(5):
      step(v2, make_cs(v_ego), hold * 0.8)  # settled again: rate 0, integrating resumes
    assert v2.pid.i != pytest.approx(i_before)

  def test_integrator_active_at_creep_speeds(self, params):
    """v0 freezes the integrator below 5 m/s; v2 keys the freeze/reset to
    max(minSteerSpeed, 0.3) so it keeps working at creep speeds."""
    v0, v2 = make_pair()
    v_ego = 3.0
    desired = 0.05 / v_ego ** 2  # small error: the boosted low-speed P gain must not clip the output
    for _ in range(50):
      log0 = step(v0, make_cs(v_ego), desired)
      log2 = step(v2, make_cs(v_ego), desired)
    assert log0.i == pytest.approx(0.0)
    assert log2.i > 0.0

    step(v2, make_cs(0.2), desired)  # below the threshold: full reset
    assert v2.pid.i == pytest.approx(0.0)

  def test_inactive_priming_preserves_integrator_and_buffer(self, params):
    """While lateral is inactive the buffer and rate state track the live command (no
    re-engage shove) and the integrator is deliberately NOT cleared (MADS cycles lateral
    often; the release decay and unwind freeze replace a blunt reset). v0's stale buffer
    reads a large phantom jerk on the first active frame."""
    v0, v2 = make_pair()
    v2.pid.i = 0.5
    v_ego = 15.0
    desired = 2e-3
    for _ in range(2 * DELAY_FRAMES):
      step(v0, make_cs(v_ego, desired * v_ego ** 2), desired, active=False)
      step(v2, make_cs(v_ego, desired * v_ego ** 2), desired, active=False)
    assert v2.pid.i == pytest.approx(0.5)

    log0 = step(v0, make_cs(v_ego, desired * v_ego ** 2), desired, active=True)
    log2 = step(v2, make_cs(v_ego, desired * v_ego ** 2), desired, active=True)
    assert abs(log2.desiredLateralJerk) < 1e-6
    assert abs(log0.desiredLateralJerk) > 1.0

  def test_roll_compensation_fades_at_creep(self, params):
    """Below walking pace the road-crown feedforward fades out instead of unwinding a held
    wheel at pull-away. At 1.0 m/s the fade is 0.25 of v0's full compensation."""
    v0, v2 = make_pair()
    lp_roll = SimpleNamespace(angleOffsetDeg=0.0, roll=0.1)
    log0 = step(v0, make_cs(1.0), 0.0, lp=lp_roll)
    log2 = step(v2, make_cs(1.0), 0.0, lp=lp_roll)
    assert log0.f != 0.0
    assert log2.f == pytest.approx(0.25 * log0.f)

    log0 = step(v0, make_cs(15.0), 0.0, lp=lp_roll)
    log2 = step(v2, make_cs(15.0), 0.0, lp=lp_roll)
    assert log2.f == pytest.approx(log0.f)  # fully faded in above 2.5 m/s

  def test_center_chatter_deadzone_curve(self):
    """Full-strength at lane center, gone above 0.35 m/s^2 of demand."""
    assert get_center_chatter_jerk_deadzone(25.0, 0.0) == pytest.approx(0.18)
    assert get_center_chatter_jerk_deadzone(25.0, 0.5) == pytest.approx(0.0)
    assert get_center_chatter_jerk_deadzone(0.0, 0.0) == pytest.approx(0.08)

  def test_center_wobble_reduces_friction_activity(self, params):
    """With the wheel tracking the delayed command, planner wobble that flips v0's
    friction term stays inside the deadzone for v2."""
    v0, v2 = make_pair(friction=0.25)
    v_ego = 25.0
    history = deque([0.0] * (DELAY_FRAMES + 1), maxlen=DELAY_FRAMES + 1)
    v0_friction, v2_friction = [], []
    for i in range(600):
      # planner wobble around center: +-0.04 m/s^2 flipping every 0.4 s
      desired = math.copysign(0.04, math.sin(2 * math.pi * i / 80)) / v_ego ** 2
      measured_lat_accel = history[-DELAY_FRAMES] * v_ego ** 2  # wheel tracks the delayed command
      history.append(desired)
      log0 = step(v0, make_cs(v_ego, measured_lat_accel), desired)
      log2 = step(v2, make_cs(v_ego, measured_lat_accel), desired)
      if i > 200:
        request = desired * v_ego ** 2
        v0_friction.append(abs(log0.f - request))  # roll and offset are 0: f - request is the friction term
        v2_friction.append(abs(log2.f - request))
    assert sum(v2_friction) < 0.5 * sum(v0_friction)
