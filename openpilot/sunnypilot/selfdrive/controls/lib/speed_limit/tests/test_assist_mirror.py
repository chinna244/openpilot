"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Plannerd-side mirror of the card cruise arbiter's session: the default-message guard and
the required-decel publication.
"""
import pytest

from openpilot.cereal import custom
from opendbc.car.structs import car as car_struct
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.assist_mirror import CAP_FALL_RATE, SpeedLimitAssistMirror
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState


def _update(mirror, session, v_ego: float, distance: float, a_ego: float) -> None:
  mirror.session = session
  mirror.update(True, False, v_ego, a_ego, 0., 0., 0., False, distance, EventsSP())


class TestAssistMirrorDefaultMessage:
  """plannerd ignores carStateSP in its health checks, so the mirror can be fed capnp's
  default CruiseSession (vCap=0.0) before the first card message arrives or in process
  replay without a carStateSP pub. That 0 must not become the plan target."""

  def _mirror(self, op_long=False):
    CP = car_struct.CarParams(brand="mazda", pcmCruise=not op_long, openpilotLongitudinalControl=op_long)
    return SpeedLimitAssistMirror(CP, custom.CarParamsSP(pcmCruiseSpeed=op_long))

  def test_default_session_yields_unset(self):
    session = custom.CarStateSP.new_message().zoompilot.cruiseSession
    mirror = self._mirror()
    _update(mirror, session, v_ego=25.0, distance=0.0, a_ego=0.0)
    assert mirror.output_v_target == V_CRUISE_UNSET

  def test_real_cap_passes_through(self):
    session = custom.CarStateSP.new_message().zoompilot.cruiseSession
    session.vCap = 22.5
    mirror = self._mirror()
    _update(mirror, session, v_ego=20.0, distance=0.0, a_ego=0.0)
    assert mirror.output_v_target == pytest.approx(22.5)

  def test_active_cap_below_v_ego_publishes_required_decel(self):
    session = custom.CarStateSP.new_message().zoompilot.cruiseSession
    session.state = SpeedLimitAssistState.adapting
    session.vCap = 20.0
    mirror = self._mirror()
    # 25 -> 20 m/s over 150 m needs (400 - 625) / 300 = -0.75; the publication ramp
    # walks there at 2 m/s3, so run it to convergence
    for _ in range(20):
      _update(mirror, session, v_ego=25.0, distance=150.0, a_ego=0.0)
    assert mirror.output_a_target == pytest.approx(-0.75)

  def test_cap_decel_is_clipped_to_the_budget(self):
    session = custom.CarStateSP.new_message().zoompilot.cruiseSession
    session.state = SpeedLimitAssistState.adapting
    session.vCap = 15.0
    # 25 -> 15 m/s over 100 m asks -2.0; on openpilot long the wire is an actuator command and
    # on stock ACC it sizes the servo's gap, and a limit is a prediction on either: budget at most
    stock, op_long = self._mirror(), self._mirror(op_long=True)
    for _ in range(60):
      _update(stock, session, v_ego=25.0, distance=100.0, a_ego=0.0)
      _update(op_long, session, v_ego=25.0, distance=100.0, a_ego=0.0)
    assert stock.output_a_target == pytest.approx(-stock.limits.a_budget)
    assert op_long.output_a_target == pytest.approx(-op_long.limits.a_budget)
    assert op_long.limits.a_budget < 2.0

  def test_inactive_session_tracks_a_ego(self):
    session = custom.CarStateSP.new_message().zoompilot.cruiseSession
    session.vCap = 20.0  # cap present but session not active
    mirror = self._mirror()
    _update(mirror, session, v_ego=25.0, distance=150.0, a_ego=-0.2)
    assert mirror.output_a_target == pytest.approx(-0.2)


class TestCapRamp:
  """A lower cap walks down at CAP_FALL_RATE from the car's speed; it never delays a cap the car is already under."""

  def _mirror(self):
    CP = car_struct.CarParams(brand="mazda", pcmCruise=False, openpilotLongitudinalControl=True)
    return SpeedLimitAssistMirror(CP, custom.CarParamsSP(pcmCruiseSpeed=True))

  def _session(self, v_cap):
    session = custom.CarStateSP.new_message().zoompilot.cruiseSession
    session.vCap = v_cap
    return session

  def test_step_down_becomes_a_ramp(self):
    mirror = self._mirror()
    _update(mirror, self._session(28.0), v_ego=28.0, distance=0., a_ego=0.)
    _update(mirror, self._session(26.0), v_ego=28.0, distance=0., a_ego=0.)
    assert mirror.output_v_target == pytest.approx(28.0 - CAP_FALL_RATE * DT_MDL)
    for _ in range(int(2.0 / 0.05)):
      _update(mirror, self._session(26.0), v_ego=28.0, distance=0., a_ego=0.)
    assert mirror.output_v_target > 26.0  # 2 m/s at 0.6 m/s^2 takes 3.3 s; not there after 2.0 s
    for _ in range(int(2.0 / 0.05)):
      _update(mirror, self._session(26.0), v_ego=28.0, distance=0., a_ego=0.)
    assert mirror.output_v_target == pytest.approx(26.0)

  def test_ramp_starts_from_the_cars_speed_not_the_old_cap(self):
    mirror = self._mirror()
    _update(mirror, self._session(36.0), v_ego=27.0, distance=0., a_ego=0.)
    _update(mirror, self._session(25.0), v_ego=27.0, distance=0., a_ego=0.)
    assert mirror.output_v_target == pytest.approx(27.0 - CAP_FALL_RATE * DT_MDL)

  def test_rise_and_unset_pass_through(self):
    mirror = self._mirror()
    _update(mirror, self._session(25.0), v_ego=20.0, distance=0., a_ego=0.)
    _update(mirror, self._session(30.0), v_ego=20.0, distance=0., a_ego=0.)
    assert mirror.output_v_target == pytest.approx(30.0)
    _update(mirror, self._session(0.0), v_ego=20.0, distance=0., a_ego=0.)
    assert mirror.output_v_target == V_CRUISE_UNSET
    _update(mirror, self._session(18.0), v_ego=20.0, distance=0., a_ego=0.)
    assert mirror.output_v_target == pytest.approx(18.0)  # unset forgot the ramp

  def test_a_cap_above_the_cars_speed_is_not_delayed(self):
    mirror = self._mirror()
    _update(mirror, self._session(30.0), v_ego=20.0, distance=0., a_ego=0.)
    _update(mirror, self._session(25.0), v_ego=20.0, distance=0., a_ego=0.)
    assert mirror.output_v_target == pytest.approx(25.0)
