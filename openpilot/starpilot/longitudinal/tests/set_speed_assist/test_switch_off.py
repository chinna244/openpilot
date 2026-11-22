"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Switched off with a lift on, the lift goes out at its own rate instead of stepping off the
accel request.
"""
import pytest

from openpilot.common.realtime import DT_MDL as DT
from openpilot.starpilot.longitudinal import set_speed_assist as c
from openpilot.starpilot.longitudinal.set_speed_assist import Inhibit
from openpilot.starpilot.longitudinal.tests.set_speed_assist.helpers import controller_class, build_sm
from openpilot.starpilot.longitudinal.tests.set_speed_assist.test_controller import V_EGO, run, settled

E2ESetSpeedController = controller_class()


def test_switch_off_lets_the_lift_out_at_lift_fall():
  ctl = settled()
  lift = ctl.lift
  ctl.params.values["ExperimentalModeSetSpeed"] = False
  ctl.enabled = False  # the params pass lands within PARAMS_UPDATE_PERIOD
  sm = build_sm(V_EGO)
  out = run(ctl, 1, sm)
  assert out == pytest.approx(lift - c.LIFT_FALL * DT)
  assert ctl.inhibit == Inhibit.disabled
  run(ctl, round(lift / (c.LIFT_FALL * DT)) + 2, sm)
  assert ctl.lift == 0.
  assert run(ctl, 1, sm, a_model=0.3) == 0.3  # then idles on the model


def test_switch_off_while_idle_is_immediate():
  ctl = settled()
  ctl.enabled = False
  ctl.lift = 0.
  assert run(ctl, 1, build_sm(V_EGO), a_model=0.2) == 0.2
  assert ctl.inhibit == Inhibit.disabled
