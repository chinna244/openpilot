"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from pathlib import Path

import pytest

from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.system.params_migration_zp import E2E_ASSISTS, E2E_ASSISTS_MIGRATED, OLD_LANE_CENTER_READOUT, \
  run_migration_zp


class TestE2EAssistsMigration(OpenpilotTestCase):
  """Existing installs hold the old declared default (off) on disk; the migration turns the
  assists on once, then leaves the driver's choice alone."""

  def test_turns_on_the_materialized_old_default(self):
    params = Params()
    for key in E2E_ASSISTS:
      params.put_bool(key, False, block=True)  # what manager_init wrote under the old default
    run_migration_zp(params)
    assert all(params.get_bool(key) for key in E2E_ASSISTS)
    assert params.get_bool(E2E_ASSISTS_MIGRATED)

  def test_runs_once(self):
    params = Params()
    run_migration_zp(params)
    params.put_bool("ExperimentalModeSetSpeed", False, block=True)  # the driver turns it off
    run_migration_zp(params)
    assert not params.get_bool("ExperimentalModeSetSpeed")


class TestLaneCenterDefault(OpenpilotTestCase):
  """Lane Center Assist is off on a new install (upstream's Blendful default), and the migration
  leaves an existing install's choice alone."""

  def test_stays_as_stored(self):
    params = Params()
    for on in (False, True):
      params.put_bool("Blendful", on, block=True)
      run_migration_zp(params)
      assert params.get_bool("Blendful") is on


class TestLaneCenterUIMigration(OpenpilotTestCase):
  """The old int's debug (2) becomes on; BlendfulReadout, no longer a declared key, is only removed."""

  @staticmethod
  def _old_readout(params: Params) -> Path:
    return Path(params.get_param_path(OLD_LANE_CENTER_READOUT))

  def test_old_readout_is_removed(self):
    params = Params()
    params.put_bool("LaneCenterUI", False, block=True)
    self._old_readout(params).write_bytes(b"1")
    run_migration_zp(params)
    assert not params.get_bool("LaneCenterUI")
    assert not self._old_readout(params).exists()

  def test_old_debug_gets_the_gauge(self):
    params = Params()
    Path(params.get_param_path("LaneCenterUI")).write_bytes(b"2")
    run_migration_zp(params)
    assert params.get_bool("LaneCenterUI")

  def test_no_old_value_leaves_the_setting(self):
    params = Params()
    params.put_bool("LaneCenterUI", False, block=True)  # the driver turned it off
    run_migration_zp(params)
    assert not params.get_bool("LaneCenterUI")

  def test_old_key_is_gone(self):
    with pytest.raises(UnknownKeyName):
      Params().get(OLD_LANE_CENTER_READOUT)
