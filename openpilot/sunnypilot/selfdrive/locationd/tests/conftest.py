"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

import opendbc.sunnypilot.car.lateral_tune as lateral_tune
from openpilot.selfdrive.locationd import torqued
from openpilot.sunnypilot.selfdrive.locationd import torqued_ext
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams


def _route_params(monkeypatch, fake):
  # both Params sites: torqued reads the caches, torqued_ext reads the toggles
  monkeypatch.setattr(torqued, "Params", lambda: fake)
  monkeypatch.setattr(torqued_ext, "Params", lambda: fake)
  return fake


@pytest.fixture
def fake_params(monkeypatch):
  """One FakeParams behind both Params sites, Enforce Torque Control and Self-Tune on, caches
  empty. Speed-dep learning runs wherever self-tune does, and make_cp's brand is not one
  upstream self-tunes, so both toggles are needed."""
  return _route_params(monkeypatch, FakeParams(EnforceTorqueControl=True, LiveTorqueParamsToggle=True))


@pytest.fixture
def fake_params_off(monkeypatch):
  """Same, with Enforce Torque Control and Self-Tune off."""
  return _route_params(monkeypatch, FakeParams())


REAL_SPEED_DEP_CONFIG_FOR_CAR = lateral_tune.get_speed_dep_config_for_car


@pytest.fixture(autouse=True)
def every_bin_learns(monkeypatch):
  """The learning and cache tests run on a configured car's real table and exercise every bin,
  so they see it without learn_min_speed. test_torqued_learn_min_speed.py takes the entry as is
  through real_speed_dep_config."""
  def without_learn_min_speed(CP):
    cfg = REAL_SPEED_DEP_CONFIG_FOR_CAR(CP)
    cfg.pop('learn_min_speed', None)
    return cfg
  monkeypatch.setattr(lateral_tune, "get_speed_dep_config_for_car", without_learn_min_speed)


@pytest.fixture
def real_speed_dep_config(monkeypatch, every_bin_learns):
  """The entry as configured, learn_min_speed included."""
  monkeypatch.setattr(lateral_tune, "get_speed_dep_config_for_car", REAL_SPEED_DEP_CONFIG_FOR_CAR)
