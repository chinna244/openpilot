"""Fixtures for the Mazda lateral port's tests (zoompilot develop's controls, controls/lib and
locationd conftests, merged)."""
from types import SimpleNamespace

import pytest

from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.locationd import torqued
from openpilot.starpilot.car.mazda.lateral import lateral_tune, torqued_ext
from openpilot.starpilot.car.mazda.lateral import ext_override as override_module
from openpilot.starpilot.car.mazda.lateral.ext_override import LatControlTorqueExtOverride
from openpilot.starpilot.car.mazda.lateral.ext_zp import LatControlTorqueExtZP
from openpilot.starpilot.car.mazda.lateral.tests.fakes import FakeParams
from openpilot.starpilot.car.mazda.lateral.tests.speed_dep_helpers import make_cp


@pytest.fixture
def params():
  """Params under a throwaway prefix, for tests that build a real controller."""
  with OpenpilotPrefix():
    yield Params()


class TorqueParamsOverride(LatControlTorqueExtZP, LatControlTorqueExtOverride):
  """The torque-params half of LatControlTorqueExt: upstream's manual override under the fork's
  layer, without jerk-aware, which these tests do not reach. The host only carries steer_max."""

  def __init__(self, CP):
    LatControlTorqueExtOverride.__init__(self, CP)
    LatControlTorqueExtZP.__init__(self, CP)
    self.lac_torque = SimpleNamespace(steer_max=1.0)


@pytest.fixture
def make_override(monkeypatch):
  """Factory for a TorqueParamsOverride whose Params is a FakeParams; the fake is reachable
  afterwards as ovr.params."""
  def _make(enforce=False, manual_override=False, manual_lat_accel_factor='200', manual_friction='15', CP=None):
    fake = FakeParams(EnforceTorqueControl=enforce, TorqueParamsOverrideEnabled=manual_override,
                      TorqueParamsOverrideLatAccelFactor=manual_lat_accel_factor, TorqueParamsOverrideFriction=manual_friction)
    monkeypatch.setattr(override_module, "Params", lambda: fake)
    return TorqueParamsOverride(CP or make_cp(brand='test'))  # develop's default: no tune scale
  return _make


@pytest.fixture
def set_speed_dep_config(monkeypatch):
  """Replaces the speed_dependent.toml contents get_speed_dep_config_for_car reads."""
  def _set(cfg):
    monkeypatch.setattr(lateral_tune, "get_speed_dep_config", lambda: cfg)
  return _set


def _route_params(monkeypatch, fake):
  # both Params sites: torqued reads the caches, torqued_ext reads the toggles
  monkeypatch.setattr(torqued, "Params", lambda: fake)
  monkeypatch.setattr(torqued_ext, "Params", lambda: fake)
  return fake


@pytest.fixture
def fake_params(monkeypatch):
  """One FakeParams behind both Params sites, Enforce Torque Control and Self-Tune on, caches empty."""
  return _route_params(monkeypatch, FakeParams(EnforceTorqueControl=True, LiveTorqueParamsToggle=True))


@pytest.fixture
def fake_params_off(monkeypatch):
  """Same, with Enforce Torque Control and Self-Tune off."""
  return _route_params(monkeypatch, FakeParams())
