# Fail-closed get_lat_active for Mazda TJA Panda lateral auth.

import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock

import pytest

if "msgq" not in sys.modules:
  _msgq = ModuleType("msgq")
  _msgq.MultiplePublishersError = type("MultiplePublishersError", (Exception,), {})
  _msgq.IpcError = type("IpcError", (Exception,), {})
  for _name in (
    "fake_event_handle", "drain_sock_raw", "Context", "Poller", "SubSocket", "PubSocket",
    "SocketEventHandle", "toggle_fake_events", "set_fake_prefix", "get_fake_prefix",
    "delete_fake_prefix", "wait_for_one_event",
  ):
    setattr(_msgq, _name, MagicMock())
  sys.modules["msgq"] = _msgq

if "openpilot.common.params_pyx" not in sys.modules:
  _params_pyx = ModuleType("openpilot.common.params_pyx")
  class Params:
    def get_bool(self, key):
      return False
    def get(self, key, **kwargs):
      return None
    def put(self, *args, **kwargs):
      return None
  _params_pyx.Params = Params
  _params_pyx.ParamKeyFlag = type("ParamKeyFlag", (), {})
  _params_pyx.ParamKeyType = type("ParamKeyType", (), {})
  _params_pyx.UnknownKeyName = type("UnknownKeyName", (Exception,), {})
  sys.modules["openpilot.common.params_pyx"] = _params_pyx

from opendbc.car import structs
from opendbc.car.mazda.values import MazdaSafetyFlags
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt

SafetyModel = structs.CarParams.SafetyModel


class FakeSM(dict):
  def __init__(self, panda_states, seen=True, valid=True, alive=True, mads_active=True, mads_available=True):
    super().__init__()
    self["pandaStates"] = panda_states
    self["carState"] = SimpleNamespace(leftBlinker=False, rightBlinker=False)
    self["selfdriveStateSP"] = SimpleNamespace(
      mads=SimpleNamespace(available=mads_available, active=mads_active, enabled=mads_active, state=0)
    )
    self["selfdriveState"] = SimpleNamespace(active=False)
    self.seen = {"pandaStates": seen}
    self.valid = {"pandaStates": valid}
    self.alive = {"pandaStates": alive}


def _ext(tja=True):
  CP = structs.CarParams()
  CP.brand = "mazda"
  cfg = structs.CarParams.SafetyConfig()
  cfg.safetyParam = int(MazdaSafetyFlags.TJA) if tja else 0
  CP.safetyConfigs = [cfg]
  ext = ControlsExt.__new__(ControlsExt)
  ext.CP = CP
  ext.blinker_pause_lateral = MagicMock()
  ext.blinker_pause_lateral.update.return_value = False
  return ext


def _ps(lat=True, model=SafetyModel.mazda):
  return SimpleNamespace(controlsAllowedLateral=lat, safetyModel=model)


def test_lat_active_requires_panda_true():
  ext = _ext()
  assert ext.get_lat_active(FakeSM([_ps(True)])) is True
  assert ext.get_lat_active(FakeSM([_ps(False)])) is False


@pytest.mark.parametrize("kwargs", [
  {"panda_states": []},
  {"panda_states": [_ps(True)], "seen": False},
  {"panda_states": [_ps(True)], "valid": False},
  {"panda_states": [_ps(True)], "alive": False},
  {"panda_states": [_ps(True, SafetyModel.silent)]},
])
def test_lat_active_fail_closed(kwargs):
  ext = _ext()
  panda_states = kwargs.pop("panda_states")
  sm = FakeSM(panda_states, **kwargs)
  assert ext.get_lat_active(sm) is False


def test_mads_inactive_is_not_lat_active():
  ext = _ext()
  sm = FakeSM([_ps(True)], mads_active=False)
  assert ext.get_lat_active(sm) is False


def test_non_tja_does_not_require_panda_lat():
  ext = _ext(tja=False)
  sm = FakeSM([], mads_active=True)
  assert ext.get_lat_active(sm) is True


def test_controlsd_subscribes_panda_states():
  import inspect
  from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
  assert "pandaStates" in inspect.getsource(ControlsExt.__init__)
