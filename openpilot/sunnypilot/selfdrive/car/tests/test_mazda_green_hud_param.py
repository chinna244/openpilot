"""End-to-end plumbing: initialize_params -> OpenDBC setup_interfaces -> Mazda GREEN flag."""

from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from opendbc.sunnypilot.car.interfaces import setup_interfaces as opendbc_setup_interfaces
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP
from openpilot.sunnypilot.selfdrive.car.interfaces import initialize_params

GREEN_KEY = "MazdaExperimentalMadsGreenHud"


class FakeParams:
  """Mirrors Params.get for BOOL keys: return_default yields a Python bool."""

  def __init__(self, initial=None):
    self._store = dict(initial or {})

  def get(self, key, return_default=False):
    if key in self._store:
      return self._store[key]
    if return_default:
      return False
    return None


def _mazda_cp_sp():
  fingerprint = {0: {}, 1: {}, 2: {}}
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fingerprint, [], alpha_long=False,
                               is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, fingerprint, [],
                                     False, False, False)
  return CP, CP_SP


def _plumb(params):
  param_list = initialize_params(params)
  CP, CP_SP = _mazda_cp_sp()
  # Same call get_car() makes: class + CP/CP_SP + initialize_params() list.
  opendbc_setup_interfaces(CarInterface, CP, CP_SP, param_list)
  return param_list, CP_SP


class TestMazdaGreenHudParamPlumbing:
  def test_initialize_params_includes_mazda_key(self):
    param_list = initialize_params(FakeParams())
    keys = {k for d in param_list for k in d}
    assert GREEN_KEY in keys
    assert any(d.get(GREEN_KEY) is False for d in param_list)

  def test_param_off_does_not_set_opendbc_flag(self):
    param_list, CP_SP = _plumb(FakeParams({GREEN_KEY: False}))
    assert any(d.get(GREEN_KEY) is False for d in param_list)
    assert not (CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_GREEN_HUD)

  def test_param_on_sets_opendbc_flag(self):
    param_list, CP_SP = _plumb(FakeParams({GREEN_KEY: True}))
    assert any(d.get(GREEN_KEY) is True for d in param_list)
    assert CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_GREEN_HUD

  def test_unset_param_uses_default_off(self):
    _, CP_SP = _plumb(FakeParams())
    assert not (CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_GREEN_HUD)
