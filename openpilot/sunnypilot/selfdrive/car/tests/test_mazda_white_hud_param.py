"""End-to-end plumbing: initialize_params -> OpenDBC setup_interfaces -> Mazda WHITE flag."""

from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from opendbc.sunnypilot.car.interfaces import setup_interfaces as opendbc_setup_interfaces
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP
from openpilot.sunnypilot.selfdrive.car.interfaces import initialize_params

WHITE_KEY = "MazdaExperimentalMadsWhiteHud"
ACTIVE_KEY = "MazdaExperimentalMadsWhiteHudActive"


class FakeParams:
  def __init__(self, initial=None, default=True):
    self._store = dict(initial or {})
    self._default = default

  def get(self, key, return_default=False):
    if key in self._store:
      return self._store[key]
    return self._default if return_default else None


def _plumb(params, candidate=CAR.MAZDA_CX5_2022):
  fingerprint = {0: {}, 1: {}, 2: {}}
  param_list = initialize_params(params)
  CP = CarInterface.get_params(candidate, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, [], False, False, False)
  opendbc_setup_interfaces(CarInterface, CP, CP_SP, param_list)
  return param_list, CP_SP


def test_initialize_params_includes_default_on_mazda_keys():
  param_list, CP_SP = _plumb(FakeParams())
  assert any(param.get(WHITE_KEY) is True for param in param_list)
  assert any(param.get(ACTIVE_KEY) is True for param in param_list)
  assert CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_WHITE_HUD
  assert CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_WHITE_HUD_ACTIVE


def test_param_off_does_not_set_opendbc_flag():
  _, CP_SP = _plumb(FakeParams({WHITE_KEY: False, ACTIVE_KEY: False}))
  assert not (CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_WHITE_HUD)
  assert not (CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_WHITE_HUD_ACTIVE)


def test_param_on_sets_opendbc_flag():
  _, CP_SP = _plumb(FakeParams({WHITE_KEY: True, ACTIVE_KEY: False}))
  assert CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_WHITE_HUD
  assert not (CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_WHITE_HUD_ACTIVE)


def test_active_param_on_sets_flag_without_touching_off_flag():
  _, CP_SP = _plumb(FakeParams({ACTIVE_KEY: True, WHITE_KEY: False}))
  assert CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_WHITE_HUD_ACTIVE
  assert not (CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_WHITE_HUD)


def test_active_param_cannot_set_flag_on_non_tja_mazda():
  _, CP_SP = _plumb(FakeParams({ACTIVE_KEY: True}), CAR.MAZDA_CX9_2021)
  assert not (CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_WHITE_HUD_ACTIVE)


def test_param_cannot_set_flag_on_non_tja_mazda():
  _, CP_SP = _plumb(FakeParams({WHITE_KEY: True}), CAR.MAZDA_CX9_2021)
  assert not (CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_WHITE_HUD)
