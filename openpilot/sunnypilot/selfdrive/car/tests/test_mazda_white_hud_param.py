"""WHITE HUD is always enabled on Mazda TJA_MADS platforms (no Params toggle)."""

from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP
from openpilot.sunnypilot.selfdrive.car.interfaces import initialize_params, setup_interfaces


class FakeParams:
  def __init__(self, initial=None, default=True):
    self._store = dict(initial or {})
    self._default = default

  def get(self, key, return_default=False):
    if key in self._store:
      return self._store[key]
    return self._default if return_default else None

  def get_bool(self, key):
    v = self.get(key)
    return bool(v) if v is not None else False

  def put(self, *args, **kwargs):
    pass

  def put_bool(self, *args, **kwargs):
    pass

  def remove(self, *args, **kwargs):
    pass


class StrictParams(FakeParams):
  """Mimics Params.check_key: unknown keys raise like production UnknownKeyName."""

  KNOWN = {
    "HyundaiLongitudinalTuning",
    "SubaruStopAndGo",
    "SubaruStopAndGoManualParkingBrake",
    "TeslaCoopSteering",
    "TeslaMadsScreenButton",
    "ToyotaEnforceStockLongitudinal",
    "ToyotaStopAndGoHack",
    "TorqueControlTune",
    "MazdaTorqueDefaultsApplied",
    "EnforceTorqueControl",
    "LiveTorqueParamsToggle",
    "SpeedDependentTorqueToggle",
    "NeuralNetworkLateralControl",
    "IntelligentCruiseButtonManagement",
    "LateralJerkTorqueController",
    "DynamicExperimentalControl",
    "CustomAccIncrementsEnabled",
    "SmartCruiseControlVision",
    "SmartCruiseControlMap",
  }

  def get(self, key, return_default=False):
    if key not in self.KNOWN and key not in self._store:
      raise KeyError(key)
    return super().get(key, return_default=return_default)


def _make_ci(candidate=CAR.MAZDA_CX5_2022):
  fingerprint = {0: {}, 1: {}, 2: {}}
  CP = CarInterface.get_params(candidate, fingerprint, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, [], False, False, False)
  return CarInterface(CP, CP_SP), CP_SP


def test_initialize_params_has_no_white_hud_key():
  param_list = initialize_params(StrictParams())
  assert all("MazdaExperimentalMadsWhiteHud" not in p for p in param_list)


def test_white_hud_always_on_for_tja_mads_mazda():
  CI, CP_SP = _make_ci(CAR.MAZDA_CX5_2022)
  setup_interfaces(CI, FakeParams())
  assert CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_WHITE_HUD


def test_white_hud_not_enabled_for_non_tja_mazda():
  CI, CP_SP = _make_ci(CAR.MAZDA_CX9_2021)
  setup_interfaces(CI, FakeParams())
  assert not (CP_SP.flags & MazdaFlagsSP.EXPERIMENTAL_MADS_WHITE_HUD)
