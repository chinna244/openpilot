"""Mazda startup waits for a current native before selfdrived initializes (route 11 controlsMismatch)."""
from opendbc.car import gen_empty_fingerprint
from opendbc.car.mazda.interface import CarInterface as MazdaInterface
from opendbc.car.mazda.values import CAR as MAZDA
from opendbc.car.toyota.interface import CarInterface as ToyotaInterface
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.selfdrive.selfdrived.selfdrived import initialization_waits_for_native


def test_mazda_waits_for_native():
  for alpha in (False, True):
    cp = MazdaInterface.get_params(MAZDA.MAZDA_CX5_2022, gen_empty_fingerprint(), [], alpha, False, False)
    assert initialization_waits_for_native(cp.as_reader())


def test_other_brands_keep_their_gate():
  cp = ToyotaInterface.get_params(TOYOTA.TOYOTA_RAV4_TSS2, gen_empty_fingerprint(), [], False, False, False)
  assert not initialization_waits_for_native(cp.as_reader())
