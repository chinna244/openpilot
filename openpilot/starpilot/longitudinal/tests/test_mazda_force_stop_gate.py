"""develop 197598c0f6: on a Mazda, Force Stop asks for the stopping state only below FIRM_STOP_PLAN_V (Mazda's STOPPING
barely brakes above it, develop route 59 t=227.8); every other car keeps StarPilot's immediate stopping state."""
import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.mazda.interface import CarInterface as MazdaInterface
from opendbc.car.mazda.values import CAR as MAZDA
from opendbc.car.toyota.interface import CarInterface as ToyotaInterface
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.starpilot.longitudinal.force_stop import StopPlan
from openpilot.starpilot.longitudinal.tests.test_highlander_cruise import frame, update


def planner_should_stop(cp, speed):
  owner = LongitudinalPlanner(cp, init_v=speed)
  sm, now = frame(0, speed=speed, setting=40.)
  update(owner, sm, now, force_stop_provider=lambda follow: StopPlan(model_ns=sm.logMonoTime['modelV2'], should_stop=True, forcing=True))
  return owner.output_should_stop


@pytest.mark.parametrize('speed, expected', [(0.64, False), (0.2, True)])
def test_mazda_force_stop_waits_for_the_firm_stop_speed(speed, expected):
  cp = MazdaInterface.get_params(MAZDA.MAZDA_CX5_2022, gen_empty_fingerprint(), [], True, False, False)
  assert planner_should_stop(cp, speed) is expected


def test_other_cars_stop_at_once():
  cp = ToyotaInterface.get_non_essential_params(TOYOTA.TOYOTA_HIGHLANDER_TSS2)
  assert planner_should_stop(cp, 0.64)
