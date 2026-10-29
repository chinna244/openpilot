"""Develop's MADS with the declared TJA button: only TJA turns steering on or off. SET/RES engagement, cancel
and main never do, and the dash white wheel follows the TJA latch (stays on through a stop)."""
from types import SimpleNamespace

import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.car.mazda.aol import create_intent, policy_for
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences

ButtonType = structs.CarState.ButtonEvent.Type
SETTINGS = AolSettings(True, 0., 0, 0, (0, 0, 0), (0, 0, 0))


def tja_cp(alpha=True, tja=True):
  cp = VehicleStartupPreferences(mazda_tja=tja).prepare(
    CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], alpha, False, False))
  policy = policy_for(cp)
  cp.safetyConfigs[-1].safetyParam |= policy.safety_param_addition
  cp.alternativeExperience |= policy.alternative_experience_addition
  return cp


def driving():
  cs = structs.CarState(canValid=True, vEgo=15., gearShifter=structs.CarState.GearShifter.drive)
  cs.cruiseState.available = True
  return cs


def press(owner, cs, button, t):
  cs.buttonEvents = [structs.CarState.ButtonEvent(type=button, pressed=True)]
  owner.update(cs, now_ns=t, fault_active=False)
  cs.buttonEvents = [structs.CarState.ButtonEvent(type=button, pressed=False)]
  owner.update(cs, now_ns=t + 10_000_000, fault_active=False)
  cs.buttonEvents = []


def engage(owner, cs, t):
  owner.observe_full_engagement(cs, enabled=True, active=True, source_ns=t, observation_valid=True, now_ns=t,
                                fault_active=False)


@pytest.mark.parametrize('alpha', (False, True))
def test_cruise_engagement_never_turns_steering_on(alpha):
  cp = tja_cp(alpha)
  owner = create_intent(cp, SETTINGS)
  assert not owner.observe_stock_engagement
  cs = driving()
  owner.update(cs, now_ns=1_000_000_000, fault_active=False)
  engage(owner, cs, 1_000_000_000)  # first observation: a process start, not a gesture
  owner.observe_full_engagement(cs, enabled=False, active=False, source_ns=1_010_000_000, observation_valid=True,
                                now_ns=1_010_000_000, fault_active=False)
  for i, button in enumerate((ButtonType.setCruise, ButtonType.resumeCruise)):
    press(owner, cs, button, 1_100_000_000 + i * 100_000_000)
  engage(owner, cs, 1_500_000_000)
  owner.update(cs, now_ns=1_510_000_000, fault_active=False)
  assert not owner.allowed_latch
  # ordinary engaged lateral is paused: decide_axes needs the latch for desired lateral
  assert owner.pause_lateral


def test_tja_toggles_and_cancel_main_keep_it():
  owner = create_intent(tja_cp(), SETTINGS)
  cs = driving()
  owner.update(cs, now_ns=1_000_000_000, fault_active=False)
  press(owner, cs, ButtonType.lkas, 1_100_000_000)
  assert owner.allowed_latch and not owner.pause_lateral
  for i, button in enumerate((ButtonType.cancel, ButtonType.mainCruise, ButtonType.setCruise, ButtonType.resumeCruise)):
    press(owner, cs, button, 1_200_000_000 + i * 100_000_000)
    assert owner.allowed_latch and not owner.pause_lateral, button
  press(owner, cs, ButtonType.lkas, 2_000_000_000)
  assert not owner.allowed_latch and owner.pause_lateral
  press(owner, cs, ButtonType.lkas, 2_100_000_000)
  assert owner.allowed_latch and not owner.pause_lateral


def test_without_the_tja_button_cruise_engagement_still_arms():
  # The undeclared button keeps StarPilot's stock-engagement owner (main / cruise arm AOL).
  owner = create_intent(tja_cp(alpha=False, tja=False), SETTINGS)
  assert owner.observe_stock_engagement and not owner.tja_button


class _Controller:
  pass


@pytest.mark.parametrize('latched,lat_active,standstill,enabled,active', (
  (None, True, False, True, True), (None, False, True, False, False),  # no card value: latActive as before
  (True, False, True, True, True),    # latched at a stop: wheel stays white
  (True, True, False, True, True),
  (True, False, False, True, False),  # latched but paused moving
  (False, True, False, False, False),
))
def test_white_wheel_follows_the_latch(latched, lat_active, standstill, enabled, active):
  from opendbc.car.mazda.carcontroller import CarController
  cp = tja_cp()
  cc = CarController.__new__(CarController)
  cc.CP = cp
  cc.control_context = SimpleNamespace(mads=SimpleNamespace(available=False, enabled=False, active=False))
  if latched is not None:
    cc.aol_lateral_latched = latched
  seen = {}
  cc.update_with_context = lambda CC, ctx, CS, now: seen.update(enabled=ctx.mads.enabled, active=ctx.mads.active)
  CS = SimpleNamespace(out=SimpleNamespace(steerFaultPermanent=False, standstill=standstill))
  CarController.update(cc, SimpleNamespace(latActive=lat_active), CS, 0)
  assert seen == {'enabled': enabled, 'active': active}
