"""Filter-dev's independent MADS and cruise with the Smart CAN Filter and the TJA button both on (safety word 27):
TJA alone owns steering, SET/CANCEL own only cruise, and a filter dropout is no ACC fault that would drop the latch."""
import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from openpilot.starpilot.aol.intent import AolSettings
from openpilot.starpilot.car.mazda.aol import create_intent, policy_for
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences

ButtonType = structs.CarState.ButtonEvent.Type
SETTINGS = AolSettings(True, 0., 0, 0, (0, 0, 0), (0, 0, 0))
T0 = 1_000_000_000


def filter_tja_cp():
  cp = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], True, False, False)
  cp.fingerprintSource = structs.CarParams.FingerprintSource.fw
  cp = VehicleStartupPreferences(mazda_can_filter=True, mazda_tja=True).prepare(cp)
  policy = policy_for(cp)
  cp.safetyConfigs[-1].safetyParam |= policy.safety_param_addition
  cp.alternativeExperience |= policy.alternative_experience_addition
  return cp


def driving(standstill=False):
  cs = structs.CarState(canValid=True, vEgo=0. if standstill else 15., standstill=standstill,
                        gearShifter=structs.CarState.GearShifter.drive)
  cs.cruiseState.available = True
  return cs


def press(owner, cs, button, t):
  cs.buttonEvents = [structs.CarState.ButtonEvent(type=button, pressed=True)]
  owner.update(cs, now_ns=t, fault_active=False)
  cs.buttonEvents = [structs.CarState.ButtonEvent(type=button, pressed=False)]
  owner.update(cs, now_ns=t + 10_000_000, fault_active=False)
  cs.buttonEvents = []


def engagement(owner, cs, enabled, t):
  owner.observe_full_engagement(cs, enabled=enabled, active=enabled, source_ns=t, observation_valid=True, now_ns=t,
                                fault_active=False)


def steering_only(owner, cs):
  latch, pause_lat, pause_long = owner.output(cs)
  return latch and not pause_lat and not pause_long


@pytest.fixture
def setup():
  cp = filter_tja_cp()
  owner = create_intent(cp, SETTINGS)
  cs = driving()
  owner.update(cs, now_ns=T0, fault_active=False)
  engagement(owner, cs, False, T0)  # process start: cruise off
  return cp, owner, cs


def test_word_is_27_and_tja_owns_the_latch(setup):
  cp, owner, _ = setup
  assert cp.safetyConfigs[0].safetyParam == 27
  assert owner.tja_button and owner.explicit_latch and not owner.observe_stock_engagement


def test_tja_with_cruise_off_is_steering_only(setup):
  _, owner, cs = setup
  assert not owner.output(cs)[0]
  press(owner, cs, ButtonType.lkas, T0 + 100_000_000)
  assert steering_only(owner, cs)


def test_set_and_cancel_leave_steering_alone(setup):
  _, owner, cs = setup
  press(owner, cs, ButtonType.lkas, T0 + 100_000_000)
  press(owner, cs, ButtonType.setCruise, T0 + 200_000_000)
  engagement(owner, cs, True, T0 + 300_000_000)
  owner.update(cs, now_ns=T0 + 310_000_000, fault_active=False)
  assert steering_only(owner, cs)
  press(owner, cs, ButtonType.cancel, T0 + 400_000_000)
  engagement(owner, cs, False, T0 + 500_000_000)
  owner.update(cs, now_ns=T0 + 510_000_000, fault_active=False)
  assert steering_only(owner, cs)


def test_tja_off_while_cruising_stops_steering_only(setup):
  _, owner, cs = setup
  press(owner, cs, ButtonType.lkas, T0 + 100_000_000)
  press(owner, cs, ButtonType.setCruise, T0 + 200_000_000)
  engagement(owner, cs, True, T0 + 300_000_000)
  press(owner, cs, ButtonType.lkas, T0 + 400_000_000)
  latch, _, pause_long = owner.output(cs)
  assert not latch and not pause_long  # cruise is not paused by the steering toggle


def test_latch_holds_through_a_stop_and_a_filter_dropout(setup):
  _, owner, cs = setup
  press(owner, cs, ButtonType.lkas, T0 + 100_000_000)
  stopped = driving(standstill=True)
  owner.update(stopped, now_ns=T0 + 200_000_000, fault_active=False)
  assert owner.output(stopped)[0]
  # carstate reports a board dropout as cruise lost, never accFaulted (test_engagement_only_while_the_board_confirms)
  cs.cruiseState.enabled = False
  owner.update(cs, now_ns=T0 + 300_000_000, fault_active=False)
  assert steering_only(owner, cs)


def test_disarming_fault_still_clears_the_latch(setup):
  _, owner, cs = setup
  press(owner, cs, ButtonType.lkas, T0 + 100_000_000)
  owner.update(cs, now_ns=T0 + 200_000_000, fault_active=True)
  assert not owner.output(cs)[0]
