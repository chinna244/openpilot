"""Mazda AOL + alpha long: a brake or cancel drops the native LONG bit with controls_allowed, possibly one
sample before Card parses the pedal edge. Mazda grants no LAT at a stop, so only the Mazda withdrawal excuses it."""
from dataclasses import replace
from types import SimpleNamespace
from unittest import mock

import pytest
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.stock_aol import qualified
from opendbc.car.mazda.values import CAR
from openpilot.selfdrive.selfdrived.events import ET
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.starpilot.aol.runtime import AxisDecision
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState
from openpilot.starpilot.car.mazda.aol import policy_for
from openpilot.starpilot.vehicle_preferences import VehicleStartupPreferences

NOW = 10_000_000_000
SOURCE = NOW - 10_000_000
SESSION, PRODUCER, EPOCH = 'axis-session', 'producer', 7
PREVIOUS = AxisDecision(desired_lateral=True, desired_longitudinal=True, lateral_active=False, longitudinal_active=True)
REQUESTED = AxisDecision(desired_lateral=True, desired_longitudinal=True)


def mazda_cp(alpha=True):
  cp = VehicleStartupPreferences(mazda_tja=True).prepare(
    CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], alpha, False, False))
  policy = policy_for(cp)
  cp.safetyConfigs[-1].safetyParam |= policy.safety_param_addition
  cp.alternativeExperience |= policy.alternative_experience_addition
  return cp


def native_state(cp, *, observed=NOW - 10_000_000, controls_allowed=False, long_allowed=False):
  word = int(cp.safetyConfigs[0].safetyParam)
  model = int(structs.CarParams.SafetyModel.mazda)
  slot = PandaSlot(0, 'serial', model, word, int(cp.alternativeExperience), controlsAllowed=controls_allowed)
  return SafetyState(2, True, observed, observed + 200_000_000, model, word, False, long_allowed, True, True,
                     'serial', SESSION, observed, (slot,))


def selfdrived(cp, *, panda_controls_allowed=False, events=()):
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.CP = cp.as_reader()
  sd.enabled = sd.active = sd.conditional_car_state_valid = True
  sd.aol_car_state_log_ns, sd.aol_session_id = SOURCE, SESSION
  sd.aol_published_input_proof = (SESSION, SOURCE, NOW - 5_000_000, NOW + 200_000_000, PRODUCER, EPOCH,
                                  True, True, False, True)
  sd.aol_published_input_native = native_state(cp, observed=NOW - 50_000_000, controls_allowed=True, long_allowed=True)
  sd.aol_intent_companion = SimpleNamespace(integrity_epoch=EPOCH)
  sd.events = SimpleNamespace(contains=lambda kind: kind in events)
  sd.sm = {'pandaStates': [SimpleNamespace(controlsAllowed=panda_controls_allowed)]}
  return sd


@pytest.fixture
def case():
  cp = mazda_cp()
  assert cp.openpilotLongitudinalControl and int(cp.safetyConfigs[0].safetyParam) == 11
  assert qualified(cp, marked_only=True)
  with mock.patch('openpilot.starpilot.aol.transport_pause._configured_transport_native',
                  side_effect=lambda sm, cp, session, now, native, **kw: native) as health:
    yield SimpleNamespace(cp=cp, sd=selfdrived(cp), native=native_state(cp), cs=structs.CarState(canValid=True),
                          intent=SimpleNamespace(producerSessionId=PRODUCER, allowedLatch=False, lateralArmed=False),
                          previous=PREVIOUS, requested=REQUESTED, health=health)


def withdraw(c):
  return c.sd.mazda_alpha_long_withdrawal(c.cs, c.native, c.previous, c.requested, c.intent,
                                          source_current=True, now_ns=NOW)


def test_brake_at_a_stop_withdraws_long_without_lateral(case):
  # The LAT-anchored ordinary withdrawal cannot excuse it: no LAT at a stop, TJA latch need not be armed.
  assert not case.sd.ordinary_native_long_withdrawal(case.cs, case.native, case.previous, case.requested, case.intent,
                                                     source_current=True, now_ns=NOW)
  assert withdraw(case)
  assert case.health.call_args.kwargs == {'require_longitudinal': False, 'allow_restricted_lateral': True}


def test_lateral_active_moving_also_withdraws(case):
  case.previous = replace(PREVIOUS, lateral_active=True)
  case.intent.allowedLatch = case.intent.lateralArmed = True
  assert withdraw(case)


@pytest.mark.parametrize('change', (
  'inventory_controls', 'panda_states_controls', 'long_still_allowed', 'long_not_requested', 'not_long_active',
  'not_desired_long', 'disabled', 'stale_source', 'proof_session', 'proof_not_long', 'proof_expired', 'producer',
  'epoch', 'no_intent', 'unhealthy', 'can_invalid', 'gas', 'brake_seen', 'immediate', 'soft', 'user', 'serial',
  'older_native', 'no_inventory',
))
def test_every_other_drop_stays_a_lost_permission(case, change):
  c = case
  if change == 'inventory_controls':
    c.native = native_state(c.cp, controls_allowed=True)
  elif change == 'panda_states_controls':
    c.sd.sm['pandaStates'][0].controlsAllowed = True
  elif change == 'long_still_allowed':
    c.native = native_state(c.cp, long_allowed=True)
  elif change == 'long_not_requested':
    c.native = replace(c.native, requestedLongitudinal=False)
  elif change == 'not_long_active':
    c.previous = replace(PREVIOUS, longitudinal_active=False)
  elif change == 'not_desired_long':
    c.requested = replace(REQUESTED, desired_longitudinal=False)
  elif change == 'disabled':
    c.sd.enabled = False
  elif change == 'stale_source':
    c.sd.aol_car_state_log_ns = NOW - 2_000_000_000
  elif change == 'proof_session':
    c.sd.aol_published_input_proof = ('foreign',) + c.sd.aol_published_input_proof[1:]
  elif change == 'proof_not_long':
    c.sd.aol_published_input_proof = c.sd.aol_published_input_proof[:9] + (False,)
  elif change == 'proof_expired':
    p = c.sd.aol_published_input_proof
    c.sd.aol_published_input_proof = p[:3] + (NOW - 1,) + p[4:]
  elif change == 'producer':
    c.intent.producerSessionId = 'foreign'
  elif change == 'epoch':
    c.sd.aol_intent_companion.integrity_epoch = EPOCH + 1
  elif change == 'no_intent':
    c.intent = None
  elif change == 'unhealthy':
    c.health.side_effect = lambda *args, **kw: None
  elif change == 'can_invalid':
    c.cs.canValid = False
  elif change == 'gas':
    c.cs.gasPressed = True
  elif change == 'brake_seen':
    c.cs.brakePressed = True  # driver_override already excuses it in the caller
  elif change in ('immediate', 'soft', 'user'):
    kind = {'immediate': ET.IMMEDIATE_DISABLE, 'soft': ET.SOFT_DISABLE, 'user': ET.USER_DISABLE}[change]
    c.sd.events = SimpleNamespace(contains=lambda k: k == kind)
  elif change == 'serial':
    c.native = replace(c.native, pandaSerial='foreign')
  elif change == 'older_native':
    c.native = native_state(c.cp, observed=NOW - 60_000_000)
  elif change == 'no_inventory':
    c.native = replace(c.native, pandaInventory=())
  assert not withdraw(c)


@pytest.mark.parametrize('alpha', (False, True))
def test_only_mazda_aol_alpha_long(alpha):
  cp = mazda_cp(alpha)
  if alpha:
    cp.alternativeExperience = 0  # no AOL: ordinary cruise-coupled path, nothing to excuse here
  else:
    assert not cp.openpilotLongitudinalControl
  sd = selfdrived(cp)
  with mock.patch('openpilot.starpilot.aol.transport_pause._configured_transport_native',
                  side_effect=lambda sm, cp, session, now, native, **kw: native):
    assert not sd.mazda_alpha_long_withdrawal(structs.CarState(canValid=True), native_state(cp), PREVIOUS, REQUESTED,
                                              SimpleNamespace(producerSessionId=PRODUCER), source_current=True, now_ns=NOW)
