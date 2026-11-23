"""Route 00000003--941020f467 t=194.234 and 294.046: a Mazda brake/cancel withdraws long at the panda
(pandaStates.controlsAllowed False) in the same pandad cycle as the wire that withdraws it, 0.3 ms apart. card read
the new pandaStates against the previous wire (long still requested) and the transport check hard-failed the TJA
latch (aol_transport_failed not_current_or_continuity, transport_healthy False). Ford/VW/GM/HKG already had this
bounded window; Mazda had none."""
from dataclasses import replace

import pytest
from opendbc.car import structs
from openpilot.cereal import log
from openpilot.starpilot.aol.transport_pause import TransportPauseFeedback
from openpilot.starpilot.aol.wire import PandaSlot, SafetyState, encode_safety
from openpilot.starpilot.car.mazda.tests.test_filter_tja_transitions import filter_tja_cp

NOW = 10_000_000_000
MS = 1_000_000


class Peers:
  def __init__(self, messages):
    self.messages = {m.which(): m for m in messages}
    self.logMonoTime = {k: m.logMonoTime for k, m in self.messages.items()}
    self.seen = dict.fromkeys(self.messages, True)
    self.valid = dict(self.seen)
    self.alive = dict(self.seen)

  def __getitem__(self, name):
    return getattr(self.messages[name], name)


def message(name, stamp, size=None):
  m = log.Event.new_message(logMonoTime=stamp, valid=True)
  m.init(name) if size is None else m.init(name, size)
  return m


def sample(cp, stamp, sequence, native, *, combined, events=(), panda_stamp=None, raw_controls=False):
  w = message('aolSafetyWire', native.observedMonoTime, 0)
  w.aolSafetyWire = encode_safety(native)
  a = message('aolAxisState', stamp)
  a.aolAxisState = {'sessionId': 'axis', 'faultSessionId': 'axis', 'sequence': sequence, 'qualified': True,
    'faultReason': 'none', 'observedMonoTime': stamp, 'sourceCarStateMonoTime': stamp, 'validUntilMonoTime': stamp + 30 * MS,
    'intentProducerSessionId': 'producer', 'intentIntegrityEpoch': 1, 'faultEventMonoTime': stamp,
    'desiredLateral': True, 'lateralActive': True, 'desiredLongitudinal': combined, 'longitudinalActive': combined,
    'nativeAcknowledged': True, 'mode': 'combined' if combined else 'lateralOnly'}
  config = cp.safetyConfigs[0]
  p = message('pandaStates', stamp if panda_stamp is None else panda_stamp, 1)
  p.pandaStates[0] = {'safetyModel': config.safetyModel, 'safetyParam': config.safetyParam,
                      'alternativeExperience': cp.alternativeExperience, 'controlsAllowed': raw_controls}
  e = message('onroadEvents', stamp, len(events))
  for i, (name, user) in enumerate(events):
    e.onroadEvents[i] = {'name': name, 'userDisable': user}
  return Peers([w.as_reader(), a.as_reader(), p.as_reader(), e.as_reader()])


def fixture(route_case):
  cp = filter_tja_cp()
  assert cp.brand == 'mazda' and cp.openpilotLongitudinalControl and cp.safetyConfigs[0].safetyParam == 27
  config = cp.safetyConfigs[0]
  cs = structs.CarState(canValid=True, vEgo=2.3, vEgoRaw=2.3, gearShifter='drive')
  cs.cruiseState.available = cs.cruiseState.enabled = True
  slot = PandaSlot(0, 'panda', int(config.safetyModel.raw), config.safetyParam, cp.alternativeExperience, True)
  native = SafetyState(1, True, NOW, NOW + 200 * MS, int(config.safetyModel.raw), config.safetyParam, True, True, True, True,
                       'panda', 'axis', sourcePandaStatesMonoTime=NOW, pandaInventory=(slot,))
  f = TransportPauseFeedback(cp)
  # healthy combined baseline, panda controls allowed
  assert not f.observe(sample(cp, NOW, 1, native, combined=True, raw_controls=True), cp, cs, NOW,
                       latched=True, intent_producer_session='producer')
  # the race frame: brake; new pandaStates (controls withdrawn) newer than the still-current previous wire
  cs.brakePressed = True
  stamp = NOW + 100 * MS
  if route_case == 194:  # cruise already off, axis lateral-only, pedal + pcm disable events
    cs.cruiseState.enabled = False
    sm = sample(cp, stamp, 2, native, combined=False, events=(('pedalPressed', True), ('pcmDisable', True)))
  else:  # 294: first brake frames, cruise still on, axis still combined, no events yet
    sm = sample(cp, stamp, 2, native, combined=True)
  return cp, cs, f, sm, native, stamp + MS


def invoke(cp, cs, f, sm, now, latched=True):
  return f.observe(sm, cp, cs, now, latched=latched, intent_producer_session='producer')


@pytest.mark.parametrize('route_case', [194, 294])
def test_brake_withdrawal_race_keeps_lateral(route_case):
  cp, cs, f, sm, native, now = fixture(route_case)
  assert not invoke(cp, cs, f, sm, now)
  assert not f.failed


@pytest.mark.parametrize('route_case', [194, 294])
@pytest.mark.parametrize('fault', ['lat_denied', 'lat_not_requested', 'panda_not_newer', 'reverse',
                                   'door', 'can_invalid', 'steer_fault', 'gas', 'immediate', 'unlatched', 'main_off',
                                   'no_brake_cruise_on', 'session'])
def test_unproven_withdrawal_still_fails(route_case, fault):
  cp, cs, f, sm, native, now = fixture(route_case)
  latched = True
  if fault in ('lat_denied', 'lat_not_requested', 'session'):
    changed = (replace(native, lateralAllowed=False) if fault == 'lat_denied' else
               replace(native, requestedLateral=False) if fault == 'lat_not_requested' else replace(native, axisSessionId='other'))
    m = sm.messages['aolSafetyWire'].as_builder()
    m.aolSafetyWire = encode_safety(changed)
    sm.messages['aolSafetyWire'] = m.as_reader()
  elif fault == 'panda_not_newer':
    m = sm.messages['pandaStates'].as_builder()
    m.logMonoTime = NOW
    sm.messages['pandaStates'] = m.as_reader()
    sm.logMonoTime['pandaStates'] = NOW
  elif fault == 'reverse':
    cs.gearShifter = 'reverse'
  elif fault == 'door':
    cs.doorOpen = True
  elif fault == 'can_invalid':
    cs.canValid = False
  elif fault == 'steer_fault':
    cs.steerFaultTemporary = True
  elif fault == 'gas':
    cs.gasPressed = True
  elif fault == 'immediate':
    m = sm.messages['onroadEvents'].as_builder()
    events = m.init('onroadEvents', 1)
    events[0] = {'name': 'controlsMismatch', 'immediateDisable': True}
    sm.messages['onroadEvents'] = m.as_reader()
  elif fault == 'unlatched':
    latched = False
  elif fault == 'main_off':
    cs.cruiseState.available = False
  elif fault == 'no_brake_cruise_on':
    if route_case == 194:
      pytest.skip('cruise is already off in the 194 case')
    cs.brakePressed = False
  result = invoke(cp, cs, f, sm, now, latched)
  assert result or not latched, fault


@pytest.mark.parametrize('route_case', [194, 294])
def test_consistent_panda_and_wire_is_ordinary_health(route_case):
  """No race (the panda still allows control): the ordinary healthy path, nothing retained by the new window."""
  cp, cs, f, sm, native, now = fixture(route_case)
  m = sm.messages['pandaStates'].as_builder()
  m.pandaStates[0].controlsAllowed = True
  sm.messages['pandaStates'] = m.as_reader()
  assert not invoke(cp, cs, f, sm, now)


def test_other_brands_are_unchanged():
  """The Ford fixture keeps its own window; nothing here widens it (the Mazda flag needs brand mazda)."""
  from openpilot.starpilot.aol.tests import test_ford_cancel_raw_health as ford
  cp, cs, f, sm, native, now = ford.fixture()
  assert not ford.invoke(cp, cs, f, sm, now)
