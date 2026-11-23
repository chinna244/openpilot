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


def sample(cp, stamp, sequence, native, *, combined, events=(), panda_stamp=None, raw_controls=False, event_stamp=None):
  event_stamp = stamp if event_stamp is None else event_stamp
  w = message('aolSafetyWire', native.observedMonoTime, 0)
  w.aolSafetyWire = encode_safety(native)
  a = message('aolAxisState', stamp)
  a.aolAxisState = {'sessionId': 'axis', 'faultSessionId': 'axis', 'sequence': sequence, 'qualified': True,
    'faultReason': 'none', 'observedMonoTime': stamp, 'sourceCarStateMonoTime': stamp, 'validUntilMonoTime': stamp + 30 * MS,
    'intentProducerSessionId': 'producer', 'intentIntegrityEpoch': 1, 'faultEventMonoTime': event_stamp,
    'desiredLateral': True, 'lateralActive': True, 'desiredLongitudinal': combined, 'longitudinalActive': combined,
    'nativeAcknowledged': True, 'mode': 'combined' if combined else 'lateralOnly'}
  config = cp.safetyConfigs[0]
  p = message('pandaStates', stamp if panda_stamp is None else panda_stamp, 1)
  p.pandaStates[0] = {'safetyModel': config.safetyModel, 'safetyParam': config.safetyParam,
                      'alternativeExperience': cp.alternativeExperience, 'controlsAllowed': raw_controls}
  e = message('onroadEvents', event_stamp, len(events))
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
    if route_case == 194:
      pytest.skip('cruise off + fresh pcmDisable is the cancel evidence; gas held is covered by the 169 tests')
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


def gas_fixture():
  """Route 00000004--0ed7a0ac2e t=169.660: cruise off with the accelerator held, brake up. The panda withdrew long only
  (wire 169.7259 lat allowed, long withdrawn); card ran at 169.7278 on the new pandaStates (169.7263, controlsAllowed
  False) against the previous wire (169.6237, long still requested) and cleared the TJA latch."""
  cp, cs, f, _, native, now = fixture(194)
  cs.brakePressed = False
  cs.gasPressed = True
  # onroadEvents is sent on change: the pcmDisable message is 61 ms old when the panda's withdrawal lands
  sm = sample(cp, now - MS, 2, native, combined=False, events=EVENTS, event_stamp=now - 61 * MS)
  return cp, cs, f, sm, native, now


EVENTS = (('gasPressedOverride', False), ('pcmDisable', True))


def test_gas_held_cruise_off_keeps_lateral():
  cp, cs, f, sm, native, now = gas_fixture()
  assert not invoke(cp, cs, f, sm, now)
  assert not f.failed
  # next pandad cycle: the wire now shows the panda kept lateral and withdrew long
  lat_only = replace(native, observedMonoTime=now + 98 * MS, validUntilMonoTime=now + 298 * MS, longitudinalAllowed=False,
                     requestedLongitudinal=False, sourcePandaStatesMonoTime=now + 98 * MS)
  sm2 = sample(cp, now + 99 * MS, 3, lat_only, combined=False, events=EVENTS, panda_stamp=now + 98 * MS, event_stamp=now - 61 * MS)
  assert not invoke(cp, cs, f, sm2, now + 100 * MS)
  assert not f.failed


@pytest.mark.parametrize('fault', ['no_cancel_event', 'cruise_on', 'axis_long', 'lat_denied', 'next_wire_lat_denied',
                                   'stale_event', 'event_not_acknowledged', 'event_past_safety_age', 'reverse', 'can_invalid'])
def test_gas_held_unproven_withdrawal_still_fails(fault):
  cp, cs, f, sm, native, now = gas_fixture()
  if fault == 'no_cancel_event':
    sm = sample(cp, now - MS, 2, native, combined=False, events=(('gasPressedOverride', False),), event_stamp=now - 61 * MS)
  elif fault == 'cruise_on':
    cs.cruiseState.enabled = True
    sm = sample(cp, now - MS, 2, native, combined=True)
  elif fault == 'axis_long':
    sm = sample(cp, now - MS, 2, native, combined=True, events=EVENTS, event_stamp=now - 61 * MS)
  elif fault == 'lat_denied':
    sm = sample(cp, now - MS, 2, replace(native, lateralAllowed=False), combined=False, events=EVENTS, event_stamp=now - 61 * MS)
  elif fault == 'event_not_acknowledged':
    sm = sample(cp, now - MS, 2, native, combined=False, events=EVENTS, event_stamp=now - 61 * MS)
    m = sm.messages['aolAxisState'].as_builder()
    m.aolAxisState.faultEventMonoTime = now - 90 * MS
    sm.messages['aolAxisState'] = m.as_reader()
  elif fault == 'event_past_safety_age':
    sm = sample(cp, now - MS, 2, native, combined=False, events=EVENTS, event_stamp=now - 250 * MS)
  elif fault == 'stale_event':
    m = sm.messages['onroadEvents'].as_builder()
    m.logMonoTime = NOW - 2_000 * MS
    sm.messages['onroadEvents'] = m.as_reader()
    sm.logMonoTime['onroadEvents'] = NOW - 2_000 * MS
  elif fault == 'reverse':
    cs.gearShifter = 'reverse'
  elif fault == 'can_invalid':
    cs.canValid = False
  if fault == 'next_wire_lat_denied':
    assert not invoke(cp, cs, f, sm, now)
    gone = replace(native, observedMonoTime=now + 98 * MS, validUntilMonoTime=now + 298 * MS, lateralAllowed=False,
                   longitudinalAllowed=False, requestedLongitudinal=False, sourcePandaStatesMonoTime=now + 98 * MS)
    sm = sample(cp, now + 99 * MS, 3, gone, combined=False, events=EVENTS, panda_stamp=now + 98 * MS, event_stamp=now - 61 * MS)
    now += 100 * MS
  assert invoke(cp, cs, f, sm, now), fault


def test_route_194_brake_and_gas_with_cruise_off_keeps_lateral():
  """Both pedals with cruise already off and a fresh pcmDisable: the cancel evidence carries it (as the gas-held 169 case);
  the next wire still has to keep lateral allowed."""
  cp, cs, f, sm, native, now = fixture(194)
  cs.gasPressed = True
  assert not invoke(cp, cs, f, sm, now)
  assert not f.failed



def test_gas_held_cancel_bridge_ends_without_fresh_panda_data():
  cp, cs, feedback, first, native, now = gas_fixture()
  assert not invoke(cp, cs, feedback, first, now)

  # Axis ticks continue, but both the Panda message and its wire remain the same: the bridge covers only the pandad
  # race, so it ends 30 ms after the withdrawn pandaStates, not at the old wire's 200 ms lease.
  panda = first.logMonoTime['pandaStates']
  values = dict(combined=False, events=EVENTS, panda_stamp=panda, event_stamp=first.logMonoTime['onroadEvents'])
  assert not invoke(cp, cs, feedback, sample(cp, panda + 28 * MS, 3, native, **values), panda + 29 * MS)
  assert invoke(cp, cs, feedback, sample(cp, panda + 30 * MS, 4, native, **values), panda + 31 * MS)
  assert feedback.failed


def race_frame(case, stamp, panda_stamp):
  """Axis and events fresh at stamp; the withdrawn pandaStates at panda_stamp; the bridged wire is the fixture's (NOW)."""
  cp, cs, f, _, native, _ = fixture(194 if case != 'brake294' else 294)
  if case == 'gas169':
    cs.brakePressed, cs.gasPressed = False, True
    sm = sample(cp, stamp, 2, native, combined=False, events=EVENTS, event_stamp=stamp - 61 * MS, panda_stamp=panda_stamp)
  elif case == 'brake194':
    sm = sample(cp, stamp, 2, native, combined=False, events=(('pedalPressed', True), ('pcmDisable', True)), panda_stamp=panda_stamp)
  else:
    sm = sample(cp, stamp, 2, native, combined=True, panda_stamp=panda_stamp)
  return cp, cs, f, sm


@pytest.mark.parametrize('case', ['brake194', 'brake294', 'gas169'])
@pytest.mark.parametrize('fault', ['wire_stalled', 'card_late'])
def test_bridge_only_covers_the_pandad_race(case, fault):
  """A stalled wire (pandaStates 180 ms past it, wire lease still valid) or a card reading the withdrawn pandaStates
  41 ms late must not ride the previous wire's lateral grant."""
  if fault == 'wire_stalled':
    stamp = panda = NOW + 180 * MS
  else:
    panda, stamp = NOW + 100 * MS, NOW + 140 * MS
  cp, cs, f, sm = race_frame(case, stamp, panda)
  assert invoke(cp, cs, f, sm, stamp + MS), (case, fault)


@pytest.mark.parametrize('case', ['brake194', 'gas169'])
def test_bridge_window_edges_still_hold(case):
  """pandaStates 150 ms after the bridged wire (pandad jitter) and card 29 ms after it: still the race."""
  edge = NOW + 150 * MS
  cp, cs, f, sm = race_frame(case, edge, edge)
  assert not invoke(cp, cs, f, sm, edge + 29 * MS)
