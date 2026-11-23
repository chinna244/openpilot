"""Route 00000004--0ed7a0ac2e t=419.022: stopped, TJA latched, D->unknown->N->R. stock_intent's shift hold paused
lateral, but selfdrived's axis reported the wrongGear soft disable as availabilityPause and the transport check
cleared the latch (aol_transport_failed not_current_or_continuity, baseline False, reason availabilityPause)."""
from dataclasses import replace

import pytest
from openpilot.starpilot.aol.tests.test_mazda_brake_withdrawal import MS, NOW, fixture, sample


def shift_frame(native, cp, stamp, *, events=(('wrongGear', 'soft'), ('pcmDisable', 'user')), wire=None):
  withheld = replace(native, observedMonoTime=stamp - MS, validUntilMonoTime=stamp + 199 * MS, lateralAllowed=False,
                     longitudinalAllowed=False, requestedLateral=False, requestedLongitudinal=False,
                     sourcePandaStatesMonoTime=stamp - MS) if wire is None else wire
  sm = sample(cp, stamp, 3, withheld, combined=False, panda_stamp=stamp - MS)
  m = sm.messages['aolAxisState'].as_builder()
  a = m.aolAxisState
  a.mode, a.faultReason = 'off', 'availabilityPause'
  a.desiredLateral = a.lateralActive = a.nativeAcknowledged = False
  sm.messages['aolAxisState'] = m.as_reader()
  e = sm.messages['onroadEvents'].as_builder()
  out = e.init('onroadEvents', len(events))
  for i, (name, kind) in enumerate(events):
    out[i] = {'name': name, 'softDisable': kind == 'soft', 'userDisable': kind == 'user', 'immediateDisable': kind == 'immediate'}
  sm.messages['onroadEvents'] = e.as_reader()
  return sm


def setup():
  cp, cs, f, _, native, now = fixture(294)
  cs.brakePressed = False
  cs.vEgo = cs.vEgoRaw = 0.
  cs.cruiseState.enabled = False
  cs.gearShifter = 'unknown'
  stamp = NOW + 100 * MS
  return cp, cs, f, native, stamp, stamp + MS


def observe(f, sm, cp, cs, now, **kw):
  return f.observe(sm, cp, cs, now, latched=kw.pop('latched', True), intent_producer_session='producer', intent_paused=True, **kw)


@pytest.mark.parametrize('gear', ['unknown', 'neutral', 'reverse', 'park'])
def test_shift_hold_keeps_latch(gear):
  cp, cs, f, native, stamp, now = setup()
  cs.gearShifter = gear
  assert not observe(f, shift_frame(native, cp, stamp), cp, cs, now, shift_hold=True)
  assert not f.failed


def test_shift_path_refuses_drive():
  """Drive is not a shift: the new path never retains it (Drive keeps its existing availability handling)."""
  cp, cs, f, native, stamp, now = setup()
  observe(f, shift_frame(native, cp, stamp - 50 * MS), cp, cs, now - 50 * MS, shift_hold=True)
  cs.gearShifter = 'drive'
  sm = shift_frame(native, cp, stamp)
  axis = sm['aolAxisState']
  assert not f._mazda_shift_pause_current(sm, cp, cs, axis, int(sm.logMonoTime['aolAxisState']), now, latched=True,
                                          producer='producer')


def test_without_shift_hold_still_fails():
  cp, cs, f, native, stamp, now = setup()
  assert observe(f, shift_frame(native, cp, stamp), cp, cs, now, shift_hold=False)


@pytest.mark.parametrize('fault', ['can_invalid', 'can_timeout', 'steer_perm', 'acc_fault', 'immediate',
                                   'other_soft', 'seatbelt_exit', 'no_wrong_gear', 'wire_lat_allowed', 'wire_lat_requested',
                                   'wire_stale', 'event_not_acknowledged', 'axis_lateral_active', 'producer'])
def test_unproven_shift_still_fails(fault):
  cp, cs, f, native, stamp, now = setup()
  events = (('wrongGear', 'soft'), ('pcmDisable', 'user'))
  wire = None
  if fault == 'can_invalid':
    cs.canValid = False
  elif fault == 'can_timeout':
    cs.canTimeout = True
  elif fault == 'steer_perm':
    cs.steerFaultPermanent = True
  elif fault == 'acc_fault':
    cs.accFaulted = True
  elif fault == 'immediate':
    events += (('controlsMismatch', 'immediate'),)
  elif fault == 'other_soft':
    events += (('steerTempUnavailable', 'soft'),)
  elif fault == 'seatbelt_exit':  # route 04 t=430.74: unbuckled in Park is an exit, not a shift
    events += (('seatbeltNotLatched', 'soft'),)
  elif fault == 'no_wrong_gear':
    events = (('pcmDisable', 'user'),)
  elif fault in ('wire_lat_allowed', 'wire_lat_requested'):
    wire = replace(native, observedMonoTime=stamp - MS, validUntilMonoTime=stamp + 199 * MS, longitudinalAllowed=False,
                   requestedLongitudinal=False, sourcePandaStatesMonoTime=stamp - MS,
                   lateralAllowed=fault == 'wire_lat_allowed', requestedLateral=fault == 'wire_lat_requested')
  elif fault == 'wire_stale':
    wire = replace(native, observedMonoTime=stamp - 250 * MS, validUntilMonoTime=stamp - 50 * MS, lateralAllowed=False,
                   longitudinalAllowed=False, requestedLateral=False, requestedLongitudinal=False,
                   sourcePandaStatesMonoTime=stamp - 250 * MS)
  sm = shift_frame(native, cp, stamp, events=events, wire=wire)
  if fault == 'event_not_acknowledged':
    m = sm.messages['aolAxisState'].as_builder()
    m.aolAxisState.faultEventMonoTime = stamp - 40 * MS
    sm.messages['aolAxisState'] = m.as_reader()
  elif fault == 'axis_lateral_active':
    m = sm.messages['aolAxisState'].as_builder()
    m.aolAxisState.lateralActive = True
    sm.messages['aolAxisState'] = m.as_reader()
  elif fault == 'producer':
    m = sm.messages['aolAxisState'].as_builder()
    m.aolAxisState.intentProducerSessionId = 'other'
    sm.messages['aolAxisState'] = m.as_reader()
  assert observe(f, sm, cp, cs, now, shift_hold=True), fault
