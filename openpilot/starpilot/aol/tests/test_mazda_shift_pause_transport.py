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


def test_drive_in_the_settle_window_keeps_latch():
  """Route 00000007--7d9475933b t=362.19: R->N->D, the first Drive frame still carried the shift's availabilityPause."""
  cp, cs, f, native, stamp, now = setup()
  cs.gearShifter = 'drive'
  assert not observe(f, shift_frame(native, cp, stamp), cp, cs, now, shift_hold=True)


def test_shift_path_refuses_drive_at_speed():
  cp, cs, f, native, stamp, now = setup()
  observe(f, shift_frame(native, cp, stamp - 50 * MS), cp, cs, now - 50 * MS, shift_hold=True)
  cs.gearShifter = 'drive'
  cs.vEgo = cs.vEgoRaw = 3.
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


def drive_settle_frame(native, cp, stamp, event_offset_ms):
  """R->D settle window with onroadEvents one selfdrived cycle off the event the axis acknowledged."""
  sm = shift_frame(native, cp, stamp)
  m = sm.messages['aolAxisState'].as_builder()
  m.aolAxisState.faultEventMonoTime = stamp - 10 * MS
  sm.messages['aolAxisState'] = m.as_reader()
  sm.logMonoTime['onroadEvents'] = stamp - 10 * MS + event_offset_ms * MS
  return sm


@pytest.mark.parametrize('offset_ms', [10, -10])
def test_drive_settle_survives_a_one_cycle_event_order_gap(offset_ms):
  """After a 3 s reverse, the gear event 10 ms behind (or ahead of) the axis must not clear the TJA latch."""
  cp, cs, f, native, stamp, now = setup()
  cs.gearShifter = 'drive'
  sm = drive_settle_frame(native, cp, stamp, offset_ms)
  assert not observe(f, sm, cp, cs, now + 10 * MS, shift_hold=True)
  assert not f.failed


def test_drive_settle_order_gap_is_bounded():
  cp, cs, f, native, stamp, now = setup()
  cs.gearShifter = 'drive'
  sm = drive_settle_frame(native, cp, stamp, 30)
  assert observe(f, sm, cp, cs, now + 30 * MS, shift_hold=True)


def test_drive_settle_order_gap_still_needs_steering_withheld():
  cp, cs, f, native, stamp, now = setup()
  cs.gearShifter = 'drive'
  wire = replace(native, observedMonoTime=stamp - MS, validUntilMonoTime=stamp + 199 * MS, lateralAllowed=True,
                 longitudinalAllowed=False, requestedLateral=False, requestedLongitudinal=False, sourcePandaStatesMonoTime=stamp - MS)
  sm = shift_frame(native, cp, stamp, wire=wire)
  m = sm.messages['aolAxisState'].as_builder()
  m.aolAxisState.faultEventMonoTime = stamp - 10 * MS
  sm.messages['aolAxisState'] = m.as_reader()
  assert observe(f, sm, cp, cs, now, shift_hold=True)


def reverse_history(f, native, cp, cs, stamp):
  """Three seconds of a stopped Reverse shift hold from the fixture's first shift frame; returns the Drive frame's stamp
  (Codex review of 49bd11b2c)."""
  cs.gearShifter = 'reverse'
  for t in range(stamp, stamp + 3_000 * MS, 100 * MS):
    assert not observe(f, shift_frame(native, cp, t), cp, cs, t + MS, shift_hold=True)
  cs.gearShifter = 'drive'
  return stamp + 3_000 * MS


def test_drive_settle_event_strictly_newer_than_the_axis_keeps_latch():
  cp, cs, f, native, stamp, now = setup()
  stamp = reverse_history(f, native, cp, cs, stamp)
  sm = shift_frame(native, cp, stamp)
  m = sm.messages['aolAxisState'].as_builder()
  m.aolAxisState.faultEventMonoTime = stamp - 5 * MS
  sm.messages['aolAxisState'] = m.as_reader()
  sm.logMonoTime['onroadEvents'] = stamp + 5 * MS  # published after the axis it lags
  assert not observe(f, sm, cp, cs, stamp + 10 * MS, shift_hold=True)
  assert not f.failed


def test_drive_settle_event_30ms_older_than_the_axis_fails():
  cp, cs, f, native, stamp, now = setup()
  stamp = reverse_history(f, native, cp, cs, stamp)
  now = stamp + MS
  sm = shift_frame(native, cp, stamp)
  m = sm.messages['aolAxisState'].as_builder()
  m.aolAxisState.faultEventMonoTime = stamp - MS
  sm.messages['aolAxisState'] = m.as_reader()
  sm.logMonoTime['onroadEvents'] = stamp - 31 * MS
  assert observe(f, sm, cp, cs, now, shift_hold=True)


@pytest.mark.parametrize('gear', ['park', 'reverse', 'neutral'])
def test_steady_gear_with_unchanged_events_keeps_latch(gear):
  """Route 00000008--80148a3eeb t=477.69 -> 479.37: shifted to Park, selfdrived's events stopped changing, so onroadEvents
  (published on change) aged past 1.5 s while the axis was current every frame; the transport failed the latch
  (aol_transport_failed not_current_or_continuity -> mazda_tja_latch_cleared disarming_fault). Reverse survived only
  because steerOverride kept toggling the event set."""
  cp, cs, f, native, stamp, now = setup()
  cs.gearShifter = gear
  first = stamp
  for k in range(0, 500, 10):  # 5 s at 10 Hz samples
    t = stamp + k * 10 * MS
    sm = shift_frame(native, cp, t)
    m = sm.messages['aolAxisState'].as_builder()
    m.aolAxisState.faultEventMonoTime = first  # the shift's event, still the axis's fault
    sm.messages['aolAxisState'] = m.as_reader()
    sm.logMonoTime['onroadEvents'] = first  # unchanged since the shift
    assert not observe(f, sm, cp, cs, t + MS, shift_hold=True), k
  assert not f.failed


@pytest.mark.parametrize('gear', ['drive', 'unknown'])
def test_transitional_gear_keeps_the_event_age_bound(gear):
  cp, cs, f, native, stamp, now = setup()
  cs.gearShifter = gear
  first = stamp
  failed_at = None
  for k in range(0, 300, 10):
    t = stamp + k * 10 * MS
    sm = shift_frame(native, cp, t)
    m = sm.messages['aolAxisState'].as_builder()
    m.aolAxisState.faultEventMonoTime = first
    sm.messages['aolAxisState'] = m.as_reader()
    sm.logMonoTime['onroadEvents'] = first
    if observe(f, sm, cp, cs, t + MS, shift_hold=True):
      failed_at = k
      break
  assert failed_at is not None and 140 <= failed_at <= 160  # Transitional gears keep the short pause window
