"""Review items: stale/unfilled/invalid planner data at card, planner loss during a SET walk, resets that must not
announce "Adjusting to", and StarPilot's saved per-speed-range offsets end to end. Runs card's own Car.mazda_sla_update."""
from types import SimpleNamespace as NS

import pytest

from openpilot.starpilot.car.mazda.sla import AssistState
from openpilot.starpilot.car.mazda.sla.mirror import SpeedLimitAssistMirror
from openpilot.starpilot.car.mazda.sla.session import MazdaSlaSession
from openpilot.starpilot.car.mazda.sla.tests.rig import CP, MPH, Rig, frame, spans

KPH_PER_MPH = 1.609344


def run(rig, n, limit_at, *, dash=36, presses=None, stall=lambda i: False, enabled=lambda i: True, hooks=None, v_mph=33.):
  presses = presses or {}
  for i in range(n):
    if hooks and i in hooks:
      hooks[i](rig)
    btn = presses.get(i, ())
    for name, pressed in btn:
      if pressed and name in ('accelCruise', 'decelCruise'):
        dash += 1 if name == 'accelCruise' else -1
    f = frame(i, v_mph=v_mph, dash=dash, map_mph=limit_at(i), vision_mph=limit_at(i), btn=btn, enabled=enabled(i))
    f['stall'] = stall(i)
    rig.step(f)
  return rig.rows


def tap(at, button='decelCruise', hold=8):
  return {at: [(button, True)], at + hold: [(button, False)]}


def first(rows, pred, start=0):
  return next(i for i in range(start, len(rows)) if pred(rows[i]))


def setpoint(r):
  return round(r.setpoint / KPH_PER_MPH)


# ---------------------------------------------------------------- stale / unfilled / invalid planner data at card

def test_stale_planner_data_holds_no_new_decision_and_no_set_press():
  """Active on a 30 cap (25 + 20 %), then plannerd stops publishing for 5 s while the limit changes to 35: card must not
  act on the old message (no prompt, no SET press, no new target); when plannerd returns the change is asked normally."""
  rig = Rig(card=True)
  rows = run(rig, 4000, lambda i: 25 if i < 1200 else 35, dash=30, stall=lambda i: 1100 <= i < 1600)
  assert rows[1090].state == AssistState.active and round(rows[1090].cap * MPH) == 30
  stalled = rows[1125:1600]   # slcState older than 200 ms from frame ~1120
  assert all(r.send == 'none' for r in stalled)
  assert all(round(r.offer * MPH) == 30 for r in stalled)        # no decision on data the card cannot trust
  assert not any(r.text for r in rows[1135:1600])                 # selfdrived drops the stale message too (100 ms)
  after = first(rows, lambda r: r.state == AssistState.preActive, 1600)
  assert round(rows[after].offer * MPH) == 42
  assert any(s[2] == 'Press + for 42 mph' for s in spans(rows[1600:]))


def test_unfilled_mazda_sla_never_forms_a_session_or_cap():
  """UseStarPilotLongitudinalPlanner off: slcState without mazdaSla (all capnp zeros). No limit, no cap, no prompt, no press."""
  rig = Rig(card=True)
  real_fill = rig.planner.fill

  def unfilled(out):
    real_fill(out)
    for k in vars(out):
      setattr(out, k, 0 if not isinstance(getattr(out, k), bool) else False)
  rig.planner.fill = unfilled
  rows = run(rig, 3000, lambda i: 25, dash=36)
  assert all(r.cap == 255. and r.send == 'none' and not r.text for r in rows)
  assert all(r.state in (AssistState.disabled, AssistState.inactive) for r in rows)
  assert all(setpoint(r) == 36 for r in rows[100:])


def test_invalid_slc_verdict_does_not_stop_the_session():
  """slcState.valid False (StarPilot SLC's own verdict): card and selfdrived both still run the session (prompt shown)."""
  rig = Rig(card=True)
  rows = run(rig, 2500, lambda i: 30 if i < 1000 else 25, dash=36,
             hooks={0: lambda r: setattr(r.sd.sm, 'valid', {**r.sd.sm.valid, 'slcState': False})})
  assert any(s[2] == 'Press - for 30 mph' for s in spans(rows))


# ---------------------------------------------------------------- planner loss during an active SET walk

@pytest.mark.parametrize('disengage', [False, True])
def test_planner_loss_mid_walk_stops_set_presses(disengage):
  """- confirms 30 from SET 36 and the walk starts; plannerd stalls 3 s mid-walk. Once its data has expired (card: older than
  200 ms) no factory SET press goes out, including the one already queued; when plannerd returns the walk restarts from a
  fresh decision and finishes on 30, never below. Disengaging during the stall keeps it stopped."""
  rig0 = Rig(card=True)
  rows0 = run(rig0, 2000, lambda i: 30 if i < 1000 else 25)
  p = first(rows0, lambda r: r.state == AssistState.preActive, 1000)
  rig = Rig(card=True)
  stall0, stall1 = p + 120, p + 420
  rows = run(rig, 3500, lambda i: 30 if i < 1000 else 25, presses=tap(p + 50), stall=lambda i: stall0 <= i < stall1,
             enabled=(lambda i: not (disengage and i >= stall0 + 60)))
  assert setpoint(rows[p + 70]) == 30
  assert any(r.send == 'decrease' for r in rows[p + 60:stall0])          # the walk was running
  expired = stall0 + 21                                                   # last tick + 200 ms freshness + the queued frame
  assert all(r.send == 'none' for r in rows[expired:stall1])             # expiry: nothing more, queued press dropped
  dash_at_stall = rows[stall1 - 1].dash
  assert dash_at_stall > 30
  assert all(r.send != 'increase' for r in rows) and min(r.dash for r in rows) >= 30
  if disengage:
    assert all(r.send == 'none' for r in rows[stall0 + 61:])
  else:
    assert rows[-1].dash == 30                                            # resumed after data returned
    resumed = first(rows, lambda r: r.send == 'decrease', stall1)
    assert resumed - stall1 >= 40                                         # a fresh servo decision (0.4 s), not the old queue


# ---------------------------------------------------------------- resets must not announce

def _active_after_confirm():
  rig0 = Rig(card=True)
  rows0 = run(rig0, 2000, lambda i: 30 if i < 1000 else 25)
  return first(rows0, lambda r: r.state == AssistState.preActive, 1000)


def _adjusting(rows, after):
  return [s for s in spans(rows) if s[2].startswith('Adjusting') and s[0] >= rows[after].ts]


def test_plannerd_restart_does_not_announce():
  p = _active_after_confirm()
  rig = Rig(card=True)

  def restart(r):   # plannerd comes back: a fresh mirror and SubMaster, its first tick before card's session arrives
    from openpilot.starpilot.car.mazda.sla.tests.rig import FakeSM
    r.planner.mirror = SpeedLimitAssistMirror()
    r.sm, r.session_msg = FakeSM(), None
  rows = run(rig, 4000, lambda i: 30 if i < 1000 else 25, presses=tap(p + 50), hooks={2500: restart})
  assert rig.session.arbiter.announce_counter >= 1
  assert _adjusting(rows, 2400) == []


def test_card_restart_does_not_announce():
  p = _active_after_confirm()
  rig = Rig(card=True)

  def restart(r):   # card comes back: a new session, counter 0 again
    r.session = MazdaSlaSession(CP)
  rows = run(rig, 4000, lambda i: 30 if i < 1000 else 25, presses=tap(p + 50), hooks={2500: restart})
  assert _adjusting(rows, 2400) == []


def test_selfdrived_restart_and_reengage_do_not_announce():
  p = _active_after_confirm()
  rig = Rig(card=True)

  def sd_restart(r):
    r.sd.mazda_sla_announce_seen = None
  rows = run(rig, 5000, lambda i: 30 if i < 1000 else 25, presses=tap(p + 50), hooks={2500: sd_restart},
             enabled=lambda i: not 3000 <= i < 3200)
  assert _adjusting(rows, 2400) == []


def test_real_confirm_still_announces_after_a_plannerd_restart():
  rig = Rig(card=True)
  rows0 = run(Rig(card=True), 4000, lambda i: 30 if i < 2500 else 25)
  p = first(rows0, lambda r: r.state == AssistState.preActive, 2500)

  def restart(r):
    from openpilot.starpilot.car.mazda.sla.tests.rig import FakeSM
    r.planner.mirror = SpeedLimitAssistMirror()
    r.sm, r.session_msg = FakeSM(), None
  rows = run(rig, 4000, lambda i: 30 if i < 2500 else 25, presses=tap(p + 50), hooks={1500: restart})
  assert [s[2] for s in _adjusting(rows, 1400)] == ['Adjusting to 30 mph']


# ---------------------------------------------------------------- StarPilot's saved per-speed-range offsets

def saved_offset_fn(offsets_mps):
  """The offset plannerd hands the resolver, from a saved SLCOffsetSchedule through StarPilot's own parser."""
  from openpilot.selfdrive.controls import plannerd
  from openpilot.starpilot.speed_limits import offset_document as od
  from openpilot.starpilot.speed_limits.runtime_settings import parse
  doc = {'version': 1, 'bounds_mps': list(od.IMPERIAL_BOUNDS), 'offsets_mps': offsets_mps}
  settings = parse({'SpeedLimitController': True, 'SLCConfirmation': 1, 'IsMetric': False, 'SLCOffsetSchedule': doc})
  assert not settings.errors
  return plannerd.mazda_sla_planner(CP, NS(get_bool=lambda k, **kw: False), NS(runtime=NS(settings=settings))).resolver.offset_fn


def test_two_offset_bands_agree_end_to_end():
  """Band 0 (< 11.2 m/s, 25 mph) +5 mph, band 1 (11.2-15.2 m/s, 30 mph) +3 mph. SET 36 on a 25 then a 30: the prompt, the
  press it asks for, "Adjusting to", the factory SET walk and the plan cap all name 30, then all name 33."""
  fn = saved_offset_fn([5 / MPH, 3 / MPH, 0., 0., 0., 0., 0.])
  assert round((11.176 + fn(11.176)) * MPH) == 30 and round((13.411 + fn(13.411)) * MPH) == 33
  limit = lambda i: 25 if i < 3000 else 30  # noqa: E731
  rig0 = Rig(offset=fn, card=True)
  rows0 = run(rig0, 6000, limit)
  p1 = first(rows0, lambda r: r.state == AssistState.preActive)
  rig1 = Rig(offset=fn, card=True)
  rows1 = run(rig1, 6000, limit, presses=tap(p1 + 50))
  p2 = first(rows1, lambda r: r.state == AssistState.preActive, 3000)
  rig = Rig(offset=fn, card=True)
  rows = run(rig, 6000, limit, presses={**tap(p1 + 50), **tap(p2 + 50, 'accelCruise')})
  texts = [s[2] for s in spans(rows)]
  assert texts == ['Press - for 30 mph', 'Adjusting to 30 mph', 'Press + for 33 mph', 'Adjusting to 33 mph']
  for start, mph_ in ((p1, 30), (p2, 33)):
    r = rows[start + 300]
    assert round(r.offer * MPH) == mph_ and setpoint(r) == mph_ and round(r.cap * MPH) == mph_
  assert rows[2900].dash == 30 and round(rows[2900].plan_cap * MPH) == 30
  assert rows[-1].dash == 33 and round(rows[-1].plan_cap * MPH) == 33


def test_one_long_walk_stops_at_the_ecu_minimum():
  """15 zone (offset off), car already at 16 mph, SET 25: the confirmed walk runs down in one go toward the 15 cap and must
  stop at the ECU's 30 km/h (19 mph) floor (reference: decreasing exits at v_cruise_min)."""
  limit = lambda i: 25 if i < 1000 else 15  # noqa: E731
  rows0 = run(Rig(offset=lambda lim: 0., card=True), 2000, limit, dash=25, v_mph=16.)
  p = first(rows0, lambda r: r.state == AssistState.preActive, 1000)
  rows = run(Rig(offset=lambda lim: 0., card=True), 3500, limit, dash=25, v_mph=16., presses=tap(p + 50))
  assert min(r.dash for r in rows) == 19
  assert all(r.send == 'none' for r in rows[-500:])


# ---------------------------------------------------------------- plannerd: only valid, current card and source data

def _planner():
  from openpilot.starpilot.car.mazda.sla.planner import MazdaSlaPlanner
  return MazdaSlaPlanner(CP, NS(get_bool=lambda k, **kw: False), lambda lim: lim * 0.2)


def _sm(now, *, session_age_ns=0, session_valid=True, dash_age_ns=0, dash_valid=True):
  from openpilot.starpilot.car.mazda.sla.tests.rig import FakeSM
  sm = FakeSM()
  sm.put('carState', NS(vEgo=13., vCruise=60.), now)
  sm.put('starpilotCarState', NS(mazdaSlaSession=NS(state=5, vCap=30 / MPH, announceCounter=3, setpointKph=48.3,
                                                     target=30 / MPH)), now - session_age_ns)
  sm.valid['starpilotCarState'] = session_valid
  sm.put('slcDashboardObservation', NS(status='valid', speedMps=20 / MPH), now - dash_age_ns)
  sm.valid['slcDashboardObservation'] = dash_valid
  for k in ('mapdOut', 'slcVisionObservation', 'gpsLocation', 'gpsLocationExternal'):
    sm.seen.setdefault(k, False)
  return sm


@pytest.mark.parametrize('age_s, valid, capped', [(0., True, True), (0.15, True, True), (0.25, True, False), (30., False, False),
                                                  (0., False, False), (30., True, False)])
def test_plannerd_caps_only_on_a_valid_current_card_session(age_s, valid, capped):
  now = 10**12
  p = _planner()
  cap = p.step(_sm(now, session_age_ns=int(age_s * 1e9), session_valid=valid), now)
  out = NS()
  p.fill(out)
  assert (cap is not None) is capped and out.sessionValid is capped
  if capped:
    assert round(cap * MPH) == 30


def test_plannerd_cap_expires_and_recovers():
  now = 10**12
  p = _planner()
  assert p.step(_sm(now), now) is not None
  assert p.step(_sm(now + 300_000_000, session_age_ns=300_000_000), now + 300_000_000) is None   # card silent 300 ms
  assert p.step(_sm(now + 400_000_000), now + 400_000_000) is not None                          # card back


@pytest.mark.parametrize('age_s, valid, used', [(0., True, True), (0.4, True, True), (0.6, True, False), (30., False, False),
                                                (0., False, False)])
def test_plannerd_dashboard_reading_only_when_valid_and_current(age_s, valid, used):
  from openpilot.starpilot.car.mazda.sla.planner import sources_from_sm
  now = 10**12
  src = sources_from_sm(_sm(now, dash_age_ns=int(age_s * 1e9), dash_valid=valid), now, 'gpsLocationExternal')
  assert (round(src.car * MPH) == 20) is used and (src.car == 0.) is (not used)


# ---------------------------------------------------------------- card: no offer is answered while plannerd's data is out

def _hidden_offer_case(press_at, hold=8, outage=(None, None), n=4500):
  rows0 = run(Rig(card=True), 2500, lambda i: 30 if i < 1000 else 25)
  p = first(rows0, lambda r: r.state == AssistState.preActive, 1000)
  o0, o1 = outage
  o0 = p + 100 if o0 is None else o0
  o1 = p + 400 if o1 is None else o1
  rig = Rig(card=True)
  rows = run(rig, n, lambda i: 30 if i < 1000 else 25, presses=tap(press_at(p, o0, o1), hold=hold), stall=lambda i: o0 <= i < o1)
  return rows, p, o0, o1


def test_press_during_outage_is_manual():
  """Offer "Press - for 30" open, plannerd's data expires, the driver taps SET-: a manual step (36 -> 35), no confirm, no cap,
  no "Adjusting to"; once fresh data has settled nothing is re-offered (the press was the driver's answer)."""
  rows, p, o0, o1 = _hidden_offer_case(lambda p, o0, o1: o0 + 100)
  at = o0 + 100
  assert rows[o0 + 30].state == AssistState.inactive                      # the offer closed when the data expired
  assert rows[at + 40].cap == 255. and setpoint(rows[at + 40]) == 35
  assert all(r.state != AssistState.active for r in rows[o0:o1])
  assert not [s for s in spans(rows) if s[2].startswith('Adjusting')]
  assert not [s for s in spans(rows[o1:]) if s[2].startswith('Press')]


def test_press_held_across_recovery_is_manual():
  """SET- pressed during the outage and released after plannerd is back: still a manual step, nothing confirmed."""
  rows, p, o0, o1 = _hidden_offer_case(lambda p, o0, o1: o1 - 20, hold=60)
  assert rows[o1 + 100].cap == 255. and rows[o1 + 100].state == AssistState.inactive
  assert setpoint(rows[o1 + 100]) == 35
  assert not [s for s in spans(rows) if s[2].startswith('Adjusting')]


def test_press_right_after_recovery_waits_for_fresh_input():
  """Back from the outage, the old offer is not answerable: a SET- 0.5 s after recovery (before fresh data has held 2 s) is
  manual; only after that is the offer asked again, on fresh data."""
  rows, p, o0, o1 = _hidden_offer_case(lambda p, o0, o1: o1 + 50)
  assert rows[o1 + 120].cap == 255. and setpoint(rows[o1 + 120]) == 35
  assert not [s for s in spans(rows) if s[2].startswith('Adjusting')]


def test_untouched_offer_is_asked_again_on_fresh_data():
  rows, p, o0, o1 = _hidden_offer_case(lambda p, o0, o1: 4400)
  again = [s for s in spans(rows) if s[2] == 'Press - for 30 mph' and s[0] > rows[o1].ts]
  assert len(again) == 1 and again[0][0] >= rows[o1].ts + 2.0
