"""Route 00000002--c9d55e7093 t=343-381: Vision 20 vs Maps 30 replaced the pending offer 20 times."""
import random

import pytest

from openpilot.starpilot.speed_limits import acceptance as acc, selection as sel
from openpilot.starpilot.speed_limits.stability import DROPOUT_HOLD_MAX_M, DROPOUT_SETTLE_NS, HOLD_NS, RAISE_HOLD_NS, SourceStabilizer

V, M, D, O = sel.Source.VISION, sel.Source.MAP, sel.Source.DASHBOARD, sel.Source.ONLINE
POLICY = sel.SelectionPolicy(sel.SelectionMode.ORDERED, (V, M, D), False)
MS = 1_000_000
MPH = 0.44704


def obs(**speeds):
  out = {s: acc.Observation(acc.ObservationKind.UNKNOWN) for s in sel.Source}
  for name, value in speeds.items():
    source = sel.Source(name)
    if value == 'stale':
      out[source] = acc.Observation(acc.ObservationKind.STALE)
    elif value is not None:
      out[source] = acc.Observation(acc.ObservationKind.VALID, acc.Candidate(
        source.value, acc.ObservationIdentity(acc.IdentityKind.PRODUCER_EPISODE, source.value), value * MPH))
  return out


def pick(stz, now_ms, **speeds):
  selection = sel.select_limit(stz.apply(obs(**speeds), POLICY, now_ms * MS), POLICY)
  return None if selection.observation.candidate is None else round(selection.observation.candidate.speed_mps / MPH)


def test_school_zone_flip_holds_vision_20():
  stz = SourceStabilizer()
  assert pick(stz, 0, vision=20, map=30) == 20
  for t in range(100, 1400, 100):  # camera stale 1.3 s, map still 30
    assert pick(stz, t, vision='stale', map=30) == 20
  assert pick(stz, 1500, vision=20, map=30) == 20


def test_hold_expires_after_a_real_gap():
  stz = SourceStabilizer()
  pick(stz, 0, vision=20, map=30)
  assert pick(stz, HOLD_NS // MS, vision='stale', map=30) == 20
  assert pick(stz, HOLD_NS // MS + 100, vision='stale', map=30) == 30


def test_lower_reading_from_another_source_applies_at_once():
  stz = SourceStabilizer()
  pick(stz, 0, vision=30)
  assert pick(stz, 100, vision='stale', map=20) == 20


def test_same_source_new_value_applies_at_once():
  stz = SourceStabilizer()
  pick(stz, 0, vision=20, map=30)
  assert pick(stz, 100, vision=25, map=30) == 25
  assert pick(stz, 200, vision=35, map=30) == 35


def test_map_or_dashboard_changes_keep_starpilot_semantics():
  """Only a camera reading is held: map/dashboard provider changes stay immediate (test_set_speed)."""
  stz = SourceStabilizer()
  assert pick(stz, 0, map=20) == 20
  assert pick(stz, 100, vision=30, map=20) == 30
  policy = sel.SelectionPolicy(sel.SelectionMode.ORDERED, (D, V, M), False)
  stz = SourceStabilizer()
  sel_at = lambda t, **kw: sel.select_limit(stz.apply(obs(**kw), policy, t * MS), policy).observation.candidate.speed_mps
  assert round(sel_at(0, dashboard=30, vision=35) / MPH) == 30
  assert round(sel_at(100, dashboard='stale', vision=35) / MPH) == 35


def test_preferred_camera_higher_than_committed_camera_is_same_source():
  stz = SourceStabilizer()
  pick(stz, 0, vision=20, map=30)
  assert pick(stz, 100, vision=30, map=30) == 30  # a new sign: same source, immediate


def test_highest_and_lowest_modes_are_untouched():
  policy = sel.SelectionPolicy(sel.SelectionMode.HIGHEST, (V, M, D), False)
  stz = SourceStabilizer()
  raw = obs(vision='stale', map=30)
  assert stz.apply(raw, policy, 0) == raw


def test_equal_limit_from_map_while_camera_is_briefly_stale_keeps_identity():
  stz = SourceStabilizer()
  first = sel.select_limit(stz.apply(obs(vision=30, map=30), POLICY, 0), POLICY).observation.candidate
  later = sel.select_limit(stz.apply(obs(vision='stale', map=30), POLICY, 100 * MS), POLICY).observation.candidate
  assert later == first and later.source == 'vision'


def test_no_valid_source_passes_through_before_anything_held():
  stz = SourceStabilizer()
  pick(stz, 0, vision=20)
  assert pick(stz, 100, vision='stale') is None


def test_short_higher_reading_gives_way_to_the_settled_one_in_a_dropout():
  stz = SourceStabilizer()
  settle(stz, 30)
  pick(stz, 3200, map=45)
  assert apply_v(stz, 3300, 10.) == 30


def test_short_lower_reading_stays_in_a_dropout():
  stz = SourceStabilizer()
  settle(stz, 30)
  pick(stz, 3200, map=20)
  assert apply_v(stz, 3300, 10.) == 20


def test_fallback_never_raises_or_replaces_the_pending_offer():
  stz, state = SourceStabilizer(), acc.new_session('s')
  auth = acc.Authority(acc.Mode.COMBINED, acc.LongitudinalOwner.SYSTEM, True, True, False, False)
  decisions = set()
  sequence = [dict(vision=20, map=30)] + [dict(vision='stale', map=30), dict(vision=20, map=30)] * 10
  for i, speeds in enumerate(sequence):
    o = stz.apply(obs(**speeds), POLICY, i * 400 * MS)
    state = acc.step(state, sel.select_limit(o, POLICY).observation, auth, acc.Policy(), now_ns=i * 400 * MS).state
    assert state.pending is not None and round(state.pending.candidate.speed_mps / MPH) == 20
    decisions.add(state.pending.decision_id)
  assert len(decisions) == 1


def test_third_source_cannot_bypass_the_camera_hold():
  """Review repro: priority Maps -> Dashboard -> Vision; Vision 20 goes stale, Maps 30 masked, Dashboard 35 won."""
  policy = sel.SelectionPolicy(sel.SelectionMode.ORDERED, (M, D, V), False)
  stz = SourceStabilizer()
  at = lambda t, **kw: sel.select_limit(stz.apply(obs(**kw), policy, t * MS), policy).observation.candidate
  assert round(at(0, vision=20).speed_mps / MPH) == 20
  held = at(100, vision='stale', map=30, dashboard=35)
  assert held.source == 'vision' and round(held.speed_mps / MPH) == 20
  assert round(at(200, vision='stale', map=30, dashboard=15).speed_mps / MPH) == 15  # a lower one still applies


def test_every_order_holds_the_camera_against_higher_fallbacks():
  import itertools
  for order in itertools.permutations((V, M, D)):
    policy = sel.SelectionPolicy(sel.SelectionMode.ORDERED, order, False)
    stz = SourceStabilizer()
    stz.apply(obs(vision=20), policy, 0)
    for t in range(100, 2900, 100):
      got = sel.select_limit(stz.apply(obs(vision='stale', map=30, dashboard=35), policy, t * MS), policy).observation.candidate
      assert round(got.speed_mps / MPH) == 20, (order, t)


def apply_v(stz, now_ms, v_ego, **speeds):
  selection = sel.select_limit(stz.apply(obs(**speeds), POLICY, now_ms * MS, v_ego=v_ego), POLICY)
  return None if selection.observation.candidate is None else round(selection.observation.candidate.speed_mps / MPH)


def settle(stz, value=30, **extra):
  for t in range(0, 3100, 100):
    pick(stz, t, map=value, **extra)


def test_settled_limit_holds_through_a_full_dropout():
  """Route 00000005--a37f13ed24 t=118.9-120.6: mapd 'unknown' and the camera gone together dropped the offer and asked
  again; filter-dev holds the last settled limit through the dropout."""
  stz = SourceStabilizer()
  settle(stz)
  for t in range(3200, 12000, 100):
    assert apply_v(stz, t, 10., map='stale', vision='stale') == 30


def test_a_cross_street_flash_is_not_held():
  stz = SourceStabilizer()
  settle(stz, 30)
  for t in range(3200, 5200, 100):
    pick(stz, t, map=45)  # 2 s cross street
  assert apply_v(stz, 5300, 10., map='stale') == 30  # the settled 30, not the flash


def test_any_reading_ends_the_hold():
  stz = SourceStabilizer()
  settle(stz)
  assert apply_v(stz, 3200, 10.) == 30
  assert apply_v(stz, 3300, 10., vision=25) == 25


def test_map_matched_road_without_a_limit_ends_the_hold():
  stz = SourceStabilizer()
  settle(stz)
  out = obs()
  out[M] = acc.Observation(acc.ObservationKind.ABSENT)
  assert sel.select_limit(stz.apply(out, POLICY, 3200 * MS, v_ego=10.), POLICY).observation.candidate is None


def test_backstop_distance_ends_the_hold():
  stz = SourceStabilizer()
  settle(stz)
  t, held = 3100, True
  while t < 3100 + 60_000:  # 25 m/s for 60 s = 1500 m
    t += 100
    held = apply_v(stz, t, 25.) == 30
    if not held:
      break
  assert not held and 3100 + 39_000 <= t <= 3100 + 41_000  # gives way at ~1000 m


def settle_vision(stz, value=25):
  for t in range(0, 3100, 100):
    pick(stz, t, vision=value)


def test_single_higher_camera_sighting_does_not_raise_a_settled_limit():
  """Route 00000005--a37f13ed24 t=117.97: one 60 mph sighting for 0.95 s in a 25 zone asked '+ for 70'."""
  stz = SourceStabilizer()
  settle_vision(stz, 25)
  for t in range(3200, 4100, 100):  # 0.9 s
    assert pick(stz, t, vision=60) == 25
  assert pick(stz, 4200, vision='stale') == 25


def test_higher_camera_reading_that_holds_applies():
  stz = SourceStabilizer()
  settle_vision(stz, 25)
  results = [pick(stz, t, vision=35) for t in range(3200, 4500, 100)]
  assert results[0] == 25 and results[-1] == 35


def test_lower_camera_reading_still_applies_at_once():
  stz = SourceStabilizer()
  settle_vision(stz, 35)
  assert pick(stz, 3200, vision=25) == 25


def test_a_discarded_higher_reading_never_returns_in_the_same_dropout():
  """Review of 3b5d2b1b0: settled 25, a 60 that held 1.2 s, then every source gone. The dropout falls back to 25 and
  stays there: time without a source is not time the 60 held."""
  stz = SourceStabilizer()
  settle_vision(stz, 25)
  for t in range(3200, 4500, 100):  # past the raise guard: 60 is committed but never settles
    pick(stz, t, vision=60)
  for t in range(4600, 12000, 100):
    assert apply_v(stz, t, 10.) == 25, t


def test_after_a_dropout_only_fresh_evidence_raises():
  stz = SourceStabilizer()
  settle_vision(stz, 25)
  for t in range(3200, 4500, 100):
    pick(stz, t, vision=60)
  for t in range(4600, 8000, 100):
    apply_v(stz, t, 10.)
  assert pick(stz, 8100, vision=60) == 25  # a fresh 60 must hold the raise guard again
  assert [pick(stz, t, vision=60) for t in range(8200, 9300, 100)][-1] == 60


def test_camera_raise_over_a_settled_map_limit_is_guarded():
  """Review of 3b5d2b1b0: settled map 25 plus one preferred Vision 60 sample selected 60 at once."""
  stz = SourceStabilizer()
  settle(stz, 25)
  assert pick(stz, 3200, vision=60, map=25) == 25
  assert pick(stz, 3300, vision='stale', map=25) == 25  # a single sample, then the camera goes stale
  assert pick(stz, 3400, vision='stale', map='stale') == 25  # and a full dropout keeps the settled map 25


def test_camera_raise_over_a_settled_map_limit_applies_once_it_holds():
  stz = SourceStabilizer()
  settle(stz, 25)
  results = [pick(stz, t, vision=60, map=25) for t in range(3200, 4400, 100)]
  assert results[0] == 25 and results[-1] == 60


def test_camera_raise_over_a_stale_settled_map_limit_holds_the_map_limit():
  stz = SourceStabilizer()
  settle(stz, 25)
  assert pick(stz, 3200, vision=60, map='stale') == 25


def test_backstop_marks_the_hold_expired_once():
  stz = SourceStabilizer()
  settle(stz)
  t, expired = 3100, []
  while t < 3100 + 45_000:
    t += 100
    apply_v(stz, t, 25.)
    expired.append(stz.expired)
  assert expired.count(True) == 1


def test_unconfigured_source_does_not_block_the_dropout_hold():
  """Review of 2dc4bb06d: a dashboard reading outside a Maps-only policy prevented the dropout hold."""
  policy = sel.SelectionPolicy(sel.SelectionMode.ORDERED, (M, None), False)
  stz = SourceStabilizer()
  at = lambda t, **kw: sel.select_limit(stz.apply(obs(**kw), policy, t * MS, v_ego=10.), policy).observation.candidate
  for t in range(0, 3100, 100):
    at(t, map=30)
  held = at(3200, map='stale', dashboard=45)
  assert held is not None and round(held.speed_mps / MPH) == 30


def test_restored_settled_source_keeps_its_own_freshness():
  """Codex review of e2240f6b1: Vision 25 settled, Maps 60 briefly selected, every source gone. The fallback restored
  Vision 25 with Maps' last-valid time, so a fresh Maps 45 was held off although the camera was stale for over 3 s."""
  stz = SourceStabilizer()
  settle_vision(stz, 25)
  for t in range(3200, 6300, 100):  # camera stale; Maps 60 takes over once the hold runs out
    pick(stz, t, vision='stale', map=60)
  assert pick(stz, 6200, vision='stale', map=60) == 60
  assert pick(stz, 6300, vision='stale', map='stale') == 25  # dropout: the short 60 gives way to the settled 25
  assert pick(stz, 6400, vision='stale', map=45) == 45  # the camera has been gone 3.3 s: no hold for it


def test_guarded_camera_raise_lets_a_lower_fallback_commit():
  """Codex review: Vision 60 under the raise guard plus a fresh Maps 20 showed 20 for one frame, then 25 in a dropout."""
  stz = SourceStabilizer()
  settle(stz, 25)
  assert pick(stz, 3200, vision=60, map=20) == 20
  assert pick(stz, 3300, vision='stale', map='stale') == 20


def test_guarded_camera_raise_never_replaces_an_explicit_no_limit():
  stz = SourceStabilizer()
  settle(stz, 25)
  out = obs(vision=60)
  out[M] = acc.Observation(acc.ObservationKind.ABSENT)
  assert sel.select_limit(stz.apply(out, POLICY, 3200 * MS), POLICY).observation.candidate is None
  # Codex review: the next all-stale frame resurrected the withdrawn map 25
  assert pick(stz, 3300, vision='stale', map='stale') is None


def test_guarded_camera_raise_over_its_own_settled_limit_still_holds_against_a_higher_map():
  stz = SourceStabilizer()
  settle_vision(stz, 25)
  assert pick(stz, 3200, vision=60, map=40) == 25


def test_backstop_bounds_a_dropout_that_never_settled():
  """Codex review: an accepted limit whose source vanished before settling kept its cap past 1 km."""
  stz = SourceStabilizer()
  for t in range(0, 1000, 100):
    pick(stz, t, map=30)  # 1 s: never settles, so nothing is held
  t, expired = 1000, []
  while t < 1000 + 45_000:
    t += 100
    apply_v(stz, t, 25.)
    expired.append(stz.expired)
  assert expired.count(True) == 1 and 39_000 <= (expired.index(True) + 1) * 100 <= 41_000


def test_a_guarded_camera_sample_does_not_restart_the_backstop():
  """Codex review: one rejected Vision 60 sample at 897.5 m reset the dropout budget, and the old 25 outlived 1 km."""
  stz = SourceStabilizer()
  settle_vision(stz, 25)
  t, expired, last = 3000, [], None
  while t < 3000 + 35_900:  # 897.5 m at 25 m/s
    t += 100
    last = apply_v(stz, t, 25.)
    expired.append(stz.expired)
  assert last == 25 and not any(expired)
  t += 100
  assert apply_v(stz, t, 25., vision=60) == 25  # guarded: not a usable reading
  while t < 3000 + 42_000:
    t += 100
    last = apply_v(stz, t, 25.)
    expired.append(stz.expired)
  assert last is None and expired.count(True) == 1


def test_a_camera_raise_stays_guarded_after_the_map_withdraws_its_limit():
  """Codex review: after a Maps no-limit withdrew the old 25, the still-present Vision 60 was selected after 100 ms."""
  stz = SourceStabilizer()
  settle(stz, 25)
  out = obs(vision=60)
  out[M] = acc.Observation(acc.ObservationKind.ABSENT)
  assert sel.select_limit(stz.apply(out, POLICY, 3200 * MS), POLICY).observation.candidate is None
  assert [pick(stz, t, vision=60, map='stale') for t in range(3300, 4200, 100)] == [None] * 9
  assert pick(stz, 4200, vision=60, map='stale') == 60  # held RAISE_HOLD_NS from its first sighting


def test_a_camera_raise_stays_guarded_across_the_backstop():
  stz = SourceStabilizer()
  settle_vision(stz, 25)
  t = 3000
  while t < 3000 + 39_600:  # 990 m at 25 m/s
    t += 100
    apply_v(stz, t, 25.)
  start, seen = t + 100, []
  while t < start + 1_500:
    t += 100
    seen.append((t - start, apply_v(stz, t, 25., vision=60), stz.expired))
  assert any(expired for _, _, expired in seen)  # the cutoff fell inside the guard window
  assert all(value != 60 for dt, value, _ in seen if dt < 1_000) and seen[-1][1] == 60


def test_a_rejected_higher_map_fallback_does_not_reset_the_backstop():
  """Codex review: retained Vision 25, guarded Vision 60 and Maps 40 kept 25 but reset the budget from 122.5 m."""
  stz = SourceStabilizer()
  settle_vision(stz, 25)
  t = 3000
  while t < 3000 + 4_900:  # 122.5 m
    t += 100
    apply_v(stz, t, 25.)
  budget = stz.dropout_m
  t += 100
  assert apply_v(stz, t, 25., vision=60, map=40) == 25
  assert stz.dropout_m >= budget


def test_a_changed_camera_value_after_withdrawal_gets_its_own_guard():
  """Codex review: Maps withdrew, Vision 60 was guarded, then Vision 55 after 100 ms was selected at once."""
  stz = SourceStabilizer()
  settle(stz, 25)
  out = obs(vision=60)
  out[M] = acc.Observation(acc.ObservationKind.ABSENT)
  assert sel.select_limit(stz.apply(out, POLICY, 3200 * MS), POLICY).observation.candidate is None
  assert [pick(stz, t, vision=55, map='stale') for t in range(3300, 4300, 100)] == [None] * 10
  assert pick(stz, 4300, vision=55, map='stale') == 55  # its own 1 s from 3300


def test_a_changed_camera_value_after_the_backstop_gets_its_own_guard():
  stz = SourceStabilizer()
  settle_vision(stz, 25)
  t = 3000
  while not stz.expired:
    t += 100
    apply_v(stz, t, 25.)
  t += 100
  assert apply_v(stz, t, 25., vision=60) is None
  start = t + 100
  values = []
  while t < start + 1_000:
    t += 100
    values.append(apply_v(stz, t, 25., vision=55))
  assert values[:-1] == [None] * (len(values) - 1) and values[-1] == 55


def test_a_held_reading_with_rejected_fallbacks_ends_at_the_backstop():
  """Codex review: from 997.5 m, Vision alternating 60/65 (guarded) and Maps 40 (rejected) kept the old 25 for another
  250 m: the retained-reading path stopped adding distance."""
  stz = SourceStabilizer()
  settle_vision(stz, 25)
  t = 3000
  while t < 3000 + 39_900:  # 997.5 m
    t += 100
    apply_v(stz, t, 25.)
  expired, values = [], []
  for i in range(100):  # 250 m more
    t += 100
    values.append(apply_v(stz, t, 25., vision=60 if i % 2 else 65, map=40))
    expired.append(stz.expired)
  crossed = expired.index(True)
  assert expired.count(True) == 1 and crossed <= 2
  assert 25 not in values[crossed:]


MAPS_FIRST = sel.SelectionPolicy(sel.SelectionMode.ORDERED, (M, V, D), False)


def pick_in(stz, policy, now_ms, **speeds):
  inputs = obs(**{k: v for k, v in speeds.items() if v != 'absent'})
  inputs.update({sel.Source(k): acc.Observation(acc.ObservationKind.ABSENT) for k, v in speeds.items() if v == 'absent'})
  out = stz.apply(inputs, policy, now_ms * MS, v_ego=25.)
  candidate = sel.select_limit(out, policy).observation.candidate
  return None if candidate is None else round(candidate.speed_mps / MPH)


def test_the_cameras_own_new_value_is_guarded_behind_an_equal_preferred_map():
  """Found by the randomized check: Maps preferred at an equal 20 kept the committed camera 20, and the camera's own
  new 40 replaced it at once through the hold rules, never reaching the raise guard."""
  stz = SourceStabilizer()
  for t in range(0, 3100, 100):
    pick_in(stz, MAPS_FIRST, t, vision=20)
  assert pick_in(stz, MAPS_FIRST, 3200, vision=20, map=20) == 20
  assert [pick_in(stz, MAPS_FIRST, t, vision=40, map=20) for t in range(3300, 4300, 100)] == [20] * 10


def test_a_no_limit_answer_withdraws_an_unsettled_reading_too():
  """Found by the randomized check: Maps 55 (not yet settled) withdrawn by a Maps no-limit came back behind a guarded
  camera reading."""
  stz = SourceStabilizer()
  settle(stz, 25)
  for t in range(3200, 3700, 100):
    pick_in(stz, POLICY, t, map=55)
  for t in range(3700, 4200, 100):
    assert pick_in(stz, POLICY, t, map='absent') is None
  assert pick_in(stz, POLICY, 4300, map='stale', vision=65) is None


def test_a_no_limit_answer_withdraws_that_sources_settled_reading():
  """Found by the randomized check: a settled Maps 20 came back in a later dropout although Maps had since answered
  no limit."""
  stz = SourceStabilizer()
  for t in range(0, 3100, 100):
    pick_in(stz, MAPS_FIRST, t, map=20)
  for t in range(3200, 5000, 100):
    pick_in(stz, MAPS_FIRST, t, map='absent', vision=40)
  assert pick_in(stz, MAPS_FIRST, 5100, map='stale', vision='stale') != 20


FUZZ_SPEEDS = (20, 25, 40, 55, 60, 65)


def fuzz_observation(source, value):
  if value in ('stale', 'absent', None):
    return acc.Observation({'stale': acc.ObservationKind.STALE, 'absent': acc.ObservationKind.ABSENT,
                            None: acc.ObservationKind.UNKNOWN}[value])
  return obs(**{source.value: value})[source]


def fuzz_sequence(seed, policy, frames=800):
  """Random camera/map/dashboard sequences at 25 m/s, checked frame by frame against the three rules."""
  rng = random.Random(seed)
  stz = SourceStabilizer()
  vision, map_value, dashboard = 25, 25, None
  since_fresh_m, absent_since_fresh = 0., False
  raise_value, raise_start, shown_before = None, 0, False
  # the raise rule's own model, from inputs and outputs only: the trusted speed is the shown limit once one shown limit
  # has held its speed DROPOUT_SETTLE_NS (fresh at the start of that run), then the last limit shown, kept while nothing is
  shown_speed, shown_since, trusted = None, 0, None
  for frame in range(frames):
    now = frame * 100 * MS
    if rng.random() < 0.08:
      vision = rng.choice((*FUZZ_SPEEDS, 'stale', None))
    if rng.random() < 0.03:
      map_value = rng.choice((*FUZZ_SPEEDS, 'stale', 'absent', None))
    if rng.random() < 0.01:
      dashboard = rng.choice((None, None, 40, 'stale'))
    inputs = {V: fuzz_observation(V, vision), M: fuzz_observation(M, map_value), D: fuzz_observation(D, dashboard),
              O: fuzz_observation(O, None)}
    # a camera raise starts when its raw value first exceeds the trusted speed (it may have been reported for a while)
    if vision in FUZZ_SPEEDS and trusted is not None and vision * MPH > trusted + 1e-6:
      if raise_value != vision:
        raise_value, raise_start = vision, now
    else:
      raise_value = None
    out = stz.apply(inputs, policy, now, v_ego=25.)
    final = sel.select_limit(out, policy).selected_source
    fresh = final is not None and inputs[final].kind is acc.ObservationKind.VALID and out[final] == inputs[final]
    speed = out[final].candidate.speed_mps if final is not None else None
    since_fresh_m = 0. if fresh else since_fresh_m + (2.5 if frame else 0.)
    absent_since_fresh = False if fresh else (absent_since_fresh or map_value == 'absent')
    context = (seed, policy.slots, frame)
    # 1 km backstop: no retained reading once 1 km has passed without a fresh selected one
    assert not (final is not None and not fresh and since_fresh_m >= DROPOUT_HOLD_MAX_M), context
    # a map no-limit answer withdraws the retained map reading until a fresh reading is selected
    assert not (final == M and not fresh and absent_since_fresh), context
    # raise guard: a camera value above the trusted limit counts only after it has been a raise for RAISE_HOLD_NS
    if fresh and final == V and trusted is not None and speed > trusted + 1e-6:
      assert raise_value is not None and now - raise_start >= RAISE_HOLD_NS, context
    # with every source silent, a held limit only continues one already shown: after a frame with nothing shown it never
    # reappears (the camera's flicker hold against a live fallback is separate: a source is reporting then)
    live = any(inputs[source].kind is acc.ObservationKind.VALID for source in policy.slots if source is not None)
    assert not (final is not None and not fresh and not live and not shown_before), context
    shown_before = final is not None
    if speed is not None:
      if speed != shown_speed:
        shown_speed, shown_since = speed, now
      if trusted is not None or now - shown_since >= DROPOUT_SETTLE_NS:
        trusted = speed
    else:
      shown_speed = None


@pytest.mark.parametrize('slots', [(V, M, D), (M, V, D), (M, V)])
def test_randomized_sequences_keep_the_guard_backstop_and_withdrawal_rules(slots):
  """Codex review rounds 2-4 each found one more edge; this checks the three rules on random sequences (a 1500-seed run
  found three more, fixed above)."""
  policy = sel.SelectionPolicy(sel.SelectionMode.ORDERED, slots, False)
  for seed in range(100):
    fuzz_sequence(seed, policy)


def test_a_fresh_lower_camera_reading_behind_a_masked_map_becomes_the_limit():
  """Codex review: Maps preferred, Vision settled 25; Vision 20 with Maps 40 showed 20 but kept 25 as the limit, so the
  next frame went back to 25 at once (a fresh 25 sample, or a total dropout)."""
  def setup():
    stz = SourceStabilizer()
    for t in range(0, 3100, 100):
      pick_in(stz, MAPS_FIRST, t, vision=25)
    assert pick_in(stz, MAPS_FIRST, 3200, vision=20, map=40) == 20
    return stz
  stz = setup()
  assert [pick_in(stz, MAPS_FIRST, t, vision=25, map=40) for t in range(3300, 4300, 100)] == [20] * 10  # guarded raise
  stz = setup()
  assert pick_in(stz, MAPS_FIRST, 3300, vision='stale', map='stale') == 20  # the dropout keeps 20


def test_the_preferred_challenger_keeps_its_timer_while_the_camera_reading_is_recorded():
  """Recording the camera's own fresh winner every frame must not restart a persisting preferred Maps reading."""
  stz = SourceStabilizer()
  for t in range(0, 3100, 100):
    pick_in(stz, MAPS_FIRST, t, vision=25)
  results = [pick_in(stz, MAPS_FIRST, t, vision=20, map=40) for t in range(3200, 5400, 100)]
  assert results[0] == 20 and results[-1] == 40  # Maps 40 persisted STABLE_NS


def test_a_limit_settles_only_while_it_is_shown():
  """Codex review: Vision 20 shown at 0 and 100 ms, nothing selected 200-3000 ms, then 20 came back at 3100 ms with
  every source stale: time with nothing shown counted toward settling."""
  stz = SourceStabilizer()
  assert pick(stz, 0, vision=20) == 20
  assert pick(stz, 100, vision=20) == 20
  for t in range(200, 3100, 100):
    assert pick(stz, t, vision='stale', map='stale') in (20, None)
  assert [pick(stz, t, vision='stale', map='stale') for t in range(3100, 3600, 100)] == [None] * 5
  assert stz.settled is None


def test_a_continuously_shown_hold_settles():
  stz = SourceStabilizer()
  for t in range(0, 1000, 100):
    assert pick(stz, t, vision=20, map=30) == 20
  for t in range(1000, 3100, 100):
    assert pick(stz, t, vision='stale', map=30) == 20  # held through the camera gap, shown every frame
  assert stz.settled is not None and round(stz.settled[1].candidate.speed_mps / MPH) == 20
