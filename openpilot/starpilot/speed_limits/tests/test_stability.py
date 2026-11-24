"""Route 00000002--c9d55e7093 t=343-381: Vision 20 vs Maps 30 replaced the pending offer 20 times."""
from openpilot.starpilot.speed_limits import acceptance as acc, selection as sel
from openpilot.starpilot.speed_limits.stability import HOLD_NS, SourceStabilizer

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
