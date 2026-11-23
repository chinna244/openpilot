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


def test_no_valid_source_passes_through():
  stz = SourceStabilizer()
  pick(stz, 0, vision=20)
  assert pick(stz, 100, vision='stale') is None


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
