"""Keep one speed-limit reading through short source gaps; never let a fallback raise it.

Route 00000002--c9d55e7093 t=343-381 (school zone, Vision 20 mph vs Maps 30 mph): the camera reading
went stale for 0.2-1.3 s between sign sightings, the ordered selection fell back to the map's 30 and
back to 20, and every flip replaced the pending offer (20 replacements, offers 25/36). The same flips
between equal map/vision 30 readings minted a new presentation each time, which can unbind a press
from the offer it answered.

Applied to the per-source observations before selection (pure, one instance per runtime session), in the
ordered selection mode and only while the committed reading is the camera's (Highest/Lowest compare by value
on purpose; map and dashboard changes keep their own semantics):
- a reading from the committed source always applies at once, including a new value;
- a LOWER reading from another source applies at once;
- a HIGHER or EQUAL reading from another source does not replace the committed one while the committed
  source is only briefly stale (HOLD_NS since it was last valid): the committed reading is held and the
  other source masked. An equal reading from a preferred source while the committed source is still
  valid keeps the committed one (same limit, stable identity); a higher one must persist STABLE_NS.

When no source is valid at all (route 00000005--a37f13ed24: mapd 'unknown' for 7-9 s at a time while the camera lost
its reading every second; the pending offer dropped at t=119.1 and was asked again at 120.6), the last reading that
held its value DROPOUT_SETTLE_NS is kept, as filter-dev's resolver holds the last settled limit (MAP_SETTLE_S 3 s, so a
cross street shown for 2-4 s is not held). It gives way to any valid reading, to the map's matched road without a
limit (ABSENT), and after DROPOUT_HOLD_MAX_M driven without one (filter-dev's backstop).
"""
from collections.abc import Mapping

from openpilot.starpilot.speed_limits.acceptance import Observation, ObservationKind
from openpilot.starpilot.speed_limits.selection import SelectionMode, SelectionPolicy, Source, select_limit

HOLD_NS = 3_000_000_000
STABLE_NS = 2_000_000_000
DROPOUT_SETTLE_NS = 3_000_000_000
DROPOUT_HOLD_MAX_M = 1000.
# filter-dev's SPEED_LIMIT_CHANGED_HOLD_PERIOD: a higher camera reading must hold this long before it replaces a settled
# limit (route 00000005--a37f13ed24 t=117.97 and 120.59: one 60 mph sighting, 0.95 s each, asked "+ for 70" in a 25 zone)
RAISE_HOLD_NS = 1_000_000_000


class SourceStabilizer:
  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.source: Source | None = None
    self.observation: Observation | None = None
    self.last_valid_ns = 0
    self.challenger: tuple[Source, float] | None = None
    self.challenger_since_ns = 0
    self.settled: tuple[Source, Observation] | None = None  # the last limit the FINAL selection showed DROPOUT_SETTLE_NS
    self.shown: tuple[float, int] | None = None  # (speed, since): the final selection's continuous run; None on an empty frame
    self.dropout_m = 0.
    self.last_now_ns: int | None = None
    self.raise_key: float | None = None
    self.raise_since_ns = 0
    # the speed a higher camera reading is judged against: the last committed limit once anything has settled. It outlives
    # the held reading (a map no-limit withdrawal, the backstop), so a camera raise is guarded until it qualifies
    self.trusted_mps: float | None = None
    self.expired = False  # this frame's dropout reached DROPOUT_HOLD_MAX_M: no carried limit bounds control any more
    self.valid_ns: dict[Source, int] = {}  # when each source last had a valid reading

  def _commit(self, source: Source, observation: Observation, now_ns: int) -> None:
    self.source, self.observation, self.last_valid_ns = source, observation, now_ns
    self.challenger, self.challenger_since_ns = None, 0
    if self.settled is not None or self.trusted_mps is not None:
      self.trusted_mps = observation.candidate.speed_mps

  def _dropout(self, out: dict, now_ns: int, v_ego: float | None, *, guarded: bool = False) -> dict:
    """No source valid: as filter-dev's last limit, keep the last reading; one that held less than DROPOUT_SETTLE_NS and
    is higher than the last settled one gives way to it (a lower short reading stays, it may be a real sign)."""
    if not guarded:
      self.raise_key = None  # no camera reading at all; a guarded one keeps its timer
    if any(out.get(source) is not None and out[source].kind is ObservationKind.ABSENT for source in (Source.MAP, self.source)):
      # the map (or the committed source) answered "no limit": a real answer, settled or not; it is never held again
      self.settled = self.observation = self.source = None
      return out
    if self.observation is None or self.source is None or self.settled is None:
      return out  # nothing has held yet: the gap passes through and acceptance keeps a pending offer
    short = self.settled is None or self.settled[1] != self.observation  # the committed reading has not settled
    if (short and self.settled is not None and
        self.observation.candidate.speed_mps > self.settled[1].candidate.speed_mps):
      # Decide once, at the dropout: time without any source is not time the short higher reading held, and only a
      # fresh reading may raise the settled limit again.
      self.source, self.observation = self.settled
      self.trusted_mps = self.observation.candidate.speed_mps
      self.last_valid_ns = self.valid_ns.get(self.source, 0)  # the restored source's own freshness, not the discarded one's
    out[self.source] = self.observation
    return out

  def _backstop(self, observations, out: dict, policy: SelectionPolicy, now_ns: int, v_ego: float | None) -> dict:
    """filter-dev's 1 km backstop, judged once per frame on the FINAL arbitration: a fresh reading of this frame resets
    it; a retained reading or no reading at all adds the distance driven. Crossing DROPOUT_HOLD_MAX_M (once) withdraws
    every retained reading on that same frame and marks it expired, so whatever cap was carried ends with it."""
    final = select_limit(out, policy).selected_source
    if final is not None and observations[final].kind is ObservationKind.VALID and out[final] == observations[final]:
      self.dropout_m = 0.
      return out
    dt = (now_ns - self.last_now_ns) / 1e9 if self.last_now_ns is not None and now_ns > self.last_now_ns else 0.
    before = self.dropout_m
    self.dropout_m += max(0., v_ego or 0.) * min(dt, 1.)
    if not before < DROPOUT_HOLD_MAX_M <= self.dropout_m:
      return out
    self.settled = self.observation = self.source = None
    self.expired = True
    stripped = {}
    for source, observation in out.items():
      if observation != observations[source] and observation is not None and observation.kind is ObservationKind.VALID:
        # a retained reading: back to what the source reported (a guarded camera reading stays masked)
        observation = observations[source] if observations[source].kind is not ObservationKind.VALID else Observation(ObservationKind.UNKNOWN)
      stripped[source] = observation
    return stripped

  def _settle(self, out: dict, policy: SelectionPolicy, now_ns: int) -> None:
    """A limit settles once the FINAL selection has shown its speed continuously for DROPOUT_SETTLE_NS, fresh or held
    through a short camera gap. Any frame showing no limit restarts the run: time with nothing shown never counts."""
    final = select_limit(out, policy).selected_source
    if final is None:
      self.shown = None
      return
    speed = out[final].candidate.speed_mps
    if self.shown is None or self.shown[0] != speed:
      self.shown = (speed, now_ns)
    if (now_ns - self.shown[1] >= DROPOUT_SETTLE_NS and self.observation is not None and self.source is not None and
        self.observation.candidate.speed_mps == speed and (self.settled is None or self.settled[1] != self.observation)):
      self.settled = (self.source, self.observation)
      self.trusted_mps = speed

  def apply(self, observations: Mapping[Source, Observation], policy: SelectionPolicy, now_ns: int,
            v_ego: float | None = None) -> dict:
    try:
      out = self._apply(observations, policy, now_ns, v_ego)
      if isinstance(policy, SelectionPolicy) and policy.mode is SelectionMode.ORDERED:
        out = self._backstop(observations, out, policy, now_ns, v_ego)
        self._settle(out, policy, now_ns)
      return out
    finally:
      self.last_now_ns = now_ns

  def _apply(self, observations, policy, now_ns, v_ego):
    self.expired = False
    out = dict(observations)
    if not isinstance(policy, SelectionPolicy) or policy.mode is not SelectionMode.ORDERED:
      self.reset()  # Highest/Lowest compare sources by value on purpose
      return out
    for source, observation in out.items():
      if observation is not None and observation.kind is ObservationKind.VALID:
        self.valid_ns[source] = now_ns
    if self.settled is not None and out.get(self.settled[0]) is not None and out[self.settled[0]].kind is ObservationKind.ABSENT:
      self.settled = None  # that source has since answered "no limit": its old settled reading is withdrawn, never restored
    # Whichever source the trusted limit came from, a camera reading above it must hold RAISE_HOLD_NS before it counts as
    # evidence at all (selected, held against, or replacing the camera's own committed value), each different value on its
    # own timer, even when the held reading was withdrawn or expired meanwhile. Until then the camera is judged silent.
    camera = out.get(Source.VISION)
    value = camera.candidate.speed_mps if camera is not None and camera.kind is ObservationKind.VALID else None
    guarded = False
    if value is not None and self.trusted_mps is not None and value > self.trusted_mps:
      if self.raise_key != value:
        self.raise_key, self.raise_since_ns = value, now_ns
      if now_ns - self.raise_since_ns < RAISE_HOLD_NS:
        guarded = True
        out[Source.VISION] = Observation(ObservationKind.UNKNOWN)
        if self.source == Source.VISION:
          self.last_valid_ns = now_ns  # the camera is still reading signs: its committed limit is not stale
    else:
      self.raise_key = None
    raw = select_limit(out, policy)
    selected = raw.selected_source
    if selected is None:
      self.challenger, self.challenger_since_ns = None, 0
      # Source loss is judged on the sources this policy selects from: an unconfigured one does not count. A guarded
      # camera reading is not a usable one either: the same dropout rules apply, only its raise timer runs on.
      return self._dropout(out, now_ns, v_ego, guarded=guarded)
    if self.source is None or self.observation is None or selected == self.source:
      self._commit(selected, raw.observation, now_ns)
      return out
    if self.source is not Source.VISION:
      # only camera readings flicker between sightings; map/dashboard changes keep their own semantics
      self._commit(selected, raw.observation, now_ns)
      return out

    own = out.get(self.source)
    own_valid = own is not None and own.kind is ObservationKind.VALID
    if own_valid:
      self.last_valid_ns = now_ns
    committed = self.observation.candidate.speed_mps
    held = not own_valid and now_ns - self.last_valid_ns <= HOLD_NS
    # Judge the FINAL selection: masking one source must not let a third one at or above the held limit win.
    challenged = False
    for _ in range(len(Source)):
      raw = select_limit(out, policy)
      selected = raw.selected_source
      if selected is None or selected == self.source or raw.observation.kind is not ObservationKind.VALID:
        break
      candidate = raw.observation.candidate
      if candidate.speed_mps < committed:
        self._commit(selected, raw.observation, now_ns)  # lower applies at once
        return out
      if not own_valid and not held:
        self._commit(selected, raw.observation, now_ns)  # camera gone past the hold
        return out
      if own_valid and candidate.speed_mps > committed and not challenged:
        challenged = True
        key = (selected, candidate.speed_mps)
        if self.challenger != key:
          self.challenger, self.challenger_since_ns = key, now_ns
        if now_ns - self.challenger_since_ns >= STABLE_NS:
          self._commit(selected, raw.observation, now_ns)  # a preferred source that persisted
          return out
      out[selected] = Observation(ObservationKind.UNKNOWN)
    if not challenged:
      self.challenger, self.challenger_since_ns = None, 0
    if own_valid and select_limit(out, policy).selected_source == self.source:
      # the camera's own fresh reading won the final arbitration (a preferred source masked): it is the limit now, for
      # the raise guard and a later dropout alike; a preferred challenger keeps its timer
      challenger, since = self.challenger, self.challenger_since_ns
      self._commit(self.source, own, now_ns)
      self.challenger, self.challenger_since_ns = challenger, since
    if held:
      out[self.source] = self.observation  # nothing lower: the camera's reading stands through the gap
    return out
