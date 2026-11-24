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
    self.value_since_ns = 0  # since when the committed reading has had its value
    self.settled: tuple[Source, Observation] | None = None  # the last reading that held its value DROPOUT_SETTLE_NS
    self.dropout_m = 0.
    self.last_now_ns: int | None = None
    self.raise_key: float | None = None
    self.raise_since_ns = 0
    self.expired = False  # this frame's dropout hold reached DROPOUT_HOLD_MAX_M: the held limit no longer bounds control

  def _commit(self, source: Source, observation: Observation, now_ns: int) -> None:
    previous = self.observation.candidate.speed_mps if self.observation is not None and self.observation.candidate else None
    if previous != (observation.candidate.speed_mps if observation.candidate else None):
      self.value_since_ns = now_ns
    self.source, self.observation, self.last_valid_ns = source, observation, now_ns
    self.challenger, self.challenger_since_ns = None, 0
    if now_ns - self.value_since_ns >= DROPOUT_SETTLE_NS:
      self.settled = (source, observation)

  def _dropout(self, out: dict, now_ns: int, v_ego: float | None) -> dict:
    """No source valid: as filter-dev's last limit, keep the last reading; one that held less than DROPOUT_SETTLE_NS and
    is higher than the last settled one gives way to it (a lower short reading stays, it may be a real sign)."""
    self.raise_key = None
    if self.observation is None or self.source is None or self.settled is None:
      return out  # nothing has held yet: the gap passes through and acceptance keeps a pending offer
    if out.get(Source.MAP) is not None and out[Source.MAP].kind is ObservationKind.ABSENT:
      self.settled = self.observation = self.source = None  # the map matched a road without a limit: a real answer
      return out
    dt = (now_ns - self.last_now_ns) / 1e9 if self.last_now_ns is not None and now_ns > self.last_now_ns else 0.
    self.dropout_m += max(0., v_ego or 0.) * min(dt, 1.)
    if self.dropout_m >= DROPOUT_HOLD_MAX_M:
      self.settled = self.observation = self.source = None
      self.expired = True
      return out
    short = now_ns - self.value_since_ns < DROPOUT_SETTLE_NS
    if (short and self.settled is not None and
        self.observation.candidate.speed_mps > self.settled[1].candidate.speed_mps):
      # Decide once, at the dropout: time without any source is not time the short higher reading held, and only a
      # fresh reading may raise the settled limit again.
      self.source, self.observation = self.settled
      self.value_since_ns = now_ns - DROPOUT_SETTLE_NS
    out[self.source] = self.observation
    return out

  def apply(self, observations: Mapping[Source, Observation], policy: SelectionPolicy, now_ns: int,
            v_ego: float | None = None) -> dict:
    try:
      return self._apply(observations, policy, now_ns, v_ego)
    finally:
      self.last_now_ns = now_ns

  def _apply(self, observations, policy, now_ns, v_ego):
    self.expired = False
    out = dict(observations)
    if not isinstance(policy, SelectionPolicy) or policy.mode is not SelectionMode.ORDERED:
      self.reset()  # Highest/Lowest compare sources by value on purpose
      return out
    raw = select_limit(out, policy)
    selected = raw.selected_source
    if selected is None:
      # Source loss is judged on the sources this policy selects from: an unconfigured one does not count.
      self.challenger, self.challenger_since_ns = None, 0
      return self._dropout(out, now_ns, v_ego)
    self.dropout_m = 0.
    if (selected == Source.VISION and self.observation is not None and self.settled is not None and
        raw.observation.candidate.speed_mps > self.observation.candidate.speed_mps):
      # Whichever source the settled limit came from, a higher camera reading must hold RAISE_HOLD_NS first.
      value = raw.observation.candidate.speed_mps
      if self.raise_key != value:
        self.raise_key, self.raise_since_ns = value, now_ns
      if now_ns - self.raise_since_ns < RAISE_HOLD_NS:
        if self.source == Source.VISION:
          self.last_valid_ns = now_ns
          out[selected] = self.observation  # the settled limit stands until the higher reading has held
        else:
          out[selected] = Observation(ObservationKind.UNKNOWN)
          own = out.get(self.source)
          if own is None or own.kind is not ObservationKind.VALID:
            out[self.source] = self.observation
        return out
    else:
      self.raise_key = None
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
    if held:
      out[self.source] = self.observation  # nothing lower: the camera's reading stands through the gap
    return out
