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
Gaps where no source is valid pass through unchanged: acceptance keeps a pending offer through them.
"""
from collections.abc import Mapping

from openpilot.starpilot.speed_limits.acceptance import Observation, ObservationKind
from openpilot.starpilot.speed_limits.selection import SelectionMode, SelectionPolicy, Source, select_limit

HOLD_NS = 3_000_000_000
STABLE_NS = 2_000_000_000


class SourceStabilizer:
  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.source: Source | None = None
    self.observation: Observation | None = None
    self.last_valid_ns = 0
    self.challenger: tuple[Source, float] | None = None
    self.challenger_since_ns = 0

  def _commit(self, source: Source, observation: Observation, now_ns: int) -> None:
    self.source, self.observation, self.last_valid_ns = source, observation, now_ns
    self.challenger, self.challenger_since_ns = None, 0

  def apply(self, observations: Mapping[Source, Observation], policy: SelectionPolicy, now_ns: int) -> dict:
    out = dict(observations)
    if not isinstance(policy, SelectionPolicy) or policy.mode is not SelectionMode.ORDERED:
      self.reset()  # Highest/Lowest compare sources by value on purpose
      return out
    raw = select_limit(out, policy)
    selected = raw.selected_source
    if selected is None or raw.observation.kind is not ObservationKind.VALID:
      self.challenger, self.challenger_since_ns = None, 0
      return out
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
