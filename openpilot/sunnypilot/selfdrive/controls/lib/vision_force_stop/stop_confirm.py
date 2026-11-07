"""Fresh camera STOP evidence and bounded confirmation (StarPilot starpilot/speed_limits/vision/stop_observation.py, Dom 1d30dc3).

The evidence comes from the sign reader's customReserved10 instead of StarPilot's slcVisionObservation: one reader, one
road stream, so the session/stream/model identity checks reduce to constants, and the frame age is checked on the
monotonic receipt time only.
"""
from dataclasses import dataclass
import math

STOP_MAX_AGE_NS = 1_500_000_000
CONTROL_STOP_MAX_AGE_NS = 750_000_000
RECEIPT_MAX_AGE_NS = 1_000_000_000
STOP_CONFIRM_CONFIDENCE = .7


@dataclass(frozen=True)
class StopBox:
  confidence: float
  bbox: tuple[int, int, int, int]


@dataclass(frozen=True)
class StopEvidence:
  session: str
  stream: str
  observed_ns: int
  eof_boot_ns: int
  frame_id: int
  width: int
  height: int
  boxes: tuple[StopBox, ...]


def read_stop_evidence(sm, *, now_ns: int, drive_id: int) -> StopEvidence | None:
  try:
    service = 'customReserved10'
    if drive_id <= 0 or not sm.seen[service]:
      return None
    published = int(sm.logMonoTime[service])
    if not (drive_id < published <= now_ns and now_ns - published <= RECEIPT_MAX_AGE_NS):
      return None
    sample = sm[service]
    observed, eof, frame = int(sample.stopObservedMonoTime), int(sample.stopFrameEofTime), int(sample.stopFrameId)
    width, height = int(sample.stopImageWidth), int(sample.stopImageHeight)
    confidences, coords = list(sample.stopConfidences), list(sample.stopBoxes)
    if (not sample.stopValid or not drive_id < observed <= published or not 0 <= now_ns - observed < STOP_MAX_AGE_NS or
        not 1 <= width <= 8192 or not 1 <= height <= 8192 or len(confidences) > 4 or len(coords) != 4 * len(confidences)):
      return None
    boxes = []
    for i, confidence in enumerate(confidences):
      confidence = float(confidence)
      box = tuple(int(v) for v in coords[4 * i:4 * i + 4])
      if not math.isfinite(confidence) or not .6 <= confidence <= 1 or not (0 <= box[0] < box[2] <= width and 0 <= box[1] < box[3] <= height):
        return None
      boxes.append(StopBox(confidence, box))
    return StopEvidence('reader', 'road', observed, eof, frame, width, height, tuple(boxes))
  except (AttributeError, KeyError, TypeError, ValueError, OverflowError, RuntimeError):
    return None


def _matches(a: StopBox, b: StopBox) -> bool:
  ax, ay, ar, ab = a.bbox
  bx, by, br, bb = b.bbox
  aw, ah, bw, bh = ar - ax, ab - ay, br - bx, bb - by
  return (0.5 <= bw * bh / (aw * ah) <= 4 and
          abs(bx + br - ax - ar) <= 6 * max(aw, bw) and
          abs(by + bb - ay - ab) <= 3 * max(ah, bh))


class StopConfirmation:
  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.previous: StopEvidence | None = None
    self.confirmed_ns = 0
    self.first_seen_ns = 0

  def observe(self, sample: StopEvidence | None, *, now_ns: int) -> int:
    if sample is None or not 0 <= now_ns - sample.observed_ns < STOP_MAX_AGE_NS:
      self.reset()
      return 0
    previous = self.previous
    if previous is not None and (sample.session, sample.stream, sample.width, sample.height) != (
        previous.session, previous.stream, previous.width, previous.height):
      self.reset()
      previous = None
    if previous is not None and sample.observed_ns == previous.observed_ns and sample.frame_id == previous.frame_id:
      if sample != previous:
        self.reset()
        return 0
      return self.confirmed_ns
    if previous is not None and (sample.observed_ns <= previous.observed_ns or sample.frame_id <= previous.frame_id):
      self.reset()
      return 0
    candidates = tuple(box for box in sample.boxes if box.confidence >= STOP_CONFIRM_CONFIDENCE)
    coherent = (previous is not None and 100_000_000 <= sample.observed_ns - previous.observed_ns < STOP_MAX_AGE_NS and
                any(_matches(old, new) for old in previous.boxes if old.confidence >= STOP_CONFIRM_CONFIDENCE for new in candidates))
    if not candidates:
      self.first_seen_ns = 0
    elif not coherent:
      self.first_seen_ns = sample.observed_ns
    self.previous = sample
    self.confirmed_ns = (sample.observed_ns if candidates and coherent and
                         sample.observed_ns - self.first_seen_ns >= 500_000_000 else 0)
    return self.confirmed_ns
