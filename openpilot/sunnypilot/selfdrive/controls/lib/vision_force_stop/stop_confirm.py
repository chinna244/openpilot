"""Fresh camera STOP evidence and bounded confirmation (StarPilot starpilot/speed_limits/vision/stop_observation.py, Dom 1d30dc3).

The evidence comes from the sign reader's customReserved10 instead of StarPilot's slcVisionObservation: one reader, one
road stream, so the session/stream/model identity checks reduce to constants, and the frame age is checked on the
monotonic receipt time only.
"""
from dataclasses import dataclass
import math

from openpilot.common.transformations.camera import DEVICE_CAMERAS

STOP_MAX_AGE_NS = 1_500_000_000
# fork: StarPilot's 750 ms counts from the camera frame and assumes its result lands ~0.25 s later; our reader's land
# 0.3-0.9 s later while driving (routes 4a-4c), leaving 0-0.45 s. 1.2 s keeps StarPilot's usable time. Replay on routes
# 18-2a (33 image-checked STOPs) at the busy cadence: 750 ms + follow-up 6/33, 1.2 s + follow-up 14/33, 0 false either way.
CONTROL_STOP_MAX_AGE_NS = 1_200_000_000
RECEIPT_MAX_AGE_NS = 1_000_000_000
# fork: StarPilot 0.7 missed backlit STOPs (routes 5b/68, same intersection: hits 0.68/0.80/0.79/0.66 never spanned 0.5 s).
# The reader publishes boxes >= 0.6 only; live replay of routes 45-69 at 0.6: +6 confirmations, all image-checked real STOPs.
STOP_CONFIRM_CONFIDENCE = .6
STOP_SIGN_HEIGHT_M = .762
# Fitted on 17 confirmed sign approaches: where the car stops relative to the box range. (StarPilot 2a854471f)
STOP_RANGE_OFFSET_M = 2.0
ROAD_FOCAL_PX = {camera.narrow_road.size: camera.narrow_road.focal_length for camera in DEVICE_CAMERAS.values()}


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


def stop_distance(sample: StopEvidence, box: StopBox) -> float | None:
  focal = ROAD_FOCAL_PX.get((sample.width, sample.height)) if sample.stream == 'road' else None
  x1, y1, x2, y2 = box.bbox
  if not focal or y2 <= y1:
    return None
  across = ((x1 + x2) / 2 - sample.width / 2) / focal
  up = ((y1 + y2) / 2 - sample.height / 2) / focal
  return focal * STOP_SIGN_HEIGHT_M / (y2 - y1) / math.sqrt(1 + across ** 2 + up ** 2) + STOP_RANGE_OFFSET_M


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
    self.distance_m: float | None = None
    self._rejected = False

  def observe(self, sample: StopEvidence | None, *, now_ns: int, model_agrees: bool = False) -> int:
    """model_agrees: the driving model plans to slow within SIGN_MODEL_AGREE_M. fork: two independent sources agreeing
    is the evidence StarPilot's second camera hit stands for. At 18-26 mph the sign is first seen 15-25 m out and the
    0.5 s second-hit wait cost 5-8 m (home junction, routes 51-6c); acting on the first hit there commits ~5 m earlier
    (replay of routes 45-6c: braking needed 1.59 -> 1.39 m/s2 median, the same 2 non-STOP episodes)."""
    self._rejected = False
    confirmed = self._observe(sample, now_ns=now_ns)
    if not confirmed and model_agrees and not self._rejected and sample is not None and 0 <= now_ns - sample.observed_ns < STOP_MAX_AGE_NS:
      candidates = [box for box in sample.boxes if box.confidence >= STOP_CONFIRM_CONFIDENCE]
      if candidates:
        self.confirmed_ns = sample.observed_ns
        self.distance_m = stop_distance(sample, max(candidates, key=lambda box: box.confidence))
        confirmed = self.confirmed_ns
    return confirmed

  def _observe(self, sample: StopEvidence | None, *, now_ns: int) -> int:
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
        self._rejected = True
        return 0
      return self.confirmed_ns
    if previous is not None and (sample.observed_ns <= previous.observed_ns or sample.frame_id <= previous.frame_id):
      self.reset()
      self._rejected = True
      return 0
    candidates = tuple(box for box in sample.boxes if box.confidence >= STOP_CONFIRM_CONFIDENCE)
    matched = tuple(new for new in candidates if previous is not None and
                    100_000_000 <= sample.observed_ns - previous.observed_ns < STOP_MAX_AGE_NS and
                    any(_matches(old, new) for old in previous.boxes if old.confidence >= STOP_CONFIRM_CONFIDENCE))
    if not candidates:
      self.first_seen_ns = 0
    elif not matched:
      self.first_seen_ns = sample.observed_ns
    self.previous = sample
    self.confirmed_ns = (sample.observed_ns if matched and
                         sample.observed_ns - self.first_seen_ns >= 500_000_000 else 0)
    self.distance_m = stop_distance(sample, max(matched, key=lambda box: box.confidence)) if self.confirmed_ns else None
    return self.confirmed_ns
