from functools import lru_cache

import cv2
import numpy as np

GLYPHS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"


def yellow_panels(frame):
  height, width = frame.shape[:2]
  top, bottom = int(height * 0.12), int(height * 0.65)
  hsv = cv2.cvtColor(frame[top:bottom], cv2.COLOR_BGR2HSV)
  mask = cv2.inRange(hsv, (14, 80, 55), (42, 255, 255))
  mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
  contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
  for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:12]:
    x, y, w, h = cv2.boundingRect(contour)
    if 0.020 * width <= w <= 0.17 * width and 0.046 * height <= h <= 0.40 * height and 0.49 <= w / h <= 0.95 and cv2.contourArea(contour) > 0.52 * w * h:
      yield x, y + top, w, h


def normalize(glyph):
  x, y, w, h = cv2.boundingRect(glyph)
  return cv2.resize(glyph[y : y + h, x : x + w], (14, 20), interpolation=cv2.INTER_AREA) / 255.0


@lru_cache(maxsize=1)
def glyph_templates():
  samples = []
  for letter in GLYPHS:
    variants = []
    for font in (cv2.FONT_HERSHEY_SIMPLEX, cv2.FONT_HERSHEY_DUPLEX):
      for thickness in (3, 5, 7):
        tile = np.zeros((48, 48), np.uint8)
        cv2.putText(tile, letter, (4, 38), font, 1.2, 255, thickness)
        variants.append(normalize(tile))
    samples.append(variants)
  return np.asarray(samples)


def ramp_header(crop):
  gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
  h, w = gray.shape
  header = gray[int(0.045 * h) : int(0.368 * h), int(0.08 * w) : int(0.92 * w)]
  header = cv2.resize(header, (int(100 * header.shape[1] / header.shape[0]), 100), interpolation=cv2.INTER_CUBIC)
  # Test two contrast levels because distant strokes can fade independently.
  low_contrast = cv2.adaptiveThreshold(header, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 45, 6)
  high_contrast = cv2.adaptiveThreshold(header, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 45, 10)
  otsu = cv2.threshold(header, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
  for mask in (high_contrast, otsu, low_contrast, cv2.erode(otsu, np.ones((3, 3), np.uint8))):
    _, _, boxes, _ = cv2.connectedComponentsWithStats(mask)
    boxes = sorted(
      (x, y, width, height) for x, y, width, height, area in boxes[1:] if 22 <= height <= 81 and 4 <= width <= 60 and area >= 60 and y > 0 and y + height < 100
    )
    if len(boxes) != 4:
      continue
    heights = [box[3] for box in boxes]
    y_positions = [box[1] for box in boxes]
    if max(heights) > 1.5 * min(heights) or np.ptp(y_positions) > 0.4 * np.median(heights):
      continue
    errors = np.stack([np.mean(np.abs(glyph_templates() - normalize(mask[y : y + h, x : x + w])), axis=(2, 3)).min(axis=1) for x, y, w, h in boxes])
    best = errors.min(axis=1)
    for word in ("EXIT", "RAMP"):
      target = errors[np.arange(4), [GLYPHS.index(letter) for letter in word]]
      # Permit one slightly blurred glyph when the other letters agree.
      if np.max(target) <= 0.33 and np.sum(target - best) <= 0.055 and np.count_nonzero(target - best > 0.005) <= 1:
        return True
  return False


class RampSignReader:
  def __init__(self):
    self.previous = None
    self.observation = None

  def update(self, frame, timestamp, classify):
    if self.observation is not None and 0 <= timestamp - self.observation[0] < 1_000_000_000:
      return
    frame_width = frame.shape[1]
    for x, y, w, h in yellow_panels(frame):
      relative_x = (x + w / 2) / frame_width
      if 0.42 <= relative_x <= 0.58:
        continue
      crop = frame[y : y + h, x : x + w]
      header = ramp_header(crop)
      previous = self.previous
      contiguous = (
        previous is not None
        and 0 < timestamp - previous[0] <= 850_000_000
        and abs(x - previous[3]) <= 1.8 * max(w, previous[4])
        and 0.65 * previous[4] <= w <= 2.15 * previous[4]
      )
      reading = classify(crop)
      if reading is None or reading[1] < 0.85 or not 10 <= reading[0] <= 75:
        continue
      mph, side = int(reading[0]), -1 if relative_x < 0.42 else 1
      if contiguous and mph == previous[1] and side == previous[2] and (header or previous[5]):
        self.observation = (timestamp, mph, side)
      self.previous = (timestamp, mph, side, x, w, header)
      break
