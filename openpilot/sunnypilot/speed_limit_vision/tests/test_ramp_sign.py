import cv2
import numpy as np
import pytest

from openpilot.sunnypilot.speed_limit_vision.ramp_sign import RampSignReader


def frame(label, side, yellow=True):
  image = np.zeros((760, 1344, 3), dtype=np.uint8)
  x = 250 if side == -1 else 950
  background = (0, 220, 255) if yellow else (255, 255, 255)
  image[230:380, x : x + 107] = background
  cv2.putText(image, label, (x + 12, 268), cv2.FONT_HERSHEY_SIMPLEX, 0.86, (0, 0, 0), 2, cv2.LINE_AA)
  cv2.putText(image, "45", (x + 22, 337), cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 0, 0), 4, cv2.LINE_AA)
  return image


@pytest.mark.parametrize("label,side", [("EXIT", 1), ("RAMP", -1)])
def test_ramp_word_and_two_numeric_reads(label, side):
  reader = RampSignReader()
  image = frame(label, side)
  reader.update(image, 1_000_000_000, lambda _: (45, 0.98))
  assert reader.observation is None
  reader.update(image, 1_150_000_000, lambda _: (45, 0.98))
  assert reader.observation == (1_150_000_000, 45, side)


@pytest.mark.parametrize("label,yellow", [("SLOW", True), ("EXIT", False), ("", True)])
def test_numeric_read_is_not_enough(label, yellow):
  reader = RampSignReader()
  image = frame(label, 1, yellow)
  for t in (1_000_000_000, 1_150_000_000):
    reader.update(image, t, lambda _: (45, 0.98))
  assert reader.observation is None
