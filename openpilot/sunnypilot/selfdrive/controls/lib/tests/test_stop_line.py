import math

import numpy as np
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.stop_line import FOCAL, line_distances, row_distance

HEIGHT, PITCH = 1.339, 0.039  # the comma 4's extrinsicsCalibration on routes 0000002c and 0000002d


def row_at(dist: float, h: int = 760) -> float:
  return h / 2. + FOCAL * math.tan(math.atan(HEIGHT / dist) - PITCH)


def road(bands=(), stripes=(), shape=(760, 1344)) -> np.ndarray:
  img = np.full(shape, 80, np.uint8)
  img += np.random.default_rng(0).integers(0, 6, shape, dtype=np.uint8)
  scale = shape[0] / 760
  for d in bands:
    r = int(round(row_at(d) * scale))
    img[r - max(1, int(3 * scale)):r + max(2, int(3 * scale)), :] = 200
  for x in stripes:  # lane lines run along the road: a narrow bright column
    img[:, int(x * shape[1]):int(x * shape[1]) + 10] = 220
  return img


def test_row_distance_round_trips():
  for d in (6., 10., 20.):
    assert row_distance(row_at(d), 760, HEIGHT, PITCH) == pytest.approx(d, rel=1e-6)


@pytest.mark.parametrize("shape", [(760, 1344), (330, 526)])  # the live camera, and the dashcam's qcamera
def test_bands_across_the_lane_are_found_at_their_distance(shape):
  found = line_distances(road(bands=(8., 15.), shape=shape), HEIGHT, PITCH)
  assert len(found) == 2
  assert found[0] == pytest.approx(8., rel=0.08) and found[1] == pytest.approx(15., rel=0.12)


def test_lane_lines_and_a_plain_road_are_not_bands():
  assert line_distances(road(stripes=(0.35, 0.6)), HEIGHT, PITCH) == []
  assert line_distances(road(), HEIGHT, PITCH) == []


def test_bands_beyond_reach_are_dropped():
  assert line_distances(road(bands=(45.,)), HEIGHT, PITCH) == []
