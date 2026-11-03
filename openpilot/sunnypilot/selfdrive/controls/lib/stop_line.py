"""
Stop line: the painted band across the lane ahead (a stop bar or a crosswalk's edge), found in the road camera's Y plane.

A GPS spot is only good to a few metres from one day to the next: the line at the southbound stop sign
42.97205,-87.88467 came out 6 m apart on routes 0000002c and 0000002d, the northbound one 7.5 m apart, with the
camera measuring the same line each time. So the stop is placed by the camera on the approach, and GPS only says
which stop sign is coming.

Each row of a patch of road ahead is averaged across the lane; a row brighter than its neighbourhood by BAND_CONTRAST,
with BAND_COVER of its pixels that bright, is a band running across the lane, not a lane line along it. Its distance
on flat ground comes from the camera's height and pitch (extrinsicsCalibration) and focal length: the row of a band
is the angle below the optical axis, plus the pitch, below the horizon. The hood hides the road closer than about
5.4 m; the ROI starts above it.
"""
import math

import numpy as np

# os04c10 road camera (comma 4), as openpilot.common.transformations.camera: 1344 x 760 at 1141.5 px
FOCAL = 1141.5
FULL_HEIGHT = 760
ROI_X = (0.285, 0.722)  # share of the width: the lane ahead
ROI_Y = (0.364, 0.82)  # share of the height: from about 30 m out to just above the hood
BAND_CONTRAST = 10.  # grey levels over the row's neighbourhood
BAND_COVER = 0.5  # share of the row's pixels that bright: across the lane, not along it
BAND_WINDOW = 21  # rows either side for the neighbourhood median
MAX_DIST = 30.  # m
MIN_ANGLE = 0.01  # rad below the horizon; anything flatter is too far to place


def band_rows(y_plane: np.ndarray, window: int = BAND_WINDOW) -> list[int]:
  """Rows of the bands across the lane, nearest first."""
  h, w = y_plane.shape
  r0, r1 = int(h * ROI_Y[0]), int(h * ROI_Y[1])
  roi = y_plane[r0:r1, int(w * ROI_X[0]):int(w * ROI_X[1])].astype(np.float32)
  prof = roi.mean(axis=1)
  pad = np.pad(prof, window, mode='edge')
  windows = np.lib.stride_tricks.sliding_window_view(pad, 2 * window + 1)
  base = np.median(windows, axis=1)
  rows: list[int] = []
  for i in range(1, len(prof) - 1):
    if prof[i] - base[i] > BAND_CONTRAST and prof[i] >= prof[i - 1] and prof[i] >= prof[i + 1]:
      if np.mean(roi[i] > base[i] + BAND_CONTRAST) > BAND_COVER:
        if rows and i + r0 - rows[-1] <= window // 2:
          # the same painted band, a few rows thick: keep its brightest row
          if prof[i] > prof[rows[-1] - r0]:
            rows[-1] = i + r0
        else:
          rows.append(i + r0)
  return rows[::-1]


def row_distance(row: float, height: int, cam_height: float, pitch: float) -> float:
  """m ahead on flat ground of an image row, for a camera cam_height up and pitched down by pitch rad."""
  angle = math.atan((row - height / 2.) / FOCAL) + pitch
  return cam_height / math.tan(angle) if angle > MIN_ANGLE else math.inf


def line_distances(y_plane: np.ndarray, cam_height: float, pitch: float, full_height: int = FULL_HEIGHT) -> list[float]:
  """Distances of the bands across the lane ahead, nearest first, within MAX_DIST. y_plane may be the camera's
  full frame or a scaled one of it (the dashcam's qcamera, 526 x 330 of 1344 x 760): rows scale to full_height."""
  scale = full_height / y_plane.shape[0]
  rows = band_rows(y_plane, max(3, round(BAND_WINDOW / scale)))
  return [d for d in (row_distance(r * scale, full_height, cam_height, pitch) for r in rows) if d <= MAX_DIST]
