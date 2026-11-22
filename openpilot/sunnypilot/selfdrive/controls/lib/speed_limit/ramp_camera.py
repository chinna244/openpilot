"""Offline road-edge cue. An independently verified exit sign supplies the side."""

from bisect import bisect_right
from itertools import pairwise
import math

import numpy as np


def find_camera_split(frames, sign_time, side):
  if side not in (-1, 1):
    return None
  outside = 1 if side == 1 else 0
  inside = 1 - outside
  open_from = open_seen = narrow_from = -math.inf
  previous = -math.inf
  for t, lane_probs, edge_stds, tracks in frames:
    if t < sign_time or t > sign_time + 60.0:
      continue
    if t - previous > 0.55:
      open_from = narrow_from = -math.inf
    previous = t
    distances = (5.0, 18.0, 35.0)
    projected = [np.interp(distances, x, y) for x, y in tracks]
    path, _, lane_left, lane_right, _, road_left, road_right = projected
    edge_widths = (path - road_left, road_right - path)
    inner, outer = edge_widths[inside], edge_widths[outside]
    outer_valid = edge_stds[outside] <= 0.65 and np.all((outer >= 1.3) & (outer <= 6.0))
    if outer_valid and edge_stds[inside] >= 0.8 and np.min(inner) >= max(7.0, float(np.max(outer)) + 2.5):
      if open_from == -math.inf:
        open_from = t
      if t - open_from >= 0.8:
        open_seen = t
    else:
      open_from = -math.inf
    lane_width = lane_right[0] - lane_left[0]
    contained = lane_left[0] <= path[0] <= lane_right[0] and 2.6 <= lane_width <= 5.7 and min(lane_probs[1], lane_probs[2]) >= 0.5
    separated = outer_valid and edge_stds[inside] <= 0.65 and np.all((inner >= 1.3) & (inner <= 5.5)) and np.max(inner + outer) <= 10.5 and contained
    if not separated or t - open_seen > 6.5:
      narrow_from = -math.inf
    elif narrow_from == -math.inf:
      narrow_from = t
    elif t - narrow_from >= 0.7:
      return round(t, 2)
  return None


def wheel_distance(samples, start, end):
  if not samples or not 0.0 < end - start <= 45.0:
    return None
  times = [r[0] for r in samples]
  lo, hi = bisect_right(times, start) - 1, bisect_right(times, end)
  if lo < 0 or hi - lo < 2 or start - times[lo] > 0.40 or end - times[hi - 1] > 0.40:
    return None
  chosen = [(start, samples[lo][1]), *[(t, v) for t, v, _ in samples[lo + 1 : hi]], (end, samples[hi - 1][1])]
  if any(not (math.isfinite(v) and 0 <= v <= 55) for _, v in chosen):
    return None
  if any(b[0] - a[0] > 0.45 for a, b in pairwise(chosen)):
    return None
  return sum((b[0] - a[0]) * (a[1] + b[1]) / 2 for a, b in pairwise(chosen))


def split_at_bound_ramp(sign, bound, motion, frames):
  if bound is None:
    return None
  t = find_camera_split(frames, sign["last_at"], sign["side"])
  if t is None:
    return None
  traveled = wheel_distance(list(motion), bound.anchor_at, t)
  if traveled is None or not -12.0 <= traveled - bound.remaining_m <= 130.0:
    return None
  return t
