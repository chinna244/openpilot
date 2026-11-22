"""Assign a tracked sign to a unique upcoming same-side branch before it is taken."""

from collections import defaultdict
from dataclasses import dataclass

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.ramp_map import MAINLINE, angle_dist, direction_geometry

MAX_APPROACH_METERS = 800.0
MAX_FIX_AGE = 0.45


def branch_side(fork):
  _, _, _, _, branch, main, _ = fork
  # The heading of the two branches immediately after the fork provides the turn side.
  delta = (branch[3][0] - main[3][0] + 180.0) % 360.0 - 180.0
  return int(1 if delta > 0.0 else -1) if abs(delta) >= 0.20 else 0


@dataclass(frozen=True)
class BoundRamp:
  way_id: str
  anchor_at: float
  remaining_m: float


class MapSignBinder:
  def __init__(self, roads, forks, scale):
    self.scale = scale
    self.roads = roads
    self.forks = defaultdict(list)
    self.outgoing = defaultdict(list)
    self.geometry = {}
    self.lengths = {}
    for fork in forks:
      side = branch_side(fork)
      self.forks[fork[0]].append((fork[1], side))
    for i, road in enumerate(roads):
      if road["highway"] not in MAINLINE:
        continue
      self.outgoing[road["start"]].append(i)
      geo = direction_geometry(road["points"], scale)
      self.geometry[i] = geo
      self.lengths[i] = np.sqrt(geo[2])

  def _reachable(self, index, distance, side):
    found, visited = {}, set()
    while distance <= MAX_APPROACH_METERS and index not in visited:
      visited.add(index)
      exits = self.forks[index]
      if any(s == 0 for _, s in exits):
        return None
      for ramp, ramp_side in exits:
        if side == ramp_side:
          found.setdefault(self.roads[ramp]["id"], distance)
      parent = self.roads[index]
      ramps = {ramp for ramp, _ in exits}
      mainline = [i for i in self.outgoing[parent["end"]] if i not in ramps and self.roads[i]["end"] != parent["start"]]
      if len(mainline) != 1:
        return None
      index = mainline[0]
      distance += float(np.sum(self.lengths[index]))
    return next(iter(found.items())) if len(found) == 1 else None

  def associate(self, pose, side):
    result = self.binding(pose, side)
    return result.way_id if result else None

  def binding(self, pose, side):
    t, lat, lon, speed, heading, accuracy, fix = pose
    if not fix or not 0.0 < accuracy <= 10.0 or speed < 5.0 or side not in (-1, 1):
      return None
    point = np.array([lat, lon]) * self.scale
    candidates = []
    for index, (origins, deltas, lensq, bearings) in self.geometry.items():
      if not len(lensq):
        continue
      diff = point - origins
      fraction = np.clip(np.sum(diff * deltas, axis=1) / lensq, 0, 1)
      d = np.linalg.norm(diff - fraction[:, None] * deltas, axis=1)
      angle = angle_dist(heading, bearings)
      s = d + 0.30 * angle
      j = int(np.argmin(s))
      if d[j] <= 12.0 + accuracy and angle[j] <= 30.0:
        lengths = self.lengths[index]
        remaining = float(lengths[j] * (1.0 - fraction[j]) + np.sum(lengths[j + 1 :]))
        candidates.append((float(s[j]), index, remaining))
    if not candidates:
      return None
    best = min(score for score, _, _ in candidates)
    outcomes = [self._reachable(index, remaining, side) for score, index, remaining in candidates if score <= best + 4.0]
    if any(outcome is None for outcome in outcomes) or len({way_id for way_id, _ in outcomes}) != 1:
      return None
    distances = [distance for _, distance in outcomes]
    if max(distances) - min(distances) > 20.0:
      return None
    return BoundRamp(outcomes[0][0], t, float(np.median(distances)))
