import collections
import math
import numpy as np
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.ramp_map import angle_dist


def fit(p, heading, geom, offset=0.0):
  origin, delta, lensq, direction = geom
  diff = p - origin
  along = np.sum(diff * delta, axis=1) / lensq
  cross = (-diff[:, 0] * delta[:, 1] + diff[:, 1] * delta[:, 0]) / np.sqrt(lensq)
  outside = (along - np.clip(along, 0, 1)) * np.sqrt(lensq)
  distance = np.hypot(cross - offset, outside)
  angle = angle_dist(heading, direction)
  j = np.argmin(distance + 0.30 * angle)
  return distance[j], angle[j], distance[j] + 0.30 * angle[j], cross[j], 0 < along[j] < 1


def scores(p, heading, geom, offset=0.0):
  return fit(p, heading, geom, offset)[:3]


class RampMatcher:
  def __init__(self, roads, forks, scale):
    self.roads = roads
    self.forks = forks
    self.scale = scale
    self.nodes = np.asarray([fork[3] for fork in forks]) * scale
    self.reset()

  def reset(self):
    self.streak = {}
    self.approached = {}
    self.calibration = collections.defaultdict(collections.deque)

  def update(self, pose):
    t, lat, lon, speed, heading, accuracy, fix = pose
    if not fix or not 0 < accuracy <= 15.0 or speed < 3.0:
      self.streak.clear()
      return [], []
    point = np.array([lat, lon]) * self.scale
    matches = []
    nearby = [self.forks[i] for i in np.flatnonzero(np.linalg.norm(self.nodes - point, axis=1) <= 260.0)]
    for fork in nearby:
      i, ramp, main, node, branch_geo, main_geo, parent_geo = fork
      pd, pa, ps, lateral, is_along_parent = fit(point, heading, parent_geo)
      bd, ba, bs = scores(point, heading, branch_geo)
      _, _, ms = scores(point, heading, main_geo)
      normal_tolerance = max(10.0, min(20.0, accuracy * 2))
      if pd < normal_tolerance and pa < 30 and ps < bs:
        self.approached[ramp] = t

      history = self.calibration[ramp]
      while history and t - history[0][0] > 9.0:
        history.popleft()
      if is_along_parent and pa < 12.0 and bd > 45.0 and ps + 15.0 < bs and pd < 6.0 + 4.5 * self.roads[i]['lanes']:
        history.append((t, lateral))

      if (t - self.approached.get(ramp, -math.inf) > 12 or bd > normal_tolerance) and len(history) >= 8:
        offsets = [value for _, value in history]
        if history[-1][0] - history[0][0] >= 1.5 and np.ptp(offsets) <= 8.0:
          correction = float(np.median(offsets))
          if abs(correction) > normal_tolerance and ms - bs >= max(7.0, accuracy):
            adjusted_b, _, adjusted_bs = scores(point, heading, branch_geo, correction)
            adjusted_m, _, adjusted_ms = scores(point, heading, main_geo, correction)
            if adjusted_b < normal_tolerance and adjusted_ms - adjusted_bs >= max(7.0, accuracy):
              bd, bs, ms = adjusted_b, adjusted_bs, adjusted_ms
              self.approached[ramp] = t

      if t - self.approached.get(ramp, -math.inf) > 12 or bd > normal_tolerance or ba > 35 or ms - bs < max(7.0, accuracy):
        self.streak.pop(ramp, None)
        continue
      began, last = self.streak.get(ramp, (t, t))
      if t - last > 0.5:
        began = t
      self.streak[ramp] = (began, t)
      if t - began >= 1.0:
        matches.append(fork)
    return nearby, matches
