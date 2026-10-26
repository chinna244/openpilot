"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Map-based curve speed targets on the shared decel solver: a target binds under the same
commit gate as the vision path, publishes the decel it needs (budget at most), and at
speed must persist before it binds. The state machine and update loop are sunnypilot's.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.navd.helpers import Coordinate
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import COMMIT_FRAC, PlanningLimits, get_planning_limits, \
  publish_cap_decel
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import (
  ACTIVE_STATES, ENABLED_STATES, R, TO_RADIANS, MapState, SmartCruiseControlMap as UpstreamMap, distance_to_point, velocities_from_param)
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.speed_profile import lead_distance, required_decel

__all__ = ['ACTIVE_STATES', 'ENABLED_STATES', 'MapState', 'R', 'SmartCruiseControlMap', 'target_binds']

_T_FALLBACK = 2.8  # s; decel horizon when the target's distance is degenerate
# a map target is a prediction that can flicker on over road that never curves (mapd matching
# the car to a side street: route 25c, seven targets of 19-49 mph at 55 mph, 0.7-2.1 s each).
# A new or deeper target must be the selection for this long before it binds. Corpus (6.2 h
# with the map on): phantoms at speed last 0.7-2.1 s (one 6 s), real targets 5-12.6 s and
# appear a median 10.5 s before the apex. None below 45 mph, where no phantom was seen and a
# curve comes up fast; over the ramp a real curve at 45-46 mph waits under 0.4 s. At 2.5 s
# this clears 7 of the 10 phantoms (4.3 to 2.2 per hour above 45 mph) and leaves the
# tightest real curve 0.6 s of slack after the actuation lead
_CONFIRM_V_BP = [20.1, 22.4]  # m/s (45, 50 mph)
_CONFIRM_T = [0., 2.5]  # s
# gpsLocation is 1 Hz: between fixes the distances are dead-reckoned from v_ego, or the published
# decel (which divides by the distance) saws at every fix. A tunnel or underpass sends no fix; past
# this the position counts as lost rather than reckoned on
_FIX_HOLD_T = 3.  # s
# mapd sets a target at every OSM node from that node's curvature at its fixed lateral accel.
# Interchanges map a sweeping bend as a few kinked nodes, so a bend the road takes at 61 mph
# reads as a 40 mph corner (route 18b 465 s: the car drove 1.7 m from the point, at 57 mph).
# At speed, a target is relaxed to the bend's curvature over a longer stretch (the heading
# change between the chords L before and after the node, over L) when that is clearly
# gentler. Map geometry alone cannot tell a kink from a real short curve (a well-mapped 45 deg
# curve of 100 m radius reads 14.1 -> 17.8 m/s), so the camera must agree: a target relaxes
# only once the model's path covers it, to the lower of the two readings, and then stays
# relaxed until passed. The model path reaches 190-360 m at speed and covers a target within
# about 2 s of it binding; it reads real curves within 2-5 mph of mapd (186 461 s: 39 vs 44)
# and the kinks as straight or gentle (18b 465 s: 40 vs 78); a closer, tighter reading lowers it
# again. Not in town, where a real corner
# is a single node (junction turns would read 29 -> 36 and 37 -> 60 mph), nor below 25 mph,
# where a ramp ends in a short tight corner (186 859 s)
_MAPD_LAT_ACC = 2.0  # m/s2, mapd's TARGET_LAT_ACCEL
_RELAX_V_EGO = 20.1  # m/s (45 mph)
_RELAX_MIN_V = 11.18  # m/s (25 mph)
_RELAX_RATIO = 1.2
_RELAX_L = (100., 150.)  # m
_RELAX_V_MAX = 40.  # m/s; a straight reads as an infinite curve speed
_CAMERA_L = (30., 50.)  # m; the model path is read over this either side of the target
# the camera must read the target gentle this many judged frames in a row (its tightest reading is
# taken): one noisy frame at the path's far end must not relax a real curve
_CAMERA_FRAMES = 10
# mapd can match the car to the wrong way where two cross or run side by side (route 17a on a
# bridge, 16a): its list then turns back on itself at one node, and the targets around that node
# read as a 17-23 mph corner on a straight road (about 4 s of braking each). A target is dropped
# when a node within two of it (its curvature uses those), within _GUARD_REACH m of the car and
# _GUARD_LATERAL m of the model's path (17a's other carriageway was 9.9 m off), turns by more than
# _GUARD_NODE_DEG and leaves more than _GUARD_HEAD_DEG off the path, which must run straight on
# _GUARD_PAST m past the node: the camera sees no corner there. The
# reversals left 153-169 deg off the path (17a at least 109, 159 in 9 frames of 10), a real corner
# mostly 100-135; the path ran straight 30 m past them in every frame. A node ahead the camera
# cannot see 30 m past is not judged; one behind the car is, on the map and gps course alone (its
# exit on a road the car drove points ahead). The target being braked for is never dropped.
# The first listed node has no map predecessor: its turn is taken against the model's path.
# Only on a near-straight path: the position is the last 1 Hz fix reckoned straight ahead, so in a
# bend (town corners at 10 mph turn 90-130 deg over the reach) the map sits rotated against the
# path; the corpus's reversals came with the path within 3 deg of straight. Nor below 15 mph,
# where a gps course is unreliable (parking lots read as reversals); they were at 20-34 mph
_GUARD_V_EGO = 6.7  # m/s (15 mph)
_GUARD_REACH = 60.  # m
_GUARD_LATERAL = 12.  # m
_GUARD_STRAIGHT_DEG = 15.
_GUARD_NODE_DEG = 100.
_GUARD_HEAD_DEG = 135.
_GUARD_PAST = 30.  # m


def relaxed_velocities(targets) -> np.ndarray:
  """Each target's velocity, raised to its longer-stretch curve speed where that is clearly higher.
  A point without L of path on both sides is not relaxed."""
  v = np.array([float(p["velocity"]) for p in targets])
  if len(v) < 3:
    return v
  lat = np.radians([p["latitude"] for p in targets])
  lon = np.radians([p["longitude"] for p in targets])
  P = np.stack([R * lon * np.cos(lat[0]), R * lat], axis=1)
  s = np.concatenate([[0.], np.cumsum(np.hypot(*np.diff(P, axis=0).T))])
  v_long = np.full(len(P), np.inf)
  for L in _RELAX_L:
    a = np.stack([np.interp(s - L, s, P[:, 0]), np.interp(s - L, s, P[:, 1])], axis=1)
    b = np.stack([np.interp(s + L, s, P[:, 0]), np.interp(s + L, s, P[:, 1])], axis=1)
    h_in = np.arctan2(*(P - a).T[::-1])
    h_out = np.arctan2(*(b - P).T[::-1])
    turn = np.abs(np.angle(np.exp(1j * (h_out - h_in))))
    with np.errstate(divide='ignore'):
      v_L = np.sqrt(_MAPD_LAT_ACC * L / turn)
    covered = (s - L >= 0.) & (s + L <= s[-1])
    v_long = np.minimum(v_long, np.where(covered, v_L, 0.))
  v_long = np.minimum(v_long, _RELAX_V_MAX)
  return np.where((v >= _RELAX_MIN_V) & (v_long >= _RELAX_RATIO * v), v_long, v)


def model_curve_speed(px, py, x0: float) -> float | None:
  """Curve speed (at mapd's lateral accel) of the model's path around x0 m ahead: the lower over
  _CAMERA_L of the heading change between the path L before and L after, over L. None when the path
  does not reach L past x0."""
  px, py = np.asarray(px, dtype=float), np.asarray(py, dtype=float)
  if len(px) < 2 or not np.isfinite(x0) or not (np.isfinite(px).all() and np.isfinite(py).all()):
    return None
  if x0 - max(_CAMERA_L) < 0. or x0 + max(_CAMERA_L) > px[-1] or np.any(np.diff(px) <= 0.):
    return None
  v = np.inf
  for L in _CAMERA_L:
    xs = np.array([x0 - L, x0, x0 + L])
    ys = np.interp(xs, px, py)
    turn = abs(np.angle(np.exp(1j * (np.arctan2(ys[2] - ys[1], L) - np.arctan2(ys[1] - ys[0], L)))))
    with np.errstate(divide='ignore'):
      v = min(v, float(np.sqrt(_MAPD_LAT_ACC * L / turn)))
  return v


def ahead_distance(lat0: float, lon0: float, bearing_deg: float, lats, lons) -> np.ndarray:
  """How far ahead (m, along the bearing) each point lies from (lat0, lon0)"""
  east = R * np.radians(np.asarray(lons) - lon0) * np.cos(np.radians(lat0))
  north = R * np.radians(np.asarray(lats) - lat0)
  h = np.radians(bearing_deg)
  return east * np.sin(h) + north * np.cos(h)


def car_frame(lat0: float, lon0: float, bearing_deg: float, lats, lons) -> tuple[np.ndarray, np.ndarray]:
  """Points in the model's frame at (lat0, lon0): m ahead along the bearing, and to the right"""
  east = R * np.radians(np.asarray(lons) - lon0) * np.cos(np.radians(lat0))
  north = R * np.radians(np.asarray(lats) - lat0)
  h = np.radians(bearing_deg)
  return east * np.sin(h) + north * np.cos(h), east * np.cos(h) - north * np.sin(h)


def reversal_keep(px, py, x, y) -> np.ndarray:
  """Which targets (x ahead, y right of the car, in list order) to keep against the model's path
  (px, py): not those within two nodes of a covered node that doubles back off the path"""
  x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
  px, py = np.asarray(px, dtype=float), np.asarray(py, dtype=float)
  keep = np.ones(len(x), dtype=bool)
  if len(x) < 2 or len(px) < 2 or not all(np.isfinite(a).all() for a in (x, y, px, py)):
    return keep
  sr = np.concatenate([[0.], np.cumsum(np.hypot(np.diff(px), np.diff(py)))])
  # the path's heading: of its chord to _GUARD_PAST m, steadier than its first (sub-metre) step
  l0 = min(_GUARD_PAST, sr[-1])
  h0 = np.arctan2(np.interp(l0, sr, py) - py[0], np.interp(l0, sr, px) - px[0])
  h_path = np.arctan2(np.diff(py), np.diff(px))
  if np.any(np.abs(np.angle(np.exp(1j * (h_path[sr[:-1] < _GUARD_REACH + _GUARD_PAST] - h0)))) > np.radians(_GUARD_STRAIGHT_DEG)):
    return keep
  covered = (np.hypot(x, y) < _GUARD_REACH) & (np.abs(y - np.interp(x, px, py)) < _GUARD_LATERAL) & (x + _GUARD_PAST <= sr[-1])
  h_map = np.arctan2(np.diff(y), np.diff(x))  # node i to i+1
  h_off = np.abs(np.angle(np.exp(1j * (h_map - h0))))
  off = np.zeros(len(x), dtype=bool)
  off[:-1] = covered[:-1] & (h_off > np.radians(_GUARD_HEAD_DEG))
  turn = np.zeros(len(x))
  turn[1:-1] = np.abs(np.angle(np.exp(1j * np.diff(h_map))))
  turn[0] = h_off[0]  # off only where covered
  bad = (turn > np.radians(_GUARD_NODE_DEG)) & off
  return np.convolve(bad, np.ones(5), 'full')[2:len(x) + 2] == 0


def target_binds(v_ego: float, tv, d, lim: PlanningLimits) -> np.ndarray:
  """Which targets (speeds tv at distances d) bind: the decel each requires past the actuation
  lead reaches the commit fraction of the budget; same solver and gate as the vision path."""
  tv = np.asarray(tv, dtype=float)
  d = np.asarray(d, dtype=float)
  jerk = lim.jerk(v_ego)
  d_lead = [lead_distance(v_ego, lim.t_lead + lim.dash_traversal_time(v_ego - v), lim.a_budget, jerk) for v in tv]
  a_req = np.array([required_decel(v_ego, [v], [dd], dl) for v, dd, dl in zip(tv, d, d_lead, strict=True)])
  return (tv <= v_ego) & (a_req >= COMMIT_FRAC * lim.a_budget)


class SmartCruiseControlMap(UpstreamMap):
  def __init__(self, CP):
    super().__init__()
    self.limits = get_planning_limits(CP)
    self.v_target = 0.
    self.target_distance = 0.
    self.candidate = None  # (mapd v, lat, lon) selected this frame, bound once confirmed
    self.candidate_age = 0  # frames the same candidate has been selected
    self._a_out = 0.
    self.fix: tuple[Coordinate, int, float] | None = None  # the last position with a fix, its logMonoTime and bearing
    self.relaxed: dict[tuple[float, float, float], float] = {}  # (lat, lon, mapd velocity) -> relaxed, kept until passed
    self.votes: dict[tuple[float, float, float], tuple[int, float]] = {}  # camera frames agreeing so far, tightest reading
    self.target_mapd_v = 0.  # the bound target's mapd velocity: its identity while its published velocity moves

  def get_a_target_from_control(self) -> float:
    # keys ICBM's overshoot gap on stock ACC; ramped since the plan aTarget seeds the MPC
    if self.is_active and 0. < self.v_target < self.v_ego:
      self._a_out = publish_cap_decel(self.v_target, self.target_distance, _T_FALLBACK, self._a_out, self.limits, self.v_ego)
    else:
      self._a_out = self.a_ego
    return self._a_out

  def fix_age(self, sm: messaging.SubMaster) -> float | None:
    """Seconds since the last fix, which becomes last_position; None with no fix in _FIX_HOLD_T.
    Upstream takes every message, and one without a fix reads as (0, 0), thousands of km away."""
    gps = sm[self.gps_location_service]
    if gps.hasFix:
      self.fix = Coordinate(gps.latitude, gps.longitude), sm.logMonoTime[self.gps_location_service], gps.bearingDeg
    if self.fix is None:
      return None
    self.last_position = self.fix[0]
    age = max(0., (sm.logMonoTime['modelV2'] - self.fix[1]) * 1e-9)
    return age if age <= _FIX_HOLD_T else None

  def relax_targets(self, path, pose) -> None:
    """Swap in relaxed velocities: new ones only at speed and once the model path (px, py) covers the
    target, earlier ones while the point is still listed with the same mapd velocity. A later, tighter
    camera reading lowers a relaxed target (never under mapd's). An empty list (mapd or the localizer
    dropping out) keeps them. pose: (lat, lon, bearing, m travelled since the fix)."""
    keys = [(p["latitude"], p["longitude"], p["velocity"]) for p in self.target_velocities]
    if keys:
      self.relaxed = {k: self.relaxed[k] for k in keys if k in self.relaxed}
      self.votes = {k: self.votes[k] for k in keys if k in self.votes}
    if path is not None:
      v_map = relaxed_velocities(self.target_velocities) if self.v_ego >= _RELAX_V_EGO else np.zeros(len(keys))
      # once per frame each: a repeated node (two ways joining) must not vote twice
      judge = list({k: i for i, (k, p) in enumerate(zip(keys, self.target_velocities, strict=True))
                    if v_map[i] > p["velocity"] or k in self.relaxed}.values())
      if judge:
        x = ahead_distance(pose[0], pose[1], pose[2], [keys[i][0] for i in judge], [keys[i][1] for i in judge]) - pose[3]
        for i, xi in zip(judge, x, strict=True):
          k, v_cam = keys[i], model_curve_speed(path[0], path[1], float(xi))
          if v_cam is None:
            continue
          if k in self.relaxed:
            self.relaxed[k] = max(k[2], min(self.relaxed[k], v_cam))
          elif min(v_map[i], v_cam) >= _RELAX_RATIO * k[2]:
            n, v_low = self.votes.get(k, (0, np.inf))
            n, v_low = n + 1, min(v_low, v_map[i], v_cam)
            if n >= _CAMERA_FRAMES:
              self.relaxed[k] = float(v_low)
              self.votes.pop(k, None)
            else:
              self.votes[k] = (n, v_low)
          else:
            self.votes.pop(k, None)
    self.target_velocities = [{**p, "velocity": self.relaxed.get(k, p["velocity"]), "mapd_v": p["velocity"]}
                              for k, p in zip(keys, self.target_velocities, strict=True)]

  def _release_target(self) -> None:
    self.v_target, self.target_lat, self.target_lon, self.target_distance, self.target_mapd_v = 0., 0., 0., 0., 0.

  def update_calculations(self, sm: messaging.SubMaster) -> None:
    age = self.fix_age(sm)
    if age is None:
      self.candidate, self.candidate_age = None, 0
      self._release_target()
      return
    # targets belong to the position mapd was given; without a valid one they are stale
    self.target_velocities = (velocities_from_param("MapTargetVelocities", self.mem_params) or []) if self.position_valid else []
    try:
      position = sm['modelV2'].position
      path = (np.array(position.x), np.array(position.y))
    except KeyError:
      path = None
    # a gps without a course reports 0 (or nothing): the camera window would sit on the wrong road
    if not np.isfinite(self.fix[2]) or self.fix[2] == 0.:
      path = None
    if path is not None and self.target_velocities and self.v_ego >= _GUARD_V_EGO:
      x, y = car_frame(self.fix[0].latitude, self.fix[0].longitude, self.fix[2], [p["latitude"] for p in self.target_velocities],
                       [p["longitude"] for p in self.target_velocities])
      keep = reversal_keep(path[0], path[1], x - self.v_ego * age, y)
      bound = (self.target_mapd_v, self.target_lat, self.target_lon)  # (0, 0, 0) once released
      self.target_velocities = [p for p, k in zip(self.target_velocities, keep, strict=True)
                                if k or (p["velocity"], p["latitude"], p["longitude"]) == bound]
    self.relax_targets(path, (self.fix[0].latitude, self.fix[0].longitude, self.fix[2], self.v_ego * age))
    lat, lon = self.last_position.latitude * TO_RADIANS, self.last_position.longitude * TO_RADIANS
    distances = [distance_to_point(lat, lon, p["latitude"] * TO_RADIANS, p["longitude"] * TO_RADIANS) for p in self.target_velocities]

    # only the path from our position forward (the nearest point, if one is within 1 km), less the
    # road covered since the fix; a point that falls behind is passed
    start = int(np.argmin(distances)) if distances and min(distances) < 1000. else 0
    travelled = self.v_ego * age
    ahead = [(p, d - travelled) for p, d in zip(self.target_velocities[start:], distances[start:], strict=True) if d >= travelled]
    forward = [p for p, _ in ahead]
    forward_d = [d for _, d in ahead]
    binds = target_binds(self.v_ego, [p["velocity"] for p in forward], forward_d, self.limits)
    valid = [(float(p["velocity"]), p["latitude"], p["longitude"], d, p["mapd_v"]) for p, d, b in zip(forward, forward_d, binds, strict=True) if b]
    best = min(valid, key=lambda x: x[0], default=None)  # the slowest target, nearest on a tie

    # identity is mapd's own velocity: a relaxed target lowered by the camera stays the same target
    candidate = (best[4], best[1], best[2]) if best else None
    self.candidate_age = self.candidate_age + 1 if candidate == self.candidate else 0
    self.candidate = candidate
    t_confirm = float(np.interp(self.v_ego, _CONFIRM_V_BP, _CONFIRM_T))
    confirmed = candidate is not None and self.candidate_age * DT_MDL >= t_confirm

    if self.v_target != 0. and (best is None or self.v_target < best[0] or not confirmed):
      for p, d in zip(forward, forward_d, strict=True):
        if p["velocity"] <= self.v_ego and (p["mapd_v"], p["latitude"], p["longitude"]) == (self.target_mapd_v, self.target_lat, self.target_lon):
          # still ahead, just under the commit gate (or a deeper target is not confirmed yet):
          # keep the target at its current distance, since the published decel divides by it, and
          # at its current velocity, which the camera may have lowered
          self.v_target, self.target_distance = float(p["velocity"]), d
          return
      self._release_target()

    if confirmed:
      self.v_target, self.target_lat, self.target_lon, self.target_distance, self.target_mapd_v = best
