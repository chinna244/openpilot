"""
The limit of the road a freeway ramp joins, as the limit ahead (StarPilot mapd ramp_end.go 195d0258da, in Python).

A motorway/trunk link has no limit of its own in OSM, so mapd gives nothing while on it: the freeway's limit held down an
exit until the car read the next sign (route 87 t=965-1001, 70 held to Moorland's 40), and an on-ramp gave the freeway's
limit only at the merge. mapd v1.12 publishes a next limit only where the next way joins at a way end, so a ramp that joins
a street mid-way (Moorland, Touhy) or ends in an untagged piece gives none.

While mapd itself is on a ramp (its limit and road name are the matched link's own), this finds the ramp's end along the
link chain and publishes the one limit of the roads that meet there as the limit ahead. The street is joined by its end
points and by its interior nodes. A junction offering two different limits, a ramp that splits, a ramp longer than
RAMP_END_MAX_M, no tile, or any error gives nothing: mapd's own output is then unchanged.
"""
import math
import os

import capnp

TILE_ROOT = "/data/media/0/osm/offline"
TILE_RETRY = 30  # updates (mapd's 1 Hz loop): a missing tile is looked for again this often (mapd may download it)
TILE_SIZE = 0.25  # deg (mapd AREA_BOX_DEGREES)
GROUP_SIZE = 2.  # deg: tiles are filed in directories of this many degrees (mapd GROUP_AREA_BOX_DEGREES)
RAMP_END_MAX_M = 1500.  # a ramp this long or longer is not followed to its end
MATCH_MAX_M = 15.  # the position this close to the ramp's centre line
MATCH_MAX_DEG = 30.  # and heading along it
LIMIT_AGREE = .5  # m/s: the joined roads' limits must agree this closely
EARTH_R = 6371000.

capnp.remove_import_hook()
_offline = capnp.load(os.path.join(os.path.dirname(__file__), "offline.capnp"))
_HC = _offline.HighwayClass
RAMP_CLASSES = (_HC.motorwayLink, _HC.trunkLink)


def _xy(lat0: float, lat: float, lon: float) -> tuple[float, float]:
  """metres east/north in a local equirectangular frame at lat0"""
  return math.radians(lon) * EARTH_R * math.cos(math.radians(lat0)), math.radians(lat) * EARTH_R


def _angle_diff(a: float, b: float) -> float:
  return abs((a - b + 180.) % 360. - 180.)


def effective_limit(way, forward: bool) -> float:
  """mapd's EffectiveMaxSpeed: the directional limit when tagged, else maxSpeed (m/s)"""
  directional = way.maxSpeedForward if forward else way.maxSpeedBackward
  return float(directional if directional > 0. else way.maxSpeed)


def _nearest(way, lat: float, x: float, y: float):
  """(distance, segment index, fraction along it, segment bearing, segment length) of way's segment nearest the point
  within MATCH_MAX_M, or None; also the projected points"""
  pts = [_xy(lat, n.latitude, n.longitude) for n in way.nodes]
  best = None
  for k in range(len(pts) - 1):
    (ax, ay), (bx, by) = pts[k], pts[k + 1]
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    if seg2 <= 0.:
      continue
    t = max(0., min(1., ((x - ax) * dx + (y - ay) * dy) / seg2))
    dist = math.hypot(ax + t * dx - x, ay + t * dy - y)
    if dist <= MATCH_MAX_M and (best is None or dist < best[0]):
      best = (dist, k, t, math.degrees(math.atan2(dx, dy)) % 360., math.sqrt(seg2))
  return best, pts


def _heading_ok(way, seg_bearing: float, bearing: float, forward: bool) -> bool:
  if not forward and way.oneWay:
    return False
  return _angle_diff(bearing, seg_bearing if forward else (seg_bearing + 180.) % 360.) <= MATCH_MAX_DEG


class Tile:
  def __init__(self, path: str):
    with open(path, "rb") as f:
      self.msg = _offline.Offline.from_bytes_packed(f.read(), traversal_limit_in_words=2 ** 62)
    self.ways = self.msg.ways
    self.ramps = [i for i, w in enumerate(self.ways) if w.highwayClass in RAMP_CLASSES and len(w.nodes) >= 2]
    # ramp way index by end node, for following the chain
    self.ramp_by_node: dict[tuple[float, float], list[int]] = {}
    for i in self.ramps:
      nodes = self.ways[i].nodes
      for node in (nodes[0], nodes[len(nodes) - 1]):
        self.ramp_by_node.setdefault((node.latitude, node.longitude), []).append(i)


def tile_path(root: str, lat: float, lon: float) -> str:
  min_lat, min_lon = math.floor(lat / TILE_SIZE) * TILE_SIZE, math.floor(lon / TILE_SIZE) * TILE_SIZE
  name = f"{min_lat:f}_{min_lon:f}_{min_lat + TILE_SIZE:f}_{min_lon + TILE_SIZE:f}"
  group_lat, group_lon = math.floor(min_lat / GROUP_SIZE) * GROUP_SIZE, math.floor(min_lon / GROUP_SIZE) * GROUP_SIZE
  return os.path.join(root, str(int(group_lat)), str(int(group_lon)), name)


class RampEnd:
  def __init__(self, root: str = TILE_ROOT):
    self.root = root
    self._tile_path = None
    self._tile: Tile | None = None
    self._joined: dict[tuple[int, bool], float] = {}  # (end ramp way index, forward) -> joined limit
    self._missing = 0  # updates since a missing tile was last looked for

  def _load(self, lat: float, lon: float) -> Tile | None:
    path = tile_path(self.root, lat, lon)
    self._missing = self._missing + 1 if self._tile is None else 0
    if path != self._tile_path or self._missing >= TILE_RETRY:
      self._tile_path, self._joined, self._missing = path, {}, 0
      try:
        self._tile = Tile(path)
      except Exception:
        self._tile = None
    return self._tile

  def _match(self, tile: Tile, lat: float, lon: float, bearing: float, mapd_limit: float, mapd_name: str):
    """(way index, forward, metres to its end) of the ramp mapd is on, or None. Exactly one ramp may qualify."""
    found = []
    x, y = _xy(lat, lat, lon)
    pad_lat = MATCH_MAX_M / EARTH_R * 180. / math.pi
    pad_lon = pad_lat / max(math.cos(math.radians(lat)), .1)
    for i in tile.ramps:
      way = tile.ways[i]
      if not (way.minLat - pad_lat <= lat <= way.maxLat + pad_lat and way.minLon - pad_lon <= lon <= way.maxLon + pad_lon):
        continue
      if mapd_name not in (way.name, way.ref):  # mapd's road is this ramp (an untagged name matches an untagged ramp)
        continue
      best, pts = _nearest(way, lat, x, y)
      if best is None:
        continue
      _, k, t, seg_bearing, seg_len = best
      for forward in (True, False):
        if not _heading_ok(way, seg_bearing, bearing, forward):
          continue
        if abs(effective_limit(way, forward) - mapd_limit) > .02:  # mapd's limit is this ramp's own (0: untagged)
          continue
        if forward:
          left = (1. - t) * seg_len + sum(math.dist(pts[j], pts[j + 1]) for j in range(k + 1, len(pts) - 1))
        else:
          left = t * seg_len + sum(math.dist(pts[j], pts[j + 1]) for j in range(k))
        found.append((i, forward, left))
    if len(found) != 1 or self._beside_a_road(tile, lat, lon, x, y, bearing, pad_lat, pad_lon):
      return None
    return found[0]

  @staticmethod
  def _beside_a_road(tile: Tile, lat: float, lon: float, x: float, y: float, bearing: float, pad_lat: float, pad_lon: float) -> bool:
    """A road (not a ramp) this close heading the same way: the car may still be on it. mapd publishes no name and no limit
    when it matches nothing, which the name/limit gate cannot tell from an untagged ramp: on the freeway just past an exit
    gore the diverging ramp matched (review 2026-11-26: 64 of 195 mainline points on tile 42/-88). The ramp counts once it
    has separated from the road it leaves."""
    for way in tile.ways:
      if way.highwayClass in RAMP_CLASSES or not (way.minLat - pad_lat <= lat <= way.maxLat + pad_lat and
                                                   way.minLon - pad_lon <= lon <= way.maxLon + pad_lon):
        continue
      best, _ = _nearest(way, lat, x, y)
      if best is not None and (_heading_ok(way, best[3], bearing, True) or _heading_ok(way, best[3], bearing, False)):
        return True
    return False

  def _follow(self, tile: Tile, i: int, forward: bool, distance: float):
    """the last ramp way of the chain, its direction and the metres to its end; None if the ramp splits or is too long"""
    seen = {i}
    while distance <= RAMP_END_MAX_M:
      nodes = tile.ways[i].nodes
      end = nodes[len(nodes) - 1] if forward else nodes[0]
      key = (end.latitude, end.longitude)
      nexts = []
      for j in tile.ramp_by_node.get(key, []):
        if j in seen:
          continue
        w = tile.ways[j]
        first, last = w.nodes[0], w.nodes[len(w.nodes) - 1]
        if (first.latitude, first.longitude) == key:
          nexts.append((j, True))
        elif (last.latitude, last.longitude) == key and not w.oneWay:
          nexts.append((j, False))
      if not nexts:
        return i, forward, distance
      if len(nexts) > 1:
        return None  # a split: which branch is not known
      i, forward = nexts[0]
      seen.add(i)
      w = tile.ways[i]
      distance += sum(EARTH_R * math.radians(1.) * math.hypot(
        (w.nodes[j + 1].longitude - w.nodes[j].longitude) * math.cos(math.radians(w.nodes[j].latitude)),
        w.nodes[j + 1].latitude - w.nodes[j].latitude) for j in range(len(w.nodes) - 1))
    return None

  def _joined_limit(self, tile: Tile, i: int, forward: bool) -> float:
    """the one limit of the roads (not ramps) a car can drive onto where ramp way i ends, 0 for none or two"""
    nodes = tile.ways[i].nodes
    end = nodes[len(nodes) - 1] if forward else nodes[0]
    lat, lon = end.latitude, end.longitude
    msg = tile.msg
    if not (msg.minLat <= lat <= msg.maxLat and msg.minLon <= lon <= msg.maxLon):
      return 0.  # the ramp ends past this tile: the roads meeting there may not all be in it (review 2026-11-26)
    limits = []
    for j, way in enumerate(tile.ways):
      if j == i or way.highwayClass in RAMP_CLASSES or not (way.minLat <= lat <= way.maxLat and way.minLon <= lon <= way.maxLon):
        continue
      wn = way.nodes
      n = len(wn)
      for k in range(n):
        if wn[k].latitude != lat or wn[k].longitude != lon:
          continue
        # leaving the node along the way needs k before its last node, against it k after its first
        if k < n - 1:
          limits.append(effective_limit(way, True))
        if k > 0 and not way.oneWay:
          limits.append(effective_limit(way, False))
        break
    limits = [v for v in limits if v > 0. and math.isfinite(v)]
    if not limits or any(abs(v - limits[0]) > LIMIT_AGREE for v in limits[1:]):
      return 0.
    return limits[0]

  def update(self, lat: float, lon: float, bearing: float | None, mapd_limit: float, mapd_name: str) -> tuple[float, float]:
    """(limit ahead m/s, metres to it), (0, 0) for none"""
    try:
      if bearing is None:
        return 0., 0.
      tile = self._load(lat, lon)
      if tile is None:
        return 0., 0.
      matched = self._match(tile, lat, lon, bearing % 360., mapd_limit, mapd_name)
      if matched is None:
        return 0., 0.
      chain = self._follow(tile, *matched)
      if chain is None:
        return 0., 0.
      i, forward, distance = chain
      if not 0. < distance <= RAMP_END_MAX_M:
        return 0., 0.
      key = (i, forward)
      if key not in self._joined:
        self._joined[key] = self._joined_limit(tile, i, forward)
      speed = self._joined[key]
      if speed <= 0. or abs(speed - mapd_limit) <= .02:
        return 0., 0.
      return speed, distance
    except Exception:
      return 0., 0.
