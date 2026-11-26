"""The limit of the road a freeway ramp joins, as the limit ahead (live_map_data/ramp_end.py)."""
import os

import pytest

from openpilot.sunnypilot.mapd.live_map_data import ramp_end
from openpilot.sunnypilot.mapd.live_map_data.ramp_end import RAMP_END_MAX_M, TILE_RETRY, RampEnd, tile_path

MPH = 0.44704
HC = ramp_end._HC
RAMP_START, RAMP_END = (43.004, -88.1000), (43.006, -88.0990)  # straight, heading ~18 deg
ON_RAMP = (43.005, -88.0995)
RAMP_BEARING = 18.


def way(nodes, cls, limit=0., name="", oneway=True, wid=1):
  lats, lons = [n[0] for n in nodes], [n[1] for n in nodes]
  return dict(id=wid, name=name, ref="", maxSpeed=limit, highwayClass=cls, oneWay=oneway, nodes=[{"latitude": a, "longitude": b} for a, b in nodes],
              minLat=min(lats), maxLat=max(lats), minLon=min(lons), maxLon=max(lons))


def street(limit=30 * MPH, wid=3):
  # the ramp ends at this street's interior node (OSM ramps often join mid-way)
  return way([(43.006, -88.1010), RAMP_END, (43.006, -88.0970)], HC.residential, limit, "Moorland Road", oneway=False, wid=wid)


def write_tile(root, ways):
  msg = ramp_end._offline.Offline.new_message()
  msg.minLat, msg.minLon, msg.maxLat, msg.maxLon = 43.0, -88.25, 43.25, -88.0  # the tile ON_RAMP is in
  msg.init("ways", len(ways))
  for i, w in enumerate(ways):
    out = msg.ways[i]
    for k, v in w.items():
      if k == "nodes":
        out.init("nodes", len(v))
        for j, n in enumerate(v):
          out.nodes[j].latitude, out.nodes[j].longitude = n["latitude"], n["longitude"]
      else:
        setattr(out, k, v)
  path = tile_path(str(root), *ON_RAMP)
  os.makedirs(os.path.dirname(path), exist_ok=True)
  with open(path, "wb") as f:
    f.write(msg.to_bytes_packed())


@pytest.fixture
def base_ways():
  return [way([(43.000, -88.1000), (43.010, -88.1000)], HC.motorway, 70 * MPH, "Zoo Freeway", wid=1),
          way([RAMP_START, RAMP_END], HC.motorwayLink, wid=2),
          street()]


def test_tile_path_follows_mapd_grouping():
  assert tile_path("/r", 43.1, -87.9).endswith("/42/-88/43.000000_-88.000000_43.250000_-87.750000")


def test_on_the_ramp_gives_the_street_limit_ahead(tmp_path, base_ways):
  write_tile(tmp_path, base_ways)
  speed, distance = RampEnd(str(tmp_path)).update(*ON_RAMP, RAMP_BEARING, 0., "")
  assert speed == pytest.approx(30 * MPH)
  assert 100. < distance < 140.  # half of the ~234 m ramp


def test_an_on_ramp_gives_the_freeway_limit(tmp_path):
  # an on-ramp whose end is a freeway node gives the freeway's limit
  ramp = way([RAMP_START, RAMP_END], HC.motorwayLink, wid=2)
  freeway = way([(43.000, -88.0990), RAMP_END, (43.010, -88.0990)], HC.motorway, 70 * MPH, "Zoo Freeway", wid=1)
  write_tile(tmp_path, [ramp, freeway])
  assert RampEnd(str(tmp_path)).update(*ON_RAMP, RAMP_BEARING, 0., "")[0] == pytest.approx(70 * MPH)


@pytest.mark.parametrize("mapd_limit, mapd_name", [(70 * MPH, "Zoo Freeway"), (70 * MPH, ""), (0., "Zoo Freeway")])
def test_nothing_unless_mapd_is_on_the_ramp(tmp_path, base_ways, mapd_limit, mapd_name):
  # still on the freeway beside the ramp (mapd's limit or road is the freeway's): nothing
  write_tile(tmp_path, base_ways)
  assert RampEnd(str(tmp_path)).update(*ON_RAMP, RAMP_BEARING, mapd_limit, mapd_name) == (0., 0.)


def test_nothing_against_a_one_way_ramp(tmp_path, base_ways):
  write_tile(tmp_path, base_ways)
  assert RampEnd(str(tmp_path)).update(*ON_RAMP, RAMP_BEARING + 180., 0., "") == (0., 0.)


def test_nothing_without_a_bearing(tmp_path, base_ways):
  write_tile(tmp_path, base_ways)
  assert RampEnd(str(tmp_path)).update(*ON_RAMP, None, 0., "") == (0., 0.)


def test_two_limits_at_the_ramp_end_give_nothing(tmp_path, base_ways):
  other = way([RAMP_END, (43.008, -88.0990)], HC.primary, 45 * MPH, "Other Road", oneway=False, wid=4)
  write_tile(tmp_path, base_ways + [other])
  assert RampEnd(str(tmp_path)).update(*ON_RAMP, RAMP_BEARING, 0., "") == (0., 0.)


def test_an_untagged_street_gives_nothing(tmp_path, base_ways):
  write_tile(tmp_path, base_ways[:2] + [street(limit=0.)])
  assert RampEnd(str(tmp_path)).update(*ON_RAMP, RAMP_BEARING, 0., "") == (0., 0.)


def test_a_split_ramp_gives_nothing(tmp_path, base_ways):
  # the ramp splits before any street: whichever branch the car takes is not known, though one ends at a 30 street
  a = way([RAMP_END, (43.007, -88.0985)], HC.motorwayLink, wid=5)
  b = way([RAMP_END, (43.007, -88.0995)], HC.motorwayLink, wid=6)
  a_street = way([(43.007, -88.0990), (43.007, -88.0985), (43.007, -88.0980)], HC.residential, 30 * MPH, "A Street", oneway=False, wid=8)
  write_tile(tmp_path, base_ways[:2] + [a, b, a_street])
  assert RampEnd(str(tmp_path)).update(*ON_RAMP, RAMP_BEARING, 0., "") == (0., 0.)


def test_follows_a_ramp_in_pieces_to_its_end(tmp_path, base_ways):
  mid = (43.005, -88.0995)
  pieces = [way([RAMP_START, mid], HC.motorwayLink, wid=2), way([mid, RAMP_END], HC.motorwayLink, wid=7)]
  write_tile(tmp_path, [base_ways[0], *pieces, street()])
  speed, distance = RampEnd(str(tmp_path)).update(43.0045, -88.09975, RAMP_BEARING, 0., "")
  assert speed == pytest.approx(30 * MPH) and 150. < distance < 200.


def test_too_long_a_ramp_gives_nothing(tmp_path, monkeypatch, base_ways):
  monkeypatch.setattr(ramp_end, "RAMP_END_MAX_M", 50.)
  write_tile(tmp_path, base_ways)
  assert RampEnd(str(tmp_path)).update(*ON_RAMP, RAMP_BEARING, 0., "") == (0., 0.)
  assert RAMP_END_MAX_M == 1500.


def test_no_tile_gives_nothing(tmp_path):
  assert RampEnd(str(tmp_path)).update(*ON_RAMP, RAMP_BEARING, 0., "") == (0., 0.)


def test_a_corrupt_tile_gives_nothing(tmp_path):
  path = tile_path(str(tmp_path), *ON_RAMP)
  os.makedirs(os.path.dirname(path), exist_ok=True)
  with open(path, "wb") as f:
    f.write(b"\x00garbage")
  assert RampEnd(str(tmp_path)).update(*ON_RAMP, RAMP_BEARING, 0., "") == (0., 0.)


def test_a_blank_mapd_on_the_freeway_past_a_gore_gives_nothing(tmp_path):
  # mapd matching nothing publishes no name and no limit, like an untagged ramp. On the freeway 10 m past the gore, with the
  # ramp diverging a few metres away, the freeway beside the car vetoes it (review 2026-11-26: 64/195 mainline points matched)
  gore = (43.004, -88.1000)
  freeway = way([(43.000, -88.1000), gore, (43.010, -88.1000)], HC.motorway, 70 * MPH, "Zoo Freeway", wid=1)
  ramp = way([gore, (43.0042, -88.09995), RAMP_END], HC.motorwayLink, wid=2)
  write_tile(tmp_path, [freeway, ramp, street()])
  assert RampEnd(str(tmp_path)).update(43.0041, -88.1000, 0., 0., "") == (0., 0.)
  # once on the ramp, clear of the freeway, it gives the street's limit
  assert RampEnd(str(tmp_path)).update(*ON_RAMP, RAMP_BEARING, 0., "")[0] == pytest.approx(30 * MPH)


def test_a_ramp_ending_past_the_tile_gives_nothing(tmp_path):
  # the roads meeting at a node outside the tile may not all be in it
  end = (43.2504, -88.1000)  # 45 m past the tile's north edge
  ramp = way([(43.2490, -88.1000), end], HC.motorwayLink, wid=2)
  road = way([(43.2504, -88.1010), end, (43.2504, -88.0990)], HC.residential, 30 * MPH, "Edge Road", oneway=False, wid=3)
  write_tile(tmp_path, [ramp, road])  # bounds 43.0..43.25
  # the car is inside the tile (43.2495), heading north on the ramp
  assert os.path.exists(tile_path(str(tmp_path), 43.2495, -88.1))
  assert RampEnd(str(tmp_path)).update(43.2495, -88.1000, 0., 0., "") == (0., 0.)


def test_a_tile_downloaded_later_is_used(tmp_path, base_ways):
  re = RampEnd(str(tmp_path))
  assert re.update(*ON_RAMP, RAMP_BEARING, 0., "") == (0., 0.)  # no tile yet
  write_tile(tmp_path, base_ways)
  for _ in range(TILE_RETRY - 1):
    re.update(*ON_RAMP, RAMP_BEARING, 0., "")
  assert re.update(*ON_RAMP, RAMP_BEARING, 0., "")[0] == pytest.approx(30 * MPH)
