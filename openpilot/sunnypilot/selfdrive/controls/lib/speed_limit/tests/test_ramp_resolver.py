import json

import pytest

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.ramp_resolver import NO_OFFER, RampAdvisoryResolver


def graph(tmp_path, advisory):
  def road(way, highway, start, end, points, speed=0.0):
    return {"id": way, "highway": highway, "start": start, "end": end, "points": points, "lanes": 2, "advisory": speed}

  roads = [
    road("approach", "motorway", "a", "b", [[0.0, -0.004], [0.0, 0.0]]),
    road("main", "motorway", "b", "c", [[0.0, 0.0], [0.0, 0.010]]),
    road("ramp", "motorway_link", "b", "d", [[0.0, 0.0], [-0.002, 0.002]], advisory * CV.MPH_TO_MS),
  ]
  p = tmp_path / "ramp_graph.json"
  p.write_text(json.dumps({"version": 1, "latitude": 0.0, "roads": roads}))
  return p


def drive(resolver, branch, advisory, posted_change=0.0):
  sign = (2.0, advisory, 1) if advisory else None
  for frame in range(130):
    t = frame / 10.0
    lon = (t - 5.0) * 0.00018
    lat = -max(lon, 0) if branch else 0.0
    heading = 135.0 if branch and lon > 0.0 else 90.0
    pose = (t, lat, lon, 20.0, heading, 2.0, True)
    posted = (posted_change if t >= 9.0 else 65.0) * CV.MPH_TO_MS if posted_change else 0.0
    yield t, resolver.update(t, pose, sign, 20.0, 65.0 * CV.MPH_TO_MS, True, posted)


@pytest.mark.parametrize(
  "branch,sign,mapped,expected",
  [
    (True, 50, 0, 50),
    (True, 0, 35, 35),
    (True, 0, 0, 0),
    (False, 50, 0, 0),
    (True, 50, 35, 0),
  ],
)
def test_only_confirmed_branch_with_consistent_advisory(branch, sign, mapped, expected, tmp_path):
  resolver = RampAdvisoryResolver(graph(tmp_path, mapped))
  offers = [(t, offer) for t, offer in drive(resolver, branch, sign) if offer != NO_OFFER]
  if expected:
    assert offers
    assert all(t > 6.0 and speed == pytest.approx(expected * CV.MPH_TO_MS) and token > 0 and expires > int(t * 1e9) for t, (speed, token, expires) in offers)
  else:
    assert not offers


def test_replacement_and_missing_graph(tmp_path):
  resolver = RampAdvisoryResolver(graph(tmp_path, 0))
  offers = [(t, o) for t, o in drive(resolver, True, 50, posted_change=40.0) if o != NO_OFFER]
  assert offers and max(t for t, _ in offers) < 9.0
  missing = RampAdvisoryResolver(tmp_path / "missing.json")
  assert all(offer == NO_OFFER for _, offer in drive(missing, True, 50))
