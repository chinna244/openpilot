import json
import os
import time

import numpy as np
import pytest
import zstandard as zstd

from openpilot.cereal import messaging
from openpilot.sunnypilot.selfdrive.controls.lib import stop_spots_learner as L
from openpilot.sunnypilot.selfdrive.controls.lib.stop_spots import load_spots
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_stop_spots import SPOT, at


def event(along=0., route="r1", bearing=184.):
  lat, lon = at(along)
  return {"lat": lat, "lon": lon, "bearing": bearing, "route": route, "t": 1.}


def write_rlog(path, tsr=True, lead=False, accuracy=2., stop_at=-1., tsr_until=21., stop=True):
  """A 30 s drive at 12 m/s that brakes to a stop at stop_at m along SPOT at t=20 s (or keeps going), with fixes and
  the TSR's stop sign from t=10 s to tsr_until."""
  msgs, t0 = [], 1_000_000_000
  for i in range(int(30 / 0.05)):
    t = i * 0.05
    v = 12. if t < 15. or not stop else max(12. - 2.4 * (t - 15.), 0.)
    x = (stop_at - max(20. - t, 0.) * 6. if t < 20. else stop_at) if stop else stop_at + (t - 20.) * 12.
    cs = messaging.new_message('carState')
    cs.logMonoTime, cs.carState.vEgo = t0 + int(t * 1e9), v
    msgs.append(cs)
    rs = messaging.new_message('radarState')
    rs.logMonoTime, rs.radarState.leadOne.present = t0 + int(t * 1e9), lead
    msgs.append(rs)
    if i % 2 == 0:
      g = messaging.new_message('gpsLocationExternal')
      g.logMonoTime = t0 + int(t * 1e9)
      g.gpsLocationExternal.latitude, g.gpsLocationExternal.longitude = at(x)
      g.gpsLocationExternal.bearingDeg, g.gpsLocationExternal.hasFix = 184., True
      g.gpsLocationExternal.horizontalAccuracy = accuracy
      msgs.append(g)
    if tsr and 10. < t < tsr_until:
      c = messaging.new_message('can', 1)
      c.logMonoTime = t0 + int(t * 1e9)
      c.can[0].address, c.can[0].src, c.can[0].dat = 0x35F, 2, bytes([0, 0, 0, 0x90, 0, 0, 0, 0])
      msgs.append(c)
  msgs.sort(key=lambda m: m.logMonoTime)
  path.write_bytes(zstd.ZstdCompressor().compress(b"".join(m.to_bytes() for m in msgs)))


def make_route(root, name, segs=1, old=True, **kwargs):
  for n in range(segs):
    d = root / f"{name}--{n}"
    d.mkdir()
    write_rlog(d / "rlog.zst", **kwargs)
    if old:
      os.utime(d / "rlog.zst", (0, 0))


@pytest.mark.parametrize("kwargs,n", [({}, 1), ({"tsr": False}, 0), ({"lead": True}, 0), ({"accuracy": 25.}, 0),
                                      ({"tsr_until": 17.}, 0)])
def test_route_events_need_the_sign_still_shown_no_lead_and_a_good_fix(tmp_path, kwargs, n):
  """tsr_until 17: the sign went out of view 3 s before the stop (a red light past a side road's stop sign)."""
  write_rlog(tmp_path / "0.zst", **kwargs)
  evs = [e for e in L.route_events([str(tmp_path / "0.zst")], "r1") if e["kind"] == "stop"]
  assert len(evs) == n
  if n:
    assert evs[0]["route"] == "r1" and evs[0]["bearing"] == pytest.approx(184.)


def test_a_sign_passed_without_slowing_is_a_through_event(tmp_path):
  write_rlog(tmp_path / "0.zst", stop=False)
  assert [e["kind"] for e in L.route_events([str(tmp_path / "0.zst")], "r1")] == ["through"]
  write_rlog(tmp_path / "1.zst")
  assert [e["kind"] for e in L.route_events([str(tmp_path / "1.zst")], "r1")] == ["stop"]


def test_drives_through_it_outvote_a_spot():
  stops = [event(0., f"s{i}") for i in range(3)]
  through = [dict(event(0., f"t{i}"), kind="through") for i in range(2)]
  assert len(L.cluster(stops + through[:1])) == 1, "3 stopped drives to 1 through: kept"
  assert L.cluster(stops + through) == [], "3 to 2: not a stop most of the time"
  assert L.cluster(through) == []


def test_the_spot_sits_at_the_stops_not_where_signs_went_by():
  evs = [event(a, f"s{i}") for i, a in enumerate((-4., 0., 4.))] + [dict(event(18., "t1"), kind="through")] * 2
  spot, = L.cluster(evs)
  assert L.along_cross(SPOT, spot["lat"], spot["lon"])[0] == pytest.approx(0., abs=0.3)


def test_clustering_does_not_depend_on_the_order_events_arrive():
  evs = [event(a, f"r{i}") for i, a in enumerate((-18., 0., 18., -12., 12.))]
  assert L.cluster(evs) == L.cluster(evs[::-1]) == L.cluster(evs[2:] + evs[:2])


def test_a_spot_needs_two_drives_and_sits_at_their_median_stop():
  assert L.cluster([event(0., "r1"), event(2., "r1")]) == [], "two stops in one drive are one sighting"
  spots = L.cluster([event(-3., "r1"), event(0., "r2"), event(1., "r3")])
  assert len(spots) == 1 and spots[0]["routes"] == ["r1", "r2", "r3"]
  assert L.along_cross(SPOT, spots[0]["lat"], spots[0]["lon"])[0] == pytest.approx(0., abs=0.3)


def test_opposite_headings_and_far_stops_are_separate_spots():
  evs = [event(0., "r1"), event(0., "r2"), event(0., "r3", bearing=4.), event(0., "r4", bearing=4.),
         event(-60., "r5"), event(-60., "r6")]
  assert len(L.cluster(evs)) == 3


def test_only_finished_drives_are_read(tmp_path):
  make_route(tmp_path, "00000001--aaaa", segs=2)
  make_route(tmp_path, "00000002--bbbb", old=False)
  (tmp_path / "not_a_route").mkdir()
  (tmp_path / "00000003--cccc--0").mkdir()  # no rlog
  routes = L.finished_routes(str(tmp_path), now=time.time() + L.SETTLE_TIME / 2)  # noqa: TID251 (mtimes are wall-clock)
  assert list(routes) == ["00000001--aaaa"]
  assert [os.path.basename(os.path.dirname(f)) for f in routes["00000001--aaaa"]] == ["00000001--aaaa--0", "00000001--aaaa--1"]


def test_learn_reads_each_drive_once_and_keeps_spots_after_the_logs_go(tmp_path, monkeypatch):
  logs, ev_path, sp_path = tmp_path / "logs", tmp_path / "events.json", tmp_path / "spots.json"
  logs.mkdir()
  make_route(logs, "00000001--aaaa")
  make_route(logs, "00000002--bbbb", stop_at=1.)
  reads = []
  real = L.route_events
  monkeypatch.setattr(L, "route_events", lambda files, route: reads.append(route) or real(files, route))
  assert L.learn(str(logs), ev_path, sp_path, now=1e9)
  assert sorted(reads) == ["00000001--aaaa", "00000002--bbbb"]
  assert len(spots_in(sp_path)) == 1 and load_spots(sp_path) == [], "learned, written unverified: no brake"
  assert not L.learn(str(logs), ev_path, sp_path, now=1e9), "nothing new: no rewrite"
  assert len(reads) == 2
  for d in logs.iterdir():  # the deleter takes the logs
    for f in d.iterdir():
      f.unlink()
    d.rmdir()
  assert not L.learn(str(logs), ev_path, sp_path, now=1e9)
  assert len(spots_in(sp_path)) == 1


def spots_in(path):
  return json.loads(path.read_text())["spots"]


def test_a_spot_is_verified_at_the_camera_line_seen_on_two_drives():
  """The stops end past the line (route 0000002d); the camera's line, from two drives, is the spot, and it brakes."""
  def line(a, r):  # the line 6 m before the spot, wherever the car stopped
    return dict(event(a, r), line=-6. - a)
  spot, = L.cluster([line(0., "r1"), line(2., "r2")])
  assert spot["verified"] and spot["lines"] == 2
  assert L.along_cross(SPOT, spot["lat"], spot["lon"])[0] == pytest.approx(-6., abs=0.3)
  one, = L.cluster([line(0., "r1"), event(2., "r2")])
  assert not one["verified"], "a line from one drive is not enough"
  assert L.along_cross(SPOT, one["lat"], one["lon"])[0] == pytest.approx(1., abs=0.3), "unverified: at the stops"


def bands_seen(rel_bands, stop_t=25.):
  """Sightings of bands at rel_bands m from a stop at stop_t, approached at 6 m/s, as the camera sees them."""
  t = np.arange(0., 30., 0.05)
  v = np.where(t < stop_t, 6., 0.)
  odo = np.concatenate(([0.], np.cumsum(np.diff(t) * (v[1:] + v[:-1]) / 2.)))
  stop_odo = float(np.interp(stop_t, t, odo))
  lines = [(tl, sorted(d for d in (stop_odo + b - float(np.interp(tl, t, odo)) for b in rel_bands) if 5.4 < d < 30.))
           for tl in t[::2] if tl <= stop_t]
  return lines, t, odo


@pytest.mark.parametrize("bands,line", [
  ((-1.75, 1.25), -1.75),  # a crosswalk past the stop (route 0000002d southbound): its near edge
  ((-6., -3., 0.), -6.),  # a stop bar before a crosswalk: the bar
  ((-9., 0.), 0.),  # 9 m apart: not one marking, the band nearest the stop
  ((-12., -9., -6., -3., 0.), -6.),  # concrete joints every 3 m: no more than LINE_STEPS
])
def test_stop_line_steps_upstream_through_one_marking(bands, line):
  lines, t, odo = bands_seen(bands)
  assert L.stop_line(lines, t, odo, 25.) == pytest.approx(line, abs=0.3)


def test_stop_line_without_sightings_is_none():
  _, t, odo = bands_seen(())
  assert L.stop_line([], t, odo, 25.) is None



def test_a_corrupt_events_file_starts_over(tmp_path):
  logs, ev_path, sp_path = tmp_path / "logs", tmp_path / "events.json", tmp_path / "spots.json"
  logs.mkdir()
  ev_path.write_text("{oops")
  assert L.load_events(ev_path) == {}
  ev_path.write_text(json.dumps({"routes": []}))
  assert L.load_events(ev_path) == {}
  good = event(0., "r1")
  bad = [{"lat": "1", "lon": 2., "bearing": 0., "route": "r1", "t": 1.}, dict(good, lat=float("nan")), dict(good, route=3),
         dict(good, kind="maybe"), "x", None]
  ev_path.write_text(json.dumps({"routes": {"r1": [good, *bad], "r2": "oops", "3": [good]}}))
  assert L.load_events(ev_path) == {"r1": [good], "3": [good]}
  L.cluster([e for evs in L.load_events(ev_path).values() for e in evs])  # and it clusters without raising
  assert L.learn(str(logs), ev_path, sp_path, now=1e9)
  assert load_spots(sp_path) == []
