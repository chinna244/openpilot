#!/usr/bin/env python3
"""
Learns stop spots (stop_spots.py) on the device, offroad, from its own drives.

A stop event is a stop (under STOP_SPEED after moving over MOVING_SPEED in the 20 s before) with no radar lead in the
last 3 s, a GPS fix within GPS_MAX_ACCURACY at the stop, and the Mazda TSR's STOP_SIGN (CAM_TRAFFIC_SIGNS 0x35F, value
9) still shown within TSR_AT_STOP of it: at all 25 stops learned from routes 00000166-0000002b the sign was shown until
0.4 s before the stop or later, so a red light reached after passing a side road's stop sign is not one. A through
event is a TSR stop sign passed without slowing under THROUGH_SPEED. Events within MATCH_DIST and HEADING_TOL of each
other are one spot, which needs stops on MIN_ROUTES drives and on THROUGH_RATIO times as many drives as went through
it; its point is the median stop along its heading, its bearing the mean approach bearing 4 s before each stop.

Each finished drive is read once. Its events go to EVENTS_PATH, so spots outlive the logs the deleter removes, and
STOP_SPOTS_PATH is rewritten when the spots change; plannerd reads it on the next start.
"""
import glob
import json
import math
import os
import re
import time
from pathlib import Path

import numpy as np

from openpilot.common.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.stop_spots import (EARTH_M_PER_DEG, GPS_MAX_ACCURACY, HEADING_TOL,
                                                                    STOP_SPOTS_PATH, along_cross, finite, heading_diff)

EVENTS_PATH = Path("/data/stop_spots_events.json")
TSR_ADDR = 0x35F
TSR_STOP_SIGN = 9
TSR_BUSES = (2, 130)
STOP_SPEED = 0.6  # m/s
MOVING_SPEED = 4.  # m/s
TSR_WINDOW = 15.  # s
TSR_AT_STOP = 1.5  # s either side of the stop the sign must still show
TSR_GAP = 2.  # s; TSR frames further apart are separate sightings
THROUGH_SPEED = 2.7  # m/s, 6 mph: the driver's rolling stops here were 1.5-5.3 mph
THROUGH_RATIO = 2
MATCH_DIST = 20.  # m
MIN_ROUTES = 2  # drives; two stops in one pass are one sighting
SETTLE_TIME = 120.  # s since a drive's last write before it is read
POLL_TIME = 60.  # s
SEGMENT_RE = re.compile(r"^(.+)--(\d+)$")


def segment_files(route_dir: str) -> list[str]:
  """A route's rlogs in segment order, from <n>.zst, <n>/rlog.zst or <route>--<n>/rlog.zst (the PC tool's layouts)."""
  files = glob.glob(os.path.join(route_dir, "*.zst")) + glob.glob(os.path.join(route_dir, "*", "rlog.zst"))

  def seg(f):
    m = re.search(r"(\d+)(?:\.zst|/rlog\.zst)$", f)
    return int(m.group(1)) if m else 0
  return sorted(files, key=seg)


def route_events(files: list[str], route: str) -> list[dict]:
  from openpilot.tools.lib.logreader import LogReader
  t0, cs, lead, gps, tsr = None, [], [], [], []
  for f in files:
    try:
      for m in LogReader(f):
        t = m.logMonoTime * 1e-9
        t0 = t if t0 is None else t0
        t -= t0
        w = m.which()
        if w == "carState":
          cs.append((t, m.carState.vEgo))
        elif w == "radarState":
          lead.append((t, m.radarState.leadOne.present))
        elif w in ("gpsLocationExternal", "gpsLocation"):
          g = getattr(m, w)
          if g.hasFix and 0. < g.horizontalAccuracy <= GPS_MAX_ACCURACY:
            gps.append((t, g.latitude, g.longitude, g.bearingDeg))
        elif w == "can":
          for c in m.can:
            if c.address == TSR_ADDR and c.src in TSR_BUSES and len(c.dat) > 3 and (c.dat[3] >> 4) & 0xF == TSR_STOP_SIGN:
              tsr.append(t)
    except Exception:
      cloudlog.exception(f"stop_spots_learner: unreadable {f}")
  if not cs or not gps:
    return []
  cs, lead, gps, tsr = np.array(cs), np.array(lead or [(0., 0.)]), np.array(gps), np.array(tsr)
  t, v = cs[:, 0], cs[:, 1]
  events, i = [], 1
  while i < len(t):
    if v[i] < STOP_SPEED <= v[i - 1]:
      ts = t[i]
      moving = v[(t > ts - 20.) & (t < ts)].max(initial=0.) > MOVING_SPEED
      led = lead[(lead[:, 0] > ts - 3.) & (lead[:, 0] <= ts), 1].max(initial=0.) > 0.
      seen = len(tsr) and ((tsr >= ts - TSR_AT_STOP) & (tsr <= ts + TSR_AT_STOP)).any()
      gi, gb = np.argmin(abs(gps[:, 0] - ts)), np.argmin(abs(gps[:, 0] - (ts - 4.)))
      if moving and not led and seen and abs(gps[gi, 0] - ts) < 1. and abs(gps[gb, 0] - (ts - 4.)) < 1.:
        events.append({"lat": float(gps[gi, 1]), "lon": float(gps[gi, 2]), "bearing": float(gps[gb, 3]),
                       "route": route, "t": round(float(ts), 1), "kind": "stop"})
      while i < len(t) and v[i] < STOP_SPEED:
        i += 1
    i += 1
  # sightings passed without slowing: where the sign went out of view
  for seg in np.split(tsr, np.where(np.diff(tsr) > TSR_GAP)[0] + 1) if len(tsr) else []:
    w = (t >= seg[0]) & (t <= seg[-1] + TSR_GAP)
    gi = np.argmin(abs(gps[:, 0] - seg[-1]))
    if w.any() and v[w].min() >= THROUGH_SPEED and abs(gps[gi, 0] - seg[-1]) < 1.:
      events.append({"lat": float(gps[gi, 1]), "lon": float(gps[gi, 2]), "bearing": float(gps[gi, 3]),
                     "route": route, "t": round(float(seg[-1]), 1), "kind": "through"})
  return events


def _stop(e: dict) -> bool:
  return e.get("kind", "stop") == "stop"


def cluster(events: list[dict]) -> list[dict]:
  groups: list[list[dict]] = []
  for e in sorted(events, key=lambda e: (e["route"], e["t"])):
    for g in groups:
      c = {"lat": np.mean([q["lat"] for q in g]), "lon": np.mean([q["lon"] for q in g]), "bearing": g[0]["bearing"]}
      if heading_diff(e["bearing"], c["bearing"]) <= HEADING_TOL and math.hypot(*along_cross(c, e["lat"], e["lon"])) <= MATCH_DIST:
        g.append(e)
        break
    else:
      groups.append([e])
  spots = []
  for g in groups:
    stops = [q for q in g if _stop(q)]
    stop_routes, through_routes = {q["route"] for q in stops}, {q["route"] for q in g if not _stop(q)} - {q["route"] for q in stops}
    if len(stop_routes) < MIN_ROUTES or len(stop_routes) < THROUGH_RATIO * len(through_routes):
      continue
    g = stops
    b = math.degrees(math.atan2(np.mean([math.sin(math.radians(q["bearing"])) for q in g]),
                                np.mean([math.cos(math.radians(q["bearing"])) for q in g]))) % 360.
    c = {"lat": float(np.mean([q["lat"] for q in g])), "lon": float(np.mean([q["lon"] for q in g])), "bearing": b}
    along = float(np.median([along_cross(c, q["lat"], q["lon"])[0] for q in g]))
    br = math.radians(b)
    lat = c["lat"] + along * math.cos(br) / EARTH_M_PER_DEG
    lon = c["lon"] + along * math.sin(br) / (EARTH_M_PER_DEG * math.cos(math.radians(c["lat"])))
    spots.append({"lat": round(lat, 7), "lon": round(lon, 7), "bearing": round(b, 1), "events": len(g),
                  "routes": sorted({q["route"] for q in g})})
  return spots


def _valid(e) -> bool:
  return isinstance(e, dict) and all(finite(e.get(k)) for k in ("lat", "lon", "bearing", "t")) and \
    isinstance(e.get("route"), str) and e.get("kind", "stop") in ("stop", "through")


def load_events(path: Path = EVENTS_PATH) -> dict[str, list[dict]]:
  """Per-drive events; anything malformed is dropped, so a hand edit cannot stop the learning."""
  try:
    routes = json.loads(path.read_text()).get("routes", {})
  except (OSError, ValueError, AttributeError):
    return {}
  if not isinstance(routes, dict):
    return {}
  return {r: [e for e in evs if _valid(e)] for r, evs in routes.items() if isinstance(r, str) and isinstance(evs, list)}


def write_json(path: Path, data) -> None:
  tmp = path.with_suffix(".tmp")
  tmp.write_text(json.dumps(data, indent=1))
  os.replace(tmp, path)


def finished_routes(log_root: str, now: float) -> dict[str, list[str]]:
  """Drives whose last segment has not been written to for SETTLE_TIME, with their rlogs in order."""
  routes: dict[str, list[tuple[int, str, float]]] = {}
  for d in os.listdir(log_root):
    m = SEGMENT_RE.match(d)
    rlog = os.path.join(log_root, d, "rlog.zst")
    if m and os.path.isfile(rlog):
      routes.setdefault(m.group(1), []).append((int(m.group(2)), rlog, os.path.getmtime(rlog)))
  return {r: [f for _, f, _ in sorted(segs)] for r, segs in routes.items() if now - max(s[2] for s in segs) >= SETTLE_TIME}


def learn(log_root: str, events_path: Path = EVENTS_PATH, spots_path: Path = STOP_SPOTS_PATH, now: float | None = None) -> bool:
  """Read every finished drive not yet read; rewrite the spots if they changed. Returns whether they did."""
  known = load_events(events_path)
  before = cluster([e for evs in known.values() for e in evs])
  # file mtimes are wall-clock time
  for route, files in finished_routes(log_root, time.time() if now is None else now).items():  # noqa: TID251
    if route in known:
      continue
    known[route] = route_events(files, route)
    write_json(events_path, {"routes": known})
    cloudlog.info(f"stop_spots_learner: {route}: {len(known[route])} stops at a stop sign")
  spots = cluster([e for evs in known.values() for e in evs])
  if spots == before and spots_path.exists():
    return False
  write_json(spots_path, {"spots": spots})
  cloudlog.info(f"stop_spots_learner: {len(spots)} spots")
  return True


def main():
  os.nice(10)
  while True:
    try:
      learn(Paths.log_root())
    except Exception:
      cloudlog.exception("stop_spots_learner failed")
    time.sleep(POLL_TIME)


if __name__ == "__main__":
  main()
