"""
Stop spots: stop lines learned from the driver's own stops, braked to when the model does not see them.

At the southbound stop sign 42.97205,-87.88467 the model only saw the stop 30-43 m out on 14 of 31 passes and planned
to slow to 7-16 mph, then speed back up; the driver braked hard every time (routes 00000166-0000002b). The car's TSR
showed the sign 0.6-1.4 s before the driver braked, too late to act on, and OSM has almost no stop signs tagged here.
stop_spots_learner writes STOP_SPOTS_PATH from the drives; plannerd reads it at start. A spot is the stop line, as the
road camera saw it (stop_line.py) on at least two drives, and only such a "verified" spot brakes: where the driver
comes to a full stop is not the line (at this crossroad the full stops end several metres past it, route 0000002d).
On the approach the camera places the line itself once it sees it (LINE_TOL, LINE_FRAMES): GPS puts the same line a
few metres apart from one day to the next, so it only says which stop sign is coming.

On the approach to a spot, with a GPS fix, this candidate brakes to stop the car STOP_MARGIN short of the spot as the
driver does: none until it needs STOP_DECEL_BP[0], firmer toward the end, finished at FINISH_DECEL, never more than
STOP_ASK_MAX, built at START_JERK; down to SEEK_SPEED near the GPS line and LOOK_SPEED at it until the camera has
placed the line. A stop found needing more than START_MAX is left to the driver. It starts only on ARM_FIXES agreeing
fixes within ARM_ACCURACY. Once it brakes, the distance left is the car's own odometry: a missed, wandering or stale
fix (the Jetson fallback projects 1-3 s old ones) can neither release the brake nor move the line; fixes then only end
a pass whose car turned off the spot's road. Stopped there, it asks to stop for HOLD_TIME at standstill, then lets go
and the model decides when to move on; that spot is done until the car is DONE_DIST past it. The driver's gas or
brake, reaching it without having braked for it, or PASS_MAX_TIME ends the pass. The driver's gas while rolling only
pauses it: on release it resumes if the stop still needs RESUME_MAX or less and is RESUME_DIST or more away (any
distance under RESUME_SLOW); gas from a stop hands the go to the driver. A lead needs nothing here: the lead MPC is
the lower candidate behind it, and with it gone the car still stops at the line.
"""
import json
import math

import numpy as np
from pathlib import Path

from openpilot.common.realtime import DT_MDL

STOP_SPOTS_PATH = Path("/data/stop_spots.json")

EARTH_M_PER_DEG = 111320.
GPS_MAX_AGE = 1.5  # s; before it brakes, a pass ends past this without a fix
# The brake starts from fixes only: ARM_FIXES in a row within ARM_ACCURACY that agree with the odometry to AGREE_TOL.
# The ublox read 1.5-3.9 m on 29 of 32 approaches to the learned stop; route 0000002b (12 m, the Jetson fallback's
# projected fixes) and 0000016b (10-66 m) would not start it.
ARM_ACCURACY = 5.  # m
ARM_FIXES = 3
AGREE_TOL = 3.  # m
GPS_MAX_ACCURACY = 10.  # m; once braking, fixes this good only end a pass that has left the spot's road
LEAVE_CROSS = 15.  # m off the spot's line
LEAVE_HEADING = 45.  # deg off its heading, with the fix itself moving at LEAVE_SPEED or more (the Jetson fallback
# republishes a standstill fix with bearing 0 for 1-3 s after the car pulls away)
LEAVE_SPEED = 3.  # m/s
LEAVE_FIXES = 2
LEAVE_STEER = 120.  # deg of wheel short of the stop (about a 20 m radius): turning off before the line, faster than
# the fixes show it; a swerve round a parked car or a bend stays under it
PASS_MAX_TIME = 90.  # s braked-for pass on the move, at most
HEADING_TOL = 30.  # deg
CROSS_TOL = 8.  # m either side of the spot's heading line
ARM_DIST = 120.  # m ahead; nothing further out is looked at
START_JERK = 2.0  # m/s3 the brake builds at
START_MAX = 2.0  # m/s2; a stop needing more when found (a fix that jumped or came late) is the driver's
BRAKE_MAX = 3.0  # m/s2; once braking, a car delivering less than asked is still stopped at the line
# m/s2 a stop may need when the driver's gas is released: up to where the stop asks all it needs (STOP_DECEL_BP[1]),
# the ask never past STOP_ASK_MAX (route 0000002c t=174: a 1 s tap 46 m out at 27 mph, released needing 1.5-1.6)
RESUME_MAX = 2.
RESUME_DIST = 10.  # m from the aim at least: released closer, the ramp from 0 plus actuator lag reached 2-3 m/s2
STOP_SPEED = 0.5  # m/s
HOLD_TIME = 2.  # s at standstill before the model takes over
HOLD_DIST = 4.  # m; stopped further short than this is not the stop: a creep on is stopped again at the spot
HOLD_ACCEL = -1.0  # m/s2 the ask builds to while stopping at the spot
DONE_DIST = 30.  # m beyond a spot before it can start a pass again
# m short of the spot (the stop line) the stop aims at, for the camera: it sits about 2 m behind the front bumper, so
# the bumper stops about 1 m before the line
STOP_MARGIN = 3.
# The camera's line (stop_line.py) places the stop once seen. GPS is a few m off from one day to the next (the same
# line came out 6-7.5 m apart on routes 0000002c and 0000002d), so once the GPS fixes agree on the spot, the first band
# (the nearest to the car) from LINE_BEFORE before to LINE_TOL past where the spot puts the line, within LOCK_MAX_DIST, seen LINE_FRAMES
# times within LINE_AGREE of itself while driving straight, is the line: a stop line, or a crosswalk's near edge
# where there is none. Everything painted after it (the crosswalk's far edge, the cross street) is past the stop.
# Nearer than LOCK_MAX_DIST the flat-ground distance is good: a 0.5 deg nose-dive or grade moves a band at 25 m by 4 m,
# at 10 m by 0.6 m. It is then tracked within LINE_TRACK at LINE_GAIN, never more than LINE_DRIFT from where it locked.
# A lead reads as a band about 1.8x its gap (route 0000002d at 3-8 s: a band at
# 16-17 m moving with the lead), and a tracked lead pulls the stop into the junction: bands from LEAD_BAND of the lead's
# distance on are its, not road. The radar loses a close lead for up to 1.2 s (0000002c t=154.9 at 5.9 m, 0000002d
# t=382.5), so the last lead stands for LEAD_HOLD after it goes. Road before the lead stays readable, as in a queue.
# m past the GPS line a band may be the line: GPS 7.8 m early seen (route 0000002d northbound); further on is the far
# side of the junction, taken for the line were the line itself not seen
LINE_TOL = 10.
# m before it: GPS drifted to 10.4 m late turning in (route 00000030 northbound, 85 -> 90 m on the approach), and the
# nearest other band seen driving straight was 19 m before its line (route 00000178 northbound)
LINE_BEFORE = 15.
LOCK_MAX_DIST = 20.  # m
LINE_FRAMES = 3
LINE_AGREE = 1.  # m
LINE_TRACK = 1.  # m
LINE_GAIN = 0.5
LINE_DRIFT = 2.  # m
LINE_LATENCY = 0.1  # s from the frame to the planner
LEAD_BAND = 0.8
LEAD_HOLD = 2.  # s
# Bands seen within LOCK_MAX_DIST are kept BAND_MEMORY, odometry-carried, also once under the hood: a near band seen
# after the lock (worn paint) moves the stop back to it, never on. Route 00000030 northbound, before: the band nearest
# to where the GPS put the line was the crosswalk's far edge, and the car stopped on the crosswalk.
BAND_MEMORY = 3.  # s
# m the car travels between a band's first and last sighting, the band coming LINE_CLOSING of it closer, before it
# counts: a bright spot that keeps its distance (a headlight's glare, a reflection off the hood, a car the radar does
# not track) moves with the car, not the road, and three sightings a frame apart matched it (review 2026-11-03: a
# constant band at 12 m locked 25 m out and stopped the car 17 m short). Worn paint is read over as little as 2 m
# (route 00000030 northbound: a near edge only from 8-10 m); readings move about 0.4 m under braking (route 00000030)
LINE_SPAN = 1.5
LINE_CLOSING = 0.5
# deg of wheel above which a band is not read: turning in, the camera sees the road at an angle (route 00000167
# northbound: 11 of 11 bands 20-30 m before the line seen turning)
STRAIGHT_STEER = 45.
LOCK_JERK = 4.  # m/s3 the brake builds at to the camera's line: GPS off 7 m toward early, 2.0 left the stop 1.1 m past
LOCK_JERK_TIME = 1.  # s after the camera places or moves the line; then START_JERK again
RESTEP_MIN_AIM = 2.  # m: a move back after the lock only to an aim this far ahead
RESTEP_EDGE = 1.  # m short of the line, at least, a late move back stops
# The stop's shape, from the driver's own stop-sign stops (routes 00000009-00000032: 39 stops, 21 straight): braking
# from where the stop needs about 1.3 m/s2, harder toward the end, peak 2.1 m/s2 median, 5 mph to standstill in 1.1 s,
# under 2 m/s for 0.9 s. A constant decel from wherever the stop was found crawled for 2-9 s on 18 of 32 northbound
# passes (the camera moving the aim on after a gentle start) and eased off toward the end everywhere. So the ask is
# none while the stop needs under STOP_DECEL_BP[0] (the speed held), all of what it needs from STOP_DECEL_BP[1] on:
# the stop settles there and ends firm.
STOP_DECEL_BP = [1.0, 2.0]  # m/s2
# The last metres: under FINISH_SPEED, once a stop at FINISH_DECEL from here (after the car's LAG_TIME) ends at the aim,
# it is finished at FINISH_DECEL at least, not eased into a crawl as the car's own lag leaves it a little ahead of what
# the stop needs
FINISH_SPEED = 2.  # m/s
FINISH_DECEL = 1.5  # m/s2 (the driver's 5 mph to standstill in 1.1-1.6 s)
# Until the camera has placed the line the stop is only where GPS puts it, 7.8 m early to 11 m late: the car comes down
# to LOOK_SPEED by there, not to a stop, so a line the camera finds on (northbound route 0000019c: braked to a stop 9 m
# short, the line seen only then) is still stopped at; with none seen by the GPS aim, it stops there
LOOK_SPEED = 1.8  # m/s, under FINISH_SPEED: the finish takes over before the ask to it eases to 0
# s from an ask to the car delivering it (0.3 s lag + 0.2 s dead time, fitted on 21 braking windows): the stop is
# planned from where the car will be, so the ask does not climb at the end (24 of 64 passes peaked 2.5-2.9 there)
LAG_TIME = 0.3
# Until the camera has placed the line, the car is down to SEEK_SPEED by SEEK_DIST short of where the GPS puts it:
# GPS put the southbound line up to 11 m late (p90 6 m, 19 passes), and wet or at dusk the camera reads it from 12-14 m
# only; from SEEK_SPEED a line found there is stopped for at no more than about 2 m/s2. Only a cap: the stop itself
# stays at the GPS line until the camera moves it, so a line past early GPS is still found.
SEEK_SPEED = 5.  # m/s
SEEK_DIST = 14.  # m
# m/s2 the cap needs before it brakes, and from where it asks all of it: an earlier, gentler slowdown (from about 115 m
# at 27 mph) scored the same but crawls into the junction in traffic; from 1.5 it starts about where the driver does
SEEK_DECEL_BP = [1.5, 2.2]
AHEAD_AGE = 0.3  # s since a band's last sighting for it to count as paint ahead
AHEAD_SPAN = 0.5  # m travelled over its sightings, coming LINE_CLOSING of it closer, for it to count
SEEK_TAU = 1.  # s
HOLD_ON_SPEED = 4.  # m/s
# s a fix lags the car: the southbound line came out a median 2.3 m late at 8-12 m/s (19 passes with the camera's line)
GPS_LAG = 0.3
# m/s2 the stop asks at most once braking: a line placed late by the camera is stopped at this, the bumper then up to
# about 1 m further on than the aim (still short of the band); BRAKE_MAX remains the bound a stop is placed within
STOP_ASK_MAX = 2.5
# m/s a gas released under this is a crawl: the stop resumes when it needs RESUME_MAX or less, however near (a gas tap
# at a crawl ended the stop and the car rolled through it, route 00000032 northbound)
RESUME_SLOW = 5.


def finite(x) -> bool:
  return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def load_spots(path: Path = STOP_SPOTS_PATH) -> list[dict]:
  try:
    spots = json.loads(path.read_text()).get("spots", [])
  except (OSError, ValueError, AttributeError):
    return []
  if not isinstance(spots, list):
    return []
  return [s for s in spots if isinstance(s, dict) and all(finite(s.get(k)) for k in ("lat", "lon", "bearing")) and
          abs(s["lat"]) <= 90. and abs(s["lon"]) <= 180. and s.get("verified") is True]


def along_cross(spot: dict, lat: float, lon: float) -> tuple[float, float]:
  """Metres from a spot along its heading (negative = before it) and across it."""
  dn = (lat - spot["lat"]) * EARTH_M_PER_DEG
  de = (lon - spot["lon"]) * EARTH_M_PER_DEG * math.cos(math.radians(spot["lat"]))
  b = math.radians(spot["bearing"])
  return dn * math.cos(b) + de * math.sin(b), -dn * math.sin(b) + de * math.cos(b)


def heading_diff(a: float, b: float) -> float:
  return abs((a - b + 180.) % 360. - 180.)


# a band: [m ahead (mean of its sightings, odometry-carried), sightings, s since the last, m travelled since the first,
# where the first was seen, m travelled at the last, how much closer the last was than the first]
def carry_bands(bands: list[list[float]], v_ego: float) -> list[list[float]]:
  """One cycle on: odometry-carried and aged; gone BAND_MEMORY after the last sighting."""
  return [[b[0] - v_ego * DT_MDL, b[1], b[2] + DT_MDL, b[3] + v_ego * DT_MDL, b[4], b[5], b[6]]
          for b in bands if b[2] + DT_MDL <= BAND_MEMORY]


def see_band(bands: list[list[float]], d: float) -> None:
  match = min(bands, key=lambda b: abs(b[0] - d), default=None)
  if match is not None and abs(match[0] - d) <= LINE_AGREE:
    match[0], match[1], match[2] = (match[0] * match[1] + d) / (match[1] + 1), match[1] + 1, 0.
    match[5], match[6] = match[3], match[4] - d
  else:
    bands.append([d, 1, 0., 0., d, 0., 0.])


def firm_band(b: list[float]) -> bool:
  """Seen LINE_FRAMES times over LINE_SPAN of travel, coming closer as the road does."""
  return b[1] >= LINE_FRAMES and b[5] >= LINE_SPAN and b[6] >= LINE_CLOSING * b[5]


class StopSpots:
  def __init__(self, spots: list[dict] | None = None):
    self.spots = load_spots() if spots is None else spots
    self.spot = None  # the spot of this pass
    self.dist = 0.  # m to where the stop aims (STOP_MARGIN short of it): from fixes, then odometry once braking
    self.fix_age = 0.
    self.committed = False  # braked for it once: the rest of the pass is odometry
    self.agree = 0  # fixes in a row that place the spot where the odometry does
    self.off_fixes = 0  # fixes in a row, once committed, off the spot's road
    self.pass_time = 0.
    self.paused = False  # the driver's gas is held: no brake until it is released
    self.bands: list[list[float]] = []  # the camera's bands (carry_bands)
    self.expect = 0.  # m to where the GPS puts the line, odometry-carried
    self.lock_age = 0.  # s since the camera placed or moved the line
    self.line_locked = False  # the stop is placed by the camera's line: GPS no longer moves it
    self.line_anchor = 0.  # m to where the stop aimed when the line locked, odometry-carried
    self.lead_dist: float | None = None  # m to the last lead the radar had
    self.lead_age = 0.  # s since then
    self.braking = False
    self.a_target = 0.
    self.hold_frames = 0
    self.done_spot = None  # held or handed to the driver: no new pass from it until DONE_DIST past it
    self.done_dist = 0.
    self.should_stop = False

  def _locate(self, lat: float, lon: float, bearing: float, speed: float = 0.) -> None:
    """The nearest spot ahead on this heading, or none."""
    best = None
    for spot in self.spots:
      if spot is self.done_spot or heading_diff(bearing, spot["bearing"]) > HEADING_TOL:
        continue
      along, cross = along_cross(spot, lat, lon)
      if abs(cross) <= CROSS_TOL and -ARM_DIST <= along <= 0. and (best is None or -along < best[1]):
        best = (spot, -along)
    if best is None:
      self._end()
      return
    if best[0] is not self.spot:
      self._end()
      self.spot = best[0]
    dist = best[1] - speed * GPS_LAG - STOP_MARGIN
    self.expect = best[1] - speed * GPS_LAG
    self.agree = self.agree + 1 if abs(dist - self.dist) <= AGREE_TOL else 1
    self.fix_age = 0.
    if not self.line_locked:
      self.dist = dist

  def _see_line(self, line_dists: list[float], v_ego: float, steer_deg: float) -> None:
    """The camera's bands across the lane (m ahead, nearest first) place, then track, the line."""
    line = self.dist + STOP_MARGIN
    lead = self.lead_dist if self.lead_dist is not None and self.lead_age <= LEAD_HOLD else None
    dists = [d - v_ego * LINE_LATENCY for d in line_dists if lead is None or d < LEAD_BAND * lead]
    for d in (d for d in dists if d <= LOCK_MAX_DIST and abs(steer_deg) <= STRAIGHT_STEER):
      see_band(self.bands, d)
    if not (self.committed or self.agree >= ARM_FIXES):
      return
    near = [b[0] for b in self.bands if firm_band(b) and -LINE_BEFORE <= b[0] - self.expect <= LINE_TOL]
    if not near:
      return
    seen = min(near)  # the first: what is painted after it is past the stop
    if self.line_locked:
      if abs(seen - line) <= LINE_TRACK:
        dist = self.dist + LINE_GAIN * (seen - line)
        self.dist = min(max(dist, self.line_anchor - LINE_DRIFT), self.line_anchor + LINE_DRIFT)
      elif seen < line:
        # a band before the line, seen late (worn paint): only to an aim BRAKE_MAX makes, its margin shrunk to what
        # BRAKE_MAX allows (short of the band beats on it), and never further on than the aim it has
        aim = max(seen - STOP_MARGIN, min(v_ego ** 2 / (2. * START_MAX), seen - RESTEP_EDGE))
        if RESTEP_MIN_AIM <= aim < self.dist and v_ego ** 2 / (2. * aim) <= BRAKE_MAX:
          self.dist = self.line_anchor = aim
          self.lock_age = 0.
      return
    self.dist, self.line_locked = seen - STOP_MARGIN, True
    self.line_anchor, self.lock_age = self.dist, 0.
    if not self.committed:
      # the camera's line is the stop's: kept even when close, up to BRAKE_MAX; beyond that it is the driver's
      if v_ego ** 2 / (2. * max(self.dist, 0.5)) > BRAKE_MAX:
        self._finish()
      else:
        self.committed = True

  def _left_road(self, lat: float, lon: float, bearing: float, fix_speed: float) -> bool:
    _, cross = along_cross(self.spot, lat, lon)
    off = abs(cross) > LEAVE_CROSS or (fix_speed >= LEAVE_SPEED and heading_diff(bearing, self.spot["bearing"]) > LEAVE_HEADING)
    self.off_fixes = self.off_fixes + 1 if off else 0
    return self.off_fixes >= LEAVE_FIXES

  def _end(self) -> None:
    self.spot, self.committed, self.braking, self.a_target, self.hold_frames = None, False, False, 0., 0
    self.agree, self.off_fixes, self.pass_time, self.paused = 0, 0, 0., False
    self.bands, self.line_locked, self.line_anchor = [], False, 0.

  def _finish(self) -> None:
    """The pass is over at its spot: latch it until the car is well past."""
    self.done_spot, self.done_dist = self.spot, self.dist + STOP_MARGIN
    self._end()

  def update(self, lat: float, lon: float, bearing: float, accuracy: float, gps_fresh: bool, v_ego: float,
             standstill: bool, active: bool, driver_override: bool, steer_deg: float = 0.,
             fix_speed: float | None = None, line_dists: list[float] | None = None,
             lead_dist: float | None = None) -> float | None:
    """The stop's accel candidate this cycle, or None. gps_fresh: a new fix with hasFix this cycle; accuracy: its
    horizontal accuracy in m; fix_speed: its own speed (v_ego when not given); line_dists: the camera's bands across the
    lane this cycle (m ahead, nearest first), None without a new frame; lead_dist: the lead's, when there is one."""
    self.should_stop = False
    if not self.spots:
      return None
    self.fix_age += DT_MDL
    self.dist -= v_ego * DT_MDL
    self.bands = carry_bands(self.bands, v_ego)
    self.line_anchor -= v_ego * DT_MDL
    self.expect -= v_ego * DT_MDL
    self.lock_age += DT_MDL
    if lead_dist is not None:
      self.lead_dist, self.lead_age = lead_dist, 0.
    else:
      self.lead_age += DT_MDL
    if self.done_spot is not None:
      self.done_dist -= v_ego * DT_MDL
      if self.done_dist < -DONE_DIST:
        self.done_spot = None
    if not self.committed and not self.line_locked:
      if gps_fresh and 0. < accuracy <= ARM_ACCURACY:
        self._locate(lat, lon, bearing, v_ego if fix_speed is None else fix_speed)
      elif self.spot is not None and self.fix_age > GPS_MAX_AGE:
        self._end()
    elif (gps_fresh and 0. < accuracy <= GPS_MAX_ACCURACY and
          self._left_road(lat, lon, bearing, v_ego if fix_speed is None else fix_speed)) or \
         (abs(steer_deg) > LEAVE_STEER and self.dist > HOLD_DIST):
      # turned off before the line (a side street, a driveway): not this stop
      self._finish()
    if self.spot is None:
      return None
    if line_dists is not None:
      self._see_line(line_dists, v_ego, steer_deg)
    if self.committed and v_ego >= STOP_SPEED:  # moving only: a long wait in a queue short of the line is not the pass
      self.pass_time += DT_MDL
      if self.pass_time > PASS_MAX_TIME:
        self._finish()
        return None
    if not active:
      # the driver's brake, or disengaged: the pass is theirs
      self._finish()
      return None
    if driver_override:
      if v_ego < STOP_SPEED:
        # gas from a stop or a crawl (holding at the line, creeping in a queue): the go is the driver's
        self._finish()
        return None
      # the driver's gas while rolling: no brake while it is held (route 0000002c t=174: a 1 s tap 46 m out ended the
      # stop, and the model braked -2.1 at the line)
      self.paused, self.braking, self.a_target, self.hold_frames = True, False, 0., 0
      return None
    if self.paused:
      self.paused = False
      if (self.dist < RESUME_DIST and v_ego >= RESUME_SLOW) or self.dist < 0.5 or v_ego ** 2 / (2. * max(self.dist, 0.5)) > RESUME_MAX:
        # released at the line, or too close to it to stop gently: the driver's
        self._finish()
        return None

    if v_ego < STOP_SPEED and self.braking and self.dist > HOLD_DIST:
      # stopped or crawling short (the model's own stop): let it be, and stop the creep on at the spot
      self.braking, self.a_target, self.hold_frames = False, 0., 0
      return None
    if v_ego < STOP_SPEED and self.braking and self.dist <= HOLD_DIST:
      # at the spot: stop and hold, then hand the go to the model
      self.hold_frames = self.hold_frames + 1 if standstill else 0
      if self.hold_frames * DT_MDL >= HOLD_TIME:
        # held where GPS or the camera put the line: the go is the model's. Not stopped again for a band seen on: the
        # line itself is under the hood by then, and the first band in view is the crosswalk's far edge (review
        # 2026-11-03: a second stop on the crosswalk)
        self._finish()
        return None
      self.should_stop = True
      # to HOLD_ACCEL at START_JERK from either side: a stop braked harder than it eases off, no step
      self.a_target += min(max(HOLD_ACCEL - self.a_target, -START_JERK * DT_MDL), START_JERK * DT_MDL)
      return self.a_target
    # one stray fix neither starts nor ends anything; the camera's line is sure on its own (route 0000002d northbound:
    # GPS 4.5 m off the line it saw, so no fix agreed with it, and GPS would have passed the spot before the line)
    sure = self.committed or self.line_locked or self.agree >= ARM_FIXES
    turning = abs(steer_deg) > LEAVE_STEER  # not yet braked for: a turn onto the road, or off it, starts nothing
    if self.dist <= 0. and not self.braking and sure:
      # reached without needing to brake (or the fix puts it behind): nothing to stop for
      self._finish()
      return None
    d_eff = max(self.dist - v_ego * LAG_TIME, 0.5)
    need = v_ego ** 2 / (2. * d_eff)
    if not self.committed and sure and need > START_MAX:
      # found too late to stop gently: the driver's
      self._finish()
      return None
    # paint glimpsed ahead within where the line can be: a recent sighting coming closer (a glare that keeps its
    # distance does not; review 2026-11-03: one let the car roll through)
    ahead = not self.line_locked and any(STOP_MARGIN < b[0] <= self.expect + LINE_TOL and b[2] <= AHEAD_AGE and
                                         b[5] >= AHEAD_SPAN and b[6] >= LINE_CLOSING * b[5] for b in self.bands)
    if self.braking and self.dist < -HOLD_DIST and not ahead:
      # well past the line still rolling (a car short of what was asked, a stop found too late): the model's, not a
      # hard stop in the junction
      self._finish()
      return None
    seek = 0.
    if not self.line_locked and v_ego > SEEK_SPEED:
      to_seek = self.dist - SEEK_DIST - v_ego * LAG_TIME
      # down to SEEK_SPEED by SEEK_DIST; inside it, shed what is over in SEEK_TAU rather than all at once (review
      # 2026-11-03: braked at 2.5 to under SEEK_SPEED, let off, then braked again at 13-18 mph)
      seek = (v_ego ** 2 - SEEK_SPEED ** 2) / (2. * to_seek) if to_seek > SEEK_TAU * v_ego else (v_ego - SEEK_SPEED) / SEEK_TAU
    finish_dist = v_ego ** 2 / (2. * FINISH_DECEL) + v_ego * (LAG_TIME + FINISH_DECEL / (2. * START_JERK))  # lag + build
    crawl_in = v_ego < FINISH_SPEED and self.dist < finish_dist  # a crawl the model lets roll on to the stop
    was_braking = self.braking
    if self.braking or (sure and not turning and (need >= STOP_DECEL_BP[0] or seek >= SEEK_DECEL_BP[0] or crawl_in)):
      self.braking = self.committed = True
      # past where GPS puts the line with paint glimpsed ahead: roll on at LOOK_SPEED until the camera places it; with
      # nothing seen, the stop is there
      if self.line_locked or (self.dist <= 0. and not ahead):
        stop = need
      else:
        stop = max(v_ego ** 2 - LOOK_SPEED ** 2, 0.) / (2. * d_eff)  # down to LOOK_SPEED at the GPS aim
      ask = max(stop * float(np.interp(stop, STOP_DECEL_BP, [0., 1.])), seek * float(np.interp(seek, SEEK_DECEL_BP, [0., 1.])))
      if was_braking and v_ego > HOLD_ON_SPEED:
        # once braking, never let off below what the stop still needs: the cap's and the stop's onsets both ease in,
        # and where one hands over to the other the ask fell to 0 and braked again (review 2026-11-03, 20-31 mph,
        # around SEEK_SPEED); slower, the stop's own ease-in shapes the end (held on there, it crawled longer)
        ask = max(ask, min(stop, -self.a_target))
      if v_ego < FINISH_SPEED and not ahead and self.dist < finish_dist:
        ask = max(ask, FINISH_DECEL)
      jerk = LOCK_JERK if self.line_locked and self.lock_age < LOCK_JERK_TIME else START_JERK
      target = -min(ask, STOP_ASK_MAX)
      self.a_target += min(max(target - self.a_target, -jerk * DT_MDL), START_JERK * DT_MDL)
      return self.a_target
    return None
