"""
Stop spots: stop lines learned from the driver's own stops, braked to when the model does not see them.

At the southbound stop sign 42.97205,-87.88467 the model only saw the stop 30-43 m out on 14 of 31 passes and planned
to slow to 7-16 mph, then speed back up; the driver braked hard every time (routes 00000166-0000002b). The car's TSR
showed the sign 0.6-1.4 s before the driver braked, too late to act on, and OSM has almost no stop signs tagged here.
The local learner (tools/mazda_long/learn_stop_spots.py, outside git) writes STOP_SPOTS_PATH: a spot where the TSR
showed a stop sign and the car stopped with no lead, on at least two drives from the same heading, at the median of
those stops. plannerd reads it at start.

On the approach to a spot, with a GPS fix, this candidate asks the constant decel that stops the
car STOP_MARGIN short of the spot once that is START_DECEL or more (about 70 m out from 27 mph), or within START_TIME of
it (a car the model slowed to a crawl and lets roll on), built at START_JERK. A stop found needing more than START_MAX
is left to the driver. It starts only on ARM_FIXES agreeing fixes within ARM_ACCURACY. Once it brakes, the distance
left is the car's own odometry: a missed, wandering or stale fix (the Jetson fallback projects 1-3 s old ones) can
neither release the brake nor move the line; fixes then only end a pass whose car turned off the spot's road. Stopped
there, it asks to stop for HOLD_TIME at standstill, then lets go and the model decides when to move on; that spot is
done until the car is DONE_DIST past it. The driver's gas or brake, reaching it without having braked for it, or
PASS_MAX_TIME ends the pass. A lead
needs nothing here: the lead MPC is the lower candidate behind it, and with it gone the car still stops at the line.
"""
import json
import math
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
PASS_MAX_TIME = 90.  # s braked-for pass, at most
HEADING_TOL = 30.  # deg
CROSS_TOL = 8.  # m either side of the spot's heading line
ARM_DIST = 120.  # m ahead; nothing further out is looked at
START_DECEL = 1.0  # m/s2 the stop must need before it brakes
START_TIME = 3.  # s from the spot: a car the model already slowed to a crawl is stopped from here
START_JERK = 1.5  # m/s3 the brake builds at
START_MAX = 2.0  # m/s2; a stop needing more when found (a fix that jumped or came late) is the driver's
BRAKE_MAX = 3.0  # m/s2; once braking, a car delivering less than asked is still stopped at the line
STOP_SPEED = 0.5  # m/s
HOLD_TIME = 2.  # s at standstill before the model takes over
HOLD_DIST = 4.  # m; stopped further short than this is not the stop: a creep on is stopped again at the spot
HOLD_ACCEL = -1.0  # m/s2 the ask builds to while stopping at the spot
DONE_DIST = 30.  # m beyond a spot before it can start a pass again
# m short of the spot the stop aims at: a 5 m GPS error toward the line put the replayed stop 5 m past the drivers'
# median, about at the line; the model creeps the rest after the hold
STOP_MARGIN = 2.


def _finite(x) -> bool:
  return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def load_spots(path: Path = STOP_SPOTS_PATH) -> list[dict]:
  try:
    spots = json.loads(path.read_text()).get("spots", [])
  except (OSError, ValueError, AttributeError):
    return []
  if not isinstance(spots, list):
    return []
  return [s for s in spots if isinstance(s, dict) and all(_finite(s.get(k)) for k in ("lat", "lon", "bearing")) and
          abs(s["lat"]) <= 90. and abs(s["lon"]) <= 180.]


def along_cross(spot: dict, lat: float, lon: float) -> tuple[float, float]:
  """Metres from a spot along its heading (negative = before it) and across it."""
  dn = (lat - spot["lat"]) * EARTH_M_PER_DEG
  de = (lon - spot["lon"]) * EARTH_M_PER_DEG * math.cos(math.radians(spot["lat"]))
  b = math.radians(spot["bearing"])
  return dn * math.cos(b) + de * math.sin(b), -dn * math.sin(b) + de * math.cos(b)


def heading_diff(a: float, b: float) -> float:
  return abs((a - b + 180.) % 360. - 180.)


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
    self.braking = False
    self.a_target = 0.
    self.hold_frames = 0
    self.done_spot = None  # held or handed to the driver: no new pass from it until DONE_DIST past it
    self.done_dist = 0.
    self.should_stop = False

  def _locate(self, lat: float, lon: float, bearing: float) -> None:
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
    dist = best[1] - STOP_MARGIN
    self.agree = self.agree + 1 if abs(dist - self.dist) <= AGREE_TOL else 1
    self.dist, self.fix_age = dist, 0.

  def _left_road(self, lat: float, lon: float, bearing: float, fix_speed: float) -> bool:
    _, cross = along_cross(self.spot, lat, lon)
    off = abs(cross) > LEAVE_CROSS or (fix_speed >= LEAVE_SPEED and heading_diff(bearing, self.spot["bearing"]) > LEAVE_HEADING)
    self.off_fixes = self.off_fixes + 1 if off else 0
    return self.off_fixes >= LEAVE_FIXES

  def _end(self) -> None:
    self.spot, self.committed, self.braking, self.a_target, self.hold_frames = None, False, False, 0., 0
    self.agree, self.off_fixes, self.pass_time = 0, 0, 0.

  def _finish(self) -> None:
    """The pass is over at its spot: latch it until the car is well past."""
    self.done_spot, self.done_dist = self.spot, self.dist + STOP_MARGIN
    self._end()

  def update(self, lat: float, lon: float, bearing: float, accuracy: float, gps_fresh: bool, v_ego: float,
             standstill: bool, active: bool, driver_override: bool, steer_deg: float = 0.,
             fix_speed: float | None = None) -> float | None:
    """The stop's accel candidate this cycle, or None. gps_fresh: a new fix with hasFix this cycle; accuracy: its
    horizontal accuracy in m; fix_speed: its own speed (v_ego when not given)."""
    self.should_stop = False
    if not self.spots:
      return None
    self.fix_age += DT_MDL
    self.dist -= v_ego * DT_MDL
    if self.done_spot is not None:
      self.done_dist -= v_ego * DT_MDL
      if self.done_dist < -DONE_DIST:
        self.done_spot = None
    if not self.committed:
      if gps_fresh and 0. < accuracy <= ARM_ACCURACY:
        self._locate(lat, lon, bearing)
      elif self.spot is not None and self.fix_age > GPS_MAX_AGE:
        self._end()
    elif (gps_fresh and 0. < accuracy <= GPS_MAX_ACCURACY and
          self._left_road(lat, lon, bearing, v_ego if fix_speed is None else fix_speed)) or \
         (abs(steer_deg) > LEAVE_STEER and self.dist > HOLD_DIST):
      # turned off before the line (a side street, a driveway): not this stop
      self._finish()
    if self.spot is None:
      return None
    if self.committed:
      self.pass_time += DT_MDL
      if self.pass_time > PASS_MAX_TIME:
        self._finish()
        return None
    if not active or driver_override:
      # the driver has it: the pass is theirs
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
        self._finish()
        return None
      self.should_stop = True
      self.a_target = max(HOLD_ACCEL, self.a_target - START_JERK * DT_MDL)
      return self.a_target
    sure = self.committed or self.agree >= ARM_FIXES  # one stray fix neither starts nor ends anything
    turning = abs(steer_deg) > LEAVE_STEER  # not yet braked for: a turn onto the road, or off it, starts nothing
    if self.dist <= 0. and not self.braking and sure:
      # reached without needing to brake (or the fix puts it behind): nothing to stop for
      self._finish()
      return None
    need = v_ego ** 2 / (2. * max(self.dist, 0.5))
    if not self.committed and sure and need > START_MAX:
      # found too late to stop gently: the driver's
      self._finish()
      return None
    if self.braking or (sure and not turning and (need >= START_DECEL or self.dist <= v_ego * START_TIME)):
      self.braking = self.committed = True
      self.a_target = max(-min(need, BRAKE_MAX), self.a_target - START_JERK * DT_MDL)
      return self.a_target
    return None
