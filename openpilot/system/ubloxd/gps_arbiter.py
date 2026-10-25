"""
Which fix ubloxd publishes as gpsLocationExternal: the ublox's own whenever it
is trustworthy, and the Jetson's USB GPS (customReserved18, from modeld) only
while it is not.

- The ublox has priority: its messages pass through unchanged unless it has
  been without a trustworthy fix for FALLBACK_AFTER (a dropped message or two
  passes as it is) and the Jetson's fix is fresh and valid.
- While falling back, each ublox message is replaced by the Jetson's fix,
  projected to now along its course, so consumers see the same cadence and
  the `alive` checks hold. A silent ublox is covered by `tick`.
- Back to the ublox once it has been good (RECOVER_MAX_H_ACC) for RECOVER_AFTER
  without a break, or at once when the Jetson's fix is EXTERNAL_EXIT_AGE old: the ublox's no-fix
  messages then pass through as before, so nothing downstream ever drives on
  a stale position.
- A Jetson fix that implies an impossible jump from the one before is held
  back until EXTERNAL_JUMP_RESETS in a row agree it moved.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# the ublox's fix is trustworthy with hasFix and at most this horizontal accuracy
INTERNAL_MAX_H_ACC = 25.  # m
# Route 0000018f: at 2 s every ublox dropout with a fresh Jetson fix went out as no fix (7 gaps, 6.1 s in 92 min, five
# in the first 4 min); at 0.35 s only a dropped message or two does.
FALLBACK_AFTER = 0.35  # s without a trustworthy ublox fix before the Jetson's is used
RECOVER_AFTER = 5.  # s of unbroken good ublox fixes before switching back
# Back only to a good ublox: route 0000018f's start handed back to an 11-20 m ublox four times in 4 min, and each time
# it dropped again. Good is this, or as good as the Jetson's fix as published at EXTERNAL_MAX_AGE (steady through its
# 1 s cycle; at speed the 1 Hz projection is the worse one: ~40 m at 30 m/s). Up to INTERNAL_MAX_H_ACC the ublox is still
# trusted, so it never forces a fallback.
RECOVER_MAX_H_ACC = 10.  # m
# the Jetson's fix: its age there (the receiver's 0.75 s serial latency included)
# plus the time since modeld published it. The comma asks for telemetry about
# once a second and the receiver fixes once a second, so a good fix is 0.75 to
# ~2.9 s old by the time the next one comes. Used from at most EXTERNAL_MAX_AGE,
# kept until EXTERNAL_EXIT_AGE, so one late reply or missed epoch does not flap
EXTERNAL_MAX_AGE = 2.5  # s
EXTERNAL_EXIT_AGE = 4.5  # s
EXTERNAL_MAX_SPEED = 100.  # m/s
EXTERNAL_MIN_SATS = 6
EXTERNAL_MAX_HDOP = 2.5
EXTERNAL_QUALITIES = (1, 2, 4, 5)  # NMEA GGA: GPS, DGPS/SBAS, RTK fixed, RTK float
EXTERNAL_MAX_JUMP_SPEED = 70.  # m/s implied between consecutive Jetson fixes
EXTERNAL_JUMP_RESETS = 3
UERE = 5.  # m of horizontal error per unit HDOP
# a silent ublox: publish the Jetson's fix from tick() at this rate
SILENT_UBLOX = 0.5  # s without a ublox message
# tick() runs before the wake's ublox message, so it falls back only for a ublox this long silent: a late message
# (a serial or CPU hiccup) is judged by internal() when it comes, not switched away from first
SILENT_FALLBACK = 1.  # s
TICK_PUBLISH_PERIOD = 0.09  # s, under the loop's 0.1 s wake so each wake publishes
EARTH_RADIUS = 6378137.  # m


@dataclass(frozen=True)
class ExternalFix:
  latitude: float
  longitude: float
  altitude: float
  speed: float
  bearing_deg: float  # NaN when unknown
  hdop: float
  satellites: int
  quality: int
  fix_type: int
  utc_ms: int
  received: float  # monotonic s when modeld published it, less its age on the Jetson

  @staticmethod
  def from_msg(fix, log_mono_time_ns: int) -> ExternalFix:
    """customReserved18 and its logMonoTime."""
    return ExternalFix(fix.latitude, fix.longitude, fix.altitude, fix.speed, fix.bearingDeg, fix.hdop, fix.satelliteCount,
                       fix.fixQuality, fix.fixType, fix.unixTimestampMillis, log_mono_time_ns * 1e-9 - fix.ageMs * 1e-3)

  def age(self, now: float) -> float:
    return now - self.received

  def valid(self) -> bool:
    return (abs(self.latitude) <= 90 and abs(self.longitude) <= 180 and self.utc_ms > 0 and
            self.quality in EXTERNAL_QUALITIES and self.fix_type in (0, 3) and self.satellites >= EXTERNAL_MIN_SATS and
            math.isfinite(self.hdop) and 0 < self.hdop <= EXTERNAL_MAX_HDOP and 0 <= self.speed < EXTERNAL_MAX_SPEED)


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  """Haversine."""
  p1, p2 = math.radians(lat1), math.radians(lat2)
  a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(math.radians(lon2 - lon1) / 2) ** 2
  return 2 * EARTH_RADIUS * math.asin(min(1., math.sqrt(a)))


def project(lat: float, lon: float, bearing_deg: float, distance: float) -> tuple[float, float]:
  """`distance` m along `bearing_deg` from lat/lon; flat earth, fine for tens of metres."""
  b = math.radians(bearing_deg)
  dlat = distance * math.cos(b) / EARTH_RADIUS
  dlon = distance * math.sin(b) / (EARTH_RADIUS * max(math.cos(math.radians(lat)), 1e-6))
  return lat + math.degrees(dlat), lon + math.degrees(dlon)


class GpsArbiter:
  def __init__(self, now: float):
    self.using_external = False
    self.switches = 0
    self._last_trusted = now  # as if trustworthy at start: no fallback in the first FALLBACK_AFTER
    self._good_since: float | None = None
    self._last_internal = now
    self._last_tick_publish = -math.inf
    self._external: ExternalFix | None = None
    self._jumps = 0

  # -- inputs -------------------------------------------------------------------------------
  def internal(self, has_fix: bool, horizontal_accuracy: float, now: float) -> bool:
    """A ublox gpsLocationExternal: True to publish it as it is, False to
    publish the Jetson's fix (`fill`) in its place."""
    self._last_internal = now
    if has_fix and 0 < horizontal_accuracy <= INTERNAL_MAX_H_ACC:
      self._last_trusted = now
    # never past trusted: an untrusted fix handed back to would fall straight back on the same wake
    if has_fix and 0 < horizontal_accuracy <= min(INTERNAL_MAX_H_ACC, max(RECOVER_MAX_H_ACC, self.external_h_acc(EXTERNAL_MAX_AGE))):
      if self._good_since is None:
        self._good_since = now
    else:
      self._good_since = None
    self._decide(now)
    return not self.using_external

  def external(self, fix: ExternalFix) -> bool:
    """A Jetson fix from modeld: True when it is accepted as the current one."""
    if not fix.valid():
      return False
    last = self._external
    if last is not None and fix.received - last.received > EXTERNAL_EXIT_AGE:
      last = None  # long gone: a fresh start, whatever its time said
    if last is not None:
      dt = (fix.utc_ms - last.utc_ms) * 1e-3
      if dt <= 0:
        return False  # the same fix again, or older
      implied = distance_m(last.latitude, last.longitude, fix.latitude, fix.longitude) / max(dt, 0.5)
      if implied > EXTERNAL_MAX_JUMP_SPEED and self._jumps + 1 < EXTERNAL_JUMP_RESETS:
        self._jumps += 1
        return False
    self._jumps = 0
    self._external = fix
    return True

  def tick(self, now: float) -> bool:
    """Every loop: True to publish the Jetson's fix now, for a ublox that has
    gone silent while falling back."""
    if self.using_external or now - self._last_internal >= SILENT_FALLBACK:
      self._decide(now)
    if not self.using_external or now - self._last_internal < SILENT_UBLOX or now - self._last_tick_publish < TICK_PUBLISH_PERIOD:
      return False
    self._last_tick_publish = now
    return True

  # -- state --------------------------------------------------------------------------------
  def external_h_acc(self, age: float) -> float:
    """The horizontal accuracy `fill` publishes for the Jetson's fix at this age; 0 without one."""
    fix = self._external
    if fix is None:
      return 0.
    age = max(age, 0.)
    no_course = fix.speed > 0.5 and not math.isfinite(fix.bearing_deg)
    # projected, the error is the course's; not projected, it is the whole way travelled since the fix
    return fix.hdop * UERE + (fix.speed * age if no_course else 0.5 * fix.speed * age)

  def external_fresh(self, now: float, max_age: float = EXTERNAL_MAX_AGE) -> bool:
    return self._external is not None and 0 <= self._external.age(now) <= max_age

  def _decide(self, now: float) -> None:
    if self.using_external:
      recovered = self._good_since is not None and now - self._good_since >= RECOVER_AFTER
      if recovered or not self.external_fresh(now, EXTERNAL_EXIT_AGE):
        self.using_external = False
        self.switches += 1
    elif now - self._last_trusted >= FALLBACK_AFTER and self.external_fresh(now):
      self.using_external = True
      self.switches += 1
      # the hand-back counts good fixes from here: a ublox gone silent after good ones has not recovered
      self._good_since = None

  # -- output -------------------------------------------------------------------------------
  def fill(self, gps, now: float) -> None:
    """The Jetson's fix, projected to now, into a gpsLocationExternal builder."""
    fix = self._external
    assert fix is not None
    age = max(fix.age(now), 0.)
    moving = fix.speed > 0.5 and math.isfinite(fix.bearing_deg)
    # moving with no course (a receiver may leave it blank at low speed): the velocity's direction is unknown,
    # so it goes out as zero with an accuracy as large as the speed, never as a confident standstill
    no_course = fix.speed > 0.5 and not moving
    lat, lon = project(fix.latitude, fix.longitude, fix.bearing_deg, fix.speed * age) if moving else (fix.latitude, fix.longitude)
    bearing = math.radians(fix.bearing_deg) if moving else 0.
    h_acc = self.external_h_acc(age)

    gps.source = 'external'
    gps.hasFix = True
    gps.flags = 1  # gnssFixOK, as a ublox fix has it
    gps.latitude = lat
    gps.longitude = lon
    gps.altitude = fix.altitude
    gps.speed = fix.speed
    gps.bearingDeg = fix.bearing_deg if moving else 0.
    gps.vNED = [fix.speed * math.cos(bearing), fix.speed * math.sin(bearing), 0.] if moving else [0., 0., 0.]
    gps.horizontalAccuracy = h_acc
    gps.verticalAccuracy = 2 * h_acc
    gps.speedAccuracy = max(1., fix.speed) if no_course else 1.
    gps.bearingAccuracyDeg = 5. if moving and fix.speed > 2 else 180.
    gps.satelliteCount = fix.satellites
    gps.unixTimestampMillis = int(fix.utc_ms + age * 1e3)
