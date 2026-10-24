"""
Which fix ubloxd publishes as gpsLocationExternal: the ublox's own whenever it
is trustworthy, and the Jetson's USB GPS (customReserved18, from modeld) only
while it is not.

- The ublox has priority: its messages pass through unchanged unless it has
  been without a trustworthy fix for FALLBACK_AFTER and the Jetson's fix is
  fresh and valid.
- While falling back, each ublox message is replaced by the Jetson's fix,
  projected to now along its course, so consumers see the same cadence and
  the `alive` checks hold. A silent ublox is covered by `tick`.
- Back to the ublox once it has been trustworthy for RECOVER_AFTER without a
  break, or at once when the Jetson's fix is EXTERNAL_EXIT_AGE old: the ublox's no-fix
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
FALLBACK_AFTER = 2.  # s without a trustworthy ublox fix before the Jetson's is used
RECOVER_AFTER = 5.  # s of unbroken trustworthy ublox fixes before switching back
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
    self._trusted_since: float | None = None
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
      if self._trusted_since is None:
        self._trusted_since = now
    else:
      self._trusted_since = None
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
    self._decide(now)
    if not self.using_external or now - self._last_internal < SILENT_UBLOX or now - self._last_tick_publish < TICK_PUBLISH_PERIOD:
      return False
    self._last_tick_publish = now
    return True

  # -- state --------------------------------------------------------------------------------
  def external_fresh(self, now: float, max_age: float = EXTERNAL_MAX_AGE) -> bool:
    return self._external is not None and 0 <= self._external.age(now) <= max_age

  def _decide(self, now: float) -> None:
    if self.using_external:
      recovered = self._trusted_since is not None and now - self._trusted_since >= RECOVER_AFTER
      if recovered or not self.external_fresh(now, EXTERNAL_EXIT_AGE):
        self.using_external = False
        self.switches += 1
    elif now - self._last_trusted >= FALLBACK_AFTER and self.external_fresh(now):
      self.using_external = True
      self.switches += 1

  # -- output -------------------------------------------------------------------------------
  def fill(self, gps, now: float) -> None:
    """The Jetson's fix, projected to now, into a gpsLocationExternal builder."""
    fix = self._external
    assert fix is not None
    age = max(fix.age(now), 0.)
    moving = fix.speed > 0.5 and math.isfinite(fix.bearing_deg)
    lat, lon = project(fix.latitude, fix.longitude, fix.bearing_deg, fix.speed * age) if moving else (fix.latitude, fix.longitude)
    bearing = math.radians(fix.bearing_deg) if moving else 0.
    h_acc = fix.hdop * UERE + 0.5 * fix.speed * age

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
    gps.speedAccuracy = 1.
    gps.bearingAccuracyDeg = 5. if fix.speed > 2 else 180.
    gps.satelliteCount = fix.satellites
    gps.unixTimestampMillis = int(fix.utc_ms + age * 1e3)
