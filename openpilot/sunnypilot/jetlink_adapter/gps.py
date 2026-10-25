"""
The Jetson's USB GPS, as it rides on JetLink's telemetry.

The Jetson's server merges `gps` into the telemetry it appends to a frame's
reply when the comma asks for it (about once a second). modeld hands each new
one here, and publishes it as customReserved18 for ubloxd's arbiter
(system/ubloxd/gps_arbiter.py) and pigeond's aiding. Nothing here decides
whether the fix is good enough to use: it only checks that it is well formed.

Standard library only: modeld calls it on its frame loop.
"""
from __future__ import annotations

import math
import time

JETSON_GPS_SERVICE = "customReserved18"
# the receiver fixes once a second
TELEMETRY_PERIOD = 1.0  # s


def _ask() -> None:
  pass


class JetsonGps:
  """New fixes from the link: each telemetry dict once, while the large model
  drives (the small model has no link, so no fix)."""

  def __init__(self):
    self._last_state: dict | None = None
    self._last_ask = -math.inf

  def ask(self):
    """model.run's after_enqueue about once a second, else None: a frame run with one asks the Jetson for its
    telemetry, and so its fix. jetlink's own once-a-second ask seldom fires (model_state: prepare's log resets
    the timer send reads just after), which left fixes 1 to 11 s apart. For a modeld without a chestnut's
    after_enqueue of its own."""
    now = time.monotonic()
    if now - self._last_ask < TELEMETRY_PERIOD:
      return None
    self._last_ask = now
    return _ask

  def poll(self, model) -> dict | None:
    state = getattr(getattr(model, 'client', None), 'last_state', None)
    if not isinstance(state, dict) or state is self._last_state:
      return None
    self._last_state = state
    gps = state.get('gps')
    return gps if isinstance(gps, dict) else None


def _number(gps: dict, key: str, default: float | None = None) -> float | None:
  value = gps.get(key, default)
  if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
    return default
  return float(value)


def fill(msg, gps: dict) -> bool:
  """`gps` into msg.customReserved18; False, leaving it unsent, when the fix
  is missing its position, time or age, or has one out of range."""
  lat, lon = _number(gps, 'lat'), _number(gps, 'lon')
  utc_ms, age_ms = _number(gps, 'utc_ms'), _number(gps, 'age_ms')
  if lat is None or lon is None or utc_ms is None or age_ms is None:
    return False
  if abs(lat) > 90 or abs(lon) > 180 or not 0 <= age_ms < 2**32 or utc_ms <= 0:
    return False
  fix = msg.customReserved18
  fix.latitude = lat
  fix.longitude = lon
  fix.altitude = _number(gps, 'alt_m', 0.)
  fix.speed = max(0., _number(gps, 'speed_mps', 0.))
  fix.bearingDeg = _number(gps, 'course_deg', math.nan)
  fix.hdop = _number(gps, 'hdop', math.nan)
  fix.satelliteCount = int(min(max(_number(gps, 'sats', 0.), 0), 255))
  fix.fixQuality = int(min(max(_number(gps, 'quality', 0.), 0), 255))
  fix.fixType = int(min(max(_number(gps, 'fix_type', 0.), 0), 255))
  fix.unixTimestampMillis = int(utc_ms)
  fix.ageMs = int(age_ms)
  return True
