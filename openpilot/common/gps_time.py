"""Helpers for carrying u-blox NAV-PVT validity through GpsLocationData."""

from __future__ import annotations

from datetime import UTC, datetime, timedelta
from math import isfinite

UBLOX_NAV_PVT_VALID_SHIFT = 8
UBLOX_NAV_PVT_TRUSTED_TIME_MASK = 0x07

# NAV-PVT fixType (u-blox Interface Description).
UBLOX_FIX_TYPE_NO_FIX = 0
UBLOX_FIX_TYPE_DEAD_RECKONING = 1
UBLOX_FIX_TYPE_2D = 2
UBLOX_FIX_TYPE_3D = 3
UBLOX_FIX_TYPE_GNSS_DR = 4
UBLOX_FIX_TYPE_TIME_ONLY = 5

# gpsLocationExternal.hasFix policy for downstream KF consumers (locationd_llk):
# advertise only GNSS-anchored solutions that safely support lat/lon/alt + NED.
# 2D is rejected (altitude unconstrained). DR-only and time-only are rejected.
UBLOX_HAS_FIX_TYPES = (
  UBLOX_FIX_TYPE_3D,
  UBLOX_FIX_TYPE_GNSS_DR,
)

# GPS epoch and current GPS-UTC leap-second offset.
# Update GPS_UTC_LEAP_SECONDS here when a new leap second is introduced.
GPS_EPOCH_UTC = datetime(1980, 1, 6, tzinfo=UTC)
GPS_UTC_LEAP_SECONDS = 18
GPS_WEEK_MILLISECONDS = 7 * 24 * 60 * 60 * 1000

# GPS_UTC_LEAP_SECONDS (=18) is valid from the 2017-01-01 leap era onward.
# Reject earlier weeks rather than applying today's offset to historical epochs.
# GPS week 1930 begins 2017-01-01; the 18s offset applies for week >= 1930.
GPS_UTC_LEAP_SECONDS_VALID_FROM_WEEK = 1930


def encode_ublox_gps_flags(fix_flags: int, time_valid_flags: int) -> int:
  """Pack existing u-blox fix flags and NAV-PVT validity into UInt16."""
  return (fix_flags & 0xFF) | ((time_valid_flags & 0xFF) << UBLOX_NAV_PVT_VALID_SHIFT)


def ublox_gps_time_valid(flags: int) -> bool:
  """Return whether NAV-PVT date/time are valid and fully resolved."""
  validity = (flags >> UBLOX_NAV_PVT_VALID_SHIFT) & 0xFF
  return (validity & UBLOX_NAV_PVT_TRUSTED_TIME_MASK) == UBLOX_NAV_PVT_TRUSTED_TIME_MASK


def ublox_nav_pvt_has_fix(flags: int, fix_type: int) -> bool:
  """Return whether NAV-PVT should set gpsLocationExternal.hasFix.

  Requires gnssFixOk (flags bit 0) and fixType in {3D, GNSS+DR}.
  """
  if type(flags) is not int or type(fix_type) is not int:
    return False
  gnss_fix_ok = (flags & 0x01) != 0
  return gnss_fix_ok and fix_type in UBLOX_HAS_FIX_TYPES


def gps_week_tow_to_unix_millis(
  gps_week: int,
  gps_tow_ms: float,
  *,
  leap_seconds: int | None = None,
) -> float:
  """Convert GPS week + time-of-week milliseconds to Unix epoch milliseconds (UTC).

  Modem timestamps are GPS time. UTC = GPS - leap_seconds.

  Default leap_seconds is GPS_UTC_LEAP_SECONDS and is only applied for GPS weeks
  in the current leap era (see GPS_UTC_LEAP_SECONDS_VALID_FROM_WEEK). Pass an
  explicit leap_seconds only when the caller has independent authority for that
  epoch; do not use the default for pre-2017 history.
  """
  if type(gps_week) is not int or gps_week < 0 or gps_week >= 0xFFFF:
    raise ValueError("gps_week must be a valid GPS week")
  if isinstance(gps_tow_ms, bool) or not isinstance(gps_tow_ms, (int, float)):
    raise ValueError("gps_tow_ms must be numeric")
  tow_ms = float(gps_tow_ms)
  if not isfinite(tow_ms) or tow_ms < 0.0 or tow_ms >= GPS_WEEK_MILLISECONDS:
    raise ValueError("gps_tow_ms must be finite within one GPS week")

  if leap_seconds is None:
    if gps_week < GPS_UTC_LEAP_SECONDS_VALID_FROM_WEEK:
      raise ValueError("gps_week is outside the maintained GPS_UTC_LEAP_SECONDS era; pass an explicit leap_seconds for historical conversions")
    applied_leap = GPS_UTC_LEAP_SECONDS
  else:
    if type(leap_seconds) is not int or leap_seconds < 0:
      raise ValueError("leap_seconds must be a non-negative int")
    applied_leap = leap_seconds

  utc = GPS_EPOCH_UTC + timedelta(weeks=gps_week) + timedelta(milliseconds=tow_ms) - timedelta(seconds=applied_leap)
  return utc.timestamp() * 1e3
