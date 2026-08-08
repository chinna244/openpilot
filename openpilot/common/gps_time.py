"""Helpers for carrying u-blox NAV-PVT validity through GpsLocationData."""

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
