"""Helpers for carrying u-blox NAV-PVT validity through GpsLocationData."""

UBLOX_NAV_PVT_VALID_SHIFT = 8
UBLOX_NAV_PVT_TRUSTED_TIME_MASK = 0x07


def encode_ublox_gps_flags(fix_flags: int, time_valid_flags: int) -> int:
  """Pack existing u-blox fix flags and NAV-PVT validity into UInt16."""
  return (
    (fix_flags & 0xFF)
    | ((time_valid_flags & 0xFF) << UBLOX_NAV_PVT_VALID_SHIFT)
  )


def ublox_gps_time_valid(flags: int) -> bool:
  """Return whether NAV-PVT date/time are valid and fully resolved."""
  validity = (flags >> UBLOX_NAV_PVT_VALID_SHIFT) & 0xFF
  return (
    validity & UBLOX_NAV_PVT_TRUSTED_TIME_MASK
  ) == UBLOX_NAV_PVT_TRUSTED_TIME_MASK
