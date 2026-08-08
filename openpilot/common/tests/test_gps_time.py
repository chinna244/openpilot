from openpilot.common.gps_time import (
  UBLOX_FIX_TYPE_2D,
  UBLOX_FIX_TYPE_3D,
  UBLOX_FIX_TYPE_DEAD_RECKONING,
  UBLOX_FIX_TYPE_GNSS_DR,
  UBLOX_FIX_TYPE_NO_FIX,
  UBLOX_FIX_TYPE_TIME_ONLY,
  encode_ublox_gps_flags,
  ublox_gps_time_valid,
  ublox_nav_pvt_has_fix,
)


def test_encode_preserves_fix_flags():
  flags = encode_ublox_gps_flags(0xA5, 0x07)

  assert flags & 0xFF == 0xA5
  assert (flags >> 8) & 0xFF == 0x07


def test_time_requires_fully_resolved_date_and_time():
  assert ublox_gps_time_valid(encode_ublox_gps_flags(1, 0x07))
  assert not ublox_gps_time_valid(encode_ublox_gps_flags(1, 0x00))
  assert not ublox_gps_time_valid(encode_ublox_gps_flags(1, 0x01))
  assert not ublox_gps_time_valid(encode_ublox_gps_flags(1, 0x02))
  assert not ublox_gps_time_valid(encode_ublox_gps_flags(1, 0x03))


def test_nav_pvt_has_fix_matrix():
  # gnssFixOk required for every accepted type.
  for fix_type in (
    UBLOX_FIX_TYPE_NO_FIX,
    UBLOX_FIX_TYPE_DEAD_RECKONING,
    UBLOX_FIX_TYPE_2D,
    UBLOX_FIX_TYPE_3D,
    UBLOX_FIX_TYPE_GNSS_DR,
    UBLOX_FIX_TYPE_TIME_ONLY,
  ):
    assert ublox_nav_pvt_has_fix(0x00, fix_type) is False

  assert ublox_nav_pvt_has_fix(0x01, UBLOX_FIX_TYPE_NO_FIX) is False
  assert ublox_nav_pvt_has_fix(0x01, UBLOX_FIX_TYPE_DEAD_RECKONING) is False
  assert ublox_nav_pvt_has_fix(0x01, UBLOX_FIX_TYPE_2D) is False
  assert ublox_nav_pvt_has_fix(0x01, UBLOX_FIX_TYPE_TIME_ONLY) is False
  assert ublox_nav_pvt_has_fix(0x01, UBLOX_FIX_TYPE_3D) is True
  assert ublox_nav_pvt_has_fix(0x01, UBLOX_FIX_TYPE_GNSS_DR) is True
  # Other flag bits must not alone create a fix.
  assert ublox_nav_pvt_has_fix(0x02, UBLOX_FIX_TYPE_3D) is False
