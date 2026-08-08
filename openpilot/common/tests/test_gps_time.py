from datetime import timedelta

import pytest

from openpilot.common.gps_time import (
  GPS_EPOCH_UTC,
  GPS_UTC_LEAP_SECONDS,
  GPS_UTC_LEAP_SECONDS_VALID_FROM_WEEK,
  GPS_WEEK_MILLISECONDS,
  UBLOX_FIX_TYPE_2D,
  UBLOX_FIX_TYPE_3D,
  UBLOX_FIX_TYPE_DEAD_RECKONING,
  UBLOX_FIX_TYPE_GNSS_DR,
  UBLOX_FIX_TYPE_NO_FIX,
  UBLOX_FIX_TYPE_TIME_ONLY,
  encode_ublox_gps_flags,
  gps_week_tow_to_unix_millis,
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


def test_gps_tow_bounds():
  week = GPS_UTC_LEAP_SECONDS_VALID_FROM_WEEK
  assert gps_week_tow_to_unix_millis(week, 0.0) > 0
  assert gps_week_tow_to_unix_millis(week, GPS_WEEK_MILLISECONDS - 1) > 0
  with pytest.raises(ValueError):
    gps_week_tow_to_unix_millis(week, GPS_WEEK_MILLISECONDS)
  with pytest.raises(ValueError):
    gps_week_tow_to_unix_millis(week, GPS_WEEK_MILLISECONDS + 1)
  with pytest.raises(ValueError):
    gps_week_tow_to_unix_millis(week, -1.0)
  with pytest.raises(ValueError):
    gps_week_tow_to_unix_millis(week, float("nan"))
  with pytest.raises(ValueError):
    gps_week_tow_to_unix_millis(week, float("inf"))


def test_historical_week_rejected_under_default_leap_authority():
  with pytest.raises(ValueError):
    gps_week_tow_to_unix_millis(0, 0.0)
  # Explicit historical leap offset remains available to callers with authority.
  expected = GPS_EPOCH_UTC.timestamp() * 1e3
  assert gps_week_tow_to_unix_millis(0, 0.0, leap_seconds=0) == pytest.approx(expected)


def test_current_era_known_conversion():
  week = 2300
  expected = (GPS_EPOCH_UTC + timedelta(weeks=week) - timedelta(seconds=GPS_UTC_LEAP_SECONDS)).timestamp() * 1e3
  assert gps_week_tow_to_unix_millis(week, 0.0) == pytest.approx(expected)


def test_gps_week_tow_leap_offset_applied_once():
  week = GPS_UTC_LEAP_SECONDS_VALID_FROM_WEEK
  with_default = gps_week_tow_to_unix_millis(week, 0.0)
  with_zero_leap = gps_week_tow_to_unix_millis(week, 0.0, leap_seconds=0)
  assert with_zero_leap - with_default == pytest.approx(GPS_UTC_LEAP_SECONDS * 1000.0)
