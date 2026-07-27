from openpilot.common.gps_time import (
  encode_ublox_gps_flags,
  ublox_gps_time_valid,
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
