"""Real Cereal/Cap'n Proto regression coverage for gpsard vNED conversion."""

from __future__ import annotations

import math

import pytest

import openpilot.cereal.messaging as messaging
from openpilot.system.gpsard import _gps_msg_to_sample, _vned_from_msg


def _build_gps_event(service: str, v_ned: list[float]) -> bytes:
  msg = messaging.new_message(service, valid=True)
  gps = getattr(msg, service)
  gps.hasFix = True
  gps.latitude = 32.8
  gps.longitude = -96.8
  gps.altitude = 150.0
  gps.horizontalAccuracy = 3.0
  gps.verticalAccuracy = 4.0
  gps.speedAccuracy = 0.5
  gps.bearingDeg = 45.0
  gps.bearingAccuracyDeg = 2.0
  gps.unixTimestampMillis = 1_700_000_000_000
  gps.vNED = v_ned
  gps.measurementMonoNs = 1_000_000_000
  return msg.to_bytes()


def _reader_gps(service: str, v_ned: list[float]):
  return getattr(messaging.log_from_bytes(_build_gps_event(service, v_ned)), service)


def test_dynamic_list_reader_rejects_vned_slice():
  """Document the live gpsard crash: Cap'n Proto lists are not sliceable."""
  vned = _reader_gps("gpsLocationExternal", [9.0, 8.0, 7.0]).vNED
  assert type(vned).__name__ == "_DynamicListReader"
  with pytest.raises(TypeError, match="integer"):
    _ = vned[:3]


def test_gps_msg_to_sample_real_cereal_reader_no_exception():
  gps = _reader_gps("gpsLocationExternal", [1.5, -2.5, 0.25])
  sample = _gps_msg_to_sample(gps, recv_mono=12.5)
  assert sample.v_ned == pytest.approx((1.5, -2.5, 0.25))
  assert sample.has_fix is True
  assert sample.latitude == pytest.approx(32.8)
  assert sample.longitude == pytest.approx(-96.8)
  assert sample.measurement_mono_ns == 1_000_000_000


def test_gps_msg_to_sample_short_cereal_vned_is_nan():
  gps = _reader_gps("gpsLocationExternal", [1.0, 2.0])
  assert type(gps.vNED).__name__ == "_DynamicListReader"
  assert len(gps.vNED) == 2
  sample = _gps_msg_to_sample(gps, recv_mono=1.0)
  assert all(math.isnan(x) for x in sample.v_ned)


def test_gps_msg_to_sample_empty_cereal_vned_is_nan():
  gps = _reader_gps("gpsLocation", [])
  assert type(gps.vNED).__name__ == "_DynamicListReader"
  assert len(gps.vNED) == 0
  sample = _gps_msg_to_sample(gps, recv_mono=1.0)
  assert all(math.isnan(x) for x in sample.v_ned)


def test_vned_from_msg_matches_production_path():
  gps = _reader_gps("gpsLocationExternal", [4.0, 5.0, 6.0])
  assert _vned_from_msg(gps.vNED) == pytest.approx((4.0, 5.0, 6.0))
