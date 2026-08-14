"""Real Cereal/Cap'n Proto regression coverage for gpsard conversion + arbitration."""

from __future__ import annotations

import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.gps_source_arbiter import (
  STARTUP_HEALTH_CONFIRM_SECONDS,
  GpsSourceArbiter,
  SelectedSource,
)
from openpilot.system.gpsard import _gps_msg_to_sample, _vned_from_msg


def _build_gps_event(
  service: str,
  *,
  v_ned: tuple[float, float, float] = (1.5, -2.5, 0.25),
  has_fix: bool = True,
  latitude: float = 32.8,
  longitude: float = -96.8,
  horizontal_accuracy: float = 3.0,
  measurement_mono_ns: int = 1_000_000_000,
) -> bytes:
  msg = messaging.new_message(service, valid=True)
  gps = getattr(msg, service)
  gps.hasFix = has_fix
  gps.latitude = latitude
  gps.longitude = longitude
  gps.altitude = 150.0
  gps.horizontalAccuracy = horizontal_accuracy
  gps.verticalAccuracy = 4.0
  gps.speedAccuracy = 0.5
  gps.bearingDeg = 45.0
  gps.bearingAccuracyDeg = 2.0
  gps.unixTimestampMillis = 1_700_000_000_000
  gps.vNED = list(v_ned)
  gps.measurementMonoNs = measurement_mono_ns
  gps.flags = 1 if has_fix else 0
  gps.satelliteCount = 8
  return msg.to_bytes()


def test_dynamic_list_reader_rejects_vned_slice():
  """Document the live gpsard crash: Cap'n Proto lists are not sliceable."""
  raw = _build_gps_event("gpsLocationExternal", v_ned=(9.0, 8.0, 7.0))
  with log.Event.from_bytes(raw) as evt:
    vned = evt.gpsLocationExternal.vNED
    assert type(vned).__name__ == "_DynamicListReader"
    with pytest.raises(TypeError, match="integer"):
      _ = vned[:3]


def test_gps_msg_to_sample_real_cereal_reader_no_exception():
  raw = _build_gps_event("gpsLocationExternal", v_ned=(1.5, -2.5, 0.25))
  with log.Event.from_bytes(raw) as evt:
    sample = _gps_msg_to_sample(evt.gpsLocationExternal, recv_mono=12.5)
  assert sample.v_ned == pytest.approx((1.5, -2.5, 0.25))
  assert sample.has_fix is True
  assert sample.latitude == pytest.approx(32.8)
  assert sample.longitude == pytest.approx(-96.8)
  assert sample.measurement_mono_ns == 1_000_000_000


def test_vned_from_msg_short_list_is_nan():
  assert all(x != x for x in _vned_from_msg([]))  # NaN
  assert all(x != x for x in _vned_from_msg([1.0, 2.0]))


def test_real_cereal_ublox_arbitration_selects_ublox_primary():
  arbiter = GpsSourceArbiter(ublox_hardware_available=True)
  t0 = 100.0
  arbiter.reset(now_mono=t0, ublox_hardware_available=True)

  # Sustained valid ublox samples beyond startup confirmation window.
  steps = int(STARTUP_HEALTH_CONFIRM_SECONDS / 0.1) + 5
  for i in range(steps):
    now = t0 + i * 0.1
    raw = _build_gps_event(
      "gpsLocationExternal",
      v_ned=(0.1 * i, -0.2, 0.0),
      measurement_mono_ns=int(now * 1e9),
    )
    with log.Event.from_bytes(raw) as evt:
      sample = _gps_msg_to_sample(evt.gpsLocationExternal, recv_mono=now)
    arbiter.observe_ublox(sample, now_mono=now)
    arbiter.step(now_mono=now)

  assert arbiter.state.selected == SelectedSource.UBLOX_PRIMARY
  assert arbiter.state.generation >= 1
  assert arbiter.state.ublox.health.name == "HEALTHY"


def test_real_cereal_qcom_arbitration_selects_qcom_fallback():
  # No ublox hardware → QCOM is the only viable startup winner.
  arbiter = GpsSourceArbiter(ublox_hardware_available=False)
  t0 = 200.0
  arbiter.reset(now_mono=t0, ublox_hardware_available=False)

  steps = int(STARTUP_HEALTH_CONFIRM_SECONDS / 1.0) + 3
  for i in range(steps):
    now = t0 + i * 1.0
    raw = _build_gps_event(
      "gpsLocation",
      v_ned=(0.0, 0.1, 0.0),
      measurement_mono_ns=int(now * 1e9),
    )
    with log.Event.from_bytes(raw) as evt:
      sample = _gps_msg_to_sample(evt.gpsLocation, recv_mono=now)
    arbiter.observe_qcom(sample, now_mono=now)
    arbiter.step(now_mono=now)

  assert arbiter.state.selected == SelectedSource.QCOM_FALLBACK
  assert arbiter.state.generation >= 1
  assert arbiter.state.qcom.health.name == "HEALTHY"
