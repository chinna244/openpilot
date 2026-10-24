import math

import pytest

from openpilot.cereal import messaging
from openpilot.sunnypilot.jetlink_adapter.gps import JETSON_GPS_SERVICE, JetsonGps, fill

# the bench Jetson's telemetry `gps`, as NmeaGps.encode writes it
GPS = {"lat": 42.971638, "lon": -87.893982, "alt_m": 209.7, "speed_mps": 0.005, "course_deg": 16.57, "hdop": 0.45, "sats": 36,
       "quality": 2, "fix_type": 3, "utc_ms": 1_792_876_061_000, "age_ms": 412}


class Client:
  def __init__(self):
    self.last_state = None


class Model:
  def __init__(self):
    self.client = Client()


class TestPoll:
  def test_each_telemetry_once(self):
    model, gps = Model(), JetsonGps()
    assert gps.poll(model) is None
    model.client.last_state = {"temp_c": 50., "gps": GPS}
    assert gps.poll(model) == GPS
    assert gps.poll(model) is None
    model.client.last_state = {"temp_c": 51., "gps": GPS}
    assert gps.poll(model) == GPS

  def test_telemetry_without_gps(self):
    model, gps = Model(), JetsonGps()
    for state in ({}, {"temp_c": 50.}, {"gps": None}, {"gps": [1, 2]}, "{}"):
      model.client.last_state = state
      assert gps.poll(model) is None

  def test_small_model_has_no_client(self):
    gps = JetsonGps()
    assert gps.poll(object()) is None
    model = Model()
    model.client = None
    assert gps.poll(model) is None


class TestFill:
  def msg(self):
    return messaging.new_message(JETSON_GPS_SERVICE)

  def test_bench_fix(self):
    msg = self.msg()
    assert fill(msg, GPS)
    fix = msg.customReserved18
    assert (fix.latitude, fix.longitude, fix.altitude) == (GPS["lat"], GPS["lon"], GPS["alt_m"])
    assert fix.speed == pytest.approx(0.005)
    assert fix.bearingDeg == pytest.approx(16.57)
    assert fix.hdop == pytest.approx(0.45)
    assert (fix.satelliteCount, fix.fixQuality, fix.fixType) == (36, 2, 3)
    assert (fix.unixTimestampMillis, fix.ageMs) == (GPS["utc_ms"], 412)

  def test_optional_fields_missing(self):
    msg = self.msg()
    gps = {k: GPS[k] for k in ("lat", "lon", "utc_ms", "age_ms")}
    assert fill(msg, gps)
    fix = msg.customReserved18
    assert math.isnan(fix.bearingDeg) and math.isnan(fix.hdop)
    assert (fix.altitude, fix.speed, fix.satelliteCount, fix.fixQuality, fix.fixType) == (0., 0., 0, 0, 0)

  @pytest.mark.parametrize("change", [{"lat": None}, {"lon": "x"}, {"utc_ms": None}, {"age_ms": None}, {"lat": 91.}, {"lon": -181.},
                                      {"lat": math.nan}, {"lat": True}, {"age_ms": -1}, {"age_ms": 2**32}, {"utc_ms": 0}])
  def test_malformed_is_not_sent(self, change):
    gps = {**GPS, **change}
    gps = {k: v for k, v in gps.items() if v is not None}
    assert not fill(self.msg(), gps)

  def test_out_of_range_counts_are_clamped(self):
    msg = self.msg()
    assert fill(msg, {**GPS, "sats": 400, "quality": -1, "fix_type": math.inf, "speed_mps": -3.})
    fix = msg.customReserved18
    assert (fix.satelliteCount, fix.fixQuality, fix.fixType, fix.speed) == (255, 0, 0, 0.)
