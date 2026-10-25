import math

import pytest

from openpilot.cereal import messaging
from openpilot.system.ubloxd import gps_arbiter as ga
from openpilot.system.ubloxd.gps_arbiter import ExternalFix, GpsArbiter

LAT, LON = 42.971638, -87.893982
UTC0 = 1_792_876_061_000


def ext(t: float, age: float = 0.3, lat: float = LAT, lon: float = LON, speed: float = 0., bearing: float = math.nan, **kw) -> ExternalFix:
  """A Jetson fix modeld published at monotonic `t`, `age` s old on the Jetson, its UTC on a 1 s grid."""
  fields = {"latitude": lat, "longitude": lon, "altitude": 209.7, "speed": speed, "bearing_deg": bearing, "hdop": 0.45, "satellites": 36,
            "quality": 2, "fix_type": 3, "utc_ms": UTC0 + round((t - age) * 1000), "received": t - age}
  fields.update(kw)
  return ExternalFix(**fields)


class Sim:
  """The ublox at 10 Hz and the Jetson at 1 Hz on a clock the test drives."""

  def __init__(self):
    self.t = 0.
    self.arb = GpsArbiter(self.t)
    self.next_ext = 0.
    self.ticks = 0

  def run(self, seconds: float, ublox_fix: bool | None, jetson: bool = True, h_acc: float = 3.) -> list[bool]:
    """ublox_fix None: the ublox is silent. Returns, per ublox message, whether the ublox's own went out."""
    out = []
    for _ in range(round(seconds * 10)):
      self.t = round(self.t + 0.1, 6)
      if jetson and self.t >= self.next_ext:
        self.arb.external(ext(self.t))
        self.next_ext = self.t + 1.
      self.ticks += self.arb.tick(self.t)
      if ublox_fix is not None:
        out.append(self.arb.internal(ublox_fix, h_acc, self.t))
    return out


class TestArbitration:
  def test_ublox_passes_through_without_jetson(self):
    sim = Sim()
    assert all(sim.run(30, ublox_fix=False, jetson=False))
    assert sim.arb.switches == 0

  def test_trusted_ublox_keeps_priority_over_jetson(self):
    sim = Sim()
    assert all(sim.run(30, ublox_fix=True))
    assert not sim.arb.using_external

  def test_low_accuracy_ublox_fix_is_not_trusted(self):
    sim = Sim()
    assert all(sim.run(5, ublox_fix=True, h_acc=25.))  # 25 m is still trusted
    out = sim.run(5, ublox_fix=True, h_acc=26.)
    assert sim.arb.using_external and not out[-1]

  def test_falls_back_after_two_seconds_without_ublox_fix(self):
    sim = Sim()
    sim.run(10, ublox_fix=True)
    out = sim.run(5, ublox_fix=False)
    first_external = out.index(False)
    assert first_external == 19  # 2.0 s after the last trusted fix, at index -1
    assert not any(out[first_external:])
    assert sim.arb.switches == 1

  def test_returns_to_ublox_after_five_unbroken_seconds(self):
    sim = Sim()
    sim.run(10, ublox_fix=False)
    assert sim.arb.using_external
    out = sim.run(6, ublox_fix=True)
    back = out.index(True)
    assert back == 50  # 5.0 s after the first trusted fix, at index 0
    assert all(out[back:])
    assert sim.arb.switches == 2

  def test_broken_recovery_restarts_the_count(self):
    sim = Sim()
    sim.run(10, ublox_fix=False)
    sim.run(4, ublox_fix=True)
    sim.run(0.1, ublox_fix=False)
    assert not any(sim.run(4.8, ublox_fix=True))
    assert any(sim.run(0.5, ublox_fix=True))

  def test_flickering_ublox_does_not_flap(self):
    sim = Sim()
    sim.run(10, ublox_fix=False)
    for _ in range(20):  # 1 s with a fix, 1 s without, for 40 s
      sim.run(1, ublox_fix=True)
      sim.run(1, ublox_fix=False)
    assert sim.arb.using_external and sim.arb.switches == 1

  def test_stale_jetson_hands_back_to_ublox_no_fix(self):
    sim = Sim()
    sim.run(10, ublox_fix=False)
    assert sim.arb.using_external
    out = sim.run(5, ublox_fix=False, jetson=False)
    back = out.index(True)
    # the last fix arrived 0.3 s old at most 1 s before the Jetson went quiet
    assert (back + 1) * 0.1 <= ga.EXTERNAL_EXIT_AGE
    assert all(out[back:])
    assert not sim.arb.external_fresh(sim.t)

  def test_a_missed_jetson_fix_does_not_flap(self):
    arb = GpsArbiter(0.)
    t = 0.
    for _ in range(30):  # ublox without a fix, the Jetson's every 1.05 s at its oldest (1.75 s)
      t += 1.05
      arb.external(ext(t, age=1.75))
      arb.internal(False, 0., t)
    assert arb.using_external and arb.switches == 1
    for step in range(22):  # one telemetry reply lost: 2.1 s to the next
      arb.internal(False, 0., t + 0.1 * (step + 1))
    assert arb.using_external
    arb.external(ext(t + 2.2, age=1.75))
    assert arb.internal(False, 0., t + 2.2) is False and arb.switches == 1

  def test_entry_needs_a_fresh_fix_but_exit_waits_longer(self):
    arb = GpsArbiter(0.)
    arb.external(ext(10., age=0.3))
    arb.internal(False, 0., 9.)
    assert arb.internal(False, 0., 12.9)  # 3.2 s old: too old to start using
    arb.external(ext(13., age=0.3))
    assert not arb.internal(False, 0., 13.)
    assert not arb.internal(False, 0., 13. - 0.3 + ga.EXTERNAL_EXIT_AGE - 0.01)
    assert arb.internal(False, 0., 13. - 0.3 + ga.EXTERNAL_EXIT_AGE + 0.01)

  def test_no_fallback_in_the_first_seconds(self):
    sim = Sim()
    out = sim.run(1.9, ublox_fix=False)
    assert all(out)

  def test_silent_ublox_is_covered_by_tick(self):
    sim = Sim()
    sim.run(3, ublox_fix=False)
    assert sim.arb.using_external and sim.ticks == 0
    sim.run(2, ublox_fix=None)
    # not for the first SILENT_UBLOX, then every TICK_PUBLISH_PERIOD
    assert sim.arb.using_external and 15 <= sim.ticks <= 16

  def test_tick_is_quiet_while_ublox_talks(self):
    sim = Sim()
    sim.run(3, ublox_fix=False)
    assert not any(sim.arb.tick(sim.t) for _ in range(5))


class TestExternalFix:
  @pytest.mark.parametrize("change", [{"satellites": 5}, {"hdop": 2.6}, {"hdop": math.nan}, {"hdop": 0.}, {"quality": 0},
                                      {"quality": 6}, {"fix_type": 2}, {"speed": 100.}, {"speed": math.nan}, {"latitude": 91.}, {"utc_ms": 0}])
  def test_invalid_fixes_are_rejected(self, change):
    assert not GpsArbiter(0.).external(ext(1., **change))

  def test_unknown_fix_type_is_accepted(self):
    assert GpsArbiter(0.).external(ext(1., fix_type=0))

  def test_same_or_older_fix_is_ignored(self):
    arb = GpsArbiter(0.)
    assert arb.external(ext(1.))
    assert not arb.external(ext(1.))
    assert not arb.external(ext(0.5))
    assert arb.external(ext(2.))

  def test_a_bad_future_time_does_not_block_the_fallback_for_good(self):
    arb = GpsArbiter(0.)
    assert arb.external(ext(1., utc_ms=UTC0 + 10**9))  # a receiver's bad clock, weeks ahead
    assert not arb.external(ext(2.))
    assert arb.external(ext(1. + ga.EXTERNAL_EXIT_AGE + 0.5))  # once that one is gone, a fresh start

  def test_jump_is_held_until_confirmed(self):
    arb = GpsArbiter(0.)
    assert arb.external(ext(1.))
    far = LAT + 0.01  # 1.1 km in 1 s
    assert not arb.external(ext(2., lat=far))
    assert not arb.external(ext(3., lat=far))
    assert arb.external(ext(4., lat=far))  # the third in a row is believed
    assert arb.external(ext(5., lat=far))

  def test_highway_speed_is_not_a_jump(self):
    arb = GpsArbiter(0.)
    assert arb.external(ext(1.))
    assert arb.external(ext(2., lat=LAT + 40 / 111_320))

  def test_from_msg_ages_by_jetson_age(self):
    msg = messaging.new_message('customReserved18')
    fix = msg.customReserved18
    fix.latitude, fix.longitude, fix.hdop, fix.satelliteCount, fix.fixQuality, fix.fixType = LAT, LON, 0.5, 9, 1, 3
    fix.unixTimestampMillis, fix.ageMs, fix.bearingDeg = UTC0, 700, math.nan
    e = ExternalFix.from_msg(fix, int(100e9))
    assert e.received == pytest.approx(99.3)
    assert e.age(100.2) == pytest.approx(0.9)
    assert e.valid()


class TestFill:
  def fill(self, fix: ExternalFix, now: float):
    arb = GpsArbiter(0.)
    assert arb.external(fix)
    msg = messaging.new_message('gpsLocationExternal')
    arb.fill(msg.gpsLocationExternal, now)
    return msg.gpsLocationExternal

  def test_projects_along_course_by_age(self):
    # 20 m/s due east, published 0.5 s old, filled 0.5 s later: 20 m east
    gps = self.fill(ext(10., age=0.5, speed=20., bearing=90.), 10.5)
    north = (gps.latitude - LAT) * 111_320
    east = (gps.longitude - LON) * 111_320 * math.cos(math.radians(LAT))
    assert abs(north) < 0.01 and east == pytest.approx(20., abs=0.05)
    assert gps.unixTimestampMillis == UTC0 + 10_500
    assert list(gps.vNED) == pytest.approx([0., 20., 0.], abs=1e-6)
    assert gps.horizontalAccuracy == pytest.approx(0.45 * ga.UERE + 0.5 * 20 * 1.)

  def test_standing_still_is_not_projected(self):
    gps = self.fill(ext(10., speed=0.2, bearing=16.57), 11.)
    assert (gps.latitude, gps.longitude) == (LAT, LON)
    assert gps.bearingAccuracyDeg == 180.
    assert list(gps.vNED) == [0., 0., 0.]

  def test_moving_without_a_course_is_not_a_standstill(self):
    # 20 m/s, the receiver left the course blank: no projection, and the zero velocity carries no weight
    gps = self.fill(ext(10., age=0.5, speed=20.), 10.5)
    assert (gps.latitude, gps.longitude) == (LAT, LON)
    assert list(gps.vNED) == [0., 0., 0.]
    assert gps.speed == 20.
    assert gps.speedAccuracy >= 20.
    assert gps.bearingAccuracyDeg == 180.
    assert gps.horizontalAccuracy == pytest.approx(0.45 * ga.UERE + 20 * 1.)

  def test_passes_locationd_sanity_checks(self):
    # sunnypilot/selfdrive/locationd/locationd.cc Localizer::handle_gps
    for speed, bearing in ((0., math.nan), (35., 271.), (35., math.nan)):
      gps = self.fill(ext(10., speed=speed, bearing=bearing), 10. + ga.EXTERNAL_EXIT_AGE)
      assert gps.hasFix and gps.flags % 2 == 1
      assert gps.verticalAccuracy > 0 and gps.speedAccuracy > 0 and gps.bearingAccuracyDeg > 0
      assert math.hypot(gps.horizontalAccuracy, gps.verticalAccuracy) < 1500
      assert gps.source == 'external'
      assert all(math.isfinite(v) for v in (gps.latitude, gps.longitude, gps.bearingDeg, *gps.vNED))
