"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import numpy as np
import pytest

from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.map_controller import SmartCruiseControlMap, \
  ahead_distance, model_curve_speed, relaxed_velocities

LAT0, LON0 = 42.96, -87.93
R_EARTH = 6373000.


def to_targets(xy, v):
  """points in meters east/north of (LAT0, LON0) as mapd's target list"""
  return [{"latitude": LAT0 + math.degrees(y / R_EARTH), "longitude": LON0 + math.degrees(x / (R_EARTH * math.cos(math.radians(LAT0)))),
           "velocity": float(vv)} for (x, y), vv in zip(xy, v, strict=True)]


def kinked_straight(kink_v, turn_deg=20., spacing=30., n=21):
  """a straight road with one node turning by turn_deg in the middle; that node reads kink_v, the rest 40 m/s"""
  h = np.where(np.arange(n - 1) < n // 2, 0., math.radians(turn_deg))
  xy = np.concatenate([[[0., 0.]], np.cumsum(np.stack([np.cos(h), np.sin(h)], 1) * spacing, 0)])
  v = np.full(n, 40.)
  v[n // 2] = kink_v
  return to_targets(xy, v), n // 2


def arc(radius, step=20., n=40):
  a = np.arange(n) * step / radius
  xy = np.stack([radius * np.sin(a), radius * (1 - np.cos(a))], 1)
  return to_targets(xy, np.full(n, math.sqrt(2. * radius)))


class TestRelaxedVelocities:
  def test_kink_relaxed_to_long_stretch_speed(self):
    tg, j = kinked_straight(15.)
    # 20 deg over the 100 m chords: sqrt(2.0 * 100 / 0.349) m/s; the 150 m chords read higher
    assert relaxed_velocities(tg)[j] == pytest_approx(23.94)

  def test_real_arc_unchanged(self):
    tg = arc(200.)
    np.testing.assert_allclose(relaxed_velocities(tg), [p["velocity"] for p in tg])

  def test_slow_target_not_relaxed(self):
    tg, j = kinked_straight(11.)
    assert relaxed_velocities(tg)[j] == 11.

  def test_slow_floor_inclusive(self):
    tg, j = kinked_straight(11.18)
    assert relaxed_velocities(tg)[j] == pytest_approx(23.94)

  def test_not_clearly_gentler_unchanged(self):
    tg, j = kinked_straight(20.)  # 23.94 < 1.2 * 20
    assert relaxed_velocities(tg)[j] == 20.
    tg, j = kinked_straight(19.9)  # 23.94 >= 1.2 * 19.9
    assert relaxed_velocities(tg)[j] == pytest_approx(23.94)

  def test_needs_path_on_both_sides(self):
    tg, j = kinked_straight(15., n=7)  # 90 m either side: shorter than the 100 m chord
    assert relaxed_velocities(tg)[j] == 15.

  def test_slowest_chord_wins(self):
    # a second, 70 deg turn 120 m on: only the 150 m chord reaches it, and it reads tighter than the 100 m one
    n, j = 21, 10
    h = np.radians(np.where(np.arange(n - 1) < j, 0., np.where(np.arange(n - 1) < j + 4, 20., 90.)))
    xy = np.concatenate([[[0., 0.]], np.cumsum(np.stack([np.cos(h), np.sin(h)], 1) * 30., 0)])
    v = np.full(n, 40.)
    v[j] = 15.
    assert relaxed_velocities(to_targets(xy, v))[j] == pytest_approx(23.10)

  def test_repeated_node_takes_its_twin(self):
    tg, j = kinked_straight(15.)
    tg = tg[:j] + [dict(tg[j])] + tg[j:]
    out = relaxed_velocities(tg)
    assert out[j] == pytest_approx(23.94) and out[j + 1] == pytest_approx(23.94)

  def test_straight_capped(self):
    # due north: no turn at all reads as an infinite curve speed, capped at 40 m/s
    tg = to_targets([[0., 30. * i] for i in range(21)], [15.] * 21)
    assert relaxed_velocities(tg)[10] == 40.

  def test_short_list(self):
    tg = to_targets([[0., 0.], [30., 0.]], [10., 10.])
    np.testing.assert_array_equal(relaxed_velocities(tg), [10., 10.])


def pytest_approx(x):
  return pytest.approx(x, abs=0.01)


def straight_path(reach=600.):
  px = np.linspace(0., reach, 33)
  return px, np.zeros_like(px)


def kinked_path(x0, turn_rad, reach=600.):
  """the model's path, straight to x0 then off at turn_rad"""
  px = np.linspace(0., reach, 301)
  return px, np.where(px < x0, 0., (px - x0) * math.tan(turn_rad))


class TestModelCurveSpeed:
  def test_straight_is_unbounded(self):
    assert model_curve_speed(*straight_path(), 300.) == math.inf

  def test_kink_reads_shortest_chord(self):
    # 0.15 rad over the 30 m chords: sqrt(2.0 * 30 / 0.15); the 50 m chords read 25.82
    assert model_curve_speed(*kinked_path(300., 0.15), 300.) == pytest_approx(20.0)

  def test_needs_50_m_past(self):
    assert model_curve_speed(*straight_path(349.), 300.) is None
    assert model_curve_speed(*straight_path(350.), 300.) == math.inf

  def test_needs_50_m_before(self):
    assert model_curve_speed(*straight_path(), 49.) is None
    assert model_curve_speed(*straight_path(), 50.) == math.inf

  def test_path_must_run_forward(self):
    px, py = straight_path()
    px[16], px[17] = px[17], px[16]
    assert model_curve_speed(px, py, 300.) is None

  def test_non_finite_path(self):
    px, py = straight_path()
    py[5] = math.nan
    assert model_curve_speed(px, py, 300.) is None
    px, py = straight_path()
    px[-1] = math.nan
    assert model_curve_speed(px, py, 300.) is None

  def test_unknown_position(self):
    assert model_curve_speed(*straight_path(), math.nan) is None


def test_ahead_distance():
  # 300 m east of the car, which heads east (90 deg) or north (0 deg)
  lat, lon = LAT0, LON0 + math.degrees(300. / (R_EARTH * math.cos(math.radians(LAT0))))
  assert ahead_distance(LAT0, LON0, 90., [lat], [lon])[0] == pytest_approx(300.)
  assert ahead_distance(LAT0, LON0, 0., [lat], [lon])[0] == pytest_approx(0.)


class TestRelaxLatch:
  def setup_method(self):
    cp = structs.CarParams(brand="mazda", openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.36)
    self.m = SmartCruiseControlMap(cp)
    self.tg, self.j = kinked_straight(15.)

  def relax(self, v_ego, tg, path=None, travelled=0., frames=10):
    # the car at the first node, heading east along the road; by default the camera sees it straight
    self.m.v_ego = v_ego
    for _ in range(frames):
      self.m.target_velocities = tg
      self.m.relax_targets(straight_path() if path is None else path, (LAT0, LON0, 90., travelled))
    return [p["velocity"] for p in self.m.target_velocities]

  def test_ten_frames_to_relax(self):
    assert self.relax(25., self.tg, frames=9)[self.j] == 15.
    assert self.relax(25., self.tg, frames=1)[self.j] == pytest_approx(23.94)

  def test_one_tight_frame_restarts_the_count(self):
    self.relax(25., self.tg, frames=9)
    self.relax(25., self.tg, kinked_path(300., 0.3), frames=1)
    assert self.relax(25., self.tg, frames=9)[self.j] == 15.
    assert self.relax(25., self.tg, frames=1)[self.j] == pytest_approx(23.94)

  def test_frames_it_cannot_judge_do_not_count_or_reset(self):
    self.relax(25., self.tg, frames=9)
    assert self.relax(25., self.tg, straight_path(100.), frames=5)[self.j] == 15.
    assert self.relax(25., self.tg, frames=1)[self.j] == pytest_approx(23.94)

  def test_repeated_node_votes_once_per_frame(self):
    tg = self.tg[:self.j] + [dict(self.tg[self.j])] + self.tg[self.j:]
    assert self.relax(25., tg, frames=9)[self.j] == 15.
    assert self.relax(25., tg, frames=1)[self.j] == pytest_approx(23.94)

  def test_votes_forgotten_when_unlisted(self):
    self.relax(25., self.tg, frames=9)
    self.relax(25., self.tg[self.j + 1:], frames=1)
    assert self.relax(25., self.tg, frames=1)[self.j] == 15.

  def test_tightest_agreeing_reading_taken(self):
    self.relax(25., self.tg, kinked_path(300., 0.15), frames=1)  # reads 20.0
    assert self.relax(25., self.tg, frames=9)[self.j] == pytest_approx(20.0)

  def test_publishes_mapd_velocity(self):
    self.relax(25., self.tg)
    assert self.m.target_velocities[self.j]["mapd_v"] == 15.
    assert self.m.target_velocities[0]["mapd_v"] == 40.

  def test_not_relaxed_without_camera(self):
    self.m.v_ego = 25.
    for _ in range(10):
      self.m.target_velocities = self.tg
      self.m.relax_targets(None, (LAT0, LON0, 90., 0.))
    assert self.m.target_velocities[self.j]["velocity"] == 15.

  def test_camera_must_cover(self):
    # the kink is 300 m ahead: a path to 349 m does not reach 50 m past it; one to 351 m does
    assert self.relax(25., self.tg, straight_path(349.))[self.j] == 15.
    assert self.relax(25., self.tg, straight_path(351.))[self.j] == pytest_approx(23.94)

  def test_travelled_since_fix(self):
    # 10 m driven since the fix: the kink is 290 m ahead, so a 341 m path covers it
    assert self.relax(25., self.tg, straight_path(341.), travelled=10.)[self.j] == pytest_approx(23.94)
    self.m.relaxed = {}
    assert self.relax(25., self.tg, straight_path(339.), travelled=10.)[self.j] == 15.

  def test_camera_sees_real_curve(self):
    # the camera shows a 0.3 rad bend there (14.1 m/s): a real curve, kept at mapd's 15
    assert self.relax(25., self.tg, kinked_path(300., 0.3))[self.j] == 15.
    assert self.m.relaxed == {}

  def test_closer_tighter_reading_lowers(self):
    self.relax(25., self.tg)
    assert self.relax(25., self.tg, kinked_path(300., 0.15))[self.j] == pytest_approx(20.0)
    # a gentler reading later does not raise it back
    assert self.relax(25., self.tg)[self.j] == pytest_approx(20.0)

  def test_never_lowered_under_mapd(self):
    self.relax(25., self.tg)
    assert self.relax(25., self.tg, kinked_path(300., 0.6))[self.j] == 15.

  def test_lowered_in_town_too(self):
    self.relax(25., self.tg)
    assert self.relax(15., self.tg, kinked_path(300., 0.15))[self.j] == pytest_approx(20.0)

  def test_kept_when_camera_cannot_judge(self):
    self.relax(25., self.tg)
    assert self.relax(25., self.tg, straight_path(100.))[self.j] == pytest_approx(23.94)
    assert self.relax(25., self.tg, (np.array([0.]), np.array([0.])))[self.j] == pytest_approx(23.94)

  def test_lower_of_map_and_camera(self):
    # the camera reads 20.0, the map's long chords 23.94
    assert self.relax(25., self.tg, kinked_path(300., 0.15))[self.j] == pytest_approx(20.0)

  def test_camera_reading_must_be_clearly_gentler(self):
    # 0.27 rad reads 14.91 m/s, 0.17 rad 18.79 m/s: both under 1.2 * 15
    assert self.relax(25., self.tg, kinked_path(300., 0.27))[self.j] == 15.
    assert self.relax(25., self.tg, kinked_path(300., 0.165))[self.j] == pytest_approx(19.07)

  def test_relaxed_at_speed(self):
    assert self.relax(20.1, self.tg)[self.j] == pytest_approx(23.94)

  def test_not_relaxed_in_town(self):
    assert self.relax(20.0, self.tg)[self.j] == 15.

  def test_kept_after_slowing(self):
    self.relax(25., self.tg)
    assert self.relax(10., self.tg)[self.j] == pytest_approx(23.94)

  def test_forgotten_once_passed(self):
    self.relax(25., self.tg)
    self.relax(10., self.tg[self.j + 1:])
    assert self.relax(10., self.tg)[self.j] == 15.

  def test_first_value_kept(self):
    self.relax(25., self.tg)
    # the list now runs out 90 m past the kink: at speed it is no longer relaxable, the earlier value holds
    assert self.relax(25., self.tg[:self.j + 4])[self.j] == pytest_approx(23.94)

  def test_mapd_lower_value_wins_over_latch(self):
    self.relax(25., self.tg)
    tg = [dict(p) for p in self.tg]
    tg[self.j]["velocity"] = 8.
    assert self.relax(10., tg)[self.j] == 8.

  def test_dropout_keeps_latch(self):
    self.relax(25., self.tg)
    self.relax(19., [])
    assert self.relax(19., self.tg)[self.j] == pytest_approx(23.94)

  def test_unrelaxed_node_tracks_mapd(self):
    # under the 25 mph floor: never relaxed, so mapd's later values come through, up and down
    tg = [dict(p) for p in self.tg]
    tg[self.j]["velocity"] = 10.
    assert self.relax(25., tg)[self.j] == 10.
    tg[self.j]["velocity"] = 9.
    assert self.relax(25., tg)[self.j] == 9.
    tg[self.j]["velocity"] = 11.
    assert self.relax(25., tg)[self.j] == 11.

  def test_relaxed_once_road_is_known(self):
    # first listed 90 m short of the 100 m chord (not relaxable), then with the whole road
    assert self.relax(25., self.tg[:self.j + 4])[self.j] == 15.
    assert self.relax(25., self.tg)[self.j] == pytest_approx(23.94)

  def test_list_not_mutated(self):
    self.relax(25., self.tg)
    assert self.tg[self.j]["velocity"] == 15.
