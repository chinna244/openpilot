"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import numpy as np
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.map_controller import car_frame, reversal_keep

LAT0, LON0 = 42.96, -87.93
R_EARTH = 6373000.


def straight_path(reach=200.):
  px = np.linspace(0., reach, 33)
  return px, np.zeros_like(px)


def bent_path(x0, turn_deg, reach=200.):
  """the model's path, straight to x0 then off to the right at turn_deg"""
  px = np.linspace(0., reach, 201)
  return px, np.where(px < x0, 0., (px - x0) * math.tan(math.radians(turn_deg)))


def nodes(start, headings_deg, step=20.):
  """map nodes (x ahead, y right) from start, each step m on at the next heading"""
  xy = [np.array(start, dtype=float)]
  for h in np.radians(headings_deg):
    xy.append(xy[-1] + step * np.array([math.cos(h), math.sin(h)]))
  xy = np.array(xy)
  return xy[:, 0], xy[:, 1]


def reversal():
  """ahead to 45 m, then back towards the car at 150 deg (route 17a's shape); node 2 turns"""
  return nodes((5., 0.), [0., 0.] + [150.] * 5)


class TestReversalKeep:
  def test_reversal_on_a_straight_path_dropped(self):
    np.testing.assert_array_equal(reversal_keep(*straight_path(), *reversal()), [False] * 5 + [True] * 3)

  def test_window_is_two_nodes_either_side(self):
    x, y = nodes((-95., 0.), [0.] * 7 + [150.] * 6)  # turning node 7 at 45 m
    np.testing.assert_array_equal(reversal_keep(*straight_path(), x, y), [True] * 5 + [False] * 5 + [True] * 4)

  def test_path_taking_the_bend_keeps_all(self):
    assert reversal_keep(*bent_path(20., 90.), *reversal()).all()

  def test_path_straight_within_15_deg(self):
    assert not reversal_keep(*bent_path(30., 14.), *reversal()).all()
    assert reversal_keep(*bent_path(30., 16.), *reversal()).all()

  def test_path_straight_to_90_m(self):
    assert not reversal_keep(*bent_path(91., 40.), *reversal()).all()
    assert reversal_keep(*bent_path(85., 40.), *reversal()).all()

  def test_node_within_60_m(self):
    x, y = nodes((5., 0.), [0., 0., 150., 150.], step=27.)  # turning node at 59 m
    assert not reversal_keep(*straight_path(), x, y).all()
    x, y = nodes((5., 0.), [0., 0., 150., 150.], step=28.)  # at 61 m
    assert reversal_keep(*straight_path(), x, y).all()

  def test_node_within_12_m_of_the_path(self):
    x, y = nodes((5., 11.), [0., 0., 150., 150.])  # a carriageway 11 m to the right
    assert not reversal_keep(*straight_path(), x, y).all()
    x, y = nodes((5., 13.), [0., 0., 150., 150.])
    assert reversal_keep(*straight_path(), x, y).all()

  def test_lateral_measured_from_the_path(self):
    # the path runs 14 deg right of the car's axis; the reversal lies on it, 14 m right of the axis at 58 m
    px = np.linspace(0., 200., 201)
    x, y = nodes((0., 0.), [14., 14., 164., 164.], step=29.)
    assert not reversal_keep(px * math.cos(math.radians(14.)), px * math.sin(math.radians(14.)), x, y).all()

  def test_path_seen_30_m_past_the_node(self):
    assert reversal_keep(*straight_path(74.), *reversal()).all()  # turning node at 45 m
    assert not reversal_keep(*straight_path(76.), *reversal()).all()

  def test_node_turn_over_100_deg(self):
    x, y = nodes((10., 0.), [55., 150., 150.], step=10.)
    assert reversal_keep(*straight_path(), x, y).all()
    x, y = nodes((10., 0.), [45., 150., 150.], step=10.)
    assert not reversal_keep(*straight_path(), x, y).all()

  def test_exit_over_135_deg_off_the_path(self):
    # a real corner leaves at 100-135 deg: kept
    x, y = nodes((10., 0.), [0., 130., 130.])
    assert reversal_keep(*straight_path(), x, y).all()
    x, y = nodes((10., 0.), [0., 140., 140.])
    assert not reversal_keep(*straight_path(), x, y).all()
    x, y = nodes((10., 0.), [0., -140., -140.])
    assert not reversal_keep(*straight_path(), x, y).all()

  def test_way_joining_from_the_side_kept(self):
    # the list comes in from the side at 135 deg and runs on along the car's road
    x, y = np.array([40., 20., 50.]), np.array([20., 0., 0.])
    assert reversal_keep(*straight_path(), x, y).all()

  def test_first_node_turned_against_the_path(self):
    # the list starts on the opposite carriageway, beside the car, heading back
    x, y = nodes((8., -3.), [180., 180., 180.])
    np.testing.assert_array_equal(reversal_keep(*straight_path(), x, y), [False] * 3 + [True])

  def test_hairpin_just_driven_keeps_the_road_ahead(self):
    # the list still starts on the leg before a hairpin the car has just come round (apex 15 m behind):
    # the passed hairpin's targets, up to the node 5 m ahead, are dropped; the road on from 25 m is kept
    x, y = np.array([10., -15., 5., 25., 45., 65.]), np.array([8., 4., 0., 0., 0., 0.])
    np.testing.assert_array_equal(reversal_keep(*straight_path(), x, y), [False] * 3 + [True] * 3)

  def test_first_node_out_of_reach_not_judged(self):
    x, y = nodes((70., 0.), [180., 180., 180.])  # coming towards the car from 70 m
    assert reversal_keep(*straight_path(), x, y).all()

  def test_path_heading_from_its_chord(self):
    # a first step of 0.2 m, 0.04 m to the right (11 deg): the chord to 30 m reads straight, so an
    # exit at 140 deg is 140 off the path, not 129
    px, py = straight_path()
    px, py = np.concatenate([[0., 0.2], px[1:]]), np.concatenate([[0., 0.04], py[1:]])
    x, y = nodes((10., 0.), [0., 140., 140.])
    assert not reversal_keep(px, py, x, y).all()

  def test_degenerate_inputs_keep_all(self):
    x, y = reversal()
    assert reversal_keep(np.array([0.]), np.array([0.]), x, y).all()
    assert reversal_keep(np.array([0.]), np.array([0.]), *nodes((-35., 0.), [180., 180.])).all()  # nothing seen past a node behind
    assert reversal_keep(*straight_path(), x[:1], y[:1]).all()
    px, py = straight_path()
    py[3] = math.nan
    assert reversal_keep(px, py, x, y).all()
    xn = x.copy()
    xn[4] = math.nan
    assert reversal_keep(*straight_path(), xn, y).all()


@pytest.mark.parametrize("bearing, ahead, right", [(90., 300., 0.), (0., 0., 300.), (180., 0., -300.), (270., -300., 0.)])
def test_car_frame(bearing, ahead, right):
  lon = LON0 + math.degrees(300. / (R_EARTH * math.cos(math.radians(LAT0))))  # 300 m east
  x, y = car_frame(LAT0, LON0, bearing, [LAT0], [lon])
  assert x[0] == pytest.approx(ahead, abs=0.5) and y[0] == pytest.approx(right, abs=0.5)
