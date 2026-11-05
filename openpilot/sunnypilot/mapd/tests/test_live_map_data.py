"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest
from openpilot.common.parameterized import parameterized

from openpilot.cereal import log
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.mapd.live_map_data import base_map_data, osm_map_data
from openpilot.sunnypilot.mapd.live_map_data.osm_map_data import OsmMapData

Status = log.LiveLocationKalman.Status
MPH = 0.44704


class FakeParams:
  """In-memory stand-in for the params mapd shares through /dev/shm/params."""
  values: dict = {}

  def __init__(self, *args, **kwargs):
    pass

  def get(self, key, *args, **kwargs):
    return self.values.get(key)

  def put(self, key, value, *args, **kwargs):
    self.values[key] = value


def make_map_data(mocker, map_limit=30 * MPH, next_limit=0., next_latitude=None):
  FakeParams.values = {
    'MapSpeedLimit': map_limit,
    'NextMapSpeedLimit': {'speedlimit': next_limit, 'latitude': next_latitude, 'longitude': -87.9} if next_limit else None,
  }
  mocker.patch.object(base_map_data, 'Params', FakeParams)
  mocker.patch.object(osm_map_data, 'Params', FakeParams)
  mocker.patch.object(base_map_data.messaging, 'SubMaster')
  mocker.patch.object(base_map_data.messaging, 'PubMaster')
  return OsmMapData()


def tick(map_data, status, position_valid, gps_ok=None, alive=True, msg_valid=True):
  map_data.sm.alive = {'liveLocationKalman': alive}
  map_data.sm.valid = {'liveLocationKalman': msg_valid}
  location = map_data.sm.__getitem__.return_value
  location.status = status
  location.positionGeodetic.valid = position_valid
  location.positionGeodetic.value = [43.0, -87.9, 180.]
  location.calibratedOrientationNED.value = [0., 0., 0.]
  location.gpsOK = (status == Status.valid and position_valid) if gps_ok is None else gps_ok
  map_data.tick()
  return map_data.pm.send.call_args.args[1].liveMapDataSP


class TestLiveMapDataPublisher(OpenpilotTestCase):
  """The limit mapd holds belongs to the position it was last given. Without a valid localizer the position stays at
  the last one stored (routes 00000155/156: 'South Clement Avenue' 30 mph all drive), so no limit may be marked valid."""

  def test_a_valid_localizer_publishes_the_limits(self, mocker):
    map_data = make_map_data(mocker, next_limit=20 * MPH, next_latitude=43.001)
    msg = tick(map_data, Status.valid, True)
    assert msg.speedLimitValid and msg.speedLimit == pytest.approx(30 * MPH)
    assert msg.speedLimitAheadValid and msg.speedLimitAhead == pytest.approx(20 * MPH)

  @parameterized.expand([
    ("warming_up_with_position", Status.uninitialized, True),   # 00000014a/14e: ~7-12 s after start, GPS already fixed
    ("warming_up_no_position", Status.uninitialized, False),
    ("position_not_valid", Status.valid, False),
    ("uncalibrated", Status.uncalibrated, True),
  ], names=["case", "status", "position_valid"])
  def test_an_invalid_localizer_publishes_no_valid_limit(self, case, status, position_valid, mocker):
    map_data = make_map_data(mocker, next_limit=20 * MPH, next_latitude=43.001)
    msg = tick(map_data, status, position_valid)
    assert not msg.speedLimitValid
    assert not msg.speedLimitAheadValid

  def test_limits_are_held_back_only_until_the_localizer_is_valid(self, mocker):
    map_data = make_map_data(mocker, next_limit=20 * MPH, next_latitude=43.001)
    for _ in range(10):
      msg = tick(map_data, Status.uninitialized, True)
      assert not msg.speedLimitValid and not msg.speedLimitAheadValid
    msg = tick(map_data, Status.valid, True)
    assert msg.speedLimitValid and msg.speedLimitAheadValid

  def test_losing_the_localizer_drops_the_limits_again(self, mocker):
    map_data = make_map_data(mocker)
    assert tick(map_data, Status.valid, True).speedLimitValid
    assert not tick(map_data, Status.uncalibrated, True).speedLimitValid

  @parameterized.expand([
    ("localizer_valid_gps_not_ok", Status.valid, True, False, True),   # a short GPS dropout: the localizer still dead-reckons
    ("localizer_warming_up_gps_ok", Status.uninitialized, True, True, False),
    ("localizer_valid_gps_ok", Status.valid, True, True, True),
    ("localizer_lost_gps_not_ok", Status.uninitialized, False, False, False),
  ], names=["case", "status", "position_valid", "gps_ok", "expected"])
  def test_the_message_is_valid_while_the_localizer_is(self, case, status, position_valid, gps_ok, expected, mocker):
    # SCC-Map drops its targets on this flag: it follows the position mapd was given, not gpsOK
    map_data = make_map_data(mocker)
    tick(map_data, status, position_valid, gps_ok)
    assert map_data.pm.send.call_args.args[1].valid is expected

  @parameterized.expand([("localizer_stopped", False, True), ("message_not_valid", True, False)],
                        names=["case", "alive", "msg_valid"])
  def test_a_stopped_localizer_publishes_no_valid_limit(self, case, alive, msg_valid, mocker):
    # review: the SubMaster keeps the last valid sample after liveLocationKalman stops, and each tick republished
    # its old position as fresh valid map data
    map_data = make_map_data(mocker, next_limit=20 * MPH, next_latitude=43.001)
    assert tick(map_data, Status.valid, True).speedLimitValid
    msg = tick(map_data, Status.valid, True, alive=alive, msg_valid=msg_valid)
    assert not msg.speedLimitValid and not msg.speedLimitAheadValid
    assert map_data.pm.send.call_args.args[1].valid is False
    # and it comes back with the localizer
    assert tick(map_data, Status.valid, True).speedLimitValid
