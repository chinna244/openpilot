"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import random
import time

from openpilot.common.parameterized import parameterized

from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE

from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver, ALL_SOURCES, CAR_LIMIT_MIN
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy
from openpilot.common.test import OpenpilotTestCase

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source


def create_mock(properties, mocker):
  mock = mocker.MagicMock()
  for _property, value in properties.items():
    setattr(mock, _property, value)
  return mock


def setup_sm_mock(mocker):
  cruise_speed_limit = random.uniform(CAR_LIMIT_MIN, 120)
  live_map_data_limit = random.uniform(0, 120)

  car_state = create_mock({
    'gasPressed': False,
    'brakePressed': False,
    'standstill': False,
  }, mocker)
  car_state_sp = create_mock({
    'speedLimit': cruise_speed_limit,
  }, mocker)
  live_map_data = create_mock({
    'speedLimit': live_map_data_limit,
    'speedLimitValid': True,
    'speedLimitAhead': 0.,
    'speedLimitAheadValid': 0.,
    'speedLimitAheadDistance': 0.,
  }, mocker)
  gps_data = create_mock({
    'hasFix': True,
  }, mocker)
  sm_mock = mocker.MagicMock()
  sm_mock.__getitem__.side_effect = lambda key: {
    'carState': car_state,
    'liveMapDataSP': live_map_data,
    'carStateSP': car_state_sp,
    'gpsLocation': gps_data,
  }[key]
  sm_mock.logMonoTime = {'gpsLocation': int(time.monotonic() * 1e9), 'liveMapDataSP': int(time.monotonic() * 1e9)}
  return sm_mock


parametrized_policies = parameterized.expand(
  [
    (Policy.car_state_only, 'carStateSP', SpeedLimitSource.car),
    (Policy.car_state_priority, 'carStateSP', SpeedLimitSource.car),
    (Policy.map_data_only, 'liveMapDataSP', SpeedLimitSource.map),
    (Policy.map_data_priority, 'liveMapDataSP', SpeedLimitSource.map),
  ],
  names=["policy", "sm_key", "function_key"]
)


def resolver_class():
  return SpeedLimitResolver


class TestSpeedLimitResolverValidation(OpenpilotTestCase):

  @parameterized.expand(list(Policy), names=["policy"])
  def test_initial_state(self, resolver_class, policy):
    resolver = resolver_class()
    resolver.policy = policy
    for source in ALL_SOURCES:
      if source in resolver.limit_solutions:
        assert resolver.limit_solutions[source] == 0.
        assert resolver.distance_solutions[source] == 0.

  @parametrized_policies
  def test_resolver(self, resolver_class, policy, sm_key, function_key, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the resolver
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.speed_limit == source_speed_limit
    assert resolver.source == ALL_SOURCES[function_key]

  def test_resolver_combined(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.combined
    sm_mock = setup_sm_mock(mocker)
    socket_to_source = {'carStateSP': SpeedLimitSource.car, 'liveMapDataSP': SpeedLimitSource.map}
    minimum_key, minimum_speed_limit = min(
      ((key, sm_mock[key].speedLimit) for key in
       socket_to_source.keys()), key=lambda x: x[1])

    # Assert the resolver
    resolver.update(minimum_speed_limit, sm_mock)
    assert resolver.speed_limit == minimum_speed_limit
    assert resolver.source == socket_to_source[minimum_key]

  @parametrized_policies
  def test_parser(self, resolver_class, policy, sm_key, function_key, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = setup_sm_mock(mocker)
    source_speed_limit = sm_mock[sm_key].speedLimit

    # Assert the parsing
    resolver.update(source_speed_limit, sm_mock)
    assert resolver.limit_solutions[ALL_SOURCES[function_key]] == source_speed_limit
    assert resolver.distance_solutions[ALL_SOURCES[function_key]] == 0.

  @parameterized.expand(list(Policy), names=["policy"])
  def test_resolve_interaction_in_update(self, resolver_class, policy, mocker):
    v_ego = 50
    resolver = resolver_class()
    resolver.policy = policy

    sm_mock = setup_sm_mock(mocker)
    resolver.update(v_ego, sm_mock)

    # After resolution
    assert resolver.speed_limit is not None
    assert resolver.distance is not None
    assert resolver.source is not None

  @parameterized.expand(list(Policy), names=["policy"])
  def test_old_map_data_ignored(self, resolver_class, policy, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    sm_mock = mocker.MagicMock()
    sm_mock['gpsLocation'].hasFix = True
    sm_mock.logMonoTime = {'gpsLocation': int((time.monotonic() - 2 * LIMIT_MAX_MAP_DATA_AGE) * 1e9)}
    resolver._get_from_map_data(sm_mock)
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.
    assert resolver.distance_solutions[SpeedLimitSource.map] == 0.


MPH = 0.44704


def gps_sm(mocker, car_limit, map_limit, has_fix=True, fix_age=0., ahead_limit=0., ahead_distance=0., map_age=0.):
  """carStateSP, liveMapDataSP and gpsLocation as the resolver reads them; fix_age: seconds since the GPS message
  was logged, on the monotonic clock (negative: logged in the future); map_age: the same for liveMapDataSP."""
  car_state_sp = create_mock({'speedLimit': car_limit}, mocker)
  live_map_data = create_mock({
    'speedLimit': map_limit,
    'speedLimitValid': map_limit > 0.,
    'speedLimitAhead': ahead_limit,
    'speedLimitAheadValid': ahead_limit > 0.,
    'speedLimitAheadDistance': ahead_distance,
  }, mocker)
  gps_data = create_mock({'hasFix': has_fix}, mocker)
  sm_mock = mocker.MagicMock()
  sm_mock.__getitem__.side_effect = lambda key: {
    'carStateSP': car_state_sp,
    'liveMapDataSP': live_map_data,
    'gpsLocation': gps_data,
  }[key]
  sm_mock.logMonoTime = {'gpsLocation': int((time.monotonic() - fix_age) * 1e9),
                         'liveMapDataSP': int((time.monotonic() - map_age) * 1e9)}
  return sm_mock


MAP_POLICIES = [Policy.map_data_only, Policy.map_data_priority, Policy.combined]
UNUSABLE_FIXES = [
  ("no_fix", False, 0.),
  ("stale_fix", True, 2 * LIMIT_MAX_MAP_DATA_AGE),
  ("future_fix", True, -5.),
]


class TestSpeedLimitResolverGpsFix(OpenpilotTestCase):
  """Map limits need a current GPS fix: without one mapd keeps the last road it matched, and that limit must not
  reach the planner (routes 00000152-159: no fix all drive, a stale 30 mph limit capped a 55 mph highway)."""

  @parameterized.expand([(p, n, f, a) for p in MAP_POLICIES for n, f, a in UNUSABLE_FIXES],
                        names=["policy", "case", "has_fix", "fix_age"])
  def test_map_limit_needs_a_current_fix(self, resolver_class, policy, case, has_fix, fix_age, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    resolver.update(20., gps_sm(mocker, 0., 30 * MPH, has_fix=has_fix, fix_age=fix_age))
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.
    # nothing else to go on: no limit at all, never a low or zero target
    assert resolver.source == SpeedLimitSource.none
    assert resolver.speed_limit == 0. and not resolver.speed_limit_valid
    assert resolver.speed_limit_final == 0.

  @parameterized.expand([(p,) for p in MAP_POLICIES], names=["policy"])
  def test_map_limit_needs_a_current_map_message(self, resolver_class, policy, mocker):
    # plannerd ignores liveMapDataSP liveness, so a mapd that stopped publishing leaves its last limit in the
    # SubMaster; a fresh GPS fix must not make that limit current again
    resolver = resolver_class()
    resolver.policy = policy
    resolver.update(20., gps_sm(mocker, 0., 30 * MPH, fix_age=1., map_age=2 * LIMIT_MAX_MAP_DATA_AGE))
    assert resolver.limit_solutions[SpeedLimitSource.map] == 0.
    assert resolver.source == SpeedLimitSource.none
    assert resolver.speed_limit == 0.

  @parameterized.expand([(p,) for p in MAP_POLICIES], names=["policy"])
  def test_a_current_fix_uses_the_map(self, resolver_class, policy, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    resolver.update(20., gps_sm(mocker, 0., 30 * MPH, fix_age=1.))
    assert resolver.source == SpeedLimitSource.map
    assert resolver.speed_limit == 30 * MPH

  @parameterized.expand([(Policy.combined,), (Policy.map_data_priority,), (Policy.car_state_priority,)], names=["policy"])
  def test_stale_map_limit_falls_back_to_the_car(self, resolver_class, policy, mocker):
    # route 00000159 t232: the car reads 55, mapd still holds 30 from a road left long ago, GPS has no fix
    resolver = resolver_class()
    resolver.policy = policy
    resolver.update(26.8, gps_sm(mocker, 55 * MPH, 30 * MPH, has_fix=False))
    assert resolver.source == SpeedLimitSource.car
    assert resolver.speed_limit == 55 * MPH

  @parameterized.expand([(p,) for p in Policy], names=["policy"])
  def test_a_misread_low_car_limit_is_ignored(self, resolver_class, policy, mocker):
    # route 00000169 t207: the camera read 5 mph, the map had no limit; the assist braked toward 6 mph
    resolver = resolver_class()
    resolver.policy = policy
    resolver.update(6.8, gps_sm(mocker, 5 * MPH, 0.))
    assert resolver.limit_solutions[SpeedLimitSource.car] == 0.
    assert resolver.source == SpeedLimitSource.none
    assert resolver.speed_limit == 0.

  @parameterized.expand([(p,) for p in Policy], names=["policy"])
  def test_a_10_mph_car_limit_still_counts(self, resolver_class, policy, mocker):
    resolver = resolver_class()
    resolver.policy = policy
    resolver.update(6.8, gps_sm(mocker, 10 * MPH, 0.))
    if policy == Policy.map_data_only:
      assert resolver.source == SpeedLimitSource.none
    else:
      assert resolver.source == SpeedLimitSource.car
      assert resolver.speed_limit == 10 * MPH

  def test_car_state_only_ignores_the_map(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.car_state_only
    resolver.update(26.8, gps_sm(mocker, 55 * MPH, 30 * MPH, fix_age=1.))
    assert resolver.source == SpeedLimitSource.car
    assert resolver.speed_limit == 55 * MPH

  def test_combined_with_a_current_fix_still_takes_the_lower(self, resolver_class, mocker):
    resolver = resolver_class()
    resolver.policy = Policy.combined
    resolver.update(26.8, gps_sm(mocker, 55 * MPH, 45 * MPH, fix_age=1.))
    assert resolver.source == SpeedLimitSource.map
    assert resolver.speed_limit == 45 * MPH

  def test_distance_to_the_next_limit_counts_from_the_map_message(self, resolver_class, mocker):
    # 20 m/s, a 10 m/s limit 100 m ahead where mapd published 2 s ago: 60 m ahead now, inside the
    # 150 m it takes to slow at LIMIT_ADAPT_ACC, so the next limit applies
    resolver = resolver_class()
    resolver.policy = Policy.map_data_only
    resolver.update(20., gps_sm(mocker, 0., 15., ahead_limit=10., ahead_distance=100., map_age=2.))
    assert resolver.speed_limit == 10.
    assert abs(resolver.distance - 60.) < 0.5

  def test_an_older_fix_does_not_advance_a_current_distance(self, resolver_class, mocker):
    # review: mapd's 200 m is from the localizer's position now; a 9 s old fix (still accepted) took 180 m off
    # and slowed for the 10 m/s limit 50 m before its 150 m braking distance
    resolver = resolver_class()
    resolver.policy = Policy.map_data_only
    resolver.update(20., gps_sm(mocker, 0., 15., fix_age=9., ahead_limit=10., ahead_distance=200.))
    assert resolver.speed_limit == 15.
