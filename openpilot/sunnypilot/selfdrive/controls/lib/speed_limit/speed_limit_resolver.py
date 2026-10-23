"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD, get_sanitize_int_param
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import LIMIT_MAX_MAP_DATA_AGE, LIMIT_ADAPT_ACC
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import COMMIT_FRAC, OP_LONG_COAST_A, get_planning_limits
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy, OffsetType

SpeedLimitSource = custom.LongitudinalPlanSP.SpeedLimit.Source

ALL_SOURCES = tuple(SpeedLimitSource.schema.enumerants.values())

# A GPS message logged further ahead of now than this is not trusted either.
MAP_FIX_FUTURE_TOLERANCE = 1.  # s

# A car (camera) limit below this is a misread sign, not a road limit: route 00000169 t207 and t234 read 5 mph
# with no map limit, and the assist braked at -1.2 m/s2 toward 6 mph. 9 mph keeps 10 mph and 15 km/h signs.
CAR_LIMIT_MIN = 9 * CV.MPH_TO_MS  # m/s


class SpeedLimitResolver:
  limit_solutions: dict[custom.LongitudinalPlanSP.SpeedLimit.Source, float]
  distance_solutions: dict[custom.LongitudinalPlanSP.SpeedLimit.Source, float]
  v_ego: float
  speed_limit: float
  speed_limit_last: float
  speed_limit_final: float
  speed_limit_final_last: float
  distance: float
  source: custom.LongitudinalPlanSP.SpeedLimit.Source
  speed_limit_offset: float

  def __init__(self, CP=None):
    self.params = Params()
    self.frame = -1
    # swap to the upcoming lower limit once we are inside the braking distance the
    # platform can actually deliver (openpilot long coasts to it); the fixed LIMIT_ADAPT_ACC remains the no-CP fallback
    if CP is None:
      self._adapt_acc = LIMIT_ADAPT_ACC
    else:
      a_budget = OP_LONG_COAST_A if CP.openpilotLongitudinalControl else get_planning_limits(CP).a_budget
      self._adapt_acc = -COMMIT_FRAC * a_budget

    self._gps_location_service = get_gps_location_service(self.params)
    self.limit_solutions = {}  # Store for speed limit solutions from different sources
    self.distance_solutions = {}  # Store for distance to current speed limit start for different sources

    self.policy = self.params.get("SpeedLimitPolicy", return_default=True)
    self.policy = get_sanitize_int_param(
      "SpeedLimitPolicy",
      Policy.min().value,
      Policy.max().value,
      self.params
    )
    self._policy_to_sources_map = {
      Policy.car_state_only: [SpeedLimitSource.car],
      Policy.map_data_only: [SpeedLimitSource.map],
      Policy.car_state_priority: [SpeedLimitSource.car, SpeedLimitSource.map],
      Policy.map_data_priority: [SpeedLimitSource.map, SpeedLimitSource.car],
      Policy.combined: [SpeedLimitSource.car, SpeedLimitSource.map],
    }
    self.source = SpeedLimitSource.none
    for source in ALL_SOURCES:
      self._reset_limit_sources(source)

    self.is_metric = self.params.get_bool("IsMetric")
    self.offset_type = get_sanitize_int_param(
      "SpeedLimitOffsetType",
      OffsetType.min().value,
      OffsetType.max().value,
      self.params
    )
    self.offset_value = self.params.get("SpeedLimitValueOffset", return_default=True)

    self.speed_limit = 0.
    self.speed_limit_last = 0.
    self.speed_limit_final = 0.
    self.speed_limit_final_last = 0.
    self.speed_limit_offset = 0.

  def update_speed_limit_states(self) -> None:
    self.speed_limit_final = self.speed_limit + self.speed_limit_offset

    if self.speed_limit > 0.:
      self.speed_limit_last = self.speed_limit
      self.speed_limit_final_last = self.speed_limit_final

  @property
  def speed_limit_valid(self) -> bool:
    return self.speed_limit > 0.

  @property
  def speed_limit_last_valid(self) -> bool:
    return self.speed_limit_last > 0.

  def update_params(self):
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.policy = self.params.get("SpeedLimitPolicy", return_default=True)
      self.is_metric = self.params.get_bool("IsMetric")
      self.offset_type = self.params.get("SpeedLimitOffsetType", return_default=True)
      self.offset_value = self.params.get("SpeedLimitValueOffset", return_default=True)

  def _get_speed_limit_offset(self) -> float:
    if self.offset_type == OffsetType.off:
      return 0
    elif self.offset_type == OffsetType.fixed:
      return float(self.offset_value * (CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS))
    elif self.offset_type == OffsetType.percentage:
      return float(self.offset_value * 0.01 * self.speed_limit)
    else:
      raise NotImplementedError("Offset not supported")

  def _reset_limit_sources(self, source: custom.LongitudinalPlanSP.SpeedLimit.Source) -> None:
    self.limit_solutions[source] = 0.
    self.distance_solutions[source] = 0.

  def _get_from_car_state(self, sm: messaging.SubMaster) -> None:
    self._reset_limit_sources(SpeedLimitSource.car)
    speed_limit = sm['carStateSP'].speedLimit
    self.limit_solutions[SpeedLimitSource.car] = speed_limit if speed_limit >= CAR_LIMIT_MIN else 0.
    self.distance_solutions[SpeedLimitSource.car] = 0.

  def _get_from_map_data(self, sm: messaging.SubMaster) -> None:
    self._reset_limit_sources(SpeedLimitSource.map)
    self._process_map_data(sm)

  @staticmethod
  def _message_age(sm: messaging.SubMaster, service: str) -> float | None:
    """Seconds since the last message of a service, on the monotonic clock it was logged on; None if too old (or
    logged in the future). unixTimestampMillis is wall-clock time and cannot be aged against monotonic."""
    age = time.monotonic() - sm.logMonoTime[service] * 1e-9
    if not -MAP_FIX_FUTURE_TOLERANCE <= age <= LIMIT_MAX_MAP_DATA_AGE:
      return None
    return age

  def _gps_fix_age(self, sm: messaging.SubMaster) -> float | None:
    """Age of the last GPS message, which must carry a fix; None without a current fix. Without one mapd keeps
    the last road it matched, and its limit must not reach the planner (routes 00000152-159)."""
    if not sm[self._gps_location_service].hasFix:
      return None
    return self._message_age(sm, self._gps_location_service)

  def _process_map_data(self, sm: messaging.SubMaster) -> None:
    map_data = sm['liveMapDataSP']

    if self._gps_fix_age(sm) is None:
      return

    # plannerd ignores liveMapDataSP liveness: a mapd that stopped publishing leaves its last limit in the SubMaster
    map_data_age = self._message_age(sm, 'liveMapDataSP')
    if map_data_age is None:
      return

    speed_limit = map_data.speedLimit if map_data.speedLimitValid else 0.
    next_speed_limit = map_data.speedLimitAhead if map_data.speedLimitAheadValid else 0.

    self._calculate_map_data_limits(sm, speed_limit, next_speed_limit, map_data_age)

  def _calculate_map_data_limits(self, sm: messaging.SubMaster, speed_limit: float, next_speed_limit: float,
                                 map_data_age: float) -> None:
    map_data = sm['liveMapDataSP']

    # mapd measures the distance from the localizer's position as it publishes, not from the last GPS fix, which
    # the localizer has already driven past: only the message's own age is still to come off
    distance_since_publish = self.v_ego * max(map_data_age, 0.)
    distance_to_speed_limit_ahead = max(0., map_data.speedLimitAheadDistance - distance_since_publish)

    self.limit_solutions[SpeedLimitSource.map] = speed_limit
    self.distance_solutions[SpeedLimitSource.map] = 0.

    # FIXME-SP: this is not working as expected
    # only a lower limit ahead is braked for early: above both, a higher one would lift the current limit before its sign
    if 0. < next_speed_limit < self.v_ego and (speed_limit <= 0. or next_speed_limit < speed_limit):
      adapt_time = (next_speed_limit - self.v_ego) / self._adapt_acc
      adapt_distance = self.v_ego * adapt_time + 0.5 * self._adapt_acc * adapt_time ** 2

      if distance_to_speed_limit_ahead <= adapt_distance:
        self.limit_solutions[SpeedLimitSource.map] = next_speed_limit
        self.distance_solutions[SpeedLimitSource.map] = distance_to_speed_limit_ahead

  def _get_source_solution_according_to_policy(self) -> custom.LongitudinalPlanSP.SpeedLimit.Source:
    sources_for_policy = self._policy_to_sources_map[Policy(self.policy)]

    if Policy(self.policy) != Policy.combined:
      # They are ordered in the order of preference, so we pick the first that's non-zero
      for source in sources_for_policy:
        if self.limit_solutions[source] > 0.:
          return source
      return SpeedLimitSource.none

    sources_with_limits = [(s, limit) for s, limit in [(s, self.limit_solutions[s]) for s in sources_for_policy] if limit > 0.]
    if sources_with_limits:
      return min(sources_with_limits, key=lambda x: x[1])[0]

    return SpeedLimitSource.none

  def _resolve_limit_sources(self, sm: messaging.SubMaster) -> tuple[float, float, custom.LongitudinalPlanSP.SpeedLimit.Source]:
    """Get limit solutions from each data source"""
    self._get_from_car_state(sm)
    self._get_from_map_data(sm)

    source = self._get_source_solution_according_to_policy()
    speed_limit = self.limit_solutions[source] if source else 0.
    distance = self.distance_solutions[source] if source else 0.

    return speed_limit, distance, source

  def update(self, v_ego: float, sm: messaging.SubMaster) -> None:
    self.v_ego = v_ego
    self.update_params()

    self.speed_limit, self.distance, self.source = self._resolve_limit_sources(sm)
    self.speed_limit_offset = self._get_speed_limit_offset()

    self.update_speed_limit_states()

    self.frame += 1
