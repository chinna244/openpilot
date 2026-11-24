"""filter-dev's SpeedLimitResolver (ce9225e178 speed_limit_resolver.py), Combined policy, on StarPilot's sources.

The logic below is the reference's, line for line. Only the inputs differ: update() takes a Sources snapshot built
by plannerd from StarPilot's own messages (sources_from_sm) instead of filter-dev's SubMaster, the clock is passed in,
and the offset is StarPilot's SLC offset (offset_fn) rather than sunnypilot's SpeedLimitOffsetType params.
  car    <- slcDashboardObservation (valid)          filter-dev carStateSP.speedLimit
  vision <- slcVisionObservation.vision (valid)      filter-dev customReserved10.speedLimit (StarPilot's sign reader)
  map    <- mapdOut speedLimit / nextSpeedLimit       filter-dev liveMapDataSP
  gps    <- the GPS service mapd follows (hasFix)     filter-dev gpsLocation(External)
"""
from collections.abc import Callable
from dataclasses import dataclass

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.starpilot.car.mazda.sla import Source

LIMIT_MAX_MAP_DATA_AGE = 10.  # s
LIMIT_ADAPT_ACC = -1.  # m/s^2
OP_LONG_COAST_A = 0.5  # smart_cruise_control.limits
COMMIT_FRAC = 0.7


class Policy:
  combined = 4


@dataclass(frozen=True)
class Sources:
  now_ns: int
  car: float = 0.                       # m/s
  vision: float = 0.                    # m/s, 0 for no sign
  vision_ns: int = 0                    # log time of the last vision message
  map_limit: float = 0.                 # m/s, 0 for none
  map_ahead: float = 0.
  map_ahead_distance: float = 0.
  map_ns: int = 0
  gps_fix: bool = False
  gps_ns: int = 0


SpeedLimitSource = Source

ALL_SOURCES = tuple(Source)

# A GPS message logged further ahead of now than this is not trusted either.
MAP_FIX_FUTURE_TOLERANCE = 1.  # s

# The last limit a source dropout falls back to must have held this long: route 00000009 at 220.05 s the camera read 35
# for 0.72 s turning into a parking lot as the map dropped out, and that 35 stayed the last limit for 47 s ("Press + for 41"
# twice, no limit anywhere). A reading that ends sooner gives way to the held one it took over from without a gap, if that
# one is lower; a lower short reading stays (it may be a real sign), and one that began after a dropout stays as before.
# Unless the camera itself took it back: a school 20 read before its plaque showed (route 12 484 s) is withdrawn while
# the camera still publishes, and the last limit goes back to what it was before that reading.
LAST_LIMIT_SETTLE = 2.  # s

# A car (camera) limit below this is a misread sign, not a road limit: route 00000169 t207 and t234 read 5 mph
# with no map limit, and the assist braked at -1.2 m/s2 toward 6 mph. 9 mph keeps 10 mph and 15 km/h signs.
CAR_LIMIT_MIN = 9 * CV.MPH_TO_MS  # m/s

# speed_limit_vision publishes at 5 Hz; a reading older than this means the process stopped or the toggle is off.
VISION_MAX_AGE = 1.  # s

# Combined policy: two sources this close agree on the limit (map and car round differently).
VOTE_TOLERANCE = 2 * CV.MPH_TO_MS  # m/s

# Combined policy: a source dropping out is no evidence of a new limit. When the map goes blank (mapd at a junction, a
# lost fix), the limit it gave holds until the map returns, two sources agree, or the car reads a sign since the dropout.
MAP_SETTLE_S = 3.  # a map limit is held through a dropout only once it held this long (a cross street shows for 2-4 s)
DROPOUT_FRESH_S = 1.  # a car reading this close before the dropout came with it (the junction's own sign)
DROPOUT_HOLD_MAX_M = 1000.  # backstop: driven this far with no map, the held limit gives way
# a map limit far below the speed driven, that the car's reading contradicts, is an overpass or cross street matched
# for a few seconds (routes 138/186: 25-30 for 4-14 s at 52-67 mph on a freeway); it waits for the car or this long
CROSS_MATCH_GAP = 20 * CV.MPH_TO_MS  # m/s
CROSS_MATCH_S = 15.
# a map limit just shown that the car's reading contradicts, with nothing backing it, is often a cross street matched at
# a junction for 1-4 s (route 6d t=36: East Norwich 25 on a 30 road at the start of the drive, nothing settled yet);
# the limit given so far stays until the map's has held this long or the car or camera agrees
NEW_MAP_WAIT_S = 5.
# an on-ramp has no limit of its own in the map: the town limit held through the blank stayed until the car read the
# freeway's sign near the merge (route 86 t=480-498: "50 ahead @ 611 m" for 18 s while 30 held). A fast road's limit
# ahead, shown this long with the map blank and above the limit given, is given as the limit now (Combined only)
AHEAD_RAISE_S = 4.
AHEAD_RAISE_MIN = 50 * CV.MPH_TO_MS  # m/s; a slower road's limit ahead is not raised to before its sign (town streets)
AHEAD_RAISE_MAX_M = 800.  # only a fast road this close (ramps 478-701 m: routes 44/86/18f); not one kilometres on
AHEAD_RAISE_GAP_S = 1.  # the ahead value missing this briefly (a mapd frame) neither ends the raise nor restarts its clock


class SpeedLimitResolver:
  limit_solutions: dict[Source, float]
  distance_solutions: dict[Source, float]
  v_ego: float
  speed_limit: float
  speed_limit_last: float
  speed_limit_final: float
  speed_limit_final_last: float
  distance: float
  source: Source
  speed_limit_offset: float

  def __init__(self, CP=None, offset_fn: Callable[[float], float] | None = None):
    self.frame = -1
    self.offset_fn = offset_fn or (lambda limit: 0.)
    self._now = 0.
    # swap to the upcoming lower limit once we are inside the braking distance the
    # platform can actually deliver (openpilot long coasts to it); the fixed LIMIT_ADAPT_ACC remains the no-CP fallback
    if CP is None:
      self._adapt_acc = LIMIT_ADAPT_ACC
    else:
      a_budget = OP_LONG_COAST_A  # openpilot longitudinal only (eligible())
      self._adapt_acc = -COMMIT_FRAC * a_budget

    self.limit_solutions = {}  # Store for speed limit solutions from different sources
    self._map_vote_limit = 0.
    self._reset_hold()
    self.distance_solutions = {}  # Store for distance to current speed limit start for different sources

    self.policy = Policy.combined
    self._policy_to_sources_map = {
      Policy.combined: [SpeedLimitSource.vision, SpeedLimitSource.map, SpeedLimitSource.car],
    }
    self.source = SpeedLimitSource.none
    for source in ALL_SOURCES:
      self._reset_limit_sources(source)


    self.speed_limit = 0.
    self.speed_limit_last = 0.
    self.speed_limit_final = 0.
    self.speed_limit_final_last = 0.
    self.speed_limit_offset = 0.
    self._reading = 0.  # the current non-zero limit, its final, and how many frames it has held
    self._reading_final = 0.
    self._reading_frames = 0
    self._fallback: tuple[float, float] | None = None  # the held (limit, final) the current reading took over from
    self._before = (0., 0.)  # the last (limit, final) when the current reading began
    self._reading_source = SpeedLimitSource.none
    self._vision_alive = False  # speed_limit_vision publishing, with or without a sign

  def update_speed_limit_states(self) -> None:
    self.speed_limit_final = self.speed_limit + self.speed_limit_offset

    settle = int(LAST_LIMIT_SETTLE / DT_MDL)
    if self.speed_limit > 0.:
      if self.speed_limit != self._reading:
        self._before = (self.speed_limit_last, self.speed_limit_final_last)
        self._reading_source = self.source
        # only a reading that took over from a held one, with no dropout between, can give way to it
        if self._reading_frames >= settle:
          self._fallback = (self._reading, self._reading_final)
        elif self._reading <= 0.:
          self._fallback = None
        self._reading = self.speed_limit
        self._reading_frames = 0
      self._reading_frames += 1
      self._reading_final = self.speed_limit_final
      self.speed_limit_last = self.speed_limit
      self.speed_limit_final_last = self.speed_limit_final
    elif self._reading > 0.:
      # every source dropped out: a reading that never held does not raise the last limit
      if self._reading_frames < settle and self._reading_source == SpeedLimitSource.vision and self._vision_alive:
        # the camera withdrew a sign it read moments ago
        self.speed_limit_last, self.speed_limit_final_last = self._before
      elif self._reading_frames < settle and self._fallback is not None and self._fallback[0] < self._reading:
        self.speed_limit_last, self.speed_limit_final_last = self._fallback
      self._reading = 0.
      self._reading_frames = 0
      self._fallback = None

  @property
  def speed_limit_valid(self) -> bool:
    return self.speed_limit > 0.

  @property
  def speed_limit_last_valid(self) -> bool:
    return self.speed_limit_last > 0.

  def _get_speed_limit_offset(self) -> float:
    # StarPilot's SLC offset for this limit (filter-dev: SpeedLimitOffsetType / SpeedLimitValueOffset)
    return float(self.offset_fn(self.speed_limit)) if self.speed_limit > 0. else 0.

  def _reset_limit_sources(self, source: Source) -> None:
    self.limit_solutions[source] = 0.
    self.distance_solutions[source] = 0.

  def _get_from_car_state(self, sm: Sources) -> None:
    self._reset_limit_sources(SpeedLimitSource.car)
    speed_limit = sm.car
    self.limit_solutions[SpeedLimitSource.car] = speed_limit if speed_limit >= CAR_LIMIT_MIN else 0.
    self.distance_solutions[SpeedLimitSource.car] = 0.

  def _get_from_vision(self, sm: Sources) -> None:
    self._reset_limit_sources(SpeedLimitSource.vision)
    speed_limit = sm.vision
    age = self._now - sm.vision_ns * 1e-9
    self._vision_alive = -MAP_FIX_FUTURE_TOLERANCE <= age <= VISION_MAX_AGE
    if speed_limit >= CAR_LIMIT_MIN and self._vision_alive:
      self.limit_solutions[SpeedLimitSource.vision] = speed_limit

  def _get_from_map_data(self, sm: Sources) -> None:
    self._reset_limit_sources(SpeedLimitSource.map)
    self._map_vote_limit = 0.
    self._process_map_data(sm)

  def _message_age(self, log_ns: int) -> float | None:
    """Seconds since the last message of a service, on the monotonic clock it was logged on; None if too old (or
    logged in the future). unixTimestampMillis is wall-clock time and cannot be aged against monotonic."""
    age = self._now - log_ns * 1e-9
    if not -MAP_FIX_FUTURE_TOLERANCE <= age <= LIMIT_MAX_MAP_DATA_AGE:
      return None
    return age

  def _gps_fix_age(self, sm: Sources) -> float | None:
    """Age of the last GPS message, which must carry a fix; None without a current fix. Without one mapd keeps
    the last road it matched, and its limit must not reach the planner (routes 00000152-159)."""
    if not sm.gps_fix:
      return None
    return self._message_age(sm.gps_ns)

  def _process_map_data(self, sm: Sources) -> None:

    if self._gps_fix_age(sm) is None:
      self._reset_ahead_raise()
      return

    # plannerd ignores liveMapDataSP liveness: a mapd that stopped publishing leaves its last limit in the SubMaster
    map_data_age = self._message_age(sm.map_ns)
    if map_data_age is None:
      self._reset_ahead_raise()
      return

    speed_limit = sm.map_limit if sm.map_limit > 0. else 0.
    next_speed_limit = sm.map_ahead if sm.map_ahead > 0. else 0.

    self._calculate_map_data_limits(sm, speed_limit, next_speed_limit, map_data_age)

  def _calculate_map_data_limits(self, sm: Sources, speed_limit: float, next_speed_limit: float,
                                 map_data_age: float) -> None:

    # mapd measures the distance from the localizer's position as it publishes, not from the last GPS fix, which
    # the localizer has already driven past: only the message's own age is still to come off
    distance_since_publish = self.v_ego * max(map_data_age, 0.)
    distance_to_speed_limit_ahead = max(0., sm.map_ahead_distance - distance_since_publish)

    self._map_vote_limit = speed_limit  # the vote compares the road's own limit, not one swapped in early from ahead
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

    # the on-ramp raise: its clock runs per value shown with the map blank; it starts raising only above the limit given,
    # and then keeps raising while that value shows (the limit given becomes the raised one)
    # a sign the car reads after the raise began ends it (review: a work-zone 40 on the ramp), until the ahead value changes
    car = self.limit_solutions[SpeedLimitSource.car]
    if speed_limit > 0.:
      self._reset_ahead_raise()
    elif next_speed_limit >= AHEAD_RAISE_MIN - .1 and distance_to_speed_limit_ahead <= AHEAD_RAISE_MAX_M:
      self._ahead_raise_seen = self.frame
      if abs(next_speed_limit - self._ahead_raise) > .1:
        self._reset_ahead_raise()
        self._ahead_raise, self._ahead_raise_frame, self._ahead_raise_seen = next_speed_limit, self.frame, self.frame
    elif self.frame - self._ahead_raise_seen > int(AHEAD_RAISE_GAP_S / DT_MDL):
      self._reset_ahead_raise()
    if self._ahead_raising and car > 0. and abs(car - self._ahead_raise_car) > VOTE_TOLERANCE:
      self._ahead_raising, self._ahead_raise_done = False, True
    if (self.policy == Policy.combined and self._ahead_raise > 0. and not self._ahead_raise_done and
        not self._ahead_raising and self.frame - self._ahead_raise_frame >= int(AHEAD_RAISE_S / DT_MDL) and
        self._ahead_raise > self._given + VOTE_TOLERANCE):
      self._ahead_raising, self._ahead_raise_car = True, car
    if self._ahead_raising:
      self.limit_solutions[SpeedLimitSource.map] = self._ahead_raise
      self.distance_solutions[SpeedLimitSource.map] = 0.

  def _reset_ahead_raise(self) -> None:
    """The on-ramp raise (AHEAD_RAISE_S): the ahead limit, since when, last seen, raising, the car's reading then, ended."""
    self._ahead_raise, self._ahead_raise_frame, self._ahead_raise_seen = 0., 0, -10 ** 9
    self._ahead_raising, self._ahead_raise_car, self._ahead_raise_done = False, 0., False

  def _get_source_solution_according_to_policy(self) -> Source:
    sources = self._policy_to_sources_map[self.policy]
    if self.policy == Policy.combined:
      # 2 of 3 decide: routes 45/48 the camera read 40/45 on the highway for 10-14% of the time while map and car both
      # said 55/70; the first source in order that another agrees with wins, and only with no agreement the order alone
      found = [s for s in sources if self.limit_solutions[s] > 0.]
      for source in found:
        if any(abs(self.limit_solutions[source] - self.limit_solutions[other]) <= VOTE_TOLERANCE for other in found if other != source):
          return source
      # route 6a t=195: map had already taken its 25 ahead while map and car both said 30 -> no agreement, a school 20 won.
      # With no other majority, a source agreeing with the map's own limit (not the one ahead) still backs the map
      if SpeedLimitSource.map in found and self._map_vote_limit > 0. and any(
          abs(self._map_vote_limit - self.limit_solutions[other]) <= VOTE_TOLERANCE for other in found if other != SpeedLimitSource.map):
        return SpeedLimitSource.map
    # They are ordered in the order of preference, so we pick the first that's non-zero
    for source in sources:
      if self.limit_solutions[source] > 0.:
        return source
    return SpeedLimitSource.none

  def _resolve_limit_sources(self, sm: Sources) -> tuple[float, float, Source]:
    """Get limit solutions from each data source"""
    self._get_from_car_state(sm)
    self._get_from_map_data(sm)
    self._get_from_vision(sm)

    source = self._get_source_solution_according_to_policy()
    speed_limit = self.limit_solutions[source] if source else 0.
    distance = self.distance_solutions[source] if source else 0.

    if self.policy == Policy.combined:
      out = self._hold_on_evidence(speed_limit, distance, source)
      self._given, self._given_distance, self._given_source = out
      return out
    self._reset_hold()

    return speed_limit, distance, source

  def _usable_in_dropout(self, source: Source) -> bool:
    """While the map is blank: a source another one agrees with, or the car having read a sign since (or just before)."""
    limit = self.limit_solutions[source]
    backed = any(abs(limit - self.limit_solutions[other]) <= VOTE_TOLERANCE
                 for other in ALL_SOURCES if other != source and self.limit_solutions[other] > 0.)
    fresh = source == SpeedLimitSource.car and self._car_read_frame >= self._dropout_frame - int(DROPOUT_FRESH_S / DT_MDL)
    return limit > 0. and (backed or fresh)

  def _reset_hold(self) -> None:
    self._out, self._out_frame, self._out_settled = 0., 0, 0.  # the limit given, since when, and the last one that held
    self._map_own, self._map_own_frame, self._map_settled = 0., 0, 0.  # the same for the map's own limit
    self._dropout_frame: int | None = None
    self._dropout_m = 0.
    self._dropout_limit = 0.
    self._cross: tuple[int, float, float] | None = None  # (since, the map limit waited on, the limit held)
    self._car_reading = 0.
    self._given, self._given_distance, self._given_source = 0., 0., SpeedLimitSource.none  # the limit given last, held ones included
    self._wait_since: int | None = None  # the new-map wait: its first frame, the last frame it applied, the limit it holds
    self._wait_last = 0
    self._wait_held = 0.
    self._car_read_frame = -10 ** 9  # when the car last read a new limit
    self._reset_ahead_raise()

  def _hold_on_evidence(self, speed_limit: float, distance: float, source: Source):
    """Combined: the limit changes on evidence of a new one, not on a source dropping out. Routes 54/66/67/6a "Press +
    for 41" and 6c "Press - for 29": mapd blanked at a junction and the car's older reading, outvoted until then, took over."""
    settle = int(MAP_SETTLE_S / DT_MDL)
    car, own = self.limit_solutions[SpeedLimitSource.car], self._map_vote_limit
    if car > 0. and abs(car - self._car_reading) > .1:
      self._car_read_frame = self.frame
    if car > 0.:
      self._car_reading = car
    # the map's own limit settles on its own clock: which source won the vote (the camera agreeing with it) does not matter
    # a limit settles only while shown without a break: a blank restarts its clock (review: 50, blank, 50 for a frame)
    if own <= 0.:
      self._map_own = 0.
    elif abs(own - self._map_own) > .1:
      self._map_own, self._map_own_frame = own, self.frame
    elif self.frame - self._map_own_frame >= settle:
      self._map_settled = own

    dropout = self.limit_solutions[SpeedLimitSource.map] <= 0. and self._map_settled > 0. and self._out_settled > 0.
    if not dropout:
      self._dropout_frame = None
      self._dropout_m = 0.
    else:
      if self._dropout_frame is None:
        self._dropout_frame, self._dropout_limit = self.frame, self._out_settled
      self._dropout_m += self.v_ego * DT_MDL
      if self._dropout_m >= DROPOUT_HOLD_MAX_M:
        self._map_settled = 0.  # no map this far: what the sources give now stands
      elif source not in (SpeedLimitSource.map, SpeedLimitSource.none):
        usable = [s for s in self._policy_to_sources_map[Policy.combined] if self._usable_in_dropout(s)]
        if not usable:
          self._out = 0.
          return self._dropout_limit, 0., SpeedLimitSource.map
        source = usable[0]
        speed_limit, distance = self.limit_solutions[source], self.distance_solutions[source]

    # once waiting on a cross match, it waits until the car agrees, the map moves on, or CROSS_MATCH_S: not re-judged
    # every frame on a speed or a car reading that flickers
    if self._cross is not None:
      since, waited_on, _ = self._cross
      if abs(own - waited_on) > .1 or (car > 0. and abs(car - own) <= VOTE_TOLERANCE) or self.frame - since >= int(CROSS_MATCH_S / DT_MDL):
        self._cross = None
    if (self._cross is None and source == SpeedLimitSource.map and self._out_settled > 0. and 0. < own < self._map_settled - VOTE_TOLERANCE and
        own < self.v_ego - CROSS_MATCH_GAP and abs(car - self._map_settled) <= VOTE_TOLERANCE):
      self._cross = (self.frame, own, self._out_settled)
    if self._cross is not None:
      self._out = 0.
      return self._cross[2], 0., SpeedLimitSource.map

    # a new map limit the car contradicts, that nothing backs, waits NEW_MAP_WAIT_S (the limit given so far stays). The
    # clock is per limit held: the map changing value or blanking for less than the wait does not restart it, another
    # source agreeing with the map ends it, and holding a different limit starts a new one (review: junctions < 5 s apart).
    # Not while a limit ahead is in play (review: an off-ramp's early slow-down, a car agreeing with the ahead, an ahead
    # limit held as the current one)
    wait = int(NEW_MAP_WAIT_S / DT_MDL)
    backed = own > 0. and any(abs(own - self.limit_solutions[o]) <= VOTE_TOLERANCE
                              for o in ALL_SOURCES if o != SpeedLimitSource.map and self.limit_solutions[o] > 0.)
    if backed:
      self._wait_since = None
    if (source == SpeedLimitSource.map and own > 0. and not backed and abs(speed_limit - own) <= .1 and self._given_distance <= 0. and
        self.frame - self._map_own_frame < wait and car > 0. and abs(car - own) > VOTE_TOLERANCE):
      keep, keep_source = (self._given, self._given_source) if self._given > 0. else (car, SpeedLimitSource.car)
      if self._wait_since is None or self.frame - self._wait_last > wait or abs(keep - self._wait_held) > .1:
        self._wait_since, self._wait_held = self.frame, keep
      self._wait_last = self.frame
      if self.frame - self._wait_since < wait:
        return keep, 0., keep_source

    if source == SpeedLimitSource.none:
      self._out = 0.
    elif abs(speed_limit - self._out) > .1:
      self._out, self._out_frame = speed_limit, self.frame
    elif self.frame - self._out_frame >= settle:
      self._out_settled = speed_limit
    return speed_limit, distance, source

  def update(self, v_ego: float, sm: Sources) -> None:
    self.v_ego = v_ego
    self._now = sm.now_ns * 1e-9

    self.speed_limit, self.distance, self.source = self._resolve_limit_sources(sm)
    self.speed_limit_offset = self._get_speed_limit_offset()

    self.update_speed_limit_states()

    self.frame += 1
