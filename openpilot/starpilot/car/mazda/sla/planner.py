"""plannerd side of the Mazda SLA port: filter-dev's resolver on StarPilot's sources, and the mirror of card's session.

Per modelV2 tick: build the source snapshot, run the resolver, mirror the session card published, and hand plannerd
the plan cap (the mirror's ramped target) and the slcState.mazdaSla fields card and selfdrived read.
"""
from dataclasses import replace

from openpilot.common.gps import get_gps_location_service
from openpilot.starpilot.car.mazda.sla import V_CRUISE_UNSET
from openpilot.starpilot.car.mazda.sla.mirror import SpeedLimitAssistMirror
from openpilot.starpilot.car.mazda.sla.resolver import Sources, SpeedLimitResolver
from openpilot.starpilot.speed_limits.map_source import EVENT_MAX_AGE_NS


DASHBOARD_MAX_AGE_NS = 500_000_000  # card publishes it at 100 Hz
SESSION_MAX_AGE_NS = 200_000_000  # card's session, 100 Hz


def usable(sm, service: str, now_ns: int | None = None, max_age_ns: int | None = None) -> bool:
  """Seen and valid, and with a max age, no older than that (a message from the future is not usable either)."""
  if not (sm.seen.get(service, False) and sm.valid.get(service, False)):
    return False
  if max_age_ns is None:
    return True
  stamp = int(sm.logMonoTime.get(service, 0))
  return 0 < stamp <= now_ns and now_ns - stamp <= max_age_ns


def sources_from_sm(sm, now_ns: int, gps_service: str) -> Sources:
  """StarPilot's source messages as filter-dev's resolver reads its own (see resolver.py). Only valid messages count; the
  resolver ages vision, map and GPS itself (as filter-dev), the dashboard reading is aged here."""
  car = vision = 0.
  vision_ns = map_ns = gps_ns = 0
  map_limit = map_ahead = map_ahead_d = 0.
  gps_fix = False
  if usable(sm, 'slcDashboardObservation', now_ns, DASHBOARD_MAX_AGE_NS):
    d = sm['slcDashboardObservation']
    car = float(d.speedMps) if str(d.status) == 'valid' else 0.
  if usable(sm, 'slcVisionObservation'):
    v = sm['slcVisionObservation'].vision
    vision = float(v.speedMps) if str(v.status) == 'valid' else 0.
    vision_ns = int(sm.logMonoTime['slcVisionObservation'])
  if usable(sm, 'mapdOut'):
    m = sm['mapdOut']
    map_limit, map_ahead, map_ahead_d = float(m.speedLimit), float(m.nextSpeedLimit), float(m.nextSpeedLimitDistance)
    map_ns = int(sm.logMonoTime['mapdOut'])
  if usable(sm, gps_service):
    gps_fix = bool(sm[gps_service].hasFix)
    gps_ns = int(sm.logMonoTime[gps_service])
  return Sources(now_ns, car, vision, vision_ns, map_limit, map_ahead, map_ahead_d, map_ns, gps_fix, gps_ns)


class MazdaSlaPlanner:
  def __init__(self, CP, params, offset_fn):
    self.gps_service = get_gps_location_service(params)
    self.resolver = SpeedLimitResolver(CP, offset_fn=offset_fn)
    self.mirror = SpeedLimitAssistMirror()
    self.servo_v_target = V_CRUISE_UNSET
    self.limiter_active = False
    self.matched_road: tuple[str | int, float] | None = None
    self.vision_fence_ns = 0

  def services(self) -> list[str]:
    return ['slcDashboardObservation', 'slcVisionObservation', 'mapdOut', self.gps_service, 'starpilotCarState']

  def step(self, sm, now_ns: int) -> float | None:
    """Returns the plan cap in m/s, or None for no cap."""
    v_ego = float(sm['carState'].vEgo)
    sources = sources_from_sm(sm, now_ns, self.gps_service)
    if usable(sm, 'mapdOut', now_ns, EVENT_MAX_AGE_NS):
      road = sm['mapdOut']
      if getattr(road, 'wayId', 0) > 0 and str(road.roadStatus) in ('matchedLimit', 'matchedNoLimit'):
        # Ignore serialization precision when identifying the same posted limit.
        identity = (road.wayRef or road.roadName or road.wayName or int(road.wayId), round(sources.map_limit, 3))
        if self.matched_road is not None and identity != self.matched_road:
          self.vision_fence_ns = sources.map_ns
        self.matched_road = identity
    # The camera republishes its last sign while no new sign is detected. A
    # fresh heartbeat cannot carry that evidence onto a different mapped road or speed-limit zone.
    if sources.vision > 0. and self.vision_fence_ns:
      supported_ns = int(sm['slcVisionObservation'].vision.observedMonoTime)
      if not self.vision_fence_ns <= supported_ns <= now_ns:
        sources = replace(sources, vision=0.)
    self.resolver.update(v_ego, sources)
    # card's session counts only while valid and current: an old one must not keep a cap on the plan
    session = sm['starpilotCarState'].mazdaSlaSession if usable(sm, 'starpilotCarState', now_ns, SESSION_MAX_AGE_NS) else None
    self.mirror.update(session, v_ego)
    cap = self.mirror.plan_v_target
    # the plan target filter-dev's ICBM walks the dash to (LP_SP.vTarget), from the set speed this tick saw
    v_cruise_kph = float(sm['carState'].vCruise)
    v_cruise = v_cruise_kph / 3.6 if 0. < v_cruise_kph < V_CRUISE_UNSET else V_CRUISE_UNSET
    self.limiter_active = 0. < cap < v_cruise
    self.servo_v_target = min(v_cruise, cap) if self.limiter_active else v_cruise
    return cap if 0. < cap < V_CRUISE_UNSET else None

  def fill(self, out) -> None:
    r, m = self.resolver, self.mirror
    out.speedLimit = float(r.speed_limit)
    out.speedLimitFinalLast = float(r.speed_limit_final_last)
    out.speedLimitValid = bool(r.speed_limit_valid)
    out.speedLimitLastValid = bool(r.speed_limit_last_valid)
    out.source = int(r.source)
    out.assistState = int(m.state)
    out.vTarget = float(m.output_v_target)
    out.planVTarget = float(m.plan_v_target)
    out.announceCounter = int(m.announce_counter)
    out.setpointKph = float(m.setpoint_kph)
    out.servoVTarget = float(self.servo_v_target)
    out.limiterActive = bool(self.limiter_active)
    out.offerTarget = float(m.offer_target)
    out.sessionValid = bool(m.session_valid)
    out.roadChangeMonoTime = self.vision_fence_ns
