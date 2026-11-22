from collections import deque
import math

from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.ramp_camera import split_at_bound_ramp
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.ramp_map import RAMP_GRAPH_PATH, load_graph
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.ramp_match import RampMatcher, scores
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.ramp_sign_binding import MapSignBinder

GPS_MAX_AGE = 0.45
SIGN_MAX_AGE = 60.0
NO_OFFER = (0.0, 0, 0)


class RampAdvisoryResolver:
  def __init__(self, path=RAMP_GRAPH_PATH):
    self.binder = None
    try:
      self.roads, self.forks, self.scale = load_graph(path)
      if not self.forks:
        return
      self.matcher = RampMatcher(self.roads, self.forks, self.scale)
      self.binder = MapSignBinder(self.roads, self.forks, self.scale)
    except (OSError, ValueError, TypeError, KeyError, IndexError, OverflowError):
      return
    self.reset()
    self.enabled = False

  def reset(self):
    self.matcher.reset()
    self.fixes = deque(maxlen=24)
    self.motion = deque(maxlen=960)
    self.frames = deque(maxlen=420)
    self.bound = None
    self.last_sign_at = -math.inf
    self.last_match_at = -math.inf
    self.active = None
    self.blocked_way = None
    self.blocked_at = -math.inf
    self.posted = 0.0
    self.posted_change_at = -math.inf
    self.sign_conflict = None

  def update(self, now, pose, sign, speed, cruise_speed, enabled, posted, camera=None):
    if self.binder is None:
      return NO_OFFER
    if not enabled:
      if self.enabled:
        self.reset()
      self.enabled = False
      return NO_OFFER
    self.enabled = True

    if pose and not (all(math.isfinite(v) for v in pose[:6]) and -85 <= pose[1] <= 85 and -180 <= pose[2] <= 180):
      pose = None
    self.motion.append((now, speed, 0.0))
    if pose and (not self.fixes or pose[0] > self.fixes[-1][0]):
      self.fixes.append(pose)
      while self.fixes and now - self.fixes[0][0] > 2.0:
        self.fixes.popleft()

    fresh = (
      pose is not None
      and pose[6]
      and 0 <= now - pose[0] <= GPS_MAX_AGE
      and 0 < pose[5] <= 15.0
      and all(math.isfinite(v) for v in pose[:6])
      and -85 <= pose[1] <= 85
      and -180 <= pose[2] <= 180
    )

    if sign and 0 <= now - sign[0] <= 2.0 and sign[0] > self.last_sign_at:
      at, mph, side = sign
      self.last_sign_at = at
      fixed = next((fix for fix in reversed(self.fixes) if 0 <= at - fix[0] <= GPS_MAX_AGE), None)
      binding = self.binder.binding(fixed, side) if fixed else None
      if binding and 10 <= mph <= 75:
        if self.bound is None or self.bound[1].way_id != binding.way_id:
          self.frames.clear()
        elif mph != self.bound[0]["mph"]:
          self.sign_conflict = (binding.way_id, now + SIGN_MAX_AGE)
          self.active = None
        self.bound = ({"last_at": at, "mph": mph, "side": side}, binding)
    if self.bound and now - self.bound[0]["last_at"] > SIGN_MAX_AGE:
      self.bound = None
      self.frames.clear()

    # A fresh, changed road limit supersedes the advisory.
    replaced = self.posted > 0.0 and posted > 0.0 and abs(posted - self.posted) > CV.MPH_TO_MS
    if replaced:
      self.posted_change_at = now
    if posted > 0.0:
      self.posted = posted

    if self.bound:
      if camera is not None:
        self.frames.append(camera)

    candidate = None
    if fresh and pose[0] > self.last_match_at:
      self.last_match_at = pose[0]
      point = self.matcher.scale * pose[1:3]
      nearby, matches = self.matcher.update(pose)
      if nearby:
        if len(matches) == 1:
          ramp = matches[0][1]
          candidate = self.roads[ramp]["id"], float(self.roads[ramp]["advisory"]), ramp
        if self.active:
          active_ramp = self.active[3]
          if active_ramp is None:
            if candidate is None or candidate[0] != self.active[0]:
              self.active = None
          else:
            keep = any(
              fork[1] == active_ramp
              and scores(point, pose[4], fork[4])[0] <= 24.0
              and scores(point, pose[4], fork[5])[2] - scores(point, pose[4], fork[4])[2] >= 3.0
              for fork in nearby
            )
            if not keep and (candidate is None or candidate[2] != active_ramp):
              self.active = None
      elif self.active:
        self.active = None

    if not fresh and camera is not None and self.bound and self.active is None:
      tracked, binding = self.bound
      # A camera split must be recent and must line up with the pre-outage fork.
      # A usable position always takes precedence over this bridge.
      if now - self.last_match_at <= 25.0 and (split := split_at_bound_ramp(tracked, binding, self.motion, self.frames)) is not None:
        if 0 <= now - split <= 0.4:
          candidate = binding.way_id, 0.0, None

    if replaced and self.active:
      self.active = None
    if self.active:
      way, target, started, ramp = self.active
      self.ramp_min_speed = min(self.ramp_min_speed, speed)
      if (
        now - started > 12.0
        or (speed > target + 5 * CV.MPH_TO_MS and speed > self.ramp_min_speed + 5 * CV.MPH_TO_MS)
        or cruise_speed <= target
        or (not fresh and now - self.last_match_at > 2.0 and ramp is not None)
      ):
        self.active = None

    if self.active is None and candidate is not None:
      way, mapped, ramp = candidate
      conflict = self.sign_conflict is not None and way == self.sign_conflict[0] and now < self.sign_conflict[1]
      if not conflict and now - self.posted_change_at > 12.0 and (way != self.blocked_way or now - self.blocked_at > 60.0):
        read = self.bound[0]["mph"] * CV.MPH_TO_MS if self.bound and self.bound[1].way_id == way else 0.0
        target = read or mapped
        # Disagreement leaves the normal SLA path in charge.
        if (
          10 * CV.MPH_TO_MS <= target <= 75 * CV.MPH_TO_MS
          and (not read or not mapped or abs(read - mapped) <= 2 * CV.MPH_TO_MS)
          and target + CV.MPH_TO_MS < cruise_speed
        ):
          self.blocked_way, self.blocked_at = way, now
          self.active = way, target, now, ramp
          self.ramp_min_speed = speed

    if self.active:
      _, target, start, _ = self.active
      return target, max(1, int(start * 1e9)), int((now + 0.55) * 1e9)
    return NO_OFFER


def camera_sample(now, model):
  if len(model.laneLines) != 4 or len(model.roadEdges) != 2 or len(model.roadEdgeStds) != 2 or len(model.laneLineProbs) != 4:
    return None
  tracks = [model.position, *model.laneLines, *model.roadEdges]
  if any(len(line.x) < 2 or len(line.x) != len(line.y) for line in tracks):
    return None
  return now, list(model.laneLineProbs), list(model.roadEdgeStds), [(list(line.x), list(line.y)) for line in tracks]
