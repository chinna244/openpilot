"""
SCC-M's map targets from StarPilot's mapd. develop's mapd (pfeiferj v1) writes MapTargetVelocities, a JSON list of
{latitude, longitude, velocity}: one point per node of the current and next ways, velocity = sqrt(TARGET_LAT_ACCEL /
curvature) at its fixed TARGET_LAT_ACCEL of 2.0 m/s^2 (its GetTargetVelocities). StarPilot's mapd publishes the same nodes
ahead as mapdExtendedOut.path with each node's (signed) curvature; its own targetVelocity follows StarPilot's map curve
settings, so the velocity is rebuilt from the curvature at develop's 2.0. A node without curvature has no target (v1
leaves an empty entry at 0, 0 there, which never binds).
"""
import math

MAPD_SERVICE = 'mapdExtendedOut'
TARGET_LAT_ACCEL = 2.0  # m/s^2, develop's mapd


def targets_from_path(path) -> list[dict]:
  targets = []
  for point in path:
    lat, lon, curvature = float(point.latitude), float(point.longitude), abs(float(point.curvature))
    if curvature > 0. and all(math.isfinite(v) for v in (lat, lon, curvature)):
      targets.append({"latitude": lat, "longitude": lon, "velocity": math.sqrt(TARGET_LAT_ACCEL / curvature)})
  return targets


class MapdTargets:
  """The last path mapd published, handed to the map controller as develop's param read would: a fresh list every call
  (the Curve Speed profile scales it in place)."""
  def __init__(self):
    self._targets: list[dict] = []

  def update(self, sm) -> None:
    try:
      if sm.updated[MAPD_SERVICE]:
        self._targets = targets_from_path(sm[MAPD_SERVICE].path)
    except (KeyError, AttributeError):
      self._targets = []

  def targets(self) -> list[dict]:
    return [dict(p) for p in self._targets]
