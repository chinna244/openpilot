"""Car GPS fallback on the existing ubloxd-owned standard logging channel."""
import math

from openpilot.cereal import messaging
from openpilot.starpilot.gps.publisher import FIELDS
from openpilot.starpilot.gps.source import GPS_MAX_AGE_NS, observation


class CarGpsExternal:
  def __init__(self):
    self.native_ns = 0
    self.native_jetson = False  # the native fix in native_ns was the Jetson's standing in for the ublox
    self.last_car_ns = 0
    self.last_publish_ns = 0

  def forget_jetson(self) -> None:
    """The Jetson's fix stands in no longer (its fallback ended, or its Mazda session): if the native fix holding car
    fixes back was the Jetson's, forget it; a genuine ublox fix keeps its priority."""
    if self.native_jetson:
      self.native_ns = 0
      self.native_jetson = False

  def select(self, sm, now_ns, native=None, jetson=False):
    """Keep native fixes first; repeat car fixes at the standard 10Hz cadence.

    The source stamp is retained as the standard Event stamp. Repeated polling
    cannot refresh an old CAN fix, and the source remains explicitly 'car'.
    """
    if native is not None:
      self.native_ns = int(native.logMonoTime) if native.valid and native.gpsLocationExternal.hasFix else 0
      self.native_jetson = bool(jetson) and self.native_ns > 0
    if 0 < self.native_ns <= now_ns <= self.native_ns + GPS_MAX_AGE_NS:
      return native
    fix = observation(sm, 'starpilotCarState', now_ns)
    if fix is None:
      return native
    stamp, gps = fix
    # Preserve the existing decoder's real metric motion and uncertainty fields.
    numbers = (gps.altitude, gps.speed, gps.bearingDeg, gps.verticalAccuracy,
               gps.bearingAccuracyDeg, gps.speedAccuracy, *gps.vNED)
    if (len(gps.vNED) != 3 or not all(math.isfinite(v) for v in numbers) or
        not 0 <= gps.speed <= 200 or not 0 <= gps.bearingDeg < 360 or
        min(gps.verticalAccuracy, gps.bearingAccuracyDeg, gps.speedAccuracy) < 0):
      return native
    if stamp < self.last_car_ns or stamp == self.last_car_ns and now_ns - self.last_publish_ns < 100_000_000:
      # Keep the standard channel alive without refreshing the source stamp.
      return None
    event = messaging.new_message('gpsLocationExternal', valid=True, logMonoTime=stamp)
    event.gpsLocationExternal = {key: getattr(gps, key) for key in FIELDS}
    event.gpsLocationExternal.source = 'car'
    # The companion has no satellite-count field: zero means unknown.
    event.gpsLocationExternal.satelliteCount = 0
    self.last_car_ns, self.last_publish_ns = stamp, now_ns
    return event
