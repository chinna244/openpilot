"""
The Jetson's USB GPS, as it rides on Jetlink's telemetry (port of zoompilot develop's jetlink_adapter/gps.py).

The Jetson's server merges `gps` into the telemetry it appends to a frame's reply when the comma asks for it (about
once a second). modeld hands each new one here and writes it to JETSON_GPS_PATH, a JSON file in /dev/shm (StarPilot
has no free reserved schema slot for it), for ubloxd's arbiter (system/ubloxd/gps_arbiter.py) and pigeond's aiding.
Nothing here decides whether the fix is good enough to use: it only checks that it is well formed.

Standard library only: modeld calls it on its frame loop.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
import time

JETSON_GPS_PATH = "/dev/shm/jetson_gps.json"
# the receiver fixes once a second
TELEMETRY_PERIOD = 1.0  # s


def _ask() -> None:
  pass


class JetsonGps:
  """New fixes from the link: each telemetry dict once, while the Jetlink model drives (the small model has no link,
  so no fix)."""

  def __init__(self):
    self._last_state: dict | None = None
    self._last_ask = -math.inf

  def ask(self):
    """model.run's after_enqueue about once a second, else None: a frame run with one asks the Jetson for its
    telemetry, and so its fix. jetlink's own once-a-second ask seldom fires (model_state: prepare's log resets the
    timer send reads just after), which left fixes 1 to 11 s apart. For a modeld without a chestnut's after_enqueue."""
    now = time.monotonic()
    if now - self._last_ask < TELEMETRY_PERIOD:
      return None
    self._last_ask = now
    return _ask

  def poll(self, model) -> dict | None:
    state = getattr(getattr(model, 'client', None), 'last_state', None)
    if not isinstance(state, dict) or state is self._last_state:
      return None
    self._last_state = state
    gps = state.get('gps')
    return gps if isinstance(gps, dict) else None


def _number(gps: dict, key: str, default: float | None = None) -> float | None:
  value = gps.get(key, default)
  if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
    return default
  return float(value)


def record(gps: dict, mono: float) -> dict | None:
  """`gps` as the record ubloxd reads, stamped with modeld's monotonic time `mono`; None, leaving it unwritten, when
  the fix is missing its position, time or age, or has one out of range."""
  lat, lon = _number(gps, 'lat'), _number(gps, 'lon')
  utc_ms, age_ms = _number(gps, 'utc_ms'), _number(gps, 'age_ms')
  if lat is None or lon is None or utc_ms is None or age_ms is None:
    return None
  if abs(lat) > 90 or abs(lon) > 180 or not 0 <= age_ms < 2**32 or utc_ms <= 0:
    return None
  bearing, hdop = _number(gps, 'course_deg'), _number(gps, 'hdop')
  return {
    'latitude': lat,
    'longitude': lon,
    'altitude': _number(gps, 'alt_m', 0.),
    'speed': max(0., _number(gps, 'speed_mps', 0.)),
    'bearingDeg': bearing,  # None when the receiver gave none (JSON has no NaN)
    'hdop': hdop,
    'satelliteCount': int(min(max(_number(gps, 'sats', 0.), 0), 255)),
    'fixQuality': int(min(max(_number(gps, 'quality', 0.), 0), 255)),
    'fixType': int(min(max(_number(gps, 'fix_type', 0.), 0), 255)),
    'unixTimestampMillis': int(utc_ms),
    'ageMs': int(age_ms),
    'mono': mono,
  }


def write(rec: dict, path: str = JETSON_GPS_PATH) -> None:
  """Atomically: a reader never sees half a fix."""
  directory = os.path.dirname(path) or "."
  with tempfile.NamedTemporaryFile(mode="w", dir=directory, prefix=".jetson_gps.", delete=False) as f:
    json.dump(rec, f)
  os.replace(f.name, path)


def mazda_session() -> bool:
  """The Jetson fallback is the Mazda port's: only this session's CarParams (cleared at manager start and each onroad
  transition) saying Mazda admits a record, so a record left by a Mazda session never reaches another car."""
  try:
    from openpilot.common.params import Params
    from openpilot.starpilot.car.mazda.last_car import car_brand
    return car_brand(Params(), live_only=True) == "mazda"
  except Exception:
    return False


class JetsonGpsReader:
  """The latest record in `path`, each one once; at most every `period` s; only during a Mazda session (`gate`,
  asked at most once a second)."""

  def __init__(self, path: str = JETSON_GPS_PATH, period: float = 0.1, gate=None):
    self.path, self.period = path, period
    self.gate = gate if gate is not None else (lambda: mazda_session())
    self._gate_checked = -math.inf
    self._gate_open = False
    self._last_check = -math.inf
    self._last_mtime: int | None = None
    self._last_mono: float | None = None

  @property
  def gate_open(self) -> bool:
    """The last gate answer: consumers drop any fix they hold once it closes."""
    return self._gate_open

  def poll(self, now: float) -> dict | None:
    if now - self._last_check < self.period:
      return None
    self._last_check = now
    if now - self._gate_checked >= 1.0:
      self._gate_checked = now
      self._gate_open = bool(self.gate())
    if not self._gate_open:
      return None
    try:
      mtime = os.stat(self.path).st_mtime_ns
      if mtime == self._last_mtime:
        return None
      self._last_mtime = mtime
      with open(self.path) as f:
        rec = json.load(f)
    except (OSError, ValueError):
      return None
    if not isinstance(rec, dict) or rec.get('mono') == self._last_mono:
      return None
    self._last_mono = rec.get('mono')
    return rec


class JetsonGpsHook:
  """The Jetlink model modeld runs, from StarPilot's jetlink adapter (modeld.py stays upstream's byte for byte: Auto
  Learn pins its source). bind() makes the joined model ask for the Jetson's telemetry about once a second; frame(), each
  frame after it runs, writes a new fix for ubloxd and pigeond."""

  def __init__(self, path: str = JETSON_GPS_PATH):
    self.path = path
    self.gps = JetsonGps()

  def bind(self, model) -> None:
    """Wrap the joined model's run: a frame modeld runs without an after_enqueue of its own (no chestnut) gets the
    once-a-second ask. jetlink's JoiningModelState.run hands it to whichever model drives; the small one just calls
    it, a no-op."""
    run = getattr(model, 'run', None)
    if not callable(run) or getattr(run, '_jetson_gps', False):
      return

    def run_asking(bufs, transforms, inputs, after_enqueue=None, *args, **kwargs):
      if after_enqueue is None:
        after_enqueue = self.gps.ask()
      return run(bufs, transforms, inputs, after_enqueue, *args, **kwargs)
    run_asking._jetson_gps = True  # type: ignore[attr-defined]
    model.run = run_asking

  def frame(self, model) -> bool:
    """True when a new fix was written."""
    if (gps := self.gps.poll(model)) is None or (rec := record(gps, time.monotonic())) is None:
      return False
    write(rec, self.path)
    return True
