"""Curve corrections: the driver's gas and brake presses against the curve controller, kept as a bounded scale on its
lateral-acceleration table at each table speed. Only corrections teach it: curves driven by hand are not a limit
(the driver had no reason to slow), so they are never recorded. Pure; the caller owns Params."""

import json
import math

KEY = "CurveCorrectionData"  # Params: the saved scales (JSON)
DOC_LIMIT = 4096  # bytes: a larger saved document is unreadable everywhere (planner and settings alike)
SPEEDS = (15.6, 24.6, 31.3)  # m/s, the vision solver's table speeds (35, 55, 70 mph)
LIMIT = 0.10  # each scale stays within 1 +- LIMIT
UP_FRAC = 0.25  # a gas correction moves this share of the way to the lateral acceleration the driver took
DOWN_STEP = 0.03  # a brake correction lowers by this, weighted by speed
WATCH_S = 6.0  # after a gas press, how long the driver's curve is watched
ENTER_S = 2.0  # a gas press the car is not yet in a bend for must reach one within this
IN_CURVE_LAT = 1.0  # m/s2: the watched curve counts while at least this
BRAKE_CURVE_LAT = 1.3  # m/s2: a brake press in a curve at least this tight is a correction
MIN_SPEED = 5.0  # m/s
LEAD_HEADWAY_S = 3.0  # a brake press with a lead this close is for the lead, not the curve
SAVE_INTERVAL_S = 30.0


def runs_on(CP) -> bool:
  """The vehicles whose Curve Speed Controller is the vision solver with these corrections (the Mazda planner's SCC)."""
  return CP is not None and getattr(CP, 'brand', '') == 'mazda' and bool(getattr(CP, 'openpilotLongitudinalControl', False))


def finite(x) -> bool:
  try:
    return type(x) in (int, float) and math.isfinite(x)
  except OverflowError:  # an int too large for a float
    return False


def clamp_scale(x: float) -> float:
  return min(max(x, 1.0 - LIMIT), 1.0 + LIMIT)


def speed_weights(v: float) -> tuple[float, ...]:
  """Linear share of each table speed at v; below the first or above the last, all of it goes to that end."""
  if v <= SPEEDS[0]:
    return (1.0, 0.0, 0.0)
  if v >= SPEEDS[-1]:
    return (0.0, 0.0, 1.0)
  w = [0.0, 0.0, 0.0]
  for i in range(len(SPEEDS) - 1):
    if SPEEDS[i] <= v <= SPEEDS[i + 1]:
      f = (v - SPEEDS[i]) / (SPEEDS[i + 1] - SPEEDS[i])
      w[i], w[i + 1] = 1.0 - f, f
      break
  return tuple(w)


UNIT = (1.0,) * len(SPEEDS)


def parse(document: object) -> tuple[float, ...] | None:
  """Scales from a stored document (no document: no correction); None if it is malformed."""
  if document is None:
    return UNIT
  try:
    data = json.loads(document) if isinstance(document, (str, bytes)) else document
    scales = data["scales"] if type(data) is dict and data.get("version") == 1 else None
    if type(scales) is list and len(scales) == len(SPEEDS) and all(finite(s) for s in scales):
      return tuple(clamp_scale(float(s)) for s in scales)
  except (TypeError, ValueError, KeyError, OverflowError, RecursionError, UnicodeDecodeError):
    pass
  return None


def load(document: object) -> tuple[float, ...]:
  """Scales from a stored document; anything malformed is no correction at all."""
  scales = parse(document)
  return scales if scales is not None else UNIT


def describe(scales: tuple[float, ...]) -> str:
  """Settings text: the learned change at each table speed."""
  return " · ".join(f"{round(v * 2.236936 / 5) * 5} mph {round((s - 1.0) * 100):+d}%" for v, s in zip(SPEEDS, scales, strict=True))


class Corrections:
  def __init__(self, scales: tuple[float, ...] | None = None):
    self.scales = tuple(scales) if scales is not None else (1.0,) * len(SPEEDS)
    self.revision = 0
    self.saved_revision = 0
    self.was_gas = True  # a pedal already held when this starts is not a press
    self.was_brake = True
    self.watch_left = 0.0
    self.watch_peak = 0.0
    self.watch_v = 0.0
    self.watch_table = 0.0
    self.watch_age = 0.0
    self.watch_in_curve = False
    self.was_long_active = False
    self.was_slowing = False

  def document(self) -> dict:
    return {"version": 1, "scales": [round(s, 4) for s in self.scales]}

  @property
  def dirty(self) -> bool:
    return self.revision != self.saved_revision

  def _apply(self, scales: tuple[float, ...]) -> None:
    scales = tuple(clamp_scale(s) for s in scales)
    if scales != self.scales:
      self.scales = scales
      self.revision += 1

  def idle(self, *, gas: bool, brake: bool) -> None:
    """The controller is off: no watch survives, and a pedal held now is not a press later."""
    self.watch_left, self.watch_peak = 0.0, 0.0
    self.was_gas, self.was_brake = gas, brake
    self.was_long_active = self.was_slowing = False

  def step(self, *, dt: float, v_ego: float, lat_accel: float, gas: bool, brake: bool, long_active: bool, engaged: bool,
           slowing: bool, close_lead: bool, table_lat: float) -> None:
    """One planner frame. slowing: the curve controller's target reached the published plan (the caller's last verdict).
    engaged: openpilot is engaged (a gas override keeps it engaged; a cancel does not).
    table_lat: the controller's unscaled lateral-acceleration ceiling at v_ego, m/s2.
    A pedal edge is judged against this frame or the one before it: the press itself can already have
    released long control or handed the plan back to cruise."""
    gas_edge, brake_edge = gas and not self.was_gas, brake and not self.was_brake
    was_long_active, was_slowing = self.was_long_active, self.was_slowing
    self.was_gas, self.was_brake = gas, brake
    self.was_long_active, self.was_slowing = long_active, slowing
    if not (finite(dt) and finite(v_ego) and finite(lat_accel) and finite(table_lat) and table_lat > 0):
      self.watch_left, self.watch_peak = 0.0, 0.0
      return

    if self.watch_left > 0.0:
      self.watch_age += dt
      self.watch_left -= dt
      if brake or not engaged:
        self.watch_left, self.watch_peak = 0.0, 0.0  # overshoot or disengage: the gas press teaches nothing
      elif lat_accel >= IN_CURVE_LAT:
        self.watch_in_curve = True
        if lat_accel > self.watch_peak:
          self.watch_peak, self.watch_v, self.watch_table = lat_accel, v_ego, table_lat
        if self.watch_left <= 0.0:
          self._finish_watch()
      elif self.watch_in_curve:
        self._finish_watch()  # the bend the press was for is over; a later one is not watched
      elif self.watch_age >= ENTER_S or self.watch_left <= 0.0:
        self.watch_left, self.watch_peak = 0.0, 0.0  # no bend followed the press

    if v_ego < MIN_SPEED:
      return
    controlled = long_active or was_long_active
    controller_slowing = slowing or was_slowing
    if gas_edge and controlled and controller_slowing and engaged:
      self.watch_left, self.watch_peak, self.watch_v = WATCH_S, 0.0, v_ego
      # a press already inside a bend watches that bend: its exit closes the watch
      self.watch_age, self.watch_in_curve = 0.0, lat_accel >= IN_CURVE_LAT
      if self.watch_in_curve:
        self.watch_peak, self.watch_v, self.watch_table = lat_accel, v_ego, table_lat
    elif brake_edge and controlled and not close_lead and (controller_slowing or lat_accel >= BRAKE_CURVE_LAT):
      w = speed_weights(v_ego)
      self._apply(tuple(s - DOWN_STEP * wi for s, wi in zip(self.scales, w, strict=True)))

  def _finish_watch(self) -> None:
    peak, v, table_lat = self.watch_peak, self.watch_v, self.watch_table
    self.watch_left, self.watch_peak = 0.0, 0.0
    if peak < IN_CURVE_LAT:
      return
    ratio = peak / table_lat  # the table at the speed the driver took the curve
    w = speed_weights(v)
    self._apply(tuple(s + UP_FRAC * wi * (ratio - s) if ratio > s else s for s, wi in zip(self.scales, w, strict=True)))
