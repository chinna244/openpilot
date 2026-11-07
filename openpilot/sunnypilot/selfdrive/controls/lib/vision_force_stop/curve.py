"""StarPilot's frozen CEM curve predicate (starpilot/conditional_mode/curve.py, Dom 1d30dc3), without the filter."""

from __future__ import annotations

import math

from openpilot.sunnypilot.selfdrive.controls.lib.vision_force_stop.model_geometry import road_curvature as frozen_road_curvature


CRUISING_SPEED_MPS = 5.0
MIN_LATERAL_ACCEL_MPS2 = 1.3


def _finite(value: object, *, low: float, high: float) -> float | None:
  if isinstance(value, bool) or not isinstance(value, (int, float)):
    return None
  try:
    result = float(value)
  except (OverflowError, ValueError):
    return None
  return result if math.isfinite(result) and low <= result <= high else None


def frozen_raw_curve(model: object, speed_mps: float, curvature: float, left_blinker: bool, right_blinker: bool) -> tuple[bool, bool] | None:
  """Return (predicted road curve, measured curve), before CEM filtering."""
  speed = _finite(speed_mps, low=0.0, high=80.0)
  measured = _finite(curvature, low=-1.0, high=1.0)
  if speed is None or measured is None or type(left_blinker) is not bool or type(right_blinker) is not bool:
    return None
  road = frozen_road_curvature(model, speed)
  if road is None:
    return None
  road_curvature, _ = road
  # Algebraically equivalent to sqrt(1 / abs(curvature)) < vEgo,
  # including the zero-curvature false case without division by zero.
  predicted = speed > CRUISING_SPEED_MPS and abs(road_curvature) * speed * speed > 1.0 and not (left_blinker or right_blinker)
  driving = abs(speed * speed * measured) >= MIN_LATERAL_ACCEL_MPS2
  return predicted, driving
