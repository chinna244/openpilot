"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.sunnypilot import IntEnumBase


class Policy(IntEnumBase):
  car_state_only = 0
  map_data_only = 1
  car_state_priority = 2
  map_data_priority = 3
  combined = 4
  vision_priority = 5


class OffsetType(IntEnumBase):
  off = 0
  fixed = 1
  percentage = 2
  per_speed = 3


# Per-speed offset: a limit up to each bound (driver's unit) uses that range's offset; a limit above the last bound uses
# the last range. Offsets are stored in the driver's unit, like SpeedLimitValueOffset.
TIER_LIMITS_IMPERIAL = (25, 30, 35, 40, 45, 55)  # mph: one range per common US limit, 60+ last
TIER_LIMITS_METRIC = (40, 50, 60, 70, 80, 100)   # km/h
TIER_PARAMS = tuple(f"SpeedLimitOffsetTier{i}" for i in range(1, len(TIER_LIMITS_IMPERIAL) + 2))


def tier_bounds(is_metric: bool) -> list[tuple[int | None, int | None]]:
  """(lower, upper) of each range in the driver's unit; None = open end. A range covers lower < limit <= upper."""
  limits = TIER_LIMITS_METRIC if is_metric else TIER_LIMITS_IMPERIAL
  return [(limits[i - 1] if i > 0 else None, limits[i] if i < len(limits) else None) for i in range(len(limits) + 1)]


class Mode(IntEnumBase):
  off = 0
  information = 1
  warning = 2
  assist = 3
