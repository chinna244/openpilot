"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Smart Cruise Control (vision and map curve speed) as chinna244/openpilot develop runs it
(openpilot/sunnypilot/selfdrive/controls/lib/smart_cruise_control, mazda-smart-can-filter-dev 197598c0f6), for the Mazda
port. Same laws and constants; StarPilot has no sunnypilot cereal structs, Params period or navd, so the pieces they
provided are defined here, and the map targets come from StarPilot's mapd (scc/mapd_targets.py) instead of the
MapTargetVelocities param develop's mapd writes.
"""
from dataclasses import dataclass
from enum import IntEnum

from openpilot.common.constants import CV

MIN_V = 20 * CV.KPH_TO_MS  # Do not operate under 20 km/h
PARAMS_UPDATE_PERIOD = 3  # seconds, develop's openpilot.sunnypilot.PARAMS_UPDATE_PERIOD


class VisionState(IntEnum):
  # develop's custom.LongitudinalPlanSP.SmartCruiseControl.VisionState
  disabled = 0
  enabled = 1
  entering = 2
  turning = 3
  leaving = 4
  overriding = 5


class MapState(IntEnum):
  # develop's custom.LongitudinalPlanSP.SmartCruiseControl.MapState
  disabled = 0
  enabled = 1
  turning = 2
  overriding = 3


@dataclass
class Coordinate:
  # the part of develop's openpilot.sunnypilot.navd.helpers.Coordinate the map controller uses
  latitude: float
  longitude: float
