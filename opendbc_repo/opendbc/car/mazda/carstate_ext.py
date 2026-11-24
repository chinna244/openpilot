from opendbc.car.mazda import context
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.dashboard_speed_limit import Status, Tracker as LimitTracker, parser_expiry


def decode_speed_sign(sign) -> float:
  """SPEED_SIGN_UNIT encodes display state and unit: 1 = mph, 2 = km/h, 0 = none.
  Plausibility: 90 covers the highest US posting (85 mph); the 7-bit all-ones 127 is a sentinel. Under 10 mph
  (15 km/h) is a misread, not a road limit (route 00000169: the camera read 5 mph), so it reads as none and the
  nav unit's limit can fill in."""
  speed_sign = sign["SPEED_SIGN"]
  if sign["SPEED_SIGN_UNIT"] == 1 and 10 <= speed_sign <= 90:
    return float(speed_sign) * CV.MPH_TO_MS
  if sign["SPEED_SIGN_UNIT"] == 2 and 15 <= speed_sign < 127:
    return float(speed_sign) * CV.KPH_TO_MS
  return 0.0


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.dashboard_limit = LimitTracker()

  def update_dashboard_limit(self, can_parsers: dict[StrEnum, CANParser]) -> None:
    """The SLA dashboard (car) source: the same camera-first, navigation-fallback limit as speedLimit, but a source counts
    only while its message is within the parser's learned timeout, so a reading the car stopped sending is not reused."""
    cam, pt = can_parsers[Bus.cam], can_parsers[Bus.pt]
    now = max(int(getattr(cam, '_last_update_nanos', 0)), int(getattr(pt, '_last_update_nanos', 0)))
    fresh = []
    for parser, message in ((cam, "CAM_TRAFFIC_SIGNS"), (pt, "NAV_SPEED_LIMIT")):
      timestamp, expiry = parser_expiry(parser, message, "SPEED_SIGN")
      if timestamp and expiry >= now:
        fresh.append((decode_speed_sign(parser.vl[message]), expiry))
    if now <= 0:
      return
    reading = next(((speed, expiry) for speed, expiry in fresh if speed > 0.0), None)
    if reading is not None:
      self.dashboard_limit.update(now, Status.VALID, reading[0], valid_until_ns=reading[1])
    elif fresh:
      self.dashboard_limit.update(now, Status.ABSENT, valid_until_ns=max(expiry for _, expiry in fresh))
    else:
      self.dashboard_limit.update(now, Status.UNKNOWN)

  def update(self, ret: structs.CarState, ret_sp: context.CarStateSP, can_parsers: dict[StrEnum, CANParser]) -> None:
    # Two sources share one layout. CAM_TRAFFIC_SIGNS is the camera's sign recognition, fused
    # with the map, and is sent blank by cars without it. NAV_SPEED_LIMIT is the navigation
    # unit's map limit, sent only with the nav SD card. Where both exist the camera wins: it
    # reads posted signs the map lacks or has stale.
    ret_sp.speedLimit = decode_speed_sign(can_parsers[Bus.cam].vl["CAM_TRAFFIC_SIGNS"])
    if ret_sp.speedLimit == 0.0:
      ret_sp.speedLimit = decode_speed_sign(can_parsers[Bus.pt].vl["NAV_SPEED_LIMIT"])
    self.update_dashboard_limit(can_parsers)
