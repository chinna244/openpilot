"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import openpilot.cereal.messaging as messaging
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import SmartCruiseControlMap


class SmartCruiseControl:
  def __init__(self):
    self.vision = SmartCruiseControlVision()
    self.map = SmartCruiseControlMap()

  def update(self, sm: messaging.SubMaster, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, v_cruise: float) -> None:
    # liveMapDataSP is valid while the localizer is and alive while mapd publishes: without both, the position
    # and targets mapd holds are stale (a frozen target would hold a speed cap the car never reaches)
    position_valid = bool(sm.valid['liveMapDataSP'] and sm.alive['liveMapDataSP'])
    self.map.update(long_enabled, long_override, v_ego, a_ego, v_cruise, position_valid)
    self.vision.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)
