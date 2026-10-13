"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import custom, log
from opendbc.car.structs import car
from opendbc.car import structs
from openpilot.common.params import Params

ButtonType = car.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
StockEcuState = custom.CarStateZP.StockEcuState

DISTANCE_LONG_PRESS = 50


class CruiseHelper:
  # selfdrived's SubMaster, as its base; a helper built on its own has none
  sm = None

  def __init__(self, CP: structs.CarParams):
    self.CP = CP
    self.params = Params()

    self.button_frame_counts = {ButtonType.gapAdjustCruise: 0}
    self._experimental_mode = False
    self.experimental_mode_switched = False
    self.distance_farther_prev = False
    self.car_cruise = False

  def update(self, CS, events, experimental_mode) -> None:
    # the car's own cruise driving: its distance button is the car's, not openpilot's personality
    self.car_cruise = self.sm is not None and self.car_cruise_on(self.sm['carStateSP'])
    if self.CP.openpilotLongitudinalControl and not self.car_cruise:
      if CS.cruiseState.available:
        self.update_button_frame_counts(CS)

        # toggle experimental mode once on distance button hold
        self.update_experimental_mode(events, experimental_mode)

    if self.sm is not None:
      self.update_selfdrived(CS, self.sm['carStateSP'])

  def update_selfdrived(self, CS, CS_SP) -> None:
    """zoompilot, run by selfdrived just before its personality step. The wheel's "farther" distance
    button steps the personality up, unless upstream's gapAdjustCruise release steps it down this
    frame, and ICBM follows card's activation."""
    gap_released = any(not be.pressed and be.type == ButtonType.gapAdjustCruise for be in CS.buttonEvents)
    farther_released = self.distance_farther_released(CS_SP)
    if self.CP.openpilotLongitudinalControl and not self.car_cruise and not gap_released and farther_released:
      self.personality = (self.personality + 1) % 3
      self.params.put('LongitudinalPersonality', self.personality)
      self.events.add(EventName.personalityChanged)

    self.icbm.update_activation(CS_SP)

  def car_cruise_on(self, CS_SP) -> bool:
    """The car's own cruise engaged on its own radar (Mazda stockCruiseOn): the distance button
    sets that cruise's gap, so openpilot's personality and the long-press experimental toggle
    leave it alone and show nothing. A press held into it is dropped, not counted on after."""
    on = CS_SP.zoompilot.stockEcu == StockEcuState.stockCruiseOn
    if on:
      self.button_frame_counts[ButtonType.gapAdjustCruise] = 0
      self.experimental_mode_switched = False
    return on

  def distance_farther_released(self, CS_SP) -> bool:
    """The release edge of the wheel's "farther" distance button (carStateSP.zoompilot), the
    opposite direction to upstream's gapAdjustCruise cycle. False on cars without one."""
    pressed = bool(CS_SP.zoompilot.distanceFarther)
    released = self.distance_farther_prev and not pressed
    self.distance_farther_prev = pressed
    return released

  def update_button_frame_counts(self, CS) -> None:
    for button in self.button_frame_counts:
      if self.button_frame_counts[button] > 0:
        self.button_frame_counts[button] += 1

    for button_event in CS.buttonEvents:
      button = button_event.type.raw
      if button in self.button_frame_counts:
        self.button_frame_counts[button] = int(button_event.pressed)

  def update_experimental_mode(self, events, experimental_mode) -> None:
    if self.button_frame_counts[ButtonType.gapAdjustCruise] >= DISTANCE_LONG_PRESS and not self.experimental_mode_switched:
      self._experimental_mode = not experimental_mode
      self.params.put_bool("ExperimentalMode", self._experimental_mode)
      events.add(EventNameSP.experimentalModeSwitched)
      self.experimental_mode_switched = True
