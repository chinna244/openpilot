"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.cereal import messaging
from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.car.cruise_helpers import ButtonType, CruiseHelper


def cs_sp(farther: bool, stock_ecu: str = "notNeeded"):
  msg = messaging.new_message('carStateSP').carStateSP
  msg.zoompilot.distanceFarther = farther
  msg.zoompilot.stockEcu = stock_ecu
  return msg


class TestCarCruiseOwnsDistance:
  """While the car's own cruise drives (Mazda stockCruiseOn), its distance button is the car's:
  selfdrived leaves openpilot's personality and experimental toggle alone."""

  def test_only_the_car_cruise_counts(self):
    helper = CruiseHelper(structs.CarParams())
    assert helper.car_cruise_on(cs_sp(False, "stockCruiseOn"))
    for state in ("notNeeded", "parkToTakeOver", "starting", "ready", "restoring", "restored", "failed"):
      assert not helper.car_cruise_on(cs_sp(False, state))

  def test_a_held_press_is_dropped(self):
    helper = CruiseHelper(structs.CarParams())
    helper.button_frame_counts[ButtonType.gapAdjustCruise] = 30  # held, short of the long press
    helper.experimental_mode_switched = True
    helper.car_cruise_on(cs_sp(False, "stockCruiseOn"))
    assert helper.button_frame_counts[ButtonType.gapAdjustCruise] == 0 and not helper.experimental_mode_switched


class TestDistanceFarther:
  """The wheel's second distance button reaches selfdrived as a level on carStateSP; the helper
  turns it into one release edge, the opposite direction to upstream's gapAdjustCruise cycle."""

  def test_one_step_per_release(self):
    helper = CruiseHelper(structs.CarParams())
    assert not helper.distance_farther_released(cs_sp(False))
    assert not helper.distance_farther_released(cs_sp(True))
    assert not helper.distance_farther_released(cs_sp(True))   # held: nothing yet
    assert helper.distance_farther_released(cs_sp(False))      # the release steps once
    assert not helper.distance_farther_released(cs_sp(False))

  def test_cars_without_the_button_never_step(self):
    helper = CruiseHelper(structs.CarParams())
    assert not any(helper.distance_farther_released(cs_sp(False)) for _ in range(10))
