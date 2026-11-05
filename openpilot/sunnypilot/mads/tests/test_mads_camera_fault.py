"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake
from openpilot.sunnypilot.mads.state import State
from openpilot.sunnypilot.mads.tests.mads_harness import car_state, make_mads

EventNameSP = custom.OnroadEventSP.EventName
EventNameZP = custom.OnroadEventZP.EventName
ButtonType = structs.CarState.ButtonEvent.Type


class TestMadsCameraFault(OpenpilotTestCase):
  """The Mazda camera's fault holds lateral paused and resumes it when it clears (mads.py)."""

  def setup_method(self):
    self.mads, self.sd = make_mads(self._fixture("mocker"), "mazda", True)
    self.mads.steering_mode_on_brake = MadsSteeringModeOnBrake.REMAIN_ACTIVE

  def frame(self, cs=None, fault=False, selfdrive_enabled=False):
    sd = self.sd
    sd.events.clear()
    sd.events_sp.clear()
    sd.enabled = selfdrive_enabled
    if fault:
      sd.events_sp.zp.add(EventNameZP.cameraLkasFault)
    self.mads.update(cs if cs is not None else car_state(True))
    return self.mads.state_machine.state

  def engage(self):
    self.mads.state_machine.state = State.enabled
    self.mads.enabled, self.mads.active = True, True

  def has(self, event):
    return self.sd.events_sp.zp.has(event)

  def test_fault_pauses_and_keeps_mads_enabled(self):
    for selfdrive_enabled in (False, True):
      self.engage()
      self.mads.camera_fault_prev = False
      assert self.frame(fault=True, selfdrive_enabled=selfdrive_enabled) == State.paused
      assert self.mads.enabled, "the panda drops lateral on the MADS heartbeat if this goes false"
      assert not self.mads.active
      assert self.has(EventNameZP.cameraLkasFaultOnset)

  def test_onset_is_raised_once(self):
    self.engage()
    self.frame(fault=True)
    for _ in range(50):
      assert self.frame(fault=True) == State.paused
      assert not self.has(EventNameZP.cameraLkasFaultOnset)
      assert not self.sd.events_sp.has(EventNameSP.silentLkasEnable)

  def test_no_onset_when_not_steering(self):
    assert self.frame(fault=True) == State.disabled
    assert not self.has(EventNameZP.cameraLkasFaultOnset)

  def test_clear_resumes_with_the_chime_once(self):
    self.engage()
    self.frame(fault=True, selfdrive_enabled=True)
    self.frame(fault=True, selfdrive_enabled=True)
    assert self.frame(selfdrive_enabled=True) == State.enabled
    assert self.mads.active
    assert self.has(EventNameZP.cameraLkasResumed)
    self.frame(selfdrive_enabled=True)
    assert not self.has(EventNameZP.cameraLkasResumed)

  def test_flicker_pauses_and_resumes_each_time(self):
    self.engage()
    for _ in range(3):
      assert self.frame(fault=True) == State.paused
      assert self.has(EventNameZP.cameraLkasFaultOnset)
      assert self.frame() == State.enabled
      assert self.has(EventNameZP.cameraLkasResumed)

  def test_mads_button_while_faulted_disables_for_good(self):
    self.engage()
    self.frame(fault=True)
    assert self.frame(car_state(True, button=ButtonType.lkas), fault=True) == State.disabled
    assert self.frame() == State.disabled
    assert not self.has(EventNameZP.cameraLkasResumed)

  def test_enable_after_a_disable_in_the_fault_is_not_a_resume(self):
    self.engage()
    self.frame(fault=True)
    self.frame(car_state(True, button=ButtonType.lkas), fault=True)
    self.frame()
    assert self.frame(car_state(True, button=ButtonType.lkas)) == State.enabled
    assert not self.has(EventNameZP.cameraLkasResumed)

  def test_main_off_while_faulted_disables(self):
    self.engage()
    self.frame(fault=True)
    assert self.frame(car_state(False), fault=True) == State.disabled

  def test_request_while_faulted_waits_in_paused(self):
    assert self.frame(car_state(True, button=ButtonType.lkas), fault=True) == State.paused
    assert self.frame(fault=True) == State.paused
    assert self.frame() == State.enabled
    assert self.has(EventNameZP.cameraLkasResumed)

  def test_lateral_held_while_faulted(self):
    self.engage()
    self.frame(fault=True)
    assert self.mads.lateral_held
    self.frame()
    assert not self.mads.lateral_held

  def test_onset_is_loud_in_a_brake_pause(self):
    self.mads.state_machine.state = State.paused
    self.mads.enabled = True
    self.frame(fault=True)
    assert self.has(EventNameZP.cameraLkasFaultOnset)

  def test_a_fault_already_there_at_engage_is_not_an_onset(self):
    self.frame(fault=True)
    assert self.frame(car_state(True, button=ButtonType.lkas), fault=True) == State.paused
    assert not self.has(EventNameZP.cameraLkasFaultOnset)

  def test_brake_pause_is_not_announced_as_a_camera_resume(self):
    self.mads.steering_mode_on_brake = MadsSteeringModeOnBrake.PAUSE
    self.mads.state_machine.state = State.paused
    self.mads.enabled = True
    braking = car_state(True)
    braking.brakePressed = True
    assert self.frame(braking) == State.paused
    assert self.frame() == State.enabled
    assert not self.has(EventNameZP.cameraLkasResumed)
