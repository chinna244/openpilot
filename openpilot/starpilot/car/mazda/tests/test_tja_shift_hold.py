"""Route 00000002--c9d55e7093: a low-speed shift cleared the TJA latch (t=72.28 P->unknown 0.54 s->R,
t=471.96 D->unknown 0.08 s->N 0.3 s->unknown 0.06 s->R): the gear signal's transient 'unknown' plus selfdrived's
wrongGear soft disable is a disarming fault. develop/filter-dev MADS pauses there instead (wrongGear below 2.5 m/s).
The latch now survives a bounded benign shift; steering stays paused outside Drive."""
from types import SimpleNamespace

import pytest
from opendbc.car import structs

from openpilot.cereal import log
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.aol.intent import AolProcessFaultContext, disarming_fault
from openpilot.starpilot.car.mazda.aol import create_intent
from openpilot.starpilot.car.mazda.tests.test_filter_tja_transitions import SETTINGS, T0, filter_tja_cp, press

G = structs.CarState.GearShifter
ButtonType = structs.CarState.ButtonEvent.Type
DT = 10_000_000


def wrong_gear():
  return log.OnroadEvent.new_message(name='wrongGear', softDisable=True, noEntry=True).as_reader()


def immediate():
  return log.OnroadEvent.new_message(name='controlsMismatch', immediateDisable=True).as_reader()


class Rig:
  def __init__(self, gear=G.park):
    self.cp = filter_tja_cp()
    self.intent = create_intent(self.cp, SETTINGS)
    self.t = T0
    self.cs = structs.CarState(canValid=True, vEgo=0., standstill=True, gearShifter=gear)
    self.cs.cruiseState.available = True
    self.intent.update(self.cs.as_reader(), now_ns=self.t, fault_active=False)
    press(self.intent, self.cs, ButtonType.lkas, self.t + DT)  # driver arms TJA
    self.t += 3 * DT
    assert self.intent.allowed_latch
    self.events = []
    self.card = SimpleNamespace(CP=self.cp, aol_card_intent=self.intent, aol_process_fault_context=AolProcessFaultContext(),
                                aol_transport_feedback=SimpleNamespace(retention_qualified=False, retention_pending=False,
                                                                       temporary_host_qualified=False, qualified=False, awaiting=False),
                                sm={'onroadEvents': self.events}, slc_producer_session='', _aol_can_observation=None, CI=None)
    self.card.aol_cancel_source = lambda CS, now_ns: 0
    self.paused = []

  def frame(self, gear, *, v=0., events=None, can_valid=True, perm=False, buttons=()):
    self.cs.gearShifter, self.cs.vEgo, self.cs.standstill = gear, v, v == 0.
    self.cs.canValid, self.cs.steerFaultPermanent = can_valid, perm
    self.cs.buttonEvents = [structs.CarState.ButtonEvent(type=b, pressed=True) for b in buttons]
    self.events[:] = [wrong_gear()] if events is None and gear != G.drive else (events or [])
    reader = self.cs.as_reader()
    view = self.intent.observe_lateral_holds(reader)
    fault = Car.aol_disarming_fault(self.card, view, self.t, self.t, transport_observed_healthy=True)
    self.intent.update(view, now_ns=self.t, fault_active=fault)
    self.paused.append((gear, self.intent.pause_lateral))
    self.t += DT

  def run(self, gear, seconds, **kw):
    for _ in range(round(seconds * 100)):
      self.frame(gear, **kw)

  def steering_requested(self):
    return self.intent.allowed_latch and not self.intent.pause_lateral


def test_route_72s_park_unknown_reverse_keeps_the_latch():
  rig = Rig(G.park)
  rig.run(G.park, 0.7)
  rig.run(G.unknown, 0.54)
  rig.run(G.reverse, 3.0)
  rig.run(G.unknown, 0.3)
  assert rig.intent.allowed_latch and rig.intent.pause_lateral
  rig.run(G.drive, 0.5)
  assert rig.steering_requested()


def test_route_472s_drive_unknown_neutral_unknown_reverse_keeps_the_latch():
  rig = Rig(G.drive)
  rig.run(G.drive, 0.5)
  rig.run(G.unknown, 0.08)
  rig.run(G.neutral, 0.3)
  rig.run(G.unknown, 0.06)
  rig.run(G.reverse, 2.0)
  assert rig.intent.allowed_latch
  rig.run(G.drive, 0.5)
  assert rig.steering_requested()


def test_steering_is_never_requested_outside_drive():
  rig = Rig(G.park)
  for gear, seconds in ((G.park, .5), (G.unknown, .5), (G.reverse, 1.), (G.neutral, .5), (G.unknown, .2), (G.drive, .3)):
    rig.run(gear, seconds)
  assert all(paused for gear, paused in rig.paused if gear != G.drive)


def test_without_the_hold_the_unknown_frame_is_a_disarming_fault():
  cs = structs.CarState(canValid=True, gearShifter=G.unknown).as_reader()
  assert disarming_fault([wrong_gear()], cs)  # the cause: generic AOL rule, unchanged for other brands


def test_unknown_gear_that_lasts_disarms():
  rig = Rig(G.park)
  rig.run(G.unknown, 1.2)
  assert not rig.intent.allowed_latch and rig.intent.latch_clear_reason == 'disarming_fault'


def test_unknown_gear_that_keeps_returning_disarms():
  rig = Rig(G.park)
  for _ in range(5):
    rig.run(G.unknown, 0.1)
    rig.run(G.park, 0.5)
  assert not rig.intent.allowed_latch


def test_unknown_gear_while_moving_disarms():
  rig = Rig(G.drive)
  rig.run(G.drive, 0.2, v=5.)
  rig.run(G.unknown, 0.1, v=5.)
  assert not rig.intent.allowed_latch


def test_invalid_can_during_a_shift_disarms():
  rig = Rig(G.park)
  rig.run(G.unknown, 0.1, can_valid=False)
  assert not rig.intent.allowed_latch


def test_immediate_disable_during_a_shift_disarms():
  rig = Rig(G.park)
  rig.run(G.unknown, 0.1, events=[wrong_gear(), immediate()])
  assert not rig.intent.allowed_latch


def test_steering_fault_during_a_shift_is_no_shift_hold():
  rig = Rig(G.park)
  rig.frame(G.unknown, perm=True)
  assert not rig.intent.shift_hold and rig.intent.pause_lateral


def test_explicit_tja_off_during_a_shift_turns_it_off():
  rig = Rig(G.reverse)
  rig.run(G.reverse, 0.3)
  rig.frame(G.reverse, buttons=(ButtonType.lkas,))
  rig.cs.buttonEvents = [structs.CarState.ButtonEvent(type=ButtonType.lkas, pressed=False)]
  rig.intent.update(rig.intent.observe_lateral_holds(rig.cs.as_reader()), now_ns=rig.t, fault_active=False)
  assert not rig.intent.allowed_latch
  rig.run(G.drive, 0.5)
  assert not rig.steering_requested()


def test_route_362s_lagging_wrong_gear_back_in_drive_keeps_the_latch():
  """Route 00000007--7d9475933b t=362.19: R->N->D; selfdrived's wrongGear still stood on the first Drive frames."""
  rig = Rig(G.drive)
  rig.run(G.drive, 0.5)
  rig.run(G.unknown, 0.04)
  rig.run(G.neutral, 0.3)
  rig.run(G.reverse, 2.0)
  rig.run(G.neutral, 0.3)
  rig.run(G.drive, 0.05, events=[wrong_gear()])  # the shift's own event, a few cycles late
  assert rig.intent.allowed_latch
  rig.run(G.drive, 0.5)
  assert rig.steering_requested()


def test_settle_window_ends_after_one_second_in_drive():
  rig = Rig(G.drive)
  rig.run(G.drive, 0.5)
  rig.run(G.reverse, 1.0)
  rig.run(G.drive, 0.98)
  assert rig.intent.shift_settling
  rig.run(G.drive, 0.05)
  assert not rig.intent.shift_settling


def test_no_settle_window_without_a_shift_hold():
  rig = Rig(G.drive)
  rig.run(G.drive, 1.0)
  assert not rig.intent.shift_settling


def test_settle_window_is_not_a_steering_pause():
  rig = Rig(G.drive)
  rig.run(G.drive, 0.5)
  rig.run(G.reverse, 1.0)
  rig.run(G.drive, 0.05)
  assert rig.intent.shift_settling and rig.steering_requested()
