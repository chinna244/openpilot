"""Closed loop: servo -> simulated body ECU (one step per registered press, 200 ms floor) -> dash SET.

Cases follow filter-dev's test_icbm_sla_session.py where they apply to StarPilot's SLC
(route 00000002--c9d55e7093 t=185.649 SET- rejected 30 at 35; t=390.807 SET+ accepted 25 at 34
and the ECU stepped 34->35).
"""
from types import SimpleNamespace

import pytest

from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.selfdrive.car.cruise import SlcConsumedButton, SlcPendingConfirmation
from openpilot.starpilot.car.mazda import sla_set_servo as m

ButtonType = car.CarState.ButtonEvent.Type
DT_NS = 10_000_000
MPH = CV.MPH_TO_MS


class Ecu:
  """Mazda body ECU: each registered press moves SET one mph; presses closer than 200 ms are dropped."""
  def __init__(self, set_mph):
    self.set_mph = set_mph
    self.last_ns = -10**12

  def press(self, direction, now_ns):
    if now_ns - self.last_ns < 200_000_000:
      return
    self.last_ns = now_ns
    self.set_mph = max(m.MIN_SET_MPH, self.set_mph + (1 if direction == 'increase' else -1))


def cs(set_mph, *, buttons=(), standstill=False, gas=False, cruise=True, can_valid=True):
  events = [SimpleNamespace(type=t, pressed=True) for t in buttons]
  return SimpleNamespace(buttonEvents=events, canValid=can_valid, canTimeout=False, gasPressed=gas, brakePressed=False,
                         standstill=standstill, cruiseState=SimpleNamespace(enabled=cruise, standstill=False,
                                                                            speedCluster=set_mph * MPH))


class Loop:
  def __init__(self, set_mph):
    self.ecu = Ecu(set_mph)
    self.servo = m.MazdaSlaSetServo(is_metric=False)
    self.now = 1_000_000_000
    self.accepted = None
    self.accepted_ns = 0
    self.prompting = False
    self.sent = []

  def confirm(self, pressed, offer_mph):
    """Driver press during an offer: card maps direction, arms on accept; the ECU steps from the press."""
    answer = m.directional_answer(pressed, offer_mph * MPH, self.ecu.set_mph * MPH, False)
    if answer == 'accel':
      self.servo.arm(offer_mph * MPH, self.now)
    button = ButtonType.accelCruise if pressed == 'accel' else ButtonType.decelCruise
    self.step(buttons=(button,), confirm_frame=answer == 'accel')
    self.ecu.press('increase' if pressed == 'accel' else 'decrease', self.now)  # the factory's own one-step
    self.prompting = False
    if answer == 'accel':
      self.accepted, self.accepted_ns = offer_mph * MPH, self.now + 20_000_000
    return answer

  def step(self, *, buttons=(), confirm_frame=False, enabled=True, long_active=True, **kw):
    out = self.servo.update(cs(self.ecu.set_mph, buttons=buttons, **kw), enabled=enabled, long_active=long_active, cancel=False, resume=False,
                            accepted_mps=self.accepted if self.now >= self.accepted_ns else None,
                            accepted_ns=self.accepted_ns, prompting=self.prompting, now_ns=self.now,
                            confirm_frame=confirm_frame)
    if out != m.MazdaSlaSetServo.NONE:
      self.sent.append(out)
      self.ecu.press(out, self.now)
    self.now += DT_NS
    return out

  def run(self, seconds, **kw):
    for _ in range(int(seconds * 100)):
      self.step(**kw)


def test_route_minus_confirms_lower_and_walks_35_to_30():
  loop = Loop(35)
  assert loop.confirm('decel', 30) == 'accel'  # SET- toward a lower offer accepts (was: reject)
  assert loop.ecu.set_mph == 34  # the factory's own step from the confirming press
  loop.run(3)
  assert loop.ecu.set_mph == 30 and not loop.servo.active and loop.servo.end_reason == 'settled'
  assert set(loop.sent) == {'decrease'}


def test_plus_confirms_higher_and_walks_30_to_36():
  loop = Loop(30)
  assert loop.confirm('accel', 36) == 'accel'
  assert loop.ecu.set_mph == 31
  loop.run(3)
  assert loop.ecu.set_mph == 36 and set(loop.sent) == {'increase'}


def test_route_plus_against_a_lower_offer_rejects_and_never_walks():
  loop = Loop(34)
  assert loop.confirm('accel', 25) == 'decel'  # was: accepted 25 while the ECU went 34->35
  loop.run(3)
  assert loop.ecu.set_mph == 35 and loop.sent == [] and not loop.servo.active


def test_offer_equal_to_set_keeps_starpilot_meaning():
  assert m.directional_answer('accel', 30 * MPH, 30 * MPH, False) == 'accel'
  assert m.directional_answer('decel', 30 * MPH, 30 * MPH, False) == 'decel'


def test_factory_step_never_replaces_the_target():
  loop = Loop(35)
  loop.confirm('decel', 30)
  loop.run(0.25)  # inside the react window the dash sits at the press's 34
  assert loop.servo.target == 30


def test_target_gone_for_one_frame_ends_the_walk_for_good():
  loop = Loop(35)
  loop.confirm('decel', 30)
  loop.run(0.5)
  accepted, sent = loop.accepted, len(loop.sent)
  loop.accepted = None
  assert loop.step() == m.MazdaSlaSetServo.NONE and loop.servo.end_reason == 'target_lost'
  loop.accepted = accepted  # the value comes back: no new confirm, no resumed walk
  loop.run(3)
  assert len(loop.sent) == sent and not loop.servo.active


@pytest.mark.parametrize('new', [36, 25])
def test_changed_accepted_value_ends_the_walk(new):
  loop = Loop(35)
  loop.confirm('decel', 30)
  loop.run(0.5)
  sent, at = len(loop.sent), loop.ecu.set_mph
  loop.accepted = new * MPH
  loop.run(3)
  assert len(loop.sent) == sent and loop.ecu.set_mph == at and loop.servo.end_reason == 'target_changed'


def test_settled_driver_press_reanchors_without_drag_back():
  loop = Loop(35)
  loop.confirm('decel', 30)
  loop.run(3)
  loop.step(buttons=(ButtonType.accelCruise,))
  loop.ecu.press('increase', loop.now)
  sent = len(loop.sent)
  loop.run(3)
  assert loop.ecu.set_mph == 31 and len(loop.sent) == sent and loop.servo.end_reason == 'settled'


def test_mid_walk_driver_press_ends_session_without_climbing_back():
  loop = Loop(45)
  loop.confirm('decel', 30)
  loop.run(1.0)
  mid = loop.ecu.set_mph
  assert 30 < mid < 44
  loop.step(buttons=(ButtonType.decelCruise,))
  loop.run(3)
  assert loop.ecu.set_mph == mid and not loop.servo.active


@pytest.mark.parametrize('kw', [dict(standstill=True), dict(gas=True), dict(can_valid=False), dict(long_active=False)])
def test_interruption_ends_the_walk_and_recovery_cannot_restart_it(kw):
  loop = Loop(40)
  loop.confirm('decel', 30)
  loop.run(0.6)  # walk under way
  at, sent = loop.ecu.set_mph, len(loop.sent)
  assert 30 < at < 39 and sent > 0
  loop.run(0.5, **kw)
  assert not loop.servo.active and loop.servo.end_reason == 'interrupted'
  loop.run(3)  # control, CAN and the accepted target all back
  assert loop.ecu.set_mph == at and len(loop.sent) == sent


def test_interruption_before_the_accept_shows_disarms():
  loop = Loop(35)
  loop.servo.arm(30 * MPH, loop.now)
  loop.step(gas=True)
  loop.accepted, loop.accepted_ns = 30 * MPH, loop.now
  loop.run(3)
  assert loop.sent == [] and not loop.servo.active


def test_new_offer_ends_the_walk():
  loop = Loop(35)
  loop.confirm('decel', 30)
  loop.run(0.6)
  sent = len(loop.sent)
  loop.prompting = True
  loop.step()
  loop.prompting = False
  loop.run(3)
  assert len(loop.sent) == sent and loop.servo.end_reason == 'new_offer'


def test_resume_cancel_and_disengage_end_the_session():
  for end in ('resume', 'cancel', 'disengage'):
    loop = Loop(35)
    loop.confirm('decel', 30)
    loop.run(0.5)
    if end == 'disengage':
      loop.step(enabled=False)
    else:
      loop.servo.update(cs(34), enabled=True, long_active=True, cancel=end == 'cancel', resume=end == 'resume', accepted_mps=loop.accepted,
                        accepted_ns=loop.accepted_ns, prompting=False, now_ns=loop.now)
    assert not loop.servo.active
    ended_at = loop.ecu.set_mph
    loop.run(2)
    assert loop.ecu.set_mph == ended_at > 30


def test_accept_that_never_shows_up_disarms():
  loop = Loop(35)
  loop.servo.arm(30 * MPH, loop.now)
  loop.run(2.5)
  assert not loop.servo.active and loop.servo.armed is None and loop.sent == []


def test_dash_that_does_not_move_stalls_out():
  loop = Loop(35)
  loop.confirm('decel', 30)
  loop.ecu.press = lambda direction, now_ns: None  # ECU ignores synthesized presses
  loop.run(4)
  assert not loop.servo.active and loop.servo.end_reason == 'stalled'


def test_never_walks_below_the_ecu_minimum():
  loop = Loop(25)
  loop.confirm('decel', 15)
  loop.run(4)
  assert loop.ecu.set_mph == m.MIN_SET_MPH


def test_metric_units():
  servo = m.MazdaSlaSetServo(is_metric=True)
  assert m.confirm_direction(50 / 3.6, 60 / 3.6, True) == 'decel'
  servo.arm(50 / 3.6, 0)
  assert servo.armed.target == 50


def test_card_answer_binds_to_the_shown_offer():
  from openpilot.selfdrive.car.card import Car
  pending = SlcPendingConfirmation('s', 3, 38)
  state = SimpleNamespace(presentationId=38, hasPresentationAdjustedLimit=True, presentationAdjustedLimit=30 * MPH)
  fake = SimpleNamespace(sm=SimpleNamespace(seen={'slcState': True}, __getitem__=None), is_metric=False,
                         mazda_sla_servo=m.MazdaSlaSetServo(), mazda_sla_confirm_frame=False)
  fake.sm = type('SM', (), {'seen': {'slcState': True}, '__getitem__': lambda self, k: state})()
  out = Car.mazda_sla_answer(fake, SlcConsumedButton(pending, 'decel'), cs(35), 5)
  assert out.button == 'accel' and out.pending == pending and fake.mazda_sla_servo.armed.target == 30
  assert fake.mazda_sla_confirm_frame
  state.presentationId = 39  # a different offer than the press consumed: StarPilot's own meaning
  fake.mazda_sla_servo.reset()
  out = Car.mazda_sla_answer(fake, SlcConsumedButton(pending, 'decel'), cs(35), 6)
  assert out.button == 'decel' and fake.mazda_sla_servo.armed is None


def test_settled_walk_disarms_and_a_newer_acceptance_never_presses():
  """Review: settled at 36, factory SET later reads 35, a newer 36.3 mph acceptance rounds to 36 -> no SET+."""
  loop = Loop(30)
  loop.confirm('accel', 36)
  loop.run(3)
  assert loop.ecu.set_mph == 36 and not loop.servo.active and loop.servo.end_reason == 'settled'
  sent = len(loop.sent)
  loop.ecu.set_mph = 35
  loop.accepted, loop.accepted_ns = 36.3 * MPH, loop.now + 10_000_000
  loop.run(3)
  assert len(loop.sent) == sent and loop.ecu.set_mph == 35


def test_newer_acceptance_rounding_to_the_same_mph_ends_a_walk_in_progress():
  loop = Loop(30)
  loop.confirm('accel', 36)
  loop.run(0.6)
  at, sent = loop.ecu.set_mph, len(loop.sent)
  assert 31 < at < 36
  loop.accepted, loop.accepted_ns = 36.3 * MPH, loop.now  # a newer planner acknowledgment, same rounded mph
  loop.run(3)
  assert loop.ecu.set_mph == at and len(loop.sent) == sent and loop.servo.end_reason == 'target_changed'
