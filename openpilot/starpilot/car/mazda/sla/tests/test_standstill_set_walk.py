"""Route 08: the Mazda SLA SET walk tapped SET+ to the factory ECU while the car stood at a held stop; Mazda's ECU can read
SET+ at a standstill as a resume. End to end through card (Car.mazda_sla_update -> session -> control_context) and the real
alpha-long Mazda controller: no synthetic SET frame on CRZ_BTNS while the car is stopped or crawling, queued presses are
discarded, and the walk restarts from a fresh decision only after the car has driven off."""
from types import SimpleNamespace as NS

import pytest

from opendbc.car.mazda.tests.conftest import CRZ_BTNS, car_control, car_controller, frames, mazda_car_state, parse_frame, set_car_state
from openpilot.selfdrive.car import card as card_mod
from openpilot.selfdrive.car.card import Car
from openpilot.starpilot.car.mazda.sla.session import DEPARTURE_FRAMES, MazdaSlaSession

MPH = 2.2369363
MAZDA = NS(brand='mazda', openpilotLongitudinalControl=True, pcmCruise=True, passive=False, dashcamOnly=False)
T0 = 10**12
DT = 10_000_000


class Btn:
  def __init__(self, t, p):
    self.type, self.pressed = t, p


class Drive:
  """card + the real Mazda controller at 100 Hz. The planner asks the SET walk for 36 mph (30 mph + 20%) over a dash of 33."""

  def __init__(self, target_mph=36., dash_mph=33.):
    self.cc = car_controller(alpha_long=True)
    self.cs = mazda_car_state(self.cc.CP, self.cc.CP_SP)
    self.session = MazdaSlaSession(MAZDA)
    self.target, self.dash = target_mph / MPH, dash_mph / MPH
    self.now = T0
    self.set_frames = []  # (frame index, 'SET_P'/'SET_M')
    self.sends = []
    self.i = 0

  def step(self, *, v, standstill=None, engaged=True, buttons=(), slc_age_ns=10_000_000, offer=False):
    standstill = v < 0.01 if standstill is None else standstill

    class SM(dict):
      pass
    sla = NS(speedLimit=30 / MPH, speedLimitFinalLast=30 / MPH, speedLimitValid=True, speedLimitLastValid=True,
             assistState=2 if offer else 0, servoVTarget=self.target, limiterActive=False, roadChangeMonoTime=0)
    sm = SM(slcState=NS(mazdaSla=sla), carControl=NS(enabled=engaged, cruiseControl=NS(override=False, cancel=False, resume=False)))
    sm.seen = {'slcState': True}
    sm.logMonoTime = {'slcState': self.now - slc_age_ns}
    car = NS(sm=sm, mazda_sla=self.session, is_metric=False, slc_control_enabled=True, can_log_mono_time=self.now)
    cruise = NS(available=True, enabled=engaged, speed=self.dash, speedCluster=self.dash, standstill=standstill)
    CS = NS(vEgo=v, gasPressed=False, standstill=standstill, steeringAngleDeg=0., cruiseState=cruise,
            buttonEvents=[Btn(b, True) for b in buttons], vCruise=0., vCruiseCluster=0.)
    with pytest.MonkeyPatch.context() as mp:
      mp.setattr(card_mod, 'REPLAY', True)
      Car.mazda_sla_update(car, CS)
    # card's apply: the session's button goes to the controller through control_context
    set_car_state(self.cs, available=True, cruise_engaged=engaged, v_ego=v, standstill=standstill)
    ctx = self.cc.control_context
    ctx.intelligentCruiseButtonManagement.sendButton = self.session.send_button
    ctx.zoompilot.icbmHold = self.session.hold
    sends = self.cc.update(car_control(long_active=engaged, accel=0.), self.cs, self.now)[1]
    ctx.intelligentCruiseButtonManagement.sendButton = 'none'
    ctx.zoompilot.icbmHold = False
    for dat in frames(sends, CRZ_BTNS, bus=0):
      sig = parse_frame(CRZ_BTNS, dat)
      for name in ('SET_P', 'SET_M'):
        if sig[name]:
          self.set_frames.append((self.i, name))
    self.now += DT
    self.i += 1

  def run(self, n, **kw):
    start = self.i
    for _ in range(n):
      self.step(**kw)
    return [f for f in self.set_frames if f[0] >= start]


def walking(d):
  """Drive until the walk is pressing SET+ (from a cold start the engage gate and the servo take a few seconds)."""
  for _ in range(30):
    if d.run(100, v=13.):
      return d
  raise AssertionError('the SET walk never started while driving')


def test_moving_walk_still_presses():
  walking(Drive())  # the walk itself is unchanged while driving


def test_red_light_hold_sends_no_set_press():
  d = walking(Drive())  # walking toward 36 mph
  d.run(150, v=3.)  # braking for the light
  assert d.run(1500, v=0.) == []  # 15 s held at the red light
  assert d.session.send_button == 'none' and not d.session.hold


def test_crawl_after_stop_sends_none_until_driven_off():
  d = Drive()
  d.run(500, v=0.)
  assert d.run(300, v=0.6, standstill=False) == []  # creeping under 1 m/s


@pytest.mark.parametrize('case', ['stop_sign_res', 'green_auto_go'])
def test_walk_restarts_only_after_a_valid_departure(case):
  d = Drive()
  d.run(300, v=0.)
  d.run(1, v=0., buttons=('resumeCruise',) if case == 'stop_sign_res' else ())  # the driver's own RES (not synthetic)
  assert d.run(DEPARTURE_FRAMES, v=1.5) == []  # driving off: nothing for the first second
  later = d.run(400, v=4.)
  assert later and later[0][1] == 'SET_P'  # then a fresh walk toward 36 mph


def test_disengaged_and_stale_planner_send_nothing():
  d = Drive()
  assert d.run(300, v=0., engaged=False) == []
  assert d.run(300, v=13., engaged=False) == []
  assert d.run(300, v=13., slc_age_ns=2_000_000_000) == []  # stale plannerd data
  assert d.run(300, v=0., slc_age_ns=2_000_000_000) == []


def test_queued_press_from_before_the_stop_is_discarded():
  d = walking(Drive())
  while d.session._next_button == 'none':  # stop on a frame where a press is queued for the next apply
    d.step(v=13.)
  d.step(v=0.)  # the car is stopped the next frame: whatever was queued must not reach CRZ_BTNS
  assert d.run(200, v=0.) == []
  assert d.session._next_button == 'none' and d.session.send_button == 'none'


def test_physical_set_plus_at_a_stop_is_not_echoed():
  d = Drive()
  d.run(300, v=0., offer=True)
  assert d.run(30, v=0., buttons=('accelCruise',), offer=True) == []  # the driver's SET+ answers the offer
  assert d.run(500, v=0.) == []


def test_controller_never_emits_a_synthetic_set_at_standstill():
  # last line: even if card handed it a button, the controller sends no SET frame while stopped
  cc = car_controller(alpha_long=True)
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  sends = []
  for i in range(200):
    set_car_state(cs, available=True, cruise_engaged=True, v_ego=0., standstill=True)
    cc.control_context.intelligentCruiseButtonManagement.sendButton = 'increase'
    sends.extend(cc.update(car_control(long_active=True, accel=0.), cs, T0 + i * DT)[1])
  assert frames(sends, CRZ_BTNS, bus=0) == []
