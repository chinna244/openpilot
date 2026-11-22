import time
from types import SimpleNamespace

from opendbc.car import structs
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.cruise_arbiter import CruiseArbiter

Button = structs.CarState.ButtonEvent.Type
State = custom.LongitudinalPlanSP.SpeedLimit.AssistState


class TestRampPrompt:
  def setup_method(self):
    self.arb = CruiseArbiter(SimpleNamespace(pcmCruise=True, openpilotLongitudinalControl=True), SimpleNamespace(pcmCruiseSpeed=False))
    self.arb.enabled = True
    self.arb.state = State.inactive
    self.setpoint = 81 * CV.MPH_TO_KPH

  def frame(self, offer=50, events=(), enabled=True):
    lp = custom.LongitudinalPlanSP()
    lp.speedLimit.rampSpeed = offer * CV.MPH_TO_MS
    lp.speedLimit.rampId = 1 if offer else 0
    lp.speedLimit.rampExpiresMonoTime = time.monotonic_ns() + 5_000_000_000 if offer else 0
    self.arb.update_limit(lp)
    cs = structs.CarState()
    cs.buttonEvents = [structs.CarState.ButtonEvent(type=b, pressed=p) for b, p in events]
    cs.vEgo = 59 * CV.MPH_TO_MS
    self.setpoint = self.arb.step(cs, enabled, self.setpoint, self.setpoint)

  def test_mph_50_requires_press_and_is_retained_after_advisory_clears(self):
    self.frame()
    assert self.arb.prompting
    assert round(self.setpoint * CV.KPH_TO_MPH) == 81
    self.frame(events=[(Button.decelCruise, True)])
    assert round(self.setpoint * CV.KPH_TO_MPH) == 81
    self.frame(events=[(Button.decelCruise, False)])
    assert self.arb.press_owned(Button.decelCruise)
    assert round(self.setpoint * CV.KPH_TO_MPH) == 50
    self.frame(offer=0)
    assert not self.arb.prompting
    assert round(self.setpoint * CV.KPH_TO_MPH) == 50

  def test_lost_offer_between_press_and_release_does_not_adopt(self):
    self.frame()
    self.frame(events=[(Button.decelCruise, True)])
    self.frame(offer=0, events=[(Button.decelCruise, False)])
    assert not self.arb.prompting
    assert round(self.setpoint * CV.KPH_TO_MPH) == 81

  def test_offer_does_not_raise_set_speed(self):
    self.setpoint = 45 * CV.MPH_TO_KPH
    self.frame()
    assert not self.arb.prompting
    assert round(self.setpoint * CV.KPH_TO_MPH) == 45

  def test_disengagement_clears_prompt(self):
    self.frame()
    self.frame(enabled=False)
    assert not self.arb.prompting
    assert round(self.setpoint * CV.KPH_TO_MPH) == 81

  def test_plus_declines_without_adopting_or_stepping(self):
    self.frame()
    self.frame(events=[(Button.accelCruise, True)])
    assert not self.arb.prompting
    assert self.arb.press_owned(Button.accelCruise)
    self.frame(events=[(Button.accelCruise, False)])
    assert self.arb.press_owned(Button.accelCruise)
    self.frame()
    assert not self.arb.prompting
    assert round(self.setpoint * CV.KPH_TO_MPH) == 81
