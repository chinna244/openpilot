"""The Mazda SLA port's hooks in card, plannerd, planner_host and the planner ceiling."""
from types import SimpleNamespace as NS

import pytest

from openpilot.starpilot.car.mazda.sla import eligible
from openpilot.starpilot.car.mazda.sla.session import MazdaSlaSession
from openpilot.starpilot.longitudinal.cruise_ceiling import CruiseCeiling, select_cruise_ceiling
from openpilot.starpilot.speed_limits import speed_domain as sd

MAZDA = NS(brand='mazda', openpilotLongitudinalControl=True, pcmCruise=True, passive=False, dashcamOnly=False)


@pytest.mark.parametrize('cp, ok', [
  (MAZDA, True),
  (NS(**{**vars(MAZDA), 'openpilotLongitudinalControl': False}), False),  # stock ACC: not ported
  (NS(**{**vars(MAZDA), 'brand': 'toyota'}), False),
  (NS(**{**vars(MAZDA), 'dashcamOnly': True}), False),
])
def test_eligibility(cp, ok):
  assert eligible(cp) is ok


def test_plannerd_offset_is_starpilots_slc_offset():
  from openpilot.selfdrive.controls import plannerd
  schedule = sd.OffsetSchedule((sd.OffsetBand(0.0, 11.2, 2.235), sd.OffsetBand(11.2, None, 2.682)))
  lifecycle = NS(runtime=NS(settings=NS(offsets=schedule)))
  planner = plannerd.mazda_sla_planner(MAZDA, NS(get_bool=lambda k, **kw: False), lifecycle)
  fn = planner.resolver.offset_fn
  assert fn(11.176) == pytest.approx(sd.adjusted_limit(11.176, schedule) - 11.176)
  assert fn(13.411) == pytest.approx(2.682)
  assert plannerd.mazda_sla_planner(MAZDA, NS(get_bool=lambda k, **kw: False), NS(runtime=None)).resolver.offset_fn(11.176) == 0.


def test_plannerd_ceiling_applies_like_slc():
  from openpilot.selfdrive.controls import plannerd
  ceiling = CruiseCeiling(30 / 2.2369363, plannerd.MAZDA_SLA_AUTHORITY)
  kw = {'driver_v_cruise_kph': 56., 'driver_cruise_mps': 56. / 3.6, 'system_long_available': True, 'long_control_active': True,
            'selfdrive_enabled': True, 'force_decel': False}
  assert select_cruise_ceiling(ceiling, **kw).effective_mps == pytest.approx(30 / 2.2369363)
  assert select_cruise_ceiling(ceiling, **{**kw, 'selfdrive_enabled': False}).effective_mps is None
  assert select_cruise_ceiling(ceiling, **{**kw, 'long_control_active': False}).effective_mps is None


def test_starpilot_slc_makes_no_offers_of_its_own_on_mazda(monkeypatch):
  from openpilot.starpilot.speed_limits import planner_host
  from openpilot.starpilot.speed_limits.runtime_settings import parse
  saved = parse({'SpeedLimitController': True, 'SLCConfirmation': 1, 'IsMetric': False})  # "Both": StarPilot asks either way
  assert saved.acceptance.confirm_lower and saved.acceptance.confirm_higher
  captured = {}
  monkeypatch.setattr(planner_host, 'slc_runtime_settings', lambda *a, **kw: saved)
  monkeypatch.setattr(planner_host, 'feature_enabled', lambda *a, **kw: False)
  monkeypatch.setattr(planner_host, 'vision_control_enabled', lambda *a, **kw: False)
  monkeypatch.setattr(planner_host, 'read_slc_settings', lambda p: None)
  monkeypatch.setattr(planner_host, 'read_saved', lambda *a, **kw: (None, None))

  class Params:
    def get_bool(self, k, **kw):
      return False

  for cp, confirms in ((MAZDA, False), (NS(**{**vars(MAZDA), 'brand': 'toyota'}), True)):
    life = planner_host.SavedSlcLifecycle(Params(), cp, {})
    life.refresh(0)
    captured[cp.brand] = life.configuration[1].acceptance
    assert captured[cp.brand].confirm_lower is confirms and captured[cp.brand].confirm_higher is confirms


class _Btn:
  def __init__(self, t, p):
    self.type, self.pressed = t, p


def test_card_hands_the_session_setpoint_to_carstate():
  from openpilot.selfdrive.car.card import Car

  class SM(dict):
    seen = {'slcState': True}
  sla = NS(speedLimit=0., speedLimitFinalLast=0., speedLimitValid=False, speedLimitLastValid=False, assistState=0,
           servoVTarget=36 / 2.2369363, limiterActive=False)
  sm = SM(slcState=NS(mazdaSla=sla), carControl=NS(enabled=False, cruiseControl=NS(override=False, cancel=False, resume=False)))
  sm.logMonoTime = {'slcState': 10**12 - 10_000_000}
  car = NS(sm=sm, mazda_sla=MazdaSlaSession(MAZDA), is_metric=False, slc_control_enabled=True, can_log_mono_time=10**12)
  cs = NS(available=True, enabled=True, speed=36 / 2.2369363, speedCluster=36 / 2.2369363, standstill=False)
  CS = NS(vEgo=10., gasPressed=False, standstill=False, steeringAngleDeg=0., cruiseState=cs, buttonEvents=[], vCruise=0., vCruiseCluster=0.)
  Car.mazda_sla_update(car, CS)
  assert CS.vCruise == pytest.approx(36 * 1.609344, abs=0.1) and CS.vCruiseCluster == pytest.approx(CS.vCruise)
  assert car.mazda_sla.send_button == 'none'


@pytest.mark.parametrize('case', ['unfilled', 'stale'])
def test_card_ignores_an_unfilled_or_stale_mazda_sla(case):
  """The upstream planner publishes slcState without mazdaSla (servoVTarget 0), and a stalled plannerd leaves an old one:
  neither may drive the SET walk (the target would be 0 mph and the servo would press - to the ECU minimum)."""
  from openpilot.selfdrive.car import card as card_mod
  from openpilot.selfdrive.car.card import Car

  class SM(dict):
    pass
  zero = NS(speedLimit=0., speedLimitFinalLast=0., speedLimitValid=False, speedLimitLastValid=False, assistState=0,
            servoVTarget=0. if case == 'unfilled' else 20 / 2.2369363, limiterActive=case == 'stale')
  sm = SM(slcState=NS(mazdaSla=zero), carControl=NS(enabled=True, cruiseControl=NS(override=False, cancel=False, resume=False)))
  now = 10**12
  sm.seen = {'slcState': True}
  sm.logMonoTime = {'slcState': now - (10_000_000 if case == 'unfilled' else 2_000_000_000)}
  car = NS(sm=sm, mazda_sla=MazdaSlaSession(MAZDA), is_metric=False, slc_control_enabled=True, can_log_mono_time=now)
  cs = NS(available=True, enabled=True, speed=40 / 2.2369363, speedCluster=40 / 2.2369363, standstill=False)
  sends = []
  with pytest.MonkeyPatch.context() as mp:
    mp.setattr(card_mod, 'REPLAY', True)
    for _ in range(400):
      CS = NS(vEgo=40 / 2.2369363, gasPressed=False, standstill=False, steeringAngleDeg=0., cruiseState=cs, buttonEvents=[],
              vCruise=0., vCruiseCluster=0.)
      Car.mazda_sla_update(car, CS)
      sends.append(car.mazda_sla.send_button)
  assert set(sends) == {'none'}
