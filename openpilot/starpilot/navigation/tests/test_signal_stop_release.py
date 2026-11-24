from types import SimpleNamespace as NS

import pytest

from opendbc.car import structs
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.starpilot.navigation.intent import TurnIntent

NOW = 20_000_000_000


@pytest.fixture
def case(tmp_path):
  class SM(dict):
    valid = dict.fromkeys(('carState', 'carControl', 'deviceState', 'longitudinalPlan'), True)
    alive = dict(valid)
    logMonoTime = dict.fromkeys(valid, NOW)
  params = Params(str(tmp_path))
  cp = structs.CarParams.new_message(carFingerprint='real', brand='hyundai', steerControlType='torque')
  cp.safetyConfigs = [structs.CarParams.SafetyConfig.new_message(safetyModel='hyundaiCanfd')]
  sm = SM(carState=NS(canValid=True, canTimeout=False, gearShifter='drive', vEgo=2., standstill=False,
                     leftBlinker=True, rightBlinker=False, leftBlindspot=False, rightBlindspot=False),
          carControl=NS(latActive=True), deviceState=NS(started=True, startedMonoTime=1),
          longitudinalPlan=NS(shouldStop=False, modelMonoTime=NOW))
  from openpilot.starpilot.lateral.turn_initiation_preferences import KEY
  params.put_bool(KEY, True, block=True)
  owner = TurnIntent(params, cp)
  owner.guidance.sample(NOW - 1)
  return owner, sm


def sample(owner, sm, model_stop=False, supported=True, desire=log.Desire.none):
  return owner.select(sm, NOW, desire, supported=supported, model_stop=model_stop, plan_end_speed=5.)


@pytest.mark.parametrize('right', [False, True])
@pytest.mark.parametrize('source', ['model', 'planner'])
@pytest.mark.parametrize('lagged_release', [False, True])
def test_completed_stop_releases_same_signal_after_stop_request_clears(case, right, source, lagged_release):
  owner, sm = case
  cs, plan = sm['carState'], sm['longitudinalPlan']
  cs.leftBlinker, cs.rightBlinker = not right, right
  expected = log.Desire.turnRight if right else log.Desire.turnLeft
  assert sample(owner, sm) == expected
  def stop(value):
    plan.shouldStop = value if source == 'planner' else False
    return sample(owner, sm, model_stop=value if source == 'model' else False)
  for _ in range(3):
    assert stop(True) == log.Desire.none
    assert stop(False) == log.Desire.none  # An approach cannot release before standstill.
    cs.standstill, cs.vEgo = True, 0.
    assert stop(True) == log.Desire.none
    assert stop(True) == log.Desire.none
    cs.standstill, cs.vEgo = False, 1.
    if lagged_release:
      assert stop(True) == log.Desire.none
    assert stop(False) == expected
    assert stop(False) == expected


@pytest.mark.parametrize('service', ['carState', 'carControl', 'longitudinalPlan', 'deviceState'])
@pytest.mark.parametrize('invalid', ['stale', 'future', 'invalid', 'dead'])
def test_invalid_delivery_cannot_acknowledge_a_completed_stop(case, service, invalid):
  owner, sm = case
  assert sample(owner, sm, model_stop=True) == log.Desire.none
  cs = sm['carState']
  cs.standstill, cs.vEgo = True, 0.
  stamp = sm.logMonoTime[service]
  if invalid == 'stale':
    sm.logMonoTime[service] = 1
  elif invalid == 'future':
    sm.logMonoTime[service] = NOW + 1
  elif invalid == 'invalid':
    sm.valid[service] = False
  else:
    sm.alive[service] = False
  assert sample(owner, sm) == log.Desire.none
  sm.logMonoTime[service] = stamp
  sm.valid[service] = sm.alive[service] = True
  cs.standstill, cs.vEgo = False, 1.
  assert sample(owner, sm) == log.Desire.none
  cs.standstill, cs.vEgo = True, 0.
  assert sample(owner, sm) == log.Desire.none
  cs.standstill, cs.vEgo = False, 1.
  assert sample(owner, sm) == log.Desire.turnLeft


@pytest.mark.parametrize('field,value', [('canValid', False), ('canTimeout', True), ('gearShifter', 'reverse'),
                                        ('leftBlindspot', True), ('rightBlinker', True), ('vEgo', 10.),
                                        ('vEgo', float('nan')), ('vEgo', 0.)])
def test_release_keeps_vehicle_guards(case, field, value):
  owner, sm = case
  sm['carState'].standstill = True
  assert sample(owner, sm) == log.Desire.none
  sm['carState'].standstill = False
  setattr(sm['carState'], field, value)
  assert sample(owner, sm) == log.Desire.none


def test_signal_cancel_releases_approach_latch_and_existing_desire_is_preserved(case):
  owner, sm = case
  assert sample(owner, sm, model_stop=True) == log.Desire.none
  sm['carState'].leftBlinker = False
  assert sample(owner, sm) == log.Desire.none
  sm['carState'].leftBlinker = True
  assert sample(owner, sm) == log.Desire.turnLeft
  assert sample(owner, sm, supported=False) == log.Desire.none
  assert sample(owner, sm, desire=log.Desire.laneChangeRight) == log.Desire.laneChangeRight


@pytest.mark.parametrize('guard', ['lateral', 'device', 'unsupported', 'plan-source', 'off', 'passive'])
def test_completed_stop_never_bypasses_remaining_admission(case, guard):
  from openpilot.starpilot.lateral.turn_initiation_preferences import KEY
  owner, sm = case
  sm['carState'].standstill = True
  assert sample(owner, sm) == log.Desire.none
  sm['carState'].standstill = False
  if guard == 'lateral':
    sm['carControl'].latActive = False
  elif guard == 'device':
    sm['deviceState'].started = False
  elif guard == 'plan-source':
    sm['longitudinalPlan'].modelMonoTime = NOW + 1
  elif guard == 'off':
    owner.guidance.params.put_bool(KEY, False, block=True)
    owner.guidance.last_read_ns = None
  elif guard == 'passive':
    owner.steering_capable = False
  assert sample(owner, sm, supported=guard != 'unsupported') == log.Desire.none


# Route 08 t=375.35: blinker on at 0.39 m/s into a red light while the model's plan ended at 0.13 m/s; the turn desire
# made the model drive on. A turn desire starts only when the car and the model's own plan already drive on.
@pytest.mark.parametrize('v_ego,plan_end_speed', [(0.39, 0.13), (0.72, 0.13), (5., 0.13), (0.49, 5.), (1., None),
                                                  (1., float('nan')), (float('nan'), 5.), (1., 1.55)])
def test_turn_desire_needs_car_and_plan_driving_on(case, v_ego, plan_end_speed):
  owner, sm = case
  sm['carState'].vEgo = v_ego
  assert owner.select(sm, NOW, log.Desire.none, supported=True, model_stop=False,
                      plan_end_speed=plan_end_speed) == log.Desire.none


@pytest.mark.parametrize('v_ego,plan_end_speed', [(0.5, 1.57), (1., 5.), (8., 10.)])
def test_turn_desire_admitted_when_driving_on(case, v_ego, plan_end_speed):
  owner, sm = case
  sm['carState'].vEgo = v_ego
  assert owner.select(sm, NOW, log.Desire.none, supported=True, model_stop=False,
                      plan_end_speed=plan_end_speed) == log.Desire.turnLeft


def test_gate_keeps_existing_desire(case):
  owner, sm = case
  sm['carState'].vEgo = 0.3
  assert owner.select(sm, NOW, log.Desire.laneChangeLeft, supported=True, model_stop=False,
                      plan_end_speed=0.) == log.Desire.laneChangeLeft
