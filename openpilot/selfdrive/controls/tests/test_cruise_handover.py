"""After a gas override the cruise candidate starts no lower than its target asks, only for the driver's gas while engaged.
Route 00000177 t=183: reset to a -0.55 measured accel, the cruise candidate sent -0.41 while the model asked +0.24."""
import numpy as np
import pytest

from openpilot.cereal import custom, messaging
from opendbc.car import structs
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, A_CRUISE_MIN

V_EGO = 6.0  # m/s
FAST = V_EGO * 3.6 + 30.  # kph: a permissive cruise target


class MockSubMaster(dict):
  def __init__(self, services):
    super().__init__(services)
    self.valid, self.alive, self.updated = (dict.fromkeys(services, True) for _ in range(3))
    self.logMonoTime, self.recv_frame = dict.fromkeys(services, 0), dict.fromkeys(services, 1)

  def all_checks(self, service_list=None):
    return True


def build_sm(v_cruise_kph=FAST, a_ego=-0.6, gas=False, brake=False, enabled=True, override=False, model_a=0.2, lead=None, e2e=True):
  services = {s: getattr(messaging.new_message(s), s) for s in ("vehicleParameters", "carStateSP", "liveMapDataSP",
                                                                "gpsLocationExternal", "gpsLocation")}
  cs = messaging.new_message('carState')
  cs.carState.vEgo, cs.carState.aEgo, cs.carState.vCruise, cs.carState.vCruiseCluster = V_EGO, a_ego, v_cruise_kph, v_cruise_kph
  cs.carState.gasPressed, cs.carState.brakePressed = gas, brake
  services['carState'] = cs.carState.as_reader()
  ctl = messaging.new_message('controlsState')
  ctl.controlsState.longControlState = LongCtrlState.off if override or not enabled else LongCtrlState.pid
  services['controlsState'] = ctl.controlsState.as_reader()
  sd = messaging.new_message('selfdriveState')
  sd.selfdriveState.experimentalMode, sd.selfdriveState.enabled = e2e, enabled
  services['selfdriveState'] = sd.selfdriveState.as_reader()
  cc = messaging.new_message('carControl')
  cc.carControl.enabled = enabled
  cc.carControl.orientationNED = [0., 0., 0.]  # flat: the coast accel is -0.3
  services['carControl'] = cc.carControl.as_reader()
  rs = messaging.new_message('radarState')
  if lead is not None:
    rs.radarState.leadOne.present, rs.radarState.leadOne.dRel, rs.radarState.leadOne.vLead = True, lead, 0.
    rs.radarState.leadOne.vRel, rs.radarState.leadOne.modelProb = -V_EGO, 1.
  services['radarState'] = rs.radarState.as_reader()
  md = messaging.new_message('modelV2')
  md.modelV2.orientationRate.z = [0.01] * 33  # nonzero: a straight path divides by zero in SCC vision
  md.modelV2.velocity.x = [V_EGO] * 33
  md.modelV2.position.x = [V_EGO * i * 0.3 for i in range(33)]
  md.modelV2.action.desiredAcceleration = model_a
  md.modelV2.meta.disengagePredictions.gasPressProbs = [0., 0.]  # the model expects no throttle: ACC caps at the coast
  services['modelV2'] = md.modelV2.as_reader()
  return MockSubMaster(services)


class ReferencePlanner(LongitudinalPlanner):
  """The planner with the handover correction never armed: the committed behaviour."""
  gas_override = property(lambda self: False, lambda self, v: None)
  gas_return = property(lambda self: False, lambda self, v: None)


def planner(cls=LongitudinalPlanner):
  CP = structs.CarParams()
  CP.steerRatio, CP.wheelbase, CP.longitudinalActuatorDelay, CP.openpilotLongitudinalControl = 15.0, 2.7, 0.2, True
  return cls(CP, custom.CarParamsSP.new_message().as_reader(), init_v=V_EGO)


def reference():
  return planner(ReferencePlanner)


def press(p, n=10, **kwargs):
  for _ in range(n):
    p.update(build_sm(override=True, **kwargs))


def sent(p):
  return p.a_target_published  # the plan the car gets


def test_gas_release_starts_no_lower_than_a_permissive_target():
  p = planner()
  press(p, gas=True, a_ego=-0.6)
  p.update(build_sm())
  assert sent(p) >= 0.  # no braking pulse: the cruise candidate climbs from 0 toward the model's +0.2


def test_the_plan_published_while_the_gas_is_held_is_not_braking():
  # the first control cycle after the release still uses this plan (route 00000177 t=183.37: -0.48 consumed)
  p = planner()
  press(p, gas=True, a_ego=-0.6)
  assert sent(p) >= 0.


def test_the_gas_is_latched_when_long_control_trails_the_pedal():
  # the pedal is up a cycle before longControlState leaves off (route 00000177: 2 of 16 releases)
  p = planner()
  press(p, gas=True, a_ego=-0.6)
  press(p, n=2, gas=False, a_ego=-0.6)
  assert sent(p) >= 0.
  p.update(build_sm())
  assert sent(p) >= 0.


@pytest.mark.parametrize("lead", [40., 60., 80.])
def test_the_mpc_restarts_from_the_plain_plan(lead):
  # only the published plan is lifted while the gas is held: the MPC seeds from what it would have
  p, ref = planner(), reference()
  for q in (p, ref):
    press(q, gas=True, a_ego=-0.6, lead=lead)
  np.testing.assert_allclose(p.output_a_target, ref.output_a_target)
  np.testing.assert_allclose(p.v_desired_filter.x, ref.v_desired_filter.x)
  for q in (p, ref):
    q.update(build_sm(lead=lead))
  np.testing.assert_allclose(p.mpc.a_solution, ref.mpc.a_solution)


def test_after_the_return_the_cruise_candidate_is_the_committed_one():
  # once the state is back at its target's ask the correction is over: braking for a target and climbing back after it
  # are exactly as committed
  p, ref = planner(), reference()
  for q in (p, ref):
    press(q, gas=True, a_ego=-0.6)
    q.update(build_sm(a_ego=-0.6))
  for i in range(60):
    kph = (V_EGO - 1.5) * 3.6 if i < 30 else FAST
    p.update(build_sm(v_cruise_kph=kph, a_ego=-0.6))
    ref.update(build_sm(v_cruise_kph=kph, a_ego=-0.6))
    if i >= 1:
      assert sent(p) == pytest.approx(sent(ref), abs=1e-6)


@pytest.mark.parametrize("a_ego", [-0.3, -1.0])
def test_a_target_that_turns_restrictive_at_release_still_brakes(a_ego):
  # permissive while the gas is held, 10 km/h under the speed at the release: the target asks the most braking it can
  p, ref = planner(), reference()
  for q in (p, ref):
    press(q, gas=True, a_ego=a_ego)
    q.update(build_sm(v_cruise_kph=V_EGO * 3.6 - 10., a_ego=a_ego))
  assert sent(p) <= sent(ref) + 1e-6
  assert sent(p) <= max(a_ego, A_CRUISE_MIN) + 0.1


def test_a_target_mildly_under_speed_brakes_as_it_asks():
  # 0.5 m/s over the target, measured -1.0: the cruise candidate starts at the target's -0.5, not at -1.0
  p = planner()
  press(p, gas=True, a_ego=-1.0, v_cruise_kph=(V_EGO - 0.5) * 3.6)
  p.update(build_sm(v_cruise_kph=(V_EGO - 0.5) * 3.6, a_ego=-1.0))
  assert -0.6 <= sent(p) <= -0.4


@pytest.mark.parametrize("offset", [1, 2, 3, 5, 8])
def test_a_target_that_turns_restrictive_after_release_brakes_at_least_as_it_asks(offset):
  # permissive at the release, restrictive `offset` cycles later (review: 0.0 sent where the committed -0.53 was)
  p, ref = planner(), reference()
  for q in (p, ref):
    press(q, gas=True, a_ego=-0.6)
    for _ in range(offset):
      q.update(build_sm(a_ego=-0.6))
  slow = (V_EGO - 1.0) * 3.6  # 1 m/s over the target: it asks -1.0
  for _ in range(20):
    p.update(build_sm(v_cruise_kph=slow, a_ego=-0.6))
    ref.update(build_sm(v_cruise_kph=slow, a_ego=-0.6))
    assert sent(p) <= max(sent(ref), -1.0) + 1e-6


def test_the_committed_braking_is_kept_one_cycle_after_release():
  p = planner()
  press(p, gas=True, a_ego=-0.6)
  p.update(build_sm(a_ego=-0.6))
  p.update(build_sm(v_cruise_kph=(V_EGO - 1.0) * 3.6, a_ego=-0.6))
  assert sent(p) <= -0.5


@pytest.mark.parametrize("a_ego", [-0.1, -0.8])
def test_acc_mode_brakes_at_least_to_the_coast_target(a_ego):
  # without throttle ACC caps the cruise accel at the coast (-0.3 here), even under the set speed
  p, ref = planner(), reference()
  for q in (p, ref):
    press(q, gas=True, a_ego=a_ego, e2e=False)
    q.update(build_sm(a_ego=a_ego, e2e=False))
  assert sent(p) <= max(sent(ref), -0.3) + 1e-6


def test_repeated_short_gas_presses_each_hand_over_clean():
  p = planner()
  for _ in range(4):
    press(p, n=2, gas=True, a_ego=-0.6)
    p.update(build_sm(a_ego=-0.6))
    assert sent(p) >= 0.
    p.update(build_sm(a_ego=-0.6))


@pytest.mark.parametrize("kwargs", [
  {'gas': True, 'brake': True},      # gas and brake together
  {'brake': True},                   # brake release alone
  {'gas': True, 'enabled': False},   # gas while disengaged
])
def test_other_resets_keep_the_measured_seed(kwargs):
  p = planner()
  press(p, a_ego=-0.6, **kwargs)
  assert sent(p) == pytest.approx(p.output_a_target)
  p.update(build_sm(a_ego=-0.6))
  assert sent(p) < -0.4  # unchanged: the cruise candidate starts from the measured -0.6


def test_a_model_stop_still_wins():
  p = planner()
  press(p, gas=True, a_ego=-0.6)
  p.update(build_sm(model_a=-1.5))
  assert sent(p) <= -1.4


def test_a_close_lead_still_brakes():
  p = planner()
  press(p, gas=True, a_ego=-0.6, lead=8.)
  p.update(build_sm(lead=8.))
  assert sent(p) < -0.5


def test_a_release_while_accelerating_keeps_the_climb():
  # the correction only lifts: a positive measured accel climbs on from where it was, one jerk step per cycle
  p = planner()
  press(p, gas=True, a_ego=0.7)
  p.update(build_sm(a_ego=0.7, model_a=1.5))
  assert p.a_cruise > 0.7 + 0.1


class CapturePubMaster:
  def __init__(self):
    self.sent = {}

  def send(self, name, msg):
    self.sent[name] = msg.as_reader() if hasattr(msg, 'as_reader') else msg


@pytest.mark.parametrize("model_a, source", [(0.2, 'cruise'), (-0.2, 'e2e')])
def test_both_published_plans_carry_the_command_and_its_winner(model_a, source):
  # while the gas is held the internal winner is the measured -0.53 cruise state; the car gets the lifted plan, and both
  # messages and the source must say so (review: LP 0.0 / SP -0.53, and 'cruise' where the model won)
  p = planner()
  press(p, gas=True, a_ego=-0.6, model_a=model_a)
  sm = build_sm(override=True, gas=True, a_ego=-0.6, model_a=model_a)
  p.update(sm)
  pm = CapturePubMaster()
  p.publish(sm, pm)
  lp, lpsp = pm.sent['longitudinalPlan'].longitudinalPlan, pm.sent['longitudinalPlanSP'].longitudinalPlanSP
  assert p.output_a_target < -0.4  # the internal plan is untouched
  assert lp.aTarget == pytest.approx(lpsp.aTarget)
  assert lp.aTarget == pytest.approx(min(model_a, 0.), abs=0.01)  # the lifted cruise candidate is 0 here
  assert str(lp.longitudinalPlanSource) == source


def test_outside_the_correction_both_plans_match_the_internal_one():
  p = planner()
  for _ in range(5):
    p.update(build_sm(a_ego=-0.6))
  pm = CapturePubMaster()
  p.publish(build_sm(a_ego=-0.6), pm)
  lp, lpsp = pm.sent['longitudinalPlan'].longitudinalPlan, pm.sent['longitudinalPlanSP'].longitudinalPlanSP
  assert lp.aTarget == pytest.approx(p.output_a_target) == pytest.approx(lpsp.aTarget)
  assert lp.longitudinalPlanSource == p.mpc.source
