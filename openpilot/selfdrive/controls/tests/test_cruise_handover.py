"""After a gas override or an engagement the cruise candidate starts no lower than its target asks (not after a brake).
Route 00000177 t=183: reset to a -0.55 measured accel, the cruise candidate sent -0.41 while the model asked +0.24."""
import numpy as np
import pytest

from openpilot.cereal import custom, messaging
from opendbc.car import structs
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, A_CRUISE_MIN
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import SCC_SOURCES, LongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import OP_LONG_A_BUDGET

V_EGO = 6.0  # m/s
FAST = V_EGO * 3.6 + 30.  # kph: a permissive cruise target


class MockSubMaster(dict):
  def __init__(self, services):
    super().__init__(services)
    self.valid, self.alive, self.updated = (dict.fromkeys(services, True) for _ in range(3))
    self.logMonoTime, self.recv_frame = dict.fromkeys(services, 0), dict.fromkeys(services, 1)

  def all_checks(self, service_list=None):
    return True


def build_sm(v_cruise_kph=FAST, a_ego=-0.6, gas=False, brake=False, enabled=True, override=False, model_a=0.2, lead=None, e2e=True,
             force_decel=False):
  services = {s: getattr(messaging.new_message(s), s) for s in ("vehicleParameters", "carStateSP", "liveMapDataSP",
                                                                "gpsLocationExternal", "gpsLocation", "customReserved10")}
  cs = messaging.new_message('carState')
  cs.carState.vEgo, cs.carState.aEgo, cs.carState.vCruise, cs.carState.vCruiseCluster = V_EGO, a_ego, v_cruise_kph, v_cruise_kph
  cs.carState.gasPressed, cs.carState.brakePressed = gas, brake
  services['carState'] = cs.carState.as_reader()
  ctl = messaging.new_message('controlsState')
  ctl.controlsState.longControlState = LongCtrlState.off if override or not enabled else LongCtrlState.pid
  ctl.controlsState.forceDecel = force_decel
  services['controlsState'] = ctl.controlsState.as_reader()
  sd = messaging.new_message('selfdriveState')
  sd.selfdriveState.enabled = enabled
  services['selfdriveState'] = sd.selfdriveState.as_reader()
  sd_sp = messaging.new_message('selfdriveStateSP')
  sd_sp.selfdriveStateSP.zoompilot.experimentalMode = e2e
  services['selfdriveStateSP'] = sd_sp.selfdriveStateSP.as_reader()
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
  # permissive while the gas is held, 10 km/h under the speed at the release: the target asks the most braking it can,
  # A_CRUISE_MIN (upstream's gentle set-speed drop). The reference can brake harder only from the measured accel the
  # reset left in its state, the artefact this correction removes.
  p, ref = planner(), reference()
  for q in (p, ref):
    press(q, gas=True, a_ego=a_ego)
    q.update(build_sm(v_cruise_kph=V_EGO * 3.6 - 10., a_ego=a_ego))
  assert sent(p) <= max(sent(ref), A_CRUISE_MIN) + 1e-6
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
def test_acc_mode_hands_over_clean(a_ego):
  # upstream's cruise accel has no coast cap: in ACC under the set speed the target asks no braking
  p = planner()
  press(p, gas=True, a_ego=a_ego, e2e=False)
  p.update(build_sm(a_ego=a_ego, e2e=False))
  assert sent(p) >= 0.


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


def test_the_ask_takes_scc_budget_without_its_jerk_limit():
  # SCC brakes through the cruise candidate on its own budget: the handover's ask is that budget, not a jerk step
  p = planner()
  p.source, p.output_v_target, p.v_ego, p.a_cruise_prev = SCC_SOURCES[0], V_EGO - 3., V_EGO, 0.
  assert p.update_cruise_accel(0., unlimited=True) == pytest.approx(-OP_LONG_A_BUDGET)
  assert p.update_cruise_accel(0.) > -0.1


def engage(p, **kwargs):
  press(p, enabled=False, **kwargs)


@pytest.mark.parametrize("a_ego", [-0.3, -0.6])
def test_engaging_while_slowing_under_a_high_set_speed_does_not_brake(a_ego):
  # routes 00000169-00000180: engaged slowing, set speed well above, -0.27 to -0.48 for 0.2-0.3 s
  p = planner()
  engage(p, a_ego=a_ego)
  for _ in range(5):
    p.update(build_sm(a_ego=a_ego))
    assert sent(p) >= 0.


def test_an_engagement_long_control_trails_still_hands_over():
  p = planner()
  engage(p, a_ego=-0.6)
  press(p, n=2, a_ego=-0.6)  # engaged, longControlState still off
  p.update(build_sm(a_ego=-0.6))
  assert sent(p) >= 0.


def test_a_brake_after_engaging_keeps_the_measured_seed():
  p = planner()
  engage(p, a_ego=-0.6)
  press(p, n=3, brake=True, a_ego=-0.6)  # engaged, then a brake override
  p.update(build_sm(a_ego=-0.6))
  assert sent(p) < -0.4


def test_engaging_while_disengaged_on_the_gas_hands_over_clean():
  p = planner()
  engage(p, gas=True, a_ego=-0.6)
  p.update(build_sm(a_ego=-0.6))
  assert sent(p) >= 0.


@pytest.mark.parametrize("over", [0.2, 0.4, 0.9])
def test_engaging_just_over_the_set_speed_lifts_off(over):
  # route 00000184 t=1125: SET at 32.7 mph set 32, gaining speed, the model at +0.53: -0.43 sent
  p = planner()
  kph = (V_EGO - over) * 3.6
  engage(p, a_ego=0.6, v_cruise_kph=kph)
  for _ in range(int(5.0 / p.dt) - 1):
    p.update(build_sm(a_ego=0.6, v_cruise_kph=kph, model_a=0.5))
    assert -0.15 - 1e-6 <= sent(p) <= 0.6 + 1e-6
  assert sent(p) == pytest.approx(-0.15, abs=1e-6)


def test_the_settle_ends_and_the_set_speed_is_held():
  # still over after the window: the set speed brakes as committed, from the lift-off, at the jerk limit
  p, ref = planner(), reference()
  kph = (V_EGO - 0.5) * 3.6
  for q in (p, ref):
    engage(q, a_ego=0.6, v_cruise_kph=kph)
  for _ in range(int(5.0 / p.dt) - 1):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.5))
  assert sent(p) == pytest.approx(-0.15, abs=1e-6)
  for _ in range(40):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.5))
  assert sent(p) <= -0.45


@pytest.mark.parametrize("over", [1.2, 2.0])
def test_engaging_well_over_the_set_speed_brakes_as_committed(over):
  # RES to a stored speed well under: the gentle set-speed drop as before
  p = planner()
  kph = (V_EGO - over) * 3.6
  engage(p, a_ego=0.0, v_cruise_kph=kph)
  for _ in range(20):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.5))
  assert sent(p) <= -0.45


def test_the_settle_ends_once_back_at_the_set_speed():
  # under the set speed once, the window is over: a later climb over it brakes as committed
  p = planner()
  kph = (V_EGO - 0.5) * 3.6
  engage(p, a_ego=0.6, v_cruise_kph=kph)
  for _ in range(5):
    p.update(build_sm(a_ego=0.6, v_cruise_kph=kph, model_a=0.5))
  for _ in range(5):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=FAST, model_a=0.5))
  for _ in range(30):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.5))
  assert sent(p) <= -0.45


def test_set_with_the_gas_held_settles_after_the_release():
  # the usual rolling SET (route 00000184 t=1125): engaged with the foot still on the gas, released 0.9 mph over
  p = planner()
  kph = (V_EGO - 0.5) * 3.6
  engage(p, a_ego=0.6, v_cruise_kph=kph)
  press(p, n=3, gas=True, a_ego=0.6, v_cruise_kph=kph)  # engaged, gas held
  for _ in range(40):
    p.update(build_sm(a_ego=0.6, v_cruise_kph=kph, model_a=0.5))
    assert sent(p) >= -0.15 - 1e-6


def test_a_lowered_set_speed_ends_the_settle():
  p = planner()
  kph = (V_EGO - 0.3) * 3.6
  engage(p, a_ego=0.0, v_cruise_kph=kph)
  for _ in range(5):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.5))
  lower = (V_EGO - 0.8) * 3.6  # still inside the band
  for _ in range(20):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=lower, model_a=0.5))
  assert sent(p) <= -0.45


class NoSettlePlanner(LongitudinalPlanner):
  engage_settle_frames = property(lambda self: 0, lambda self, v: None)


@pytest.mark.parametrize("a_ego", [0.6, -0.6])
@pytest.mark.parametrize("over", [0.5, 0.9])
def test_the_settle_leaves_the_cruise_state_alone(a_ego, over):
  # only the candidate is lifted: the cruise state (e2e lead gap, accel boost and SCC's jerk all read it) is as without the
  # settle, through the window and after it
  p, ref = planner(), planner(NoSettlePlanner)
  kph = (V_EGO - over) * 3.6
  for q in (p, ref):
    engage(q, a_ego=a_ego, v_cruise_kph=kph)
  for _ in range(int(8.0 / p.dt)):
    for q in (p, ref):
      q.update(build_sm(a_ego=a_ego, v_cruise_kph=kph, model_a=0.5))
    assert p.a_cruise == pytest.approx(ref.a_cruise, abs=1e-9)


def test_after_the_window_the_candidate_eases_back_at_the_jerk_limit():
  p = planner()
  kph = (V_EGO - 0.5) * 3.6
  engage(p, a_ego=0.0, v_cruise_kph=kph)
  for _ in range(int(5.0 / p.dt) - 1):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.5))
  last = sent(p)
  assert last == pytest.approx(-0.15, abs=1e-6)
  for _ in range(20):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.5))
    assert last - 1.6 * p.dt - 1e-6 <= sent(p) <= last + 1e-6
    last = sent(p)
  assert sent(p) == pytest.approx(p.a_cruise, abs=1e-6)


def test_a_curve_or_speed_limit_target_after_engaging_brakes_as_it_asks(monkeypatch):
  # the settle is for the driver's set speed only: an SCC or SLA target is not lifted
  orig = LongitudinalPlannerSP.update_targets

  def as_scc(self, *args):
    out = orig(self, *args)
    self.source = SCC_SOURCES[1]
    return out
  monkeypatch.setattr(LongitudinalPlannerSP, 'update_targets', as_scc)
  p = planner()
  kph = (V_EGO - 0.5) * 3.6
  engage(p, a_ego=0.0, v_cruise_kph=kph)
  for _ in range(20):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.5))
  assert sent(p) <= -0.45


@pytest.mark.parametrize("closer", [0.9, 0.5])
def test_coming_into_the_band_after_engaging_well_over_is_no_step(closer):
  # review: engaged 1.3 m/s over, the set speed raised to 0.9 over: the candidate stepped -0.5 -> -0.15 in one frame
  p, ref = planner(), planner(NoSettlePlanner)
  for q in (p, ref):
    engage(q, a_ego=0.0, v_cruise_kph=(V_EGO - 1.3) * 3.6)
  for kph, n in (((V_EGO - 1.3) * 3.6, 20), ((V_EGO - closer) * 3.6, 40)):
    for _ in range(n):
      for q in (p, ref):
        q.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.5))
      assert sent(p) == pytest.approx(sent(ref), abs=1e-6)


def test_a_raised_set_speed_mid_settle_rises_without_a_dip():
  # route 00000184 t=1129.5: the set speed raised while the lift held -0.15; easing down to the state dipped to -0.26
  p = planner()
  kph = (V_EGO - 0.4) * 3.6
  engage(p, a_ego=0.0, v_cruise_kph=kph)
  for _ in range(30):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.5))
  last = sent(p)
  assert last == pytest.approx(-0.15, abs=1e-6)
  for _ in range(30):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=FAST, model_a=0.5))
    assert last - 1e-6 <= sent(p) <= last + 1.6 * p.dt + 1e-6  # up only, at the jerk limit
    assert sent(p) <= max(0., p.a_cruise) + 1e-6  # never lifted past what the gas handover would
    last = sent(p)
  assert sent(p) >= 0.


def test_a_later_set_speed_drop_brakes_as_committed():
  # once the candidate is back at the state nothing is left over: braking later in the drive is as without the settle
  p, ref = planner(), planner(NoSettlePlanner)
  over, under, lowered = (V_EGO - 0.5) * 3.6, FAST, (V_EGO - 0.9) * 3.6
  for q in (p, ref):
    engage(q, a_ego=0.0, v_cruise_kph=over)
  for kph, n in ((over, 20), (under, 200), (lowered, 60)):
    for _ in range(n):
      for q in (p, ref):
        q.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.5))
      if kph == lowered:
        assert sent(p) == pytest.approx(sent(ref), abs=1e-6)
  assert sent(p) <= -0.45


def test_a_curve_target_mid_window_brakes_from_the_cruise_state_at_once(monkeypatch):
  orig = LongitudinalPlannerSP.update_targets
  scc = {'on': False}

  def maybe_scc(self, *args):
    out = orig(self, *args)
    if scc['on']:
      self.source = SCC_SOURCES[0]
    return out
  monkeypatch.setattr(LongitudinalPlannerSP, 'update_targets', maybe_scc)
  p = planner()
  kph = (V_EGO - 0.5) * 3.6
  engage(p, a_ego=0.0, v_cruise_kph=kph)
  for _ in range(20):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.5))
  assert sent(p) == pytest.approx(-0.15, abs=1e-6)
  scc['on'] = True
  p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.5))
  assert sent(p) == pytest.approx(p.a_cruise, abs=1e-6)
  assert sent(p) <= -0.45


def test_a_model_stop_and_a_close_lead_still_win_after_engaging():
  p = planner()
  engage(p, a_ego=-0.6)
  p.update(build_sm(model_a=-1.5))
  assert sent(p) <= -1.4
  q = planner()
  engage(q, a_ego=-0.6, lead=8.)
  q.update(build_sm(lead=8.))
  assert sent(q) < -0.5



def lift_over(p, over, **kwargs):
  """Gas held over the set speed by `over` m/s, then released; the set speed in kph for the frames that follow."""
  kph = (V_EGO - over) * 3.6
  press(p, gas=True, a_ego=0.1, v_cruise_kph=kph, **kwargs)
  return kph


@pytest.mark.parametrize("over", [1.5, 3.0, 6.0])
def test_lifting_off_over_the_set_speed_coasts_down(over):
  # route 0000018b t=2689: 84 mph, set 77, the model at +0.1: -0.50 from 0.8 s on
  p, ref = planner(), reference()
  for q in (p, ref):
    kph = lift_over(q, over)
  seen, prev = [], 0.
  for _ in range(int(3.0 / p.dt)):
    for q in (p, ref):
      q.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.1))
    seen.append(sent(p))
    assert sent(p) >= -0.3 - 1e-6
    assert prev - sent(p) <= 0.3 * p.dt + 1e-6  # in at 0.3 m/s^3
    prev = sent(p)
  assert sent(p) == pytest.approx(-0.3, abs=1e-6)
  assert sent(ref) == pytest.approx(A_CRUISE_MIN, abs=1e-6)  # what it did before


def test_a_release_just_over_the_set_speed_is_unchanged():
  p, ref = planner(), reference()
  for q in (p, ref):
    kph = lift_over(q, 0.4)
  for _ in range(20):
    for q in (p, ref):
      q.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.1))
    assert sent(p) == pytest.approx(sent(ref), abs=1e-6)


def test_a_set_speed_lowered_at_the_release_brakes_as_before():
  p, ref = planner(), reference()
  for q in (p, ref):
    lift_over(q, 0.)
  for _ in range(30):
    for q in (p, ref):
      q.update(build_sm(a_ego=0.0, v_cruise_kph=(V_EGO - 3.0) * 3.6, model_a=0.1))
  assert not p.gas_coast and sent(p) == pytest.approx(A_CRUISE_MIN, abs=1e-6)


def test_the_coast_leaves_the_cruise_state_alone():
  p = planner()
  kph = lift_over(p, 3.0)
  for _ in range(40):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.1))
  assert p.a_cruise == pytest.approx(A_CRUISE_MIN, abs=1e-6)
  assert sent(p) > p.a_cruise


def test_back_at_the_set_speed_the_coast_eases_out_without_a_step():
  p = planner()
  kph = lift_over(p, 3.0)
  for _ in range(40):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.1))
  at = V_EGO * 3.6  # the set speed reached
  prev = sent(p)
  for _ in range(40):
    p.update(build_sm(a_ego=-0.3, v_cruise_kph=at, model_a=0.1))
    assert abs(sent(p) - prev) <= np.interp(V_EGO, [0., 10.], [1.6, 1.2]) * p.dt + 1e-6  # the cruise jerk limit
    prev = sent(p)
  assert not p.gas_coast and sent(p) == pytest.approx(0., abs=1e-6)


def test_a_lowered_set_speed_ends_the_coast():
  p = planner()
  kph = lift_over(p, 2.0)
  for _ in range(10):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.1))
  for _ in range(40):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph - 10., model_a=0.1))
  assert not p.gas_coast and sent(p) <= A_CRUISE_MIN + 1e-6


def test_the_gas_again_ends_the_coast():
  p = planner()
  kph = lift_over(p, 3.0)
  for _ in range(10):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.1))
  press(p, n=2, gas=True, v_cruise_kph=kph)
  assert not p.gas_coast and p.a_settle is None


def test_a_close_lead_and_a_model_stop_still_win_while_coasting():
  p = planner()
  kph = lift_over(p, 3.0)
  for _ in range(5):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=-1.5))
  assert sent(p) <= -1.0
  q = planner()
  kph = lift_over(q, 3.0)
  for _ in range(20):
    q.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.1, lead=8.))
  assert sent(q) < -1.0


def test_a_curve_target_while_coasting_brakes_from_the_cruise_state_at_once(monkeypatch):
  orig = LongitudinalPlannerSP.update_targets
  scc = {'on': False}

  def maybe_scc(self, *args):
    out = orig(self, *args)
    if scc['on']:
      self.source = SCC_SOURCES[0]
    return out
  monkeypatch.setattr(LongitudinalPlannerSP, 'update_targets', maybe_scc)
  p = planner()
  kph = lift_over(p, 3.0)
  for _ in range(40):
    p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.1))
  assert sent(p) == pytest.approx(-0.3, abs=1e-6)
  scc['on'] = True
  p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.1))
  assert not p.gas_coast and sent(p) == pytest.approx(p.a_cruise, abs=1e-6)
  assert sent(p) <= -0.45


@pytest.mark.parametrize("from_the_gas", [True, False])
def test_a_forced_decel_brakes_as_before_never_coasts(from_the_gas):
  # review: forceDecel already on while the gas was held opened a coast that held -0.3 for good
  p, ref = planner(), reference()
  for q in (p, ref):
    kph = lift_over(q, 3.0, force_decel=from_the_gas)
  if not from_the_gas:
    for _ in range(10):
      p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.1))
      ref.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.1))
  for i in range(40):
    for q in (p, ref):
      q.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.1, force_decel=True))
    # from the first forced frame on, the plan is the one before this change: no coast, no easing in
    assert not p.gas_coast and sent(p) == pytest.approx(sent(ref), abs=1e-6), i
  assert sent(p) <= A_CRUISE_MIN + 1e-6


def test_gas_held_through_set_settles_and_does_not_coast():
  p = planner()
  kph = (V_EGO - 3.0) * 3.6
  engage(p, gas=True, a_ego=0.1, v_cruise_kph=kph)
  p.update(build_sm(a_ego=0.0, v_cruise_kph=kph, model_a=0.1))
  assert not p.gas_coast
