"""StarPilot Mazda: one curve controller. The Curve Speed Controller toggle runs the vision solver; the map controller
and the native curve envelope never lower the Mazda cruise target."""
import time
from pathlib import Path

import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.tests import test_cruise_handover as H
from openpilot.starpilot.longitudinal.cruise_ceiling import CurveCeiling
from openpilot.starpilot.longitudinal.scc.planner import SccSource
from openpilot.starpilot.longitudinal.scc.tests.test_zp_vision_in_turn import FakeParams
from openpilot.starpilot.longitudinal.scc.tests.vision_harness import make_cp, model_for_road, vehicle_params
from openpilot.starpilot.longitudinal.scc.zoompilot.vision_controller import SmartCruiseControlVision


def curve_sm(v=15.0, kappa=0.02):
  sm = {'modelV2': model_for_road(v, lambda s: kappa).modelV2,
        'controlsState': messaging.new_message('controlsState').controlsState,
        'carState': messaging.new_message('carState').carState, 'vehicleParameters': vehicle_params()}
  return sm


@pytest.mark.parametrize("curve_on, old_on", [(True, False), (False, True)])
def test_curve_speed_controller_toggle_runs_the_vision_solver(curve_on, old_on):
  scc = SmartCruiseControlVision(make_cp())
  scc.params = FakeParams(CurveSpeedController=curve_on, SmartCruiseControlVision=old_on, CurveSpeedProfile=1)
  scc.frame = 0
  scc.update(curve_sm(), True, False, 15.0, 0., 30.0)
  assert scc.enabled is curve_on


def test_there_is_no_map_controller():
  p = H.planner()
  assert not hasattr(p.scc.scc, 'map')
  assert [s.name for s in SccSource] == ['cruise', 'sccVision']
  p.scc.update_targets(H.build_sm(), 20.0, 0.0, 25.0, 25.0, 0.0)
  assert p.scc.source in (SccSource.cruise, SccSource.sccVision)


def _ceiling_status(p):
  sm = H.build_sm()
  sm.logMonoTime['modelV2'] = 123
  p.update(sm, curve_provider=lambda follow: CurveCeiling(5.0, 123))
  return p.last_curve_ceiling_status


def test_native_envelope_caps_without_the_mazda_solver():
  p = H.planner()
  p.scc = None
  assert _ceiling_status(p) == "selected"


def test_native_envelope_never_caps_the_mazda_planner():
  assert _ceiling_status(H.planner()) == "absent"


def test_learned_corrections_move_the_vision_ceiling():
  from openpilot.starpilot.longitudinal.scc.zoompilot.vision_controller import curve_speed, curve_table
  kappa = 0.02
  base = float(curve_speed(kappa, curve_table()))
  assert float(curve_speed(kappa, curve_table(1.0, (1.1, 1.0, 1.0)))) > base
  assert float(curve_speed(kappa, curve_table(1.0, (0.9, 1.0, 1.0)))) < base
  scc = SmartCruiseControlVision(make_cp())
  scc.set_corrections((1.1, 1.0, 1.0))
  assert float(curve_speed(kappa, scc._curve_table)) > base


def _curve_frame(p, brake):
  sm = H.build_sm(brake=brake)
  ctl = messaging.new_message('controlsState')
  ctl.controlsState.longControlState, ctl.controlsState.curvature = H.LongCtrlState.pid, 1.5 / H.V_EGO ** 2
  sm['controlsState'] = ctl.controlsState.as_reader()
  p.scc.update_targets(sm, H.V_EGO, 0.0, 25.0, 25.0, 0.0)


@pytest.mark.parametrize("curve_on", [True, False])
def test_a_brake_in_a_curve_lowers_the_planner_ceiling_only_with_the_toggle_on(curve_on):
  p = H.planner()
  vision = p.scc.scc.vision
  vision.params = FakeParams(CurveSpeedController=curve_on, CurveSpeedProfile=1)
  vision.enabled = curve_on
  p.scc.corrections_owned = True
  _curve_frame(p, brake=False)
  before = p.scc.corrections.scales
  _curve_frame(p, brake=True)
  lowered = any(s < b for s, b in zip(p.scc.corrections.scales, before, strict=True))
  assert lowered is curve_on
  assert vision.corrections == p.scc.corrections.scales


def test_corrections_are_saved_and_restored():
  from openpilot.common.params import Params
  from openpilot.starpilot.longitudinal.scc.planner import CORRECTIONS_KEY
  params = Params()
  params.remove(CORRECTIONS_KEY)
  try:
    p = H.planner()
    p.scc.scc.vision.enabled = True
    p.scc.scc.vision.params = FakeParams(CurveSpeedController=True, CurveSpeedProfile=1)
    p.scc.take_ownership(True)
    _curve_frame(p, brake=False)
    p.scc.corrections_save_frames = 10 ** 6  # the save interval has passed
    _curve_frame(p, brake=True)
    learned = p.scc.corrections.scales
    assert any(s < 1.0 for s in learned)
    assert p.scc.close_corrections()  # the write in flight finishes (plannerd does this before releasing the lease)
    assert not p.scc.corrections.dirty
    restored = H.planner()
    assert restored.scc.corrections.scales == (1.0, 1.0, 1.0)  # nothing is read before the lease is held
    restored.scc.take_ownership(True)
    assert restored.scc.corrections.scales == pytest.approx(learned, abs=1e-4)
    assert restored.scc.scc.vision.corrections == restored.scc.corrections.scales
  finally:
    time.sleep(0.2)  # let the planner's non-blocking save land before clearing it
    params.remove(CORRECTIONS_KEY)


def _forced_planner():
  """The vision solver held binding (as if a bend were ahead) so message ordering can be scripted."""
  p = H.planner()
  vision = p.scc.scc.vision
  vision.params = FakeParams(CurveSpeedController=True, CurveSpeedProfile=1)
  vision.enabled = True
  vision.update = lambda *args, **kwargs: None
  p.scc.corrections_owned = True
  return p, vision


def _frame(p, vision, *, lat, active=True, gas=False, brake=False, long_active=True, enabled=True, lead=None, radar_valid=True,
           lead_two=None):
  vision.is_active = active
  vision.output_v_target, vision.output_a_target = (H.V_EGO - 2.0, -1.0) if active else (255., 0.)
  p.scc.binding = active  # the planner's verdict on the previous plan: the vision target was followed
  sm = H.build_sm(gas=gas, brake=brake, enabled=enabled, lead=lead)
  cc = messaging.new_message('carControl')
  cc.carControl.enabled, cc.carControl.longActive = enabled, long_active
  sm['carControl'] = cc.carControl.as_reader()
  ctl = messaging.new_message('controlsState')
  ctl.controlsState.longControlState, ctl.controlsState.curvature = H.LongCtrlState.pid, lat / H.V_EGO ** 2
  sm['controlsState'] = ctl.controlsState.as_reader()
  if lead_two is not None:
    rs = messaging.new_message('radarState')
    rs.radarState.leadTwo.present, rs.radarState.leadTwo.dRel = True, lead_two
    sm['radarState'] = rs.radarState.as_reader()
  sm.valid['radarState'] = radar_valid
  p.scc.update_targets(sm, H.V_EGO, 0.0, 25.0, 25.0, 0.0)


def test_gas_press_that_hands_back_to_cruise_on_the_same_frame_still_learns():
  p, vision = _forced_planner()
  _frame(p, vision, lat=1.5)
  assert p.scc.source == SccSource.sccVision
  _frame(p, vision, lat=1.5, active=False, gas=True, long_active=False)  # override: vision steps back, cruise wins
  assert p.scc.source == SccSource.cruise
  for _ in range(20):
    _frame(p, vision, lat=3.2, active=False, gas=True, long_active=False)
  _frame(p, vision, lat=0.5, active=False, long_active=False)
  assert p.scc.corrections.scales[0] > 1.0


def test_brake_press_that_already_disengaged_still_learns():
  p, vision = _forced_planner()
  _frame(p, vision, lat=1.6)
  _frame(p, vision, lat=1.6, brake=True, long_active=False, enabled=False)
  assert p.scc.corrections.scales[0] < 1.0


@pytest.mark.parametrize("kwargs", [{"lead": 10.0}, {"lead": float('nan')}, {"lead_two": 10.0}, {"radar_valid": False}])
def test_brake_with_a_lead_or_unknown_radar_teaches_nothing(kwargs):
  p, vision = _forced_planner()
  _frame(p, vision, lat=1.6, **kwargs)
  _frame(p, vision, lat=1.6, brake=True, **kwargs)
  assert p.scc.corrections.scales == (1.0, 1.0, 1.0)


@pytest.mark.parametrize("v, d_rel, close", [(4.0, 14.9, True), (4.0, 15.0, False), (10.0, 29.9, True), (10.0, 30.0, False)])
def test_lead_boundary_is_max_of_3_s_and_15_m(v, d_rel, close):
  from openpilot.starpilot.longitudinal.scc.planner import close_lead
  assert close_lead(H.build_sm(lead=d_rel), v) is close


def test_toggle_off_mid_watch_drops_it():
  p, vision = _forced_planner()
  _frame(p, vision, lat=1.5)
  _frame(p, vision, lat=1.5, gas=True)
  assert p.scc.corrections.watch_left > 0
  vision.enabled = False
  _frame(p, vision, lat=3.2, gas=True)
  assert p.scc.corrections.watch_left == 0.0
  vision.enabled = True
  for _ in range(20):
    _frame(p, vision, lat=3.2, gas=True)  # still held: not a new press
  _frame(p, vision, lat=0.5)
  assert p.scc.corrections.scales == (1.0, 1.0, 1.0)


def test_lead_pause_takes_the_vision_target_out():
  p, vision = _forced_planner()
  _frame(p, vision, lat=1.5)
  assert p.scc.source == SccSource.sccVision
  vision.is_active = True
  sm = H.build_sm()
  v, _ = p.scc.update_targets(sm, H.V_EGO, 0.0, 25.0, 25.0, 0.0, lead_pause=True)
  assert p.scc.source == SccSource.cruise and v == 25.0


@pytest.mark.parametrize("no_lead, following, paused", [(True, True, True), (True, None, True), (True, False, False),
                                                         (False, True, False)])
def test_plannerd_hands_the_lead_pause_to_the_planner(no_lead, following, paused):
  from types import SimpleNamespace
  from openpilot.selfdrive.controls.plannerd import update_curve_frame
  seen = {}
  planner = SimpleNamespace(update=lambda sm, **kwargs: seen.update(kwargs), last_curve_ceiling_applied=False)
  host = SimpleNamespace(runtime=SimpleNamespace(no_lead=no_lead, confirm_applied=lambda *a, **k: None),
                         last_following_lead=following)
  update_curve_frame(planner, {}, None, 1, host=host)
  assert seen['curve_lead_pause'] is paused


@pytest.mark.parametrize("pause", [False, True])
def test_the_planner_passes_the_lead_pause_on(pause):
  p, vision = _forced_planner()
  vision.is_active, vision.output_v_target, vision.output_a_target = True, H.V_EGO - 2.0, -1.0
  p.update(H.build_sm(), curve_lead_pause=pause)
  assert p.scc.source == (SccSource.cruise if pause else SccSource.sccVision)


def test_native_curve_learning_is_not_saved_on_the_mazda():
  from openpilot.selfdrive.controls.plannerd import native_curve_learning_saved
  p = H.planner()
  assert not native_curve_learning_saved(p)
  p.scc = None
  assert native_curve_learning_saved(p)


class KeyedOnly:
  """The production SubMaster: keyed access only (no __contains__ or __iter__), plus valid/alive maps."""
  def __init__(self, sm, **flags):
    self._sm, self.valid, self.alive = sm, {'radarState': True, **flags}, {'radarState': True}

  def __getitem__(self, key):
    return self._sm[key]


def test_close_lead_reads_a_keyed_only_submaster():
  from openpilot.starpilot.longitudinal.scc.planner import close_lead
  assert close_lead(KeyedOnly(H.build_sm()), 10.0) is False
  assert close_lead(KeyedOnly(H.build_sm(lead=10.0)), 10.0) is True
  stale = KeyedOnly(H.build_sm())
  stale.alive['radarState'] = False
  assert close_lead(stale, 10.0) is True


def test_nothing_is_learned_without_the_learning_lease():
  p, vision = _forced_planner()
  p.scc.corrections_owned = False
  _frame(p, vision, lat=1.6)
  _frame(p, vision, lat=1.6, brake=True)
  assert p.scc.corrections.scales == (1.0, 1.0, 1.0)


@pytest.mark.parametrize("lead, binding", [(None, True), (4.0, False)])
def test_binding_is_the_plan_actually_followed(lead, binding):
  p, vision = _forced_planner()
  vision.is_active, vision.output_v_target, vision.output_a_target = True, H.V_EGO - 2.0, -0.5
  p.update(H.build_sm(lead=lead))
  assert p.scc.source == SccSource.sccVision
  assert p.scc.binding is binding  # a close lead's MPC braking is not the curve controller's


def test_plannerd_owns_corrections_only_with_the_lease():
  from types import SimpleNamespace
  from openpilot.selfdrive.controls.plannerd import corrections_owned
  assert corrections_owned(SimpleNamespace(session_fd=5))
  assert not corrections_owned(SimpleNamespace(session_fd=None))
  assert not corrections_owned(None)


def _saved_corrections(raw):
  from openpilot.common.params import Params
  path = Path(Params().get_param_path('CurveCorrectionData'))
  path.write_bytes(raw)
  return path


def test_saved_corrections_apply_only_under_the_lease():
  path = _saved_corrections(b'{"version":1,"scales":[1.05,1.0,1.0]}')
  try:
    p = H.planner()
    assert p.scc.corrections.scales == (1.0, 1.0, 1.0)
    p.scc.take_ownership(False)
    assert p.scc.corrections.scales == (1.0, 1.0, 1.0) and not p.scc.corrections_owned
    p.scc.take_ownership(True)
    assert p.scc.corrections.scales == (1.05, 1.0, 1.0) and p.scc.scc.vision.corrections == (1.05, 1.0, 1.0)
  finally:
    path.unlink()


def test_an_oversized_saved_document_is_no_correction_in_the_planner_too():
  path = _saved_corrections(b'{"version":1,"scales":[1.05,1.0,1.0]}' + b' ' * 5000)  # valid JSON, over the limit
  try:
    p = H.planner()
    p.scc.take_ownership(True)
    assert p.scc.corrections.scales == (1.0, 1.0, 1.0)
  finally:
    path.unlink()


def _binding_after(p, vision, sm, **patches):
  vision.is_active, vision.output_v_target, vision.output_a_target = True, H.V_EGO - 2.0, -0.5
  p.update(sm)
  return p.scc.binding


def test_binding_needs_the_published_accel_to_be_the_scc_candidate(monkeypatch):
  from openpilot.selfdrive.controls.lib import longitudinal_planner as LP
  p, vision = _forced_planner()
  assert _binding_after(p, vision, H.build_sm()) is True
  monkeypatch.setattr(LP, 'adjust_lead_behavior', lambda *args, **kwargs: -2.5)  # a later cap takes over
  p, vision = _forced_planner()
  assert _binding_after(p, vision, H.build_sm()) is False


def test_an_mpc_win_reported_as_cruise_is_not_the_scc_candidate():
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalPlanSource
  p, vision = _forced_planner()
  original = p.mpc.update

  def force_stop_mpc(*args, **kwargs):  # e.g. a Force Stop obstacle owning the MPC solution reports "cruise"
    original(*args, **kwargs)
    p.mpc.source = LongitudinalPlanSource.cruise
  p.mpc.update = force_stop_mpc
  assert _binding_after(p, vision, H.build_sm(lead=4.0)) is False


def test_shutdown_releases_the_lease_only_after_the_write_lands():
  import threading
  from types import SimpleNamespace
  from openpilot.selfdrive.controls.plannerd import close_curve_learning
  closed = []
  prefs = SimpleNamespace(close=lambda: closed.append(True))
  p = H.planner()
  release = threading.Event()
  p.scc.corrections_writer = threading.Thread(target=release.wait, daemon=True)
  p.scc.corrections_writer.start()
  p.scc.close_corrections = lambda timeout=0.05, f=p.scc.close_corrections: f(timeout)
  close_curve_learning(p, prefs)
  assert closed == [] and not p.scc.corrections_owned  # still writing: the lease is kept, learning stops
  release.set()
  p.scc.corrections_writer.join()
  close_curve_learning(p, prefs)
  assert closed == [True]


def test_a_later_speed_ceiling_takes_the_credit_from_scc(monkeypatch):
  from openpilot.selfdrive.controls.lib import longitudinal_planner as LP
  p, vision = _forced_planner()
  assert _binding_after(p, vision, H.build_sm()) is True
  monkeypatch.setattr(LP, 'navigation_ceiling', lambda *args, **kwargs: 0.5)  # a lower ceiling after SCC's target
  p, vision = _forced_planner()
  assert _binding_after(p, vision, H.build_sm()) is False
