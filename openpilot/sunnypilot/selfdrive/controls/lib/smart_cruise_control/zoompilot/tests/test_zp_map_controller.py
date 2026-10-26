"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import math
import platform

import numpy as np
import pytest


from openpilot.cereal import custom, messaging
from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import COMMIT_FRAC
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.map_controller import _CONFIRM_T, _CONFIRM_V_BP, _FIX_HOLD_T, R, \
  SmartCruiseControlMap
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.speed_profile import lead_distance, required_decel
from openpilot.common.test import OpenpilotTestCase

MapState = VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState


class FakeSM(dict):
  """plannerd's SubMaster as the map controller reads it: the gps service and the modelV2 clock.
  Fresh: every frame brings a fix at the current position, as the old param path read it."""

  def __init__(self, service: str):
    super().__init__()
    self.service = service
    self.logMonoTime = {'modelV2': 0, service: 0}
    self.valid = {'liveMapDataSP': True}
    self.alive = {'liveMapDataSP': True}
    self.fresh = True
    self.fix(0., 0.)

  def fix(self, lat: float, lon: float, has_fix: bool = True) -> None:
    gps = getattr(messaging.new_message(self.service), self.service)
    gps.latitude, gps.longitude, gps.hasFix = lat, lon, has_fix
    self[self.service] = gps
    self.logMonoTime[self.service] = self.logMonoTime['modelV2']

  def tick(self) -> 'FakeSM':
    self.logMonoTime['modelV2'] += int(DT_MDL * 1e9)
    if self.fresh:
      self.logMonoTime[self.service] = self.logMonoTime['modelV2']
    return self


class TestSmartCruiseControlMap(OpenpilotTestCase):

  def setup_method(self):
    self.params = Params()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.reset_params()
    cp = structs.CarParams(brand="mazda", openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.36)
    self.scc_m = SmartCruiseControlMap(cp)
    self.sm = FakeSM(self.scc_m.gps_location_service)

  @staticmethod
  def confirm_frames(v_ego):
    return int(np.interp(v_ego, _CONFIRM_V_BP, _CONFIRM_T) / DT_MDL) + 1

  def bind(self, scc, v_ego, v_cruise):
    for _ in range(self.confirm_frames(v_ego) + 2):
      scc.update(self.sm.tick(), True, False, v_ego, 0., v_cruise)

  def reset_params(self):
    self.params.put_bool("SmartCruiseControlMap", True, block=True)

    self.params.put("MapTargetVelocities", "{}", block=True)

  def test_moderate_curve(self):
    # Regression: `... / 2 * a` parsed as `(.../2)*a` instead of `.../(2*a)`,
    # making max_d ~11x too small so the moderate-curve branch never tripped.
    # v_ego=25, a_ego=0, tv=24: fixed max_d≈45m vs buggy ≈4m at a 40m waypoint.
    waypoint_lon_deg = (40.0 / R) * (180.0 / math.pi)
    self.sm.fix(0.0, 0.0)
    self.mem_params.put("MapTargetVelocities",
                        json.dumps([{"latitude": 0.0, "longitude": waypoint_lon_deg, "velocity": 24.0}]), block=True)

    self.bind(self.scc_m, 25.0, 30.0)

    self.assertAlmostEqual(self.scc_m.v_target, 24.0, delta=24.0 * 1e-6)

  # TODO-SP: mock data from modelV2 to test other states

  def test_active_target_publishes_required_decel(self):
    lat0, lon0 = 32.0, -117.0
    dlat = 200.0 / 111194.9  # ~200 m north
    self.sm.fix(lat0, lon0)
    self.mem_params.put("MapTargetVelocities", json.dumps([
      {"latitude": lat0, "longitude": lon0, "velocity": 30.0},
      {"latitude": lat0 + dlat, "longitude": lon0, "velocity": 15.0},
    ]), block=True)

    v_ego = 25.
    self.bind(self.scc_m, v_ego, 25.)
    assert self.scc_m.state == MapState.turning
    assert self.scc_m.output_v_target == pytest.approx(15.)
    assert 150. < self.scc_m.target_distance < 250.

    # required decel to the target, reached through the publication ramp
    expected = (15. ** 2 - v_ego ** 2) / (2. * self.scc_m.target_distance)
    for _ in range(40):
      self.scc_m.update(self.sm.tick(), True, False, v_ego, 0., 25.)
    assert self.scc_m.output_a_target == pytest.approx(expected, abs=1e-3)

  def test_kinked_highway_target_relaxed(self):
    """A sweeping bend mapped as one kinked node (route 18b 465 s): at speed the node's 15 m/s
    is relaxed to the bend's 100 m curve speed, so the car does not brake to 15"""
    lat0, lon0 = 32.0, -117.0
    m_per_deg = R * math.pi / 180.
    targets, north, east = [], 0., 0.
    for i in range(21):
      targets.append({"latitude": lat0 + north / m_per_deg, "longitude": lon0 + east / (m_per_deg * math.cos(math.radians(lat0))),
                      "velocity": 15.0 if i == 10 else 40.0})
      h = 0. if i < 10 else math.radians(20.)
      north, east = north + 30. * math.cos(h), east + 30. * math.sin(h)
    self.sm.fix(lat0, lon0)  # bearing 0: heading north along the road
    self.mem_params.put("MapTargetVelocities", json.dumps(targets), block=True)

    # without a model path the camera cannot agree: mapd's 15 binds
    self.bind(self.scc_m, 27., 40.)
    assert self.scc_m.v_target == pytest.approx(15.)

    # the camera sees the road straight to 600 m, but a gps bearing of exactly 0 is "no course": not judged
    model = messaging.new_message('modelV2').modelV2
    model.position.x = np.linspace(0., 600., 33).tolist()
    model.position.y = [0.] * 33
    self.sm['modelV2'] = model
    self.bind(self.scc_m, 27., 40.)
    assert self.scc_m.v_target == pytest.approx(15.)

    # with a course (just west of north): relaxed, and the bound 15 is released
    self.sm[self.sm.service].bearingDeg = 359.9
    self.bind(self.scc_m, 27., 40.)
    assert self.scc_m.v_target != pytest.approx(15.)
    assert self.scc_m.v_target in (0., pytest.approx(23.94, abs=0.01))

  def test_lowered_target_stays_bound(self):
    """A bound relaxed target that the camera then reads tighter is lowered in place: same target, no
    release, no new confirmation wait"""
    lat0, lon0 = 32.0, -117.0
    m_per_deg = R * math.pi / 180.
    targets, north, east = [], 0., 0.
    for i in range(21):
      targets.append({"latitude": lat0 + north / m_per_deg, "longitude": lon0 + east / (m_per_deg * math.cos(math.radians(lat0))),
                      "velocity": 15.0 if i == 10 else 40.0})
      h = 0. if i < 10 else math.radians(20.)
      north, east = north + 30. * math.cos(h), east + 30. * math.sin(h)
    self.sm.fix(lat0, lon0)
    self.sm[self.sm.service].bearingDeg = 359.9
    self.mem_params.put("MapTargetVelocities", json.dumps(targets), block=True)
    model = messaging.new_message('modelV2').modelV2
    model.position.x = np.linspace(0., 600., 301).tolist()
    model.position.y = [0.] * 301
    self.sm['modelV2'] = model
    self.bind(self.scc_m, 32., 40.)
    assert self.scc_m.v_target == pytest.approx(23.94, abs=0.01)
    age = self.scc_m.candidate_age

    # the camera now sees 0.15 rad at the kink (20.0 m/s over the 30 m chords)
    x = np.linspace(0., 600., 301)
    model.position.y = np.where(x < 300., 0., (x - 300.) * math.tan(0.15)).tolist()
    self.sm['modelV2'] = model
    self.scc_m.update(self.sm.tick(), True, False, 32., 0., 40.)
    assert self.scc_m.v_target == pytest.approx(20.0, abs=0.01)
    assert self.scc_m.candidate_age == age + 1

  def test_lowered_target_retained_under_the_gate(self):
    """The car has already slowed to 21 m/s, so the relaxed target no longer passes the commit gate but is
    still ahead: retained, and the camera's lower reading moves it in place"""
    lat0, lon0 = 32.0, -117.0
    m_per_deg = R * math.pi / 180.
    targets, north, east = [], 0., 0.
    for i in range(21):
      targets.append({"latitude": lat0 + north / m_per_deg, "longitude": lon0 + east / (m_per_deg * math.cos(math.radians(lat0))),
                      "velocity": 15.0 if i == 10 else 40.0})
      h = 0. if i < 10 else math.radians(20.)
      north, east = north + 30. * math.cos(h), east + 30. * math.sin(h)
    self.sm.fix(lat0, lon0)
    self.sm[self.sm.service].bearingDeg = 359.9
    self.mem_params.put("MapTargetVelocities", json.dumps(targets), block=True)
    x = np.linspace(0., 600., 301)
    model = messaging.new_message('modelV2').modelV2
    model.position.x = x.tolist()
    model.position.y = [0.] * 301
    self.sm['modelV2'] = model
    self.bind(self.scc_m, 32., 40.)
    assert self.scc_m.v_target == pytest.approx(23.94, abs=0.01)

    model.position.y = np.where(x < 300., 0., (x - 300.) * math.tan(0.15)).tolist()
    self.sm['modelV2'] = model
    self.scc_m.update(self.sm.tick(), True, False, 21., 0., 40.)
    assert self.scc_m.v_target == pytest.approx(20.0, abs=0.01)
    assert 250. < self.scc_m.target_distance < 310.

  def approach_real_curve(self, v_cam, seed=0):
    """Drive at a real curve mapped as one 20 deg node (mapd 15 m/s) 600 m ahead, the camera reading
    it at v_cam(distance, rng) m/s; returns the speed reaching it and how often a bound target dropped"""
    rng = np.random.default_rng(seed)
    lat0, lon0 = 32.0, -117.0
    m_per_deg = R * math.pi / 180.
    targets, north, east = [], 0., 0.
    for i in range(41):
      targets.append({"latitude": lat0 + north / m_per_deg, "longitude": lon0 + east / (m_per_deg * math.cos(math.radians(lat0))),
                      "velocity": 15.0 if i == 20 else 40.0})
      h = 0. if i < 20 else math.radians(20.)
      north, east = north + 30. * math.cos(h), east + 30. * math.sin(h)
    self.mem_params.put("MapTargetVelocities", json.dumps(targets), block=True)
    s, v, prev, releases = 100., 27., 0., 0
    while s < 595.:
      self.sm.fix(lat0 + s / m_per_deg, lon0)
      self.sm[self.sm.service].bearingDeg = 359.9
      d = 600. - s
      if v_cam is not None:
        x = np.linspace(0., 300., 61)
        turn = 60. / v_cam(d, rng) ** 2  # the 30 m chord's heading change for that curve speed
        model = messaging.new_message('modelV2').modelV2
        model.position.x = x.tolist()
        model.position.y = np.where(x > d, (x - d) * math.tan(turn), 0.).tolist()
        self.sm['modelV2'] = model
      self.scc_m.update(self.sm.tick(), True, False, v, 0., 40.)
      releases += bool(prev and not self.scc_m.v_target)
      prev = self.scc_m.v_target
      v = max(0., v + min(0., self.scc_m.output_a_target if self.scc_m.v_target else 0.) * DT_MDL)
      s += v * DT_MDL
    return v, releases

  def test_real_curve_braking_kept_with_camera(self):
    """The camera reads a real curve near mapd's speed, noisily or tightening as it nears (review of the
    relaxation): a bound target is never dropped, noise changes nothing, and a camera that reads the curve
    gentle (18.5 m/s) until 150 m out still has the car take it under 2.8 m/s2 (mapd's 15 m/s at 2.0 is a
    112.5 m radius; it arrives at 17.0 m/s, 2.57 m/s2)"""
    v_map, rel = self.approach_real_curve(None)
    assert rel == 0 and v_map == pytest.approx(16.27, abs=0.01)
    for v_cam, seed, same in [(lambda d, r: 15., 0, True), (lambda d, r: 16. + r.normal(0., 1.2), 1, True),
                              (lambda d, r: 16. + r.normal(0., 1.2), 2, True), (lambda d, r: 17. + r.normal(0., 0.7), 3, True),
                              (lambda d, r: 18.5 if d > 150. else 15.5 + 3. * max(d - 100., 0.) / 50., 0, False)]:
      self.scc_m = SmartCruiseControlMap(structs.CarParams(brand="mazda", openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.36))
      self.sm = FakeSM(self.scc_m.gps_location_service)
      v, rel = self.approach_real_curve(v_cam, seed)
      assert rel == 0
      assert v ** 2 / (15. ** 2 / 2.0) <= 2.6
      if same:
        assert v == pytest.approx(v_map, abs=0.01)

  def test_real_curve_read_gentle_until_100_m(self):
    """The accepted exposure, pinned: a camera that reads a real curve gentle (22 m/s) until 100 m out
    lets the car arrive at 19.1 m/s, 3.25 m/s2 on that curve (review 3); mapd alone arrives at 16.27"""
    v, rel = self.approach_real_curve(lambda d, r: 22. if d > 100. else 15.)
    assert rel == 0
    assert v == pytest.approx(19.1, abs=0.1)

  def test_kinked_target_relaxed_heading_east(self):
    """The same road heading east: the camera check finds the kink along the gps bearing"""
    lat0, lon0 = 32.0, -117.0
    m_per_deg = R * math.pi / 180.
    targets, north, east = [], 0., 0.
    for i in range(21):
      targets.append({"latitude": lat0 + north / m_per_deg, "longitude": lon0 + east / (m_per_deg * math.cos(math.radians(lat0))),
                      "velocity": 15.0 if i == 10 else 40.0})
      h = 0. if i < 10 else math.radians(20.)
      north, east = north + 30. * math.sin(h), east + 30. * math.cos(h)
    self.sm.fix(lat0, lon0)
    self.sm[self.sm.service].bearingDeg = 90.
    self.mem_params.put("MapTargetVelocities", json.dumps(targets), block=True)
    # the camera sees the road straight to 600 m; the kink lies 300 m ahead only along a 90 deg bearing
    model = messaging.new_message('modelV2').modelV2
    model.position.x = np.linspace(0., 600., 33).tolist()
    model.position.y = [0.] * 33
    self.sm['modelV2'] = model
    self.bind(self.scc_m, 27., 40.)
    assert self.scc_m.v_target != pytest.approx(15.)
    assert self.scc_m.v_target in (0., pytest.approx(23.94, abs=0.01))

  def _reversal(self, start, v=2.):
    """mapd's list from start m ahead, doubling back at its third node (route 17a, the bridge); the car heads east"""
    lat0, lon0 = 32.0, -117.0
    m_per_deg = R * math.pi / 180.
    targets, ahead, right = [], start, 0.
    for i in range(8):
      # heading east: ahead is east, right is south
      targets.append({"latitude": lat0 - right / m_per_deg, "longitude": lon0 + ahead / (m_per_deg * math.cos(math.radians(lat0))),
                      "velocity": v if 1 <= i <= 3 else 40.0})
      h = 0. if i < 2 else math.radians(150.)
      ahead, right = ahead + 20. * math.cos(h), right + 20. * math.sin(h)
    self.sm.fix(lat0, lon0)
    self.sm[self.sm.service].bearingDeg = 90.
    self.mem_params.put("MapTargetVelocities", json.dumps(targets), block=True)

  def _straight_model(self):
    model = messaging.new_message('modelV2').modelV2
    model.position.x = np.linspace(0., 200., 33).tolist()
    model.position.y = [0.] * 33
    self.sm['modelV2'] = model

  def _fresh(self, model: bool):
    self.scc_m = SmartCruiseControlMap(structs.CarParams(brand="mazda", openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.36))
    self.sm = FakeSM(self.scc_m.gps_location_service)
    if model:
      self._straight_model()

  def test_reversal_on_a_straight_road_dropped(self):
    """With the camera seeing the road straight, the reversal's targets are dropped; not below 15 mph,
    where a gps course is unreliable"""
    self._fresh(model=True)
    self._reversal(5.)
    self.bind(self.scc_m, 6.7, 20.)
    assert self.scc_m.v_target == 0.
    self._fresh(model=True)
    self._reversal(5.)
    self.bind(self.scc_m, 6.6, 20.)
    assert self.scc_m.v_target == pytest.approx(2.)

  def test_reversal_target_being_braked_for_is_kept(self):
    """A target bound before the guard could judge it (here: no model path yet) is not dropped"""
    self._fresh(model=False)
    self._reversal(5.)
    self.bind(self.scc_m, 6.7, 20.)
    assert self.scc_m.v_target == pytest.approx(2.)
    self._straight_model()
    self.bind(self.scc_m, 6.7, 20.)
    assert self.scc_m.v_target == pytest.approx(2.)

  def test_reversal_judged_on_the_reckoned_position(self):
    """Between 1 Hz fixes the reversal node (65 m from the fix) comes within the 60 m judged as the car
    covers the road, before its 7 m/s targets would bind"""
    first = {}
    for model in (False, True):
      self._fresh(model)
      self._reversal(25., v=7.)
      self.sm.fresh = False
      bound = []
      for _ in range(int(2.9 / DT_MDL)):
        self.scc_m.update(self.sm.tick(), True, False, 10., 0., 20.)
        bound.append(self.scc_m.v_target)
      first[model] = next((i for i, v in enumerate(bound) if v != 0.), None)
    assert first[False] is not None and first[False] == 12  # 0.6 s: the node is in reach by then
    assert first[True] is None

  def test_retained_target_tracks_distance(self):
    """A target that slips under the commit gate while still ahead (the car is already
    slowing) is retained. Its distance must keep tracking the car: the published decel
    divides by it every frame, and frozen at the commit-time value it under-requests
    more the closer the car gets."""
    lat0, lon0 = 32.0, -117.0
    m_per_deg = 111194.9
    target_lat = lat0 + 200.0 / m_per_deg
    self.sm.fix(lat0, lon0)
    self.mem_params.put("MapTargetVelocities", json.dumps([
      {"latitude": lat0, "longitude": lon0, "velocity": 30.0},
      {"latitude": target_lat, "longitude": lon0, "velocity": 15.0},
    ]), block=True)
    self.bind(self.scc_m, 25., 25.)
    assert self.scc_m.state == MapState.turning

    # slowing harder than the gate needs while closing: the target leaves valid_velocities
    # after a few frames (a_req falls under COMMIT_FRAC * a_budget) but stays ahead
    v_ego, d = 25., 200.
    for _ in range(12):
      v_ego -= 0.7
      d -= 6.
      self.sm.fix(target_lat - d / m_per_deg, lon0)
      self.scc_m.update(self.sm.tick(), True, False, v_ego, 0., 25.)
      assert self.scc_m.is_active and self.scc_m.v_target == pytest.approx(15.)
      assert self.scc_m.target_distance == pytest.approx(d, abs=1.), "retained target kept its commit-time distance"

    # hold the state so the ramp settles: the decel must be the one for the current distance
    for _ in range(40):
      self.scc_m.update(self.sm.tick(), True, False, v_ego, 0., 25.)
    expected = (15. ** 2 - v_ego ** 2) / (2. * max(d, v_ego * 2.8))
    assert self.scc_m.output_a_target == pytest.approx(expected, abs=1e-3)

  def test_commit_gate_and_decel_are_the_shared_solver(self):
    # The commit gate and output must share speed_profile.required_decel.
    lat0, lon0 = 32.0, -117.0
    self.sm.fix(lat0, lon0)
    self.mem_params.put("MapTargetVelocities", json.dumps([
      {"latitude": lat0, "longitude": lon0, "velocity": 30.0},
      {"latitude": lat0 + 200.0 / 111194.9, "longitude": lon0, "velocity": 15.0},
    ]), block=True)
    v_ego = 25.
    self.bind(self.scc_m, v_ego, 25.)
    lim = self.scc_m.limits
    d = self.scc_m.target_distance
    d_lead = lead_distance(v_ego, lim.t_lead + lim.dash_traversal_time(v_ego - 15.), lim.a_budget, lim.jerk(v_ego))
    a_req = required_decel(v_ego, [15.], [d], d_lead)
    assert a_req >= COMMIT_FRAC * lim.a_budget
    assert self.scc_m.is_active
    for _ in range(60):
      self.scc_m.update(self.sm.tick(), True, False, v_ego, 0., 25.)
    assert self.scc_m.output_a_target == pytest.approx(-required_decel(v_ego, [15.], [max(d, v_ego * 2.8)]), abs=1e-3)

  def _steep_target(self, scc, v_ego=25.):
    # 5 m/s target ~200 m ahead: raw required decel 1.5 m/s2, past the op-long budget
    self._targets(5.)
    for _ in range(self.confirm_frames(v_ego) + 60):
      scc.update(self.sm.tick(), True, False, v_ego, 0., v_ego)
    assert scc.is_active
    return (5. ** 2 - v_ego ** 2) / (2. * scc.target_distance)

  def test_op_long_published_decel_is_clipped_to_the_budget(self):
    # the plan aTarget seeds the MPC on openpilot long, and stage 0 is pinned to the seed,
    # so anything under the budget would bypass A_CRUISE_MIN through the MPC candidate
    raw = self._steep_target(self.scc_m)
    assert raw < -1.2
    assert self.scc_m.output_a_target == pytest.approx(-self.scc_m.limits.a_budget, abs=1e-6)

  def test_stock_path_asks_for_the_budget_at_most(self):
    # on stock ACC the wire is ICBM's overshoot lever; a map target is a prediction, so it may
    # ask for the planning budget but never for the ECU's range past it (route 25c: map targets
    # flickered on for 1-2 s at 55 mph over road that never curved)
    cp = structs.CarParams(brand="mazda", openpilotLongitudinalControl=False)
    stock = SmartCruiseControlMap(cp)
    raw = self._steep_target(stock)
    assert raw < -stock.limits.a_budget
    assert stock.output_a_target == pytest.approx(-stock.limits.a_budget, abs=1e-6)

  def _targets(self, *speeds, d0=200.):
    # ego at the origin; each target another 100 m north, at the given speeds
    lat0, lon0 = 32.0, -117.0
    m_per_deg = 111194.9
    self.sm.fix(lat0, lon0)
    pts = [{"latitude": lat0, "longitude": lon0, "velocity": 30.0}]
    pts += [{"latitude": lat0 + (d0 + 100. * i) / m_per_deg, "longitude": lon0, "velocity": v} for i, v in enumerate(speeds)]
    self.mem_params.put("MapTargetVelocities", json.dumps(pts), block=True)

  def test_target_binds_at_once_below_45mph(self):
    self._targets(8., d0=100.)
    self.scc_m.update(self.sm.tick(), True, False, 15., 0., 20.)
    assert self.scc_m.v_target == pytest.approx(8.)
    self.scc_m.update(self.sm.tick(), True, False, 15., 0., 20.)
    assert self.scc_m.state == MapState.turning

  def test_target_needs_confirmation_at_speed(self):
    # a target at 25 m/s (56 mph) is a prediction: it must hold for the confirmation time
    # before it binds, and a frame without it starts the count over
    self._targets(15.)
    n = self.confirm_frames(25.)
    for _ in range(n - 1):
      self.scc_m.update(self.sm.tick(), True, False, 25., 0., 25.)
      assert self.scc_m.v_target == 0. and not self.scc_m.is_active
    self.mem_params.put("MapTargetVelocities", "[]", block=True)
    self.scc_m.update(self.sm.tick(), True, False, 25., 0., 25.)
    self._targets(15.)
    for _ in range(n - 1):
      self.scc_m.update(self.sm.tick(), True, False, 25., 0., 25.)
      assert self.scc_m.v_target == 0.
    self.scc_m.update(self.sm.tick(), True, False, 25., 0., 25.)
    assert self.scc_m.v_target == pytest.approx(15.)
    self.scc_m.update(self.sm.tick(), True, False, 25., 0., 25.)
    assert self.scc_m.state == MapState.turning

  def test_deeper_target_waits_while_the_bound_one_holds(self):
    self._targets(15.)
    self.bind(self.scc_m, 25., 25.)
    assert self.scc_m.v_target == pytest.approx(15.)
    # a slower target appears further on: the bound one keeps driving the plan until the
    # new one is confirmed, then it takes over
    self._targets(15., 10.)
    for _ in range(self.confirm_frames(25.) - 1):
      self.scc_m.update(self.sm.tick(), True, False, 25., 0., 25.)
      assert self.scc_m.v_target == pytest.approx(15.)
      assert self.scc_m.is_active
    self.scc_m.update(self.sm.tick(), True, False, 25., 0., 25.)
    assert self.scc_m.v_target == pytest.approx(10.)

  def test_release_is_immediate(self):
    self._targets(15.)
    self.bind(self.scc_m, 25., 25.)
    assert self.scc_m.is_active
    self.mem_params.put("MapTargetVelocities", "[]", block=True)
    self.scc_m.update(self.sm.tick(), True, False, 25., 0., 25.)
    assert self.scc_m.v_target == 0. and not self.scc_m.is_active

  def test_flicker_during_a_bound_target_keeps_it(self):
    # a phantom deeper target that flickers for under the confirmation time never binds and
    # the bound target keeps its distance tracking meanwhile
    self._targets(15.)
    self.bind(self.scc_m, 25., 25.)
    d0 = self.scc_m.target_distance
    self._targets(15., 5.)
    for _ in range(10):
      self.scc_m.update(self.sm.tick(), True, False, 25., 0., 25.)
      assert self.scc_m.v_target == pytest.approx(15.)
    self._targets(15.)
    self.scc_m.update(self.sm.tick(), True, False, 25., 0., 25.)
    assert self.scc_m.v_target == pytest.approx(15.)
    assert self.scc_m.target_distance == pytest.approx(d0, abs=1.)

  def drive(self, scc, v_ego, frames, position_valid=True):
    self.sm.valid['liveMapDataSP'] = self.sm.alive['liveMapDataSP'] = position_valid
    for _ in range(frames):
      scc.update(self.sm.tick(), True, False, v_ego, 0., v_ego)

  def test_invalid_position_before_a_curve_does_not_bind(self):
    # the same 15 m/s target binds at 25 m/s with a valid position (test_active_target_publishes_required_decel)
    self._targets(15.)
    self.drive(self.scc_m, 25., self.confirm_frames(25.) + 60, position_valid=False)
    assert self.scc_m.state == MapState.enabled and not self.scc_m.is_active
    assert self.scc_m.v_target == 0.
    assert self.scc_m.output_v_target == V_CRUISE_UNSET

  def test_position_lost_while_turning_releases_the_target(self):
    self._targets(15.)
    self.drive(self.scc_m, 25., self.confirm_frames(25.) + 2)
    assert self.scc_m.state == MapState.turning and self.scc_m.output_v_target == pytest.approx(15.)

    self.drive(self.scc_m, 25., 1, position_valid=False)
    assert self.scc_m.state == MapState.enabled and not self.scc_m.is_active
    assert self.scc_m.v_target == 0. and self.scc_m.target_distance == 0.
    assert self.scc_m.output_v_target == V_CRUISE_UNSET
    assert self.scc_m.output_a_target == 0.  # a_ego, no decel request left behind

  def test_a_frozen_target_cannot_hold_a_speed_cap(self):
    # routes 155/156: the position stays where it was while the car drives on at speed. A valid position
    # holds the cap as long as the target is listed (control); an invalid one never lets it bind or hold.
    self._targets(15.)
    cp = structs.CarParams(brand="mazda", openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.36)
    held = SmartCruiseControlMap(cp)
    self.drive(held, 25., int(60. / DT_MDL))
    assert held.state == MapState.turning and held.output_v_target == pytest.approx(15.)

    self.drive(self.scc_m, 25., self.confirm_frames(25.) + 2)
    assert self.scc_m.is_active
    for _ in range(int(60. / DT_MDL)):
      self.drive(self.scc_m, 25., 1, position_valid=False)
      assert not self.scc_m.is_active and self.scc_m.output_v_target == V_CRUISE_UNSET

  def test_valid_position_restores_curve_behavior(self):
    self._targets(15.)
    self.drive(self.scc_m, 25., self.confirm_frames(25.) + 2)
    self.drive(self.scc_m, 25., 5, position_valid=False)
    assert not self.scc_m.is_active

    # back to a valid position: the target is confirmed again before it binds, as for any new target
    self.drive(self.scc_m, 25., 1)
    assert not self.scc_m.is_active
    self.drive(self.scc_m, 25., self.confirm_frames(25.) + 2)
    assert self.scc_m.state == MapState.turning
    assert self.scc_m.output_v_target == pytest.approx(15.)

  def _bound_200m_ahead(self, v_ego=25.):
    lat0, lon0 = 32.0, -117.0
    self.sm.fix(lat0, lon0)
    self.mem_params.put("MapTargetVelocities", json.dumps([
      {"latitude": lat0, "longitude": lon0, "velocity": 30.0},
      {"latitude": lat0 + 200.0 / 111194.9, "longitude": lon0, "velocity": 15.0},
    ]), block=True)
    self.bind(self.scc_m, v_ego, 25.)
    assert self.scc_m.is_active
    return lat0, lon0

  def test_message_without_a_fix_keeps_the_last_position(self):
    # upstream reads (0, 0) off such a message: thousands of km to every point, the target released
    lat0, lon0 = self._bound_200m_ahead()
    d = self.scc_m.target_distance
    self.sm.fix(0., 0., has_fix=False)
    self.scc_m.update(self.sm.tick(), True, False, 25., 0., 25.)
    assert self.scc_m.is_active and self.scc_m.v_target == pytest.approx(15.)
    assert self.scc_m.target_distance == pytest.approx(d - 25. * DT_MDL, abs=0.1)  # reckoned on from the fix
    assert (self.scc_m.last_position.latitude, self.scc_m.last_position.longitude) == (lat0, lon0)

  def test_distance_is_dead_reckoned_between_fixes(self):
    v_ego = 25.
    self._bound_200m_ahead(v_ego)
    d = self.scc_m.target_distance
    self.sm.fresh = False  # gpsLocation is 1 Hz
    frames = 10
    for _ in range(frames):
      self.scc_m.update(self.sm.tick(), True, False, v_ego, 0., 25.)
    assert self.scc_m.target_distance == pytest.approx(d - v_ego * frames * DT_MDL, abs=0.1)

  def test_position_counts_as_lost_past_the_hold(self):
    self._bound_200m_ahead()
    self.sm.fresh = False
    for _ in range(int(_FIX_HOLD_T / DT_MDL) + 2):  # 25 m/s for 3 s: the target is still ahead
      self.scc_m.update(self.sm.tick(), True, False, 25., 0., 25.)
    assert self.scc_m.v_target == 0. and self.scc_m.candidate is None
