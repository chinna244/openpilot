import numpy as np

from openpilot.cereal import messaging
from opendbc.car.structs import car
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.vision_force_stop.runtime import VisionForceStop, SERVICES

DT_NS = 50_000_000
DRIVE_ID = 1_000


class FakeParams:
  def __init__(self, enabled=True):
    self.values = {'VisionForceStop': enabled, 'SpeedLimitVision': True}

  def get_bool(self, key):
    return self.values[key]


class FakeSM:
  def __init__(self):
    self.msgs, self.logMonoTime = {}, {}
    self.valid, self.alive, self.seen = {}, {}, {}

  def put(self, name, msg, t):
    self.msgs[name] = getattr(msg, name)
    self.logMonoTime[name] = t
    self.valid[name] = self.alive[name] = self.seen[name] = True

  def __getitem__(self, name):
    return self.msgs[name]


def model_msg(horizon):
  msg = messaging.new_message('modelV2')
  md = msg.modelV2
  md.position.x = np.linspace(0., horizon, 33).tolist()
  md.position.y = [0.] * 33
  md.velocity.x = np.linspace(10., 0., 33).tolist()
  md.orientationRate.z = [0.] * 33
  md.orientationRate.t = ModelConstants.T_IDXS
  return msg


def car_params():
  CP = car.CarParams.new_message()
  CP.openpilotLongitudinalControl = True
  CP.carFingerprint = 'MAZDA_CX5_2022'
  return CP


def frame(sm, t, *, horizon, v_ego, stop_box, gas=False, standstill=False):
  sm.put('modelV2', model_msg(horizon), t)
  cs = messaging.new_message('carState')
  cs.carState.vEgo = v_ego
  cs.carState.canValid = True
  cs.carState.gasPressed = gas
  cs.carState.standstill = standstill
  sm.put('carState', cs, t)
  cc = messaging.new_message('carControl')
  cc.carControl.longActive = not gas  # the gas overrides longitudinal control
  sm.put('carControl', cc, t)
  sm.put('controlsState', messaging.new_message('controlsState'), t)
  ss = messaging.new_message('selfdriveState')
  ss.selfdriveState.enabled = True
  sm.put('selfdriveState', ss, t)
  sm.put('radarState', messaging.new_message('radarState'), t)
  if t % (4 * DT_NS) == 0:  # the reader's sample, every 0.2 s
    msg = messaging.new_message('customReserved10')
    out = msg.customReserved10
    if stop_box:
      out.stopValid = True
      out.stopObservedMonoTime = t
      out.stopFrameId = t // DT_NS
      out.stopImageWidth, out.stopImageHeight = 1344, 760
      out.stopConfidences = [0.8]
      out.stopBoxes = [900, 200, 940, 240]
    sm.put('customReserved10', msg, t)


def run(params, *, horizon=40., stop_box=True, seconds=2.0):
  vfs, sm, CP = VisionForceStop(params), FakeSM(), car_params()
  plans = []
  for i in range(int(seconds / 0.05)):
    t = DRIVE_ID + 2_000_000_000 + i * DT_NS - (DRIVE_ID + 2_000_000_000) % (4 * DT_NS)
    frame(sm, t, horizon=horizon, v_ego=10., stop_box=stop_box)
    plans.append(vfs.sample(sm, CP, now_ns=t, drive_id=DRIVE_ID, follow_seconds=1.45))
  return plans


def test_services_are_the_planner_inputs():
  assert set(SERVICES) <= {'modelV2', 'carState', 'carControl', 'controlsState', 'selfdriveState', 'radarState'}


def test_confirmed_stop_sign_forces_a_stop():
  plans = run(FakeParams())
  assert not plans[0].forcing
  assert plans[-1].forcing
  assert plans[-1].speed_ceiling_mps is not None and plans[-1].speed_ceiling_mps < 10.
  assert plans[-1].obstacle_m is not None and plans[-1].obstacle_m <= 27.


def test_toggle_off_never_plans():
  assert not any(p.forcing or p.speed_ceiling_mps is not None for p in run(FakeParams(enabled=False)))


def test_no_sign_no_stop():
  # the model alone slowing is StarPilot's light path, not ported: no stop without a camera STOP
  assert not any(p.forcing or p.speed_ceiling_mps is not None for p in run(FakeParams(), stop_box=False))


def test_sign_needs_the_model_to_agree():
  # a STOP read while the model's path runs past 50 m (a false STOP on a wheel or a barrel) does nothing
  assert not any(p.forcing for p in run(FakeParams(), horizon=80.))


def test_gas_away_from_a_sign_stop_does_not_rearm():
  vfs, sm, CP = VisionForceStop(FakeParams()), FakeSM(), car_params()
  t = DRIVE_ID + 2_000_000_000 - (DRIVE_ID + 2_000_000_000) % (4 * DT_NS)

  def step(**kw):
    nonlocal t
    t += DT_NS
    frame(sm, t, horizon=kw.pop('horizon', 40.), stop_box=kw.pop('stop_box', True), **kw)
    return vfs.sample(sm, CP, now_ns=t, drive_id=DRIVE_ID, follow_seconds=1.45)

  for _ in range(40):
    step(v_ego=8.)
  assert vfs.plan.forcing
  for _ in range(20):
    plan = step(v_ego=0., standstill=True)
  assert plan.manual_hold  # held at the sign
  for _ in range(10):
    step(v_ego=1., gas=True)  # the driver pulls away on the gas, the sign still in view
  plans = [step(v_ego=2.) for _ in range(60)]  # gas released, 3 s rolling past the sign
  assert not any(p.forcing or p.speed_ceiling_mps is not None for p in plans)
