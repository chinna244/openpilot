from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import messaging
from openpilot.common.constants import CV
import openpilot.sunnypilot.speed_limit_vision.speed_limit_vision as slv
from openpilot.sunnypilot.speed_limit_vision.sign_models import SignModels, model_pkl_path
from openpilot.sunnypilot.speed_limit_vision.speed_limit_vision import ForkParams, MemoryParams, SpeedLimitVisionDaemon
from openpilot.system.manager.process_config import speed_limit_vision


class FakeParams:
  def __init__(self, values):
    self.values = values

  def get_bool(self, key):
    return self.values.get(key, False)


def daemon(map_data=None):
  d = SpeedLimitVisionDaemon.__new__(SpeedLimitVisionDaemon)
  d.is_metric = False
  d.params_memory = MemoryParams()
  d.sm = {"liveMapDataSP": map_data} if map_data is not None else None
  return d


def test_fork_params_turn_off_starpilot_only_features():
  params = ForkParams(FakeParams(dict.fromkeys(slv.OFF_PARAMS, True) | {"IsMetric": True}))
  assert not any(params.get_bool(key) for key in slv.OFF_PARAMS)
  assert not params.get_bool("VisionSpeedLimitTrainingCollector", default=True)
  assert params.get_bool("IsMetric")


def test_memory_params_round_trip():
  params = MemoryParams()
  params.put_float("VisionSpeedLimit", 24.6)
  params.put("VisionSpeedLimitStatus", "Holding 55 mph")
  assert params.get_float("VisionSpeedLimit") == 24.6
  assert params.get("VisionSpeedLimitStatus", encoding="utf-8") == "Holding 55 mph"
  params.remove("VisionSpeedLimit")
  assert params.get_float("VisionSpeedLimit") == 0.0


def test_map_context_reads_live_map_data_and_its_valid_flags():
  map_data = SimpleNamespace(speedLimitValid=True, speedLimit=55 * CV.MPH_TO_MS, speedLimitAheadValid=True,
                             speedLimitAhead=45 * CV.MPH_TO_MS, speedLimitAheadDistance=80.04)
  ctx = daemon(map_data)._get_map_context()
  assert (ctx["source"], ctx["current_speed_limit_mph"], ctx["next_speed_limit_mph"], ctx["next_speed_limit_distance_m"]) == ("mapd", 55, 45, 80.0)

  map_data.speedLimitValid = map_data.speedLimitAheadValid = False
  ctx = daemon(map_data)._get_map_context()
  assert (ctx["source"], ctx["current_speed_limit_mph"], ctx["next_speed_limit_mph"]) == ("none", 0, 0)


def test_send_state_publishes_the_reading_in_m_s():
  sent = []
  d = daemon()
  d.messaging = messaging
  d.pm = SimpleNamespace(send=lambda service, msg: sent.append((service, msg)))
  d.published_speed_limit_mph, d.published_confidence = 55, 0.93
  d.last_candidate_speed_limit_mph, d.last_candidate_confidence = 45, 0.61
  d.last_frame_process_duration_s, d.last_inference_interval = 0.31, 0.78
  d.params_memory.put_int("VisionSpeedLimitSupportCount", 2)
  d.params_memory.put("VisionSpeedLimitStatus", "Holding 55 mph (93%)")

  d._send_state()

  service, msg = sent[0]
  out = msg.customReserved10
  assert service == "customReserved10"
  assert abs(out.speedLimit - 55 * CV.MPH_TO_MS) < 1e-4 and abs(out.candidateSpeedLimit - 45 * CV.MPH_TO_MS) < 1e-4
  assert abs(out.confidence - 0.93) < 1e-6 and out.supportCount == 2 and out.status == "Holding 55 mph (93%)"


def test_send_state_sends_zero_without_a_sign():
  sent = []
  d = daemon()
  d.messaging = messaging
  d.pm = SimpleNamespace(send=lambda service, msg: sent.append(msg))
  d.published_speed_limit_mph = d.published_confidence = d.last_candidate_speed_limit_mph = d.last_candidate_confidence = 0
  d.last_frame_process_duration_s = d.last_inference_interval = 0.0
  d._send_state()
  assert sent[0].customReserved10.speedLimit == 0.0 and sent[0].customReserved10.status == ""


def test_process_runs_only_onroad_with_the_toggle():
  on, off = FakeParams({"SpeedLimitVision": True}), FakeParams({})
  assert speed_limit_vision(True, on, None)
  assert not speed_limit_vision(False, on, None)
  assert not speed_limit_vision(True, off, None)


class SizedNet:
  def __init__(self, size):
    self.size = size
    self.tried = []

  def setInput(self, blob):
    self.shape = blob.shape

  def forward(self):
    self.tried.append(self.shape[2])
    if self.shape[2] != self.size:
      raise slv.cv2.error("wrong size")


needs_cv2 = pytest.mark.skipif(slv.cv2 is None, reason="the probe only serves StarPilot's cv2 single-model path")


@needs_cv2
def test_probe_finds_the_size_the_model_runs_at():
  net = SizedNet(256)
  assert SpeedLimitVisionDaemon._probe_input_size(net, 640, slv.DETECTOR_INPUT_SIZE_CANDIDATES) == 256
  assert net.tried == [640, 512, 448, 416, 384, 320, 288, 256]


@needs_cv2
def test_probe_keeps_a_size_that_runs():
  net = SizedNet(128)
  assert SpeedLimitVisionDaemon._probe_input_size(net, 128, slv.CLASSIFIER_INPUT_SIZE_CANDIDATES) == 128
  assert net.tried == [128]


@pytest.mark.skipif(not model_pkl_path("speed_limit_us_detector", 1344, 760).is_file(), reason="models not compiled (scons)")
def test_loaded_models_get_their_real_input_sizes():
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  d = SpeedLimitVisionDaemon(use_runtime=False)
  assert d.model_mode == "detector_classifier", d.last_error
  d.client = SimpleNamespace(width=1344, height=760, stride=get_nv12_info(1344, 760)[0])
  d._load_sign_models()
  assert (d.detector_input_size, d.classifier_input_size) == (256, 128), d.last_error


def sign(center_bgr):
  """A white 40x56 sign with a black border and a 20x22 patch where the number goes."""
  crop = np.full((56, 40, 3), 235, dtype=np.uint8)
  crop[:2], crop[-2:], crop[:, :2], crop[:, -2:] = 0, 0, 0, 0
  crop[22:44, 10:30] = center_bgr
  return crop


def test_blue_symbol_where_the_number_goes_is_not_read():
  # route 12 t=80: a disabled-parking sign (blue wheelchair square) read as 65 at 0.93
  d = SpeedLimitVisionDaemon.__new__(SpeedLimitVisionDaemon)
  d.classifier_net = object()  # never reached
  assert d._classify_speed_limit_from_model(sign((170, 60, 20)), (0, 0)) is None


def test_black_number_is_not_blue():
  assert SpeedLimitVisionDaemon._center_blue_ratio(sign((10, 10, 10))) == 0.0
  assert SpeedLimitVisionDaemon._center_blue_ratio(sign((170, 60, 20))) > slv.SIGN_CENTER_BLUE_MAX


def test_runtime_daemon_starts():
  """The process as manager starts it: on 45ae07ae3f it died at start on every drive (cereal, VisionStreamType and
  livePose are openpilot.cereal, openpilot.cereal.visionipc and deviceMotion in this fork)."""
  d = SpeedLimitVisionDaemon()
  assert d.sm is not None and d.pm is not None and d.VisionStreamType is not None
  d.sm.update(0)
  d._send_state()


def test_padded_nv12_frame_reads_as_the_image():
  """camerad pads the NV12 planes apart (stride, uv_offset) and msgq hands out a memoryview."""
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  width, height = 1344, 760
  stride, y_height, _, size = get_nv12_info(width, height)
  raw = np.zeros(size, np.uint8)
  raw[:stride * height].reshape(height, stride)[:, :width] = 120
  raw[stride * y_height:stride * y_height + stride * height // 2].reshape(height // 2, stride)[:, :width] = 128
  raw[:stride * height].reshape(height, stride)[100:107, 301:310] = 200  # an odd sized box at odd bounds
  d = SpeedLimitVisionDaemon.__new__(SpeedLimitVisionDaemon)
  d.sign_models = SignModels.__new__(SignModels)
  d.sign_models.geometry, d.sign_models.frame_size = (width, height, stride, stride * y_height), size
  buf = SimpleNamespace(data=memoryview(raw), uv_offset=stride * y_height)
  d.client = SimpleNamespace(recv=lambda: buf, width=width, height=height, stride=stride)
  frame = d._receive_frame_bgr()
  assert frame.shape == (height, width, 3)
  full = frame[:, :]
  assert abs(int(full[:50].mean()) - 120) <= 6  # flat grey, no stripes from a misread plane
  np.testing.assert_array_equal(frame[101:106, 303:309], full[101:106, 303:309])  # a crop converts like the frame


def drive(rc, seconds, v=12., yaw=0., car=35, map_limit=0, sign=25, dt=0.1):
  for _ in range(int(seconds / dt)):
    reason = rc.update(dt, v, yaw, car, map_limit, sign)
    if reason:
      return reason
  return None


def test_sign_holds_on_the_same_road():
  """Car 35 on a 25 road (routes 0f/10/12): the camera's 25 stands while nothing says the road changed."""
  rc = slv.RoadChange()
  rc.reset(35, 30)
  assert drive(rc, 300., car=35, map_limit=30) is None  # 3.6 km: longer than the old 5 min would not matter here


def test_a_turn_ends_the_sign():
  rc = slv.RoadChange()
  rc.reset(35, 0)
  assert drive(rc, 2., yaw=0.1) is None  # a lane change or a bend: ~11 deg
  assert drive(rc, 10., yaw=0.2) == "turn"  # route 18 0:39 right turn read 80 deg


def test_a_held_limit_change_ends_the_sign_but_a_glitch_does_not():
  rc = slv.RoadChange()
  rc.reset(40, 0)
  assert drive(rc, 5., car=70, sign=40) is None  # route 17 5:14: the car read 70 on a 40 road for ~5 s
  assert drive(rc, 5., car=40, sign=40) is None
  assert drive(rc, 9., car=55, sign=40) is None
  assert drive(rc, 2., car=55, sign=40) == "car_limit"


def test_a_limit_moving_to_the_signs_value_is_agreement_not_a_new_road():
  rc = slv.RoadChange()
  rc.reset(35, 30)
  assert drive(rc, 30., car=25, map_limit=25, sign=25) is None


def test_distance_backstop():
  rc = slv.RoadChange()
  rc.reset(55, 55)
  assert drive(rc, 300., v=30., car=55, map_limit=55, sign=55) == "distance"  # 5 km with no repeat sign


def test_road_change_resets_on_a_repeat_sighting():
  d = SpeedLimitVisionDaemon.__new__(SpeedLimitVisionDaemon)
  d.is_metric = False
  d.road_change, d.road_change_key, d.road_change_at = slv.RoadChange(), None, 0.
  d.published_speed_limit_mph, d.last_published_support_at = 25, 1.
  gyro = SimpleNamespace(z=0.)
  d.sm = {"carStateSP": SimpleNamespace(speedLimit=35 * CV.MPH_TO_MS), "carState": SimpleNamespace(vEgo=10.),
          "liveMapDataSP": SimpleNamespace(speedLimit=0., speedLimitValid=False),
          "deviceMotion": SimpleNamespace(angularVelocityDevice=gyro)}
  cleared = []
  d._write_debug_event = lambda *a, **k: None
  d._clear_detection = lambda: cleared.append(True)
  t = [100.]

  def run(seconds, z):
    gyro.z = z
    for _ in range(int(seconds * 10)):
      t[0] += 0.1
      d._update_road_change(t[0])

  run(1., 0.)
  run(2.5, 0.3)  # 43 deg
  run(1., 0.)
  d.last_published_support_at = t[0]  # seen again on the straight: the count starts over
  run(1., 0.)
  run(2.5, 0.3)  # 43 deg since the repeat
  assert not cleared
  run(1.5, 0.3)  # 69 deg
  assert cleared


def test_a_sign_read_mid_turn_survives_that_turn():
  """Route 12 t=156: a 30 read 18 deg into a right turn stands on the new road; finishing the turn must not drop it,
  the next turn does."""
  rc = slv.RoadChange()
  rc.reset(0, 0, yaw_rate=0.35)  # 20 deg/s
  assert drive(rc, 4., yaw=0.35, car=0) is None  # the rest of the same turn
  assert drive(rc, 20., car=0) is None
  assert drive(rc, 6., yaw=0.3, car=0) == "turn"


def test_stopped_gyro_drift_is_not_a_turn():
  rc = slv.RoadChange()
  rc.reset(0, 0)
  assert drive(rc, 120., v=0., yaw=0.02, car=0) is None  # ~140 deg of drift at a light


def plaque_sign(top_bgr, body_bgr, patchy=False):
  """A 60x40 sign: the top 12 rows in top_bgr (every other 4-px block if patchy), the rest in body_bgr."""
  img = np.zeros((60, 40, 3), np.uint8)
  img[:] = body_bgr
  if patchy:
    for x in range(0, 40, 8):
      img[:12, x:x + 4] = top_bgr
  else:
    img[:12] = top_bgr
  return img


YELLOW, WHITE, SODIUM = (0, 220, 240), (235, 235, 235), (40, 170, 220)


def test_school_plaque_is_a_school_zone_sign():
  assert SpeedLimitVisionDaemon._is_school_zone_sign(plaque_sign(YELLOW, WHITE))  # routes 0f/10/12 by day, 1b at night


def test_a_plain_sign_is_not():
  assert not SpeedLimitVisionDaemon._is_school_zone_sign(plaque_sign(WHITE, WHITE))


def test_street_light_tint_is_not_a_plaque():
  """Route 1b 16:08: orange street light turns the whole 30 sign yellow."""
  assert not SpeedLimitVisionDaemon._is_school_zone_sign(plaque_sign(SODIUM, SODIUM))


def test_leaves_behind_a_sign_are_not_a_plaque():
  """Route 10 249 s: a 25 with autumn leaves above it."""
  assert not SpeedLimitVisionDaemon._is_school_zone_sign(plaque_sign(YELLOW, WHITE, patchy=True))
