from types import SimpleNamespace

import numpy as np

from openpilot.cereal import messaging
from openpilot.common.constants import CV
import openpilot.sunnypilot.speed_limit_vision.speed_limit_vision as slv
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


def test_probe_finds_the_size_the_model_runs_at():
  net = SizedNet(256)
  assert SpeedLimitVisionDaemon._probe_input_size(net, 640, slv.DETECTOR_INPUT_SIZE_CANDIDATES) == 256
  assert net.tried == [640, 512, 448, 416, 384, 320, 288, 256]


def test_probe_keeps_a_size_that_runs():
  net = SizedNet(128)
  assert SpeedLimitVisionDaemon._probe_input_size(net, 128, slv.CLASSIFIER_INPUT_SIZE_CANDIDATES) == 128
  assert net.tried == [128]


def test_loaded_models_get_their_real_input_sizes():
  d = SpeedLimitVisionDaemon(use_runtime=False)
  assert (d.model_mode, d.detector_input_size, d.classifier_input_size) == ("detector_classifier", 256, 128), d.last_error


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
  assert d._classify_speed_limit_from_model(sign((170, 60, 20))) is None


def test_black_number_is_not_blue():
  assert SpeedLimitVisionDaemon._center_blue_ratio(sign((10, 10, 10))) == 0.0
  assert SpeedLimitVisionDaemon._center_blue_ratio(sign((170, 60, 20))) > slv.SIGN_CENTER_BLUE_MAX
