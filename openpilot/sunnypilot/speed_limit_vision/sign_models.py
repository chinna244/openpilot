"""Runs the compiled sign models (see compile_sign_models.py) on the GPU, and hands the reader the camera frame lazily:
slicing a CameraFrame converts only that box from NV12 to BGR."""
from pathlib import Path

import numpy as np

from openpilot.sunnypilot.speed_limit_vision.image_ops import nv12_to_bgr

MODEL_DIR = Path(__file__).parent / "models"
PAD_VALUE = 114


def model_pkl_path(name, width, height):
  return MODEL_DIR / f"{name}_{width}x{height}_tinygrad.pkl"


class CameraFrame:
  def __init__(self, raw, width, height, stride, uv_offset):
    self.raw = raw
    self.shape = (height, width, 3)
    self.size = height * width * 3
    self.y = raw[:stride * height].reshape(height, stride)[:, :width]
    self.uv = raw[uv_offset:uv_offset + stride * (height // 2)].reshape(height // 2, stride)[:, :width]

  def __getitem__(self, key):
    rows, cols = key
    height, width = self.shape[:2]
    y1, y2, _ = rows.indices(height)
    x1, x2, _ = cols.indices(width)
    if y2 <= y1 or x2 <= x1:
      return np.zeros((max(y2 - y1, 0), max(x2 - x1, 0), 3), np.uint8)
    # chroma covers 2x2 pixels: convert the even aligned box around it
    ey1, ex1 = y1 & ~1, x1 & ~1
    ey2, ex2 = min((y2 + 1) & ~1, height), min((x2 + 1) & ~1, width)
    bgr = nv12_to_bgr(self.y[ey1:ey2, ex1:ex2], self.uv[ey1 // 2:ey2 // 2, ex1:ex2])
    return bgr[y1 - ey1:y2 - ey1, x1 - ex1:x2 - ex1]


def letterbox_params(box_width, box_height, size):
  """Where cv2-style letterboxing puts a box in a size x size input: (ratio, pad_x, pad_y, (left, top, width, height))."""
  ratio = min(size / box_height, size / box_width)
  resized_width, resized_height = int(round(box_width * ratio)), int(round(box_height * ratio))
  pad_width, pad_height = (size - resized_width) / 2, (size - resized_height) / 2
  return ratio, pad_width, pad_height, (int(round(pad_width - 0.1)), int(round(pad_height - 0.1)), resized_width, resized_height)


def square_params(box_width, box_height, size):
  """Where _square_resize puts a crop: centered, aspect kept."""
  ratio = min(size / max(box_height, 1), size / max(box_width, 1))
  resized_width, resized_height = max(int(round(box_width * ratio)), 1), max(int(round(box_height * ratio)), 1)
  return (size - resized_width) // 2, (size - resized_height) // 2, resized_width, resized_height


class SignModels:
  def __init__(self, width, height, stride, uv_offset, frame_size):
    from tinygrad.dtype import dtypes
    from tinygrad.engine.realize import lower_and_compile
    from tinygrad.tensor import Tensor
    from tinygrad.uop.ops import UOp
    from openpilot.selfdrive.modeld.helpers import load_oob

    self.geometry = (width, height, stride, uv_offset)
    self.frame_size = frame_size
    self.runs, self.input_sizes, self.outputs = {}, {}, {}
    for name in ("speed_limit_us_detector", "speed_limit_us_value_classifier"):
      jit = load_oob(model_pkl_path(name, width, height))
      assert jit['input_specs']['input_frame'][0] == (frame_size,), jit['input_specs']
      jit['run'].captured._linear = lower_and_compile(jit['run'].captured._linear)
      self.runs[name] = jit['run']
      self.input_sizes[name] = jit['metadata']['input_size']
      self.outputs[name] = {k: Tensor(np.zeros(shape, dtype), device=device).realize() for k, (shape, dtype, device) in jit['output_specs'].items()}
    device = jit['input_specs']['input_frame'][2]

    self.frame_host = np.zeros(frame_size, np.uint8)
    self.params_host = np.zeros(8, np.float32)
    self.frame_npy, self.params_npy = (Tensor(a, device='NPY')._buffer() for a in (self.frame_host, self.params_host))
    self.frame_device, self.params_device = (Tensor(a, device=device)._buffer() for a in (self.frame_host, self.params_host))
    # the JIT reads these views of the device buffers, as modeld's input_view does
    self.frame_input = Tensor(UOp.from_buffer(self.frame_device.ensure_allocated(), dtypes.uint8))
    self.params_input = Tensor(UOp.from_buffer(self.params_device.ensure_allocated(), dtypes.float32))
    self.frame_uploaded = None

  @property
  def detector_input_size(self):
    return self.input_sizes["speed_limit_us_detector"]

  @property
  def classifier_input_size(self):
    return self.input_sizes["speed_limit_us_value_classifier"]

  def receive(self, data):
    """Copy one camera buffer and wrap it; the GPU copy happens on the first model run."""
    raw = np.array(data[:self.frame_size], copy=True)
    width, height, stride, uv_offset = self.geometry
    return CameraFrame(raw, width, height, stride, uv_offset)

  def _run(self, name, frame, box, placement):
    if self.frame_uploaded is not frame:
      np.copyto(self.frame_host, frame.raw)
      self.frame_device.copy_from(self.frame_npy)
      self.frame_uploaded = frame
    self.params_host[:] = (*box, *placement)
    self.params_device.copy_from(self.params_npy)
    self.runs[name](output_buffers=self.outputs[name], input_frame=self.frame_input, params=self.params_input)
    return self.outputs[name]['output'].numpy().astype(np.float32)

  def detect(self, frame, x, y, width, height):
    """Detector predictions for one box of the frame, and the letterbox geometry to map them back."""
    ratio, pad_width, pad_height, placement = letterbox_params(width, height, self.detector_input_size)
    return self._run("speed_limit_us_detector", frame, (x, y, width, height), placement), ratio, pad_width, pad_height

  def classify(self, frame, x, y, width, height):
    placement = square_params(width, height, self.classifier_input_size)
    return self._run("speed_limit_us_value_classifier", frame, (x, y, width, height), placement).reshape(-1)

