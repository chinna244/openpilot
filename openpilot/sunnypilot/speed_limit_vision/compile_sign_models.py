"""Compile a sign vision ONNX model into a TinyJit artifact that reads the NV12 camera frame.

The JIT samples one box of the frame into the model's square input on the GPU, so the reader never converts a whole
frame on the CPU. It matches cv2: NV12 -> BGR, INTER_LINEAR resize of the box, gray (114) padding, then
blobFromImage(scalefactor=1/255, swapRB=True). Inputs: input_frame (the camera buffer) and params (float32 x8, see
sample_params).
"""
import argparse
import numpy as np
from tinygrad import Tensor, Device, TinyJit, Context, dtypes
from tinygrad.dtype import _to_np_dtype
from tinygrad.nn.onnx import OnnxRunner
from examples.openpilot.helpers import allocate_inputs, benchmark, dump_pickle, load_pickle

PAD_VALUE = 114


def nv12_box_sample(frame, stride, uv_offset, params, size):
  """frame: flat uint8 NV12. params: box x, y, w, h in the frame, then where the resized box sits in the size x size
  output: left, top, width, height. Returns (1, 3, size, size) float32 RGB in 0..1."""
  f32 = dtypes.float32
  cx, cy, cw, ch, ox, oy, rw, rh = (params[k] for k in range(8))
  j = Tensor.arange(size, dtype=f32).reshape(1, size)
  i = Tensor.arange(size, dtype=f32).reshape(size, 1)

  def axis(pos, offset, resized, length, origin):
    src = (pos - offset + 0.5) * (length / resized) - 0.5
    p0 = src.floor()
    frac = (p0 < 0).where(0., src - p0)
    p0 = p0.maximum(0.).minimum(length - 1)
    p1 = (p0 + 1).minimum(length - 1)
    # cv2's 11 bit weights
    w0, w1 = ((1. - frac) * 2048).round().cast(dtypes.int32), (frac * 2048).round().cast(dtypes.int32)
    return (p0 + origin).cast(dtypes.int32), (p1 + origin).cast(dtypes.int32), w0, w1

  x0, x1, wx0, wx1 = axis(j, ox, rw, cw, cx)
  y0, y1, wy0, wy1 = axis(i, oy, rh, ch, cy)

  def bgr(x, y):
    # cv2's fixed point NV12 -> BGR, each pixel clipped to uint8 before the resize
    luma = (frame[y * stride + x].cast(dtypes.int32) - 16).maximum(0) * 1220542 + (1 << 19)
    uv = uv_offset + (y // 2) * stride + (x // 2) * 2
    u = frame[uv].cast(dtypes.int32) - 128
    v = frame[uv + 1].cast(dtypes.int32) - 128
    def chan(c): return (c // (1 << 20)).clip(0, 255)
    return chan(luma + 1673527 * v), chan(luma - 409993 * u - 852492 * v), chan(luma + 2116026 * u)

  corners = [bgr(x0, y0), bgr(x1, y0), bgr(x0, y1), bgr(x1, y1)]
  inside = (j >= ox) & (j < ox + rw) & (i >= oy) & (i < oy + rh)
  channels = []
  for c in range(3):  # r, g, b
    # cv2's fixed point INTER_LINEAR for uint8: horizontal pass, then (((top >> 4)*wy0 >> 16) + ((bottom >> 4)*wy1 >> 16) + 2) >> 2
    top = corners[0][c] * wx0 + corners[1][c] * wx1
    bottom = corners[2][c] * wx0 + corners[3][c] * wx1
    value = ((top // 16 * wy0) // 65536 + (bottom // 16 * wy1) // 65536 + 2) // 4
    channels.append(inside.where(value.clip(0, 255).cast(f32), float(PAD_VALUE)) / 255.)
  return Tensor.stack(*channels).reshape(1, 3, size, size)


def parse_frame(value): return tuple(map(int, value.split(',')))


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('onnx')
  parser.add_argument('output')
  parser.add_argument('--frame', type=parse_frame, required=True, help='width,height,stride,y_height,uv_height,buffer_size')
  parser.add_argument('--benchmark-runs', type=int, default=5)
  args = parser.parse_args()

  width, height, stride, y_height, uv_height, frame_size = args.frame
  runner = OnnxRunner(args.onnx)
  (input_name, input_tensor), = runner.graph_inputs.items()
  size = input_tensor.shape[-1]
  assert input_tensor.shape == (1, 3, size, size), input_tensor.shape

  specs = {'input_frame': ((frame_size,), np.dtype(np.uint8).str, Device.DEFAULT),
           'params': ((8,), np.dtype(np.float32).str, Device.DEFAULT)}
  (sample_output,) = runner({input_name: Tensor.zeros(1, 3, size, size).contiguous().realize()}).values()
  # the JIT writes into buffers the caller keeps, as compile_onnx.py does: reading them back needs no new graph
  output_specs = {'output': (sample_output.shape, np.dtype(_to_np_dtype(sample_output.dtype)).name, Device.DEFAULT)}

  def make_inputs(seed):
    rng = np.random.default_rng(seed)
    def initialize(arrays):
      arrays['input_frame'][:] = rng.integers(0, 256, frame_size, dtype=np.uint8)
      w, h = rng.integers(16, 300, 2)
      arrays['params'][:] = (rng.integers(0, width - w), rng.integers(0, height - h), w, h, 0, 0, size, size)
    return allocate_inputs(specs, initialize) | {'output_buffers': allocate_inputs(output_specs)}

  @TinyJit(prune=True)
  def run(output_buffers, input_frame, params):
    # one gather kernel, as compile_warp.py does it; realized apart so it doesn't fuse into the first conv
    with Context(SPLIT_REDUCEOP=0):
      blob = nv12_box_sample(input_frame, stride, stride * y_height, params, size).realize()
    (output,) = runner({input_name: blob}).values()
    output_buffers['output'].assign(output).realize()

  expected = benchmark(run, **(inputs := make_inputs(42)))
  for _ in range(2):  # capture the jit
    np.testing.assert_equal(benchmark(run, **inputs), expected)
  with np.testing.assert_raises(AssertionError):  # its output follows its inputs
    np.testing.assert_equal(benchmark(run, **make_inputs(43)), expected)
  for k in range(args.benchmark_runs):
    np.testing.assert_equal(benchmark(run, cb=lambda t, k=k: print(f"  [{k}/{args.benchmark_runs}] {t*1e3:.2f} ms"), **inputs), expected)

  output_shape = output_specs['output'][0]
  artifact = {'metadata': {'input_size': size}, 'run': run, 'input_specs': specs, 'output_specs': output_specs}
  dump_pickle(artifact, args.output)

  loaded = load_pickle(args.output, out_of_band=True)
  np.testing.assert_equal(benchmark(loaded['run'], **make_inputs(42)), expected)
  print(f"{args.output}: input {size}, output {output_shape}")
