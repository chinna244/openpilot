"""The few OpenCV image steps the sign reader runs on the CPU, in numpy, matching cv2 on uint8 images."""
import numpy as np

HSV_SHIFT = 12
_SDIV = np.array([0] + [round((255 << HSV_SHIFT) / i) for i in range(1, 256)], dtype=np.int32)
_HDIV = np.array([0] + [round((180 << HSV_SHIFT) / (6 * i)) for i in range(1, 256)], dtype=np.int32)
_LUMA = (np.maximum(np.arange(256) - 16, 0) * 1220542 + (1 << 19)).astype(np.int32)


def nv12_to_bgr(y, uv):
  """cv2.COLOR_YUV2BGR_NV12 of a region: y (h, w), uv (h/2, w) interleaved, h and w even."""
  h, w = y.shape
  u = uv[:, 0::2].astype(np.int32) - 128
  v = uv[:, 1::2].astype(np.int32) - 128
  luma = _LUMA[y].reshape(h // 2, 2, w // 2, 2)
  out = np.empty((h // 2, 2, w // 2, 2, 3), np.int32)
  out[..., 0] = luma + (2116026 * u)[:, None, :, None]
  out[..., 1] = luma + (-409993 * u - 852492 * v)[:, None, :, None]
  out[..., 2] = luma + (1673527 * v)[:, None, :, None]
  out >>= 20
  return np.clip(out, 0, 255).astype(np.uint8).reshape(h, w, 3)


def bgr_to_hsv(image):
  """cv2.COLOR_BGR2HSV for uint8: hue 0..179, saturation and value 0..255."""
  b, g, r = (image[..., k].astype(np.int32) for k in range(3))
  v = np.maximum(np.maximum(b, g), r)
  diff = v - np.minimum(np.minimum(b, g), r)
  s = (diff * _SDIV[v] + (1 << (HSV_SHIFT - 1))) >> HSV_SHIFT
  h = np.where(v == r, g - b, np.where(v == g, b - r + 2 * diff, r - g + 4 * diff))
  h = (h * _HDIV[diff] + (1 << (HSV_SHIFT - 1))) >> HSV_SHIFT
  h = np.where(h < 0, h + 180, h)
  return np.stack((h, s, v), axis=-1).astype(np.uint8)


def _components(mask):
  """8-connected components of a 0/1 mask as lists of runs (row, start, end)."""
  runs, parent = [], []
  def find(a):
    while parent[a] != a:
      parent[a] = parent[parent[a]]
      a = parent[a]
    return a
  previous = []
  for row in range(mask.shape[0]):
    edges = np.flatnonzero(np.diff(np.concatenate(([0], mask[row].astype(np.int8), [0]))))
    current = []
    k = 0
    for start, end in zip(edges[0::2].tolist(), edges[1::2].tolist(), strict=True):
      idx = len(runs)
      runs.append((row, start, end))
      parent.append(idx)
      while k < len(previous) and runs[previous[k]][2] < start:
        k += 1
      m = k
      while m < len(previous) and runs[previous[m]][1] <= end:
        a, b = find(idx), find(previous[m])
        if a != b:
          parent[max(a, b)] = min(a, b)
        m += 1
      if m > k and runs[previous[m - 1]][2] > end:
        k = m - 1
      current.append(idx)
    previous = current
  groups = {}
  for idx in range(len(runs)):
    groups.setdefault(find(idx), []).append(runs[idx])
  return list(groups.values())


# Moore neighbourhood, clockwise in image coordinates starting east
_DIRS = ((0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1))


def _trace(padded, row, col):
  """Outer border of the component whose top-left pixel is (row, col), as pixel centres."""
  points = [(row, col)]
  r, c, back = row, col, 4  # we entered from the west
  start_state = None
  while True:
    for k in range(1, 9):
      d = (back + k) % 8
      nr, nc = r + _DIRS[d][0], c + _DIRS[d][1]
      if padded[nr, nc]:
        break
    else:
      return points  # isolated pixel
    state = (r, c, d)
    if start_state is None:
      start_state = state
    elif state == start_state:
      return points[:-1]
    r, c, back = nr, nc, (d + 4) % 8
    points.append((r, c))


def _inside(points, row, col):
  inside = False
  n = len(points)
  for k in range(n):
    r1, c1 = points[k]
    r2, c2 = points[(k + 1) % n]
    if (r1 > row) != (r2 > row) and col < c1 + (row - r1) * (c2 - c1) / (r2 - r1):
      inside = not inside
  return inside


def external_contours(mask, min_box_area=0):
  """cv2.findContours(RETR_EXTERNAL) + contourArea + boundingRect of a 0/1 mask, as (area, x, y, w, h). Skips
  components whose bounding box is under min_box_area: their contour area is smaller still."""
  padded = np.pad(mask.astype(bool), 1)
  candidates = []
  for runs in _components(mask):
    top = min(r for r, _, _ in runs)
    x1, x2 = min(s for _, s, _ in runs), max(e for _, _, e in runs)
    y2 = max(r for r, _, _ in runs) + 1
    if (x2 - x1) * (y2 - top) < min_box_area:
      continue
    start = min(s for r, s, _ in runs if r == top)
    candidates.append((top, start, x1, x2, y2))
  candidates.sort()

  results, outlines = [], []
  for top, start, x1, x2, y2 in candidates:
    if any(ox1 <= start < ox2 and oy1 <= top < oy2 and _inside(points, top + 1, start + 1)
           for (ox1, oy1, ox2, oy2), points in outlines):
      continue  # inside another contour's hole
    points = _trace(padded, top + 1, start + 1)
    rows = np.array([p[0] for p in points], np.float64)
    cols = np.array([p[1] for p in points], np.float64)
    area = abs(float(np.dot(cols, np.roll(rows, -1)) - np.dot(rows, np.roll(cols, -1)))) / 2.
    outlines.append(((x1, top, x2, y2), points))
    results.append((area, x1, top, x2 - x1, y2 - top))
  return results
