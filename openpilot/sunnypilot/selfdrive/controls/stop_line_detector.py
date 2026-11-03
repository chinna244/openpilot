#!/usr/bin/env python3
"""
Publishes the bands across the lane the road camera sees ahead (stop_line.py) as customReserved11, for stopping at a
stop line (stop_spots.py). Every other frame (10 Hz), from the Y plane only: a row average over a patch of road, no
model and no GPU. Nothing is sent before the camera is calibrated.
"""
import os
import time

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.controls.lib.stop_line import line_distances

SERVICE = "customReserved11"
FRAME_SKIP = 2


def y_plane(client, buffer) -> np.ndarray | None:
  data = np.frombuffer(buffer.data, dtype=np.uint8)  # this fork's msgq hands out a memoryview
  width, height, stride = client.width, client.height, client.stride
  if data.size < stride * height:
    return None
  return data[:stride * height].reshape((height, stride))[:, :width]


def main():
  from msgq.visionipc import VisionIpcClient
  from openpilot.cereal.visionipc import VisionStreamType

  os.nice(5)
  pm = messaging.PubMaster([SERVICE])
  sm = messaging.SubMaster(['extrinsicsCalibration'])
  client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_NARROW_ROAD, True)
  while not client.connect(False):
    time.sleep(0.1)
  cloudlog.info("stop_line_detector: connected")
  while True:
    buffer = client.recv()
    if buffer is None:
      continue
    sm.update(0)
    frame_id = buffer.frame_id
    if frame_id % FRAME_SKIP:
      continue
    cal = sm['extrinsicsCalibration']
    dists: list[float] = []
    if cal.calStatus == log.ExtrinsicsCalibration.Status.calibrated and len(cal.height) and len(cal.rpyCalib) == 3:
      y = y_plane(client, buffer)
      if y is not None:
        dists = line_distances(y, float(cal.height[0]), float(cal.rpyCalib[1]))
    msg = messaging.new_message(SERVICE)
    msg.customReserved11.lineDistances = dists
    msg.customReserved11.frameId = frame_id
    pm.send(SERVICE, msg)


if __name__ == "__main__":
  main()
