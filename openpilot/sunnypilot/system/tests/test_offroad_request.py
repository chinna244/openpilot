"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.sunnypilot.system.offroad_request import STANDSTILL_T, STANDSTILL_V, StandstillGate

RATE = 10.0
STOP_FRAMES = int(STANDSTILL_T * RATE)


def _run(gate, n, v_ego):
  stopped = False
  for _ in range(n):
    stopped = gate.update(v_ego)
  return stopped


class TestStandstillGate:
  def test_needs_the_full_debounce(self):
    g = StandstillGate(RATE)
    for _ in range(STOP_FRAMES - 1):
      assert not g.update(0.0)
    assert g.update(0.0)
    assert g.stopped

  def test_threshold(self):
    g = StandstillGate(RATE)
    assert _run(g, STOP_FRAMES, v_ego=STANDSTILL_V / 2)
    assert not _run(g, STOP_FRAMES, v_ego=STANDSTILL_V)

  def test_motion_resets(self):
    g = StandstillGate(RATE)
    _run(g, STOP_FRAMES - 1, v_ego=0.0)
    g.update(5.0)
    assert not _run(g, STOP_FRAMES - 1, v_ego=0.0)

  def test_at_least_one_frame(self):
    assert StandstillGate(0.1).frames_needed == 1

