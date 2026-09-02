"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

from openpilot.cereal import log

# Hold ignition ON this long after CAN ignition drops while the ignition line is still high.
# panda drops ignitionCan 2 s after the last ignition frame, so a bus gap shorter than this
# rides through on the line instead of taking the device offroad mid-drive. A real key-off
# reaches offroad 5 s later: the line alone would take ~30 s (it stays high that long on
# Mazda after key-off, which is why line-only was abandoned), and the old permanent
# "CAN seen once, ignore the line forever" latch went offroad immediately on any gap.
IGNITION_CAN_DROP_HOLD_S = 5.0

# Monotonic time CAN ignition was last seen on a valid panda; None until it has been.
# Process-local and never latched: each caller (hardwared, manager, ui) keeps its own and
# they agree to within a frame because the window is wall time.
_ignition_can_last_seen: float | None = None


def get_ignition_state(panda_states, now: float | None = None) -> bool:
  global _ignition_can_last_seen
  if now is None:
    now = time.monotonic()

  valid = [ps for ps in panda_states if ps.pandaType != log.PandaState.PandaType.unknown]
  if not valid:
    _ignition_can_last_seen = None
    return False

  if any(ps.ignitionCan for ps in valid):
    _ignition_can_last_seen = now
    return True

  if not any(ps.ignitionLine for ps in valid):
    return False

  # line high, CAN silent: a line-only car follows the line; a car that has shown CAN
  # ignition gets the hold window and is then treated as off
  if _ignition_can_last_seen is None:
    return True
  return (now - _ignition_can_last_seen) < IGNITION_CAN_DROP_HOLD_S
