"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Standstill gate shared by the two consumers that end an onroad session on request:
card's AlphaLongToggleMonitor (toggle cycle and force-offroad grant) and hardwared's
fallback grant of OffroadModeRequested.

The UIs only refuse both actions while engaged, so a toggle flip or a force-offroad
press while rolling used to end the session at speed: 8 of the alpha-long routes on
file finish with the ordered radar hand-back and the log ending 0.1-0.4 s later at
1-19 m/s, and in two of them the device stayed offroad for the rest of the drive.
Nothing about either flow needs the transition to happen while moving, so the request
is held (hand-back already done, radar stock) until the car has been stopped for
STANDSTILL_T and taken there.
"""

STANDSTILL_V = 0.1   # m/s, below this the car counts as stopped
STANDSTILL_T = 0.5   # s the car has to stay below STANDSTILL_V before a request is acted on

# Seconds hardwared gives card's stock-ECU hand-back before it grants a force-offroad
# request itself. Past card's own HANDBACK_TIMEOUT_T so the ordered path normally wins.
OFFROAD_REQUEST_TIMEOUT = 10.


class StandstillGate:
  """Debounced 'stopped for STANDSTILL_T' at a fixed update rate."""

  def __init__(self, rate_hz: float):
    self.frames_needed = max(1, int(round(STANDSTILL_T * rate_hz)))
    self.stopped_frames = 0

  def update(self, v_ego: float) -> bool:
    if v_ego < STANDSTILL_V:
      self.stopped_frames = min(self.stopped_frames + 1, self.frames_needed)
    else:
      self.stopped_frames = 0
    return self.stopped_frames >= self.frames_needed

  @property
  def stopped(self) -> bool:
    return self.stopped_frames >= self.frames_needed


class OffroadRequestGate:
  """hardwared's side of OffroadModeRequested.

  Grants at once when there is no onroad session to hand back from. With a session it
  waits OFFROAD_REQUEST_TIMEOUT for card, then grants only disengaged and stopped, so
  the fallback cannot undo card's gate by firing while the car is still moving. A dead
  carState counts as stopped: the request must never silently fail, and without card
  there is nothing rolling that openpilot could report.
  """

  def __init__(self, rate_hz: float):
    self.timeout_frames = int(round(OFFROAD_REQUEST_TIMEOUT * rate_hz))
    self.standstill = StandstillGate(rate_hz)
    self.request_frames = 0

  def update(self, requested: bool, session_active: bool, engaged: bool, v_ego: float | None) -> bool:
    stopped = self.standstill.update(0.0 if v_ego is None else v_ego)
    if not requested:
      self.request_frames = 0
      return False
    if not session_active:
      return True
    self.request_frames += 1
    return self.request_frames >= self.timeout_frames and not engaged and stopped
