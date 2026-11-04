"""
Model stop line: wherever the model brings the car to a stop, the stop goes before the first line painted across the
lane, not where the model would put it.

Learned stop spots (stop_spots.py) only cover stop signs the car has stopped at before, and neither the car's TSR (8 of
25 stops the driver braked for) nor the map (about 3 of 153 junctions tagged here) knows the rest. The model stops for
every stop sign and red light it brakes for, on any road, but where it stops is its own guess: route 00000030
northbound it stopped on the crosswalk. So while the model's plan reaches a stop (MODEL_STOP_SPEED) with no lead, the
first firm band (stop_line.py) from GO_DIST before that stop to STOP_MARGIN past it is the line: a stop line, or a
crosswalk's near edge where there is none. The stop aims STOP_MARGIN short of it, so it only ever moves the stop
earlier than the model's; with no band seen, the model's own stop stands. A stop the model does not brake for at all
is the learned spots' job.

Seen within LOCK_MAX_DIST only, the margin shrinks to what START_MAX makes (RESTEP_EDGE at least); a stop
needing more than BRAKE_MAX even so is the model's. Once placed, the distance is the car's own odometry and the
band is tracked as for a spot (LINE_TRACK, LINE_DRIFT), and a nearer band seen late moves it earlier, never later. The
car holds at the line for as long as the model still stops within GO_DIST past it; once the model goes (a green light,
a clear junction) for GO_TIME, it lets go. The driver's gas or brake, disengaging, or turning off before the line hand
the stop back to the model until it plans to go again.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.stop_spots import (BRAKE_MAX, HOLD_ACCEL, HOLD_DIST, LEAD_BAND, LEAD_HOLD,
                                                                    LEAVE_STEER, LINE_DRIFT, LINE_GAIN, LINE_LATENCY, LINE_TRACK,
                                                                    LOCK_JERK, LOCK_JERK_TIME, LOCK_MAX_DIST, RESTEP_EDGE,
                                                                    RESTEP_MIN_AIM, RESUME_MAX, START_JERK, START_MAX, STOP_MARGIN,
                                                                    STOP_SPEED, STRAIGHT_STEER, carry_bands, firm_band, see_band)

MODEL_STOP_SPEED = 0.5  # m/s the model's planned speed falls below where it stops
# m past the line the model may still stop and be stopping for it: its own stop on the crosswalk is 2-3 m past the
# line, the crosswalk's far edge about 4-5 m; further on (or no stop at all) is the model going
GO_DIST = 8.
GO_TIME = 0.5  # s the model plans to go before the stop lets go: one odd plan is not a green light
# m/s2 the model's own accel must be over for a plan with no stop to be a go while still rolling: route 0000002d
# t=316.8-318.5 the plan stayed over MODEL_STOP_SPEED for 1.75 s mid-approach with the model asking -2.4 to -2.5, and
# the stop let go and was placed again later, on the crosswalk (review 2026-11-03). The model's accel lags a launch
# (route 0000002d t=15.0-15.6 and t=386.0-388.4: -0.5 to -0.7 for 0.65-2.4 s with the plan going), so a plan faster at
# GO_PLAN_T than the car is a go too: through route 0000002d t=316.6-318.3 it stayed 0.8-3.9 m/s under the car's 4.4-8.4
GO_ACCEL = -0.3
GO_PLAN_T = 2.  # s
# s after the driver's gas a new stop may need at most RESUME_MAX, as for a stop spot: released close to a line, the
# full BRAKE_MAX ask came on top of the model's own
OVERRIDE_TIME = 2.
# s the model plans a stop before one is placed: on route 0000002c 21 of 33 moving model stops with no lead lasted 3
# frames or less (13 one), and a stop placed on one frame braked to -2.2 for GO_TIME, then let go in a step (review
# 2026-11-03)
STOP_TIME = 0.3
STOP_HOLD = 2.  # s a stopped car's plan stays under MODEL_STOP_SPEED while the model holds: a launch is over it by then


def model_stop_dist(velocity: list[float], position: list[float], t_idxs: list[float] = ModelConstants.T_IDXS) -> float | None:
  """m ahead where the model's plan comes to a stop, or None when it does not. The plan starts at the car's own speed:
  stopped, the model stops only while its plan stays under MODEL_STOP_SPEED for STOP_HOLD (a launch is over it)."""
  v = np.asarray(velocity, dtype=float)
  if not len(v) or len(position) != len(v) or len(t_idxs) != len(v):
    return None
  if v[0] < MODEL_STOP_SPEED:
    if np.all(v[np.asarray(t_idxs) <= STOP_HOLD] < MODEL_STOP_SPEED):
      return float(position[0])
    # moving off only to stop again (a creep up to where the model would stop, on the crosswalk): that stop, not a go
    v_from = v[np.flatnonzero(v >= MODEL_STOP_SPEED)[0]:]
    again = np.flatnonzero(v_from < MODEL_STOP_SPEED)
    return float(position[len(v) - len(v_from) + again[0]]) if len(again) else None
  below = np.flatnonzero(v < MODEL_STOP_SPEED)
  return float(position[below[0]]) if len(below) else None


class ModelStopLine:
  def __init__(self):
    self.bands: list[list[float]] = []  # the camera's bands (stop_spots.carry_bands)
    self.locked = False  # a stop is placed at a band
    self.done = False  # the model's stop is left to the model until it plans to go
    self.dist = 0.  # m to where the stop aims (STOP_MARGIN short of the band), odometry-carried
    self.anchor = 0.  # m to where it aimed when placed, odometry-carried
    self.lock_age = 0.
    self.go_time = 0.  # s the model has planned to go
    self.stop_frames = 0  # cycles in a row the model has planned a stop
    self.override_age = OVERRIDE_TIME  # s since the driver's gas or brake
    self.lead_dist: float | None = None
    self.lead_age = 0.
    self.a_target = 0.
    self.should_stop = False

  def _end(self, done: bool) -> None:
    self.locked, self.done, self.a_target = False, done, 0.

  def _see(self, line_dists: list[float], v_ego: float, steer_deg: float, lead: float | None) -> None:
    if abs(steer_deg) > STRAIGHT_STEER:
      return  # turning in, the camera sees the road at an angle
    for d in (d - v_ego * LINE_LATENCY for d in line_dists):
      if d > LOCK_MAX_DIST or (lead is not None and d >= LEAD_BAND * lead):
        continue
      see_band(self.bands, d)

  def update(self, v_ego: float, standstill: bool, active: bool, driver_override: bool, steer_deg: float,
             model_stop: float | None, line_dists: list[float] | None, lead_dist: float | None,
             model_accel: float = 0., model_v: float | None = None) -> float | None:
    """The stop's accel candidate this cycle, or None. model_stop: m ahead where the model's plan stops (None: it does
    not); line_dists: the camera's bands this cycle (m ahead, nearest first), None without a new frame; model_accel:
    the model's own accel ask (modelV2.action.desiredAcceleration); model_v: its plan's speed GO_PLAN_T on."""
    self.should_stop = False
    self.dist -= v_ego * DT_MDL
    self.anchor -= v_ego * DT_MDL
    self.lock_age += DT_MDL
    self.bands = carry_bands(self.bands, v_ego)
    if lead_dist is not None:
      self.lead_dist, self.lead_age = lead_dist, 0.
    else:
      self.lead_age += DT_MDL
    lead = self.lead_dist if self.lead_dist is not None and self.lead_age <= LEAD_HOLD else None
    if line_dists is not None:
      self._see(line_dists, v_ego, steer_deg, lead)

    line = self.dist + STOP_MARGIN
    stopping = model_stop is not None and (not self.locked or model_stop <= line + GO_DIST)
    going = not stopping and (v_ego < STOP_SPEED or model_accel > GO_ACCEL or (model_v is not None and model_v > v_ego))
    self.go_time = self.go_time + DT_MDL if going else 0.
    self.override_age = 0. if driver_override or not active else self.override_age + DT_MDL
    self.stop_frames = self.stop_frames + 1 if model_stop is not None else 0
    if self.go_time >= GO_TIME:
      self._end(False)  # the model goes: the stop is over
      return None
    if self.done:
      return None
    if not active or driver_override:
      self._end(self.locked)  # the driver's, or disengaged
      return None

    firm = [b[0] for b in self.bands if firm_band(b)]
    if not self.locked:
      if not stopping or self.stop_frames < round(STOP_TIME / DT_MDL) or lead is not None or v_ego < STOP_SPEED:
        return None  # behind a lead the lead MPC places the stop
      # from GO_DIST before the model's stop: a line further back would read as the model going once stopped at it
      near = [d for d in firm if d > STOP_MARGIN and model_stop - GO_DIST <= d < model_stop + STOP_MARGIN]
      if not near:
        return None
      # the first band: what is painted after it is past the stop. The camera reads it from LOCK_MAX_DIST only, so the
      # margin shrinks to what START_MAX makes, down to RESTEP_EDGE short of the band
      seen = min(near)
      aim = max(seen - STOP_MARGIN, min(v_ego ** 2 / (2. * START_MAX), seen - RESTEP_EDGE))
      if aim >= model_stop:
        return None  # the model stops short of it anyway
      if aim < RESTEP_MIN_AIM or v_ego ** 2 / (2. * aim) > (RESUME_MAX if self.override_age < OVERRIDE_TIME else BRAKE_MAX):
        self._end(True)  # too close to stop before it: the model's
        return None
      self.locked, self.dist, self.anchor, self.lock_age, self.a_target = True, aim, aim, 0., 0.
    else:
      if abs(steer_deg) > LEAVE_STEER and self.dist > HOLD_DIST:
        self._end(True)  # turned off before the line: not this stop
        return None
      near = [d for d in firm if d - line < STOP_MARGIN and model_stop is not None and d >= model_stop - GO_DIST]
      if near:
        seen = min(near)
        if abs(seen - line) <= LINE_TRACK:
          self.dist = min(max(self.dist + LINE_GAIN * (seen - line), self.anchor - LINE_DRIFT), self.anchor + LINE_DRIFT)
        elif seen < line:
          # a band before the line, seen late (worn paint): only to an aim BRAKE_MAX makes, never further on
          aim = max(seen - STOP_MARGIN, min(v_ego ** 2 / (2. * START_MAX), seen - RESTEP_EDGE))
          if RESTEP_MIN_AIM <= aim < self.dist and v_ego ** 2 / (2. * aim) <= BRAKE_MAX:
            self.dist = self.anchor = aim
            self.lock_age = 0.
      if self.dist < -HOLD_DIST:
        self._end(True)  # well past it: the model's
        return None

    if v_ego < STOP_SPEED:
      if self.dist > HOLD_DIST:
        self.a_target = 0.
        return None  # the model's own stop, short of the line: a creep on is stopped at it
      # at the line: hold while the model still stops
      self.should_stop = True
      # to HOLD_ACCEL at START_JERK from either side: a stop braked harder than it eases off, no step
      self.a_target += min(max(HOLD_ACCEL - self.a_target, -START_JERK * DT_MDL), START_JERK * DT_MDL)
      return self.a_target
    need = v_ego ** 2 / (2. * max(self.dist, 0.5))
    if self.dist <= 0.5:
      need = max(need, -HOLD_ACCEL)  # at or past the aim still rolling (a car short of what was asked): stop now
    jerk = LOCK_JERK if self.lock_age < LOCK_JERK_TIME else START_JERK
    self.a_target = max(-min(need, BRAKE_MAX), self.a_target - jerk * DT_MDL)
    return self.a_target
