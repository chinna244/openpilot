import math

import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants

# Under FIRM_STOP_SPEED with the model planning a stop, finish it at a steady decel to FIRM_STOP_MARGIN short of the model's
# stop point instead of the plan's taper. Clean engaged stops (routes 00000008-0000003c, 87 op / 118 driver): the last 3 mph
# took op 1.8 s and the driver 1.0 s; from 5 mph op 3.0 s, driver 1.6-1.9 s. At 5 mph the model's stop point was within
# +-0.7 m (p10-p90) of where op stopped. Only ever a candidate in the planner's minimum: it adds braking, never removes it.
FIRM_STOP_SPEED = 2.24  # m/s, 5 mph
FIRM_STOP_RELEASE_SPEED = 2.7  # m/s, once finishing a stop it lets go only above this
FIRM_STOP_MARGIN = 0.5  # m
FIRM_STOP_MIN_DIST = 0.3  # m
# behind a lead it stops this far back (radarState dRel), not at the model's stop point: at 5 mph the model plans its stop
# 2-3 m ahead with the lead 8-9 m ahead (routes 0000004a-0000004c), so firm stops ended 6-7.6 m back; the driver stops 3.9 m
FIRM_STOP_LEAD_GAP = 4.0  # m
FIRM_STOP_PLAN_V = 0.3  # m/s, plan speed that counts as stopped; under it longcontrol's stopping state finishes
FIRM_STOP_GO_V = 0.5  # m/s, a plan this fast within FIRM_STOP_STAY of its stop point stops and goes: not a stop
FIRM_STOP_STAY = 2.0  # s
FIRM_STOP_ARM_FRAMES = 3  # consecutive model frames planning a stop
FIRM_STOP_SETTLE_FRAMES = int(1.0 / DT_MDL)  # after a reset (gas release, engagement) the car is the driver's for a second
FIRM_STOP_JERK = 2.0  # m/s^3, into the brake from the candidate it starts under
# eased into standstill: at 1.5 the car settles harder than the driver's own stops
FIRM_STOP_DECEL_BP = [0.3, 1.0]  # m/s
FIRM_STOP_DECEL_MAX = [0.8, 1.5]  # m/s^2
T_IDXS = np.asarray(ModelConstants.T_IDXS)


def plan_stop_distance(vel, pos) -> float:
  """Distance to the first point of the model's plan slower than FIRM_STOP_PLAN_V that it stays stopped after, inf if it
  plans none. A plan that starts that slow (pulling away, its first point is the current speed) plans no stop ahead."""
  vel, pos = np.asarray(vel, dtype=float), np.asarray(pos, dtype=float)
  if len(vel) < 2 or len(vel) != len(pos) or not (np.isfinite(vel).all() and np.isfinite(pos).all()) or vel[0] < FIRM_STOP_PLAN_V:
    return math.inf
  stop = np.flatnonzero(vel < FIRM_STOP_PLAN_V)
  if not len(stop):
    return math.inf
  i = stop[0]
  if len(vel) == len(T_IDXS) and (vel[i:][T_IDXS[i:] <= T_IDXS[i] + FIRM_STOP_STAY] >= FIRM_STOP_GO_V).any():
    return math.inf
  return float(pos[i] - pos[0])


class FirmStop:
  def __init__(self, dt: float = DT_MDL):
    self.dt = dt
    self.frames = 0  # consecutive frames planning a stop
    self.active_frames = 0  # consecutive frames in control
    self.finishing = False  # armed on a planned stop, until it lets go
    self.a_target = math.inf
    self.should_stop = False

  def update(self, v_ego: float, vel, pos, active: bool, a_candidate: float, lead_d: float = math.inf) -> float:
    """The firm-stop candidate in m/s^2, inf while it has nothing to ask. a_candidate: the candidate it brakes under, the
    starting point of its ramp. lead_d: distance to the lead, inf without one. should_stop: finishing under FIRM_STOP_PLAN_V,
    for longcontrol's stopping state."""
    dist = plan_stop_distance(vel, pos)
    self.active_frames = self.active_frames + 1 if active else 0
    planned = math.isfinite(dist)
    self.frames = self.frames + 1 if planned and FIRM_STOP_PLAN_V <= v_ego < FIRM_STOP_SPEED else 0

    if not active or v_ego >= FIRM_STOP_RELEASE_SPEED or not math.isfinite(v_ego):
      self.finishing = False
    elif self.finishing and v_ego >= FIRM_STOP_PLAN_V and not planned:
      self.finishing = False  # the plan goes on: its stop is gone
    elif self.finishing and v_ego < FIRM_STOP_PLAN_V and np.asarray(vel, dtype=float)[T_IDXS[:len(vel)] <= FIRM_STOP_STAY].max(initial=0.) >= FIRM_STOP_GO_V:
      self.finishing = False  # under the stopping speed its first point is the current speed: let go once it plans to go
    elif not self.finishing and self.frames >= FIRM_STOP_ARM_FRAMES and self.active_frames >= FIRM_STOP_SETTLE_FRAMES:
      self.finishing = True
      self.a_target = min(a_candidate, 0.)

    self.should_stop = self.finishing and v_ego < FIRM_STOP_PLAN_V
    if not self.finishing:
      self.a_target = math.inf
    elif v_ego >= FIRM_STOP_PLAN_V:
      cap = float(np.interp(v_ego, FIRM_STOP_DECEL_BP, FIRM_STOP_DECEL_MAX))
      stop_at = max(dist - FIRM_STOP_MARGIN, lead_d - FIRM_STOP_LEAD_GAP if math.isfinite(lead_d) else -math.inf)
      want = -min(v_ego ** 2 / (2. * max(stop_at, FIRM_STOP_MIN_DIST)), cap) if planned else self.a_target
      self.a_target = max(want, self.a_target - FIRM_STOP_JERK * self.dt) if want < self.a_target else want
    return self.a_target
