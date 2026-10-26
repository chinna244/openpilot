#!/usr/bin/env python3
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
J_CRUISE_BP = [0., 10.0, 25., 40.]
A_CRUISE_MIN = -0.5
# SET pressed while gaining speed sets the speed rounded down, so the car is up to a mph or two over it at once; for
# ENGAGE_SETTLE_TIME after engaging (or after a gas held through SET), while that close over a plain set speed that has not
# been lowered, the cruise candidate lifts off instead of braking, then eases to what the set speed asks at its jerk limit
# (routes 00000152-00000186: 7 of 82 rolling engagements braked -0.2 to -0.5 for 0.6-1.6 s while the model asked more,
# route 00000184 t=1125: -0.43 at 0.9 mph over with the model at +0.53). Only the candidate is lifted, as for the gas
# handover: the cruise state and everything that reads it are unchanged.
ENGAGE_SETTLE_BAND = 1.0  # m/s over the set speed
ENGAGE_SETTLE_ACCEL = -0.15  # m/s^2
ENGAGE_SETTLE_TIME = 5.0  # s
# Lifting off the gas over the set speed, the cruise candidate went to A_CRUISE_MIN in 0.8 s and held it to the set speed:
# at 60-75 mph -0.86 at a jerk of 1.22 where this driver's own lift-off coasts at -0.42 and 0.61 (routes
# 00000160-0000018f; route 0000018b t=2689: 84 mph, set 77, -0.50 from 0.8 s on with the model at +0.1). Released
# GAS_COAST_BAND or more over the set speed, the candidate coasts instead: to GAS_COAST_ACCEL at GAS_COAST_JERK until the
# car is back at the set speed (or it is lowered), then eases to what the set speed asks at the cruise jerk limit. Like
# the settle it lifts only the candidate: leads, the model, curves and limits brake as before.
GAS_COAST_BAND = 0.5  # m/s over the set speed
GAS_COAST_ACCEL = -0.3  # m/s^2
GAS_COAST_JERK = 0.3  # m/s^3
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]


def get_cruise_accel(v_cruise, v_ego, a_cruise_prev, dt):
  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, ACCEL_MAX)
  j_cruise = np.interp(v_ego, J_CRUISE_BP, J_CRUISE_VALS)
  target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner(LongitudinalPlannerSP):
  def __init__(self, CP, CP_SP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    LongitudinalPlannerSP.__init__(self, self.CP, CP_SP, self.mpc)
    self.fcw = False
    self.dt = dt
    self.accel_boost = AccelBoost()

    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = init_a
    self.gas_override = False  # the last reset was the driver's gas while engaged, no brake
    self.gas_return = False  # since that handover, until the cruise state is back at its target's ask
    self.disengaged = False  # a reset while disengaged since the last control cycle: the next one is an engagement
    self.engage_settle_frames = 0  # what is left of the settle window after engaging
    self.engage_v_cruise = 0.  # the set speed the window opened on: lowering it ends the window
    self.a_settle = None  # the cruise candidate the settle lifted last cycle, while it eases back to the state
    self.gas_coast = False  # coasting down to the set speed after a gas release over it
    self.gas_coast_v_cruise = 0.  # the set speed the coast started on: lowering it ends the coast
    self.gas_v_cruise = 0.  # the set speed while the gas was held: one lowered at the release is braked to as before
    self.a_coast = 0.  # the coast's own ramp, lifted onto the candidate while above it
    self.a_target_published = init_a  # the plan sent: the internal one, lifted while a gas override is held
    self.source_published = LongitudinalPlanSource.cruise
    self.output_a_target = init_a
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def update(self, sm):
    LongitudinalPlannerSP.update(self, sm)

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    if sm['controlsState'].forceDecel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.output_a_target = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)
      self.a_cruise = self.output_a_target

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # Get new v_cruise and a_target from Smart Cruise Control and Speed Limit Assist
    v_cruise, self.output_a_target = LongitudinalPlannerSP.update_targets(self, sm, self.v_desired_filter.x, self.output_a_target, v_cruise)

    # A gas override resets the cruise state to the measured accel, which in a turn or over a bump reads as braking nobody
    # asked for, and after the release it is the lowest candidate until it climbs back (route 00000177 t=183: -0.41 sent
    # while the model asked +0.24). The state keeps its own path; the cruise candidate is held at what its target asks this
    # cycle in the plan published while the gas is held (the first cycle after the release still uses it) and from the
    # handover until the state is back there: a target that turns restrictive at any point gets at least what it asks.
    # longControlState can trail the pedal by a cycle, so the gas is latched for the whole override.
    # An engagement starts from the same measured accel: engaging while slowing braked 0.2-0.3 s at -0.27 to -0.48 under a
    # set speed well above (routes 00000169-00000180, 5 of 82), so it hands over the same way.
    if reset_state:
      if not sm['selfdriveState'].enabled or sm['carState'].brakePressed:
        self.gas_override = False
      elif sm['carState'].gasPressed:
        self.gas_override = True
        self.gas_v_cruise = v_cruise
      self.gas_return = False
      # kept through the engaged cycles longControlState trails by and a gas held through SET; a brake ends it
      if not sm['selfdriveState'].enabled:
        self.disengaged = True
      elif sm['carState'].brakePressed:
        self.disengaged = False
      self.engage_settle_frames = 0
      self.a_settle = None
      self.gas_coast = False
    elif self.gas_override or self.disengaged:
      if self.disengaged:
        self.engage_settle_frames = int(ENGAGE_SETTLE_TIME / self.dt)
        self.engage_v_cruise = v_cruise
      elif v_ego - v_cruise >= GAS_COAST_BAND and v_cruise >= self.gas_v_cruise:
        self.gas_coast, self.gas_coast_v_cruise, self.a_coast = True, v_cruise, min(self.a_cruise, 0.)
      self.gas_override, self.disengaged, self.gas_return = False, False, True

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    if self.CP.openpilotLongitudinalControl:
      self.mpc.set_cur_state(self.v_desired_filter.x, self.output_a_target)
    else:
      self.mpc.set_cur_state(v_ego, sm['carState'].aEgo)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.output_a_target

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = should_stop(v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop
    output_a_target_e2e, output_should_stop_e2e = LongitudinalPlannerSP.update_e2e_target(self, sm, output_a_target_e2e,
                                                                                          output_should_stop_e2e, reset_state)

    self.a_cruise = get_cruise_accel(v_cruise, v_ego, self.a_cruise, self.dt)
    self.a_cruise = LongitudinalPlannerSP.update_cruise_accel(self, self.a_cruise)
    a_cruise = self.a_cruise
    if self.gas_override or self.gas_return:
      # what the target asks this cycle: the cruise accel without its jerk limit (SCC's budget included), never above 0
      cruise_ask = min(LongitudinalPlannerSP.update_cruise_accel(self, get_cruise_accel(v_cruise, v_ego, 0., 1e3), unlimited=True), 0.)
      if self.gas_return and self.a_cruise >= cruise_ask:
        self.gas_return = False
      else:
        a_cruise = max(self.a_cruise, cruise_ask)
    if self.engage_settle_frames > 0:
      self.engage_settle_frames -= 1
      # once under the set speed, further over it than the band or with it lowered, the window is over: the lift only ever
      # starts on engaging, never as a step mid-way
      if v_ego <= v_cruise or v_ego - v_cruise > ENGAGE_SETTLE_BAND or v_cruise < self.engage_v_cruise:
        self.engage_settle_frames = 0
    if self.gas_coast and (self.source != LongitudinalPlanSource.cruise or v_ego <= v_cruise or v_cruise < self.gas_coast_v_cruise or
                           sm['controlsState'].forceDecel):
      # back at the set speed: ease on from where the coast is; a forced decel takes the state at once
      self.gas_coast = False
      self.a_settle = None if sm['controlsState'].forceDecel else self.a_coast
    settle = None
    if self.source != LongitudinalPlanSource.cruise:
      # SCC and SLA targets brake as they ask, from the cruise state
      self.engage_settle_frames = 0
    elif self.engage_settle_frames > 0:
      settle = ENGAGE_SETTLE_ACCEL
    elif self.gas_coast:
      j = float(np.interp(v_ego, J_CRUISE_BP, J_CRUISE_VALS)) * self.dt
      self.a_coast = float(np.clip(GAS_COAST_ACCEL, self.a_coast - GAS_COAST_JERK * self.dt, self.a_coast + j))
      settle = self.a_coast
    elif self.a_settle is not None:
      # toward what the set speed asks now (never above 0) at the cruise jerk limit: down while still over it, up once
      # under it (a raised set speed), until the state is back over the candidate
      j = float(np.interp(v_ego, J_CRUISE_BP, J_CRUISE_VALS)) * self.dt
      settle = float(np.clip(min(get_cruise_accel(v_cruise, v_ego, 0., 1e3), 0.), self.a_settle - j, self.a_settle + j))
    if settle is not None and settle > a_cruise:
      a_cruise = settle
      self.a_settle = a_cruise
    else:
      self.a_settle = None
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    output_a_target_e2e = self.accel_boost.update(sm, output_a_target_e2e, output_a_target_mpc, self.a_cruise)

    # while the gas is held only the published plan is lifted: the MPC and the filter restart from the plain one
    lift_published = reset_state and a_cruise > self.a_cruise
    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise if lift_published else a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop),
                  (output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e)]

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(should_stop for _, _, should_stop in candidates)
    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)
    self.a_target_published, self.source_published = float(self.output_a_target), self.mpc.source
    if lift_published:
      published = [candidates[0], (a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)] + candidates[2:]
      a_published, self.source_published, _ = min(published, key=lambda c: c[0])
      self.a_target_published = float(np.clip(a_published, ACCEL_MIN, ACCEL_MAX))

    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.source_published
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.a_target_published)
    longitudinalPlan.accelBoost = float(self.accel_boost.total_boost)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True

    pm.send('longitudinalPlan', plan_send)

    self.publish_longitudinal_plan_sp(sm, pm)
