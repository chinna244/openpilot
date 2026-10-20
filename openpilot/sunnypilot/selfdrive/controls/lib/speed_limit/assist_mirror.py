"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Plannerd-side mirror of the card cruise arbiter's SLA session (non-pcm cars).

The session machine runs in card at 100 Hz, next to the buttons and the setpoint.
Plannerd needs three things from it: the plan cap for min() source selection, the
assist state for the UI wire, and the alert events. The mirror reads
carStateSP.zoompilot.cruiseSession and reproduces the surface SpeedLimitAssist used to provide
here, so longitudinalPlanSP consumers are unchanged.

Events: speedLimitPreActive is level-driven (the prompt alert persists for the whole
window); speedLimitActive fires on announce-counter deltas, which card bumps at 100 Hz
and never un-bumps, so a 20 Hz reader cannot miss one.
"""
from openpilot.cereal import custom
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import get_planning_limits, publish_cap_decel
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import ACTIVE_STATES, ENABLED_STATES, V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventNameSP = custom.OnroadEventSP.EventName
SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

# past the sign (distance 0) the decel degrades to a pull over the control horizon,
# matching SpeedLimitAssist's active-state form
_T_ACTIVE = float(ModelConstants.T_IDXS[CONTROL_N])

# A lower cap reaches the plan as a ramp, not a step: the cruise MPC answers a step of a few mph with up to its
# full -1.2 m/s^2 within 1.5 s (route 00000163 776.3 s: 55 -> 50 mph, 63 -> 58 mph target). The ramp starts from
# the current speed, so a cap far below it still brakes at once and a cap above the car's speed is unchanged.
# Only the plan sees the ramp (plan_v_target): the published cap (output_v_target) stays the session's own, which ICBM
# walks the dash to and the alerts announce.
# Local tuning (zoompilot 0.6): this driver lifting off at 15-25 mph slows at a median 0.26 m/s^2 (routes 00000097-0000016e),
# and pressed the gas in 10 of 12 lower-zone slowdowns at 0.6. Model, lead and curve braking still win the plan min().
CAP_FALL_RATE = 0.35  # m/s^2


class SpeedLimitAssistMirror:
  def __init__(self, CP, CP_SP):
    self.limits = get_planning_limits(CP)
    self.session = None  # carStateSP.zoompilot.cruiseSession, set by the planner before each update
    self.reset()

  def reset(self) -> None:
    """Fresh session state: at start, and when the planner hands the session to card's arbiter
    (ICBM went active), which happens only while disengaged."""
    self.state = SessionState.disabled
    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.
    self._a_out = 0.
    self._v_ramp: float | None = None  # the plan cap last published, walking down toward a lower cap
    self.plan_v_target = V_CRUISE_UNSET
    self._announce_seen: int | None = None  # sync on first update (plannerd restarts)

  @property
  def is_enabled(self) -> bool:
    return self.state in ENABLED_STATES

  @property
  def is_active(self) -> bool:
    return self.state in ACTIVE_STATES

  def update_buttons(self, release_toggle: int) -> None:
    # upstream's plannerd hook: presses are classified by the cruise arbiter in card, whose
    # session this class mirrors, so there is nothing to do here
    pass

  def update(self, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, v_cruise_cluster: float, speed_limit: float,
             speed_limit_final_last: float, has_speed_limit: bool, distance: float, events_sp: EventsSP) -> None:
    # SpeedLimitAssist's signature, so the planner calls either owner the same way; the limit
    # inputs go unused, card's arbiter already applied them to the session
    session = self.session
    self.state = session.state
    # the arbiter publishes vCap as a target, a frozen hold, or V_CRUISE_UNSET, never 0; a 0
    # is capnp's default from a not-yet-received carStateSP and would win the plan min()
    v_cap = float(session.vCap)
    if v_cap > 0.0:
      v_out = v_cap
      if self._v_ramp is not None and v_cap < self._v_ramp:
        v_out = max(v_cap, min(self._v_ramp, max(v_ego, v_cap)) - CAP_FALL_RATE * DT_MDL)
      self._v_ramp = v_out
      self.output_v_target = v_cap
      self.plan_v_target = v_out
    else:
      self._v_ramp = None
      self.output_v_target = V_CRUISE_UNSET
      self.plan_v_target = V_CRUISE_UNSET

    # the decel actually required to arrive at the cap (it keys ICBM's overshoot gap on
    # stock ACC); computed here because the resolver's distance lives in plannerd
    if self.is_active and 0.0 < v_cap < v_ego:
      self._a_out = publish_cap_decel(v_cap, distance, _T_ACTIVE, self._a_out, self.limits, v_ego)
    else:
      self._a_out = a_ego
    self.output_a_target = self._a_out

    if self.state == SessionState.preActive:
      events_sp.add(EventNameSP.speedLimitPreActive)

    announce = int(session.announceCounter)
    if self._announce_seen is not None and announce != self._announce_seen:
      events_sp.add(EventNameSP.speedLimitActive)
    self._announce_seen = announce
