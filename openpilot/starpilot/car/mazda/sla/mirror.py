"""filter-dev's SpeedLimitAssistMirror (ce9225e178 assist_mirror.py) in plannerd: the card session's plan cap and alert state.

The session runs in card at 100 Hz (session.py) and arrives here in starpilotCarState.mazdaSlaSession. The mirror
ramps a lower cap into the plan at CAP_FALL_RATE (the published cap, output_v_target, stays the session's own), keeps
the assist state for the prompt alert, and counts announce bumps for the "Adjusting to" alert. selfdrived raises the
alerts from what this publishes in slcState.mazdaSla, one plannerd tick after card moved, as filter-dev's events do.
"""
from openpilot.common.realtime import DT_MDL
from openpilot.starpilot.car.mazda.sla import ACTIVE_STATES, ENABLED_STATES, V_CRUISE_UNSET, AssistState

CAP_FALL_RATE = 0.35  # m/s^2


class SpeedLimitAssistMirror:
  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.state = AssistState.disabled
    self.output_v_target = V_CRUISE_UNSET
    self._v_ramp: float | None = None
    self.plan_v_target = V_CRUISE_UNSET
    self.announce_counter = 0
    self.setpoint_kph = V_CRUISE_UNSET
    self.offer_target = 0.
    self.session_valid = False

  @property
  def is_enabled(self) -> bool:
    return self.state in ENABLED_STATES

  @property
  def is_active(self) -> bool:
    return self.state in ACTIVE_STATES

  def update(self, session, v_ego: float) -> None:
    """session: starpilotCarState.mazdaSlaSession (state, vCap, announceCounter, setpointKph), or None before card publishes."""
    if session is None:
      self.reset()
      return
    self.session_valid = True
    self.state = AssistState(int(session.state))
    self.announce_counter = int(session.announceCounter)
    self.setpoint_kph = float(session.setpointKph)
    self.offer_target = float(session.target)
    # the arbiter publishes vCap as a target, a frozen hold, or V_CRUISE_UNSET, never 0; a 0 is capnp's default
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
