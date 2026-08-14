"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import time

from openpilot.cereal import log, custom

from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.car.mazda.values import MazdaSafetyFlags
from openpilot.common.params import Params
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake, read_steering_mode_param, MADS_NO_ACC_MAIN_BUTTON
from openpilot.sunnypilot.mads.state import StateMachine, GEARS_ALLOW_PAUSED_SILENT

State = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
ButtonType = structs.CarState.ButtonEvent.Type
EventName = log.OnroadEvent.EventName
EventNameSP = custom.OnroadEventSP.EventName
GearShifter = structs.CarState.GearShifter
SafetyModel = structs.CarParams.SafetyModel

SET_SPEED_BUTTONS = (ButtonType.accelCruise, ButtonType.resumeCruise, ButtonType.decelCruise, ButtonType.setCruise)
IGNORED_SAFETY_MODES = (SafetyModel.silent, SafetyModel.noOutput)

# After TJA enable, wait for a fresh pandaStates sample before treating
# lateral auth as lost. 500 ms is several pandaStates periods of margin.
# If no fresh sample arrives, disable MADS rather than remaining
# enabled-but-unauthorized (torque is already fail-closed).
MAZDA_TJA_FRESH_AUTH_TIMEOUT_S = 0.5
# pandaStates is ~10 Hz and can publish the post-TJA lateral latch one
# CarState cycle before buttonEvents. Syncing pred on that disagreeing
# health sample makes the following TJA look like a restart-diverge and
# drops the enable (route 0000001a event 1). Hold the mismatch until it
# is older than one pandaStates period (~100 ms) before treating it as a
# leftover latch.
MAZDA_TJA_PANDA_PRED_MISMATCH_HOLD = 12


class ModularAssistiveDrivingSystem:
  def __init__(self, selfdrive):
    self.CP = selfdrive.CP
    self.CP_SP = selfdrive.CP_SP
    self.params = selfdrive.params

    self.enabled = False
    self.active = False
    self.available = False
    self.lateral_mismatch_counter = 0
    self.lateral_auth_lost = False
    # After a valid TJA enable, pandaStates can still show the pre-enable
    # FALSE sample for up to one period. Zero-torque gating uses that sample
    # immediately; MADS disable must wait for a FRESH pandaStates sample.
    self._awaiting_fresh_lat_auth = False
    self._lat_auth_enable_panda_frame = -1
    self._lat_auth_enable_mono = 0.0
    self.allow_always = False
    self.no_main_cruise = False
    self.mazda_tja_physical_button_only = self.CP.brand == "mazda" and any(
      config.safetyParam & MazdaSafetyFlags.TJA for config in self.CP.safetyConfigs
    )
    # Predicted Panda lateral latch. Updated from pandaStates only on cycles with
    # no TJA edge so a same-cycle post-toggle health sample cannot look like
    # disagreement. Independent XOR from a divergent start would invert forever.
    self._panda_lat_pred = False
    self._panda_lat_pred_valid = False
    self._panda_lat_mismatch_hold = 0
    self._panda_sm_alive = True
    self._panda_was_dead = False
    self._tja_panda_resync = False
    self.selfdrive = selfdrive
    self.selfdrive.enabled_prev = False
    self.state_machine = StateMachine(self)
    self.events = self.selfdrive.events
    self.events_sp = self.selfdrive.events_sp
    self.disengage_on_accelerator = Params().get_bool("DisengageOnAccelerator")
    if self.CP.brand == "hyundai":
      if self.CP.flags & (HyundaiFlags.HAS_LDA_BUTTON | HyundaiFlags.CANFD):
        self.allow_always = True
    if self.CP.brand == "tesla":
      self.allow_always = True

    if self.CP.brand in MADS_NO_ACC_MAIN_BUTTON:
      self.no_main_cruise = True

    # read params on init
    self.enabled_toggle = self.params.get_bool("Mads")
    self.main_enabled_toggle = self.params.get_bool("MadsMainCruiseAllowed")
    self.steering_mode_on_brake = read_steering_mode_param(self.CP, self.CP_SP, self.params)
    self.unified_engagement_mode = self.params.get_bool("MadsUnifiedEngagementMode")

  def read_params(self):
    self.main_enabled_toggle = self.params.get_bool("MadsMainCruiseAllowed")
    self.unified_engagement_mode = self.params.get_bool("MadsUnifiedEngagementMode")

  def pedal_pressed_non_gas_pressed(self, CS: structs.CarState) -> bool:
    # ignore `pedalPressed` events caused by gas presses
    if self.events.has(EventName.pedalPressed) and not (CS.gasPressed and not self.selfdrive.CS_prev.gasPressed and self.disengage_on_accelerator):
      return True

    return False

  def should_silent_lkas_enable(self, CS: structs.CarState) -> bool:
    if self.steering_mode_on_brake == MadsSteeringModeOnBrake.PAUSE and (CS.brakePressed or CS.regenBraking or self.pedal_pressed_non_gas_pressed(CS)):
      return False

    if self.events_sp.contains_in_list(GEARS_ALLOW_PAUSED_SILENT):
      return False

    return True

  def block_unified_engagement_mode(self) -> bool:
    if self.mazda_tja_physical_button_only:
      return True

    # UEM disabled
    if not self.unified_engagement_mode:
      return True

    if self.enabled:
      return True

    if self.selfdrive.enabled and self.selfdrive.enabled_prev:
      return True

    return False

  def get_wrong_car_mode(self, alert_only: bool) -> None:
    if alert_only:
      if self.events.has(EventName.wrongCarMode):
        self.replace_event(EventName.wrongCarMode, EventNameSP.wrongCarModeAlertOnly)
    else:
      self.events.remove(EventName.wrongCarMode)

  def transition_paused_state(self):
    if self.state_machine.state != State.paused:
      self.events_sp.add(EventNameSP.silentLkasDisable)

  def replace_event(self, old_event: int, new_event: int):
    self.events.remove(old_event)
    self.events_sp.add(new_event)

  def _panda_alive(self) -> bool:
    sm = self.selfdrive.sm
    try:
      alive_map = getattr(sm, "alive", None)
      if isinstance(alive_map, dict) and "pandaStates" in alive_map:
        return bool(alive_map["pandaStates"])
    except Exception:
      return False
    return True

  def _panda_lat_allowed(self) -> bool | None:
    sm = self.selfdrive.sm
    try:
      panda_states = sm['pandaStates']
    except Exception:
      return None
    if not panda_states:
      return None
    relevant = [ps for ps in panda_states if ps.safetyModel not in IGNORED_SAFETY_MODES]
    if not relevant:
      return None
    return all(bool(ps.controlsAllowedLateral) for ps in relevant)

  def _poll_panda_alive_resync(self) -> None:
    if not self.mazda_tja_physical_button_only:
      return
    alive = self._panda_alive()
    if self._panda_sm_alive and not alive:
      self._panda_was_dead = True
    if self._panda_was_dead and alive:
      # Panda safety reinit: lateral latch is OFF. Match it fail-closed.
      # A TJA edge this cycle is applied against pred=False (in-sync with reset).
      self._panda_was_dead = False
      self._panda_lat_pred = False
      self._panda_lat_pred_valid = True
      self._panda_lat_mismatch_hold = 0
      self._tja_panda_resync = True
      self._awaiting_fresh_lat_auth = False
      self.lateral_auth_lost = False
      self.lateral_mismatch_counter = 0
    self._panda_sm_alive = bool(alive)

  def data_sample(self):
    # When the safety and selfdrived do not agree on controls_allowed_lateral
    # we want to disengage sunnypilot. However the status from the panda goes through
    # another socket other than the CAN messages and one can arrive earlier than the other.
    # Therefore we allow a mismatch for two samples, then we trigger the disengagement.
    self._poll_panda_alive_resync()
    sm = self.selfdrive.sm
    try:
      panda_states = sm['pandaStates']
    except Exception:
      panda_states = []
    panda_lat_denied = any(not ps.controlsAllowedLateral for ps in panda_states
                           if ps.safetyModel not in IGNORED_SAFETY_MODES)

    if self.mazda_tja_physical_button_only:
      if not self.enabled:
        self.lateral_mismatch_counter = 0
        self.lateral_auth_lost = False
        self._awaiting_fresh_lat_auth = False
        self._lat_auth_enable_panda_frame = -1
        self._lat_auth_enable_mono = 0.0
        return

      try:
        ps_frame = int(sm.recv_frame['pandaStates'])
      except Exception:
        ps_frame = -1
      fresh = ps_frame > self._lat_auth_enable_panda_frame

      if self._tja_panda_resync:
        # Reconnect cycle: update_events matches the reset latch. Do not also
        # lkasDisable via auth-lost or a same-cycle TJA cannot re-sync ON.
        pass
      elif self._awaiting_fresh_lat_auth:
        if not fresh:
          # Stale pre-enable FALSE must not spuriously lkasDisable a valid enable.
          # If pandaStates never advances, do not hang enabled forever — torque is
          # already fail-closed; clear MADS after a bounded wait for recovery.
          if (time.monotonic() - self._lat_auth_enable_mono) >= MAZDA_TJA_FRESH_AUTH_TIMEOUT_S:
            self.lateral_auth_lost = True
            self.lateral_mismatch_counter += 1
            self._awaiting_fresh_lat_auth = False
          else:
            self.lateral_auth_lost = False
            self.lateral_mismatch_counter = 0
        elif panda_lat_denied:
          self.lateral_auth_lost = True
          self.lateral_mismatch_counter += 1
          self._awaiting_fresh_lat_auth = False
        else:
          self._awaiting_fresh_lat_auth = False
          self.lateral_auth_lost = False
          self.lateral_mismatch_counter = 0
      elif panda_lat_denied:
        self.lateral_mismatch_counter += 1
        self.lateral_auth_lost = True
      else:
        self.lateral_mismatch_counter = 0
        # Do not clear lateral_auth_lost on a late fresh TRUE while still enabled.
        # Timeout / fresh-FALSE already requested lkasDisable; clearing here would
        # cancel that disable and resurrect MADS without a new TJA press.
        # Auth-lost clears only when MADS is actually disabled (branch above).
    elif not self.active or self.selfdrive.enabled:
      self.lateral_mismatch_counter = 0
      self.lateral_auth_lost = False
    elif panda_lat_denied:
      self.lateral_mismatch_counter += 1
      self.lateral_auth_lost = False
    else:
      self.lateral_auth_lost = False

  def _arm_lat_auth_freshness(self) -> None:
    """Call on MADS rising edge: ignore stale pre-enable pandaStates FALSE for disable."""
    sm = self.selfdrive.sm
    try:
      self._lat_auth_enable_panda_frame = int(sm.recv_frame['pandaStates'])
    except Exception:
      self._lat_auth_enable_panda_frame = -1
    self._lat_auth_enable_mono = time.monotonic()
    self._awaiting_fresh_lat_auth = True
    self.lateral_auth_lost = False
    self.lateral_mismatch_counter = 0

  def update_events(self, CS: structs.CarState):
    if not self.selfdrive.enabled and self.enabled:
      if CS.standstill:
        if self.events.has(EventName.doorOpen):
          self.replace_event(EventName.doorOpen, EventNameSP.silentDoorOpen)
          self.transition_paused_state()
        if self.events.has(EventName.seatbeltNotLatched):
          self.replace_event(EventName.seatbeltNotLatched, EventNameSP.silentSeatbeltNotLatched)
          self.transition_paused_state()
      if self.events.has(EventName.wrongGear) and (CS.vEgo < 2.5 or CS.gearShifter == GearShifter.reverse):
        self.replace_event(EventName.wrongGear, EventNameSP.silentWrongGear)
        self.transition_paused_state()
      if self.events.has(EventName.reverseGear):
        self.replace_event(EventName.reverseGear, EventNameSP.silentReverseGear)
        self.transition_paused_state()
      if self.events.has(EventName.brakeHold):
        self.replace_event(EventName.brakeHold, EventNameSP.silentBrakeHold)
        self.transition_paused_state()
      if self.events.has(EventName.parkBrake):
        self.replace_event(EventName.parkBrake, EventNameSP.silentParkBrake)
        self.transition_paused_state()

      if self.steering_mode_on_brake == MadsSteeringModeOnBrake.PAUSE:
        if self.pedal_pressed_non_gas_pressed(CS):
          self.transition_paused_state()

      self.events.remove(EventName.preEnableStandstill)
      self.events.remove(EventName.belowEngageSpeed)
      self.events.remove(EventName.speedTooLow)
      self.events.remove(EventName.cruiseDisabled)
      self.events.remove(EventName.manualRestart)
      self.events.remove(EventName.espActive)

    selfdrive_enable_events = self.events.has(EventName.pcmEnable) or self.events.has(EventName.buttonEnable)
    set_speed_btns_enable = any(be.type in SET_SPEED_BUTTONS for be in CS.buttonEvents)

    # wrongCarMode alert only or actively block control
    self.get_wrong_car_mode(selfdrive_enable_events or set_speed_btns_enable)

    if selfdrive_enable_events:
      if self.pedal_pressed_non_gas_pressed(CS):
        self.events_sp.add(EventNameSP.pedalPressedAlertOnly)

      if self.block_unified_engagement_mode():
        self.events.remove(EventName.pcmEnable)
        self.events.remove(EventName.buttonEnable)
    else:
      if self.main_enabled_toggle and not self.mazda_tja_physical_button_only:
        if CS.cruiseState.available and not self.selfdrive.CS_prev.cruiseState.available:
          self.events_sp.add(EventNameSP.lkasEnable)

    for be in CS.buttonEvents:
      if be.type == ButtonType.cancel:
        if not self.selfdrive.enabled and self.selfdrive.enabled_prev:
          self.events_sp.add(EventNameSP.manualLongitudinalRequired)
      if be.type == ButtonType.lkas and be.pressed and (
          CS.cruiseState.available or self.allow_always or self.mazda_tja_physical_button_only):
        if self.mazda_tja_physical_button_only:
          # Sequential apply: each physical rising edge is one toggle, including
          # multiple edges in one CarState. Net event drives the state machine.
          continue
        if self.enabled:
          if self.selfdrive.enabled:
            self.events_sp.add(EventNameSP.manualSteeringRequired)
          else:
            self.events_sp.add(EventNameSP.lkasDisable)
        else:
          self.events_sp.add(EventNameSP.lkasEnable)

    if self.mazda_tja_physical_button_only:
      tja_presses = sum(1 for be in CS.buttonEvents if be.type == ButtonType.lkas and be.pressed)
      panda_now = self._panda_lat_allowed()
      resync = self._tja_panda_resync
      if resync:
        self._tja_panda_resync = False

      if tja_presses == 0:
        if panda_now is not None:
          if panda_now == self.enabled:
            self._panda_lat_pred = panda_now
            self._panda_lat_pred_valid = True
            self._panda_lat_mismatch_hold = 0
          else:
            # Health moved without a userspace edge. Could be this TJA's
            # pandaStates beating CarState, or a leftover latch. Only the
            # leftover is a real diverge; wait one pandaStates period.
            self._panda_lat_mismatch_hold += 1
            if self._panda_lat_mismatch_hold >= MAZDA_TJA_PANDA_PRED_MISMATCH_HOLD:
              self._panda_lat_pred = panda_now
              self._panda_lat_pred_valid = True
        if resync and self.enabled:
          # Panda reset to OFF and no physical edge this cycle: drop the latch.
          self.events_sp.add(EventNameSP.lkasDisable)
      else:
        self._panda_lat_mismatch_hold = 0
        if not self._panda_lat_pred_valid:
          if panda_now is None:
            tja_presses = 0
          else:
            self._panda_lat_pred = panda_now
            self._panda_lat_pred_valid = True
        if tja_presses:
          want_enabled = self.enabled
          for _ in range(tja_presses):
            # In-sync: XOR both. Divergent: skip userspace; Panda XOR converges.
            if want_enabled == self._panda_lat_pred:
              want_enabled = not want_enabled
            self._panda_lat_pred = not self._panda_lat_pred
          if want_enabled and not self.enabled:
            self.events_sp.add(EventNameSP.lkasEnable)
          elif (not want_enabled) and self.enabled:
            if self.selfdrive.enabled:
              self.events_sp.add(EventNameSP.manualSteeringRequired)
            else:
              self.events_sp.add(EventNameSP.lkasDisable)

    if not CS.cruiseState.available and not self.no_main_cruise:
      self.events.remove(EventName.buttonEnable)
      if self.selfdrive.CS_prev.cruiseState.available and not self.mazda_tja_physical_button_only:
        self.events_sp.add(EventNameSP.lkasDisable)

    if self.steering_mode_on_brake == MadsSteeringModeOnBrake.DISENGAGE:
      if self.pedal_pressed_non_gas_pressed(CS):
        if self.enabled:
          self.events_sp.add(EventNameSP.lkasDisable)
        else:
          # block lkasEnable if being sent, then send pedalPressedAlertOnly event
          if self.events_sp.contains(EventNameSP.lkasEnable):
            self.events_sp.remove(EventNameSP.lkasEnable)
            self.events_sp.add(EventNameSP.pedalPressedAlertOnly)

    if self.should_silent_lkas_enable(CS):
      if self.state_machine.state == State.paused:
        self.events_sp.add(EventNameSP.silentLkasEnable)

    if self.mazda_tja_physical_button_only and self.lateral_auth_lost and self.enabled:
      # Same-cycle path: disable MADS immediately; do not wait for controlsMismatchLateral@200.
      # Zero torque is enforced in get_lat_active when panda lat is false.
      if not self.events_sp.contains(EventNameSP.lkasDisable):
        self.events_sp.add(EventNameSP.lkasDisable)
    elif self.lateral_mismatch_counter >= 200:
      self.events_sp.add(EventNameSP.controlsMismatchLateral)

    self.events.remove(EventName.pcmDisable)
    self.events.remove(EventName.buttonCancel)
    self.events.remove(EventName.pedalPressed)
    self.events.remove(EventName.wrongCruiseMode)

  def update(self, CS: structs.CarState):
    if not self.enabled_toggle:
      return

    self.data_sample()

    self.update_events(CS)

    if not self.CP.passive and (self.selfdrive.initialized or self.mazda_tja_physical_button_only):
      was_enabled = self.enabled
      self.enabled, self.active = self.state_machine.update()
      if self.mazda_tja_physical_button_only:
        # Physical TJA toggles logical enabled even when NO_ENTRY (Park,
        # wrong gear, invalid LKAS, initializing) blocks actuation. paused
        # is enabled but not active; latActive/torque stay gated.
        if self.events_sp.has(EventNameSP.lkasEnable) and self.state_machine.state == State.disabled:
          self.state_machine.state = State.paused
          self.enabled, self.active = True, False
        if self.enabled and not was_enabled:
          self._arm_lat_auth_freshness()

    # Copy of previous SelfdriveD states for MADS events handling
    self.selfdrive.enabled_prev = self.selfdrive.enabled
