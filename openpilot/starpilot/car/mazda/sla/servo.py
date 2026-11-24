"""filter-dev's ICBM servo (ce9225e178 intelligent_cruise_button_management/controller.py) for Mazda alpha long.

Under openpilot longitudinal the planner executes curve and lead targets itself; the servo only keeps the factory SET
on the plan target, which here is the setpoint the SLA session owns capped by the session's plan cap. The reference's
stock-ACC parts (decel overshoot, lookahead dips) never run on openpilot longitudinal and are left out; the state
machine, timers, deadband, driver-press grace and fast mode are the reference's.
"""
from openpilot.common.realtime import DT_CTRL

INACTIVE_TIMER = 0.4
DRIVER_PRESS_GRACE_T = 3.0
DRIVER_PRESS_GRACE_FRAMES = int(DRIVER_PRESS_GRACE_T / DT_CTRL)
REACT_DEADBAND = 2
REACT_TIMER = 0.3
RESTORE_QUIET_TIME = 1.0
RESTORE_QUIET_FRAMES = int(RESTORE_QUIET_TIME / DT_CTRL)
FAST_MODE_MIN = 3
FAST_STALL_T = 1.5
DECEL_NEEDS_STABLE_SETPOINT = True  # Mazda actuation profile

CRUISE_BUTTONS = ('accelCruise', 'decelCruise', 'setCruise', 'resumeCruise', 'cancel', 'mainCruise')


class MazdaSetServo:
  def __init__(self):
    self.fast_faulted = False
    self.reset()

  def reset(self) -> None:
    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.state = 'inactive'
    self.pre_active_timer = 0
    self.restore_quiet_timer = 0
    self.v_target_raw = 0
    self.v_target_raw_prev = 0
    self.setpoint_unset = False
    self.react_deadband = REACT_DEADBAND
    self.down_grace_timer = 0
    self.up_grace_timer = 0
    self.press_target = 0
    self.is_ready = False
    self.is_ready_prev = False
    self.prompting = False
    self.limiter_active = False
    self.fast_active = False
    self.fast_stall_frames = 0
    self.fast_last_cluster = 0
    self.timers = dict.fromkeys(CRUISE_BUTTONS, 0)

  def update_restore_quiet_timer(self) -> None:
    up_error = self.v_target_raw - self.v_cruise_cluster
    if up_error >= self.react_deadband and self.v_target_raw == self.v_target_raw_prev:
      self.restore_quiet_timer += 1
    else:
      self.restore_quiet_timer = 0

  def plan_fast_mode(self) -> None:
    remaining = abs(self.v_target - self.v_cruise_cluster)
    use_fast = not self.fast_faulted and remaining >= FAST_MODE_MIN
    if use_fast and not self.fast_active:
      self.fast_active = True
      self.fast_stall_frames = 0
      self.fast_last_cluster = self.v_cruise_cluster
    elif self.fast_active:
      if remaining < FAST_MODE_MIN:
        self.fast_active = False
      elif self.v_cruise_cluster != self.fast_last_cluster or self.prompting:
        self.fast_last_cluster = self.v_cruise_cluster
        self.fast_stall_frames = 0
      else:
        self.fast_stall_frames += 1
        if self.fast_stall_frames * DT_CTRL > FAST_STALL_T:
          self.fast_faulted = True
          self.fast_active = False

  @property
  def down_parked(self) -> bool:
    return self.down_grace_timer > 0 and self.v_target_raw >= self.press_target - self.react_deadband

  def update_state_machine(self) -> str:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)
    self.update_restore_quiet_timer()
    if self.state != 'inactive':
      if not self.is_ready:
        self.state = 'inactive'
      else:
        # no lookahead or overshoot on openpilot longitudinal
        up_allowed = not DECEL_NEEDS_STABLE_SETPOINT or self.restore_quiet_timer >= RESTORE_QUIET_FRAMES
        up_allowed = up_allowed and self.up_grace_timer <= 0 and not self.setpoint_unset
        down_allowed = not self.down_parked
        if self.state == 'preActive':
          if self.pre_active_timer <= 0:
            if self.v_target - self.v_cruise_cluster >= self.react_deadband and up_allowed:
              self.state = 'increasing'
            elif self.v_cruise_cluster - self.v_target >= self.react_deadband \
                 and self.v_cruise_cluster > self.v_cruise_min and down_allowed:
              self.state = 'decreasing'
            else:
              self.state = 'holding'
        elif self.state == 'holding':
          down_pending = self.v_cruise_cluster - self.v_target >= self.react_deadband and down_allowed
          up_pending = self.v_target - self.v_cruise_cluster >= self.react_deadband
          if down_pending or (up_pending and up_allowed):
            self.pre_active_timer = int(REACT_TIMER / DT_CTRL)
            self.state = 'preActive'
        elif self.state == 'increasing':
          if self.v_target <= self.v_cruise_cluster or self.setpoint_unset:
            self.state = 'holding'
        elif self.state == 'decreasing':
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = 'holding'
    elif self.state == 'inactive':
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = 'preActive'

    if self.state in ('increasing', 'decreasing'):
      self.plan_fast_mode()
      return 'increase' if self.state == 'increasing' else 'decrease'
    self.fast_active = False
    return 'none'

  def update(self, *, buttons, enabled: bool, override: bool, cancel: bool, resume: bool, v_target_conv: int,
             v_target_raw_conv: int, cluster_conv: int, setpoint_unset: bool, limiter_active: bool, prompting: bool,
             v_cruise_min_conv: int = 0) -> str:
    """One card frame. buttons: [(name, pressed)]. Speeds in display units (v_cruise_min_conv: the ECU's lowest SET, never
    walked below). Returns 'none'/'increase'/'decrease'."""
    self.prompting = prompting
    self.limiter_active = limiter_active
    self.v_target = v_target_conv
    self.setpoint_unset = setpoint_unset
    self.v_target_raw_prev = self.v_target_raw
    self.v_target_raw = v_target_raw_conv
    self.v_cruise_cluster = cluster_conv
    self.v_cruise_min = v_cruise_min_conv
    self.react_deadband = REACT_DEADBAND if limiter_active else 1

    # update_readiness: cruise_ext.update_manual_button_timers, then the grace windows
    for k in self.timers:
      if self.timers[k] > 0:
        self.timers[k] += 1
    for name, pressed in buttons:
      if name in self.timers:
        self.timers[name] = 1 if pressed else 0
    ready = enabled and not override and not cancel and not resume
    button_pressed = any(v > 0 for v in self.timers.values())
    if self.timers['accelCruise'] > 0:
      self.down_grace_timer = DRIVER_PRESS_GRACE_FRAMES
      self.press_target = self.v_target_raw
      self.up_grace_timer = 0
    elif self.timers['decelCruise'] > 0:
      self.up_grace_timer = DRIVER_PRESS_GRACE_FRAMES
      self.down_grace_timer = 0
    else:
      self.down_grace_timer = max(0, self.down_grace_timer - 1)
      self.up_grace_timer = max(0, self.up_grace_timer - 1)
    self.is_ready = ready and not button_pressed

    button = self.update_state_machine()
    self.is_ready_prev = self.is_ready
    return button
