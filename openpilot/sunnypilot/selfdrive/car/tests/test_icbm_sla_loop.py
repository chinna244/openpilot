"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Closed-loop integration tests for the ICBM + SLA + driver-setpoint stack.

Unit tests validate each layer alone; the bugs this stack has actually shipped were all
INTERACTIONS: the confirm press tearing down its own session through the cluster guard,
the deadband stranding the restore the servo itself created, the dash re-sync adopting a
limiter-held dash. So this harness wires the real production classes together
(VCruiseHelper in card, SpeedLimitAssist in plannerd at 20 Hz, the ICBM servo in
selfdrived) against a simulated Mazda body ECU with the measured imperfections:

- taps register at most every 200 ms, and ~7% are dropped (seeded, deterministic)
- a sustained hold snaps to the next 5 mph multiple after ~0.6 s, then every ~0.55 s,
  with a trailing extra step if released mid-cycle
- a registered press takes ~60 ms to change the dash

It also models the two DIFFERENT cluster views the real system has: SLA and the planner
see openpilot's own vCruiseCluster (= v_cruise on ICBM cars), while the servo and the
reconciler see the car's real dash from CAN. Conflating those two is exactly the class of
bug this file exists to catch.
"""
import random

from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.assist_mirror import SpeedLimitAssistMirror
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
SlaState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
PlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
EventNameSP = custom.OnroadEventSP.EventName

MPH_MS = CV.MPH_TO_MS
MPH_KPH = CV.MPH_TO_KPH

TAP_REGISTER_S = 0.2
TAP_DROP_RATE = 0.07
TAP_LATENCY_S = 0.06
HOLD_FIRST_STEP_S = 0.6
HOLD_STEP_PERIOD_S = 0.55
HOLD_TRAILING_RATE = 0.15


class FakeMazdaEcu:
  """The body ECU's cruise set-speed integrator, driven at 100 Hz.

  A genuine physical hold (the wheel) grid-snaps 5 mph. Synthesized streams never do:
  measured across all 116 recorded routes, every servo-driven dash step was 1 mph (the
  wheel's own button-up frames interleave with the forged ones). forged_mode:
    'taps':    the measured reality: forged frames register as paced discrete presses
    'ignored': rejects them outright: zero movement (must trip the tap fallback)
  """

  def __init__(self, dash_mph, seed=0, forged_mode='taps'):
    self.dash = dash_mph
    self.rng = random.Random(seed)
    self.forged_mode = forged_mode
    self.t = 0.
    self.last_tap_t = -1.
    self.hold_dir = 0
    self.hold_t = 0.
    self.hold_steps = 0
    self.pending = []  # (apply_t, delta or 'snap'±1)

  def _register_tap(self, direction):
    if self.t - self.last_tap_t < TAP_REGISTER_S:
      return
    self.last_tap_t = self.t
    if self.rng.random() < TAP_DROP_RATE:
      return
    self.pending.append((self.t + TAP_LATENCY_S, direction))

  def _snap(self, direction):

  def tick(self, tap_dir=0, hold_dir=0, forged_hold_dir=0):
    """tap_dir: paced presses (driver taps or servo taps). hold_dir: the driver's
    PHYSICAL hold (grid-snaps). forged_hold_dir: the servo's synthesized stream
    (registers per forged_mode). -1/0/+1 for this 10 ms tick; returns dash (mph)."""
    self.t += DT_CTRL

    if tap_dir != 0:
      self._register_tap(tap_dir)

    if forged_hold_dir != 0 and self.forged_mode == 'taps':
      self._register_tap(forged_hold_dir)

    if hold_dir != 0:
      if self.hold_dir != hold_dir:
        self.hold_dir, self.hold_t, self.hold_steps = hold_dir, 0., 0
      self.hold_t += DT_CTRL
      due = HOLD_FIRST_STEP_S + self.hold_steps * HOLD_STEP_PERIOD_S
      if self.hold_t >= due:
        self.pending.append((self.t + TAP_LATENCY_S, 'snap' if self.hold_steps == 0 else 5 * hold_dir))
        self.hold_steps += 1
    else:
      if self.hold_dir != 0 and self.hold_steps > 0 and self.rng.random() < HOLD_TRAILING_RATE:
        self.pending.append((self.t + TAP_LATENCY_S, 5 * self.hold_dir))
      self.hold_dir, self.hold_t, self.hold_steps = 0, 0., 0

    due = [d for at, d in self.pending if self.t >= at]
    self.pending = [(at, d) for at, d in self.pending if self.t < at]
    for delta in due:
      if delta == 'snap':
        self.dash += self._snap(1 if self.hold_dir >= 0 else -1) if self.hold_dir else 0
      else:
        self.dash += delta
      self.dash = max(20, min(90, self.dash))
    return self.dash


class Loop:
  """100 Hz co-simulation of card (arbiter inside) + plannerd mirror (20 Hz) +
  selfdrived + the fake ECU.

  The SLA session machine now runs inside card's VCruiseHelper (the cruise arbiter),
  synchronous with the buttons: that hop has genuinely zero latency in production, so
  the harness models it that way. The plannerd->card (longitudinalPlanSP) and
  selfdrived->card (carControlSP) hops keep their one-cycle transport delay, and the
  mirror consumes the session snapshot from the previous frame, as plannerd would."""

  def __init__(self, baseline_mph=60, seed=0, forged_mode='taps'):
    params = Params()
    params.put("IsReleaseSpBranch", True, block=True)
    params.put("SpeedLimitMode", int(Mode.assist), block=True)
    params.put_bool("IsMetric", False, block=True)
    params.put_bool("CustomAccIncrementsEnabled", False)

    CP = car.CarParams(pcmCruise=True, brand="mazda")
    CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
    self.helper = VCruiseHelper(CP, CP_SP)
    self.sla = self.helper.cruise_arbiter  # 100 Hz session truth; .state as before
    self.mirror = SpeedLimitAssistMirror(CP, CP_SP)  # plannerd side: plan cap + events
    self.servo = IntelligentCruiseButtonManagement(CP, CP_SP)
    self.ecu = FakeMazdaEcu(baseline_mph, seed=seed, forged_mode=forged_mode)
    self.lookahead_mph = None
    self.events_sp = EventsSP()

    self.tick_n = 0
    self.limit_mph = 0.
    self.scc_dip_mph = 0.  # SCC-vision target when active, 0 = inactive
    self.v_ego_mph = None
    self._driver_active = None  # (button, remaining_ticks)
      self._card_tick(enabled=False)
    for _ in range(5):
      self._card_tick(enabled=True)

  def _cs(self, button_events=None):
    CS = car.CarState(cruiseState={"available": True,
                                   "speed": self.ecu.dash * MPH_MS,
    v_ego_mph = self.v_ego_mph if self.v_ego_mph is not None else self.helper.v_cruise_kph / MPH_KPH
    CS.buttonEvents = button_events or []
    return CS

  def _lp_sp(self):
    LP_SP = custom.LongitudinalPlanSP()
               PlanSource.speedLimitAssist: self.mirror.output_v_target}
    if self.scc_dip_mph > 0:
      targets[PlanSource.sccVision] = self.scc_dip_mph * MPH_MS
    source = min(targets, key=lambda k: targets[k])
    LP_SP.longitudinalPlanSource = source
      LP_SP.smartCruiseControl.vision.vAheadMin = float(self.lookahead_mph * MPH_MS)
    LP_SP.aTarget = float(self.a_target)
    LP_SP.speedLimit.assist.state = self.mirror.state
    LP_SP.speedLimit.resolver.speedLimit = self.limit_mph * MPH_MS
    LP_SP.speedLimit.resolver.speedLimitFinalLast = self.limit_mph * MPH_MS
    LP_SP.speedLimit.resolver.speedLimitLastValid = self.limit_mph > 0
    CC_SP.intelligentCruiseButtonManagement.state = self.servo.state
    return CC_SP

  def _card_tick(self, CS=None, enabled=True, lp_msg=None, cc_msg=None):
    CS = CS if CS is not None else self._cs()
    self.helper.update_speed_limit_assist(False, lp_msg or self._lp_sp(), cc_msg or self._cc_sp())
    self.helper.update_v_cruise(CS, enabled=enabled, is_metric=False)
    for _ in range(int(seconds / DT_CTRL)):
      self.tick_n += 1

      lp_msg = self._lp_sp()

      events = []
      if self.tick_n in self.driver_queue:
        button, hold_ticks = self.driver_queue.pop(self.tick_n)
        self._driver_active = [button, hold_ticks, hold_ticks]
        events.append(ButtonEvent(type=button, pressed=True))
      driver_tap_dir = 0
      driver_hold_dir = 0
      if self._driver_active is not None:
          events.append(ButtonEvent(type=button, pressed=False))
          if total <= 30:

      CS = self._cs(events)

      if self.tick_n % 5 == 0:
        session_msg = custom.CarStateSP.new_message()
        self.helper.cruise_arbiter.fill_msg(session_msg)
        self.events_sp.clear()
        v_ego_mph = self.v_ego_mph if self.v_ego_mph is not None else self.helper.v_cruise_kph / MPH_KPH
                     session_state=session_state_stale)

      self._card_tick(CS, lp_msg=lp_msg, cc_msg=cc_msg)
      tap_dir, forged_hold_dir = driver_tap_dir, 0
      if self.helper.cruise_arbiter.prompting:
        sb = SendButtonState.none
      elif sb == SendButtonState.decrease:
        tap_dir = tap_dir or -1
      elif sb == SendButtonState.increaseHold:
        forged_hold_dir = 1
      elif sb == SendButtonState.decreaseHold:
        forged_hold_dir = -1
      self.ecu.tick(tap_dir=tap_dir, hold_dir=driver_hold_dir, forged_hold_dir=forged_hold_dir)

  def driver_press(self, button, in_seconds, hold_s=0.15):
    self.driver_queue[self.tick_n + int(in_seconds / DT_CTRL)] = (button, max(1, int(hold_s / DT_CTRL)))

  @property


    to exactly the driver's baseline, across ECU press drops and grid snaps."""
    loop.scc_dip_mph = 55
    loop.run(6.0)
    loop.scc_dip_mph = 0.
    loop.run(12.0)  # quiet window + restore move + latency
    assert loop.ecu.dash == 60, f"restore not exact: dash={loop.ecu.dash}"
    assert loop.v_cruise_mph == 60, f"baseline corrupted: {loop.v_cruise_mph}"

  def test_dip_train_does_not_churn(self):
    """Back-to-back dips with the horizon in view: the lookahead veto must hold the
    dash down through the gap (the second dip is visible before its source commits)."""
    loop.scc_dip_mph = 55
    dash_after_first = loop.ecu.dash

    loop.scc_dip_mph = 0.
    loop.run(1.5)  # gap between commits; the dip is still on the horizon
    loop.run(3.0)
    loop.scc_dip_mph = 0.


class TestSlaSession:
  def _confirm_lower(self, loop, limit):
    loop.limit_mph = limit
    loop.run(2.0)  # disabled->preActive engagement path
    assert loop.sla.state == SlaState.preActive, loop.sla.state
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(1.0)
    assert loop.sla.state == SlaState.active, loop.sla.state

  def test_confirm_sticks_and_dash_reaches_limit(self):
    """F1 end-to-end: one - press confirms; SLA must stay active while ICBM walks the
    dash all the way to the limit (hold + taps), and the baseline must survive."""
    loop = Loop(baseline_mph=60, seed=3)
    self._confirm_lower(loop, limit=45)

    def watch(lo):
      states.add(lo.sla.state)
    loop.run(10.0, assert_each=watch)
    assert loop.ecu.dash == 45, f"dash never reached the limit: {loop.ecu.dash}"
    assert states == {SlaState.active}, f"SLA flickered: {states}"
    assert loop.v_cruise_mph == 60, f"baseline corrupted: {loop.v_cruise_mph}"

  def test_settled_press_reanchors(self):
    """Settled at the limit, one + press: SLA steps aside, the ECU's +1 becomes the new
    setpoint, and the servo must NOT drag the dash back to the old baseline."""
    loop = Loop(baseline_mph=60, seed=4)
    self._confirm_lower(loop, limit=45)
    loop.run(10.0)
    assert loop.ecu.dash == 45

    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)
    loop.run(5.0)
    assert loop.sla.state == SlaState.inactive, loop.sla.state
    assert loop.ecu.dash == 46, f"dash: {loop.ecu.dash}"
    assert round(loop.v_cruise_mph) == 46, f"setpoint must re-anchor to 46: {loop.v_cruise_mph}"

  def test_mid_move_abort_restores_baseline(self):
    """+ while ICBM is still walking down: session aborts and the servo restores the
    exact baseline; the driver is never stranded mid-way (the upstream failure mode)."""
    loop = Loop(baseline_mph=60, seed=5)
    self._confirm_lower(loop, limit=45)

    loop.run(1.2)  # servo mid-move, dash somewhere between 60 and 45
    assert 45 < loop.ecu.dash < 60, loop.ecu.dash
    loop.driver_press(ButtonType.accelCruise, in_seconds=0.05)
    loop.run(14.0)  # abort + quiet window + restore
    assert loop.sla.state == SlaState.inactive
    assert loop.ecu.dash == 60, f"baseline not restored: {loop.ecu.dash}"
    assert loop.v_cruise_mph == 60, f"setpoint corrupted: {loop.v_cruise_mph}"

  def test_holds_read_as_taps_still_reaches_limit(self):
    """An ECU that registers synthesized holds as paced presses: same net progress as
    taps and no fault needed; the session still lands the limit."""
    loop = Loop(baseline_mph=60, seed=6)
    self._confirm_lower(loop, limit=45)

    loop.run(15.0)
    assert loop.ecu.dash == 45, f"dash never landed: {loop.ecu.dash}"
    assert loop.sla.state == SlaState.active

  def test_holds_ignored_faults_and_taps_land(self):
    """An ECU that rejects synthesized holds outright: zero movement must trip the
    long-press fallback, and the session still lands the limit on taps."""
    loop = Loop(baseline_mph=60, seed=7, forged_mode='ignored')
    self._confirm_lower(loop, limit=45)

    loop.run(15.0)
    assert loop.servo.fast_faulted
    assert loop.ecu.dash == 45, f"taps fallback never landed: {loop.ecu.dash}"
    assert loop.sla.state == SlaState.active


class TestPressTimingSweeps:
  """Every shipped bug in this stack was a single driver press racing the 20 Hz SLA
  cycle, the reconcile window, or the servo state. These sweeps land the same press at
  offsets spanning more than one full SLA cycle and assert the outcome INVARIANTS:
  the system must converge to one coherent state at every phase, never a hybrid."""

  OFFSETS_S = (0.0, 0.03, 0.07, 0.11, 0.16, 0.21)

  def _settled_session(self, seed):
    loop = Loop(baseline_mph=60, seed=seed)
    loop.limit_mph = 45
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(1.0)
    assert loop.sla.state == SlaState.active
    loop.run(10.0)
    assert loop.ecu.dash == 45
    return loop

  def test_settled_press_at_any_phase_reanchors(self):
    for offset in self.OFFSETS_S:
      loop = self._settled_session(seed=10)
      loop.run(offset)
      loop.driver_press(ButtonType.accelCruise, in_seconds=0.01)
      loop.run(6.0)
      assert loop.sla.state == SlaState.inactive, f"offset {offset}"
      assert loop.ecu.dash == 46, f"offset {offset}: dash {loop.ecu.dash}"
      assert loop.v_cruise_mph == 46, f"offset {offset}: setpoint {loop.v_cruise_mph}"

  def test_mid_move_press_at_any_phase_converges(self):
    """Abort mid-walk at every phase. Deep in the walk the baseline must survive and
    restore exactly; within the ±2 mph agreement band of the limit the press counts as
    settled and re-anchors; either way the system converges (setpoint == dash) and the
    baseline is never left corrupted at some in-between value."""
    for offset in self.OFFSETS_S:
      loop = Loop(baseline_mph=60, seed=11)
      loop.limit_mph = 45
      loop.run(2.0)
      loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
      loop.run(1.0)
      assert loop.sla.state == SlaState.active

      loop.run(0.9 + offset)  # somewhere in the walk
      dash_at_press = loop.ecu.dash
      loop.driver_press(ButtonType.accelCruise, in_seconds=0.01)
      loop.run(14.0)

      assert loop.sla.state == SlaState.inactive, f"offset {offset}"
      assert loop.v_cruise_mph == loop.ecu.dash, \
        f"offset {offset}: diverged (dash {loop.ecu.dash}, setpoint {loop.v_cruise_mph})"
      if abs(dash_at_press - 45) > 3:
        assert loop.ecu.dash == 60, f"offset {offset}: baseline not restored from {dash_at_press}: {loop.ecu.dash}"


class TestDriverInteractions:
  def test_settled_longpress_climbs_and_reanchors(self):
    """The most common real exit from a zone: settled at the limit, the driver HOLDS +
    to climb. The ECU snaps along its 5 mph grid (possibly with a trailing step), the
    increments stay suppressed (SLA owned the press), and the setpoint re-anchors to
    wherever the ECU landed, with no servo fight afterward."""
    loop = Loop(baseline_mph=60, seed=12)
    loop.limit_mph = 45
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 45

    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1, hold_s=1.3)
    loop.run(6.0)
    assert loop.sla.state == SlaState.inactive
    assert loop.ecu.dash % 5 == 0 and loop.ecu.dash >= 50, f"no grid climb: {loop.ecu.dash}"
    assert loop.v_cruise_mph == loop.ecu.dash, \
      f"setpoint must re-anchor to the ECU result: dash {loop.ecu.dash}, setpoint {loop.v_cruise_mph}"
    dash_settled = loop.ecu.dash
    loop.run(4.0)
    assert loop.ecu.dash == dash_settled, "servo fought the driver's hold result"

  def test_settled_longpress_descends_and_reanchors(self):
    """The mirror exit: settled at the limit, the driver HOLDS - to ride below it. The
    press dismisses the session, the ECU grid-descends, the setpoint re-anchors to the
    result, and the SET- grace keeps the servo from restoring the old baseline over it."""
    loop = Loop(baseline_mph=60, seed=14)
    loop.limit_mph = 45
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 45

    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1, hold_s=1.3)
    loop.run(6.0)
    assert loop.sla.state == SlaState.inactive
    assert loop.ecu.dash % 5 == 0 and loop.ecu.dash <= 40, f"no grid descent: {loop.ecu.dash}"
    assert loop.v_cruise_mph == loop.ecu.dash, \
      f"setpoint must re-anchor to the ECU result: dash {loop.ecu.dash}, setpoint {loop.v_cruise_mph}"
    dash_settled = loop.ecu.dash
    loop.run(4.0)
    assert loop.ecu.dash == dash_settled, "servo fought the driver's hold result"

  def test_up_confirm_adopts_limit(self):
    """Drive 0000000b t=415/461: cruising below a rising limit, + on the prompt must take
    the setpoint and the dash TO the limit, not leave a +1 orphan with an inert session
    (min() source selection can never let an above-setpoint SLA target win)."""
    loop = Loop(baseline_mph=40, seed=20)
    loop.limit_mph = 45
    loop.run(2.0)
    assert loop.sla.state == SlaState.preActive, loop.sla.state

    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)
    loop.run(1.0)
    assert loop.sla.state == SlaState.active, loop.sla.state
    assert loop.v_cruise_mph == 45, f"setpoint must adopt the confirmed limit: {loop.v_cruise_mph}"
    assert any(e == EventNameSP.speedLimitActive for _, e in loop.sla_events), \
      "an explicit up-confirm must announce the adjustment"

    loop.run(10.0)
    assert loop.ecu.dash == 45, f"dash never walked up to the limit: {loop.ecu.dash}"
    assert loop.sla.state == SlaState.active
    assert loop.v_cruise_mph == 45

  def test_up_confirm_keeps_higher_baseline(self):
    """Zone reopens mid-session: settled at 40 under a 48 baseline, limit rises to 45,
    the confirm walks the dash up to 45 but the 48 baseline survives (the session caps
    the plan; the setpoint is only ever raised toward the limit, never lowered by it)."""
    loop = Loop(baseline_mph=48, seed=21)
    loop.limit_mph = 40
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 40

    loop.limit_mph = 45
    loop.run(1.0)
    assert loop.sla.state == SlaState.preActive
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)  # cluster 48 > 45: confirm is -
    loop.run(12.0)
    assert loop.sla.state == SlaState.active, loop.sla.state
    assert loop.ecu.dash == 45, f"dash: {loop.ecu.dash}"
    assert loop.v_cruise_mph == 48, f"baseline corrupted: {loop.v_cruise_mph}"

  def test_pre_active_holds_dash_until_answered(self):
    """Drive 0000000b t=180.8: limit rises mid-session and ICBM restored the dash toward
    the baseline while the confirm prompt was still showing. The prompt must freeze the
    plan: no un-confirmed acceleration; the restore may only run after the timeout."""
    loop = Loop(baseline_mph=48, seed=22)
    loop.limit_mph = 40
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 40

    loop.limit_mph = 45
    def frozen(lo):
      if lo.sla.state == SlaState.preActive:
        assert lo.ecu.dash <= 41, f"dash restored during the prompt: {lo.ecu.dash}"
    loop.run(4.9, assert_each=frozen)
    assert loop.sla.state == SlaState.preActive, loop.sla.state
    loop.run(12.0)  # timeout -> inactive -> quiet window -> restore to baseline
    assert loop.sla.state == SlaState.inactive
    assert loop.ecu.dash == 48, f"restore after timeout stopped short: {loop.ecu.dash}"

  def test_pre_active_decline_by_opposite_press(self):
    """A release against the confirm direction declines the prompt: the session ends at
    once (no lingering hold shadowing the driver's dialing) and the press still counts
    as a normal increment."""
    loop = Loop(baseline_mph=50, seed=23)
    loop.limit_mph = 35
    loop.run(2.0)
    assert loop.sla.state == SlaState.preActive  # confirm would be -

    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)
    loop.run(1.0)
    assert loop.sla.state == SlaState.inactive, loop.sla.state
    assert loop.v_cruise_mph == 51, f"declining press must still increment: {loop.v_cruise_mph}"
    assert not any(e == EventNameSP.speedLimitActive for _, e in loop.sla_events)

  def test_engage_on_limit_is_silent(self):
    """Drive 0000000b t=155.1: resuming with the setpoint already at the limit fired
    'Auto adjusting to speed limit'. Activation that changes nothing must be silent."""
    loop = Loop(baseline_mph=45, seed=24)
    loop.limit_mph = 45
    loop.run(3.0)
    assert loop.sla.state == SlaState.active, loop.sla.state
    assert not loop.sla_events, f"silent activation expected: {loop.sla_events}"

  def test_dial_to_target_activates_silently_and_sticks(self):
    """Drive 0000000b t=187.05: dialing onto the limit activated SLA with an alert and
    the same press's latch dismissed it one frame later. It must latch silently and
    survive its own activating press."""
    loop = Loop(baseline_mph=43, seed=25)
    loop.limit_mph = 45
    loop.run(2.0)
    assert loop.sla.state == SlaState.preActive
    loop.run(6.0)  # let the prompt time out (driver ignores it)
    assert loop.sla.state == SlaState.inactive

    loop.sla_events.clear()
    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)
    loop.run(1.0)
    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)
    loop.run(2.0)
    assert loop.v_cruise_mph == 45, loop.v_cruise_mph
    assert loop.sla.state == SlaState.active, f"dial-to-target must latch: {loop.sla.state}"
    states = set()
    loop.run(3.0, assert_each=lambda lo: states.add(lo.sla.state))
    assert states == {SlaState.active}, f"activation did not stick: {states}"
    assert not any(e == EventNameSP.speedLimitActive for _, e in loop.sla_events), \
      "dial-to-target activation must be silent"

  def test_decline_waits_full_quiet_window_before_restore(self):
    """The prompt must not pre-pay the servo's patience: after a decline, the restore
    toward the (incremented) baseline starts only after a FULL quiet window, giving
    card time to settle the decline press's own effects first."""
    loop = Loop(baseline_mph=48, seed=27)
    loop.limit_mph = 40
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 40

    loop.limit_mph = 45
    loop.run(1.0)
    assert loop.sla.state == SlaState.preActive
    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)  # against the - confirm: decline
    loop.run(0.5)
    assert loop.sla.state == SlaState.inactive, loop.sla.state
    assert loop.v_cruise_mph == 49, f"declining press must still increment: {loop.v_cruise_mph}"

    dash_at_decline = loop.ecu.dash
    loop.run(0.3)  # still inside the quiet window (1 s, counted from the decline)
    assert loop.ecu.dash <= dash_at_decline + 1, \
      f"restore began inside the quiet window: {loop.ecu.dash} from {dash_at_decline}"
    loop.run(12.0)
    assert loop.ecu.dash == 49, f"restore never completed: {loop.ecu.dash}"

  def test_no_emission_escapes_at_prompt_onset(self):
    """The servo's own freeze is one hop stale; card's same-frame veto must stop any
    button frame from reaching the ECU from the first prompting frame on."""
    loop = Loop(baseline_mph=48, seed=28)
    loop.limit_mph = 40
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 40

    loop.limit_mph = 45
    def frozen(lo):
      if lo.helper.cruise_arbiter.prompting:
        assert lo.ecu.dash == 40, f"dash moved during the prompt: {lo.ecu.dash}"
    loop.run(4.9, assert_each=frozen)
    assert loop.sla.state == SlaState.preActive

  def test_up_confirm_press_at_any_phase_converges(self):
    """The up-confirm press swept across the 20 Hz SLA cycle: at every phase the outcome
    must be the full adoption (setpoint == dash == limit, session active), never the
    logged hybrid of a +1 increment with an inert active session."""
    for offset in TestPressTimingSweeps.OFFSETS_S:
      loop = Loop(baseline_mph=40, seed=26)
      loop.limit_mph = 45
      loop.run(2.0 + offset)
      assert loop.sla.state == SlaState.preActive
      loop.driver_press(ButtonType.accelCruise, in_seconds=0.01)
      loop.run(12.0)
      assert loop.sla.state == SlaState.active, f"offset {offset}: {loop.sla.state}"
      assert loop.v_cruise_mph == 45, f"offset {offset}: setpoint {loop.v_cruise_mph}"
      assert loop.ecu.dash == 45, f"offset {offset}: dash {loop.ecu.dash}"

  def test_press_during_scc_dip_with_sla_session(self):
    """Two limiters overlapping: settled SLA session, then a curve dips below it. A +
    press dismisses the SLA session but must NOT lift the curve limit or corrupt the
    baseline; once the dip clears, the restore goes all the way to the baseline (the
    dismissed session must not re-grab at 45)."""
    loop = Loop(baseline_mph=60, seed=13)
    loop.limit_mph = 45
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 45

    loop.scc_dip_mph = 40
    loop.run(5.0)
    assert loop.ecu.dash <= 41, f"dash never followed the dip: {loop.ecu.dash}"

    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)
    loop.run(2.0)
    assert loop.sla.state == SlaState.inactive
    assert loop.ecu.dash <= 42, "the press must not lift the still-active curve limit"
    assert loop.v_cruise_mph == 60, f"baseline corrupted: {loop.v_cruise_mph}"

    loop.scc_dip_mph = 0.
    loop.run(14.0)
    assert loop.ecu.dash == 60, f"restore stopped short: {loop.ecu.dash}"
    assert loop.v_cruise_mph == 60


class TestPromptFreezeOvershoot:
  """User report 2026-08-29 (routes ...acdc83b60f/3 and ...821e28d2fa/12): engaging near a
  known limit dropped the set speed 2-4 mph roughly 5 s later, then walked it back.

  The prompt's own freeze produced it. Capping the plan at the cluster round-tripped
  through whole mph and landed ~7 mm/s under v_cruise, so SLA won the plan min() by
  rounding error and relabelled the source as a limiter. That armed decel overshoot
  against an ordinary cruise convergence, the servo's freeze banked the resulting gap
  for the whole 5 s window, and the timeout dumped it as a SET- burst."""

  def test_ignored_prompt_never_moves_the_dash(self):
    """The reported drive: engaged at 40 with a 45 target, car 1.3 mph over the setpoint,
    prompt left unanswered. Nothing may move -- during the prompt or after it times out."""
    loop = Loop(baseline_mph=40, seed=31)
    loop.decel_overshoot = True
    loop.v_ego_mph = 41.3
    loop.a_target = -0.5  # the plan converging on the setpoint the car is sitting above
    loop.limit_mph = 45
    loop.run(1.0)
    assert loop.sla.state == SlaState.preActive

    def never_moves(lo):
      assert lo.ecu.dash == 40, f"dash moved at tick {lo.tick_n}: {lo.ecu.dash}"
      assert lo.servo.overshoot_mph == 0., f"overshoot banked behind the freeze: {lo.servo.overshoot_mph}"

    loop.run(10.0, assert_each=never_moves)  # 5 s prompt + timeout + the restore window
    assert loop.sla.state == SlaState.inactive
    assert loop.v_cruise_mph == 40

  def test_prompt_does_not_relabel_the_plan_source(self):
    """Layer 1 in isolation: prompting from idle must leave the plan on `cruise`. A cap
    equal to the baseline changes no speed but does change the source, and the source is
    what arms the overshoot."""
    loop = Loop(baseline_mph=40, seed=32)
    loop.v_ego_mph = 41.3
    loop.limit_mph = 45
    loop.run(1.0)
    assert loop.sla.state == SlaState.preActive

    def stays_cruise(lo):
      if lo.sla.prompting:
        assert lo._lp_sp().longitudinalPlanSource == PlanSource.cruise, "prompt relabelled the plan source"

    loop.run(4.0, assert_each=stays_cruise)

  def test_session_hold_still_freezes_an_active_session(self):
    """Layer 1 must not cost the freeze its real job: prompting OUT OF an active session
    still holds that session's cap, so the dash cannot restore un-confirmed."""
    loop = Loop(baseline_mph=48, seed=33)
    loop.limit_mph = 40
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 40

    loop.limit_mph = 45  # a limit change out of an active session re-prompts
    loop.run(0.5)
    assert loop.sla.state == SlaState.preActive
    assert loop.sla.v_cap < 45 * MPH_MS, f"active-session hold released: {loop.sla.v_cap}"
