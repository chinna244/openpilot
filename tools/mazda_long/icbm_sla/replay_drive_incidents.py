#!/usr/bin/env python3
"""Regression replay: drive 0000000b--b039e84091 (2026-07-26) against the current SLA machine.

That drive, recorded on the shipped build, contains every reported SLA jank in one route:

  t≈ 85.9  dialing onto the limit fired "Auto adjusting to speed limit"
  t≈155.1  resume with setpoint already at the limit fired the same alert
  t≈181.1  limit raised mid-session: ICBM restored the dash upward DURING the prompt
  t≈187.1  dial-to-target activation was dismissed one frame later by its own press
  t≈393.6  + during a down-prompt left a lingering prompt while the dash slammed to 50
  t≈415.6  + confirm on a rising limit went active but stayed inert (min() select)

This replays the recorded carState/longitudinalPlanSP stream through the CURRENT non-pcm
SpeedLimitAssist (with the machine's clock mapped to log time so the press latches see
real gaps) and asserts the fixed behavior at each documented moment. Setpoint adoption
and the ICBM walk are closed-loop concerns covered by test_icbm_sla_loop.py; here we
assert the machine's states, events, and the preActive plan hold.

Run from repo root (venv active):
  python tools/mazda_long/icbm_sla/replay_drive_incidents.py [route_glob]
"""
import glob
import sys

from openpilot.tools.lib.logreader import LogReader
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import speed_limit_assist as sla_mod
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

SlaState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
EventNameSP = custom.OnroadEventSP.EventName
ButtonType = car.CarState.ButtonEvent.Type
DEFAULT_GLOB = "tools/mazda_long/test_data/sla_drive_logs/0000000b--b039e84091--*/rlog.zst"

BUTTON_MAP = {
  'accelCruise': ButtonType.accelCruise,
  'decelCruise': ButtonType.decelCruise,
  'setCruise': ButtonType.setCruise,
  'resumeCruise': ButtonType.resumeCruise,
}


class LogClock:
  """Stands in for the time module inside the SLA machine: monotonic() = log time."""
  def __init__(self):
    self.t = 0.

  def monotonic(self):
    return self.t


def main(route_glob):
  params = Params()
  params.put("IsReleaseSpBranch", True, block=True)
  params.put("SpeedLimitMode", int(Mode.assist), block=True)
  params.put_bool("IsMetric", False, block=True)

  clock = LogClock()
  sla_mod.time = clock  # latches must see log-time gaps, not replay wall time

  CP = car.CarParams(pcmCruise=True, brand="mazda")
  CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
  sla = sla_mod.SpeedLimitAssist(CP, CP_SP)
  events_sp = EventsSP()

  paths = sorted(glob.glob(route_glob), key=lambda p: int(p.split('--')[-1].split('/')[0]))
  assert paths, f"no rlogs match {route_glob}"

  enabled = False
  limit = final = 0.0
  limit_valid = False
  cs_count = 0
  t0 = None

  transitions = []  # (t, from, to)
  fired = []        # (t, event int)
  holds = []        # (t, output_v_target) sampled while preActive

  print(f"replaying {len(paths)} segments of {paths[0].split('/')[-2].rsplit('--', 1)[0]}")
  for path in paths:
    for msg in LogReader(path):
      which = msg.which()
      t_abs = msg.logMonoTime * 1e-9
      if t0 is None:
        t0 = t_abs
      t = t_abs - t0
      clock.t = t

      if which == 'carControl':
        enabled = msg.carControl.enabled
      elif which == 'longitudinalPlanSP':
        r = msg.longitudinalPlanSP.speedLimit.resolver
        limit = r.speedLimit
        final = r.speedLimitFinalLast
        limit_valid = r.speedLimitValid or r.speedLimitLastValid
      elif which == 'carState':
        cs = msg.carState
        cs_count += 1

        CS = car.CarState()
        CS.buttonEvents = [car.CarState.ButtonEvent(type=BUTTON_MAP[str(b.type)], pressed=b.pressed)
                           for b in cs.buttonEvents if str(b.type) in BUTTON_MAP]
        sla.update_car_state(CS)

        if cs_count % 5 == 0:  # 20 Hz machine, as plannerd runs it
          prev_state = sla.state
          events_sp.clear()
          sla.update(enabled, False, cs.vEgo, cs.aEgo, cs.vCruiseCluster * CV.KPH_TO_MS,
                     limit, final, limit_valid, 0., events_sp)
          for e in events_sp.events:
            if e in (EventNameSP.speedLimitActive, EventNameSP.speedLimitChanged, EventNameSP.speedLimitPending):
              fired.append((t, e))
          if sla.state != prev_state:
            transitions.append((t, prev_state, sla.state))
            print(f"  t={t:7.2f}s  {str(prev_state):9s} -> {sla.state}")
          if sla.state == SlaState.preActive:
            holds.append((t, sla.output_v_target))

  def transitions_in(t_a, t_b, frm=None, to=None):
    return [x for x in transitions if t_a <= x[0] <= t_b
            and (frm is None or x[1] == frm) and (to is None or x[2] == to)]

  def events_in(t_a, t_b):
    return [x for x in fired if t_a <= x[0] <= t_b]

  failures = []

  # 1. silent activations: dialing onto the limit / resuming at it must not alert
  for name, (a, b) in {"dial-to-target t≈85.9": (85.0, 87.0),
                       "resume-at-limit t≈155.1": (154.0, 157.0),
                       "dial-to-target t≈187.1": (186.5, 188.5)}.items():
    if not transitions_in(a, b, to=SlaState.active):
      failures.append(f"{name}: no activation")
    if events_in(a, b):
      failures.append(f"{name}: spurious alert {events_in(a, b)}")

  # 2. the t≈187.1 activation must survive its own press (shipped build: 1-frame blip)
  if transitions_in(187.0, 190.0, frm=SlaState.active, to=SlaState.inactive):
    failures.append("t≈187.1: activation dismissed by its own press again")

  # 3. confirm presses: preActive -> active WITH the announcement
  for name, (a, b) in {"down-confirm t≈174.8": (174.3, 175.5),
                       "down-confirm t≈238.5": (238.2, 239.0),
                       "down-confirm t≈343.4": (343.0, 344.2),
                       "down-confirm t≈358.5": (358.0, 359.2),
                       "up-confirm t≈415.6": (415.2, 416.4),
                       "up-confirm t≈461.7": (461.3, 462.4)}.items():
    if not transitions_in(a, b, frm=SlaState.preActive, to=SlaState.active):
      failures.append(f"{name}: confirm did not activate")
    if not events_in(a, b):
      failures.append(f"{name}: confirm fired no announcement")

  # 4. + against a down-prompt declines the session instead of lingering
  if not transitions_in(393.5, 394.5, frm=SlaState.preActive, to=SlaState.inactive):
    failures.append("t≈393.6: wrong-direction press did not decline the prompt")

  # 5. the prompt freezes the plan: while preActive out of an active session, the output
  #    must hold the old session target (not V_CRUISE_UNSET, which releases the restore)
  for name, (a, b, tgt_mph) in {"limit 45->35 t≈171-175": (171.5, 174.5, 49.5),
                                "limit 35->45 t≈181-186": (181.0, 185.5, 38.5),
                                "limit 35->25 t≈357-358": (357.3, 358.3, 35.0)}.items():
    window = [v for tt, v in holds if a <= tt <= b]
    if not window:
      failures.append(f"hold {name}: no preActive samples")
    elif any(abs(v - tgt_mph * CV.MPH_TO_MS) > 0.7 for v in window):
      seen = sorted({round(v * CV.MS_TO_MPH, 1) for v in window})
      failures.append(f"hold {name}: plan not frozen at ~{tgt_mph} mph: {seen}")

  expected_windows = [(174.3, 175.5), (238.2, 239.0), (343.0, 344.2), (358.0, 359.2), (415.2, 416.4), (461.3, 462.4)]
  spurious = [x for x in fired if not any(a <= x[0] <= b for a, b in expected_windows)]

  print(f"\n  {cs_count} carState frames, {len(transitions)} transitions, {len(fired)} alerts ({len(spurious)} outside confirm windows)")
  for s in spurious:
    print(f"    unexpected alert at t={s[0]:.2f}: {s[1]}")
  if spurious:
    failures.append(f"{len(spurious)} alert(s) fired outside the expected confirm windows")

  if failures:
    print("\nFAIL:")
    for f in failures:
      print(f"  - {f}")
    sys.exit(1)
  print("  PASS: silent latches, announced confirms, decline, and preActive plan holds all verified")


if __name__ == "__main__":
  main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_GLOB)
