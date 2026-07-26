#!/usr/bin/env python3
"""Regression replay: the seg16 confirm bug (F1) against the cruise arbiter.

Replays the recorded carState/longitudinalPlanSP stream from the route where +/-
confirmation self-destructed (952c07dea500f4e2/0000004f--fea08aad07/16, 65 mph zone,
dash set 50, SLA target 70) through the CURRENT card-side CruiseArbiter and checks:

  1. the driver's first matching press confirms (preActive -> active), and
  2. the session STAYS active until the next genuine driver press; on the shipped build
     it fell to inactive within one cycle because the confirm press's own cluster change
     tripped the manual-override guard. The arbiter classifies presses at their edges,
     so only a dismiss-classified press may end a session.

Run from repo root (venv active):
  python tools/mazda_long/icbm_sla/replay_sla_guard.py [path-to-rlog]
"""
import sys
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader
from openpilot.common.params import Params
from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.sunnypilot.selfdrive.car.cruise_arbiter import CruiseArbiter, CruiseIntent
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode

SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
ButtonType = car.CarState.ButtonEvent.Type
DEFAULT_LOG = Path.home() / "Desktop" / "952c07dea500f4e2_0000004f--fea08aad07--16--rlog.zst"

BUTTON_MAP = {
  'accelCruise': ButtonType.accelCruise,
  'decelCruise': ButtonType.decelCruise,
  'setCruise': ButtonType.setCruise,
  'resumeCruise': ButtonType.resumeCruise,
}


def main(log_path):
  params = Params()
  params.put("IsReleaseSpBranch", True, block=True)
  params.put("SpeedLimitMode", int(Mode.assist), block=True)
  params.put_bool("IsMetric", False, block=True)

  CP = car.CarParams(pcmCruise=True, brand="mazda")
  CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
  arb = CruiseArbiter(CP, CP_SP)
  arb.read_params(params)

  enabled = False
  lp = custom.LongitudinalPlanSP()
  cs_count = 0
  press_count = 0
  confirm_time = None
  bad_end = None
  t0 = None

  print(f"replaying {log_path}")
  for msg in LogReader(str(log_path)):
    which = msg.which()
    t = msg.logMonoTime * 1e-9
    if t0 is None:
      t0 = t

    if which == 'carControl':
      enabled = msg.carControl.enabled
    elif which == 'longitudinalPlanSP':
      r = msg.longitudinalPlanSP.speedLimit.resolver
      lp = custom.LongitudinalPlanSP()
      lp.speedLimit.resolver.speedLimit = r.speedLimit
      lp.speedLimit.resolver.speedLimitFinalLast = r.speedLimitFinalLast
      lp.speedLimit.resolver.speedLimitLastValid = r.speedLimitValid or r.speedLimitLastValid
    elif which == 'carState':
      cs = msg.carState
      cs_count += 1
      CS = car.CarState()
      CS.buttonEvents = [car.CarState.ButtonEvent(type=BUTTON_MAP[str(b.type)], pressed=b.pressed)
                         for b in cs.buttonEvents if str(b.type) in BUTTON_MAP]
      press_count += sum(1 for b in cs.buttonEvents if b.pressed and str(b.type) in ('accelCruise', 'decelCruise'))

      arb.update_limit(lp)
      prev_state = arb.state
      arb.step(CS, enabled, cs.vCruise, cs.vCruiseCluster)

      if prev_state == SessionState.preActive and arb.state == SessionState.active and confirm_time is None:
        confirm_time = t - t0
        print(f"  t={confirm_time:7.2f}s  CONFIRMED (preActive -> active, intent={arb.last_intent})")
      if prev_state == SessionState.active and arb.state == SessionState.inactive:
        driver = arb.last_intent == CruiseIntent.dismiss
        tag = "driver dismiss press" if driver else "NO driver press  <-- would be the F1 bug"
        print(f"  t={t - t0:7.2f}s  active -> inactive ({tag})")
        if not driver and bad_end is None:
          bad_end = t - t0

  print(f"\n  {cs_count} carState frames, {press_count} driver +/- presses")
  assert confirm_time is not None, "FAIL: confirmation never happened in replay"
  assert bad_end is None, \
    f"FAIL: session ended without a dismiss-classified press at t={bad_end:.2f}s (F1 regressed)"
  print("  PASS: confirm fired at the documented moment; only dismiss-classified presses ended sessions")


if __name__ == "__main__":
  main(Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_LOG)
