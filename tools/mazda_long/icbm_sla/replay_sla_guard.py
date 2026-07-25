#!/usr/bin/env python3
"""Regression replay: the seg16 confirm bug (F1) against the current SLA machine.

Replays the recorded carState/longitudinalPlanSP stream from the route where +/-
confirmation self-destructed (952c07dea500f4e2/0000004f--fea08aad07/16, 65 mph zone,
dash set 50, SLA target 70) through the CURRENT non-pcm SpeedLimitAssist and checks:

  1. the driver's first matching press confirms (preActive -> active), and
  2. the session STAYS active until the next genuine driver press — on the shipped build
     it fell to inactive within one cycle because the confirm press's own cluster change
     tripped the manual-override guard, while the cluster provably changed in that window.

Run from repo root (venv active):
  python tools/mazda_long/icbm_sla/replay_sla_guard.py [path-to-rlog]
"""
import sys
from pathlib import Path

from openpilot.tools.lib.logreader import LogReader
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

SlaState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
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
  sla = SpeedLimitAssist(CP, CP_SP)

  events_sp = EventsSP()
  enabled = False
  limit = 0.0
  limit_valid = False
  cluster_ms = 0.0
  cs_count = 0

  confirm_time = None
  press_times = []
  cluster_changes_after_confirm = 0
  broke_without_press = None
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
      limit = r.speedLimitFinalLast
      limit_valid = r.speedLimitValid or r.speedLimitLastValid
    elif which == 'carState':
      cs = msg.carState
      cs_count += 1
      prev_cluster = cluster_ms
      cluster_ms = cs.vCruiseCluster * CV.KPH_TO_MS

      CS = car.CarState()
      evs = []
      for b in cs.buttonEvents:
        bt = BUTTON_MAP.get(str(b.type))
        if bt is not None:
          evs.append(car.CarState.ButtonEvent(type=bt, pressed=b.pressed))
          if b.pressed and bt in (ButtonType.accelCruise, ButtonType.decelCruise):
            press_times.append(t - t0)
      CS.buttonEvents = evs
      sla.update_car_state(CS)

      if cs_count % 5 == 0:  # 20 Hz machine, as plannerd runs it
        prev_state = sla.state
        sla.update(enabled, False, cs.vEgo, cs.aEgo, cluster_ms, limit, limit, limit_valid, 0., events_sp)
        events_sp.clear()

        if prev_state == SlaState.preActive and sla.state == SlaState.active and confirm_time is None:
          confirm_time = t - t0
          print(f"  t={confirm_time:7.2f}s  CONFIRMED (preActive -> active)")
        if confirm_time is not None and sla.state == SlaState.active and abs(cluster_ms - prev_cluster) > 0.1:
          cluster_changes_after_confirm += 1
        if prev_state == SlaState.active and sla.state == SlaState.inactive:
          last_press = max((p for p in press_times if p <= t - t0), default=None)
          gap = (t - t0 - last_press) if last_press is not None else float('inf')
          tag = f"driver press {gap*1000:.0f} ms earlier" if gap < 0.6 else "NO recent press  <-- would be the F1 bug"
          print(f"  t={t - t0:7.2f}s  active -> inactive ({tag})")
          if gap >= 0.6 and broke_without_press is None:
            broke_without_press = t - t0

  print(f"\n  {cs_count} carState frames, {len(press_times)} driver +/- presses")
  # Informational only: in this recording the driver re-pressed (mashing at the old bug)
  # before the cluster ever moved, so cluster robustness can't be shown open-loop here —
  # the closed-loop harness (test_icbm_sla_loop.py) covers it.
  print(f"  cluster changes observed while active: {cluster_changes_after_confirm}")
  assert confirm_time is not None, "FAIL: confirmation never happened in replay"
  assert broke_without_press is None, \
    f"FAIL: session deactivated without a driver press at t={broke_without_press:.2f}s (F1 regressed)"
  print("  PASS: confirm fired at the documented moment; only genuine driver presses ended sessions")


if __name__ == "__main__":
  main(Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_LOG)
