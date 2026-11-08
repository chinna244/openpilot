#!/usr/bin/env python3
"""Marks the running build as proven good after a clean drive, so the boot script keeps it as the
rollback copy when the next update installs (launch_chffrplus.sh). A drive is clean when openpilot
(or MADS steering) was engaged for at least ENGAGED_MIN_S and nothing disengaged it on an error: no
soft or immediate disable alert and no process crashing mid-drive. It is judged when the drive ends;
one clean drive is enough, a later drive with an error does not take the mark back."""
import openpilot.cereal.messaging as messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.git import get_commit
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import jetlink_adapter

ENGAGED_MIN_S = 120.
PROC_DOWN_MAX_S = 5.
ERROR_TYPES = ("softDisable", "immediateDisable")
# disables the driver causes are not errors in the build
DRIVER_EVENTS = {"doorOpen", "seatbeltNotLatched", "wrongGear", "reverseGear", "parkBrake", "wrongCarMode", "espDisabled",
                 "tooDistracted", "driverUnresponsive"}
# processes selfdrived lets crash without disengaging
OPTIONAL_PROCS = {"mapd", "speed_limit_vision", jetlink_adapter.OWNER}


class DriveProof:
  def __init__(self):
    self.reset()

  def reset(self):
    self.engaged_s = 0.
    self.error = ""
    self.seen_running: set[str] = set()
    self.proc_down_s: dict[str, float] = {}

  def update(self, dt: float, engaged: bool, alert_type: str, procs: list[tuple[str, bool, bool]]) -> None:
    if engaged:
      self.engaged_s += dt

    name, _, event_type = alert_type.partition("/")
    if event_type in ERROR_TYPES and name not in DRIVER_EVENTS and not self.error:
      self.error = alert_type

    # only a crash during this drive counts: one already down when it started is not this drive's
    for proc, running, should_run in procs:
      if running:
        self.seen_running.add(proc)
      down = self.proc_down_s.get(proc, 0.) + dt if should_run and not running and proc in self.seen_running else 0.
      self.proc_down_s[proc] = down
      if down > PROC_DOWN_MAX_S and proc not in OPTIONAL_PROCS and not self.error:
        self.error = f"{proc} not running"

  def proven(self) -> bool:
    return self.engaged_s >= ENGAGED_MIN_S and not self.error


def drive_inputs(sm) -> tuple[bool, str]:
  """Engaged and alert from live selfdrived data only: the last message kept from before the drive
  (e.g. its shutdown alert) is not this drive's."""
  if not sm.alive['selfdriveState']:
    return False, ""
  engaged = sm['selfdriveState'].enabled or (sm.alive['selfdriveStateSP'] and sm['selfdriveStateSP'].mads.enabled)
  return engaged, sm['selfdriveState'].alertType


def main():
  params = Params()
  commit = get_commit(BASEDIR)
  sm = messaging.SubMaster(['deviceState', 'selfdriveState', 'selfdriveStateSP', 'managerState'], poll='deviceState')
  proof = DriveProof()
  started = False

  while True:
    sm.update(1000)
    if not sm.updated['deviceState']:
      continue

    if sm['deviceState'].started:
      started = True
      engaged, alert = drive_inputs(sm)
      procs = [(p.name, p.running, p.shouldBeRunning) for p in sm['managerState'].processes]
      proof.update(0.5, engaged, alert, procs)
    elif started:
      started = False
      cloudlog.event("drive proof", commit=commit, engaged_s=proof.engaged_s, error=proof.error)
      if commit and proof.proven():
        params.put("UpdaterProvenCommit", commit, block=True)
      proof.reset()


if __name__ == "__main__":
  main()
