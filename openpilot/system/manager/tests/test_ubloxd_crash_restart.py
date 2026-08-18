from unittest.mock import MagicMock

from openpilot.system.manager.process import NativeProcess, ensure_running
from openpilot.system.manager.process_config import managed_processes


def test_ubloxd_restarts_on_crash() -> None:
  assert managed_processes["ubloxd"].restart_if_crash is True
  assert managed_processes["pigeond"].restart_if_crash is True
  assert "qcomgpsd" not in managed_processes
  assert managed_processes["gpsard"].restart_if_crash is True


def test_ubloxd_run_condition_unchanged() -> None:
  # enabled on TICI; should_run still bound to ublox availability predicate name.
  assert managed_processes["ubloxd"].enabled in (True, False)
  assert managed_processes["ubloxd"].should_run is managed_processes["pigeond"].should_run


def test_native_process_default_restart_if_crash_false() -> None:
  proc = NativeProcess("native_default", ".", ["./true"], lambda *_args: True)
  assert proc.restart_if_crash is False


def test_locationd_llk_restart_if_crash_enabled() -> None:
  assert managed_processes["locationd_llk"].restart_if_crash is True
  # only_onroad lifecycle policy is unchanged
  assert managed_processes["locationd_llk"].should_run is managed_processes["locationd"].should_run


def test_unrelated_native_processes_keep_default_restart_policy() -> None:
  for name in ("camerad", "mapd", "bridge"):
    if name not in managed_processes:
      continue
    proc = managed_processes[name]
    assert isinstance(proc, NativeProcess)
    assert proc.restart_if_crash is False


class _RestartCountingNative(NativeProcess):
  """Test double: count restart eligibility without launching a real binary."""

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self.restart_calls = 0

  def restart(self) -> None:
    self.restart_calls += 1
    alive = MagicMock()
    alive.is_alive.return_value = True
    alive.exitcode = None
    alive.pid = 4242
    self.proc = alive
    self.shutting_down = False

  def start(self) -> None:
    return


def test_locationd_llk_ensure_running_restarts_dead_process() -> None:
  """Dead locationd_llk while still required must take the restart path."""
  proc = _RestartCountingNative(
    "locationd_llk_test",
    ".",
    ["./true"],
    lambda started, params, CP: bool(started),
    restart_if_crash=True,
  )
  dead = MagicMock()
  dead.is_alive.return_value = False
  dead.exitcode = 1
  proc.proc = dead

  ensure_running({"locationd_llk_test": proc}.values(), started=True, params=None, CP=None)

  assert proc.restart_calls == 1
  assert proc.proc is not None
  assert proc.proc.is_alive() is True
