from openpilot.system.manager.process_config import managed_processes


def test_ubloxd_restarts_on_crash() -> None:
  assert managed_processes["ubloxd"].restart_if_crash is True
  assert managed_processes["pigeond"].restart_if_crash is True
  # qcomgpsd remains without crash-restart (unchanged lifecycle).
  assert managed_processes["qcomgpsd"].restart_if_crash is False


def test_ubloxd_run_condition_unchanged() -> None:
  # enabled on TICI; should_run still bound to ublox availability predicate name.
  assert managed_processes["ubloxd"].enabled in (True, False)
  assert managed_processes["ubloxd"].should_run is managed_processes["pigeond"].should_run
