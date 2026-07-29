from datetime import UTC, datetime, timedelta
from pathlib import Path

import pytest

from openpilot.system.ubloxd.navigation_database_restore import (
  NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS,
  NavigationDatabaseRestoreDisposition,
)
from openpilot.system.ubloxd.navigation_database_restore_runtime import (
  NavigationDatabaseRestoreBootState,
  NavigationDatabaseRestoreRuntime,
  NavigationDatabaseRestoreSnapshot,
  load_navigation_database_restore_boot_state,
  store_navigation_database_restore_boot_state,
)
from openpilot.system.ubloxd.trusted_time_anchor import (
  TimeProvenance,
  TrustedTimeSource,
)
from openpilot.system.ubloxd.trusted_time_authority import (
  AuthorizedTime,
  TimeAuthorizationEvidence,
)


NOW = datetime(2026, 7, 28, 20, 0, tzinfo=UTC)
BOOT_ID = "12345678-1234-5678-9234-567812345678"
OTHER_BOOT_ID = "87654321-4321-6789-9234-567812345678"
FRAMES = (b"frame-0", b"frame-1")


def snapshot(
  age_seconds: float = 1800.0,
  *,
  generation: str = "primary",
) -> NavigationDatabaseRestoreSnapshot:
  return NavigationDatabaseRestoreSnapshot(
    saved_at_utc=NOW - timedelta(seconds=age_seconds),
    database_frames=FRAMES,
    latitude_e7=320_000_000,
    longitude_e7=-960_000_000,
    altitude_cm=20_000,
    position_accuracy_cm=10_000,
    quality=None,
    generation=generation,
    selection_reason="test",
  )


def network_time() -> AuthorizedTime:
  return AuthorizedTime(
    utc=NOW,
    uncertainty_seconds=1.0,
    source=TrustedTimeSource.SYSTEM_SYNCHRONIZED,
    provenance=TimeProvenance.NETWORK_INDEPENDENT,
    independent=True,
    evidence=TimeAuthorizationEvidence.SYSTEM_SYNCHRONIZED,
  )


def receiver_time() -> AuthorizedTime:
  return AuthorizedTime(
    utc=NOW,
    uncertainty_seconds=1.0,
    source=TrustedTimeSource.RECEIVER_UTC_UNASSISTED_GNSS,
    provenance=TimeProvenance.GNSS_INDEPENDENT,
    independent=True,
    evidence=TimeAuthorizationEvidence.RECEIVER_UTC_UNASSISTED_GNSS,
  )


def same_boot_time() -> AuthorizedTime:
  return AuthorizedTime(
    utc=NOW,
    uncertainty_seconds=2.0,
    source=TrustedTimeSource.SYSTEM_SYNCHRONIZED,
    provenance=TimeProvenance.NETWORK_INDEPENDENT,
    independent=False,
    evidence=TimeAuthorizationEvidence.SAME_BOOT_BOOTTIME,
  )


def runtime(
  tmp_path: Path,
  *,
  selected: NavigationDatabaseRestoreSnapshot | None = None,
  boot_id: str = BOOT_ID,
  state_storer=store_navigation_database_restore_boot_state,
) -> NavigationDatabaseRestoreRuntime:
  selected = snapshot() if selected is None else selected
  return NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: selected,
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: boot_id,
    state_storer=state_storer,
  )


def no_cache_runtime(tmp_path: Path) -> NavigationDatabaseRestoreRuntime:
  return NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: None,
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
  )


def evaluate(
  value: NavigationDatabaseRestoreRuntime,
  *,
  authorized_time: AuthorizedTime | None = None,
  reliable: bool = False,
  yuma: bool = False,
  send=None,
):
  return value.evaluate(
    authorized_time=authorized_time,
    reliable_fix_available=reliable,
    yuma_already_sent=yuma,
    send_database_message=send or (lambda _frame, _index: None),
  )


def test_state_round_trip(tmp_path: Path) -> None:
  path = tmp_path / "state.json"
  state = NavigationDatabaseRestoreBootState(
    version=1,
    boot_id=BOOT_ID,
    receiver_fingerprint="receiver",
    disposition=NavigationDatabaseRestoreDisposition.SKIPPED_EXPIRED,
    restore_attempted=False,
    position_assistance_claimed=True,
    acquisition_started=True,
    yuma_sent=True,
    cache_generation="primary",
    cache_saved_at_utc=NOW,
  )
  store_navigation_database_restore_boot_state(state, path)
  assert load_navigation_database_restore_boot_state(path) == state


def test_corrupt_state_fails_closed(tmp_path: Path) -> None:
  path = tmp_path / "dbd_state.json"
  path.write_text("not-json", encoding="utf-8")
  value = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    state_path=path,
    boot_id_reader=lambda: BOOT_ID,
  )
  assert value.controller.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
  assert value.send_position_once(lambda _message: None).position_assistance_attempted is False


def test_new_linux_boot_discards_old_state(tmp_path: Path) -> None:
  path = tmp_path / "dbd_state.json"
  old = NavigationDatabaseRestoreBootState(
    version=1,
    boot_id=OTHER_BOOT_ID,
    receiver_fingerprint="receiver",
    disposition=NavigationDatabaseRestoreDisposition.SKIPPED_EXPIRED,
    restore_attempted=False,
    position_assistance_claimed=True,
    acquisition_started=True,
    yuma_sent=True,
  )
  store_navigation_database_restore_boot_state(old, path)
  value = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=path,
    boot_id_reader=lambda: BOOT_ID,
  )
  assert value.controller.pending
  assert not value.acquisition_started
  assert not value.yuma_sent


def test_snapshot_is_loaded_only_once(tmp_path: Path) -> None:
  calls = 0

  def loader(_fingerprint: str):
    nonlocal calls
    calls += 1
    return snapshot()

  value = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=loader,
    retry_delay_seconds=0.0,
    state_path=tmp_path / "state.json",
    boot_id_reader=lambda: BOOT_ID,
  )
  value.prepare()
  value.prepare()
  evaluate(value)
  assert calls == 1


def test_no_cache_is_terminal_without_writes(tmp_path: Path) -> None:
  value = no_cache_runtime(tmp_path)
  writes = []
  result = evaluate(
    value,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_NO_USABLE_CACHE
  assert writes == []


def test_position_assistance_claim_survives_process_restart(tmp_path: Path) -> None:
  first = runtime(tmp_path)
  messages = []
  first.send_position_once(messages.append)
  second = runtime(tmp_path)
  second.send_position_once(messages.append)
  assert len(messages) == 1


def test_unverified_startup_performs_zero_database_writes(tmp_path: Path) -> None:
  value = runtime(tmp_path)
  writes = []
  result = evaluate(
    value,
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.PENDING
  assert writes == []


def test_expired_cache_performs_zero_database_writes(tmp_path: Path) -> None:
  value = runtime(tmp_path, selected=snapshot(25 * 60 * 60))
  writes = []
  result = evaluate(
    value,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_EXPIRED
  assert writes == []


def test_one_hour_boundary_restores_exactly_once(tmp_path: Path) -> None:
  value = runtime(
    tmp_path,
    selected=snapshot(NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS),
  )
  writes = []
  result = evaluate(
    value,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.RESTORED
  assert writes == [(FRAMES[0], 0), (FRAMES[1], 1)]
  evaluate(value, authorized_time=network_time(), send=lambda *_: writes.append("again"))
  assert len(writes) == 2


def test_restored_terminal_state_survives_process_restart(tmp_path: Path) -> None:
  first = runtime(tmp_path)
  writes = []
  assert evaluate(
    first,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  ).database_available
  second = runtime(tmp_path)
  evaluate(
    second,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert second.controller.disposition is NavigationDatabaseRestoreDisposition.RESTORED
  assert len(writes) == len(FRAMES)


def test_interrupted_attempt_recovers_as_write_failed(tmp_path: Path) -> None:
  path = tmp_path / "dbd_state.json"
  interrupted = NavigationDatabaseRestoreBootState(
    version=1,
    boot_id=BOOT_ID,
    receiver_fingerprint="receiver",
    disposition=NavigationDatabaseRestoreDisposition.PENDING,
    restore_attempted=True,
    position_assistance_claimed=True,
    acquisition_started=False,
    yuma_sent=False,
    cache_generation="primary",
    cache_saved_at_utc=snapshot().saved_at_utc,
  )
  store_navigation_database_restore_boot_state(interrupted, path)
  value = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=path,
    boot_id_reader=lambda: BOOT_ID,
  )
  writes = []
  result = evaluate(
    value,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.WRITE_FAILED
  assert result.recovered_interrupted_attempt
  assert writes == []


def test_receiver_time_performs_zero_database_writes(tmp_path: Path) -> None:
  value = runtime(tmp_path)
  writes = []
  result = evaluate(
    value,
    authorized_time=receiver_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_LATE_RECEIVER_TIME
  assert writes == []


def test_same_boot_continuity_performs_zero_database_writes(tmp_path: Path) -> None:
  value = runtime(tmp_path)
  writes = []
  result = evaluate(
    value,
    authorized_time=same_boot_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
  assert writes == []


def test_acquisition_latch_survives_process_restart(tmp_path: Path) -> None:
  first = runtime(tmp_path)
  first.note_acquisition_started()
  second = runtime(tmp_path)
  writes = []
  result = evaluate(
    second,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_ACQUISITION_ALREADY_STARTED
  assert writes == []


def test_yuma_latch_survives_process_restart(tmp_path: Path) -> None:
  first = runtime(tmp_path)
  first.note_yuma_sent()
  second = runtime(tmp_path)
  result = evaluate(second, authorized_time=network_time())
  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_YUMA_ALREADY_SENT


def test_reliable_fix_is_terminal_before_restore(tmp_path: Path) -> None:
  value = runtime(tmp_path)
  result = evaluate(value, authorized_time=network_time(), reliable=True)
  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_RELIABLE_FIX


def test_failed_frame_is_retried_once_and_can_recover(tmp_path: Path) -> None:
  value = runtime(tmp_path)
  attempts = {0: 0, 1: 0}

  def send(_frame: bytes, index: int) -> None:
    attempts[index] += 1
    if index == 0 and attempts[index] == 1:
      raise TimeoutError("first attempt")

  result = evaluate(value, authorized_time=network_time(), send=send)
  assert result.disposition is NavigationDatabaseRestoreDisposition.RESTORED
  assert attempts == {0: 2, 1: 1}
  assert result.retry_accepted_indexes == (0,)


def test_permanent_partial_failure_marks_database_unavailable(tmp_path: Path) -> None:
  value = runtime(tmp_path)

  def send(_frame: bytes, index: int) -> None:
    if index == 1:
      raise TimeoutError("always")

  result = evaluate(value, authorized_time=network_time(), send=send)
  assert result.disposition is NavigationDatabaseRestoreDisposition.WRITE_FAILED
  assert not result.database_available
  assert result.permanently_failed_indexes == (1,)


def test_write_failure_is_not_retried_after_process_restart(tmp_path: Path) -> None:
  first = runtime(tmp_path)
  attempts = []

  def fail(frame: bytes, index: int) -> None:
    attempts.append((frame, index))
    raise TimeoutError("failure")

  assert (
    evaluate(
      first,
      authorized_time=network_time(),
      send=fail,
    ).disposition
    is NavigationDatabaseRestoreDisposition.WRITE_FAILED
  )
  second = runtime(tmp_path)
  evaluate(second, authorized_time=network_time(), send=fail)
  assert len(attempts) == 2 * len(FRAMES)


def test_snapshot_identity_change_within_boot_fails_closed(tmp_path: Path) -> None:
  first = runtime(tmp_path, selected=snapshot(generation="primary"))
  first.prepare()
  second = runtime(tmp_path, selected=snapshot(generation="previous"))
  writes = []
  result = evaluate(
    second,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
  assert writes == []


def test_persistence_failure_prevents_database_write(tmp_path: Path) -> None:
  writes = []
  store_calls = 0

  def fail_after_initial_state(state, path):
    nonlocal store_calls
    store_calls += 1
    if store_calls >= 3:
      raise OSError("disk failure")
    store_navigation_database_restore_boot_state(state, path)

  value = runtime(tmp_path, state_storer=fail_after_initial_state)
  result = evaluate(
    value,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.WRITE_FAILED
  assert result.state_persistence_error is not None
  assert writes == []


def test_boot_id_unavailable_fails_closed(tmp_path: Path) -> None:
  value = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    state_path=tmp_path / "state.json",
    boot_id_reader=lambda: None,
  )
  writes = []
  result = evaluate(
    value,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
  assert writes == []


@pytest.mark.parametrize(
  "retry_delay_seconds",
  (-1.0, float("nan"), float("inf"), True, "0.25"),
)
def test_runtime_rejects_invalid_retry_delay(
  tmp_path: Path,
  retry_delay_seconds: object,
) -> None:
  with pytest.raises(ValueError):
    NavigationDatabaseRestoreRuntime(
      "receiver",
      snapshot_loader=lambda _fingerprint: snapshot(),
      retry_delay_seconds=retry_delay_seconds,  # type: ignore[arg-type]
      state_path=tmp_path / "state.json",
      boot_id_reader=lambda: BOOT_ID,
    )
