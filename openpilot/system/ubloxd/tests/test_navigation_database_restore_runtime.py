from datetime import UTC, datetime, timedelta
from pathlib import Path

import pytest

from openpilot.system.ubloxd.navigation_database_restore import (
  NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS,
  NavigationDatabaseRestoreDisposition,
)
import openpilot.system.ubloxd.navigation_database_restore_runtime as restore_runtime
from openpilot.system.ubloxd.navigation_database_restore_runtime import (
  NavigationDatabaseRestoreBootState,
  NavigationDatabaseRestoreCandidateIdentity,
  NavigationDatabaseRestoreFrameFailureKind,
  NavigationDatabaseRestoreFrozenCaches,
  NavigationDatabaseRestoreInitializationError,
  NavigationDatabaseRestoreRuntime,
  NavigationDatabaseRestoreSnapshot,
  PositionAssistanceAckStatus,
  PositionAssistanceFailureKind,
  PositionAssistanceWriteStatus,
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

from openpilot.system.ubloxd.yuma_almanac_transmit import (
  MgaReceiverNackError,
  MgaTransactionError,
  MgaWriteError,
)


NOW = datetime(2026, 7, 28, 20, 0, tzinfo=UTC)
BOOT_ID = "12345678-1234-5678-9234-567812345678"
OTHER_BOOT_ID = "87654321-4321-6789-9234-567812345678"
TEST_BOOTTIME_SECONDS = 100.0
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
    observed_boottime_seconds=TEST_BOOTTIME_SECONDS,
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
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )


def no_cache_runtime(tmp_path: Path) -> NavigationDatabaseRestoreRuntime:
  return NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: None,
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
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
    version=2,
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


def test_corrupt_state_is_quarantined_and_fails_closed(
  tmp_path: Path,
) -> None:
  path = tmp_path / "dbd_state.json"
  path.write_text("not-json", encoding="utf-8")

  value = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    state_path=path,
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )

  assert (
    value.controller.disposition
    is NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
  )

  persisted = load_navigation_database_restore_boot_state(path)
  assert persisted is not None
  assert (
    persisted.disposition
    is NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
  )

  quarantined = list(tmp_path.glob(f"{path.name}.invalid-*"))
  assert len(quarantined) == 1
  assert quarantined[0].read_text(encoding="utf-8") == "not-json"


def test_new_linux_boot_discards_old_state(tmp_path: Path) -> None:
  path = tmp_path / "dbd_state.json"
  old = NavigationDatabaseRestoreBootState(
    version=2,
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
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
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
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
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


def test_position_assistance_success_is_structured(tmp_path: Path) -> None:
  result = runtime(tmp_path).send_position_once(lambda _message: None)

  assert result.position_assistance_attempted
  assert result.position_assistance_succeeded
  assert result.position_assistance_message_id == 0x40
  assert result.position_assistance_message_type == 0x01
  assert (
    result.position_assistance_write_status
    is PositionAssistanceWriteStatus.SUCCEEDED
  )
  assert (
    result.position_assistance_ack_status
    is PositionAssistanceAckStatus.ACCEPTED
  )
  assert result.position_assistance_ack_info_code == 0
  assert result.position_assistance_failure_kind is None
  assert result.position_assistance_error_type is None
  assert result.position_assistance_error is None


@pytest.mark.parametrize(
  (
    "exception",
    "write_status",
    "ack_status",
    "failure_kind",
    "info_code",
  ),
  (
    (
      MgaReceiverNackError(
        "receiver not ready",
        message_id=0x40,
        message_type=0x01,
        ack_type=0,
        ack_version=0,
        info_code=5,
        rejected_message_id=0x40,
      ),
      PositionAssistanceWriteStatus.SUCCEEDED,
      PositionAssistanceAckStatus.REJECTED,
      PositionAssistanceFailureKind.ACK_REJECTED,
      5,
    ),
    (
      TimeoutError("position ACK timeout"),
      PositionAssistanceWriteStatus.SUCCEEDED,
      PositionAssistanceAckStatus.TIMED_OUT,
      PositionAssistanceFailureKind.ACK_TIMEOUT,
      None,
    ),
    (
      MgaWriteError(
        "position write failed",
        message_id=0x40,
        message_type=0x01,
      ),
      PositionAssistanceWriteStatus.FAILED,
      PositionAssistanceAckStatus.NOT_ATTEMPTED,
      PositionAssistanceFailureKind.WRITE,
      None,
    ),
    (
      MgaTransactionError(
        "position ACK observation failed",
        message_id=0x40,
        message_type=0x01,
        write_succeeded=True,
      ),
      PositionAssistanceWriteStatus.SUCCEEDED,
      PositionAssistanceAckStatus.OBSERVATION_FAILED,
      PositionAssistanceFailureKind.WRITE,
      None,
    ),
  ),
)
def test_position_assistance_failures_remain_structured(
  tmp_path: Path,
  exception: Exception,
  write_status: PositionAssistanceWriteStatus,
  ack_status: PositionAssistanceAckStatus,
  failure_kind: PositionAssistanceFailureKind,
  info_code: int | None,
) -> None:
  def fail(_message: bytes) -> None:
    raise exception

  result = runtime(tmp_path).send_position_once(fail)

  assert result.position_assistance_attempted
  assert not result.position_assistance_succeeded
  assert result.position_assistance_message_id == 0x40
  assert result.position_assistance_message_type == 0x01
  assert result.position_assistance_write_status is write_status
  assert result.position_assistance_ack_status is ack_status
  assert result.position_assistance_ack_info_code == info_code
  assert result.position_assistance_failure_kind is failure_kind
  assert result.position_assistance_error_type == type(exception).__name__
  assert str(exception) in (result.position_assistance_error or "")


def test_position_assistance_build_failure_is_structured(
  tmp_path: Path,
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  monkeypatch.setattr(
    restore_runtime,
    "build_position_assistance_message",
    lambda **_kwargs: (_ for _ in ()).throw(
      ValueError("position build failed")
    ),
  )

  result = runtime(tmp_path).send_position_once(
    lambda _message: pytest.fail("position message must not be written")
  )

  assert result.position_assistance_attempted
  assert not result.position_assistance_succeeded
  assert result.position_assistance_message_id is None
  assert result.position_assistance_message_type is None
  assert (
    result.position_assistance_write_status
    is PositionAssistanceWriteStatus.NOT_ATTEMPTED
  )
  assert (
    result.position_assistance_ack_status
    is PositionAssistanceAckStatus.NOT_ATTEMPTED
  )
  assert (
    result.position_assistance_failure_kind
    is PositionAssistanceFailureKind.BUILD
  )
  assert result.position_assistance_error_type == "ValueError"
  assert "position build failed" in (
    result.position_assistance_error or ""
  )


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
    selected=snapshot(NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS - 1.0),
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
    version=2,
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
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
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


def test_same_boot_continuity_keeps_restore_pending(tmp_path: Path) -> None:
  value = runtime(tmp_path)
  writes = []
  result = evaluate(
    value,
    authorized_time=same_boot_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.PENDING
  assert writes == []

  result = evaluate(
    value,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.RESTORED
  assert writes == [(FRAMES[0], 0), (FRAMES[1], 1)]


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


def test_boot_id_unavailable_aborts_initialization(
  tmp_path: Path,
) -> None:
  with pytest.raises(
    NavigationDatabaseRestoreInitializationError,
    match="boot_id_unavailable",
  ):
    NavigationDatabaseRestoreRuntime(
      "receiver",
      snapshot_loader=lambda _fingerprint: snapshot(),
      state_path=tmp_path / "state.json",
      boot_id_reader=lambda: None,
      boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
    )


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
      retry_delay_seconds=retry_delay_seconds,  # type: ignore[arg-type, ty:invalid-argument-type]
      state_path=tmp_path / "state.json",
      boot_id_reader=lambda: BOOT_ID,
      boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
    )


def frozen_caches(
  *,
  primary: NavigationDatabaseRestoreSnapshot | None,
  previous: NavigationDatabaseRestoreSnapshot | None,
  position: NavigationDatabaseRestoreSnapshot | None = None,
) -> NavigationDatabaseRestoreFrozenCaches:
  return NavigationDatabaseRestoreFrozenCaches(
    position_snapshot=position or primary or previous,
    primary_snapshot=primary,
    previous_snapshot=previous,
  )


def multi_runtime(
  tmp_path: Path,
  *,
  primary: NavigationDatabaseRestoreSnapshot | None,
  previous: NavigationDatabaseRestoreSnapshot | None,
) -> NavigationDatabaseRestoreRuntime:
  value = frozen_caches(primary=primary, previous=previous)
  return NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: value,
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )


def test_newer_eligible_primary_wins_over_expired_higher_quality_previous(
  tmp_path: Path,
) -> None:
  primary = snapshot(30 * 60, generation="primary")
  previous = snapshot(2 * 60 * 60, generation="previous")
  value = multi_runtime(tmp_path, primary=primary, previous=previous)

  result = evaluate(value, authorized_time=network_time())

  assert result.disposition is NavigationDatabaseRestoreDisposition.RESTORED
  assert result.cache_generation == "primary"
  assert result.cache_selection_reason == "trusted_age_only_eligible:primary"


def test_future_primary_falls_back_to_valid_previous(tmp_path: Path) -> None:
  primary = snapshot(-30.0, generation="primary")
  previous = snapshot(30 * 60, generation="previous")
  value = multi_runtime(tmp_path, primary=primary, previous=previous)

  result = evaluate(value, authorized_time=network_time())

  assert result.disposition is NavigationDatabaseRestoreDisposition.RESTORED
  assert result.cache_generation == "previous"


def test_both_generations_expired_skip_without_writes(tmp_path: Path) -> None:
  value = multi_runtime(
    tmp_path,
    primary=snapshot(2 * 60 * 60, generation="primary"),
    previous=snapshot(3 * 60 * 60, generation="previous"),
  )
  writes = []

  result = evaluate(
    value,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )

  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_EXPIRED
  assert writes == []


def test_exactly_one_generation_on_one_hour_boundary_is_selected(
  tmp_path: Path,
) -> None:
  value = multi_runtime(
    tmp_path,
    primary=snapshot(
      NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS - 1.0,
      generation="primary",
    ),
    previous=snapshot(
      NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS,
      generation="previous",
    ),
  )

  result = evaluate(value, authorized_time=network_time())

  assert result.disposition is NavigationDatabaseRestoreDisposition.RESTORED
  assert result.cache_generation == "primary"


def test_recovered_same_boot_pending_state_fails_closed(
  tmp_path: Path,
) -> None:
  first = multi_runtime(
    tmp_path,
    primary=snapshot(generation="primary"),
    previous=snapshot(45 * 60, generation="previous"),
  )
  first.prepare()

  second = multi_runtime(
    tmp_path,
    primary=snapshot(generation="primary"),
    previous=snapshot(45 * 60, generation="previous"),
  )
  writes: list[tuple[bytes, int]] = []
  result = evaluate(
    second,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )

  assert (
    result.disposition
    is NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
  )
  assert writes == []

  persisted = load_navigation_database_restore_boot_state(
    tmp_path / "dbd_state.json"
  )
  assert persisted is not None
  assert (
    persisted.disposition
    is NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
  )


def test_candidate_identity_round_trip() -> None:
  identity = NavigationDatabaseRestoreCandidateIdentity.from_snapshot(snapshot())
  assert NavigationDatabaseRestoreCandidateIdentity.from_json_dict(identity.to_json_dict()) == identity


@pytest.mark.parametrize(
  ("exception", "kind", "retried"),
  (
    (MgaReceiverNackError("nack"), NavigationDatabaseRestoreFrameFailureKind.REJECTED, False),
    (TimeoutError("timeout"), NavigationDatabaseRestoreFrameFailureKind.TIMED_OUT, True),
    (MgaWriteError("write"), NavigationDatabaseRestoreFrameFailureKind.WRITE_ERROR, True),
    (MgaTransactionError("transaction"), NavigationDatabaseRestoreFrameFailureKind.TRANSACTION_ERROR, True),
    (ValueError("unexpected"), NavigationDatabaseRestoreFrameFailureKind.UNEXPECTED_ERROR, False),
  ),
)
def test_typed_frame_failure_and_retry_policy(
  tmp_path: Path,
  exception: Exception,
  kind: NavigationDatabaseRestoreFrameFailureKind,
  retried: bool,
) -> None:
  value = runtime(tmp_path)
  attempts = 0

  def send(_frame: bytes, index: int) -> None:
    nonlocal attempts
    if index != 0:
      return
    attempts += 1
    raise exception

  result = evaluate(value, authorized_time=network_time(), send=send)

  assert result.disposition is NavigationDatabaseRestoreDisposition.WRITE_FAILED
  assert result.initial_failures[0].kind is kind
  assert attempts == (2 if retried else 1)


def test_retry_sleeper_failure_records_phase_and_error(tmp_path: Path) -> None:
  value = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.25,
    sleeper=lambda _delay: (_ for _ in ()).throw(RuntimeError("sleep failed")),
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )

  result = evaluate(
    value,
    authorized_time=network_time(),
    send=lambda _frame, _index: (_ for _ in ()).throw(TimeoutError("timeout")),
  )

  assert result.disposition is NavigationDatabaseRestoreDisposition.WRITE_FAILED
  assert result.failure_phase == "retry_delay"
  assert result.execution_error == "RuntimeError:sleep failed"


def test_acquisition_during_pre_database_configuration_closes_window(
  tmp_path: Path,
) -> None:
  value = runtime(tmp_path)
  writes: list[tuple[bytes, int]] = []

  # Models RAWX/NAV-SAT dispatched by a synchronous MON-VER or NAVX5
  # transaction before initialize_receiver_cycle reaches its DBD decision.
  value.note_acquisition_started()
  result = evaluate(
    value,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )

  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_ACQUISITION_ALREADY_STARTED
  assert writes == []
  assert value.controller.terminal


# COMMIT6_DBD_SAFETY_TESTS

def test_conservative_age_includes_uncertainty_and_elapsed_time(
  tmp_path: Path,
) -> None:
  value = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(
      NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS - 15.0
    ),
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: 105.0,
  )
  authorized = AuthorizedTime(
    utc=NOW,
    uncertainty_seconds=10.0,
    source=TrustedTimeSource.SYSTEM_SYNCHRONIZED,
    provenance=TimeProvenance.NETWORK_INDEPENDENT,
    independent=True,
    evidence=TimeAuthorizationEvidence.SYSTEM_SYNCHRONIZED,
    observed_boottime_seconds=100.0,
  )
  result = evaluate(value, authorized_time=authorized)
  assert result.disposition is NavigationDatabaseRestoreDisposition.RESTORED
  assert result.cache_age_seconds == NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS


def test_conservative_age_one_second_over_boundary_skips(
  tmp_path: Path,
) -> None:
  writes: list[tuple[bytes, int]] = []
  value = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(
      NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS - 15.0
    ),
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: 106.0,
  )
  authorized = AuthorizedTime(
    utc=NOW,
    uncertainty_seconds=10.0,
    source=TrustedTimeSource.SYSTEM_SYNCHRONIZED,
    provenance=TimeProvenance.NETWORK_INDEPENDENT,
    independent=True,
    evidence=TimeAuthorizationEvidence.SYSTEM_SYNCHRONIZED,
    observed_boottime_seconds=100.0,
  )
  result = evaluate(
    value,
    authorized_time=authorized,
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_EXPIRED
  assert result.cache_age_seconds == NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS + 1.0
  assert writes == []


def test_acquisition_claim_failure_is_reported_before_receiver_start(
  tmp_path: Path,
) -> None:
  calls = 0
  def fail_claim(state, path):
    nonlocal calls
    calls += 1
    if calls >= 2:
      raise OSError("disk failure")
    store_navigation_database_restore_boot_state(state, path)

  value = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: None,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    state_storer=fail_claim,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )
  assert not value.claim_acquisition_start()
  assert value.acquisition_started
  assert value.execution.state_persistence_error is not None


def test_yuma_is_durably_claimed_before_restart(tmp_path: Path) -> None:
  first = runtime(tmp_path)
  assert first.claim_yuma_transmission()
  second = runtime(tmp_path)
  writes: list[tuple[bytes, int]] = []
  result = evaluate(
    second,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_YUMA_ALREADY_SENT
  assert writes == []

# COMMIT7_DBD_FRAME_BOUNDARY_TESTS

def test_missing_observation_boottime_fails_closed(tmp_path: Path) -> None:
  writes: list[tuple[bytes, int]] = []
  value = runtime(tmp_path)
  authorized = AuthorizedTime(
    utc=NOW,
    uncertainty_seconds=1.0,
    source=TrustedTimeSource.SYSTEM_SYNCHRONIZED,
    provenance=TimeProvenance.NETWORK_INDEPENDENT,
    independent=True,
    evidence=TimeAuthorizationEvidence.SYSTEM_SYNCHRONIZED,
    observed_boottime_seconds=None,
  )
  result = evaluate(value, authorized_time=authorized, send=lambda frame, index: writes.append((frame, index)))
  assert result.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
  assert writes == []


def test_cache_age_is_rechecked_after_restore_claim_before_frame_zero(tmp_path: Path) -> None:
  boottimes = [TEST_BOOTTIME_SECONDS, TEST_BOOTTIME_SECONDS + 2.0]
  def read_boottime() -> float:
    return boottimes.pop(0) if boottimes else TEST_BOOTTIME_SECONDS + 2.0
  value = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(NAVIGATION_DATABASE_RESTORE_MAX_AGE_SECONDS - 1.0),
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=read_boottime,
  )
  authorized = AuthorizedTime(
    utc=NOW,
    uncertainty_seconds=0.0,
    source=TrustedTimeSource.SYSTEM_SYNCHRONIZED,
    provenance=TimeProvenance.NETWORK_INDEPENDENT,
    independent=True,
    evidence=TimeAuthorizationEvidence.SYSTEM_SYNCHRONIZED,
    observed_boottime_seconds=TEST_BOOTTIME_SECONDS,
  )
  receiver_writes: list[tuple[bytes, int]] = []
  def guarded_send(frame: bytes, index: int) -> None:
    value.validate_database_write_boundary(index)
    receiver_writes.append((frame, index))
  result = evaluate(value, authorized_time=authorized, send=guarded_send)
  assert result.disposition is NavigationDatabaseRestoreDisposition.WRITE_FAILED
  assert receiver_writes == []
  assert result.permanent_failures
  assert all(failure.kind is NavigationDatabaseRestoreFrameFailureKind.VALIDATION_ERROR for failure in result.permanent_failures)

# COMMIT8_DBD_PENDING_RESTART_TEST

def test_failed_acquisition_persistence_cannot_reopen_after_restart(
  tmp_path: Path,
) -> None:
  path = tmp_path / "dbd_state.json"

  def fail_acquisition_state(
    state: NavigationDatabaseRestoreBootState,
    state_path: Path,
  ) -> None:
    if state.acquisition_started:
      raise OSError("disk failure")
    store_navigation_database_restore_boot_state(state, state_path)

  first = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=path,
    boot_id_reader=lambda: BOOT_ID,
    state_storer=fail_acquisition_state,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )
  assert not first.claim_acquisition_start()
  assert first.acquisition_started

  stale = load_navigation_database_restore_boot_state(path)
  assert stale is not None
  assert stale.disposition is NavigationDatabaseRestoreDisposition.PENDING
  assert not stale.acquisition_started

  writes: list[tuple[bytes, int]] = []
  second = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=path,
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )
  result = evaluate(
    second,
    authorized_time=network_time(),
    send=lambda frame, index: writes.append((frame, index)),
  )

  assert (
    result.disposition
    is NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
  )
  assert writes == []

  recovered = load_navigation_database_restore_boot_state(path)
  assert recovered is not None
  assert (
    recovered.disposition
    is NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
  )

# COMMIT9_COMPLETE_DURABLE_BOOT_BASELINE_TESTS


def test_boot_id_reader_exception_aborts_initialization(
  tmp_path: Path,
) -> None:
  def fail_boot_id() -> str:
    raise OSError("boot identity unavailable")

  with pytest.raises(
    NavigationDatabaseRestoreInitializationError,
    match="boot_id_read_failed",
  ):
    NavigationDatabaseRestoreRuntime(
      "receiver",
      snapshot_loader=lambda _fingerprint: snapshot(),
      state_path=tmp_path / "state.json",
      boot_id_reader=fail_boot_id,
      boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
    )


def test_state_loader_exception_aborts_without_overwriting_state(
  tmp_path: Path,
) -> None:
  path = tmp_path / "dbd_state.json"
  path.write_text("unreadable-state-sentinel", encoding="utf-8")

  def fail_load(_path: Path) -> NavigationDatabaseRestoreBootState | None:
    raise OSError("state unavailable")

  with pytest.raises(
    NavigationDatabaseRestoreInitializationError,
    match="state_load_failed",
  ):
    NavigationDatabaseRestoreRuntime(
      "receiver",
      snapshot_loader=lambda _fingerprint: snapshot(),
      state_path=path,
      boot_id_reader=lambda: BOOT_ID,
      state_loader=fail_load,
      boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
    )

  assert path.read_text(encoding="utf-8") == "unreadable-state-sentinel"


def test_invalid_state_loader_result_aborts_initialization(
  tmp_path: Path,
) -> None:
  with pytest.raises(
    NavigationDatabaseRestoreInitializationError,
    match="state_load_returned_invalid_type",
  ):
    NavigationDatabaseRestoreRuntime(
      "receiver",
      snapshot_loader=lambda _fingerprint: snapshot(),
      state_path=tmp_path / "state.json",
      boot_id_reader=lambda: BOOT_ID,
      state_loader=lambda _path: object(),  # type: ignore[arg-type, return-value, ty:invalid-argument-type]
      boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
    )


def test_missing_state_baseline_write_failure_aborts_initialization(
  tmp_path: Path,
) -> None:
  path = tmp_path / "dbd_state.json"

  def fail_write(
    _state: NavigationDatabaseRestoreBootState,
    _path: Path,
  ) -> None:
    raise OSError("storage unavailable")

  with pytest.raises(
    NavigationDatabaseRestoreInitializationError,
    match="current_boot_baseline_persist_failed",
  ):
    NavigationDatabaseRestoreRuntime(
      "receiver",
      snapshot_loader=lambda _fingerprint: snapshot(),
      state_path=path,
      boot_id_reader=lambda: BOOT_ID,
      state_storer=fail_write,
      boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
    )

  assert not path.exists()


def test_previous_boot_replacement_failure_aborts_initialization(
  tmp_path: Path,
) -> None:
  path = tmp_path / "dbd_state.json"
  previous = NavigationDatabaseRestoreBootState(
    version=2,
    boot_id=OTHER_BOOT_ID,
    receiver_fingerprint="receiver",
    disposition=NavigationDatabaseRestoreDisposition.SKIPPED_EXPIRED,
    restore_attempted=False,
    position_assistance_claimed=True,
    acquisition_started=True,
    yuma_sent=True,
  )
  store_navigation_database_restore_boot_state(previous, path)

  def fail_write(
    _state: NavigationDatabaseRestoreBootState,
    _path: Path,
  ) -> None:
    raise OSError("storage unavailable")

  with pytest.raises(
    NavigationDatabaseRestoreInitializationError,
    match="current_boot_baseline_persist_failed",
  ):
    NavigationDatabaseRestoreRuntime(
      "receiver",
      snapshot_loader=lambda _fingerprint: snapshot(),
      state_path=path,
      boot_id_reader=lambda: BOOT_ID,
      state_storer=fail_write,
      boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
    )

  assert load_navigation_database_restore_boot_state(path) == previous


def test_storage_recovery_later_process_establishes_fresh_baseline(
  tmp_path: Path,
) -> None:
  path = tmp_path / "dbd_state.json"
  storage_available = False

  def conditional_write(
    state: NavigationDatabaseRestoreBootState,
    state_path: Path,
  ) -> None:
    if not storage_available:
      raise OSError("storage unavailable")
    store_navigation_database_restore_boot_state(state, state_path)

  with pytest.raises(NavigationDatabaseRestoreInitializationError):
    NavigationDatabaseRestoreRuntime(
      "receiver",
      snapshot_loader=lambda _fingerprint: snapshot(),
      state_path=path,
      boot_id_reader=lambda: BOOT_ID,
      state_storer=conditional_write,
      boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
    )
  assert not path.exists()

  storage_available = True
  recovered = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    state_path=path,
    boot_id_reader=lambda: BOOT_ID,
    state_storer=conditional_write,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )

  assert recovered.controller.pending
  persisted = load_navigation_database_restore_boot_state(path)
  assert persisted is not None
  assert persisted.boot_id == BOOT_ID
  assert persisted.receiver_fingerprint == "receiver"
  assert persisted.disposition is NavigationDatabaseRestoreDisposition.PENDING


def test_previous_boot_replacement_succeeds_after_storage_recovers(
  tmp_path: Path,
) -> None:
  path = tmp_path / "dbd_state.json"
  previous = NavigationDatabaseRestoreBootState(
    version=2,
    boot_id=OTHER_BOOT_ID,
    receiver_fingerprint="receiver",
    disposition=NavigationDatabaseRestoreDisposition.SKIPPED_EXPIRED,
    restore_attempted=False,
    position_assistance_claimed=True,
    acquisition_started=True,
    yuma_sent=True,
  )
  store_navigation_database_restore_boot_state(previous, path)
  storage_available = False

  def conditional_write(
    state: NavigationDatabaseRestoreBootState,
    state_path: Path,
  ) -> None:
    if not storage_available:
      raise OSError("storage unavailable")
    store_navigation_database_restore_boot_state(state, state_path)

  with pytest.raises(NavigationDatabaseRestoreInitializationError):
    NavigationDatabaseRestoreRuntime(
      "receiver",
      snapshot_loader=lambda _fingerprint: snapshot(),
      state_path=path,
      boot_id_reader=lambda: BOOT_ID,
      state_storer=conditional_write,
      boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
    )
  assert load_navigation_database_restore_boot_state(path) == previous

  storage_available = True
  recovered = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    state_path=path,
    boot_id_reader=lambda: BOOT_ID,
    state_storer=conditional_write,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )

  assert recovered.controller.pending
  persisted = load_navigation_database_restore_boot_state(path)
  assert persisted is not None
  assert persisted.boot_id == BOOT_ID
  assert persisted.disposition is NavigationDatabaseRestoreDisposition.PENDING
