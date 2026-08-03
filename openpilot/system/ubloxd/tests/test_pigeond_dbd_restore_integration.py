from __future__ import annotations

from datetime import UTC, datetime, timedelta
from pathlib import Path
from types import SimpleNamespace
from typing import cast

import pytest

from openpilot.system.ubloxd import pigeond
import openpilot.system.ubloxd.navigation_database_restore_runtime as restore_runtime
from openpilot.system.ubloxd.navigation_database_restore import (
  NavigationDatabaseRestoreDisposition,
)
from openpilot.system.ubloxd.navigation_database_restore_runtime import (
  NavigationDatabaseRestoreExecution,
  NavigationDatabaseRestoreInitializationError,
  NavigationDatabaseRestoreRuntime,
  NavigationDatabaseRestoreSnapshot,
  PositionAssistanceAckStatus,
  PositionAssistanceFailureKind,
  PositionAssistanceWriteStatus,
)
from openpilot.system.ubloxd.position_assistance_retry import (
  PositionAssistanceRetryState,
  PositionAssistanceRetryStateError,
)
from openpilot.system.ubloxd.trusted_time_anchor import (
  TimeProvenance,
  TrustedTimeSource,
)
from openpilot.system.ubloxd.trusted_time_authority import (
  AuthorizedTime,
  TimeAuthorizationEvidence,
)


BOOT_ID = "12345678-1234-5678-9234-567812345678"
NOW = datetime(2026, 7, 29, 13, 0, tzinfo=UTC)
TEST_BOOTTIME_SECONDS = 100.0


class FakePigeon:
  def __init__(self, events: list[str] | None = None) -> None:
    self.sent: list[bytes] = []
    self.events = events

  def send(self, message: bytes) -> None:
    self.sent.append(message)
    if self.events is not None:
      if message == pigeond.CONTROLLED_GNSS_STOP_MESSAGE:
        self.events.append("gnss_stop")
      elif message == pigeond.CONTROLLED_GNSS_START_MESSAGE:
        self.events.append("gnss_start")


class FakeDiagnostics:
  cycle_number = 0

  def start_cycle(self, _reason: str, _now: float) -> None:
    self.cycle_number += 1

  def time_assistance_context(self, _now: float) -> str:
    return "test"


class FakeProvenance:
  cycle_id = 0

  def start_cycle(
    self,
    cycle_id: int,
    _now: float,
    *,
    observations_enabled: bool,
  ) -> None:
    assert not observations_enabled
    self.cycle_id = cycle_id

  def enable_receiver_observations(self, _now: float) -> None:
    pass


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


def snapshot() -> NavigationDatabaseRestoreSnapshot:
  return NavigationDatabaseRestoreSnapshot(
    saved_at_utc=NOW - timedelta(minutes=30),
    database_frames=(b"database-frame",),
    latitude_e7=320_000_000,
    longitude_e7=-960_000_000,
    altitude_cm=20_000,
    position_accuracy_cm=10_000,
    quality=None,
    generation="primary",
    selection_reason="test",
  )


@pytest.mark.parametrize(
  ("failure_kind", "expected_phase"),
  (
    (
      PositionAssistanceFailureKind.BUILD,
      pigeond.NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_BUILD,
    ),
    (
      PositionAssistanceFailureKind.WRITE,
      pigeond.NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_WRITE,
    ),
    (
      PositionAssistanceFailureKind.ACK_REJECTED,
      pigeond.NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_ACK_REJECTED,
    ),
    (
      PositionAssistanceFailureKind.ACK_TIMEOUT,
      pigeond.NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_ACK_TIMEOUT,
    ),
    (
      PositionAssistanceFailureKind.ACK_OBSERVATION_FAILED,
      pigeond.NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_ACK_OBSERVATION_FAILED,
    ),
  ),
)
def test_position_failure_kind_survives_runtime_result_mapping(
  failure_kind: PositionAssistanceFailureKind,
  expected_phase: pigeond.NavigationAssistanceRestoreFailurePhase,
) -> None:
  execution = NavigationDatabaseRestoreExecution(
    disposition=NavigationDatabaseRestoreDisposition.RESTORED,
    total_frame_count=70,
    accepted_frame_count=70,
    database_write_attempt_count=70,
    position_assistance_attempted=True,
    position_assistance_succeeded=False,
    position_assistance_message_id=0x40,
    position_assistance_message_type=0x01,
    position_assistance_write_status=(
      PositionAssistanceWriteStatus.SUCCEEDED
      if failure_kind in (
        PositionAssistanceFailureKind.ACK_REJECTED,
        PositionAssistanceFailureKind.ACK_TIMEOUT,
        PositionAssistanceFailureKind.ACK_OBSERVATION_FAILED,
      )
      else PositionAssistanceWriteStatus.FAILED
    ),
    position_assistance_ack_status=(
      PositionAssistanceAckStatus.REJECTED
      if failure_kind is PositionAssistanceFailureKind.ACK_REJECTED
      else (
        PositionAssistanceAckStatus.TIMED_OUT
        if failure_kind is PositionAssistanceFailureKind.ACK_TIMEOUT
        else (
          PositionAssistanceAckStatus.OBSERVATION_FAILED
          if failure_kind is PositionAssistanceFailureKind.ACK_OBSERVATION_FAILED
          else PositionAssistanceAckStatus.NOT_ATTEMPTED
        )
      )
    ),
    position_assistance_ack_info_code=(
      5
      if failure_kind is PositionAssistanceFailureKind.ACK_REJECTED
      else None
    ),
    position_assistance_failure_kind=failure_kind,
    position_assistance_error_type="InjectedPositionError",
    position_assistance_error="InjectedPositionError:receiver detail",
  )

  result = (
    pigeond.navigation_assistance_result_from_database_execution(
      execution
    )
  )
  summary = pigeond.format_navigation_assistance_restore_summary(
    result,
    attempted=True,
    time_assistance_source="system_synchronized",
  )

  assert result.failure_phase is expected_phase
  assert result.position_assistance_attempted
  assert not result.position_assistance_succeeded
  assert result.position_assistance_message_id == 0x40
  assert result.position_assistance_message_type == 0x01
  assert (
    result.position_assistance_error_type
    == "InjectedPositionError"
  )
  assert (
    result.position_assistance_error
    == "InjectedPositionError:receiver detail"
  )
  assert f"failure_phase={expected_phase.value}" in summary
  assert "position_assistance_attempted=true" in summary
  assert "position_assistance_succeeded=false" in summary
  assert "position_assistance_message_id=0x40" in summary
  assert "position_assistance_message_type=0x01" in summary
  assert (
    "position_assistance_error_type=InjectedPositionError"
    in summary
  )
  assert (
    "position_assistance_error=InjectedPositionError:receiver detail"
    in summary
  )


def test_position_nack_ack_detail_survives_restore_log() -> None:
  execution = NavigationDatabaseRestoreExecution(
    disposition=NavigationDatabaseRestoreDisposition.RESTORED,
    total_frame_count=70,
    accepted_frame_count=70,
    database_write_attempt_count=70,
    position_assistance_attempted=True,
    position_assistance_succeeded=False,
    position_assistance_message_id=0x40,
    position_assistance_message_type=0x01,
    position_assistance_write_status=(
      PositionAssistanceWriteStatus.SUCCEEDED
    ),
    position_assistance_ack_status=(
      PositionAssistanceAckStatus.REJECTED
    ),
    position_assistance_ack_info_code=5,
    position_assistance_failure_kind=(
      PositionAssistanceFailureKind.ACK_REJECTED
    ),
    position_assistance_error_type="MgaReceiverNackError",
    position_assistance_error=(
      "MgaReceiverNackError:u-blox rejected MGA message: ack_infoCode=5"
    ),
  )

  result = (
    pigeond.navigation_assistance_result_from_database_execution(
      execution
    )
  )
  summary = pigeond.format_navigation_assistance_restore_summary(
    result,
    attempted=True,
    time_assistance_source="system_synchronized",
  )

  assert (
    result.failure_phase
    is pigeond.NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_ACK_REJECTED
  )
  assert (
    result.position_assistance_ack_status
    is PositionAssistanceAckStatus.REJECTED
  )
  assert result.position_assistance_ack_info_code == 5
  assert "position_assistance_ack_status=rejected" in summary
  assert "position_assistance_ack_info_code=5" in summary
  assert "ack_infoCode=5" in summary


def test_startup_timeline_formats_correlated_fields() -> None:
  restore = pigeond.NavigationAssistanceRestoreResult(
    status=pigeond.NavigationAssistanceRestoreStatus.PARTIAL,
    total_frame_count=3,
    accepted_frame_count=2,
    initially_timed_out_indexes=(2,),
    permanently_rejected_indexes=(1,),
    permanently_timed_out_indexes=(2,),
    restored_cache_generation="primary",
    restored_cache_selection_reason=(
      "trusted_age_only_eligible:primary"
    ),
    restored_cache_age_seconds=1800.0,
    database_restore_disposition=(
      NavigationDatabaseRestoreDisposition.RESTORED
    ),
    database_frames_attempted_count=3,
  )

  time_attempt = pigeond.TimeAssistanceAttemptDiagnostic(
    attempted_at=44.6,
    written_at=44.7,
    ack_observed_at=44.8,
    write_status=pigeond.TimeAssistanceWriteStatus.SUCCEEDED,
    ack_status=pigeond.TimeAssistanceAckStatus.REJECTED,
    ack_info_code=5,
    accepted_at=None,
    message_id=0x40,
    message_type=0x10,
    source="system_synchronized",
    correction=False,
    diagnostic_context="cycle=2, reason=process_start",
  )

  message = pigeond.format_gps_startup_timeline(
    cycle=2,
    reason="process_start",
    cycle_started_at=10.0,
    trusted_time_wait_started_at=12.0,
    trusted_time_wait_completed_at=44.5,
    independent_network_time_seen_at=44.5,
    acquisition_start_claimed_at=45.0,
    gnss_start_sent_at=45.1,
    restore_result=restore,
    authorized_time=network_time(),
    time_assistance_attempts=(time_attempt,),
  )

  assert "GPS startup timeline" in message
  assert "cycle=2" in message
  assert "reason=process_start" in message
  assert (
    "trusted_time_wait_started_cycle_seconds=2.000"
    in message
  )
  assert (
    "trusted_time_wait_completed_cycle_seconds=34.500"
    in message
  )
  assert "trusted_time_wait_duration_seconds=32.500" in message
  assert (
    "independent_network_time_first_seen_cycle_seconds=34.500"
    in message
  )
  assert "trusted_time_available=true" in message
  assert "database_restore_disposition=restored" in message
  assert "restored_cache_generation=primary" in message
  assert (
    "restored_cache_selection_reason=trusted_age_only_eligible:primary"
    in message
  )
  assert "restored_cache_age_seconds=1800.0" in message
  assert "database_frames_attempted=3" in message
  assert "database_frames_accepted=2" in message
  assert "database_frames_rejected=1" in message
  assert "database_timeout_events=2" in message
  assert "time_assistance_attempted_cycle_seconds=34.600" in message
  assert "time_assistance_written_cycle_seconds=34.700" in message
  assert (
    "time_assistance_ack_observed_cycle_seconds=34.800"
    in message
  )
  assert "time_assistance_write_status=succeeded" in message
  assert "time_assistance_ack_status=rejected" in message
  assert "time_assistance_ack_info_code=5" in message
  assert "time_assistance_accepted_cycle_seconds=none" in message
  assert (
    "time_assistance_accepted_before_gnss_start=false"
    in message
  )
  assert (
    "acquisition_start_claimed_cycle_seconds=35.000"
    in message
  )
  assert "gnss_start_sent_cycle_seconds=35.100" in message
  assert (
    "related_acquisition_milestones=first_nonempty_rawx|first_fix_ok|first_reliable_fix"
    in message
  )


def test_startup_timeline_rejects_incomplete_time_stub() -> None:
  incomplete = SimpleNamespace(
    utc=NOW,
    evidence=SimpleNamespace(value="system_synchronized"),
    mga_accuracy_seconds=30,
    independent=True,
    provenance=SimpleNamespace(value="network_independent"),
    observed_boottime_seconds=100.0,
  )

  assert not pigeond._startup_timeline_has_current_network_time(
    incomplete
  )
  assert pigeond._startup_timeline_has_current_network_time(
    network_time()
  )


def test_paused_acquisition_records_start_timestamp(
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  pigeon = FakePigeon()
  monkeypatch.setattr(pigeond.time, "sleep", lambda _delay: None)
  monkeypatch.setattr(pigeond.time, "monotonic", lambda: 12.5)

  with pigeond.install_pre_acquisition_initialization(
    lambda: None
  ) as initialization:
    with pigeond.paused_gnss_acquisition(
      pigeon  # type: ignore[arg-type, ty:invalid-argument-type]
    ):
      initialization.run()

  assert initialization.gnss_start_sent_at == 12.5
  assert (
    pigeon.sent[-1]
    == pigeond.CONTROLLED_GNSS_START_MESSAGE
  )


def test_paused_gnss_acquisition_stays_stopped_after_failure(
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  pigeon = FakePigeon()
  monkeypatch.setattr(pigeond.time, "sleep", lambda _delay: None)

  with pytest.raises(RuntimeError, match="simulated setup failure"):
    with pigeond.paused_gnss_acquisition(pigeon):  # type: ignore[arg-type, ty:invalid-argument-type]
      raise RuntimeError("simulated setup failure")

  assert pigeon.sent == [pigeond.CONTROLLED_GNSS_STOP_MESSAGE]


def test_configuration_traffic_closes_database_window_before_write(
  tmp_path: Path,
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  events: list[str] = []
  pigeon = FakePigeon(events)
  runtime = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )
  database_indexes: list[int] = []

  monkeypatch.setattr(pigeond.time, "sleep", lambda _delay: None)
  monkeypatch.setattr(pigeond, "start_pigeon_transport", lambda _pigeon: None)
  monkeypatch.setattr(pigeond, "read_host_time_observation", lambda: None)
  monkeypatch.setattr(
    pigeond,
    "evaluate_time_authority",
    lambda _authority, _observation: SimpleNamespace(authorized_time=network_time()),
  )
  monkeypatch.setattr(pigeond, "poll_mon_ver", lambda _pigeon: None)
  monkeypatch.setattr(
    pigeond,
    "configure_navx5_ack_aiding",
    lambda _pigeon, _info: (
      events.append("configuration_traffic"),
      runtime.note_acquisition_started(),
    ),
  )
  monkeypatch.setattr(pigeond, "log_navx5_ack_aiding_support", lambda _info: None)
  monkeypatch.setattr(
    pigeond,
    "send_mga_with_strict_ack",
    lambda _pigeon, _message, **kwargs: database_indexes.append(kwargs["database_frame_index"]) if kwargs.get("database_frame_index") is not None else None,
  )
  monkeypatch.setattr(pigeond, "send_time_assistance", lambda *_args, **_kwargs: False)
  monkeypatch.setattr(
    pigeond,
    "log_navigation_assistance_restore_result",
    lambda *_args, **_kwargs: None,
  )
  monkeypatch.setattr(
    pigeond,
    "finish_pigeon_initialization",
    lambda _pigeon: events.append("normal_configuration"),
  )
  monkeypatch.setattr(
    pigeond,
    "log_assistnow_autonomous_support",
    lambda _info: True,
  )
  monkeypatch.setattr(
    pigeond,
    "configure_assistnow_autonomous",
    lambda _pigeon, _info: None,
  )

  result = pigeond.initialize_receiver_cycle(
    pigeon,  # type: ignore[arg-type, ty:invalid-argument-type]
    "receiver",
    FakeDiagnostics(),  # type: ignore[arg-type, ty:invalid-argument-type]
    "test",
    time_authority=object(),  # type: ignore[arg-type, ty:invalid-argument-type]
    time_provenance=FakeProvenance(),  # type: ignore[arg-type, ty:invalid-argument-type]
    navigation_database_runtime=runtime,
  )

  assert runtime.controller.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_ACQUISITION_ALREADY_STARTED
  assert database_indexes == []
  assert events.index("gnss_stop") < events.index("configuration_traffic")
  assert events.index("configuration_traffic") < events.index("gnss_start")
  assert events.index("gnss_start") < events.index("normal_configuration")
  assert result.navigation_assistance_restore_attempted


# COMMIT6_PIGEOND_SAFETY_TESTS

def test_bounded_wait_accepts_delayed_network_time() -> None:
  clock = [0.0]
  sleeps: list[float] = []
  evaluations = [SimpleNamespace(authorized_time=None), SimpleNamespace(authorized_time=network_time())]
  def monotonic() -> float:
    return clock[0]
  def sleeper(delay: float) -> None:
    sleeps.append(delay)
    clock[0] += delay
  def evaluator(_authority, _observation):
    return evaluations.pop(0)

  observation, evaluation = pigeond.wait_for_current_independent_network_time(
    object(),  # type: ignore[arg-type, ty:invalid-argument-type]
    None,
    SimpleNamespace(authorized_time=None),  # type: ignore[arg-type, ty:invalid-argument-type]
    timeout_seconds=1.0,
    poll_seconds=0.25,
    observation_reader=lambda: None,
    evaluator=evaluator,  # type: ignore[arg-type]
    monotonic=monotonic,
    sleeper=sleeper,
  )
  assert observation is None
  assert evaluation.authorized_time == network_time()
  assert sleeps == [0.25, 0.25]



def test_default_wait_covers_observed_c4_network_sync_delay() -> None:
  clock = [0.0]
  sleeps: list[float] = []

  def monotonic() -> float:
    return clock[0]

  def sleeper(delay: float) -> None:
    sleeps.append(delay)
    clock[0] += delay

  def evaluator(_authority, _observation):
    return SimpleNamespace(
      authorized_time=network_time() if clock[0] >= 32.5 else None
    )

  observation, evaluation = (
    pigeond.wait_for_current_independent_network_time(
      object(),  # type: ignore[arg-type, ty:invalid-argument-type]
      None,
      SimpleNamespace(authorized_time=None),  # type: ignore[arg-type, ty:invalid-argument-type]
      observation_reader=lambda: None,
      evaluator=evaluator,  # type: ignore[arg-type, ty:invalid-argument-type]
      monotonic=monotonic,
      sleeper=sleeper,
    )
  )

  assert (
    pigeond.NAVIGATION_DATABASE_TRUSTED_TIME_WAIT_SECONDS
    == 40.0
  )
  assert observation is None
  assert evaluation.authorized_time == network_time()
  assert clock[0] == pytest.approx(32.5)
  assert sum(sleeps) == pytest.approx(32.5)


def test_default_wait_remains_bounded_without_network_time() -> None:
  clock = [0.0]
  sleeps: list[float] = []

  def monotonic() -> float:
    return clock[0]

  def sleeper(delay: float) -> None:
    sleeps.append(delay)
    clock[0] += delay

  def evaluator(_authority, _observation):
    return SimpleNamespace(authorized_time=None)

  observation, evaluation = (
    pigeond.wait_for_current_independent_network_time(
      object(),  # type: ignore[arg-type, ty:invalid-argument-type]
      None,
      SimpleNamespace(authorized_time=None),  # type: ignore[arg-type, ty:invalid-argument-type]
      observation_reader=lambda: None,
      evaluator=evaluator,  # type: ignore[arg-type, ty:invalid-argument-type]
      monotonic=monotonic,
      sleeper=sleeper,
    )
  )

  assert observation is None
  assert evaluation.authorized_time is None
  assert clock[0] == pytest.approx(40.0)
  assert sum(sleeps) == pytest.approx(40.0)
  assert sleeps
  assert all(
    0.0 < delay
    <= pigeond.NAVIGATION_DATABASE_TRUSTED_TIME_POLL_SECONDS
    for delay in sleeps
  )



def test_yuma_claim_happens_before_receiver_write() -> None:
  events: list[str] = []
  runtime = SimpleNamespace(
    claim_yuma_transmission=lambda: events.append("claim") or True
  )
  pigeond.send_yuma_with_durable_claim(
    runtime,  # type: ignore[arg-type, ty:invalid-argument-type]
    lambda _message: events.append("write"),
    b"yuma",
  )
  assert events == ["claim", "write"]


def test_failed_yuma_claim_performs_zero_receiver_writes() -> None:
  writes: list[bytes] = []
  runtime = SimpleNamespace(claim_yuma_transmission=lambda: False)
  with pytest.raises(pigeond.YumaAssistanceStateUnavailableError):
    pigeond.send_yuma_with_durable_claim(
      runtime,  # type: ignore[arg-type, ty:invalid-argument-type]
      writes.append,
      b"yuma",
    )
  assert writes == []


def test_post_power_stop_precedes_boot_wait(
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  events: list[str] = []
  pigeon = FakePigeon(events)
  monkeypatch.setattr(pigeond.signal, "signal", lambda *_args: None)
  monkeypatch.setattr(
    pigeond,
    "set_power",
    lambda enabled: events.append("power_on" if enabled else "power_off"),
  )
  monkeypatch.setattr(pigeond.time, "sleep", lambda _delay: events.append("sleep"))
  monkeypatch.setattr(pigeond, "init_baudrate", lambda _pigeon: events.append("init_baudrate"))
  monkeypatch.setattr(
    pigeond,
    "poll_mon_ver",
    lambda _pigeon, _timeout: SimpleNamespace(),
  )
  pigeond.start_pigeon_transport(pigeon)  # type: ignore[arg-type, ty:invalid-argument-type]
  assert events.index("power_on") < events.index("gnss_stop")
  assert events.index("gnss_stop") < events.index("sleep", events.index("power_on"))
  assert events.index("gnss_stop") < events.index("init_baudrate")


def test_delayed_network_time_skips_dbd_for_early_start(
  tmp_path: Path,
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  events: list[str] = []
  pigeon = FakePigeon(events)
  def guarded_drain(
    _self,
    operation: str,
  ) -> None:
    if operation == "navigation_database_post_time_wait":
      raise AssertionError(
        "early acquisition must not execute the obsolete DBD drain"
      )
    events.append(operation)

  monkeypatch.setattr(
    type(pigeon),
    "drain_before_transaction",
    guarded_drain,
    raising=False,
  )
  runtime = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )
  original_claim_acquisition_start = (
    runtime.claim_acquisition_start
  )

  def claim_acquisition_start() -> bool:
    events.append("acquisition_start_claim")
    return original_claim_acquisition_start()

  monkeypatch.setattr(
    runtime,
    "claim_acquisition_start",
    claim_acquisition_start,
  )
  database_indexes: list[int] = []
  monkeypatch.setattr(pigeond.time, "sleep", lambda _delay: None)
  monkeypatch.setattr(pigeond, "start_pigeon_transport", lambda _pigeon: None)
  monkeypatch.setattr(pigeond, "read_host_time_observation", lambda: None)
  monkeypatch.setattr(
    pigeond,
    "evaluate_time_authority",
    lambda _authority, _observation: SimpleNamespace(authorized_time=None),
  )
  monkeypatch.setattr(
    pigeond,
    "wait_for_current_independent_network_time",
    lambda _authority, observation, _evaluation, **_kwargs: (
      events.append("trusted_time_arrived") or observation,
      SimpleNamespace(authorized_time=network_time()),
    ),
  )
  monkeypatch.setattr(pigeond, "poll_mon_ver", lambda _pigeon: None)
  monkeypatch.setattr(pigeond, "configure_navx5_ack_aiding", lambda *_args: None)
  monkeypatch.setattr(pigeond, "log_navx5_ack_aiding_support", lambda _info: None)
  def send_mga(_pigeon, _message, **kwargs):
    index = kwargs.get("database_frame_index")
    if index is None:
      events.append("position_write")
    else:
      database_indexes.append(index)
      events.append("dbd_write")
  monkeypatch.setattr(pigeond, "send_mga_with_strict_ack", send_mga)
  def send_time(*_args, **_kwargs) -> bool:
    events.append("time_write")
    return True

  monkeypatch.setattr(
    pigeond,
    "send_time_assistance",
    send_time,
  )
  monkeypatch.setattr(
    pigeond,
    "log_navigation_assistance_restore_result",
    lambda *_args, **_kwargs: None,
  )
  monkeypatch.setattr(
    pigeond,
    "finish_pigeon_initialization",
    lambda _pigeon: events.append("normal_configuration"),
  )
  monkeypatch.setattr(pigeond, "log_assistnow_autonomous_support", lambda _info: True)
  monkeypatch.setattr(pigeond, "configure_assistnow_autonomous", lambda *_args: None)

  result = pigeond.initialize_receiver_cycle(
    pigeon,  # type: ignore[arg-type, ty:invalid-argument-type]
    "receiver",
    FakeDiagnostics(),  # type: ignore[arg-type, ty:invalid-argument-type]
    "test",
    time_authority=object(),  # type: ignore[arg-type, ty:invalid-argument-type]
    time_provenance=FakeProvenance(),  # type: ignore[arg-type, ty:invalid-argument-type]
    navigation_database_runtime=runtime,
  )

  restore = result.navigation_assistance_restore_result
  assert restore is not None
  assert (
    runtime.controller.disposition
    is NavigationDatabaseRestoreDisposition.SKIPPED_EARLY_ACQUISITION
  )
  assert restore.database_frames_attempted_count == 0
  assert database_indexes == []
  assert runtime.execution.position_assistance_attempted
  assert runtime.execution.position_assistance_succeeded
  assert "dbd_write" not in events
  assert "position_write" in events
  assert events.index("gnss_stop") < events.index(
    "trusted_time_arrived"
  )
  assert events.index("trusted_time_arrived") < events.index(
    "position_write"
  )
  assert "navigation_database_post_time_wait" not in events
  assert events.index("position_write") < events.index(
    "time_write"
  )
  assert events.index("time_write") < events.index(
    "acquisition_start_claim"
  )
  assert events.index("acquisition_start_claim") < events.index(
    "gnss_start"
  )
  assert result.trusted_time_assistance_sent
  assert result.navigation_assistance_restore_attempted





def test_observed_c4_delay_does_not_block_gnss_start(
  tmp_path: Path,
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  events: list[str] = []
  clock = [0.0]
  pigeon = FakePigeon(events)
  runtime = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )
  database_indexes: list[int] = []
  timeline_calls: list[dict[str, object]] = []
  wait_timeouts: list[float] = []
  polling_events: list[str] = []

  def capture_timeline(**kwargs: object) -> None:
    timeline_calls.append(kwargs)

  monkeypatch.setattr(
    pigeond,
    "log_gps_startup_timeline",
    capture_timeline,
  )
  real_wait = pigeond.wait_for_current_independent_network_time
  real_claim = runtime.claim_acquisition_start

  def monotonic() -> float:
    return clock[0]

  def unexpected_reader():
    polling_events.append("reader")
    raise AssertionError("zero startup wait must not poll network time")

  def unexpected_evaluator(_time_authority, _observation):
    polling_events.append("evaluator")
    raise AssertionError("zero startup wait must not reevaluate network time")

  def unexpected_sleeper(_delay: float) -> None:
    polling_events.append("sleeper")
    raise AssertionError("zero startup wait must not sleep")

  def delayed_wait(
    _authority,
    observation,
    evaluation,
    *,
    timeout_seconds: float = pigeond.NAVIGATION_DATABASE_TRUSTED_TIME_WAIT_SECONDS,
  ):
    wait_timeouts.append(timeout_seconds)
    return real_wait(
      object(),  # type: ignore[arg-type, ty:invalid-argument-type]
      observation,
      evaluation,
      timeout_seconds=timeout_seconds,
      observation_reader=unexpected_reader,
      evaluator=unexpected_evaluator,  # type: ignore[arg-type]
      monotonic=monotonic,
      sleeper=unexpected_sleeper,
    )

  def send_mga(_pigeon, _message, **kwargs):
    database_frame_index = kwargs.get("database_frame_index")
    if database_frame_index is None:
      events.append("position_write")
    else:
      database_indexes.append(database_frame_index)
      events.append("dbd_write")

  def claim_acquisition_start() -> bool:
    claimed = real_claim()
    assert claimed
    events.append("acquisition_start_claim")
    return claimed

  monkeypatch.setattr(pigeond.time, "sleep", lambda _delay: None)
  monkeypatch.setattr(
    pigeond,
    "start_pigeon_transport",
    lambda _pigeon: None,
  )
  monkeypatch.setattr(
    pigeond,
    "read_host_time_observation",
    lambda: None,
  )
  monkeypatch.setattr(
    pigeond,
    "evaluate_time_authority",
    lambda _authority, _observation: SimpleNamespace(
      authorized_time=None
    ),
  )
  monkeypatch.setattr(
    pigeond,
    "wait_for_current_independent_network_time",
    delayed_wait,
  )
  monkeypatch.setattr(
    pigeond,
    "poll_mon_ver",
    lambda _pigeon: None,
  )
  monkeypatch.setattr(
    pigeond,
    "configure_navx5_ack_aiding",
    lambda *_args: None,
  )
  monkeypatch.setattr(
    pigeond,
    "log_navx5_ack_aiding_support",
    lambda _info: None,
  )
  monkeypatch.setattr(
    pigeond,
    "send_mga_with_strict_ack",
    send_mga,
  )
  def send_time(*_args, **_kwargs) -> bool:
    events.append("time_write")
    return False

  monkeypatch.setattr(
    pigeond,
    "send_time_assistance",
    send_time,
  )
  monkeypatch.setattr(
    pigeond,
    "log_navigation_assistance_restore_result",
    lambda *_args, **_kwargs: None,
  )
  monkeypatch.setattr(
    pigeond,
    "finish_pigeon_initialization",
    lambda _pigeon: events.append("normal_configuration"),
  )
  monkeypatch.setattr(
    pigeond,
    "log_assistnow_autonomous_support",
    lambda _info: True,
  )
  monkeypatch.setattr(
    pigeond,
    "configure_assistnow_autonomous",
    lambda *_args: None,
  )
  monkeypatch.setattr(
    runtime,
    "claim_acquisition_start",
    claim_acquisition_start,
  )

  result = pigeond.initialize_receiver_cycle(
    pigeon,  # type: ignore[arg-type, ty:invalid-argument-type]
    "receiver",
    FakeDiagnostics(),  # type: ignore[arg-type, ty:invalid-argument-type]
    "test",
    time_authority=object(),  # type: ignore[arg-type, ty:invalid-argument-type]
    time_provenance=FakeProvenance(),  # type: ignore[arg-type, ty:invalid-argument-type]
    navigation_database_runtime=runtime,
  )

  restore = result.navigation_assistance_restore_result
  assert restore is not None
  assert wait_timeouts == [0.0]
  assert polling_events == []
  assert clock[0] == pytest.approx(0.0)
  assert (
    runtime.controller.disposition
    is NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
  )
  assert restore.database_frames_attempted_count == 0
  assert database_indexes == []
  assert runtime.execution.position_assistance_attempted
  assert runtime.acquisition_started
  assert "trusted_time_arrived" not in events
  assert "dbd_write" not in events
  assert "position_write" in events
  assert "time_write" not in events
  assert events.index("position_write") < events.index(
    "acquisition_start_claim"
  )
  assert events.index("acquisition_start_claim") < events.index(
    "gnss_start"
  )
  assert events.index("gnss_start") < events.index(
    "normal_configuration"
  )
  assert len(timeline_calls) == 1
  timeline = timeline_calls[0]
  assert timeline["restore_result"] is restore
  assert timeline["authorized_time"] is None
  assert timeline["independent_network_time_seen_at"] is None
  wait_started_at = timeline["trusted_time_wait_started_at"]
  wait_completed_at = timeline["trusted_time_wait_completed_at"]
  acquisition_claimed_at = timeline["acquisition_start_claimed_at"]
  gnss_start_sent_at = timeline["gnss_start_sent_at"]
  assert isinstance(wait_started_at, float)
  assert isinstance(wait_completed_at, float)
  assert isinstance(acquisition_claimed_at, float)
  assert isinstance(gnss_start_sent_at, float)
  assert wait_completed_at >= wait_started_at
  assert wait_completed_at - wait_started_at < 1.0
  assert acquisition_claimed_at <= gnss_start_sent_at




def test_pending_dbd_yuma_claim_survives_restart(
  tmp_path: Path,
) -> None:
  first = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )
  yuma_writes: list[bytes] = []
  pigeond.send_yuma_with_durable_claim(
    first,
    yuma_writes.append,
    b"provisional-yuma",
  )
  assert yuma_writes == [b"provisional-yuma"]

  second = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )
  database_writes: list[tuple[bytes, int]] = []
  result = second.evaluate(
    authorized_time=network_time(),
    reliable_fix_available=False,
    yuma_already_sent=False,
    send_database_message=(
      lambda frame, index: database_writes.append((frame, index))
    ),
  )
  assert (
    result.disposition
    is NavigationDatabaseRestoreDisposition.SKIPPED_YUMA_ALREADY_SENT
  )
  assert database_writes == []


def test_new_receiver_cycle_reopens_navigation_assistance_state(
  tmp_path: Path,
) -> None:
  state_path = tmp_path / "dbd_state.json"
  first = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=state_path,
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )
  assert first.claim_yuma_transmission()
  assert first.claim_acquisition_start()

  same_cycle = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=state_path,
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )
  assert same_cycle.yuma_sent
  assert same_cycle.acquisition_started

  next_cycle = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=state_path,
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
    new_receiver_cycle=True,
  )
  assert not next_cycle.yuma_sent
  assert not next_cycle.acquisition_started
  assert next_cycle.controller.pending
  assert next_cycle.claim_yuma_transmission()
  assert next_cycle.claim_acquisition_start()


def test_new_receiver_cycle_navigation_read_error_does_not_overwrite(
  tmp_path: Path,
) -> None:
  stores: list[tuple[object, Path]] = []

  def fail_load(_path: Path) -> object:
    raise OSError("read unavailable")

  def record_store(state: object, path: Path) -> None:
    stores.append((state, path))

  with pytest.raises(
    NavigationDatabaseRestoreInitializationError,
    match="state_load_failed:OSError:read unavailable",
  ):
    NavigationDatabaseRestoreRuntime(
      "receiver",
      snapshot_loader=lambda _fingerprint: snapshot(),
      retry_delay_seconds=0.0,
      state_path=tmp_path / "dbd_state.json",
      boot_id_reader=lambda: BOOT_ID,
      boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
      state_loader=fail_load,  # type: ignore[arg-type, ty:invalid-argument-type]
      state_storer=record_store,
      new_receiver_cycle=True,
    )

  assert stores == []


def test_new_receiver_cycle_reopens_position_retry_state(
  tmp_path: Path,
) -> None:
  state_path = tmp_path / "position_retry_state.json"
  execution = NavigationDatabaseRestoreExecution(
    disposition=NavigationDatabaseRestoreDisposition.SKIPPED_NO_USABLE_CACHE,
    total_frame_count=0,
    accepted_frame_count=0,
    database_write_attempt_count=0,
    position_assistance_attempted=True,
    position_assistance_write_status=PositionAssistanceWriteStatus.SUCCEEDED,
    position_assistance_ack_status=PositionAssistanceAckStatus.REJECTED,
    position_assistance_ack_info_code=5,
  )
  first = pigeond.PositionAssistanceRetryRuntime(
    "receiver",
    state_path=state_path,
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )
  assert first.arm_from_initial(execution, b"position-message")
  first.cancel(
    pigeond.PositionAssistanceRetryResult.CANCELLED_RECEIVER_CYCLE_CHANGED,
    TEST_BOOTTIME_SECONDS,
  )
  assert first.state.retry_completed

  same_cycle = pigeond.PositionAssistanceRetryRuntime(
    "receiver",
    state_path=state_path,
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )
  assert same_cycle.state.retry_completed

  next_cycle = pigeond.PositionAssistanceRetryRuntime(
    "receiver",
    state_path=state_path,
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
    new_receiver_cycle=True,
  )
  assert not next_cycle.state.initial_attempted
  assert not next_cycle.state.retry_armed
  assert not next_cycle.state.retry_claimed
  assert not next_cycle.state.retry_completed
  assert next_cycle.arm_from_initial(
    execution,
    b"position-message",
  )


def test_new_receiver_cycle_retry_read_error_does_not_overwrite(
  tmp_path: Path,
) -> None:
  stores: list[tuple[PositionAssistanceRetryState, Path]] = []

  def fail_load(
    _path: Path,
  ) -> PositionAssistanceRetryState | None:
    raise OSError("read unavailable")

  def record_store(
    state: PositionAssistanceRetryState,
    path: Path,
  ) -> None:
    stores.append((state, path))

  with pytest.raises(
    PositionAssistanceRetryStateError,
    match="OSError:read unavailable",
  ):
    pigeond.PositionAssistanceRetryRuntime(
      "receiver",
      state_path=tmp_path / "position_retry_state.json",
      boot_id_reader=lambda: BOOT_ID,
      boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
      state_loader=fail_load,
      state_storer=record_store,
      new_receiver_cycle=True,
    )

  assert stores == []


def test_receiver_cycle_navigation_factory_requests_fresh_state(
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  navigation_calls: list[bool] = []

  class NavigationRuntime:
    def __init__(
      self,
      _receiver_fingerprint: str,
      *,
      new_receiver_cycle: bool,
    ) -> None:
      navigation_calls.append(new_receiver_cycle)

  monkeypatch.setattr(
    pigeond,
    "NavigationDatabaseRestoreRuntime",
    NavigationRuntime,
  )

  first = pigeond.create_receiver_cycle_navigation_state("receiver")
  second = pigeond.create_receiver_cycle_navigation_state("receiver")

  assert navigation_calls == [True, True]
  assert first is not second


def test_prepared_receiver_response_state_is_not_reset_twice(
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  resets = 0

  class Pigeon:
    _stream_parser = object()

    def reset_response_state(self) -> None:
      nonlocal resets
      resets += 1

    def send(self, _message: bytes) -> None:
      pass

  monkeypatch.setattr(
    pigeond.signal,
    "signal",
    lambda *_args, **_kwargs: None,
  )
  monkeypatch.setattr(
    pigeond,
    "set_power",
    lambda _enabled: None,
  )
  monkeypatch.setattr(
    pigeond.time,
    "sleep",
    lambda _delay: None,
  )
  monkeypatch.setattr(
    pigeond,
    "init_baudrate",
    lambda _pigeon: None,
  )
  monkeypatch.setattr(
    pigeond,
    "poll_mon_ver",
    lambda _pigeon, _timeout: SimpleNamespace(),
  )

  pigeon = Pigeon()
  pigeond.prepare_receiver_cycle_response_state(
    pigeon,  # type: ignore[arg-type, ty:invalid-argument-type]
  )
  pigeond.start_pigeon_transport(
    pigeon,  # type: ignore[arg-type, ty:invalid-argument-type]
  )

  assert resets == 1
  assert not pigeon._receiver_cycle_response_state_prepared

# COMMIT7_DBD_LIVE_BOUNDARY_TESTS

def test_early_acquisition_skips_post_wait_drain_and_dbd(
  tmp_path: Path,
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  events: list[str] = []
  runtime = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )

  class DrainPigeon(FakePigeon):
    def drain_before_transaction(self, operation: str) -> None:
      events.append(operation)
      if operation == "navigation_database_post_time_wait":
        raise AssertionError(
          "early acquisition must not execute the obsolete DBD drain"
        )

  pigeon = DrainPigeon(events)
  database_indexes: list[int] = []

  monkeypatch.setattr(pigeond.time, "sleep", lambda _delay: None)
  monkeypatch.setattr(
    pigeond,
    "start_pigeon_transport",
    lambda _pigeon: None,
  )
  monkeypatch.setattr(
    pigeond,
    "read_host_time_observation",
    lambda: None,
  )
  monkeypatch.setattr(
    pigeond,
    "evaluate_time_authority",
    lambda _authority, _observation: SimpleNamespace(
      authorized_time=None
    ),
  )
  monkeypatch.setattr(
    pigeond,
    "wait_for_current_independent_network_time",
    lambda _authority, observation, _evaluation, **_kwargs: (
      observation,
      SimpleNamespace(authorized_time=network_time()),
    ),
  )
  monkeypatch.setattr(
    pigeond,
    "poll_mon_ver",
    lambda _pigeon: None,
  )
  monkeypatch.setattr(
    pigeond,
    "configure_navx5_ack_aiding",
    lambda *_args: None,
  )
  monkeypatch.setattr(
    pigeond,
    "log_navx5_ack_aiding_support",
    lambda _info: None,
  )
  monkeypatch.setattr(
    pigeond,
    "send_mga_with_strict_ack",
    lambda _pigeon, _message, **kwargs: (
      database_indexes.append(kwargs["database_frame_index"])
      if kwargs.get("database_frame_index") is not None
      else None
    ),
  )
  monkeypatch.setattr(
    pigeond,
    "send_time_assistance",
    lambda *_args, **_kwargs: False,
  )
  monkeypatch.setattr(
    pigeond,
    "log_navigation_assistance_restore_result",
    lambda *_args, **_kwargs: None,
  )
  monkeypatch.setattr(
    pigeond,
    "finish_pigeon_initialization",
    lambda _pigeon: None,
  )
  monkeypatch.setattr(
    pigeond,
    "log_assistnow_autonomous_support",
    lambda _info: True,
  )
  monkeypatch.setattr(
    pigeond,
    "configure_assistnow_autonomous",
    lambda *_args: None,
  )

  result = pigeond.initialize_receiver_cycle(
    pigeon,  # ty: ignore[invalid-argument-type]
    "receiver",
    FakeDiagnostics(),  # ty: ignore[invalid-argument-type]
    "test",
    time_authority=object(),  # ty: ignore[invalid-argument-type]
    time_provenance=FakeProvenance(),  # ty: ignore[invalid-argument-type]
    navigation_database_runtime=runtime,
  )

  assert "navigation_database_post_time_wait" not in events
  assert database_indexes == []
  assert (
    runtime.controller.disposition
    is NavigationDatabaseRestoreDisposition.SKIPPED_EARLY_ACQUISITION
  )
  assert events.index("gnss_stop") < events.index("gnss_start")
  assert result.navigation_assistance_restore_attempted



def test_frame_zero_transaction_drain_guard_blocks_receiver_write(tmp_path: Path) -> None:
  runtime = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
  )
  receiver_writes: list[bytes] = []
  class DrainPigeon:
    def begin_response_transaction(self, message: bytes, _operation: str, before_send):
      assert runtime.note_acquisition_started()
      before_send()
      receiver_writes.append(message)
      raise AssertionError("DBD write guard returned after acquisition")
  pigeon = DrainPigeon()
  result = runtime.evaluate(
    authorized_time=network_time(),
    reliable_fix_available=False,
    yuma_already_sent=False,
    send_database_message=lambda message, frame_index: pigeond.send_mga_with_strict_ack(
      pigeon,  # ty: ignore[invalid-argument-type]
      message,
      database_frame_index=frame_index,
      before_send=lambda: runtime.validate_database_write_boundary(frame_index),
    ),
  )
  assert result.disposition is NavigationDatabaseRestoreDisposition.WRITE_FAILED
  assert receiver_writes == []
  assert result.permanent_failures

def test_assistance_state_initialization_failure_still_starts_gnss(
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  events: list[str] = []
  pigeon = FakePigeon(events)
  assistance_writes: list[bytes] = []
  retry_controller = (
    pigeond.PositionAssistancePostStartRetryController(
      cast(pigeond.PositionAssistanceRetryRuntime, SimpleNamespace())
    )
  )

  def unavailable_runtime(
    _receiver_fingerprint: str,
  ) -> NavigationDatabaseRestoreRuntime:
    raise NavigationDatabaseRestoreInitializationError(
      "boot_state:storage_unavailable"
    )

  monkeypatch.setattr(
    pigeond,
    "NavigationDatabaseRestoreRuntime",
    unavailable_runtime,
  )
  monkeypatch.setattr(pigeond.time, "sleep", lambda _delay: None)
  monkeypatch.setattr(
    pigeond,
    "start_pigeon_transport",
    lambda _pigeon: None,
  )
  monkeypatch.setattr(
    pigeond,
    "read_host_time_observation",
    lambda: None,
  )
  monkeypatch.setattr(
    pigeond,
    "evaluate_time_authority",
    lambda _authority, _observation: SimpleNamespace(
      authorized_time=network_time()
    ),
  )
  monkeypatch.setattr(pigeond, "poll_mon_ver", lambda _pigeon: None)
  monkeypatch.setattr(
    pigeond,
    "configure_navx5_ack_aiding",
    lambda *_args: None,
  )
  monkeypatch.setattr(
    pigeond,
    "log_navx5_ack_aiding_support",
    lambda _info: None,
  )
  monkeypatch.setattr(
    pigeond,
    "send_mga_with_strict_ack",
    lambda _pigeon, message, **_kwargs: assistance_writes.append(
      message
    ),
  )
  monkeypatch.setattr(
    pigeond,
    "send_time_assistance",
    lambda *_args, **_kwargs: events.append("time_assistance") or True,
  )
  monkeypatch.setattr(
    pigeond,
    "log_navigation_assistance_restore_result",
    lambda *_args, **_kwargs: None,
  )
  monkeypatch.setattr(
    pigeond,
    "finish_pigeon_initialization",
    lambda _pigeon: events.append("normal_configuration"),
  )
  monkeypatch.setattr(
    pigeond,
    "log_assistnow_autonomous_support",
    lambda _info: True,
  )
  monkeypatch.setattr(
    pigeond,
    "configure_assistnow_autonomous",
    lambda *_args: None,
  )

  result = pigeond.initialize_receiver_cycle(
    pigeon,  # type: ignore[arg-type, ty:invalid-argument-type]
    "receiver",
    FakeDiagnostics(),  # type: ignore[arg-type, ty:invalid-argument-type]
    "test",
    time_authority=object(),  # type: ignore[arg-type, ty:invalid-argument-type]
    time_provenance=FakeProvenance(),  # type: ignore[arg-type, ty:invalid-argument-type]
    position_assistance_retry=retry_controller,
  )

  assert assistance_writes == []
  assert retry_controller.runtime is None
  assert events.index("gnss_stop") < events.index("time_assistance")
  assert events.index("time_assistance") < events.index("gnss_start")
  assert events.index("gnss_start") < events.index(
    "normal_configuration"
  )
  restore = result.navigation_assistance_restore_result
  assert restore is not None
  assert restore.database_restore_state_error == (
    "boot_state:storage_unavailable"
  )


def test_restore_state_persistence_failure_does_not_block_gnss_start(
  tmp_path: Path,
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  events: list[str] = []
  pigeon = FakePigeon(events)
  store_calls = 0
  assistance_writes: list[bytes] = []

  def fail_after_baseline(state, path: Path) -> None:
    nonlocal store_calls
    store_calls += 1
    if store_calls > 1:
      raise OSError("storage unavailable")
    restore_runtime.store_navigation_database_restore_boot_state(
      state,
      path,
    )

  runtime = NavigationDatabaseRestoreRuntime(
    "receiver",
    snapshot_loader=lambda _fingerprint: snapshot(),
    retry_delay_seconds=0.0,
    state_path=tmp_path / "dbd_state.json",
    boot_id_reader=lambda: BOOT_ID,
    boottime_reader=lambda: TEST_BOOTTIME_SECONDS,
    state_storer=fail_after_baseline,
  )

  monkeypatch.setattr(pigeond.time, "sleep", lambda _delay: None)
  monkeypatch.setattr(
    pigeond,
    "start_pigeon_transport",
    lambda _pigeon: None,
  )
  monkeypatch.setattr(
    pigeond,
    "read_host_time_observation",
    lambda: None,
  )
  monkeypatch.setattr(
    pigeond,
    "evaluate_time_authority",
    lambda _authority, _observation: SimpleNamespace(
      authorized_time=None
    ),
  )
  monkeypatch.setattr(pigeond, "poll_mon_ver", lambda _pigeon: None)
  monkeypatch.setattr(
    pigeond,
    "configure_navx5_ack_aiding",
    lambda *_args: None,
  )
  monkeypatch.setattr(
    pigeond,
    "log_navx5_ack_aiding_support",
    lambda _info: None,
  )
  monkeypatch.setattr(
    pigeond,
    "send_mga_with_strict_ack",
    lambda _pigeon, message, **_kwargs: assistance_writes.append(
      message
    ),
  )
  monkeypatch.setattr(
    pigeond,
    "send_time_assistance",
    lambda *_args, **_kwargs: False,
  )
  monkeypatch.setattr(
    pigeond,
    "log_navigation_assistance_restore_result",
    lambda *_args, **_kwargs: None,
  )
  monkeypatch.setattr(
    pigeond,
    "finish_pigeon_initialization",
    lambda _pigeon: events.append("normal_configuration"),
  )
  monkeypatch.setattr(
    pigeond,
    "log_assistnow_autonomous_support",
    lambda _info: True,
  )
  monkeypatch.setattr(
    pigeond,
    "configure_assistnow_autonomous",
    lambda *_args: None,
  )

  result = pigeond.initialize_receiver_cycle(
    pigeon,  # type: ignore[arg-type, ty:invalid-argument-type]
    "receiver",
    FakeDiagnostics(),  # type: ignore[arg-type, ty:invalid-argument-type]
    "test",
    time_authority=object(),  # type: ignore[arg-type, ty:invalid-argument-type]
    time_provenance=FakeProvenance(),  # type: ignore[arg-type, ty:invalid-argument-type]
    navigation_database_runtime=runtime,
  )

  assert not runtime.state_available
  assert assistance_writes == []
  assert events.index("gnss_stop") < events.index("gnss_start")
  assert events.index("gnss_start") < events.index(
    "normal_configuration"
  )
  restore = result.navigation_assistance_restore_result
  assert restore is not None
  assert restore.database_restore_state_error is not None


def test_acquisition_state_failure_is_handled_once(
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  persistence_attempts = 0
  errors: list[str] = []

  def fail_once() -> bool:
    nonlocal persistence_attempts
    persistence_attempts += 1
    return False

  runtime = SimpleNamespace(
    acquisition_started=False,
    note_acquisition_started=fail_once,
  )
  retry = SimpleNamespace(runtime=object())
  guard = pigeond.ReceiverAcquisitionStateGuard()
  monkeypatch.setattr(pigeond.cloudlog, "error", errors.append)

  for _ in range(25):
    pigeond.handle_receiver_acquisition_state(
      runtime,  # type: ignore[arg-type, ty:invalid-argument-type]
      retry,  # type: ignore[arg-type, ty:invalid-argument-type]
      guard,
    )

  assert persistence_attempts == 1
  assert len(errors) == 1
  assert retry.runtime is None


def test_yuma_assistance_state_suppression_uses_explicit_marker() -> None:
  normal_outcome = SimpleNamespace(
    transmit_result=SimpleNamespace(
      assistance_state_unavailable=True,
      status="partial",
      attempted_satellite_ids=(1,),
      accepted_satellite_ids=(1,),
      unavailable_satellite_ids=(2, 3),
    ),
    terminal=True,
    retry_pending=False,
  )
  provisional_outcome = SimpleNamespace(
    transmit_result=SimpleNamespace(
      assistance_state_unavailable=True,
      status="unavailable",
      attempted_satellite_ids=(),
      accepted_satellite_ids=(),
      unavailable_satellite_ids=(1, 2, 3),
    ),
    receiver_write_attempted=False,
  )
  structural_shape_without_marker = SimpleNamespace(
    transmit_result=SimpleNamespace(
      status="unavailable",
      requested_satellite_ids=(1, 2, 3),
      attempted_satellite_ids=(),
      accepted_satellite_ids=(),
      failed_satellite_ids=(),
      unavailable_satellite_ids=(1, 2, 3),
    )
  )

  assert pigeond.yuma_assistance_state_unavailable_outcome(normal_outcome)
  assert pigeond.yuma_assistance_state_unavailable_outcome(
    provisional_outcome
  )
  assert not pigeond.yuma_assistance_state_unavailable_outcome(
    structural_shape_without_marker
  )
