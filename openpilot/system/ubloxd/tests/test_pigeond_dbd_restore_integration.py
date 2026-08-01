from __future__ import annotations

from datetime import UTC, datetime, timedelta
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.system.ubloxd import pigeond
from openpilot.system.ubloxd.navigation_database_restore import (
  NavigationDatabaseRestoreDisposition,
)
from openpilot.system.ubloxd.navigation_database_restore_runtime import (
  NavigationDatabaseRestoreRuntime,
  NavigationDatabaseRestoreSnapshot,
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
  assert (
    "acquisition_start_claimed_cycle_seconds=35.000"
    in message
  )
  assert "gnss_start_sent_cycle_seconds=35.100" in message
  assert (
    "related_acquisition_milestones=first_nonempty_rawx|first_fix_ok|first_reliable_fix"
    in message
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
  with pytest.raises(RuntimeError, match="YUMA claim persistence failed"):
    pigeond.send_yuma_with_durable_claim(
      runtime,  # type: ignore[arg-type, ty:invalid-argument-type]
      lambda message: writes.append(message),
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
  pigeond.start_pigeon_transport(pigeon)  # type: ignore[arg-type, ty:invalid-argument-type]
  assert events.index("power_on") < events.index("gnss_stop")
  assert events.index("gnss_stop") < events.index("sleep", events.index("power_on"))
  assert events.index("gnss_stop") < events.index("init_baudrate")


def test_delayed_network_time_restores_before_gnss_start(
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
    lambda _authority, _observation: SimpleNamespace(authorized_time=None),
  )
  monkeypatch.setattr(
    pigeond,
    "wait_for_current_independent_network_time",
    lambda _authority, observation, _evaluation: (
      events.append("trusted_time_arrived") or observation,
      SimpleNamespace(authorized_time=network_time()),
    ),
  )
  monkeypatch.setattr(pigeond, "poll_mon_ver", lambda _pigeon: None)
  monkeypatch.setattr(pigeond, "configure_navx5_ack_aiding", lambda *_args: None)
  monkeypatch.setattr(pigeond, "log_navx5_ack_aiding_support", lambda _info: None)
  def send_mga(_pigeon, _message, **kwargs):
    index = kwargs.get("database_frame_index")
    if index is not None:
      database_indexes.append(index)
      events.append("dbd_write")
  monkeypatch.setattr(pigeond, "send_mga_with_strict_ack", send_mga)
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

  assert runtime.controller.disposition is NavigationDatabaseRestoreDisposition.RESTORED
  assert database_indexes == [0]
  assert events.index("gnss_stop") < events.index("trusted_time_arrived")
  assert events.index("trusted_time_arrived") < events.index("dbd_write")
  assert events.index("dbd_write") < events.index("gnss_start")
  assert result.navigation_assistance_restore_attempted



def test_observed_c4_delay_restores_full_cycle_before_durable_start(
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

  def sleeper(delay: float) -> None:
    clock[0] += delay

  def delayed_wait(_authority, observation, evaluation):
    def evaluator(_time_authority, _observation):
      authorized_time = (
        network_time() if clock[0] >= 32.5 else None
      )
      if (
        authorized_time is not None
        and "trusted_time_arrived" not in events
      ):
        events.append("trusted_time_arrived")
      return SimpleNamespace(authorized_time=authorized_time)

    return real_wait(
      object(),  # type: ignore[arg-type, ty:invalid-argument-type]
      observation,
      evaluation,
      observation_reader=lambda: None,
      evaluator=evaluator,  # type: ignore[arg-type, ty:invalid-argument-type]
      monotonic=monotonic,
      sleeper=sleeper,
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
  assert clock[0] == pytest.approx(32.5)
  assert (
    runtime.controller.disposition
    is NavigationDatabaseRestoreDisposition.RESTORED
  )
  assert (
    runtime.controller.disposition
    is not NavigationDatabaseRestoreDisposition.SKIPPED_UNVERIFIED
  )
  assert restore.database_frames_attempted_count == 1
  assert restore.accepted_frame_count == 1
  assert restore.total_frame_count == 1
  assert restore.restored_cache_generation == "primary"
  assert (
    restore.restored_cache_selection_reason
    == "trusted_age_only_eligible:primary"
  )
  assert runtime.execution.position_assistance_attempted
  assert runtime.execution.position_assistance_succeeded
  assert runtime.acquisition_started
  assert database_indexes == [0]
  assert events.index("gnss_stop") < events.index(
    "trusted_time_arrived"
  )
  assert events.index("trusted_time_arrived") < events.index(
    "dbd_write"
  )
  assert events.index("dbd_write") < events.index(
    "position_write"
  )
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
  assert timeline["authorized_time"] == network_time()
  assert timeline["independent_network_time_seen_at"] is not None
  assert timeline["acquisition_start_claimed_at"] is not None
  assert timeline["gnss_start_sent_at"] is not None



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

# COMMIT7_DBD_LIVE_BOUNDARY_TESTS

def test_acquisition_dispatched_after_network_wait_blocks_dbd(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
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
        events.append("rawx_dispatched")
        assert runtime.note_acquisition_started()
  pigeon = DrainPigeon(events)
  database_indexes: list[int] = []
  monkeypatch.setattr(pigeond.time, "sleep", lambda _delay: None)
  monkeypatch.setattr(pigeond, "start_pigeon_transport", lambda _pigeon: None)
  monkeypatch.setattr(pigeond, "read_host_time_observation", lambda: None)
  monkeypatch.setattr(pigeond, "evaluate_time_authority", lambda _authority, _observation: SimpleNamespace(authorized_time=None))
  monkeypatch.setattr(
    pigeond,
    "wait_for_current_independent_network_time",
    lambda _authority, observation, _evaluation: (
      observation,
      SimpleNamespace(authorized_time=network_time()),
    ),
  )
  monkeypatch.setattr(pigeond, "poll_mon_ver", lambda _pigeon: None)
  monkeypatch.setattr(pigeond, "configure_navx5_ack_aiding", lambda *_args: None)
  monkeypatch.setattr(pigeond, "log_navx5_ack_aiding_support", lambda _info: None)
  monkeypatch.setattr(
    pigeond,
    "send_mga_with_strict_ack",
    lambda _pigeon, _message, **kwargs: (
      database_indexes.append(kwargs["database_frame_index"])
      if kwargs.get("database_frame_index") is not None
      else None
    ),
  )
  monkeypatch.setattr(pigeond, "send_time_assistance", lambda *_args, **_kwargs: False)
  monkeypatch.setattr(pigeond, "log_navigation_assistance_restore_result", lambda *_args, **_kwargs: None)
  monkeypatch.setattr(pigeond, "finish_pigeon_initialization", lambda _pigeon: None)
  monkeypatch.setattr(pigeond, "log_assistnow_autonomous_support", lambda _info: True)
  monkeypatch.setattr(pigeond, "configure_assistnow_autonomous", lambda *_args: None)
  result = pigeond.initialize_receiver_cycle(
    pigeon,  # ty: ignore[invalid-argument-type]
    "receiver",
    FakeDiagnostics(),  # ty: ignore[invalid-argument-type]
    "test",
    time_authority=object(),  # ty: ignore[invalid-argument-type]
    time_provenance=FakeProvenance(),  # ty: ignore[invalid-argument-type]
    navigation_database_runtime=runtime,
  )
  assert events.index("rawx_dispatched") < events.index("gnss_start")
  assert runtime.controller.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_ACQUISITION_ALREADY_STARTED
  assert database_indexes == []
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
