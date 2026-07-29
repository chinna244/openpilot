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


def test_paused_gnss_acquisition_always_restarts_after_failure(
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  pigeon = FakePigeon()
  monkeypatch.setattr(pigeond.time, "sleep", lambda _delay: None)

  with pytest.raises(RuntimeError, match="simulated setup failure"):
    with pigeond.paused_gnss_acquisition(pigeon):  # type: ignore[arg-type]
      raise RuntimeError("simulated setup failure")

  assert pigeon.sent == [
    pigeond.CONTROLLED_GNSS_STOP_MESSAGE,
    pigeond.CONTROLLED_GNSS_START_MESSAGE,
  ]


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
    pigeon,  # type: ignore[arg-type]
    "receiver",
    FakeDiagnostics(),  # type: ignore[arg-type]
    "test",
    time_authority=object(),  # type: ignore[arg-type]
    time_provenance=FakeProvenance(),  # type: ignore[arg-type]
    navigation_database_runtime=runtime,
  )

  assert runtime.controller.disposition is NavigationDatabaseRestoreDisposition.SKIPPED_ACQUISITION_ALREADY_STARTED
  assert database_indexes == []
  assert events.index("gnss_stop") < events.index("configuration_traffic")
  assert events.index("configuration_traffic") < events.index("gnss_start")
  assert events.index("gnss_start") < events.index("normal_configuration")
  assert result.navigation_assistance_restore_attempted
