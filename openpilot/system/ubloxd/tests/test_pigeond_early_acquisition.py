from __future__ import annotations

import pytest

from openpilot.system.ubloxd import pigeond


def test_process_start_wait_has_separate_absolute_deadline() -> None:
  assert pigeond.NAVIGATION_DATABASE_TRUSTED_TIME_WAIT_SECONDS == 40.0
  assert (
    pigeond.NAVIGATION_DATABASE_PROCESS_START_TIME_DEADLINE_SECONDS
    == 45.0
  )
  assert (
    pigeond.navigation_database_process_start_wait_seconds(
      cycle_started_at=10.0,
      now=13.0,
    )
    == 42.0
  )
  assert (
    pigeond.navigation_database_process_start_wait_seconds(
      cycle_started_at=10.0,
      now=60.0,
    )
    == 0.0
  )


@pytest.mark.parametrize(
  ("overrides", "expected"),
  (
    ({}, True),
    ({"restore_pending": False}, False),
    ({"state_available": False}, False),
    ({"candidate_available": False}, False),
    ({"allow_wait": False}, False),
    ({"network_available": False}, False),
    ({"network_available": False, "network_recheck_available": True}, True),
    ({"network_available": None}, True),
    ({"acquisition_started": True}, False),
    ({"current_network_time": True}, False),
  ),
)
def test_database_wait_requires_every_process_start_gate(
  overrides: dict[str, bool],
  expected: bool,
) -> None:
  values = {
    "restore_pending": True,
    "state_available": True,
    "candidate_available": True,
    "allow_wait": True,
    "network_available": True,
    "network_recheck_available": False,
    "acquisition_started": False,
    "current_network_time": False,
  }
  values.update(overrides)
  assert (
    pigeond.should_wait_for_navigation_database_trusted_time(**values)
    is expected
  )


@pytest.mark.parametrize(
  ("wait_attempted", "network_available", "expected"),
  (
    (False, None, False),
    (True, False, True),
    (True, None, True),
    (True, True, True),
  ),
)
def test_database_wait_timeout_requires_network_opportunity(
  wait_attempted: bool,
  network_available: bool | None,
  expected: bool,
) -> None:
  assert (
    pigeond.navigation_database_trusted_time_wait_expired(
      wait_attempted=wait_attempted,
      network_available=network_available,
    )
    is expected
  )


def test_device_network_availability_distinguishes_unready_from_offline() -> None:
  class DeviceStateSubMaster:
    def __init__(self) -> None:
      self.alive = {"deviceState": False}
      self.valid = {"deviceState": False}
      self.device_state = type("DeviceState", (), {"networkType": pigeond.log.DeviceState.NetworkType.none})()

    def update(self, _timeout: int) -> None:
      pass

    def __getitem__(self, _service: str):
      return self.device_state

  sm = DeviceStateSubMaster()
  assert pigeond.device_network_available(sm) is None  # type: ignore[arg-type, ty:invalid-argument-type]

  sm.alive["deviceState"] = True
  sm.valid["deviceState"] = True
  assert not pigeond.device_network_available(sm)  # type: ignore[arg-type, ty:invalid-argument-type]

  sm.device_state.networkType = pigeond.log.DeviceState.NetworkType.wifi
  assert pigeond.device_network_available(sm)  # type: ignore[arg-type, ty:invalid-argument-type]


def test_receiver_acquisition_guard_records_early_dbd_outcome() -> None:
  events: list[str] = []

  class Runtime:
    acquisition_started = False
    database_restore_pending = True

    def note_early_acquisition_started(self) -> bool:
      events.append("early")
      self.acquisition_started = True
      self.database_restore_pending = False
      return True

    def note_acquisition_started(self) -> bool:
      events.append("normal")
      return True

  runtime = Runtime()
  guard = pigeond.ReceiverAcquisitionStateGuard()

  assert guard.note_once(runtime)  # type: ignore[arg-type, ty:invalid-argument-type]
  assert guard.note_once(runtime) is None  # type: ignore[arg-type, ty:invalid-argument-type]
  assert events == ["early"]


def test_transition_telemetry_uses_post_send_timestamps(
  monkeypatch: pytest.MonkeyPatch,
) -> None:
  events: list[str] = []
  logs: list[str] = []
  clock = iter((10.0, 13.5))

  class Pigeon:
    def send(self, message: bytes) -> None:
      if message == pigeond.CONTROLLED_GNSS_STOP_MESSAGE:
        events.append("stop_send")
      elif message == pigeond.CONTROLLED_GNSS_START_MESSAGE:
        events.append("start_send")

  def monotonic() -> float:
    value = next(clock)
    events.append(f"monotonic:{value:.1f}")
    return value

  def log_info(message: str) -> None:
    events.append("log")
    logs.append(message)

  monkeypatch.setattr(pigeond.time, "monotonic", monotonic)
  monkeypatch.setattr(pigeond.time, "sleep", lambda _delay: None)
  monkeypatch.setattr(pigeond.cloudlog, "info", log_info)

  with pigeond.paused_gnss_acquisition(
    Pigeon(),  # type: ignore[arg-type, ty:invalid-argument-type]
  ):
    events.append("body")

  assert events == [
    "stop_send",
    "monotonic:10.0",
    "log",
    "body",
    "start_send",
    "monotonic:13.5",
    "log",
  ]
  assert logs == [
    "GPS acquisition transition: phase=stop_sent monotonic=10.000000",
    ("GPS acquisition transition: phase=start_sent monotonic=13.500000 prestart_elapsed_seconds=3.500000"),
  ]
