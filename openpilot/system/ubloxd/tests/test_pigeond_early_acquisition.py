from __future__ import annotations

import pytest

from openpilot.system.ubloxd import pigeond


def test_prestart_wait_is_zero_without_changing_general_wait() -> None:
  assert pigeond.NAVIGATION_DATABASE_TRUSTED_TIME_WAIT_SECONDS == 40.0
  assert pigeond.NAVIGATION_DATABASE_PRESTART_TRUSTED_TIME_WAIT_SECONDS == 0.0


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
