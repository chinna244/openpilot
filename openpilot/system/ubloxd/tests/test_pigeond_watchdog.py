import pytest

from openpilot.system.ubloxd import pigeond


def test_watchdog_waits_for_timeout():
  watchdog = pigeond.UbloxDataWatchdog(
    timeout=10.0,
    start_time=100.0,
  )

  assert not watchdog.check(109.999)
  assert watchdog.check(110.0)


def test_watchdog_raises_after_failed_recovery():
  watchdog = pigeond.UbloxDataWatchdog(
    timeout=10.0,
    max_recoveries=1,
    start_time=100.0,
  )

  assert watchdog.check(110.0)
  watchdog.recovery_completed(110.0)

  with pytest.raises(
    RuntimeError,
    match="No data from ublox after watchdog recovery",
  ):
    watchdog.check(120.0)


def test_watchdog_data_resets_recovery_budget():
  watchdog = pigeond.UbloxDataWatchdog(
    timeout=10.0,
    max_recoveries=1,
    start_time=100.0,
  )

  assert watchdog.check(110.0)
  watchdog.recovery_completed(110.0)

  watchdog.note_data(111.0)

  assert not watchdog.check(120.999)
  assert watchdog.check(121.0)


def test_init_raises_when_receiver_configuration_fails(
  monkeypatch,
):
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
    lambda _seconds: None,
  )
  monkeypatch.setattr(
    pigeond,
    "init_baudrate",
    lambda _pigeon: None,
  )
  monkeypatch.setattr(
    pigeond,
    "init_pigeon",
    lambda _pigeon: False,
  )

  with pytest.raises(
    RuntimeError,
    match="Failed to initialize pigeon",
  ):
    pigeond.init(object())


def test_zero_prefixed_ublox_payload_is_not_all_zero():
  payload = bytearray(4096)
  payload[1024] = 0x01

  assert not pigeond.is_all_zero_ublox_data(bytes(payload))


def test_all_zero_ublox_payload_is_detected():
  assert pigeond.is_all_zero_ublox_data(bytes(4096))


def test_empty_ublox_payload_is_not_all_zero():
  assert not pigeond.is_all_zero_ublox_data(b"")
