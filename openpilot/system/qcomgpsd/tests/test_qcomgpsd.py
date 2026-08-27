import datetime

from openpilot.common.test import OpenpilotTestCase
from openpilot.system.qcomgpsd.qcomgpsd import (
  CLOCK_JUMP_MONITOR_S,
  CLOCK_JUMP_THRESHOLD,
  XTRA_TIME_MAX_ATTEMPTS,
  clock_offset_jumped,
  maybe_send_xtra_time,
  monitor_clock_jump_and_reinject,
  retry_xtra_time_after_start,
  send_xtra_time,
  wait_until_xtra_time_sent,
)


class FakeDateTime(datetime.datetime):
  @classmethod
  def now(cls, tz=None):
    return cls(2026, 8, 27, 18, 33, 48, tzinfo=tz)


class FakeClock:
  def __init__(self, start=0.0, offset=100.0):
    self.mono = start
    self.offset = offset

  def monotonic(self):
    return self.mono

  def sleep(self, dt):
    self.mono += dt


class TestXtraTime(OpenpilotTestCase):
  def test_send_skipped_when_system_time_invalid(self, mocker):
    mocker.patch("openpilot.system.qcomgpsd.qcomgpsd.system_time_valid", return_value=False)
    at = mocker.patch("openpilot.system.qcomgpsd.qcomgpsd._at_cmd_once")
    assert send_xtra_time() is False
    at.assert_not_called()

  def test_send_ok(self, mocker):
    mocker.patch("openpilot.system.qcomgpsd.qcomgpsd.system_time_valid", return_value=True)
    mocker.patch("openpilot.system.qcomgpsd.qcomgpsd.datetime.datetime", FakeDateTime)
    at = mocker.patch("openpilot.system.qcomgpsd.qcomgpsd._at_cmd_once", return_value=("OK", ""))
    assert send_xtra_time() is True
    at.assert_called_once_with('AT+QGPSXTRATIME=0,"2026/08/27,18:33:48",1,1,1000')

  def test_send_error(self, mocker):
    mocker.patch("openpilot.system.qcomgpsd.qcomgpsd.system_time_valid", return_value=True)
    mocker.patch("openpilot.system.qcomgpsd.qcomgpsd.datetime.datetime", FakeDateTime)
    mocker.patch("openpilot.system.qcomgpsd.qcomgpsd._at_cmd_once", return_value=("ERROR", ""))
    assert send_xtra_time() is False

  def test_send_cme_error(self, mocker):
    mocker.patch("openpilot.system.qcomgpsd.qcomgpsd.system_time_valid", return_value=True)
    mocker.patch("openpilot.system.qcomgpsd.qcomgpsd.datetime.datetime", FakeDateTime)
    mocker.patch("openpilot.system.qcomgpsd.qcomgpsd._at_cmd_once", return_value=("+CME ERROR: 4", ""))
    assert send_xtra_time() is False

  def test_maybe_send_does_not_spam(self):
    calls = []
    assert maybe_send_xtra_time(True, send_fn=lambda: calls.append("sent") or True) is True
    assert calls == []
    assert maybe_send_xtra_time(False, send_fn=lambda: calls.append("sent") or True) is True
    assert calls == ["sent"]

  def test_retry_after_failure_then_stop_on_success(self):
    results = [False, False, True]
    calls = []
    sleeps = []

    def send():
      value = results.pop(0)
      calls.append(value)
      return value

    assert wait_until_xtra_time_sent(send_fn=send, sleep_fn=sleeps.append, time_valid_fn=lambda: True,
                                    max_attempts=5, retry_delay=1.0)[0] is True
    assert calls == [False, False, True]
    assert sleeps == [1.0, 1.0]

  def test_waits_for_valid_time_before_at_attempts(self):
    valid = [False, False, True]
    sends = []
    sleeps = []

    def time_valid():
      return valid.pop(0) if valid else True

    def send():
      sends.append(1)
      return True

    assert wait_until_xtra_time_sent(send_fn=send, sleep_fn=sleeps.append, interval=1.0,
                                    time_valid_fn=time_valid, max_attempts=5)[0] is True
    assert sends == [1]
    assert sleeps == [1.0, 1.0]

  def test_first_attempt_ok_stops_immediately(self):
    sends = []
    sleeps = []
    assert wait_until_xtra_time_sent(send_fn=lambda: sends.append(1) or True, sleep_fn=sleeps.append,
                                    time_valid_fn=lambda: True, max_attempts=5)[0] is True
    assert sends == [1]
    assert sleeps == []

  def test_all_attempts_fail_then_stop(self, mocker):
    mocker.patch("openpilot.system.qcomgpsd.qcomgpsd.cloudlog.error")
    sends = []
    sleeps = []

    def send():
      sends.append(1)
      return False

    assert wait_until_xtra_time_sent(send_fn=send, sleep_fn=sleeps.append, time_valid_fn=lambda: True,
                                    max_attempts=XTRA_TIME_MAX_ATTEMPTS, retry_delay=1.0)[0] is False
    assert sends == [1] * XTRA_TIME_MAX_ATTEMPTS
    assert sleeps == [1.0] * (XTRA_TIME_MAX_ATTEMPTS - 1)

  def test_startup_success_starts_clock_jump_monitor(self, mocker):
    thread = mocker.patch("openpilot.system.qcomgpsd.qcomgpsd.threading.Thread")
    retry_xtra_time_after_start(True)
    thread.assert_called_once()
    assert thread.call_args.kwargs["daemon"] is True
    thread.return_value.start.assert_called_once()

  def test_late_retry_starts_once_when_startup_skipped(self, mocker):
    thread = mocker.patch("openpilot.system.qcomgpsd.qcomgpsd.threading.Thread")
    retry_xtra_time_after_start(False)
    thread.assert_called_once()
    assert thread.call_args.kwargs["daemon"] is True
    thread.return_value.start.assert_called_once()


class TestClockJumpReinject(OpenpilotTestCase):
  def _monitor(self, clk, send, offset_fn=None, window_s=CLOCK_JUMP_MONITOR_S, threshold=CLOCK_JUMP_THRESHOLD,
               original_offset=None):
    return monitor_clock_jump_and_reinject(
      send_fn=send, sleep_fn=clk.sleep, offset_fn=offset_fn or (lambda: clk.offset),
      original_offset=clk.offset if original_offset is None else original_offset,
      monotonic_fn=clk.monotonic, window_s=window_s, poll_s=1.0, threshold=threshold,
      max_attempts=XTRA_TIME_MAX_ATTEMPTS, retry_delay=1.0)

  def test_no_jump_no_second_injection(self):
    clk = FakeClock()
    sends = []
    assert self._monitor(clk, lambda: sends.append(1) or True) is False
    assert sends == []
    assert clk.mono >= CLOCK_JUMP_MONITOR_S

  def test_15s_jump_reinjects_once(self):
    clk = FakeClock()
    sends = []

    def offset():
      return clk.offset if clk.mono < 5 else clk.offset + 15

    assert self._monitor(clk, lambda: sends.append(1) or True, offset_fn=offset) is True
    assert sends == [1]

  def test_one_hour_jump_reinjects_once(self):
    clk = FakeClock()
    sends = []

    def offset():
      return clk.offset if clk.mono < 5 else clk.offset + 3600

    assert self._monitor(clk, lambda: sends.append(1) or True, offset_fn=offset) is True
    assert sends == [1]

  def test_small_offset_drift_ignored(self):
    clk = FakeClock()
    sends = []

    def offset():
      return clk.offset + 3.0

    assert self._monitor(clk, lambda: sends.append(1) or True, offset_fn=offset) is False
    assert sends == []
    assert clock_offset_jumped(100.0, 103.0) is False
    assert clock_offset_jumped(100.0, 100.2) is False
    assert clock_offset_jumped(100.0, 109.999) is False
    assert clock_offset_jumped(100.0, 110.0) is True
    assert clock_offset_jumped(100.0, 110.001) is True

  def test_corrected_ok_stops_immediately(self):
    clk = FakeClock()
    sends = []

    def offset():
      return clk.offset if clk.mono < 1 else clk.offset + 15

    assert self._monitor(clk, lambda: sends.append(1) or True, offset_fn=offset) is True
    assert sends == [1]

  def test_corrected_fails_then_succeeds(self):
    clk = FakeClock()
    results = [False, False, True]
    sends = []

    def send():
      sends.append(results.pop(0))
      return sends[-1]

    def offset():
      return clk.offset if clk.mono < 1 else clk.offset + 15

    assert self._monitor(clk, send, offset_fn=offset) is True
    assert sends == [False, False, True]

  def test_all_corrected_injections_fail(self, mocker):
    mocker.patch("openpilot.system.qcomgpsd.qcomgpsd.cloudlog.error")
    clk = FakeClock()
    sends = []

    def offset():
      return clk.offset if clk.mono < 1 else clk.offset + 15

    assert self._monitor(clk, lambda: sends.append(1) or False, offset_fn=offset) is False
    assert sends == [1] * XTRA_TIME_MAX_ATTEMPTS

  def test_monitoring_window_expires(self):
    clk = FakeClock()
    sends = []
    assert self._monitor(clk, lambda: sends.append(1) or True, window_s=90.0) is False
    assert sends == []
    assert clk.mono >= 90.0

  def test_exact_10s_jump_reinjects(self):
    clk = FakeClock()
    sends = []
    baseline = clk.offset
    clk.offset += CLOCK_JUMP_THRESHOLD
    assert self._monitor(clk, lambda: sends.append(1) or True, original_offset=baseline) is True
    assert sends == [1]

  def test_just_under_threshold_no_reinject(self):
    clk = FakeClock()
    sends = []
    baseline = clk.offset
    clk.offset += 9.999
    assert self._monitor(clk, lambda: sends.append(1) or True, original_offset=baseline) is False
    assert sends == []

  def test_baseline_before_injection_detects_immediate_jump(self):
    clk = FakeClock()
    baseline = clk.offset
    clk.offset += 15
    sends = []
    assert self._monitor(clk, lambda: sends.append(1) or True, original_offset=baseline) is True
    assert sends == [1]

  def test_late_injection_uses_send_time_baseline(self):
    clk = FakeClock(offset=100.0)
    valid_left = [False, False, True]

    def time_valid():
      if not valid_left:
        return True
      v = valid_left.pop(0)
      if v:
        clk.offset = 200.0
      return v

    sent, baseline = wait_until_xtra_time_sent(
      send_fn=lambda: True, sleep_fn=clk.sleep, interval=1.0,
      time_valid_fn=time_valid, offset_fn=lambda: clk.offset, max_attempts=5)
    assert sent is True
    assert baseline == 200.0

    sends = []
    assert self._monitor(clk, lambda: sends.append(1) or True, original_offset=baseline) is False
    assert sends == []
