import datetime
import json
import os
import subprocess
import tempfile
from types import SimpleNamespace

from openpilot.common.test import OpenpilotTestCase
from openpilot.common.time_helpers import MAX_DATE, min_date
from openpilot.system.timed import (
  MAX_CELLULAR_SAMPLE_AGE,
  apply_clock_sources,
  gps_time_if_valid,
  parse_cellular_time,
  read_cellular_time,
  set_time,
)


def _valid_dt():
  return (min_date() + datetime.timedelta(days=2)).replace(microsecond=0)


def _state(sample, mono):
  return {
    "network_time_utc": sample.strftime("%Y-%m-%dT%H:%M:%SZ"),
    "network_time_monotonic": mono,
  }


def _record(store, result=True):
  def fn(t):
    store.append(t)
    return result
  return fn


class TestCellularTime(OpenpilotTestCase):
  def test_fresh_sample(self):
    sample = _valid_dt()
    t = parse_cellular_time(_state(sample, 1000.0), now_mono=1000.0)
    assert t == sample

  def test_sample_5_seconds_old(self):
    sample = _valid_dt()
    t = parse_cellular_time(_state(sample, 1000.0), now_mono=1005.0)
    assert t == sample + datetime.timedelta(seconds=5)

  def test_stale_sample(self):
    sample = _valid_dt()
    t = parse_cellular_time(_state(sample, 1000.0), now_mono=1000.0 + MAX_CELLULAR_SAMPLE_AGE + 1)
    assert t is None

  def test_rejects_missing_or_malformed_utc(self):
    assert parse_cellular_time({}, now_mono=1.0) is None
    assert parse_cellular_time({"network_time_utc": ""}, now_mono=1.0) is None
    assert parse_cellular_time({"network_time_utc": "not-a-time", "network_time_monotonic": 1.0}, now_mono=1.0) is None
    assert parse_cellular_time({"network_time_utc": "2026-08-27 18:33:48", "network_time_monotonic": 1.0}, now_mono=1.0) is None
    assert parse_cellular_time({"network_time_utc": 123, "network_time_monotonic": 1.0}, now_mono=1.0) is None

  def test_rejects_malformed_monotonic(self):
    sample = _valid_dt()
    raw = sample.strftime("%Y-%m-%dT%H:%M:%SZ")
    assert parse_cellular_time({"network_time_utc": raw}, now_mono=1.0) is None
    assert parse_cellular_time({"network_time_utc": raw, "network_time_monotonic": None}, now_mono=1.0) is None
    assert parse_cellular_time({"network_time_utc": raw, "network_time_monotonic": "100"}, now_mono=100.0) is None
    assert parse_cellular_time({"network_time_utc": raw, "network_time_monotonic": True}, now_mono=1.0) is None
    assert parse_cellular_time({"network_time_utc": raw, "network_time_monotonic": float("nan")}, now_mono=1.0) is None
    assert parse_cellular_time({"network_time_utc": raw, "network_time_monotonic": float("inf")}, now_mono=1.0) is None

  def test_rejects_negative_age(self):
    sample = _valid_dt()
    assert parse_cellular_time(_state(sample, 1001.0), now_mono=1000.0) is None

  def test_rejects_out_of_range(self):
    assert parse_cellular_time({"network_time_utc": "2020-01-01T00:00:00Z", "network_time_monotonic": 1.0}, now_mono=1.0) is None
    too_late = (MAX_DATE + datetime.timedelta(days=1)).strftime("%Y-%m-%dT%H:%M:%SZ")
    assert parse_cellular_time({"network_time_utc": too_late, "network_time_monotonic": 1.0}, now_mono=1.0) is None

  def test_read_cellular_time_from_file(self):
    sample = _valid_dt()
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, "modem")
      with open(path, "w") as f:
        json.dump(_state(sample, 50.0), f)
      assert read_cellular_time(path, now_mono=50.0) == sample
      assert read_cellular_time(path, now_mono=55.0) == sample + datetime.timedelta(seconds=5)

  def test_read_cellular_time_missing_file(self):
    assert read_cellular_time("/tmp/openpilot-missing-modem-state") is None

  def test_registration_loss_cleared_state_is_unusable(self):
    assert parse_cellular_time({"network_time_utc": "", "network_time_monotonic": 0.0}, now_mono=10.0) is None


class TestClockSources(OpenpilotTestCase):
  def test_gps_wins_over_cellular(self):
    setter = []
    gps = _valid_dt()
    cell = gps + datetime.timedelta(seconds=30)
    gps_used, applied = apply_clock_sources(gps, cell, False, _record(setter))
    assert gps_used is True
    assert applied is False
    assert setter == [gps]

  def test_first_cellular_success_marks_applied(self):
    setter = []
    cell = _valid_dt()
    gps_used, applied = apply_clock_sources(None, cell, False, _record(setter, True))
    assert gps_used is False
    assert applied is True
    assert setter == [cell]

  def test_first_cellular_deadband_counts_as_applied(self):
    setter = []
    cell = _valid_dt()
    gps_used, applied = apply_clock_sources(None, cell, False, _record(setter, True))
    assert applied is True
    assert setter == [cell]

  def test_first_cellular_set_time_failure_not_applied(self):
    setter = []
    cell = _valid_dt()
    gps_used, applied = apply_clock_sources(None, cell, False, _record(setter, False))
    assert gps_used is False
    assert applied is False
    assert setter == [cell]

  def test_cellular_retries_after_set_time_failure(self):
    setter = []
    cell1 = _valid_dt()
    cell2 = cell1 + datetime.timedelta(seconds=30)
    _, applied = apply_clock_sources(None, cell1, False, _record(setter, False))
    assert applied is False
    _, applied = apply_clock_sources(None, cell2, applied, _record(setter, True))
    assert applied is True
    assert setter == [cell1, cell2]

  def test_cellular_one_shot_not_reapplied(self):
    setter = []
    cell1 = _valid_dt()
    cell2 = cell1 + datetime.timedelta(seconds=30)
    _, applied = apply_clock_sources(None, cell1, False, _record(setter, True))
    assert applied is True
    gps_used, applied = apply_clock_sources(None, cell2, applied, _record(setter, True))
    assert gps_used is False
    assert applied is True
    assert setter == [cell1]

  def test_gps_still_wins_after_cellular_one_shot(self):
    setter = []
    cell = _valid_dt()
    gps = cell + datetime.timedelta(seconds=30)
    _, applied = apply_clock_sources(None, cell, False, _record(setter, True))
    gps_used, applied = apply_clock_sources(gps, cell, applied, _record(setter, True))
    assert gps_used is True
    assert setter == [cell, gps]

  def test_neither_source(self):
    setter = []
    gps_used, applied = apply_clock_sources(None, None, False, _record(setter))
    assert gps_used is False
    assert applied is False
    assert setter == []

  def test_gps_time_if_valid(self):
    ts = _valid_dt().replace(tzinfo=datetime.UTC).timestamp() * 1000
    gps = SimpleNamespace(hasFix=True, unixTimestampMillis=ts)
    got = gps_time_if_valid(gps, True, 0.5)
    assert got == _valid_dt()
    assert gps_time_if_valid(gps, False, 0.5) is None
    assert gps_time_if_valid(gps, True, 3.0) is None
    assert gps_time_if_valid(SimpleNamespace(hasFix=False, unixTimestampMillis=ts), True, 0.5) is None


class TestSetTime(OpenpilotTestCase):
  def _patch_now(self, mocker, now):
    class FakeDateTime(datetime.datetime):
      @classmethod
      def now(cls, tz=None):
        dt = cls(now.year, now.month, now.day, now.hour, now.minute, now.second)
        return dt.replace(tzinfo=tz) if tz else dt
    mocker.patch("openpilot.system.timed.datetime.datetime", FakeDateTime)
    return mocker.patch("openpilot.system.timed.subprocess.run")

  def test_invalid_system_time_plus_fresh_cellular(self, mocker):
    run = self._patch_now(mocker, datetime.datetime(2020, 1, 1, 0, 0, 0))
    sample = _valid_dt()
    cellular = parse_cellular_time(_state(sample, 10.0), now_mono=10.0)
    assert set_time(cellular) is True
    run.assert_called_once()

  def test_valid_but_wrong_system_time_plus_fresh_cellular(self, mocker):
    wrong = (min_date() + datetime.timedelta(days=1)).replace(hour=0, minute=0, second=0, microsecond=0)
    run = self._patch_now(mocker, wrong)
    sample = _valid_dt()
    cellular = parse_cellular_time(_state(sample, 10.0), now_mono=15.0)
    assert cellular is not None
    assert set_time(cellular) is True
    run.assert_called_once()

  def test_already_correct_clock_is_success(self, mocker):
    sample = _valid_dt()
    run = self._patch_now(mocker, sample)
    assert set_time(sample + datetime.timedelta(seconds=2)) is True
    run.assert_not_called()

  def test_set_time_command_failure(self, mocker):
    run = self._patch_now(mocker, datetime.datetime(2020, 1, 1, 0, 0, 0))
    run.side_effect = subprocess.CalledProcessError(1, "date")
    sample = _valid_dt()
    assert set_time(sample) is False

  def test_stale_cellular_sample_is_not_applied(self):
    sample = _valid_dt()
    setter = []
    cellular = parse_cellular_time(_state(sample, 1.0), now_mono=1.0 + MAX_CELLULAR_SAMPLE_AGE + 5)
    apply_clock_sources(None, cellular, False, _record(setter))
    assert setter == []

  def test_nitz_does_not_compete_after_first_correction(self, mocker):
    run = self._patch_now(mocker, datetime.datetime(2020, 1, 1, 0, 0, 0))
    sample = _valid_dt()
    first = parse_cellular_time(_state(sample, 10.0), now_mono=10.0)
    second = parse_cellular_time(_state(sample, 10.0), now_mono=40.0)
    _, applied = apply_clock_sources(None, first, False, set_time)
    assert applied is True
    run.assert_called_once()
    apply_clock_sources(None, second, applied, set_time)
    run.assert_called_once()
