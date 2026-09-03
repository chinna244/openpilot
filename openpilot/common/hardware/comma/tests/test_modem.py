from openpilot.common.test import OpenpilotTestCase
from openpilot.common.hardware.comma.modem import (
  NETWORK_TIME_CLEARED,
  NETWORK_TIME_REFRESH_INTERVAL,
  NETWORK_TIME_RETRY_INTERVAL,
  NETWORK_TIME_RETRY_WINDOW,
  Modem,
  State,
  parse_qlts,
  parse_qlts_utc,
)

QLTS = '"2026/08/27,18:33:48-20,1"'
QLTS_UTC = "2026-08-27T18:33:48Z"
QLTS_OFFSET_QUARTERS = -20
QLTS_DST = 1


def nitz_state(utc, monotonic, offset=QLTS_OFFSET_QUARTERS, dst=QLTS_DST):
  return {
    "network_time_utc": utc,
    "network_time_monotonic": monotonic,
    "network_timezone_offset_quarters": offset,
    "network_time_dst": dst,
  }


class TestParseQltsUtc(OpenpilotTestCase):
  def test_qlts1_is_already_utc(self):
    # Quectel EG916Q: AT+QLTS=1 returns GMT. Do not apply the -20 tz suffix again.
    assert parse_qlts_utc(QLTS) == QLTS_UTC
    assert parse_qlts_utc('2026/08/27,18:33:48-20,1') == QLTS_UTC

  def test_plus_timezone_suffix_ignored(self):
    assert parse_qlts_utc('"2026/08/27,18:33:48+00,0"') == QLTS_UTC

  def test_rejects_placeholder_255(self):
    assert parse_qlts_utc('"2026/08/27,255:255:255-20,1"') is None
    assert parse_qlts_utc('"255/255/255,00:00:00+00,0"') is None
    assert parse_qlts_utc('"2026/255/27,18:33:48-20,1"') is None

  def test_rejects_invalid_calendar(self):
    assert parse_qlts_utc('"2026/02/30,12:00:00+00,0"') is None
    assert parse_qlts_utc('"2026/13/01,12:00:00+00,0"') is None
    assert parse_qlts_utc('"2026/08/27,24:00:00+00,0"') is None

  def test_rejects_malformed(self):
    assert parse_qlts_utc(None) is None
    assert parse_qlts_utc("") is None
    assert parse_qlts_utc("+QLTS: ERROR") is None
    assert parse_qlts_utc("garbage") is None


class TestParseQlts(OpenpilotTestCase):
  def test_offset_minus_20_dst_1(self):
    assert parse_qlts("2026/08/28,14:07:00-20,1") == ("2026-08-28T14:07:00Z", -20, 1)
    assert parse_qlts('"2026/08/28,14:07:00-20,1"') == ("2026-08-28T14:07:00Z", -20, 1)

  def test_offset_minus_24_dst_0(self):
    assert parse_qlts("2026/12/28,14:07:00-24,0") == ("2026-12-28T14:07:00Z", -24, 0)

  def test_offset_plus_22(self):
    assert parse_qlts("2026/08/28,14:07:00+22,0") == ("2026-08-28T14:07:00Z", 22, 0)

  def test_offset_plus_00(self):
    assert parse_qlts("2026/08/28,14:07:00+00,0") == ("2026-08-28T14:07:00Z", 0, 0)

  def test_does_not_add_dst_to_offset(self):
    # DST=1 is +1 hour = 4 quarter-hours. Offset must stay -20, not -16.
    utc, offset, dst = parse_qlts("2026/08/28,14:07:00-20,1")
    assert utc == "2026-08-28T14:07:00Z"
    assert dst == 1
    assert offset == -20
    assert offset != -20 + 4
    assert parse_qlts("2026/08/28,14:07:00-20,0")[1] == -20

  def test_rejects_empty_qlts(self):
    assert parse_qlts(None) is None
    assert parse_qlts("") is None
    assert parse_qlts("   ") is None

  def test_rejects_malformed(self):
    assert parse_qlts("+QLTS: ERROR") is None
    assert parse_qlts("garbage") is None
    assert parse_qlts(12345) is None
    assert parse_qlts(b"2026/08/28,14:07:00-20,1") is None

  def test_rejects_placeholder_255(self):
    assert parse_qlts('"2026/08/27,255:255:255-20,1"') is None
    assert parse_qlts('"255/255/255,00:00:00+00,0"') is None
    assert parse_qlts('"2026/255/27,18:33:48-20,1"') is None

  def test_rejects_invalid_dst(self):
    assert parse_qlts("2026/08/28,14:07:00-20,3") is None
    assert parse_qlts("2026/08/28,14:07:00-20,9") is None
    assert parse_qlts("2026/08/28,14:07:00-20,255") is None

  def test_rejects_invalid_timezone_range(self):
    assert parse_qlts("2026/08/28,14:07:00-49,0") is None
    assert parse_qlts("2026/08/28,14:07:00+57,0") is None
    assert parse_qlts("2026/08/28,14:07:00+99,0") is None

  def test_utc_unchanged_from_parse_qlts_utc(self):
    for raw in (QLTS, '2026/08/27,18:33:48-20,1', '"2026/08/27,18:33:48+00,0"'):
      parsed = parse_qlts(raw)
      assert parsed is not None
      assert parsed[0] == parse_qlts_utc(raw)


class TestPollNetworkTime(OpenpilotTestCase):
  def setup_method(self):
    self.modem = Modem()
    self.calls = []
    self.modem._atv = self._atv

  def _atv(self, cmd, pfx):
    self.calls.append(cmd)
    if cmd == "AT+QLTS=1":
      return QLTS
    return None

  def test_requires_home_or_roaming(self):
    assert self.modem._poll_network_time("searching", now=10.0) == {}
    assert "AT+QLTS=1" not in self.calls
    out = self.modem._poll_network_time("home", now=10.0)
    assert out == nitz_state(QLTS_UTC, 10.0)
    self.modem._last_qlts_mono = 0.0
    self.modem._qlts_registered_since = 0.0
    out = self.modem._poll_network_time("roaming", now=11.0)
    assert out["network_time_utc"] == QLTS_UTC
    assert out["network_timezone_offset_quarters"] == QLTS_OFFSET_QUARTERS
    assert out["network_time_dst"] == QLTS_DST

  def test_clears_home_to_not_registered(self):
    self.modem.S["network_time_utc"] = QLTS_UTC
    self.modem.S["network_time_monotonic"] = 50.0
    assert self.modem._poll_network_time("not_registered", now=60.0) == NETWORK_TIME_CLEARED

  def test_clears_roaming_to_searching(self):
    self.modem.S["network_time_utc"] = QLTS_UTC
    self.modem.S["network_time_monotonic"] = 50.0
    assert self.modem._poll_network_time("searching", now=60.0) == NETWORK_TIME_CLEARED

  def test_clears_roaming_to_denied(self):
    self.modem.S["network_time_utc"] = QLTS_UTC
    self.modem.S["network_time_monotonic"] = 50.0
    assert self.modem._poll_network_time("denied", now=60.0) == NETWORK_TIME_CLEARED

  def test_registration_loss_clears_live_timezone_and_dst(self):
    self.modem.S.update(nitz_state(QLTS_UTC, 50.0, offset=-24, dst=0))
    out = self.modem._poll_network_time("not_registered", now=60.0)
    assert out == NETWORK_TIME_CLEARED
    assert out["network_timezone_offset_quarters"] is None
    assert out["network_time_dst"] is None
    assert out["network_time_utc"] == ""
    assert out["network_time_monotonic"] == 0.0

  def test_keeps_last_good_value_on_at_failure(self):
    self.modem.S.update(nitz_state(QLTS_UTC, 10.0))
    self.modem._last_qlts_mono = 0.0
    self.modem._atv = lambda *args, **kwargs: None
    assert self.modem._poll_network_time("home", now=20.0) == {}
    assert self.modem.S["network_time_utc"] == QLTS_UTC
    assert self.modem.S["network_time_monotonic"] == 10.0
    assert self.modem.S["network_timezone_offset_quarters"] == QLTS_OFFSET_QUARTERS
    assert self.modem.S["network_time_dst"] == QLTS_DST

  def test_malformed_qlts_does_not_replace_last_good(self):
    self.modem.S.update(nitz_state(QLTS_UTC, 10.0, offset=-20, dst=1))
    self.modem._last_qlts_mono = 0.0
    self.modem._atv = lambda *args, **kwargs: "garbage"
    assert self.modem._poll_network_time("home", now=20.0) == {}
    assert self.modem.S["network_time_utc"] == QLTS_UTC
    assert self.modem.S["network_timezone_offset_quarters"] == -20
    assert self.modem.S["network_time_dst"] == 1

  def test_empty_qlts_does_not_replace_last_good(self):
    self.modem.S.update(nitz_state(QLTS_UTC, 10.0))
    self.modem._last_qlts_mono = 0.0
    self.modem._atv = lambda *args, **kwargs: ""
    assert self.modem._poll_network_time("home", now=20.0) == {}
    assert self.modem.S["network_timezone_offset_quarters"] == QLTS_OFFSET_QUARTERS

  def test_queries_immediately_on_registration(self):
    out = self.modem._poll_network_time("home", now=100.0)
    assert self.calls == ["AT+QLTS=1"]
    assert out == nitz_state(QLTS_UTC, 100.0)

  def test_does_not_poll_every_second_after_valid_sample(self):
    self.modem.S["network_time_utc"] = QLTS_UTC
    self.modem._poll_network_time("home", now=100.0)
    self.calls.clear()
    assert self.modem._poll_network_time("home", now=100.0 + 10.0) == {}
    assert self.calls == []

  def test_refreshes_after_interval(self):
    self.modem.S["network_time_utc"] = QLTS_UTC
    self.modem._poll_network_time("home", now=100.0)
    self.calls.clear()
    out = self.modem._poll_network_time("home", now=100.0 + NETWORK_TIME_REFRESH_INTERVAL)
    assert self.calls == ["AT+QLTS=1"]
    assert out["network_time_monotonic"] == 100.0 + NETWORK_TIME_REFRESH_INTERVAL
    assert out["network_time_utc"] == QLTS_UTC
    assert out["network_timezone_offset_quarters"] == QLTS_OFFSET_QUARTERS
    assert out["network_time_dst"] == QLTS_DST

  def test_retries_quickly_until_sample(self):
    self.modem._atv = lambda *args, **kwargs: None
    assert self.modem._poll_network_time("home", now=100.0) == {}
    self.modem._atv = self._atv
    assert self.modem._poll_network_time("home", now=100.0 + NETWORK_TIME_RETRY_INTERVAL - 0.1) == {}
    assert self.calls == []
    out = self.modem._poll_network_time("home", now=100.0 + NETWORK_TIME_RETRY_INTERVAL)
    assert self.calls == ["AT+QLTS=1"]
    assert out["network_time_utc"] == QLTS_UTC
    assert out == nitz_state(QLTS_UTC, 100.0 + NETWORK_TIME_RETRY_INTERVAL)

  def test_slows_retry_after_window_without_sample(self):
    self.modem._atv = lambda *args, **kwargs: None
    self.modem._poll_network_time("home", now=100.0)
    later = 100.0 + NETWORK_TIME_RETRY_WINDOW
    self.modem._poll_network_time("home", now=later)
    self.calls.clear()
    self.modem._atv = self._atv
    assert self.modem._poll_network_time("home", now=later + NETWORK_TIME_RETRY_INTERVAL) == {}
    assert self.calls == []
    out = self.modem._poll_network_time("home", now=later + NETWORK_TIME_REFRESH_INTERVAL)
    assert self.calls == ["AT+QLTS=1"]
    assert out["network_time_utc"] == QLTS_UTC


class TestSearchingNitz(OpenpilotTestCase):
  def setup_method(self):
    self.modem = Modem()
    self.modem._roaming_allowed = True
    self.modem._is_roaming_allowed = lambda: True
    self.modem._searching_idle = lambda: State.SEARCHING
    self.modem._publish_state = self._publish
    self.published = {}
    self.creg = "2,5"
    self.cgreg = "0,0"
    self.modem._atv = self._atv

  def _publish(self, **kwargs):
    self.published.update(kwargs)
    self.modem.S.update(kwargs)

  def _atv(self, cmd, pfx):
    if cmd == "AT+CREG?":
      return self.creg
    if cmd == "AT+CGREG?":
      return self.cgreg
    if cmd == "AT+QLTS=1":
      return QLTS
    return None

  def test_nitz_without_packet_data(self):
    # CREG roaming, CGREG not registered: still query/publish NITZ, do not start PPP.
    assert self.modem._do_searching() == State.SEARCHING
    assert self.published["registration"] == "roaming"
    assert self.published["network_time_utc"] == QLTS_UTC
    assert self.published["network_time_monotonic"] > 0
    assert self.published["network_timezone_offset_quarters"] == QLTS_OFFSET_QUARTERS
    assert self.published["network_time_dst"] == QLTS_DST

  def test_nitz_home_without_cgreg_does_not_connect(self):
    self.creg = "2,1"
    self.cgreg = "0,0"
    assert self.modem._do_searching() == State.SEARCHING
    assert self.published["registration"] == "home"
    assert self.published["network_time_utc"] == QLTS_UTC
    assert self.published["network_timezone_offset_quarters"] == QLTS_OFFSET_QUARTERS

  def test_packet_data_still_required_to_connect(self):
    self.creg = "2,1"
    self.cgreg = "2,1"
    assert self.modem._do_searching() == State.CONNECTING
    assert self.published["network_time_utc"] == QLTS_UTC

  def test_searching_clears_time_on_registration_loss(self):
    self.modem.S["network_time_utc"] = QLTS_UTC
    self.modem.S["network_time_monotonic"] = 12.0
    self.modem.S["network_timezone_offset_quarters"] = QLTS_OFFSET_QUARTERS
    self.modem.S["network_time_dst"] = QLTS_DST
    self.modem.S["registration"] = "home"
    self.creg = "2,0"
    self.cgreg = "0,0"
    assert self.modem._do_searching() == State.SEARCHING
    assert self.published["registration"] == "not_registered"
    assert self.published["network_time_utc"] == ""
    assert self.published["network_time_monotonic"] == 0.0
    assert self.published["network_timezone_offset_quarters"] is None
    assert self.published["network_time_dst"] is None
