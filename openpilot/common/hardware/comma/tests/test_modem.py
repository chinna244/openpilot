from openpilot.common.test import OpenpilotTestCase
from openpilot.common.hardware.comma.modem import (
  NETWORK_TIME_CLEARED,
  NETWORK_TIME_REFRESH_INTERVAL,
  NETWORK_TIME_RETRY_INTERVAL,
  NETWORK_TIME_RETRY_WINDOW,
  Modem,
  State,
  parse_qlts_utc,
)

QLTS = '"2026/08/27,18:33:48-20,1"'
QLTS_UTC = "2026-08-27T18:33:48Z"


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
    assert out == {"network_time_utc": QLTS_UTC, "network_time_monotonic": 10.0}
    self.modem._last_qlts_mono = 0.0
    self.modem._qlts_registered_since = 0.0
    out = self.modem._poll_network_time("roaming", now=11.0)
    assert out["network_time_utc"] == QLTS_UTC

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

  def test_keeps_last_good_value_on_at_failure(self):
    self.modem.S["network_time_utc"] = QLTS_UTC
    self.modem.S["network_time_monotonic"] = 10.0
    self.modem._last_qlts_mono = 0.0
    self.modem._atv = lambda *args, **kwargs: None
    assert self.modem._poll_network_time("home", now=20.0) == {}
    assert self.modem.S["network_time_utc"] == QLTS_UTC
    assert self.modem.S["network_time_monotonic"] == 10.0

  def test_queries_immediately_on_registration(self):
    out = self.modem._poll_network_time("home", now=100.0)
    assert self.calls == ["AT+QLTS=1"]
    assert out == {"network_time_utc": QLTS_UTC, "network_time_monotonic": 100.0}

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

  def test_retries_quickly_until_sample(self):
    self.modem._atv = lambda *args, **kwargs: None
    assert self.modem._poll_network_time("home", now=100.0) == {}
    self.modem._atv = self._atv
    assert self.modem._poll_network_time("home", now=100.0 + NETWORK_TIME_RETRY_INTERVAL - 0.1) == {}
    assert self.calls == []
    out = self.modem._poll_network_time("home", now=100.0 + NETWORK_TIME_RETRY_INTERVAL)
    assert self.calls == ["AT+QLTS=1"]
    assert out["network_time_utc"] == QLTS_UTC

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

  def test_nitz_home_without_cgreg_does_not_connect(self):
    self.creg = "2,1"
    self.cgreg = "0,0"
    assert self.modem._do_searching() == State.SEARCHING
    assert self.published["registration"] == "home"
    assert self.published["network_time_utc"] == QLTS_UTC

  def test_packet_data_still_required_to_connect(self):
    self.creg = "2,1"
    self.cgreg = "2,1"
    assert self.modem._do_searching() == State.CONNECTING
    assert self.published["network_time_utc"] == QLTS_UTC

  def test_searching_clears_time_on_registration_loss(self):
    self.modem.S["network_time_utc"] = QLTS_UTC
    self.modem.S["network_time_monotonic"] = 12.0
    self.modem.S["registration"] = "home"
    self.creg = "2,0"
    self.cgreg = "0,0"
    assert self.modem._do_searching() == State.SEARCHING
    assert self.published["registration"] == "not_registered"
    assert self.published["network_time_utc"] == ""
    assert self.published["network_time_monotonic"] == 0.0
