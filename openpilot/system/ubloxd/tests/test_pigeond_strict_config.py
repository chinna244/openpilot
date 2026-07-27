from collections import deque
from datetime import UTC, datetime
from types import SimpleNamespace

import pytest

from openpilot.system.ubloxd import pigeond
from openpilot.system.ubloxd.gps_assistance import (
  ItfmConfig,
  MessageRateConfig,
  Nav5Config,
  OdoConfig,
  RateConfig,
  UbxStreamParser,
  add_ubx_checksum,
  build_cfg_itfm_poll_message,
  build_cfg_msg_poll_message,
  build_cfg_nav5_poll_message,
  build_cfg_odo_poll_message,
  build_cfg_rate_poll_message,
  parse_cfg_itfm,
  parse_cfg_msg,
  parse_cfg_nav5,
  parse_cfg_odo,
  parse_cfg_rate,
)


def network_host_observation():
  return pigeond.HostTimeObservation(
    utc=datetime(2026, 7, 23, 12, tzinfo=UTC),
    observed_boottime_seconds=100.0,
    uncertainty_seconds=30.0,
    source=pigeond.HostTimeSource.NETWORK_SYNCHRONIZED,
    independent=True,
    generation="network:test",
  )


def ubx_frame(message_class: int, message_id: int, payload: bytes) -> bytes:
  return add_ubx_checksum(
    b"\xb5\x62"
    + bytes((message_class, message_id))
    + len(payload).to_bytes(2, "little")
    + payload
  )


def cfg_ack(message_id: int, *, accepted: bool = True) -> bytes:
  return ubx_frame(0x05, 0x01 if accepted else 0x00, bytes((0x06, message_id)))


def cfg_write(message_id: int, payload: bytes = b"\x01") -> bytes:
  return ubx_frame(0x06, message_id, payload)


def expected_port_config(port_id: int) -> pigeond.PortConfig:
  return {
    0: pigeond.PortConfig(0, 0, 0, 0, 0, 0, 0),
    1: pigeond.PortConfig(1, 0, 0x08C0, 460800, 1, 1, 0),
    3: pigeond.PortConfig(3, 0, 0, 0, 1, 1, 0),
    4: pigeond.PortConfig(4, 0, 0, 0, 0, 0, 0),
  }[port_id]


def cfg_rate_frame(
  measurement_period_ms: int = 100,
  navigation_rate: int = 1,
  time_reference: int = 0,
) -> bytes:
  payload = (
    measurement_period_ms.to_bytes(2, "little")
    + navigation_rate.to_bytes(2, "little")
    + time_reference.to_bytes(2, "little")
  )
  return ubx_frame(0x06, 0x08, payload)


def cfg_prt_frame(config: pigeond.PortConfig) -> bytes:
  payload = bytearray(20)
  payload[0] = config.port_id
  payload[2:4] = config.tx_ready.to_bytes(2, "little")
  payload[4:8] = config.mode.to_bytes(4, "little")
  payload[8:12] = config.baud_rate.to_bytes(4, "little")
  payload[12:14] = config.input_protocol_mask.to_bytes(2, "little")
  payload[14:16] = config.output_protocol_mask.to_bytes(2, "little")
  payload[16:18] = config.flags.to_bytes(2, "little")
  return ubx_frame(0x06, 0x00, bytes(payload))


def cfg_nav5_frame(dynamic_model: int = 4, fix_mode: int = 3) -> bytes:
  payload = bytearray(36)
  payload[2] = dynamic_model
  payload[3] = fix_mode
  return ubx_frame(0x06, 0x24, bytes(payload))


def cfg_odo_frame(flags: int = 1, profile: int = 3) -> bytes:
  payload = bytearray(20)
  payload[4] = flags
  payload[5] = profile
  return ubx_frame(0x06, 0x1E, bytes(payload))


def cfg_itfm_frame(config: int = 0xAD62ADFF, config2: int = 0x0000631E) -> bytes:
  return ubx_frame(
    0x06,
    0x39,
    config.to_bytes(4, "little") + config2.to_bytes(4, "little"),
  )


def cfg_msg_frame(message_class: int, message_id: int, uart1_rate: int = 1) -> bytes:
  return ubx_frame(
    0x06,
    0x01,
    bytes((message_class, message_id, 0, uart1_rate, 0, 0, 0, 0)),
  )


def sos_frame(command: int, response: int) -> bytes:
  return ubx_frame(
    0x09, 0x14, bytes((command, 0, 0, 0, response, 0, 0, 0)),
  )


def nav_pvt_frame() -> bytes:
  payload = bytearray(92)
  payload[20] = 3
  payload[21] = 1
  payload[23] = 7
  return ubx_frame(0x01, 0x07, bytes(payload))


def rawx_frame() -> bytes:
  # Version 1 RXM-RAWX header with no measurements.
  payload = bytearray(16)
  payload[8:10] = (2300).to_bytes(2, "little")
  payload[10] = 18
  payload[13] = 1
  return ubx_frame(0x02, 0x15, bytes(payload))


def mga_ack(message: bytes, *, accepted: bool) -> bytes:
  payload = bytes((1 if accepted else 0, 0, 0 if accepted else 1, message[3]))
  payload += message[6:10].ljust(4, b"\x00")
  return ubx_frame(0x13, 0x60, payload)


class ScriptedPigeon(pigeond.TTYPigeon):
  def __init__(self, responses=(), pre_transaction=()):
    self.responses = deque(responses)
    self.available = deque(pre_transaction)
    self.sent: list[bytes] = []
    self.published: list[bytes] = []
    self._stream_parser = UbxStreamParser()
    self._pending_frames: deque[bytes] = deque()
    self._pending_frame_bytes = 0
    self._pending_unpublished = None
    self._raw_publisher = self.published.append
    self._frame_dispatcher = None
    self._receiver_cycle = 0

  def _receive_tty(self) -> bytes:
    return self.available.popleft() if self.available else b""

  def send(self, data: bytes) -> None:
    self.sent.append(data)
    if self.responses:
      response = self.responses.popleft()
      if isinstance(response, bytes):
        self.available.append(response)
      else:
        self.available.extend(response)


def test_send_with_ack_accepts_only_matching_cfg_ack():
  message = cfg_write(0x08)
  pigeon = ScriptedPigeon((cfg_ack(0x08),))
  pigeon.send_with_ack(message)
  assert pigeon.sent == [message]


def test_send_with_ack_raises_for_matching_cfg_nak():
  pigeon = ScriptedPigeon((cfg_ack(0x08, accepted=False),))
  with pytest.raises(pigeond.CfgNakError, match="0x06 0x08"):
    pigeon.send_with_ack(cfg_write(0x08))


def test_invalid_cfg_frame_is_not_sent():
  invalid = bytearray(cfg_write(0x08))
  invalid[-1] ^= 0xFF
  pigeon = ScriptedPigeon()
  with pytest.raises(pigeond.ReceiverConfigurationError, match="invalid UBX CFG"):
    pigeon.send_with_ack(bytes(invalid))
  assert pigeon.sent == []


def test_cfg_ack_ignores_unrelated_ack_before_matching_ack():
  received = cfg_ack(0x24) + cfg_ack(0x08)
  pigeon = ScriptedPigeon((received,))
  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08)
  assert pigeon.published == [received]
  assert pigeon.receive_normal() == (b"", [cfg_ack(0x24)])


def test_cfg_ack_ignores_corrupt_checksum():
  corrupt = bytearray(cfg_ack(0x08))
  corrupt[-1] ^= 0xFF
  pigeon = ScriptedPigeon((bytes(corrupt),))
  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08, timeout=0.005) is None


def test_cfg_ack_like_bytes_inside_another_frame_do_not_match():
  outer = ubx_frame(0x01, 0x07, b"prefix" + cfg_ack(0x08) + b"suffix")
  pigeon = ScriptedPigeon((outer,))
  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08, timeout=0.005) is None
  assert pigeon.published == [outer]
  assert pigeon.receive_normal() == (b"", [outer])


@pytest.mark.parametrize(("responses", "expected"), [
  (lambda: cfg_ack(0x08) + cfg_ack(0x08, accepted=False), True),
  (lambda: cfg_ack(0x08, accepted=False) + cfg_ack(0x08), False),
])
def test_cfg_ack_and_nak_in_one_read_follow_frame_order(responses, expected):
  pigeon = ScriptedPigeon((responses(),))
  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08) is expected


def test_fragmented_cfg_ack_is_reassembled():
  acknowledgment = cfg_ack(0x08)
  pigeon = ScriptedPigeon(((acknowledgment[:3], acknowledgment[3:8], acknowledgment[8:]),))
  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08)


def test_partial_ack_from_timed_out_attempt_cannot_complete_in_next_transaction():
  acknowledgment = cfg_ack(0x08)
  pigeon = ScriptedPigeon(((acknowledgment[:6],), (acknowledgment[6:],)))

  first = pigeon.begin_response_transaction(cfg_write(0x08, b"first"))
  assert pigeond.wait_for_cfg_ack(pigeon, first, 0x06, 0x08, timeout=0.005) is None

  second = pigeon.begin_response_transaction(cfg_write(0x08, b"second"))
  assert pigeond.wait_for_cfg_ack(pigeon, second, 0x06, 0x08, timeout=0.005) is None
  assert pigeon.sent == [cfg_write(0x08, b"first"), cfg_write(0x08, b"second")]


def test_complete_delayed_ack_is_drained_before_retry_write():
  acknowledgment = cfg_ack(0x08)
  pigeon = ScriptedPigeon((b"",), pre_transaction=(acknowledgment,))

  transaction = pigeon.begin_response_transaction(cfg_write(0x08, b"retry"))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08, timeout=0.005) is None
  assert pigeon.published == [acknowledgment]
  assert pigeon.receive_normal() == (b"", [acknowledgment])


def test_delayed_same_key_ack_cannot_satisfy_next_receiver_cycle():
  acknowledgment = cfg_ack(0x08)
  pigeon = ScriptedPigeon((b"", b""))

  first = pigeon.begin_response_transaction(cfg_write(0x08, b"first"))
  assert pigeond.wait_for_cfg_ack(
    pigeon, first, 0x06, 0x08, timeout=0.005,
  ) is None

  pigeon.available.append(acknowledgment)
  pigeon.reset_response_state()
  second = pigeon.begin_response_transaction(cfg_write(0x08, b"second"))
  assert pigeond.wait_for_cfg_ack(
    pigeon, second, 0x06, 0x08, timeout=0.005,
  ) is None
  assert pigeon.published == [acknowledgment]
  assert pigeon.receive_normal() == (b"", [acknowledgment])


def test_same_key_ack_after_official_response_window_is_not_reused(monkeypatch):
  clock = SimpleNamespace(now=0.0)
  monkeypatch.setattr(pigeond.time, "monotonic", lambda: clock.now)
  monkeypatch.setattr(
    pigeond.time,
    "sleep",
    lambda duration: setattr(clock, "now", clock.now + duration),
  )
  pigeon = ScriptedPigeon((b"", b""))

  first = pigeon.begin_response_transaction(cfg_write(0x08, b"first"))
  assert pigeond.wait_for_cfg_ack(
    pigeon, first, 0x06, 0x08, timeout=pigeond.CFG_ACK_TIMEOUT,
  ) is None
  assert clock.now > 1.1

  pigeon.available.append(cfg_ack(0x08))
  pigeon.reset_response_state()
  second = pigeon.begin_response_transaction(cfg_write(0x08, b"second"))
  assert pigeond.wait_for_cfg_ack(
    pigeon, second, 0x06, 0x08, timeout=0.005,
  ) is None


def test_consecutive_cfg_prt_polls_correlate_same_message_id_by_payload():
  port_one = bytearray(20)
  port_one[0] = 1
  port_three = bytearray(20)
  port_three[0] = 3
  pigeon = ScriptedPigeon((ubx_frame(0x06, 0x00, port_one), ubx_frame(0x06, 0x00, port_three)))

  assert pigeond.poll_cfg_prt(pigeon, 1).port_id == 1
  assert pigeond.poll_cfg_prt(pigeon, 3).port_id == 3
  assert [message[3] for message in pigeon.sent] == [0x00, 0x00]


@pytest.mark.parametrize("port_id", [0, 1, 3, 4])
def test_cfg_prt_poll_uses_documented_one_byte_port_id(port_id):
  config = expected_port_config(port_id)
  pigeon = ScriptedPigeon((cfg_prt_frame(config),))
  assert pigeond.poll_cfg_prt(pigeon, port_id) == config
  assert pigeon.sent == [pigeond.build_cfg_prt_poll_message(port_id)]
  assert pigeon.sent[0][4:6] == b"\x01\x00"
  assert pigeon.sent[0][6] == port_id


def test_cfg_prt_zero_length_poll_is_never_transmitted():
  responses = tuple(cfg_prt_frame(expected_port_config(port)) for port in (0, 1, 3, 4))
  pigeon = ScriptedPigeon(responses)
  for port_id in (0, 1, 3, 4):
    pigeond.poll_cfg_prt(pigeon, port_id)
  assert all(message[4:6] == b"\x01\x00" for message in pigeon.sent)


def test_cfg_prt_wrong_port_id_fails():
  pigeon = ScriptedPigeon((cfg_prt_frame(expected_port_config(3)),))
  with pytest.raises(pigeond.CfgPollTimeoutError):
    pigeond.poll_cfg_prt(pigeon, 1, timeout=0.005)


@pytest.mark.parametrize(("port_id", "field", "fails"), [
  (1, "baud_rate", True),
  (1, "mode", True),
  (1, "input_protocol_mask", True),
  (1, "output_protocol_mask", True),
  (1, "flags", True),
  (0, "baud_rate", False),
  (0, "flags", True),
  (3, "baud_rate", False),
  (3, "mode", False),
  (3, "input_protocol_mask", True),
  (3, "flags", False),
  (4, "baud_rate", False),
  (4, "flags", True),
])
def test_cfg_prt_verifies_only_fields_explicit_for_port(port_id, field, fails):
  expected = expected_port_config(port_id)
  actual = expected.__class__(**{
    **expected.__dict__,
    field: getattr(expected, field) ^ 1,
  })
  if fails:
    with pytest.raises(pigeond.ReceiverConfigurationError, match="CFG-PRT"):
      pigeond.verify_cfg_prt_config(actual, expected)
  else:
    pigeond.verify_cfg_prt_config(actual, expected)


def test_consecutive_cfg_msg_polls_correlate_same_message_id_by_payload():
  pigeon = ScriptedPigeon((cfg_msg_frame(0x01, 0x07), cfg_msg_frame(0x02, 0x15)))

  assert pigeond.poll_cfg_msg(pigeon, 0x01, 0x07).message_id == 0x07
  assert pigeond.poll_cfg_msg(pigeon, 0x02, 0x15).message_id == 0x15
  assert [message[3] for message in pigeon.sent] == [0x01, 0x01]


def test_pre_transaction_drain_publishes_but_does_not_correlate_stale_bytes():
  nav = nav_pvt_frame()
  stale_ack = cfg_ack(0x08)
  current_ack = cfg_ack(0x08)
  pigeon = ScriptedPigeon((current_ack,), pre_transaction=(nav + stale_ack,))

  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08)
  assert pigeon.published == [nav + stale_ack, current_ack]
  assert pigeon.receive_normal() == (b"", [nav, stale_ack])


def test_multiple_frames_in_one_read_preserve_unrelated_order_once():
  nav = ubx_frame(0x01, 0x07, b"nav")
  rawx = ubx_frame(0x02, 0x15, b"rawx")
  received = nav + cfg_ack(0x08) + rawx
  pigeon = ScriptedPigeon((received,))
  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08)
  assert pigeon.published == [received]
  assert pigeon.receive_normal() == (b"", [nav, rawx])
  assert pigeon.receive() == b""


def test_cfg_poll_response_and_unrelated_frame_are_preserved():
  nav = ubx_frame(0x01, 0x07, b"nav")
  received = nav + cfg_rate_frame()
  pigeon = ScriptedPigeon((received,))
  assert pigeond.poll_cfg_rate(pigeon) == RateConfig(100, 1, 0)
  assert pigeon.sent == [build_cfg_rate_poll_message()]
  assert pigeon.published == [received]
  assert pigeon.receive_normal() == (b"", [nav])


def test_corrupt_cfg_poll_response_is_ignored_until_timeout():
  corrupt = bytearray(cfg_rate_frame())
  corrupt[-1] ^= 0xFF
  pigeon = ScriptedPigeon((bytes(corrupt),))
  with pytest.raises(TimeoutError, match="No valid CFG response"):
    pigeond.poll_cfg_rate(pigeon, timeout=0.005)


def test_cfg_poll_timeout():
  pigeon = ScriptedPigeon()
  with pytest.raises(TimeoutError, match="No valid CFG response"):
    pigeond.poll_cfg_rate(pigeon, timeout=0.005)


def test_cfg_rate_readback_success_and_mismatch():
  assert parse_cfg_rate(cfg_rate_frame()) == RateConfig(100, 1, 0)
  valid = (
    RateConfig(100, 1, 0),
    Nav5Config(4, 3),
    OdoConfig(0, 1, 3),
    ItfmConfig(0xAD62ADFF, 0x0000631E),
    MessageRateConfig(0x01, 0x07, (0, 1, 0, 0, 0, 0)),
    MessageRateConfig(0x02, 0x15, (0, 1, 0, 0, 0, 0)),
  )
  pigeond.verify_startup_configuration(*valid)
  with pytest.raises(pigeond.ReceiverConfigurationError, match="CFG-RATE"):
    pigeond.verify_startup_configuration(RateConfig(1000, 1, 0), *valid[1:])


@pytest.mark.parametrize(("message_class", "message_id"), [(0x01, 0x07), (0x02, 0x15)])
def test_cfg_msg_nav_pvt_and_rawx_readback(message_class, message_id):
  frame = cfg_msg_frame(message_class, message_id)
  expected = MessageRateConfig(message_class, message_id, (0, 1, 0, 0, 0, 0))
  assert parse_cfg_msg(frame) == expected
  pigeon = ScriptedPigeon((frame,))
  assert pigeond.poll_cfg_msg(pigeon, message_class, message_id) == expected
  assert pigeon.sent == [build_cfg_msg_poll_message(message_class, message_id)]


def test_cfg_msg_poll_preserves_other_message_configuration():
  unrelated = cfg_msg_frame(0x01, 0x35)
  expected_frame = cfg_msg_frame(0x02, 0x15)
  received = unrelated + expected_frame
  pigeon = ScriptedPigeon((received,))
  assert pigeond.poll_cfg_msg(pigeon, 0x02, 0x15).message_id == 0x15
  assert pigeon.published == [received]
  assert pigeon.receive_normal() == (b"", [unrelated])


def test_cfg_nav5_odo_and_itfm_parsing_and_polls():
  nav5 = cfg_nav5_frame()
  odo = cfg_odo_frame()
  itfm = cfg_itfm_frame()
  assert parse_cfg_nav5(nav5) == Nav5Config(4, 3)
  assert parse_cfg_odo(odo) == OdoConfig(0, 1, 3)
  assert parse_cfg_itfm(itfm) == ItfmConfig(0xAD62ADFF, 0x0000631E)

  pigeon = ScriptedPigeon((nav5, odo, itfm))
  assert pigeond.poll_cfg_nav5(pigeon) == Nav5Config(4, 3)
  assert pigeond.poll_cfg_odo(pigeon) == OdoConfig(0, 1, 3)
  assert pigeond.poll_cfg_itfm(pigeon) == ItfmConfig(0xAD62ADFF, 0x0000631E)
  assert pigeon.sent == [
    build_cfg_nav5_poll_message(),
    build_cfg_odo_poll_message(),
    build_cfg_itfm_poll_message(),
  ]


def test_fragmented_unrelated_rawx_is_preserved_for_later_publication():
  rawx = ubx_frame(0x02, 0x15, b"rawx")
  acknowledgment = cfg_ack(0x08)
  chunks = (rawx[:5], rawx[5:] + acknowledgment)
  pigeon = ScriptedPigeon((chunks,))
  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08)
  assert b"".join(pigeon.published) == rawx + acknowledgment
  assert pigeon.receive_normal() == (b"", [rawx])
  assert pigeon.receive_normal() == (b"", [])


def test_rawx_fragment_crossing_normal_and_response_wait_is_preserved_once():
  rawx = ubx_frame(0x02, 0x15, b"rawx")
  acknowledgment = cfg_ack(0x08)
  pigeon = ScriptedPigeon((rawx[5:] + acknowledgment,), pre_transaction=(rawx[:5],))

  data, frames = pigeon.receive_normal()
  assert data == rawx[:5]
  assert frames == []
  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08)

  data, frames = pigeon.receive_normal()
  assert data == b""
  assert frames == [rawx]
  assert b"".join(pigeon.published) == rawx + acknowledgment
  assert pigeon.receive_normal() == (b"", [])


def test_nav_pvt_fragment_crossing_normal_and_response_wait_is_preserved_once():
  nav = nav_pvt_frame()
  acknowledgment = cfg_ack(0x08)
  pigeon = ScriptedPigeon(((nav[9:] + acknowledgment,),), pre_transaction=(nav[:9],))

  assert pigeon.receive_normal() == (nav[:9], [])
  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08)
  assert pigeon.receive_normal() == (b"", [nav])
  assert b"".join(pigeon.published) == nav + acknowledgment


def test_rawx_fragment_starts_during_wait_and_completes_afterward():
  rawx = rawx_frame()
  acknowledgment = cfg_ack(0x08)
  pigeon = ScriptedPigeon(((acknowledgment + rawx[:7],),))

  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08)
  pigeon.available.append(rawx[7:])
  assert pigeon.receive_normal() == (rawx[7:], [rawx])
  assert b"".join(pigeon.published) == acknowledgment + rawx


def test_pubmaster_callback_publishes_each_uart_chunk_exactly_once(monkeypatch):
  sent = []

  class PubMaster:
    def send(self, service, message):
      sent.append((service, bytes(message.ubloxRaw)))

  monkeypatch.setattr(
    pigeond.messaging,
    "new_message",
    lambda _service, _size, valid: SimpleNamespace(ubloxRaw=b"", valid=valid),
  )
  pm = PubMaster()
  pigeon = ScriptedPigeon((cfg_ack(0x08),), pre_transaction=(nav_pvt_frame(),))
  pigeon._raw_publisher = lambda data: pigeond.publish_ublox_raw(pm, data)

  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08)
  assert sent == [
    ("ubloxRaw", nav_pvt_frame()),
    ("ubloxRaw", cfg_ack(0x08)),
  ]


def test_valid_nav_pvt_and_rawx_reach_higher_level_processing_once():
  nav = nav_pvt_frame()
  rawx = rawx_frame()
  acknowledgment = cfg_ack(0x08)
  pigeon = ScriptedPigeon((nav + rawx + acknowledgment,))

  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08)
  _, frames = pigeon.receive_normal()
  assert frames == [nav, rawx]
  assert sum(pigeond.parse_nav_pvt(frame) is not None for frame in frames) == 1
  diagnostics = pigeond.GpsStartupDiagnostics(0.0)
  for frame in frames:
    diagnostics.note_rawx(frame, 1.0)
  assert diagnostics.first_rawx_after_initialization_logged
  assert pigeon.receive_normal() == (b"", [])


def test_matching_response_is_not_queued_for_internal_processing_twice():
  acknowledgment = cfg_ack(0x08)
  pigeon = ScriptedPigeon((acknowledgment,))
  transaction = pigeon.begin_response_transaction(cfg_write(0x08))
  assert pigeond.wait_for_cfg_ack(pigeon, transaction, 0x06, 0x08)
  assert pigeon.receive_normal() == (b"", [])


def test_pending_frames_dispatch_between_cfg_transactions():
  dispatched = []
  nav = nav_pvt_frame()
  pigeon = ScriptedPigeon((nav + cfg_ack(0x08), cfg_ack(0x24)))
  pigeon._frame_dispatcher = lambda frames: dispatched.extend(frames)
  pigeon.send_with_ack(cfg_write(0x08))
  pigeon.send_with_ack(cfg_write(0x24))
  assert dispatched == [nav]
  assert pigeon.receive_normal() == (b"", [])


def test_pending_frames_dispatch_between_mga_transactions():
  dispatched = []
  nav = nav_pvt_frame()
  first = pigeond.build_time_assistance_message(datetime(2026, 7, 10, tzinfo=UTC))
  second = pigeond.build_position_assistance_message(0, 0, 0, 1000)
  pigeon = ScriptedPigeon((nav + mga_ack(first, accepted=True), mga_ack(second, accepted=True)))
  pigeon._frame_dispatcher = lambda frames: dispatched.extend(frames)
  pigeond.send_mga_with_strict_ack(pigeon, first)
  pigeond.send_mga_with_strict_ack(pigeon, second)
  assert dispatched == [nav]


def test_pending_frames_dispatch_between_mga_dbd_restore_frames(monkeypatch):
  position = pigeond.build_position_assistance_message(0, 0, 0, 1000)
  database_frames = (
    ubx_frame(0x13, 0x80, b"database-0"),
    ubx_frame(0x13, 0x80, b"database-1"),
  )
  nav = nav_pvt_frame()
  rawx = rawx_frame()
  pigeon = ScriptedPigeon((
    mga_ack(position, accepted=True),
    nav + mga_ack(database_frames[0], accepted=True),
    rawx + mga_ack(database_frames[1], accepted=True),
  ))
  dispatched = []
  pigeon._frame_dispatcher = lambda frames: dispatched.extend(frames)
  cache = SimpleNamespace(
    saved_at_utc=datetime(2026, 7, 10, tzinfo=UTC),
    rtc_counter_seconds=100,
    quality=None,
    database_frames=database_frames,
    latitude_e7=0,
    longitude_e7=0,
    altitude_cm=0,
    position_accuracy_cm=1000,
  )
  monkeypatch.setattr(pigeond, "read_host_time_observation", lambda: None)
  monkeypatch.setattr(pigeond, "load_cache", lambda *args, **kwargs: cache)

  result = pigeond.restore_navigation_assistance(pigeon, "receiver")
  assert result.status is pigeond.NavigationAssistanceRestoreStatus.COMPLETE
  assert result.accepted_frame_count == 2
  assert dispatched == [nav, rawx]


def test_raw_publisher_failure_retains_then_publishes_chunk_once():
  frame = nav_pvt_frame()
  attempts = []
  processed = []
  pigeon = ScriptedPigeon(pre_transaction=(frame,))

  def publish(data):
    attempts.append(data)
    if len(attempts) == 1:
      raise RuntimeError("temporary messaging failure")

  pigeon._raw_publisher = publish
  with pytest.raises(pigeond.RawPublicationError):
    pigeon._read_stream()
  assert pigeon._pending_unpublished == frame
  assert pigeon._stream_parser._buffer == b""

  data, frames = pigeon._read_stream()
  processed.extend(frames)
  assert data == frame
  assert attempts == [frame, frame]
  assert processed == [frame]
  assert pigeon._pending_unpublished is None


def test_generic_ubx_ack_for_wrong_class_id_is_rejected(monkeypatch):
  monkeypatch.setattr(pigeond, "CFG_ACK_TIMEOUT", 0.005)
  command = ubx_frame(0x09, 0x14, b"\x01\x00\x00\x00")
  pigeon = ScriptedPigeon((cfg_ack(0x08),))
  with pytest.raises(TimeoutError):
    pigeon.send_with_ack(command)


def test_sos_clear_rejects_ack_for_another_command(monkeypatch):
  monkeypatch.setattr(pigeond, "CFG_ACK_TIMEOUT", 0.005)
  clear = ubx_frame(0x09, 0x14, b"\x01\x00\x00\x00")
  pigeon = ScriptedPigeon((cfg_ack(0x24),))
  with pytest.raises(TimeoutError):
    pigeon.send_with_ack(clear)


def test_legacy_mga_path_rejects_unrelated_mga_ack(monkeypatch):
  monkeypatch.setattr(pigeond, "GPS_ASSISTANCE_ACK_TIMEOUT", 0.005)
  message = pigeond.build_time_assistance_message(datetime(2026, 7, 10, tzinfo=UTC))
  unrelated = pigeond.build_position_assistance_message(0, 0, 0, 1000)
  pigeon = ScriptedPigeon((mga_ack(unrelated, accepted=True),))
  with pytest.raises(TimeoutError):
    pigeon.send_with_ack(message, ack=pigeond.UBLOX_ASSIST_ACK)


def test_mixed_corrupt_and_valid_frames_in_one_read():
  corrupt = bytearray(cfg_ack(0x08))
  corrupt[-1] ^= 0xFF
  nav = nav_pvt_frame()
  pigeon = ScriptedPigeon((bytes(corrupt) + nav + cfg_ack(0x08),))
  pigeon.send_with_ack(cfg_write(0x08))
  assert pigeon.receive_normal() == (b"", [nav])


def test_response_state_reset_across_power_cycle_discards_parser_state_only():
  nav = nav_pvt_frame()
  pigeon = ScriptedPigeon(pre_transaction=(nav[:8],))
  assert pigeon.receive_normal() == (nav[:8], [])
  pigeon.reset_response_state()
  pigeon.available.append(nav[8:])
  assert pigeon.receive_normal() == (nav[8:], [])
  assert b"".join(pigeon.published) == nav


def test_init_resets_response_state_before_receiver_power_cycle(monkeypatch):
  events = []

  class ResetTrackingPigeon(ScriptedPigeon):
    def reset_response_state(self):
      events.append("reset")
      super().reset_response_state()

  nav = nav_pvt_frame()
  pigeon = ResetTrackingPigeon(pre_transaction=(nav[:8],))
  assert pigeon.receive_normal() == (nav[:8], [])
  monkeypatch.setattr(pigeond.signal, "signal", lambda *_args: None)
  monkeypatch.setattr(pigeond, "set_power", lambda enabled: events.append(f"power={enabled}"))
  monkeypatch.setattr(pigeond, "init_baudrate", lambda _pigeon: events.append("baud"))
  monkeypatch.setattr(pigeond, "init_pigeon", lambda _pigeon: True)
  monkeypatch.setattr(pigeond.time, "sleep", lambda _duration: None)

  pigeond.init(pigeon)

  assert events[:3] == ["reset", "power=False", "power=True"]
  assert pigeon.published == [nav[:8]]


def test_pending_frames_were_already_published_before_reset():
  nav = nav_pvt_frame()
  pigeon = ScriptedPigeon(pre_transaction=(nav,))
  pigeon.drain_before_transaction()
  assert pigeon.published == [nav]
  pigeon.reset_response_state()
  assert pigeon.receive_normal() == (b"", [])
  assert pigeon.published == [nav]


def test_pending_frame_queue_bound_and_drain(monkeypatch):
  frames = [ubx_frame(0x01, 0x07, bytes((index,))) for index in range(2)]
  pigeon = ScriptedPigeon()
  monkeypatch.setattr(pigeond, "PENDING_FRAME_MAX_COUNT", 2)
  monkeypatch.setattr(pigeond, "PENDING_FRAME_MAX_BYTES", sum(map(len, frames)))
  pigeon.queue_pending_frames(frames)
  with pytest.raises(pigeond.PendingFrameOverflowError):
    pigeon.queue_pending_frames([frames[0]])
  assert pigeon.receive_normal() == (b"", [*frames, frames[0]])
  pigeon.queue_pending_frames([frames[0]])
  assert pigeon.receive_normal() == (b"", [frames[0]])


def test_queue_overflow_during_initialization_is_controlled(monkeypatch):
  logs = []
  frames = nav_pvt_frame() + rawx_frame()
  pigeon = ScriptedPigeon(pre_transaction=(frames,))
  monkeypatch.setattr(pigeond, "PENDING_FRAME_MAX_COUNT", 1)
  monkeypatch.setattr(pigeond.cloudlog, "error", logs.append)

  assert not pigeond.init_pigeon(pigeon)
  assert pigeon.sent == []
  assert len(pigeon._pending_frames) == 2
  assert "frame_count=2" in logs[0]
  assert "operation=cfg_write_06_00" in logs[0]
  assert "receiver_cycle=0" in logs[0]
  assert "frame_limit" in logs[0]


def test_queue_overflow_during_mga_restore_returns_failed_result(monkeypatch):
  frames = nav_pvt_frame() + rawx_frame()
  pigeon = ScriptedPigeon(pre_transaction=(frames,))
  cache = SimpleNamespace(
    saved_at_utc=datetime(2026, 7, 10, tzinfo=UTC),
    rtc_counter_seconds=100,
    quality=None,
    database_frames=(ubx_frame(0x13, 0x80, b"dbd"),),
    latitude_e7=0,
    longitude_e7=0,
    altitude_cm=0,
    position_accuracy_cm=1000,
  )
  monkeypatch.setattr(pigeond, "PENDING_FRAME_MAX_COUNT", 1)
  monkeypatch.setattr(pigeond, "read_host_time_observation", lambda: None)
  monkeypatch.setattr(pigeond, "load_cache", lambda *args, **kwargs: cache)

  result = pigeond.restore_navigation_assistance(pigeon, "receiver")
  assert result.status is pigeond.NavigationAssistanceRestoreStatus.FAILED
  assert result.accepted_frame_count == 0
  assert result.failure_phase is pigeond.NavigationAssistanceRestoreFailurePhase.POSITION_ASSISTANCE_WRITE
  assert len(pigeon._pending_frames) == 2




def test_cfg_ack_between_half_and_one_second_is_accepted(monkeypatch):
  clock = SimpleNamespace(now=0.0)

  class DelayedPigeon(ScriptedPigeon):
    def __init__(self):
      super().__init__()
      self.armed = False
      self.delivered = False

    def send(self, data):
      self.sent.append(data)
      self.armed = True

    def _receive_tty(self):
      if self.armed and not self.delivered and clock.now >= 0.75:
        self.delivered = True
        return cfg_ack(0x08)
      return b""

  monkeypatch.setattr(pigeond.time, "monotonic", lambda: clock.now)
  monkeypatch.setattr(pigeond.time, "sleep", lambda duration: setattr(clock, "now", clock.now + duration))
  pigeon = DelayedPigeon()
  pigeon.send_with_ack(cfg_write(0x08))
  assert 0.75 <= clock.now < pigeond.CFG_ACK_TIMEOUT


def test_cfg_ack_times_out_after_new_bounded_timeout(monkeypatch):
  clock = SimpleNamespace(now=0.0)
  monkeypatch.setattr(pigeond.time, "monotonic", lambda: clock.now)
  monkeypatch.setattr(pigeond.time, "sleep", lambda duration: setattr(clock, "now", clock.now + duration))
  pigeon = ScriptedPigeon()
  with pytest.raises(TimeoutError):
    pigeon.send_with_ack(cfg_write(0x08))
  assert clock.now >= pigeond.CFG_ACK_TIMEOUT


def test_effective_startup_configuration_logging(monkeypatch):
  logs = []
  monkeypatch.setattr(pigeond.cloudlog, "info", logs.append)
  pigeond.log_startup_configuration(
    RateConfig(100, 1, 0),
    Nav5Config(4, 3),
    OdoConfig(0, 1, 3),
    ItfmConfig(0xAD62ADFF, 0x0000631E),
    MessageRateConfig(0x01, 0x07, (0, 1, 0, 0, 0, 0)),
    MessageRateConfig(0x02, 0x15, (0, 1, 0, 0, 0, 0)),
  )
  assert "measurement_period_ms=100" in logs[0]
  assert "navigation_rate=1" in logs[0]
  assert "time_reference=0" in logs[0]
  assert any("CFG-MSG NAV-PVT effective" in message and "uart1_rate=1" in message for message in logs)
  assert any("CFG-MSG RXM-RAWX effective" in message and "uart1_rate=1" in message for message in logs)


@pytest.mark.parametrize("failures", [1, 3])
def test_init_pigeon_retries_after_matching_nak(monkeypatch, failures):
  class RetryingPigeon:
    def __init__(self):
      self.ack_writes = []

    def send_with_ack(self, message, **kwargs):
      self.ack_writes.append(message)
      if len(self.ack_writes) <= failures:
        raise pigeond.CfgNakError("matching CFG NAK")

    def send(self, message):
      pass

    def poll_backup_restore_status(self):
      return 3

  monkeypatch.setattr(pigeond, "poll_cfg_prt", lambda _pigeon, port_id: expected_port_config(port_id))
  monkeypatch.setattr(pigeond, "poll_cfg_rate", lambda *args: RateConfig(100, 1, 0))
  monkeypatch.setattr(pigeond, "poll_cfg_nav5", lambda *args: Nav5Config(4, 3))
  monkeypatch.setattr(pigeond, "poll_cfg_odo", lambda *args: OdoConfig(0, 1, 3))
  monkeypatch.setattr(pigeond, "poll_cfg_itfm", lambda *args: ItfmConfig(0xAD62ADFF, 0x0000631E))
  monkeypatch.setattr(
    pigeond,
    "poll_cfg_msg",
    lambda _pigeon, message_class, message_id: MessageRateConfig(
      message_class, message_id, (0, 1, 0, 0, 0, 0),
    ),
  )
  monkeypatch.setattr(pigeond, "Params", lambda: type("Params", (), {"get": lambda self, key: None})())

  pigeon = RetryingPigeon()
  assert pigeond.init_pigeon(pigeon)
  assert pigeon.ack_writes[:failures] == [pigeon.ack_writes[0]] * failures
  assert pigeon.ack_writes[failures] == pigeon.ack_writes[0]


def test_init_pigeon_timeout_then_successful_retry(monkeypatch):
  class RetryingPigeon:
    def __init__(self):
      self.writes = 0

    def send_with_ack(self, _message, **_kwargs):
      self.writes += 1
      if self.writes == 1:
        raise TimeoutError("matching CFG ACK timed out")

    def send(self, _message):
      pass

    def poll_backup_restore_status(self):
      return 3

  monkeypatch.setattr(pigeond, "poll_cfg_prt", lambda _pigeon, port_id: expected_port_config(port_id))
  monkeypatch.setattr(pigeond, "poll_cfg_rate", lambda *args: RateConfig(100, 1, 0))
  monkeypatch.setattr(pigeond, "poll_cfg_nav5", lambda *args: Nav5Config(4, 3))
  monkeypatch.setattr(pigeond, "poll_cfg_odo", lambda *args: OdoConfig(0, 1, 3))
  monkeypatch.setattr(pigeond, "poll_cfg_itfm", lambda *args: ItfmConfig(0xAD62ADFF, 0x0000631E))
  monkeypatch.setattr(
    pigeond,
    "poll_cfg_msg",
    lambda _pigeon, message_class, message_id: MessageRateConfig(
      message_class, message_id, (0, 1, 0, 0, 0, 0),
    ),
  )
  monkeypatch.setattr(pigeond, "Params", lambda: type("Params", (), {"get": lambda self, key: None})())

  pigeon = RetryingPigeon()
  assert pigeond.init_pigeon(pigeon)
  assert pigeon.writes > 1


def test_cfg_nak_retry_waits_for_complete_official_response_window(monkeypatch):
  clock = SimpleNamespace(now=0.0)

  class BoundaryPigeon:
    def __init__(self):
      self.write_times = []

    def send_with_ack(self, _message, **_kwargs):
      self.write_times.append(clock.now)
      if len(self.write_times) == 1:
        raise pigeond.CfgNakError(
          "matching CFG NAK",
          clock.now + pigeond.CFG_ACK_TIMEOUT,
        )
      raise pigeond.ResponseTransactionError("stop second cycle")

  monkeypatch.setattr(pigeond.time, "monotonic", lambda: clock.now)
  monkeypatch.setattr(
    pigeond.time,
    "sleep",
    lambda duration: setattr(clock, "now", clock.now + duration),
  )
  pigeon = BoundaryPigeon()

  assert not pigeond.init_pigeon(pigeon)
  assert pigeon.write_times == [0.0, pigeond.CFG_ACK_TIMEOUT]


def test_cfg_nak_retry_boundary_uses_post_send_timestamp(monkeypatch):
  clock = SimpleNamespace(now=10.0)

  class DelayedWritePigeon(ScriptedPigeon):
    def send(self, data):
      self.sent.append(data)
      clock.now += 0.4
      self.available.append(cfg_ack(data[3], accepted=False))

  monkeypatch.setattr(pigeond.time, "monotonic", lambda: clock.now)
  pigeon = DelayedWritePigeon()

  with pytest.raises(pigeond.CfgNakError) as exc_info:
    pigeon.send_with_ack(cfg_write(0x08))

  assert clock.now == pytest.approx(10.4)
  assert exc_info.value.retry_not_before == pytest.approx(11.5)
  assert exc_info.value.retry_not_before != pytest.approx(11.1)


def test_cfg_poll_timeout_aborts_current_initialization_cycle(monkeypatch):
  class PollTimeoutPigeon:
    def __init__(self):
      self.writes = []

    def send_with_ack(self, message, **_kwargs):
      self.writes.append(message)

  polled_ports = []

  def poll_timeout(_pigeon, port_id):
    polled_ports.append(port_id)
    raise pigeond.CfgPollTimeoutError("CFG-PRT response timed out")

  monkeypatch.setattr(pigeond, "poll_cfg_prt", poll_timeout)
  pigeon = PollTimeoutPigeon()

  assert not pigeond.init_pigeon(pigeon)
  assert len(pigeon.writes) == 4
  assert polled_ports == [0]


def test_ten_consecutive_initialization_failures_are_bounded():
  class AlwaysRejectingPigeon:
    def __init__(self):
      self.writes = 0

    def send_with_ack(self, _message, **_kwargs):
      self.writes += 1
      raise pigeond.CfgNakError("matching CFG NAK")

  pigeon = AlwaysRejectingPigeon()
  assert not pigeond.init_pigeon(pigeon)
  assert pigeon.writes == 10


def test_end_to_end_hpg_1_40_protocol_20_30_initialization(monkeypatch):
  responses = [
    *(cfg_ack(0x00) for _ in range(4)),
    *(cfg_prt_frame(expected_port_config(port)) for port in (0, 1, 3, 4)),
    cfg_ack(0x08),
    cfg_rate_frame(),
    cfg_ack(0x24),
    cfg_ack(0x1E),
    cfg_ack(0x39),
    cfg_nav5_frame(),
    cfg_odo_frame(),
    cfg_itfm_frame(),
    *(cfg_ack(0x01) for _ in range(6)),
    cfg_msg_frame(0x01, 0x07),
    cfg_msg_frame(0x02, 0x15),
    sos_frame(3, 3),
  ]
  monkeypatch.setattr(
    pigeond,
    "Params",
    lambda: SimpleNamespace(get=lambda _key: None),
  )
  pigeon = ScriptedPigeon(responses)

  assert pigeond.init_pigeon(pigeon)
  cfg_prt_polls = [
    message for message in pigeon.sent
    if message[2:4] == b"\x06\x00" and message[4:6] == b"\x01\x00"
  ]
  assert cfg_prt_polls == [
    pigeond.build_cfg_prt_poll_message(port) for port in (0, 1, 3, 4)
  ]
  assert not any(
    message[2:6] == b"\x06\x00\x00\x00" for message in pigeon.sent
  )
  assert not pigeon.responses
  assert not pigeon.available


@pytest.mark.parametrize("flags", [0x01])
def test_cfg_odo_required_low_flag_nibble_passes(flags):
  pigeond.verify_startup_configuration(
    RateConfig(100, 1, 0),
    Nav5Config(4, 3),
    OdoConfig(0, flags, 3),
    ItfmConfig(0xAD62ADFF, 0x0000631E),
    MessageRateConfig(0x01, 0x07, (0, 1, 0, 0, 0, 0)),
    MessageRateConfig(0x02, 0x15, (0, 1, 0, 0, 0, 0)),
  )


@pytest.mark.parametrize("flags", [0x03, 0x05, 0x09])
def test_cfg_odo_other_documented_low_flag_bits_fail(flags):
  with pytest.raises(pigeond.ReceiverConfigurationError, match="CFG-ODO"):
    pigeond.verify_startup_configuration(
      RateConfig(100, 1, 0),
      Nav5Config(4, 3),
      OdoConfig(0, flags, 3),
      ItfmConfig(0xAD62ADFF, 0x0000631E),
      MessageRateConfig(0x01, 0x07, (0, 1, 0, 0, 0, 0)),
      MessageRateConfig(0x02, 0x15, (0, 1, 0, 0, 0, 0)),
    )


@pytest.mark.parametrize(("acknowledgment", "time_assistance_expected"), [
  (pigeond.MgaAck(True, 1, 0, 0, 0x40, b"\x10\x00\x00\x80"), True),
  (pigeond.MgaAck(False, 0, 0, 1, 0x40, b"\x10\x00\x00\x80"), False),
  (None, False),
])
def test_cache_restore_is_independent_of_time_assistance_ack(
  monkeypatch,
  acknowledgment,
  time_assistance_expected,
):
  events = []

  class Diagnostics:
    def start_cycle(self, reason, now):
      pass

    def time_assistance_context(self, now):
      return "cycle=1"

  monkeypatch.setattr(pigeond, "init", lambda pigeon: None)
  monkeypatch.setattr(pigeond, "poll_mon_ver", lambda pigeon: None)
  monkeypatch.setattr(pigeond, "log_navx5_ack_aiding_support", lambda info: False)
  monkeypatch.setattr(pigeond, "configure_navx5_ack_aiding", lambda *args: None)
  monkeypatch.setattr(pigeond, "read_host_time_observation", network_host_observation)
  monkeypatch.setattr(pigeond, "wait_for_matching_mga_ack", lambda *args, **kwargs: acknowledgment)
  monkeypatch.setattr(
    pigeond,
    "restore_navigation_assistance",
    lambda *args, **kwargs: events.append("restore"),
  )
  monkeypatch.setattr(pigeond, "log_assistnow_autonomous_support", lambda info: False)
  monkeypatch.setattr(pigeond, "configure_assistnow_autonomous", lambda *args: None)

  result = pigeond.initialize_receiver_cycle(
    ScriptedPigeon(), "receiver", Diagnostics(), "process_start",
  )
  assert result.trusted_time_assistance_sent is time_assistance_expected
  assert result.navigation_assistance_restore_attempted is True
  assert events == ["restore"]


def test_time_assistance_observation_failure_is_not_success(monkeypatch):
  logs = []
  monkeypatch.setattr(pigeond.cloudlog, "exception", logs.append)
  monkeypatch.setattr(
    pigeond,
    "wait_for_matching_mga_ack",
    lambda *args, **kwargs: (_ for _ in ()).throw(OSError("read failed")),
  )
  assert not pigeond.send_time_assistance(
    ScriptedPigeon(),
    assistance_time=pigeond.datetime(2026, 7, 10, tzinfo=pigeond.UTC),
  )
  assert "write_result=succeeded" in logs[0]
  assert "ack_result=observation_failed" in logs[0]


def test_real_mga_rejection_then_later_checksum_valid_acceptance():
  assistance_time = datetime(2026, 7, 10, tzinfo=UTC)
  message = pigeond.build_time_assistance_message(assistance_time)
  pigeon = ScriptedPigeon((
    mga_ack(message, accepted=False),
    mga_ack(message, accepted=True),
  ))

  assert not pigeond.send_time_assistance(pigeon, assistance_time=assistance_time)
  assert pigeond.send_time_assistance(pigeon, assistance_time=assistance_time)


def test_real_mga_timeout_then_later_checksum_valid_acceptance():
  assistance_time = datetime(2026, 7, 10, tzinfo=UTC)
  message = pigeond.build_time_assistance_message(assistance_time)
  pigeon = ScriptedPigeon((b"", mga_ack(message, accepted=True)))

  assert not pigeond.send_time_assistance(
    pigeon, assistance_time=assistance_time, ack_timeout=0.005,
  )
  assert pigeond.send_time_assistance(pigeon, assistance_time=assistance_time)


def test_checksum_valid_sos_restore_response():
  pigeon = ScriptedPigeon((sos_frame(3, 2),))
  assert pigeon.poll_backup_restore_status() == 2


def test_corrupt_sos_restore_response_is_ignored():
  corrupt = bytearray(sos_frame(3, 2))
  corrupt[-1] ^= 0xFF
  pigeon = ScriptedPigeon((bytes(corrupt),))
  with pytest.raises(TimeoutError):
    pigeon.poll_backup_restore_status(timeout=0.005)


def test_upd_sos_invalid_responses_are_ignored_until_valid_restore_status():
  bad_checksum = bytearray(sos_frame(3, 2))
  bad_checksum[-1] ^= 0xFF
  invalid = (
    sos_frame(2, 1),
    sos_frame(3, 0),
    ubx_frame(0x01, 0x14, bytes((3, 0, 0, 0, 2, 0, 0, 0))),
    ubx_frame(0x09, 0x13, bytes((3, 0, 0, 0, 2, 0, 0, 0))),
    ubx_frame(0x09, 0x14, bytes((3, 0, 0, 0, 2, 0, 0))),
    bytes(bad_checksum),
  )
  valid = sos_frame(3, 2)
  pigeon = ScriptedPigeon((invalid + (valid,),))

  assert pigeon.poll_backup_restore_status() == 2
  assert b"".join(pigeon.published) == b"".join(invalid) + valid
