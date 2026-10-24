import math
import struct
from datetime import datetime

import pytest

from openpilot.system.ubloxd import pigeond
from openpilot.system.ubloxd.gps_arbiter import ExternalFix, UERE

UTC0 = 1_792_876_061_000  # 2026-10-24 21:07:41 UTC


def fix(received: float = 100., **kw) -> ExternalFix:
  fields = {"latitude": 42.971638, "longitude": -87.893982, "altitude": 209.7, "speed": 10., "bearing_deg": 90., "hdop": 0.5,
            "satellites": 12, "quality": 2, "fix_type": 3, "utc_ms": UTC0, "received": received}
  fields.update(kw)
  return ExternalFix(**fields)


class Pigeon:
  def __init__(self):
    self.sent: list[bytes] = []

  def send_with_ack(self, dat: bytes, ack=None, nack=None) -> None:
    assert ack == pigeond.UBLOX_ASSIST_ACK
    self.sent.append(dat)


def checksum_ok(msg: bytes) -> bool:
  return pigeond.add_ubx_checksum(msg[:-2]) == msg


class TestDue:
  def test_due_after_a_fixless_spell(self):
    assert pigeond.jetson_aid_due(130., last_ublox_fix=110., last_aid=-math.inf, fix=fix(129.))
    assert not pigeond.jetson_aid_due(129., last_ublox_fix=110., last_aid=-math.inf, fix=fix(128.))

  def test_rate_limited(self):
    assert not pigeond.jetson_aid_due(200., last_ublox_fix=0., last_aid=100., fix=fix(199.))
    assert pigeond.jetson_aid_due(220., last_ublox_fix=0., last_aid=100., fix=fix(219.))

  def test_needs_a_fresh_valid_fix(self):
    assert not pigeond.jetson_aid_due(130., 0., -math.inf, None)
    assert not pigeond.jetson_aid_due(130., 0., -math.inf, fix(127.))  # 3 s old
    assert not pigeond.jetson_aid_due(130., 0., -math.inf, fix(129., satellites=3))
    assert not pigeond.jetson_aid_due(130., 0., -math.inf, fix(131.))  # from the future


class TestSend:
  def test_time_and_position(self):
    pigeon = Pigeon()
    pigeond.send_jetson_aid(pigeon, fix(100.), 100.8)
    assert len(pigeon.sent) == 2
    time_msg, pos_msg = pigeon.sent
    assert all(checksum_ok(m) for m in pigeon.sent)

    # UBX-MGA-INI-TIME_UTC: the fix's UTC plus its 0.8 s age, good to 2 s
    assert time_msg[:6] == b"\xB5\x62\x13\x40\x18\x00"
    typ, _ver, _ref, leap, year, month, day, hour, minute, sec, ns, t_acc_s, t_acc_ns = struct.unpack("<BBBBHBBBBBxIHxxI", time_msg[6:-2])
    assert (typ, leap) == (0x10, 0x80)
    assert datetime(year, month, day, hour, minute, sec) == datetime(2026, 10, 24, 21, 7, 41)
    assert ns == pytest.approx(800_000_000, abs=1_000)
    assert (t_acc_s, t_acc_ns) == (pigeond.JETSON_AID_TIME_ACC, 0)

    # UBX-MGA-INI-POS_LLH
    assert pos_msg[:6] == b"\xB5\x62\x13\x40\x14\x00"
    typ, _ver, lat, lon, alt, acc = struct.unpack("<BB2xiiiI", pos_msg[6:-2])
    assert typ == 0x01
    assert (lat, lon, alt) == (429716380, -878939820, 20970)
    assert acc == round((0.5 * UERE + 10. * 0.8 + pigeond.JETSON_AID_POS_MARGIN) * 100)

  def test_system_time_send_is_unchanged(self, monkeypatch):
    monkeypatch.setattr(pigeond, "system_time_valid", lambda: False)
    pigeon = Pigeon()
    pigeond.send_time(pigeon)
    assert pigeon.sent == []
    monkeypatch.setattr(pigeond, "system_time_valid", lambda: True)
    pigeond.send_time(pigeon)
    assert struct.unpack("<BBBBHBBBBBxIHxxI", pigeon.sent[0][6:-2])[11] == 30
