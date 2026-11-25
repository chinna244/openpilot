"""The almanac backup is saved at shutdown, after the controlled GNSS stop, and power always turns off."""
import pytest

from openpilot.system.ubloxd import pigeond as pig

SOS_CREATE = b"\xB5\x62\x09\x14\x04\x00\x00"


class FakePigeon:
  def __init__(self, reply):
    self.sent, self.reply, self.buf = [], reply, b''

  def send(self, dat):
    self.sent.append(dat)
    if dat.startswith(b"\xB5\x62\x09\x14"):
      self.buf = self.reply

  def receive(self):
    dat, self.buf = self.buf, b''
    return dat

  def wait_for_ack(self, **kwargs):
    return pig.TTYPigeon.wait_for_ack(self, **kwargs)


@pytest.mark.parametrize('reply', [pig.UBLOX_SOS_ACK, pig.UBLOX_SOS_NACK, b''])
def test_shutdown_stops_gnss_then_saves_then_powers_off(monkeypatch, reply):
  power = []
  monkeypatch.setattr(pig, 'set_power', power.append)
  monkeypatch.setattr(pig.time, 'sleep', lambda _s: None)
  fake = FakePigeon(reply)
  with pytest.raises(SystemExit):
    pig.deinitialize_and_exit(fake)
  assert fake.sent[0] == pig.UBLOX_GNSS_STOP
  assert fake.sent[1].startswith(SOS_CREATE)
  assert power == [False]
