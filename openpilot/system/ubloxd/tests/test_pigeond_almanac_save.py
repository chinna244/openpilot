"""Shutdown stops GNSS and powers off without an almanac save: this ublox NACKs every UBX-UPD-SOS backup."""
import pytest

from openpilot.system.ubloxd import pigeond as pig

SOS = b"\xB5\x62\x09\x14"


class FakePigeon:
  def __init__(self):
    self.sent = []

  def send(self, dat):
    self.sent.append(dat)


def test_shutdown_stops_gnss_without_save_then_powers_off(monkeypatch):
  power = []
  monkeypatch.setattr(pig, 'set_power', power.append)
  fake = FakePigeon()
  with pytest.raises(SystemExit):
    pig.deinitialize_and_exit(fake)
  assert fake.sent == [pig.UBLOX_GNSS_STOP]
  assert not any(d.startswith(SOS) for d in fake.sent)
  assert power == [False]


def test_shutdown_without_pigeon_powers_off(monkeypatch):
  power = []
  monkeypatch.setattr(pig, 'set_power', power.append)
  with pytest.raises(SystemExit):
    pig.deinitialize_and_exit(None)
  assert power == [False]
