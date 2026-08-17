"""Deterministic ublox_available() debounce: no full arbiter reset on one flicker."""

from __future__ import annotations

from openpilot.common.gps_source_arbiter import GpsSourceArbiter
from openpilot.system.gpsard import UBLOX_HW_CONFIRM_SAMPLES, UbloxHwPresence


def test_single_false_sample_does_not_reset():
  p = UbloxHwPresence(committed=True)
  arbiter = GpsSourceArbiter(ublox_hardware_available=True)
  arbiter.reset(now_mono=1.0, ublox_hardware_available=True)
  gen = arbiter.state.generation
  assert p.observe(False) is False
  assert p.committed is True
  assert arbiter.state.generation == gen


def test_single_true_sample_does_not_reset():
  p = UbloxHwPresence(committed=False)
  assert p.observe(True) is False
  assert p.committed is False


def test_persistent_false_resets_once():
  p = UbloxHwPresence(committed=True)
  arbiter = GpsSourceArbiter(ublox_hardware_available=True)
  arbiter.reset(now_mono=1.0, ublox_hardware_available=True)
  resets = 0
  for sample in (False, False, False, False, False):
    if p.observe(sample):
      arbiter.reset(now_mono=2.0, ublox_hardware_available=p.committed)
      resets += 1
  assert resets == 1
  assert p.committed is False
  assert arbiter.state.ublox_hardware_available is False
  assert arbiter.state.startup_complete is False


def test_persistent_true_resets_once():
  p = UbloxHwPresence(committed=False)
  resets = 0
  for sample in (True, True, True, True):
    if p.observe(sample):
      resets += 1
  assert resets == 1
  assert p.committed is True


def test_flicker_false_true_false_does_not_commit():
  p = UbloxHwPresence(committed=True)
  seq = [False, True, False, True, False]
  assert not any(p.observe(s) for s in seq)
  assert p.committed is True


def test_confirm_count_is_three():
  assert UBLOX_HW_CONFIRM_SAMPLES == 3
  p = UbloxHwPresence(committed=True)
  assert p.observe(False) is False
  assert p.observe(False) is False
  assert p.observe(False) is True
