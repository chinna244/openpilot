"""
Word 27 under AOL: ACC_ACTIVE/CRZ_ACTIVE wait for Panda's longitudinal grant.

Route 00000001--f5fdd9a482: at 4 of 5 engagements (77.1, 157.0, 241.6, 280.9 s) the controller sent ACC-active
CRZ_INFO/CRZ_CTRL on CC.enabled, 30-90 ms before Panda's AOL long grant (mask bit 2, the host's request, reported
at 10 Hz on aolSafetyWire). Panda refused them, so the car saw the CRZ_INFO counter jump (157 s: 06 -> 0A). Filter-dev
grants from its own PEDALS edge and never refused one. The Panda gate stays; the controller sends consecutive inactive
frames until a fresh grant is observed within this engagement.

The real controller and the real Panda safety (libsafety, word 27, AOL) run together frame by frame. The test
stands in for card: `aol_long_native` is None (no fresh native under the freshness rules) or
(requestedLongitudinal and longitudinalAllowed, observedMonoTime) from the current aolSafetyWire report.
"""
import unittest

import pytest

from opendbc.car.mazda.carcontroller import CarController  # noqa: F401
from opendbc.car.mazda.tests.conftest import CRZ_CTRL, CRZ_INFO, mazda_car_state, parse_frame, step
from opendbc.car.mazda.tests.test_mazda_can_filter import CONFIRMED, confirm, filter_controller
from opendbc.car.mazda.values_ext import MazdaFlagsSP
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_mazda_measured_profiles import MazdaAolHarness

BUSES = (0, 2)
REPORT_NS = 100_000_000  # aolSafetyWire cadence


def acc_active(addr, dat):
  return bool(dat[4] & 0x02) if addr == CRZ_INFO else bool(dat[0] & 0x08)


class Drive:
  """One engagement driven through the controller and Panda together."""

  def __init__(self):
    self.cc = filter_controller()
    self.cc.CP_SP.flags |= MazdaFlagsSP.TJA_BUTTON
    self.cs = mazda_car_state(self.cc.CP, self.cc.CP_SP)
    confirm(self.cc, self.cs)
    self.panda = MazdaAolHarness(unittest.TestCase(), 27)
    self.panda.arm()  # TJA lateral grant, mask 1
    self.frame = 0
    self.log = []  # (frame, bus, addr, counter, active, accepted)
    self.obs = 1_000

  def report(self, grant):
    """A new aolSafetyWire report (fresh observation)."""
    self.obs += REPORT_NS
    return (grant, self.obs)

  def run(self, n, *, native, enabled, request=1, gas=False, long_active=False, handback=False, set_edge=False):
    p = self.panda
    p.request = request
    if set_edge:
      p.engage_cruise()
    for _ in range(n):
      p.tick()
      p.feed()
      if gas:
        p.safety.safety_rx_hook(p.msg('ENGINE_DATA', {'SPEED': 80, 'PEDAL_GAS': 50}))
      self.cc.aol_long_native = native
      self.cs.can_filter_state, self.cs.can_filter_age = CONFIRMED, 0
      self.cc.can_filter.receive([CONFIRMED], 0)
      _, sends = step(self.cc, self.cs, enabled=enabled, long_active=long_active, lat_active=True, mads_active=True,
                      available=True, cruise_engaged=enabled, mrcc_armed_raw=True, gas=gas, handback=handback,
                      v_ego=20.)
      for addr, dat, bus in sends:
        if addr in (CRZ_INFO, CRZ_CTRL) and bus in BUSES:
          ok = bool(p.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat)))
          ctr = int(parse_frame(CRZ_INFO, dat, bus)['CTR']) if addr == CRZ_INFO else None
          self.log.append((self.frame, bus, addr, ctr, acc_active(addr, dat), ok))
      self.frame += 1

  def since(self, frame, addr=CRZ_INFO):
    return [e for e in self.log if e[0] >= frame and e[2] == addr]


def refused(log):
  return [e for e in log if not e[5]]


def counters_continuous(log):
  for bus in BUSES:
    ctrs = [e[3] for e in log if e[1] == bus and e[2] == CRZ_INFO and e[5]]
    assert ctrs and all((b - a) % 16 == 1 for a, b in zip(ctrs, ctrs[1:], strict=False)), (bus, ctrs)


@pytest.fixture
def d():
  d = Drive()
  d.run(30, native=d.report(False), enabled=False)  # MAIN on, not engaged: fresh denial
  return d


def engage_with_gas(d):
  """SET with the gas held: Panda grants 5 frames after the edge, the host hears it 3 frames later."""
  set_frame = d.frame
  d.run(5, native=(False, d.obs), enabled=True, gas=True, set_edge=True)
  d.run(3, native=(False, d.obs), enabled=True, gas=True, request=3)
  grant_heard = d.frame
  d.run(20, native=d.report(True), enabled=True, gas=True, request=3)
  return set_frame, grant_heard


def test_set_with_gas_waits_for_the_grant_and_panda_accepts_every_frame(d):
  set_frame, grant_heard = engage_with_gas(d)
  d.run(40, native=(True, d.obs), enabled=True, long_active=True, request=3)
  assert not refused(d.log), refused(d.log)[:4]
  counters_continuous(d.log)
  for addr in (CRZ_INFO, CRZ_CTRL):
    first = next(e[0] for e in d.since(set_frame, addr) if e[4])
    assert first == grant_heard or first == grant_heard + 1, (addr, first, grant_heard)
    assert not any(e[4] for e in d.since(set_frame, addr) if e[0] < grant_heard)


def test_brief_feedback_delay_does_not_flicker_but_expiry_goes_inactive(d):
  engage_with_gas(d)
  # no new report for 150 ms: card re-reads the same observation inside the freshness window
  start = d.frame
  d.run(15, native=(True, d.obs), enabled=True, long_active=True, request=3)
  assert all(e[4] for e in d.since(start) + d.since(start, CRZ_CTRL))
  # the window expired (card: no fresh native): consecutive inactive frames until a fresh grant
  expired = d.frame
  d.run(30, native=None, enabled=True, long_active=True, request=3)
  assert not any(e[4] for e in d.since(expired) + d.since(expired, CRZ_CTRL))
  back = d.frame
  d.run(10, native=d.report(True), enabled=True, long_active=True, request=3)
  assert any(e[4] for e in d.since(back))
  assert not refused(d.log)
  counters_continuous(d.log)


def test_fresh_denial_is_honored(d):
  engage_with_gas(d)
  d.run(10, native=(True, d.obs), enabled=True, long_active=True, request=3)
  revoke = d.frame
  d.run(20, native=d.report(False), enabled=True, request=1)  # Panda and its report both withdraw long
  assert not any(e[4] for e in d.since(revoke) + d.since(revoke, CRZ_CTRL))
  assert not refused(d.log)
  counters_continuous(d.log)


@pytest.mark.parametrize("end", ["disengage", "handback"])
def test_next_set_cannot_reuse_the_old_grant(d, end):
  engage_with_gas(d)
  d.run(10, native=(True, d.obs), enabled=True, long_active=True, request=3)
  stale_grant = (True, d.obs)
  if end == "disengage":
    d.run(10, native=stale_grant, enabled=False, request=1)
  else:
    handback = d.frame
    d.run(10, native=stale_grant, enabled=True, request=1, handback=True)
    assert not any(e[4] for e in d.since(handback) + d.since(handback, CRZ_CTRL))
    confirm(d.cc, d.cs)  # the hand-back released the board; it confirms again (disengaged frames)
  # SET again while the last report still says granted: inactive until a newer report grants
  again = d.frame
  d.run(8, native=stale_grant, enabled=True, gas=True, request=1, set_edge=(end == "disengage"))
  assert not any(e[4] for e in d.since(again) + d.since(again, CRZ_CTRL))
  d.run(5, native=d.report(True), enabled=True, gas=True, request=3)
  assert any(e[4] for e in d.since(again))
  assert not refused(d.log)
  counters_continuous(d.log)
