"""Panda revokes Mazda long before card hears it: the host stops ACC_ACTIVE/CRZ_ACTIVE by the native freshness deadline.

The real controller and real Panda safety (word 27, AOL) run together; card's stand-in reads Panda's grant through the
real current_native (aolSafetyWire at 10 Hz, 200 ms lease) and maps it as card.py does (test_card_aol_long_native.py).
"""

from dataclasses import replace

import pytest

from opendbc.car.mazda.tests.conftest import CRZ_CTRL, CRZ_INFO
from opendbc.car.mazda.tests.test_mazda_aol_long_grant import Drive, counters_continuous
from opendbc.car.mazda.tests.test_mazda_can_filter import filter_params
from openpilot.starpilot.aol import runtime
from openpilot.starpilot.aol.tests.test_panda_inventory import BOOT_OFFSET, bound_state, sm_for

FRAME_NS = 10_000_000
REPORT_FRAMES = 10  # aolSafetyWire at 10 Hz
LEASE_FRAMES = runtime.SAFETY_MAX_AGE_NS // FRAME_NS


class Card:
  """Panda's state reported every 100 ms; card reads it each frame through current_native."""

  def __init__(self, cp, t0):
    # the car's AOL config as Panda reports it: word 27 under AOL (the controller fixture predates the AOL bits)
    self.cp, _ = filter_params()
    self.cp.alternativeExperience |= 32
    self.cp.safetyConfigs[-1].safetyParam = 27
    self.t0, self.sm, self.reporting = t0, None, True

  def report(self, now, panda):
    mask = panda.safety.aol_get_permission_mask()
    state = replace(bound_state(self.cp, now=now), requestedLongitudinal=panda.request == 3,
                    longitudinalAllowed=bool(mask & 2), sourcePandaStatesMonoTime=now + BOOT_OFFSET)
    self.sm = sm_for(self.cp, state, panda_stamp=now)

  def native(self, now):
    native = runtime.current_native(self.sm, self.cp, now_ns=now) if self.sm is not None else None
    return None if native is None else (bool(native.requestedLongitudinal and native.longitudinalAllowed),
                                        int(native.observedMonoTime))


class Timed(Drive):
  def __init__(self):
    super().__init__()
    self.card = Card(self.cc.CP, 0)
    self.now = 5_000_000_000

  def go(self, n, **kw):
    for _ in range(n):
      self.now += FRAME_NS
      if self.card.reporting and self.frame % REPORT_FRAMES == 0:
        self.card.report(self.now, self.panda)
      self.run(1, native=self.card.native(self.now), **kw)


def active_frames(d, start):
  return [e for e in d.since(start) + d.since(start, CRZ_CTRL) if e[4]]


@pytest.fixture
def d():
  d = Timed()
  d.go(30, enabled=False)
  d.go(1, enabled=True, gas=True, set_edge=True)
  d.go(4, enabled=True, gas=True)
  d.go(40, enabled=True, request=3, long_active=True)  # Panda grants; card hears it at the next report
  assert active_frames(d, d.frame - 10)
  assert not [e for e in d.log if not e[5]], 'every frame accepted through SET with the gas held'
  return d


@pytest.mark.parametrize("returns", [True, False], ids=["feedback_returns", "feedback_never_returns"])
def test_panda_revokes_before_card_hears(d, returns):
  # Panda withdraws long on the frame after a report, and its reports stop: card keeps reading the last grant
  while d.frame % REPORT_FRAMES != 1:
    d.go(1, enabled=True, request=3, long_active=True)
  last_report = d.card.sm.logMonoTime['aolSafetyWire']
  d.card.reporting = False
  revoke = d.frame
  d.go(1, enabled=True, request=1, long_active=True)
  assert not d.panda.safety.aol_get_permission_mask() & 2
  deadline = last_report + runtime.SAFETY_MAX_AGE_NS
  d.go(LEASE_FRAMES + 5, enabled=True, request=1, long_active=True)
  late = [e for e in active_frames(d, revoke) if d_frame_ns(d, e[0]) > deadline]
  assert not late, late[:2]
  assert active_frames(d, revoke), 'within the freshness window the last grant still stands'
  stop = max(e[0] for e in active_frames(d, revoke))
  inactive = [e for e in d.since(stop + 1) + d.since(stop + 1, CRZ_CTRL)]
  assert inactive and all(e[5] and not e[4] for e in inactive), 'consecutive inactive frames, all accepted'
  counters_continuous([e for e in d.log if e[0] > stop])
  if returns:
    d.card.reporting = True
    back = d.frame
    d.go(5, enabled=True, request=1, long_active=True)  # fresh denial
    assert not active_frames(d, back)
    d.go(25, enabled=True, request=3, long_active=True)  # fresh grant, same engagement
    assert active_frames(d, back)
    assert all(e[5] for e in d.log if e[0] >= back)
    counters_continuous([e for e in d.log if e[0] > stop])
  else:
    d.go(100, enabled=True, request=1, long_active=True)
    assert not active_frames(d, stop + 1)


def d_frame_ns(d, frame):
  return d.now - (d.frame - 1 - frame) * FRAME_NS
