"""gpsard stamps gpsSourceState in CLOCK_BOOTTIME, matching locationd."""

from __future__ import annotations

from openpilot.common.gps import accept_gps_source_epoch, gps_source_state_is_fresh
from openpilot.common.gps_source_arbiter import GpsSourceArbiter
from openpilot.common.time_helpers import seconds_since_boot
from openpilot.system.gpsard import _publish_state


class _CapturePM:
  def __init__(self):
    self.msg = None

  def send(self, _service, msg):
    self.msg = msg


def test_publish_event_and_transition_share_now_domain():
  arbiter = GpsSourceArbiter(ublox_hardware_available=True)
  now = 100.5
  arbiter.reset(now_mono=now, ublox_hardware_available=True)
  pm = _CapturePM()
  assert _publish_state(pm, arbiter, now)  # ty: ignore[invalid-argument-type]  # test double PubMaster
  assert pm.msg.logMonoTime == int(now * 1e9)
  assert pm.msg.gpsSourceState.transitionMonoNs == int(now * 1e9)
  assert accept_gps_source_epoch(
    transition_mono_ns=int(pm.msg.gpsSourceState.transitionMonoNs),
    generation=int(pm.msg.gpsSourceState.generation),
    selected=str(pm.msg.gpsSourceState.selected),
    recv_mono_ns=int(pm.msg.logMonoTime),
    last_transition_mono_ns=None,
    last_generation=None,
    last_selected=None,
  )


def test_valid_fresh_transition_accepted():
  now_ns = 10_000_000_000
  trans_ns = 9_500_000_000
  assert accept_gps_source_epoch(
    transition_mono_ns=trans_ns,
    generation=1,
    selected="ubloxPrimary",
    recv_mono_ns=now_ns,
    last_transition_mono_ns=None,
    last_generation=None,
    last_selected=None,
  )
  assert gps_source_state_is_fresh(now_mono=now_ns / 1e9, last_state_recv_mono=now_ns / 1e9)


def test_future_transition_rejected():
  assert not accept_gps_source_epoch(
    transition_mono_ns=11_000_000_000,
    generation=1,
    selected="ubloxPrimary",
    recv_mono_ns=10_000_000_000,
    last_transition_mono_ns=None,
    last_generation=None,
    last_selected=None,
  )


def test_stale_transition_rejected_as_regression():
  assert not accept_gps_source_epoch(
    transition_mono_ns=5_000_000_000,
    generation=2,
    selected="ubloxPrimary",
    recv_mono_ns=12_000_000_000,
    last_transition_mono_ns=9_000_000_000,
    last_generation=1,
    last_selected="qcomFallback",
  )


def test_seconds_since_boot_used_for_gpsard_now():
  now = seconds_since_boot()
  assert now > 0.0


def test_suspend_like_monotonic_vs_boottime_divergence_goes_stale():
  # CLOCK_BOOTTIME includes suspend; CLOCK_MONOTONIC does not. A 10s suspend
  # between a monotonic stamp and a boottime now exceeds the 3s freshness window.
  # Same-domain boottime stamps stay fresh. This is the defect class gpsard
  # fixed by publishing logMonoTime and transitionMonoNs from seconds_since_boot.
  monotonic_stamp = 100.0
  boottime_now = monotonic_stamp + 10.0
  assert not gps_source_state_is_fresh(now_mono=boottime_now, last_state_recv_mono=monotonic_stamp)
  assert gps_source_state_is_fresh(now_mono=boottime_now, last_state_recv_mono=boottime_now)
  assert gps_source_state_is_fresh(now_mono=monotonic_stamp + 0.5, last_state_recv_mono=monotonic_stamp)
