"""
TJA with MAIN off under the Smart CAN Filter: the white wheel stays on the dash while MADS steers.

Route 00000001--f5fdd9a482 (70.1 s, 226.5 s, 291.1 s) and develop route 0000019c (423-426 s): a TJA
press with MAIN off arms the car's MRCC standby (PEDALS ACC_OFF) for ~0.45 s, then it drops. The
cruise frame's white bit (CRZ_CTRL byte 4 0x20) rode that availability and fell with it, while the
camera frame's TJA=2 waited for its 0.5 s confirmation and 2 Hz cadence: ~0.65 s with no white
wheel and steering still active.
"""
import pytest

from opendbc.car.mazda.tests.conftest import CAM_LANEINFO, CRZ_CTRL, step
from opendbc.car.mazda.tests.test_mazda_can_filter import CONFIRMED, confirm, filter_controller
try:
  from opendbc.car.mazda.values_ext import MazdaFlagsSP
except ImportError:  # the filter-dev tree
  from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP
from opendbc.car.mazda.tests.conftest import mazda_car_state

# The camera's own CAM_LANEINFO through the sequence (route 00000001, 70.0-71.4 s).
CAM_IDLE = bytes.fromhex("4221000000004040")
CAM_STOCK_TJA = bytes.fromhex("4221000020004040")  # the camera's own TJA=2 while the stock TJA came on
CAM_TRANSITION = bytes.fromhex("4221000c00004040")  # TJA_TRANSITION after our camera press-off

PRESS = 17     # TJA held 70.179-70.351 s
ARMED = 55     # PEDALS armed until 70.642 s, public availability falls 70.729 s
TOTAL = 250


def sequence(i):
  """(frame inputs) for frame i after the press; MADS steers throughout."""
  kw = dict(enabled=False, long_active=False, lat_active=True, mads_active=True, cam_laneinfo_live=True, v_ego=20.)
  if i < PRESS:
    kw.update(tja_button=1, available=True, mrcc_armed_raw=True, cam_laneinfo_raw=CAM_STOCK_TJA)
  elif i < ARMED:
    kw.update(available=True, mrcc_armed_raw=i < ARMED - 9, cam_laneinfo_raw=CAM_IDLE)
  else:
    kw.update(available=False, mrcc_armed_raw=False, cam_laneinfo_raw=CAM_TRANSITION)
  return kw


def drive(cc, cs, frames, kw_for):
  """Every bus-0 CRZ_CTRL white bit and CAM_LANEINFO TJA=2, by frame index."""
  crz, hud = [], []
  for i in range(frames):
    cs.can_filter_state, cs.can_filter_age = CONFIRMED, 0
    cc.can_filter.receive([CONFIRMED], 0)
    _, sends = step(cc, cs, **kw_for(i))
    for a, d, b in sends:
      if b != 0:
        continue
      if a == CRZ_CTRL:
        crz.append((i, bool(d[4] & 0x20)))
      elif a == CAM_LANEINFO:
        hud.append((i, (d[4] & 0x30) >> 4 == 2))
  return crz, hud


@pytest.fixture
def tcs():
  cc = filter_controller()
  cc.CP_SP.flags |= MazdaFlagsSP.TJA_BUTTON
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  confirm(cc, cs)
  # MAIN off, not steering, camera idle: no white anywhere before the press
  crz, hud = drive(cc, cs, 120, lambda i: dict(lat_active=False, cam_laneinfo_raw=CAM_IDLE, cam_laneinfo_live=True, v_ego=20.))
  assert not any(w for _, w in crz) and not any(w for _, w in hud)
  return cc, cs


def test_white_wheel_is_continuous_from_the_press(tcs):
  cc, cs = tcs
  crz, hud = drive(cc, cs, TOTAL, sequence)
  first = next(i for i, w in crz if w)
  assert first <= PRESS, "the cruise frame's white bit comes with the press"
  gaps = [i for i, w in crz if i > first and not w]
  assert not gaps, f"white bit dropped at frames {gaps[0]}-{gaps[-1]} while MADS steered"
  assert any(w for _, w in hud), "the camera frame's TJA=2 still follows"
  assert crz[-1][1] and cc.mads_white_hud_on_bus


def test_withdrawn_when_tja_steering_disengages(tcs):
  cc, cs = tcs
  drive(cc, cs, TOTAL, sequence)
  assert cc.mads_white_hud_on_bus
  off = dict(enabled=False, long_active=False, lat_active=False, mads_active=False, available=False,
             cam_laneinfo_raw=CAM_TRANSITION, cam_laneinfo_live=True, v_ego=20.)
  crz, hud = drive(cc, cs, 6, lambda i: off)
  assert not any(w for i, w in crz if i >= 2), crz
  assert hud and not any(w for _, w in hud) and not cc.mads_white_hud_on_bus


def test_no_white_bit_while_a_button_is_held_with_cruise_off(tcs):
  # the added path is the camera frame's own gate: a held button (here SET+) blocks it
  cc, cs = tcs
  kw = dict(enabled=False, long_active=False, lat_active=True, mads_active=True, available=False,
            cam_laneinfo_raw=CAM_IDLE, cam_laneinfo_live=True, accel_button=1, v_ego=20.)
  crz, hud = drive(cc, cs, 120, lambda i: kw)
  assert not any(w for _, w in crz) and not any(w for _, w in hud)


def test_set_into_alpha_long_keeps_white(tcs):
  cc, cs = tcs
  drive(cc, cs, TOTAL, sequence)
  # MAIN on, SET pressed (button activity), then engaged and cruising
  def seq(i):
    kw = dict(lat_active=True, mads_active=True, available=True, mrcc_armed_raw=True,
              cam_laneinfo_raw=CAM_IDLE, cam_laneinfo_live=True, v_ego=20.)
    if i < 20:
      kw.update(enabled=False, long_active=False, decel_button=int(i >= 10))
    else:
      kw.update(enabled=True, long_active=True, cruise_engaged=True)
    return kw
  crz, _ = drive(cc, cs, 200, seq)
  assert all(w for _, w in crz), [i for i, w in crz if not w]
