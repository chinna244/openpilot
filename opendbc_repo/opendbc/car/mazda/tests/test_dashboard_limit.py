"""The SLA dashboard (car) source: camera sign first, navigation unit second, each only while its CAN message is fresh."""
import pytest
from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.dashboard_speed_limit import Status
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.carstate_ext import CarStateExt
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, DBC

MS = 1_000_000
PERIOD = 100 * MS  # both signs run at 10 Hz


class Rig:
  def __init__(self):
    cp = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], False, False, False)
    self.parsers = CarState.get_can_parsers(cp)
    self.packer = CANPacker(DBC[CAR.MAZDA_CX5_2022][Bus.pt])
    self.ext = CarStateExt(cp, None)
    self.t = 1_000 * MS

  def step(self, cam=None, nav=None):
    """One 10 ms CAN batch; cam/nav = (sign, unit) to send that message in this batch."""
    self.t += 10 * MS
    cam_frames = [self.packer.make_can_msg('CAM_TRAFFIC_SIGNS', 2, {'SPEED_SIGN': cam[0], 'SPEED_SIGN_UNIT': cam[1]})] if cam else []
    pt_frames = [self.packer.make_can_msg('NAV_SPEED_LIMIT', 0, {'SPEED_SIGN': nav[0], 'SPEED_SIGN_UNIT': nav[1]})] if nav else []
    self.parsers[Bus.cam].update([(self.t, cam_frames)])
    self.parsers[Bus.pt].update([(self.t, pt_frames)])
    self.ext.update_dashboard_limit(self.parsers)
    return self.ext.dashboard_limit.observation

  def run(self, seconds, cam=None, nav=None):
    obs = None
    for i in range(int(seconds * 100)):
      send = i % (PERIOD // (10 * MS)) == 0
      obs = self.step(cam if send else None, nav if send else None)
    return obs


def mph(obs):
  return round(obs.speed_mps * CV.MS_TO_MPH)


def test_unknown_until_cadence_learned_then_camera_first():
  r = Rig()
  assert r.run(0.5, cam=(30, 1), nav=(35, 1)).status is Status.UNKNOWN
  obs = r.run(1.5, cam=(30, 1), nav=(35, 1))
  assert obs.status is Status.VALID and mph(obs) == 30 and obs.valid_until_ns > r.t


def test_navigation_fills_in_when_camera_shows_no_sign():
  r = Rig()
  obs = r.run(2., cam=(0, 0), nav=(35, 1))
  assert obs.status is Status.VALID and mph(obs) == 35


@pytest.mark.parametrize('cam', [(5, 1), (127, 2), (95, 1)])
def test_implausible_camera_reading_gives_way_to_navigation(cam):
  r = Rig()
  assert mph(r.run(2., cam=cam, nav=(40, 1))) == 40


def test_both_fresh_without_a_sign_is_absent():
  r = Rig()
  obs = r.run(2., cam=(0, 0), nav=(0, 0))
  assert obs.status is Status.ABSENT and obs.speed_mps == 0.


def test_stale_camera_sign_is_not_reused():
  r = Rig()
  assert mph(r.run(2., cam=(30, 1), nav=(45, 1))) == 30
  obs = r.run(1.5, nav=(45, 1))  # camera went silent: its last 30 times out (10 periods)
  assert obs.status is Status.VALID and mph(obs) == 45


def test_stale_camera_alone_becomes_unknown():
  r = Rig()
  assert mph(r.run(2., cam=(30, 1))) == 30
  assert r.run(0.5).status is Status.VALID  # within the parser's timeout
  obs = r.run(1.)
  assert obs.status is Status.UNKNOWN and obs.speed_mps == 0. and obs.valid_until_ns == 0


def test_km_h_sign():
  r = Rig()
  assert r.run(2., cam=(50, 2)).speed_mps == pytest.approx(50 * CV.KPH_TO_MS)
