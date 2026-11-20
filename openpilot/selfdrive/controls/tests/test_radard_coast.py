import pytest

import openpilot.cereal.messaging as messaging
from opendbc.car import structs
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.radard import RADAR_COAST_T, RADAR_TO_CAMERA, RadarD

V_EGO = 15.


class SM(dict):
  def __init__(self):
    super().__init__()
    self.seen = {'modelV2': True}
    self.recv_frame = {'carState': 0}
    self.logMonoTime = {'modelV2': 0, 'radarTracks': 0}

  def all_checks(self, *args, **kwargs):
    return True


def make_rd(brand='mazda', can_filter=True) -> RadarD:
  CP = structs.CarParams(brand=brand, radarDelay=0.1)
  CP_SP = structs.CarParamsSP(flags=int(MazdaFlagsSP.CAN_FILTER) if can_filter else 0)
  return RadarD(CP, CP_SP, CP.radarDelay)


def step(rd: RadarD, sm: SM, track=None, cam=(45., -3.), dt=DT_MDL, fresh=True):
  """One model frame, dt after the last. track: (dRel, vRel) of the lead's radar point, None for no points. cam: the
  camera lead's (dRel, vRel), None for no camera lead. fresh: a radar message arrived with this frame; otherwise
  the frame reuses the last one (its time stays)."""
  sm.logMonoTime['modelV2'] += int(round(dt * 1e9))
  if fresh:
    sm.logMonoTime['radarTracks'] = sm.logMonoTime['modelV2']
  cs = messaging.new_message('carState')
  cs.carState.vEgo = V_EGO
  sm['carState'] = cs.carState
  sm.recv_frame['carState'] += 1

  md = messaging.new_message('modelV2')
  md.modelV2.velocity.x = [V_EGO] * 33
  leads = md.modelV2.init('leadsV3', 2)
  if cam is not None:
    d, v = cam
    leads[0].prob = 0.9
    leads[0].x, leads[0].y, leads[0].v, leads[0].a = [d + RADAR_TO_CAMERA] * 6, [0.] * 6, [V_EGO + v] * 6, [0.] * 6
    leads[0].xStd, leads[0].yStd, leads[0].vStd = [3.] * 6, [0.5] * 6, [1.] * 6
  for i in range(0 if cam is None else 1, 2):
    ld = leads[i]
    ld.prob = 0.
    ld.x, ld.y, ld.v, ld.a, ld.xStd, ld.yStd, ld.vStd = ([0.] * 6 for _ in range(7))
  sm['modelV2'] = md.modelV2

  rt = messaging.new_message('radarTracks')
  if track is not None:
    pt = rt.radarTracks.init('points', 1)[0]
    pt.trackId, pt.dRel, pt.yRel, pt.vRel = 7, track[0], 0., track[1]
  rd.update(sm, rt.radarTracks)
  return rd.radar_state.leadOne


@pytest.fixture
def sm():
  return SM()


def lock_on(rd, sm, track=(40., -8.), cam=(42., -3.)):
  for _ in range(20):
    lead = step(rd, sm, track=track, cam=cam)
  assert lead.radar and lead.dRel == pytest.approx(track[0])
  return lead


class TestRadarCoast:
  def test_gap_keeps_the_radar_closing_speed(self, sm):
    rd = make_rd()
    radar = lock_on(rd, sm)
    lead = step(rd, sm, track=None, cam=(45., -3.))
    assert lead.present and not lead.radar
    # the camera's 45 m / -3 m/s would brake less than the radar's carried 40 m / -8 m/s
    assert lead.dRel == pytest.approx(40. - 8. * DT_MDL)
    assert lead.vRel == pytest.approx(-8.)
    assert lead.vLead == pytest.approx(radar.vLead)
    assert lead.aLeadK <= radar.aLeadK + 1e-6

  def test_never_less_cautious_than_the_camera(self, sm):
    rd = make_rd()
    lock_on(rd, sm, track=(40., -2.), cam=(41., -2.))
    lead = step(rd, sm, track=None, cam=(35., -6.))  # a nearer, faster-closing camera car wins each value
    assert lead.dRel == pytest.approx(35.)
    assert lead.vRel == pytest.approx(-6.)

  def test_ends_after_the_window(self, sm):
    rd = make_rd()
    lock_on(rd, sm)
    n = round(RADAR_COAST_T / DT_MDL)
    for _ in range(n):
      lead = step(rd, sm, track=None, cam=(45., -3.))
      assert lead.vRel == pytest.approx(-8.)
    lead = step(rd, sm, track=None, cam=(45., -3.))
    assert lead.dRel == pytest.approx(45.) and lead.vRel == pytest.approx(-3.)
    # and stays ended
    lead = step(rd, sm, track=None, cam=(45., -3.))
    assert lead.vRel == pytest.approx(-3.)

  def test_radar_back_ends_it(self, sm):
    rd = make_rd()
    lock_on(rd, sm)
    step(rd, sm, track=None, cam=(45., -3.))
    lead = step(rd, sm, track=(39., -7.), cam=(45., -3.))
    assert lead.radar and lead.dRel == pytest.approx(39.)

  def test_carried_car_far_nearer_is_dropped(self, sm):
    rd = make_rd()
    lock_on(rd, sm, track=(20., 0.), cam=(21., 0.))
    lead = step(rd, sm, track=None, cam=(60., 0.))  # the camera moved on to a car 40 m further: the old one is gone
    assert lead.dRel == pytest.approx(60.)

  def test_no_camera_lead_invents_none(self, sm):
    rd = make_rd()
    lock_on(rd, sm)
    for _ in range(3):
      lead = step(rd, sm, track=None, cam=None)
    assert not lead.present
    # nor does a coast resume when the camera finds a car again
    lead = step(rd, sm, track=None, cam=(45., -3.))
    assert lead.dRel == pytest.approx(45.) and lead.vRel == pytest.approx(-3.)

  def test_late_frame_expires_it(self, sm):
    # the window is time since the track was seen, not a count of frames: one frame 2 s late carries nothing
    rd = make_rd()
    lock_on(rd, sm)
    lead = step(rd, sm, track=None, cam=(45., -3.), dt=2.)
    assert lead.dRel == pytest.approx(45.) and lead.vRel == pytest.approx(-3.)
    # and a frame 0.3 s late inside it carries the distance over the time that passed
    rd = make_rd()
    lock_on(rd, sm)
    lead = step(rd, sm, track=None, cam=(45., -3.), dt=0.3)
    assert lead.dRel == pytest.approx(40. - 8. * 0.3)

  def test_window_counts_from_the_first_missed_frame_late(self, sm):
    rd = make_rd()
    lock_on(rd, sm)
    step(rd, sm, track=None, cam=(45., -3.), dt=0.4)  # coasting, 0.4 s since seen
    lead = step(rd, sm, track=None, cam=(45., -3.), dt=0.2)  # 0.6 s since seen: over
    assert lead.vRel == pytest.approx(-3.)

  def test_repeated_radar_message_does_not_restart_it(self, sm):
    # model frames that reuse the last radar message still count from when that message was sent
    rd = make_rd()
    lock_on(rd, sm)
    for _ in range(4):  # 0.2 s of the same, older radar reading
      lead = step(rd, sm, track=(40., -8.), cam=(42., -3.), fresh=False)
      assert lead.radar
    n = round((RADAR_COAST_T - 4 * DT_MDL) / DT_MDL)
    for _ in range(n):
      lead = step(rd, sm, track=None, cam=(45., -3.))
      assert lead.vRel == pytest.approx(-8.)
    lead = step(rd, sm, track=None, cam=(45., -3.))
    assert lead.vRel == pytest.approx(-3.)

  @pytest.mark.parametrize('brand, can_filter', [('mazda', False), ('toyota', True)])
  def test_only_behind_the_mazda_filter(self, sm, brand, can_filter):
    rd = make_rd(brand, can_filter)
    lock_on(rd, sm)
    lead = step(rd, sm, track=None, cam=(45., -3.))
    assert lead.dRel == pytest.approx(45.) and lead.vRel == pytest.approx(-3.)
