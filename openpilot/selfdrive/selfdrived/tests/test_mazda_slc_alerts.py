"""Mazda SLA alerts (filter-dev ce9225e178 events_zp, mici): "Press + for\\n36 mph" / "Press - for\\n30 mph" while the
session's prompt is open, "Adjusting to\\n30 mph" for 5 s on each announce bump. Route 00000003--941020f467 first asked
for the press direction; the port raises both from slcState.mazdaSla, which plannerd mirrors from card's session."""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.selfdrived import selfdrived
from openpilot.selfdrive.selfdrived.events import MAZDA_EVENTS, MazdaEvents, ET

MPH = 0.44704
NOW = 100_000_000_000
PRE_ACTIVE, ACTIVE, INACTIVE = 2, 5, 1


def sla(**kw):
  base = {'sessionValid': True, 'assistState': PRE_ACTIVE, 'offerTarget': 30 * MPH, 'speedLimitFinalLast': 30 * MPH, 'vTarget': 255., 'announceCounter': 0}
  return SimpleNamespace(**{**base, **kw})


class SM:
  def __init__(self, state, stamp=NOW - 50_000_000, valid=True):
    self.state = SimpleNamespace(mazdaSla=state)
    self.logMonoTime = {'slcState': stamp}
    self.seen = {'slcState': True}
    self.valid = {'slcState': valid}
    self.alive = {'slcState': True}

  def __getitem__(self, name):
    return self.state


def cs(set_mph):
  return SimpleNamespace(vCruiseCluster=set_mph * 1.609344, cruiseState=SimpleNamespace(speedCluster=set_mph * MPH))


def run(state, set_mph, *, enabled=True, eligible=True, sd=None, now=NOW, **sm_kw):
  sd = sd or SimpleNamespace(mazda_slc_alerts=eligible, enabled=enabled, is_metric=False, mazda_events=MazdaEvents())
  sd.enabled = enabled
  sm_kw.setdefault('stamp', now - 50_000_000)
  sd.sm = SM(state, **sm_kw)
  sd.mazda_events.clear()
  selfdrived.SelfdriveD.add_mazda_slc_events(sd, cs(set_mph), now_ns=now)
  sd.set_mph = set_mph
  return sd


def alert(sd, name):
  return MAZDA_EVENTS[name][ET.WARNING](None, cs(sd.set_mph), sd.sm, False, 0, None)


@pytest.mark.parametrize('set_mph,offer,expected', [(35, 30, 'Press - for\n30 mph'), (30, 36, 'Press + for\n36 mph')])
def test_open_prompt_says_which_press(set_mph, offer, expected):
  sd = run(sla(offerTarget=offer * MPH), set_mph)
  assert sd.mazda_events.names == ['slcConfirmPrompt']
  a = alert(sd, 'slcConfirmPrompt')
  assert a.alert_text_1 == expected and a.duration == 10   # 0.1 s, raised every frame the prompt is open


def test_prompt_names_the_session_offer_not_a_flicker():
  """The resolver reads 48 for a moment (a camera flicker); the session still offers 30, and a press is judged against 30."""
  sd = run(sla(offerTarget=30 * MPH, speedLimitFinalLast=48 * MPH), 36)
  assert alert(sd, 'slcConfirmPrompt').alert_text_1 == 'Press - for\n30 mph'


def test_prompt_equal_to_set_is_an_empty_small_alert():
  # filter-dev on mici: between a confirm and plannerd's next tick the prompt is up with no text (<= 50 ms)
  a = alert(run(sla(offerTarget=30 * MPH), 30), 'slcConfirmPrompt')
  assert a.alert_text_1 == '' and str(a.alert_size) in ('small', '1')


def test_announce_bump_shows_adjusting_for_five_seconds():
  sd = run(sla(assistState=ACTIVE, announceCounter=3, vTarget=25 * MPH), 34)
  assert sd.mazda_events.names == []   # the first message only syncs the counter (plannerd restarts)
  sd = run(sla(assistState=ACTIVE, announceCounter=4, vTarget=25 * MPH), 34, sd=sd)
  assert sd.mazda_events.names == ['slcAdjusting']
  a = alert(sd, 'slcAdjusting')
  assert a.alert_text_1 == 'Adjusting to\n25 mph' and a.duration == 10   # 0.1 s, raised every frame while it applies
  # refreshed every frame of the 5 s window; the refreshes never move the window
  for t in (1., 3., 4.99):
    assert run(sla(assistState=ACTIVE, announceCounter=4, vTarget=25 * MPH), 34, sd=sd,
               now=NOW + int(t * 1e9)).mazda_events.names == ['slcAdjusting']
  assert run(sla(assistState=ACTIVE, announceCounter=4, vTarget=25 * MPH), 34, sd=sd,
             now=NOW + 5_000_000_000).mazda_events.names == []
  assert run(sla(assistState=ACTIVE, announceCounter=4, vTarget=25 * MPH), 34, sd=sd,
             now=NOW + 5_100_000_000).mazda_events.names == []


def bumped(state_after, *, t=1.):
  """Accept at NOW (counter 4 -> 5), then one frame t s later with the session in state_after."""
  sd = run(sla(assistState=ACTIVE, announceCounter=4, vTarget=25 * MPH), 34)
  run(sla(assistState=ACTIVE, announceCounter=5, vTarget=25 * MPH), 34, sd=sd)
  return run(sla(assistState=state_after, announceCounter=5, vTarget=25 * MPH), 34, sd=sd, now=NOW + int(t * 1e9))


def test_road_change_clears_adjusting_at_once():
  """Route 00000010 t=137.44: the road change dropped the session to inactive; "Adjusting to 25" stayed up to 141.05."""
  sd = bumped(INACTIVE)
  assert sd.mazda_events.names == []
  # and it does not come back when the session returns inside the old window
  assert run(sla(assistState=ACTIVE, announceCounter=5, vTarget=30 * MPH), 34, sd=sd,
             now=NOW + 2_000_000_000).mazda_events.names == []


def test_new_offer_replaces_adjusting_with_the_prompt():
  """Route 00000010 t=137.14: a new offer opened 1.2 s into "Adjusting to 25"."""
  assert bumped(PRE_ACTIVE).mazda_events.names == ['slcConfirmPrompt']


@pytest.mark.parametrize('kw', [{'enabled': False}, {'sessionValid': False}])
def test_disengage_or_lost_session_clears_adjusting(kw):
  sd = run(sla(assistState=ACTIVE, announceCounter=4, vTarget=25 * MPH), 34)
  run(sla(assistState=ACTIVE, announceCounter=5, vTarget=25 * MPH), 34, sd=sd)
  state = sla(assistState=ACTIVE, announceCounter=5, vTarget=25 * MPH, sessionValid=kw.get('sessionValid', True))
  assert run(state, 34, sd=sd, enabled=kw.get('enabled', True), now=NOW + 1_000_000_000).mazda_events.names == []
  assert run(sla(assistState=ACTIVE, announceCounter=5, vTarget=25 * MPH), 34, sd=sd,
             now=NOW + 1_100_000_000).mazda_events.names == []


def test_offer_and_acceptance_chime_once_each_and_the_banner_clears():
  """Route 00000010 t=133-141 end to end through AlertManager and soundd: one chime for the offer, one for the
  acceptance, none for the banner refreshes, and the banner gone within 0.1 s of the road change."""
  from unittest.mock import Mock
  import numpy as np
  from openpilot.cereal import log, messaging
  from openpilot.selfdrive.selfdrived.alertmanager import AlertManager
  from openpilot.selfdrive.ui.soundd import Soundd, sound_list
  AudibleAlert = log.SelfdriveState.AudibleAlert

  s = Soundd.__new__(Soundd)
  s.current_alert = s.current_sound = AudibleAlert.none
  s.current_sound_frame = 0
  s.current_volume = s.ramp_start_volume = .1
  s.ramp_start_time = 0.
  s.pending_stop = False
  s.selfdrive_timeout_alert = False
  s.startup_safety_time = None
  s.hold_applies = True
  s.loaded_sounds = {key: np.zeros(20, dtype=np.float32) for key in sound_list}  # 20 frames per chime
  s.axis_alerts = Mock(update=Mock(side_effect=lambda sm, alert, now_ns: alert))

  class FakeSM(dict):
    updated = {'selfdriveState': True}
    seen = {'pandaStates': False}

  def state_at(t):  # offer at 0 s, accepted at 2 s, road change at 3.2 s
    if t < 2.:
      return sla(assistState=PRE_ACTIVE, announceCounter=4, offerTarget=25 * MPH)
    if t < 3.2:
      return sla(assistState=ACTIVE, announceCounter=5, vTarget=25 * MPH)
    return sla(assistState=INACTIVE, announceCounter=5, vTarget=255.)

  am, sd, chimes, shown = AlertManager(), None, 0, []
  for frame in range(400):
    t = frame / 100.
    sd = run(state_at(t), 35, sd=sd, now=NOW + int(t * 1e9))
    am.add_many(frame, sd.mazda_events.create_alerts([ET.WARNING], [None, cs(35), sd.sm, False, 0, None]))
    am.process_alerts(frame, set())
    ss = messaging.new_message('selfdriveState')
    ss.selfdriveState.alertSound = am.current_alert.audible_alert
    ss.selfdriveState.alertType = am.current_alert.alert_type
    before = (s.current_alert, s.current_sound_frame)
    s.get_audible_alert(FakeSM(selfdriveState=ss.selfdriveState, pandaStates=[]))
    if s.current_alert == AudibleAlert.prompt and s.current_sound_frame == 0 and before != (AudibleAlert.prompt, 0):
      chimes += 1
    s.current_sound_frame += 1   # playback advances one frame per step
    shown.append(am.current_alert.alert_type.split('/')[0])
  assert chimes == 2
  assert shown[:200] == ['slcConfirmPrompt'] * 200
  assert set(shown[200:320]) == {'slcAdjusting'}
  assert all(a == '' for a in shown[331:])   # cleared within 0.1 s of the road change at 3.2 s


def test_adjusting_names_the_cap_or_the_set_speed_below_it():
  sd = run(sla(assistState=ACTIVE, vTarget=42 * MPH), 35)
  assert alert(sd, 'slcAdjusting').alert_text_1 == 'Adjusting to\n35 mph'


def test_prompt_shows_when_starpilot_slc_marks_the_message_invalid():
  # slcState.valid is StarPilot SLC's verdict on its own fields; card runs the session from mazdaSla regardless
  assert run(sla(), 35, valid=False).mazda_events.names == ['slcConfirmPrompt']


@pytest.mark.parametrize('kw', [{'enabled': False}, {'eligible': False}, {'stamp': NOW - 2_000_000_000},
                                {'stamp': NOW + 1}])
def test_no_alert_when_disengaged_ineligible_or_stale(kw):
  assert run(sla(), 35, **kw).mazda_events.names == []


@pytest.mark.parametrize('state', [INACTIVE, ACTIVE, 0])
def test_no_prompt_outside_preactive(state):
  assert run(sla(assistState=state), 35).mazda_events.names == []


def test_prompt_stays_through_a_source_gap():
  """Every source dropping out leaves the session's prompt and its offer as they were (route 00000004--0ed7a0ac2e t=131-132.5)."""
  sd = run(sla(speedLimitFinalLast=0.), 36)
  assert sd.mazda_events.names == ['slcConfirmPrompt']
  assert alert(sd, 'slcConfirmPrompt').alert_text_1 == 'Press - for\n30 mph'
