"""Mazda SLA: the offer prompt and "Adjusting to" share the prompt sound, so switching from one to the other is a
new event that replays the chime (routes 00000010/00000011: every acceptance was silent). Repeats stay silent, the
startup safety hold still wins, and no other alert changes behaviour."""
from unittest.mock import Mock

import numpy as np

from openpilot.cereal import log, messaging
from openpilot.selfdrive.ui.soundd import Soundd, sound_list

AudibleAlert = log.SelfdriveState.AudibleAlert
PROMPT, ADJUST = 'slcConfirmPrompt/warning', 'slcAdjusting/warning'


def soundd(hold=False, axis=None):
  # as test_soundd_safety_hold: no audio assets on the host
  s = Soundd.__new__(Soundd)
  s.current_alert = s.current_sound = AudibleAlert.none
  s.current_sound_frame = 0
  s.current_volume = s.ramp_start_volume = .1
  s.ramp_start_time = 0.
  s.pending_stop = False
  s.selfdrive_timeout_alert = False
  s.startup_safety_time = None
  s.hold_applies = True
  s.loaded_sounds = {key: np.array([.2, .4, .2, 0.], dtype=np.float32) for key in sound_list}
  s.safety_switch_hold = Mock(return_value=hold)
  s.axis_alerts = Mock(update=Mock(side_effect=axis or (lambda sm, alert, now_ns: alert)))
  return s


def step(s, sound, alert_type):
  ss = messaging.new_message('selfdriveState')
  ss.selfdriveState.alertSound = sound
  ss.selfdriveState.alertType = alert_type

  class FakeSM(dict):
    updated = {'selfdriveState': True}
    seen = {'pandaStates': False}

  s.get_audible_alert(FakeSM(selfdriveState=ss.selfdriveState, pandaStates=[]))


def played_out(s):
  s.current_sound_frame = len(s.loaded_sounds[s.current_sound])


def test_acceptance_replays_the_chime_after_the_prompt():
  s = soundd()
  step(s, AudibleAlert.prompt, PROMPT)
  assert (s.current_alert, s.current_sound_frame) == (AudibleAlert.prompt, 0)
  played_out(s)
  step(s, AudibleAlert.prompt, ADJUST)
  assert (s.current_alert, s.current_sound, s.current_sound_frame) == (AudibleAlert.prompt, AudibleAlert.prompt, 0)


def test_new_offer_over_adjusting_replays_the_chime():
  s = soundd()
  step(s, AudibleAlert.prompt, ADJUST)
  played_out(s)
  step(s, AudibleAlert.prompt, PROMPT)
  assert s.current_sound_frame == 0


def test_the_same_alert_refreshed_never_replays():
  s = soundd()
  step(s, AudibleAlert.prompt, ADJUST)
  played_out(s)
  for _ in range(5):
    step(s, AudibleAlert.prompt, ADJUST)
  assert s.current_sound_frame == len(s.loaded_sounds[AudibleAlert.prompt])


def test_no_replay_in_the_startup_safety_hold():
  s = soundd()
  step(s, AudibleAlert.prompt, PROMPT)
  played_out(s)
  s.safety_switch_hold.return_value = True
  step(s, AudibleAlert.prompt, ADJUST)
  assert s.current_sound_frame == len(s.loaded_sounds[AudibleAlert.prompt])
  # the hold over, the alert already showing does not replay late
  s.safety_switch_hold.return_value = False
  step(s, AudibleAlert.prompt, ADJUST)
  assert s.current_sound_frame == len(s.loaded_sounds[AudibleAlert.prompt])


def test_an_axis_sound_is_left_to_the_normal_path():
  s = soundd(axis=lambda sm, alert, now_ns: AudibleAlert.prompt)  # the AOL axis asks for its own prompt
  step(s, AudibleAlert.none, PROMPT)
  played_out(s)
  step(s, AudibleAlert.none, ADJUST)
  assert s.current_sound_frame == len(s.loaded_sounds[AudibleAlert.prompt])


def test_other_alerts_keep_their_behaviour():
  s = soundd()
  step(s, AudibleAlert.prompt, ADJUST)
  played_out(s)
  step(s, AudibleAlert.prompt, 'stockEcuFailed/permanent')
  assert s.current_sound_frame == len(s.loaded_sounds[AudibleAlert.prompt])
