"""develop b3ca0c1c58: a new sound waits out the panda's switch out of the startup safety mode; warnings never wait."""
from unittest.mock import Mock

import numpy as np

from openpilot.cereal import log, messaging
from openpilot.selfdrive.ui.soundd import HOLD_EXEMPT, SAFETY_SWITCH_HOLD, Soundd, sound_list

AudibleAlert = log.SelfdriveState.AudibleAlert


def soundd(mocker):
  # as test_soundd_critical_merge: no audio assets on the host
  s = Soundd.__new__(Soundd)
  s.current_alert = s.current_sound = AudibleAlert.none
  s.current_sound_frame = 0
  s.current_volume = s.ramp_start_volume = .1
  s.ramp_start_time = 0.
  s.pending_stop = False
  s.selfdrive_timeout_alert = False
  s.startup_safety_time = None
  s.loaded_sounds = {key: np.array([.2, .4, .2, 0.], dtype=np.float32) for key in sound_list}
  # the AOL axis sounds are their own feature; pass the selfdrived request through
  s.axis_alerts = Mock(update=Mock(side_effect=lambda sm, alert, now_ns: alert))
  return s


def step(s, alert, safety_model=None):
  """One soundd step: selfdriveState asks for alert; one panda in safety_model (None: no pandaStates yet)."""
  ss = messaging.new_message('selfdriveState')
  ss.selfdriveState.alertSound = alert
  ps = messaging.new_message('pandaStates', 1)
  if safety_model is not None:
    ps.pandaStates[0].safetyModel = safety_model

  class FakeSM(dict):
    updated = {'selfdriveState': True}
    seen = {'pandaStates': safety_model is not None}

  s.get_audible_alert(FakeSM(selfdriveState=ss.selfdriveState, pandaStates=ps.pandaStates))


def test_sound_waits_out_the_safety_switch(mocker):
  clock = mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=100.)
  s = soundd(mocker)
  # still in the startup safety mode: the prompt waits
  step(s, AudibleAlert.prompt, 'elm327')
  assert s.current_alert == AudibleAlert.none
  # switched to the car's mode, inside the settle window: still waits
  clock.return_value = 100. + SAFETY_SWITCH_HOLD - 0.1
  step(s, AudibleAlert.prompt, 'mazda')
  assert s.current_alert == AudibleAlert.none
  # settled: the prompt plays
  clock.return_value = 100. + SAFETY_SWITCH_HOLD + 0.1
  step(s, AudibleAlert.prompt, 'mazda')
  assert s.current_alert == AudibleAlert.prompt


def test_warnings_never_wait(mocker):
  mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=100.)
  for alert in HOLD_EXEMPT:
    s = soundd(mocker)
    step(s, alert, 'elm327')
    assert s.current_alert == alert


def test_no_hold_without_panda_states(mocker):
  mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=100.)
  s = soundd(mocker)
  step(s, AudibleAlert.prompt)
  assert s.current_alert == AudibleAlert.prompt


def test_no_hold_once_driving(mocker):
  mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=100.)
  s = soundd(mocker)
  step(s, AudibleAlert.engage, 'mazda')
  assert s.current_alert == AudibleAlert.engage
