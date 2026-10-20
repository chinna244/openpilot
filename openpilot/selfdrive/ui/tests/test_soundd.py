import threading
import time

from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal import log, messaging
from openpilot.cereal.messaging import SubMaster, PubMaster
from openpilot.selfdrive.ui.soundd import HOLD_EXEMPT, SAFETY_SWITCH_HOLD, SELFDRIVE_STATE_TIMEOUT, Soundd, check_selfdrive_timeout_alert

AudibleAlert = log.SelfdriveState.AudibleAlert


class TestSoundd(OpenpilotTestCase):
  def test_check_selfdrive_timeout_alert(self, mocker):
    sm = SubMaster(['selfdriveState', 'selfdriveStateSP'])
    pm = PubMaster(['selfdriveState', 'selfdriveStateSP'])

    cs = messaging.new_message('selfdriveState')
    cs.selfdriveState.enabled = True
    threading.Timer(0.01, pm.send, args=("selfdriveState", cs)).start()
    sm.update(100)
    assert sm.updated['selfdriveState']

    sm.recv_time['selfdriveState'] = 0
    clock = mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=SELFDRIVE_STATE_TIMEOUT)
    assert not check_selfdrive_timeout_alert(sm)

    clock.return_value = SELFDRIVE_STATE_TIMEOUT + 0.1
    assert check_selfdrive_timeout_alert(sm)

    clock.return_value = SELFDRIVE_STATE_TIMEOUT + 10
    assert not check_selfdrive_timeout_alert(sm)

  def test_check_selfdrive_timeout_alert_mads_lateral_only(self):
    sm = SubMaster(['selfdriveState', 'selfdriveStateSP'])
    pm = PubMaster(['selfdriveState', 'selfdriveStateSP'])

    for _ in range(100):
      cs = messaging.new_message('selfdriveState')
      cs.selfdriveState.enabled = False

      ss_sp = messaging.new_message('selfdriveStateSP')
      ss_sp.selfdriveStateSP.mads.enabled = True

      pm.send("selfdriveState", cs)
      pm.send("selfdriveStateSP", ss_sp)

      time.sleep(0.01)

      sm.update(0)

      assert not check_selfdrive_timeout_alert(sm)

    for _ in range(SELFDRIVE_STATE_TIMEOUT * 110):
      sm.update(0)
      time.sleep(0.01)

    assert check_selfdrive_timeout_alert(sm)

  @staticmethod
  def _step(soundd, alert, safety_model=None):
    """One soundd step: selfdriveState asks for alert; one panda in safety_model (None: no pandaStates yet)."""
    ss = messaging.new_message('selfdriveState')
    ss.selfdriveState.alertSound = alert
    ps = messaging.new_message('pandaStates', 1)
    if safety_model is not None:
      ps.pandaStates[0].safetyModel = safety_model

    class FakeSM(dict):
      updated = {'selfdriveState': True}
      seen = {'pandaStates': safety_model is not None}

    soundd.get_audible_alert(FakeSM(selfdriveState=ss.selfdriveState, pandaStates=ps.pandaStates))

  def test_sound_waits_out_the_safety_switch(self, mocker):
    clock = mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=100.)
    s = Soundd()
    # still in the startup safety mode: the prompt waits
    self._step(s, AudibleAlert.prompt, 'elm327')
    assert s.current_alert == AudibleAlert.none
    # switched to the car's mode, inside the settle window: still waits
    clock.return_value = 100. + SAFETY_SWITCH_HOLD - 0.1
    self._step(s, AudibleAlert.prompt, 'mazda')
    assert s.current_alert == AudibleAlert.none
    # settled: the prompt plays
    clock.return_value = 100. + SAFETY_SWITCH_HOLD + 0.1
    self._step(s, AudibleAlert.prompt, 'mazda')
    assert s.current_alert == AudibleAlert.prompt

  def test_warnings_never_wait(self, mocker):
    mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=100.)
    for alert in HOLD_EXEMPT:
      s = Soundd()
      self._step(s, alert, 'elm327')
      assert s.current_alert == alert

  def test_no_hold_without_panda_states(self, mocker):
    mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=100.)
    s = Soundd()
    self._step(s, AudibleAlert.prompt)
    assert s.current_alert == AudibleAlert.prompt

  def test_no_hold_once_driving(self, mocker):
    mocker.patch("openpilot.selfdrive.ui.soundd.time.monotonic", return_value=100.)
    s = Soundd()
    self._step(s, AudibleAlert.engage, 'mazda')
    assert s.current_alert == AudibleAlert.engage

  # TODO: add test with micd for checking that soundd actually outputs sounds
