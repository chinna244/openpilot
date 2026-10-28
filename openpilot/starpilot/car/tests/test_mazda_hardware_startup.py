import queue
import threading


def test_hardware_daemon_initializes_with_its_real_params_owner(tmp_path, monkeypatch):
  from openpilot.system.hardware import hardwared
  from openpilot.common.prefix import OpenpilotPrefix
  monkeypatch.setattr(hardwared, 'starpilot_storage_root', lambda: tmp_path)
  stopped = threading.Event()
  stopped.set()
  with OpenpilotPrefix():
    hardwared.hardware_thread(stopped, queue.Queue())
