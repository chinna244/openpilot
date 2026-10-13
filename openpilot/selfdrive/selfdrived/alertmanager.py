import copy
import os
import json
from collections import defaultdict
from dataclasses import dataclass

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.selfdrive.selfdrived.events import Alert
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import AudibleAlert, EmptyAlert


with open(os.path.join(BASEDIR, "openpilot/selfdrive/selfdrived/alerts_offroad.json")) as f:
  OFFROAD_ALERTS = json.load(f)


def set_offroad_alert(alert: str, show_alert: bool, extra_text: str | None = None) -> None:
  if show_alert:
    a = copy.copy(OFFROAD_ALERTS[alert])
    a['extra'] = extra_text or ''
    Params().put(alert, a, block=True)
  else:
    Params().remove(alert)


@dataclass
class AlertEntry:
  alert: Alert | None = None
  start_frame: int = -1
  end_frame: int = -1
  added_frame: int = -1
  interrupted: bool = False  # another alert was shown over it while it was still active

  def active(self, frame: int) -> bool:
    return frame <= self.end_frame

  def just_added(self, frame: int) -> bool:
    return self.active(frame) and frame == (self.added_frame + 1)

class AlertManager:
  def __init__(self):
    self.alerts: dict[str, AlertEntry] = defaultdict(AlertEntry)
    self.current_alert = EmptyAlert
    self.current_entry: AlertEntry | None = None

  def add_many(self, frame: int, alerts: list[Alert]) -> None:
    for alert in alerts:
      entry = self.alerts[alert.alert_type]
      entry.alert = alert
      if not entry.just_added(frame):
        entry.start_frame = frame
        entry.interrupted = False
      min_end_frame = entry.start_frame + alert.duration
      entry.end_frame = max(frame + 1, min_end_frame)
      entry.added_frame = frame

  def process_alerts(self, frame: int, clear_event_types: set):
    ae = AlertEntry()
    for v in self.alerts.values():
      if not v.alert:
        continue

      if v.alert.event_type in clear_event_types:
        v.end_frame = -1

      # sort by priority first and then by start_frame
      greater = ae.alert is None or (v.alert.priority, v.start_frame) > (ae.alert.priority, ae.start_frame)
      if v.active(frame) and greater:
        ae = v

    prev = self.current_entry
    if prev is not None and prev is not ae and prev.alert is not None and prev.active(frame):
      prev.interrupted = True
    self.current_entry = ae
    alert = ae.alert
    # An alert marked sound_once comes back silent after another was shown over it: its chime
    # has played, and a second one reads as a second event.
    if alert is not None and ae.interrupted and getattr(alert, "sound_once", False):
      alert = copy.copy(alert)
      alert.audible_alert = AudibleAlert.none
    self.current_alert = alert if alert is not None else EmptyAlert
