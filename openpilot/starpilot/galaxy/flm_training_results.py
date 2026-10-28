"""Read-only optional-learner progress, separate from parked FLM operations."""
import time

from openpilot.starpilot.flm.training_report import read_report, report_path


class FlmTrainingResults:
  def __init__(self, path=None, *, clock=time.monotonic_ns):
    self.clock = clock
    try:
      self.path = report_path() if path is None else path
    except (OSError, ValueError):
      self.path = None

  def snapshot(self):
    return read_report(self.path, now_ns=self.clock()) if self.path is not None else {
      "version": 1, "current": None, "history": []}
