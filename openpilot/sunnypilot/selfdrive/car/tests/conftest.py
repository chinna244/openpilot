import pytest

from openpilot.sunnypilot.selfdrive.car import cruise_arbiter


@pytest.fixture(autouse=True)
def _limits_settle_at_once(monkeypatch):
  # Session tests step the resolver one frame at a time; LIMIT_SETTLE's own tests turn it back on.
  monkeypatch.setattr(cruise_arbiter, "LIMIT_SETTLE", 0.)
