"""Review 2026-11-23: on Mazda factory cruise the press toward the offer accepts (card.mazda_sla_answer), so the
prompt's wheel hint must show - ACCEPT / + REJECT for an offer below SET, and + ACCEPT / - REJECT above it."""
import inspect
from types import SimpleNamespace

import pytest

from openpilot.starpilot.ui import speed_limit_widget
from openpilot.starpilot.ui.onroad_state import OnroadState
from openpilot.starpilot.ui.runtime_snapshot import slc_accept_key_plus

MPH = 0.44704


def cp(brand='mazda', pcm=True):
  return SimpleNamespace(brand=brand, pcmCruise=pcm, passive=False, dashcamOnly=False)


def slc(offer_mph):
  return SimpleNamespace(hasPresentationAdjustedLimit=True, presentationAdjustedLimit=offer_mph * MPH)


def cruise(set_mph):
  return SimpleNamespace(speedCluster=set_mph * MPH)


@pytest.mark.parametrize('offer,set_speed,plus', [(30, 35, False), (36, 30, True), (30, 30, True)])
def test_mazda_hint_follows_the_card_direction(offer, set_speed, plus):
  assert slc_accept_key_plus(cp(), slc(offer), cruise(set_speed), False) is plus


@pytest.mark.parametrize('car', [cp('toyota'), cp('mazda', pcm=False), None])
def test_other_cars_keep_plus_accept(car):
  assert slc_accept_key_plus(car, slc(30), cruise(35), False) is True


def test_missing_offer_keeps_plus_accept():
  assert slc_accept_key_plus(cp(), SimpleNamespace(hasPresentationAdjustedLimit=False), cruise(35), False) is True
  assert OnroadState.__dataclass_fields__['slc_accept_key_plus'].default is True


def test_widget_draws_the_accept_key_from_state():
  source = inspect.getsource(speed_limit_widget.SpeedLimitWidget.render_confirmation)
  assert "for left, plus in ((start, accept_plus), (start + 18 + 4 + accept + 10, not accept_plus))" in source
  assert "accept_plus = state.slc_accept_key_plus" in source
