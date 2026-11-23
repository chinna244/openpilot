"""Route 00000003--941020f467: SLA confirmed with one press but nothing said which press. develop/filter-dev
(events_zp) show "Press + for\\n45 mph" / "Press - for\\n45 mph" while the offer is open and "Adjusting to" after."""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.selfdrived import selfdrived
from openpilot.selfdrive.selfdrived.events import MAZDA_EVENTS, MazdaEvents, ET

MPH = 0.44704
NOW = 100_000_000_000


def slc(**kw):
  base = dict(enabled=True, displayOnly=False, hasPending=True, automaticAdoptionPending=False,
              hasPresentationAdjustedLimit=True, presentationAdjustedLimit=30 * MPH, hasAcceptedAdjustedLimit=False,
              acceptedAdjustedLimit=0., acceptedIsPreview=False, acceptedMonoTime=0)
  return SimpleNamespace(**{**base, **kw})


class SM:
  def __init__(self, state, stamp=NOW - 50_000_000, valid=True):
    self.state = state
    self.logMonoTime = {'slcState': stamp}
    self.seen = {'slcState': True}
    self.valid = {'slcState': valid}
    self.alive = {'slcState': True}

  def __getitem__(self, name):
    return self.state


def cs(set_mph):
  return SimpleNamespace(cruiseState=SimpleNamespace(speedCluster=set_mph * MPH))


def run(state, set_mph, *, enabled=True, eligible=True, **sm_kw):
  sd = SimpleNamespace(mazda_slc_alerts=eligible, enabled=enabled, is_metric=False, mazda_events=MazdaEvents(),
                       sm=SM(state, **sm_kw))
  selfdrived.SelfdriveD.add_mazda_slc_events(sd, cs(set_mph), now_ns=NOW)
  return sd


def text(sd, name):
  alert = MAZDA_EVENTS[name][ET.WARNING](None, cs(sd.set_mph), sd.sm, False, 0, None)
  return alert.alert_text_1


@pytest.mark.parametrize('set_mph,offer,expected', [(35, 30, 'Press - for\n30 mph'), (30, 36, 'Press + for\n36 mph')])
def test_open_offer_says_which_press(set_mph, offer, expected):
  sd = run(slc(presentationAdjustedLimit=offer * MPH), set_mph)
  assert sd.mazda_events.names == ['slcConfirmPrompt']
  sd.set_mph = set_mph
  assert text(sd, 'slcConfirmPrompt') == expected


def test_offer_equal_to_set_needs_no_prompt():
  assert run(slc(presentationAdjustedLimit=30 * MPH), 30).mazda_events.names == []


def test_accept_shows_adjusting_for_three_seconds():
  state = slc(hasPending=False, hasAcceptedAdjustedLimit=True, acceptedAdjustedLimit=25 * MPH, acceptedMonoTime=NOW - 1_000_000_000)
  sd = run(state, 34)
  assert sd.mazda_events.names == ['slcAdjusting']
  sd.set_mph = 34
  assert text(sd, 'slcAdjusting') == 'Adjusting to\n25 mph'
  state.acceptedMonoTime = NOW - 3_100_000_000
  assert run(state, 34).mazda_events.names == []


def test_preview_accept_is_silent():
  state = slc(hasPending=False, hasAcceptedAdjustedLimit=True, acceptedAdjustedLimit=25 * MPH,
              acceptedMonoTime=NOW - 1_000_000_000, acceptedIsPreview=True)
  assert run(state, 34).mazda_events.names == []


@pytest.mark.parametrize('kw', [dict(enabled=False), dict(eligible=False), dict(stamp=NOW - 2_000_000_000), dict(valid=False),
                                dict(stamp=NOW + 1)])
def test_no_alert_when_disengaged_ineligible_or_stale(kw):
  assert run(slc(), 35, **kw).mazda_events.names == []


@pytest.mark.parametrize('state_kw', [dict(enabled=False), dict(displayOnly=True), dict(automaticAdoptionPending=True),
                                      dict(hasPresentationAdjustedLimit=False)])
def test_no_prompt_without_a_confirmable_offer(state_kw):
  assert run(slc(**state_kw), 35).mazda_events.names == []
