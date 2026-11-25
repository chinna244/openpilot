"""StarPilot: an open SLA offer kept across a map road change (routes 00000010 t=137.35, 00000011 t=106.01).

The camera had already offered the new zone's target when the map caught up; the road change closed the offer, and the
driver's answer became a plain SET press. The offer now stays pending only when the map gives the same posted limit, the
exact internal target and the same press direction. The road change itself never applies the target or presses SET, the
5 s window restarts once per offer, a press already held at the boundary is a plain press, and any later change of the
evidence discards the kept offer and a queued confirm."""
from types import SimpleNamespace as NS

import pytest

from openpilot.common.constants import CV
from openpilot.starpilot.car.mazda.sla import AssistState, Source, compare_cluster_target
from openpilot.starpilot.car.mazda.sla.arbiter import SessionState
from openpilot.starpilot.car.mazda.sla.session import MazdaSlaSession
from openpilot.starpilot.car.mazda.sla.tests.rig import CP, MPH, Rig, spans
from openpilot.starpilot.car.mazda.sla.tests.test_failure_modes import first, run, setpoint, tap

N = 1600


def offer_and_boundary():
  """25 mph zone, SET 36: "Press - for 30" opens; the road change comes 1 s later."""
  initial = run(Rig(card=True), 1000, lambda _: 25, dash=36)
  opened = first(initial, lambda r: r.state == AssistState.preActive)
  return opened, opened + 100


def rig_with_zone(boundary, *, source=Source.map, nudge=0., second_change=None):
  """A Rig whose resolver reports a road change from `boundary` (a second one from `second_change`), with the given source
  and the internal target nudged by `nudge` m/s at and after the boundary."""
  rig = Rig(card=True)
  fill = rig.planner.fill

  def fill_with_zone(out):
    fill(out)
    if rig.frame >= boundary:
      out.roadChangeMonoTime = 10**12 + (1 if second_change is not None and rig.frame >= second_change else 0)
      out.source = int(source)
      out.speedLimitFinalLast = float(out.speedLimitFinalLast) + nudge
    else:
      out.roadChangeMonoTime = 0

  rig.planner.fill = fill_with_zone
  return rig


def adjusting(rows, start=0):
  return [s for s in spans(rows[start:]) if s[2].startswith('Adjusting')]


def test_road_change_keeps_the_offer_without_applying_it_and_a_fresh_press_confirms():
  _, boundary = offer_and_boundary()
  rows = run(rig_with_zone(boundary), N, lambda _: 25, presses=tap(boundary + 100), dash=36)
  before = rows[boundary - 1]
  assert before.state == AssistState.preActive and before.text.startswith('Press -')
  # the road change itself: the offer stays open on its frozen hold, nothing applied, no SET press, the prompt stays up
  for r in rows[boundary:boundary + 100]:
    assert r.state == AssistState.preActive and r.cap == before.cap and r.send == 'none' and r.text.startswith('Press -')
    assert r.announce == before.announce and r.setpoint == before.setpoint
  # the driver's fresh press confirms the kept offer: 30, "Adjusting to"
  confirmed = first(rows, lambda r: r.state == AssistState.active, boundary)
  assert confirmed < boundary + 130 and round(rows[confirmed].cap * MPH) == 30
  assert rows[confirmed].announce == before.announce + 1
  assert adjusting(rows, boundary)


@pytest.mark.parametrize('kw', [
  {'nudge': 1e-4},              # the same display number, a different internal target
  {'source': Source.vision},    # at the boundary only the new way's own map limit counts
], ids=['target-not-exact', 'not-map'])
def test_mismatched_evidence_at_the_road_change_expires_the_offer(kw):
  _, boundary = offer_and_boundary()
  rows = run(rig_with_zone(boundary, **kw), N, lambda _: 25, presses=tap(boundary + 10), dash=36)
  assert rows[boundary - 1].state == AssistState.preActive
  assert rows[boundary + 5].state != AssistState.preActive
  # the press after the boundary is a plain SET press: one step down, no confirm
  assert rows[boundary + 50].cap == 255. and setpoint(rows[boundary + 50]) == 35
  assert not adjusting(rows, boundary - 50)


def test_a_different_posted_limit_at_the_road_change_expires_the_offer():
  _, boundary = offer_and_boundary()
  rows = run(rig_with_zone(boundary), N, lambda i: 25 if i < boundary else 30, presses=tap(boundary + 10), dash=36)
  assert rows[boundary + 50].cap == 255. and setpoint(rows[boundary + 50]) == 35
  assert not adjusting(rows, boundary - 50)


def test_press_held_across_the_road_change_is_a_plain_press_and_a_new_press_confirms():
  _, boundary = offer_and_boundary()
  presses = {**tap(boundary - 4, hold=10), **tap(boundary + 100)}
  rows = run(rig_with_zone(boundary), N, lambda _: 25, presses=presses, dash=36)
  before = rows[boundary - 5]
  # the held press answers nothing: the car's own step (36 -> 35), the offer still pending, no SET press from the boundary
  window = rows[boundary:boundary + 100]
  assert all(r.state == AssistState.preActive and r.announce == before.announce for r in window)
  assert sum(1 for r in window if r.send != 'none') == 0
  assert window[-1].dash == 35
  # only the new press after its release confirms
  confirmed = first(rows, lambda r: r.state == AssistState.active, boundary)
  assert boundary + 100 <= confirmed < boundary + 130 and round(rows[confirmed].cap * MPH) == 30


@pytest.mark.parametrize('second', [False, True], ids=['one-change', 'repeat-change'])
def test_the_offer_window_restarts_once_per_offer(second):
  opened, boundary = offer_and_boundary()
  rig = rig_with_zone(boundary, second_change=boundary + 250 if second else None)
  rows = run(rig, N, lambda _: 25, dash=36)
  # restarted at the boundary: open past the original close (opened + 5 s), closed 5 s after the boundary, never later,
  # however many road-change signals come for the same offer
  assert rows[opened + 520].state == AssistState.preActive
  assert rows[boundary + 480].state == AssistState.preActive
  closed = first(rows, lambda r: r.state != AssistState.preActive, boundary)
  assert boundary + 495 <= closed <= boundary + 510
  assert all(r.send == 'none' for r in rows[boundary:closed + 1])


def test_kept_offer_lapses_when_the_evidence_changes_and_drops_the_queued_press():
  _, boundary = offer_and_boundary()
  change = boundary + 100
  rows = run(rig_with_zone(boundary), N, lambda i: 25 if i < change else 30, presses=tap(change - 2), dash=36)
  assert rows[change - 3].state == AssistState.preActive
  assert rows[change + 10].state != AssistState.preActive
  # the press queued over the lapse confirms nothing: a plain step, no "Adjusting to"
  assert setpoint(rows[change + 60]) == 35
  assert not adjusting(rows, boundary)


# ---------------------------------------------------------------- km/h: the gate on internal target, limit and direction

def metric_offer(cluster_kph=60, limit_kph=50.):
  arb = MazdaSlaSession(CP).arbiter
  arb.is_metric = True
  arb.state = SessionState.preActive
  arb.inputs_fresh = True
  arb._ramp_prompt = None
  arb._cluster_conv = cluster_kph
  arb._speed_limit = arb._slf = limit_kph * CV.KPH_TO_MS
  arb._offer_dir = compare_cluster_target(cluster_kph / arb._conv, arb._slf, True)
  arb._offer_window_restarted = False
  arb.pre_active_timer = 10
  return arb


def resolver_kph(limit_kph, target_kph=None, source=Source.map):
  return NS(speedLimitValid=True, source=int(source), speedLimit=limit_kph * CV.KPH_TO_MS,
            speedLimitFinalLast=(limit_kph if target_kph is None else target_kph) * CV.KPH_TO_MS)


def test_metric_offer_is_kept_only_on_an_exact_map_match_and_restarts_once():
  arb = metric_offer()
  assert arb._offer_dir == (False, True)   # SET 60 km/h, offer 50 km/h: "Press -"
  assert arb.keep_offer_across_road_change(resolver_kph(50.)) and arb.pre_active_timer == 500
  arb.pre_active_timer = 10
  assert arb.keep_offer_across_road_change(resolver_kph(50.)) and arb.pre_active_timer == 10   # never restarted again
  assert not arb.kept_offer_lapsed(resolver_kph(50.))
  assert arb.kept_offer_lapsed(resolver_kph(50., 50.4))   # displays 50 km/h, a different internal target


@pytest.mark.parametrize('resolver,cluster', [
  (resolver_kph(50.4), 60),                     # posted 50.4 km/h shows as 50: not the same zone
  (resolver_kph(60.), 60),                      # a different zone
  (resolver_kph(50., source=Source.vision), 60),
  (resolver_kph(50., source=Source.car), 60),
  (NS(speedLimitValid=False, source=int(Source.map), speedLimit=0., speedLimitFinalLast=0.), 60),
], ids=['50.4', '60', 'vision', 'car', 'no-limit'])
def test_metric_offer_is_not_kept_on_other_evidence(resolver, cluster):
  assert not metric_offer(cluster_kph=cluster).keep_offer_across_road_change(resolver)


def test_metric_offer_is_not_kept_when_the_press_direction_changed():
  arb = metric_offer()
  arb._cluster_conv = 50   # SET already on the offer: no press direction any more
  assert not arb.keep_offer_across_road_change(resolver_kph(50.))
  arb._cluster_conv = 45   # now "Press +"
  assert not arb.keep_offer_across_road_change(resolver_kph(50.))
