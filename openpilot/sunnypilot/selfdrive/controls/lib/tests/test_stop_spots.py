import json
import math

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.stop_spots import (BRAKE_MAX, EARTH_M_PER_DEG, HOLD_TIME, START_JERK,
                                                                    START_MAX, STOP_MARGIN, StopSpots, along_cross,
                                                                    load_spots)

SPOT = {"lat": 42.97205, "lon": -87.88467, "bearing": 184.}


def at(along: float, cross: float = 0., spot: dict = SPOT) -> tuple[float, float]:
  """lat, lon of a point along (negative = before) and across a spot's heading."""
  b = math.radians(spot["bearing"])
  dn = along * math.cos(b) - cross * math.sin(b)
  de = along * math.sin(b) + cross * math.cos(b)
  return spot["lat"] + dn / EARTH_M_PER_DEG, spot["lon"] + de / (EARTH_M_PER_DEG * math.cos(math.radians(spot["lat"])))


def drive(ss: StopSpots, start: float, v: float, seconds: float, bearing: float = SPOT["bearing"], cross: float = 0.,
          gas_at=None, fix=None, fix_hz: float = 10., deliver: float = 1., lag: float = 0., model=None,
          accuracy: float = 2., steer=None, fix_speed=None):
  """Drive from start (m along the spot) at v, following min(model, candidate) through a first-order lag at deliver
  of what is asked. fix(t, x) -> (along, cross, bearing[, accuracy]) or None overrides the fix (None: no fix this
  cycle). Returns per-cycle (along, v, candidate, should_stop)."""
  x, a_act, out = start, 0., []
  every = max(1, round(1. / (fix_hz * DT_MDL)))
  for i in range(int(seconds / DT_MDL)):
    t = i * DT_MDL
    f = (x, cross, bearing) if fix is None else fix(t, x)
    fresh = i % every == 0 and f is not None
    lat, lon = at(f[0], f[1]) if f is not None else (0., 0.)
    acc = f[3] if f is not None and len(f) > 3 else accuracy
    a = ss.update(lat, lon, f[2] if f is not None else 0., acc, fresh, v, v < 0.01, True, gas_at is not None and t >= gas_at,
                  0. if steer is None else steer(t, x), None if fix_speed is None else fix_speed(t, x, v))
    out.append((x, v, a, ss.should_stop))
    a_model = 0. if model is None else model(t, x, v)
    cmd = a_model if a is None else min(a_model, a)
    a_act = cmd if lag == 0. else a_act + (cmd - a_act) * DT_MDL / lag
    v = max(v + a_act * deliver * DT_MDL, 0.)
    x += v * DT_MDL
  return out


def stops(out):
  """Where each standstill began."""
  return [b[0] for a, b in zip(out, out[1:], strict=False) if a[1] > 0. and b[1] == 0.]


def test_stops_at_the_spot_and_hands_over_after_the_hold():
  ss = StopSpots([SPOT])
  out = drive(ss, -120., 12., 25.)
  braking = [r for r in out if r[2] is not None and r[2] < 0.]
  assert 60. <= -braking[0][0] <= 85., "brakes from about 70 m out at 27 mph"
  assert min(r[2] for r in braking) >= -START_MAX
  steps = [b[2] - a[2] for a, b in zip(braking, braking[1:], strict=False)]
  assert min(steps) >= -START_JERK * DT_MDL - 1e-9, "the brake builds at START_JERK"
  assert stops(out) == [pytest.approx(-2., abs=1.)], "one stop, about 2 m short of the drivers' median stop"
  held = [r for r in out if r[1] == 0. and r[3]]
  assert len(held) * DT_MDL == pytest.approx(HOLD_TIME, abs=2 * DT_MDL)
  assert out[-1][2] is None and ss.done_spot is SPOT, "the model has the go after the hold"


@pytest.mark.parametrize("bearing,cross", [(4., 0.), (184., 12.)])
def test_other_heading_or_a_parallel_road_never_brakes(bearing, cross):
  assert all(r[2] is None for r in drive(StopSpots([SPOT]), -120., 12., 15., bearing=bearing, cross=cross))


def test_gas_ends_the_pass():
  out = drive(StopSpots([SPOT]), -120., 12., 12., gas_at=5.)
  assert any(r[2] is not None for r in out[:int(5. / DT_MDL)]), "was braking before the gas"
  assert all(r[2] is None for r in out[int(5. / DT_MDL):])


def test_no_fix_before_it_brakes_ends_the_pass():
  out = drive(StopSpots([SPOT]), -120., 12., 8., fix=lambda t, x: (x, 0., 184.) if t < 1. else None)
  assert all(r[2] is None for r in out)


@pytest.mark.parametrize("start", [-1., -6.])
def test_a_fix_past_or_just_short_of_the_spot_never_brakes(start):
  """GPS first placing the car at the line, or a few metres short at 27 mph (a fix that jumped or came late): a stop
  there needs far more than START_MAX, so nothing brakes late and hard."""
  assert all(r[2] is None for r in drive(StopSpots([SPOT]), start, 12., 5.))


def test_a_slow_approach_brakes_late_and_gently():
  out = drive(StopSpots([SPOT]), -60., 3., 30.)
  braking = [r for r in out if r[2] is not None and r[2] < 0. and not r[3]]
  assert braking and -braking[0][0] <= 9.5 + STOP_MARGIN, "at 7 mph it starts 3 s out"
  assert min(r[2] for r in braking) >= -1.0
  assert stops(out) == [pytest.approx(-2., abs=1.)]


def test_a_crawl_the_model_lets_roll_on_is_stopped_at_the_spot():
  """The model slowed to 2 mph well before the line and rolls on (route 0000002b t=157: 2.2 mph, then +0.1)."""
  out = drive(StopSpots([SPOT]), -40., 0.9, 60.)
  assert stops(out) == [pytest.approx(-2., abs=1.)]
  assert min(r[2] for r in out if r[2] is not None) >= -1.0, "gently"
  assert not any(r[3] for r in out if r[0] < -2. - 4.5), "no stop asked short of the spot"


def test_a_crawl_short_of_the_spot_while_braking_is_not_asked_to_stop_there():
  """The model brakes harder and leaves the car rolling at 0.4 m/s 10 m short: no stop is asked until the spot."""
  def model(t, x, v):
    return -2.5 if -30. < x < -12. and v > 0.4 else 0.
  out = drive(StopSpots([SPOT]), -120., 12., 90., model=model)
  assert any(r[1] < 0.5 and r[0] < -8. for r in out), "crawled short of the spot"
  assert not any(r[3] for r in out if r[0] < -2. - 4.5)
  assert stops(out) == [pytest.approx(-2., abs=1.)]


def test_the_model_stopping_short_then_creeping_is_stopped_and_held_at_the_spot():
  """The model stops 8 m short on its own, waits, then creeps on: the spot stops it at the line, holds, hands over."""
  state = {"t_stop": None}

  def model(t, x, v):
    if x < -8.5:
      need = v ** 2 / (2. * max(-8. - x, 0.5))
      return -need if need >= 0.9 else 0.
    if v > 0. and state["t_stop"] is None:
      return -2.
    state["t_stop"] = t if state["t_stop"] is None else state["t_stop"]
    return 0.5 if t - state["t_stop"] > 3. and v < 1. else 0.

  ss = StopSpots([SPOT])
  out = drive(ss, -120., 12., 40., model=model)
  assert stops(out)[:2] == [pytest.approx(-8., abs=1.), pytest.approx(-2., abs=1.5)]
  assert sum(r[3] and r[1] == 0. for r in out) * DT_MDL == pytest.approx(HOLD_TIME, abs=2 * DT_MDL)
  assert ss.done_spot is SPOT


@pytest.mark.parametrize("fix", [
  lambda t, x: (x, 9., 184.) if 5. < t < 5.2 else (x, 0., 184.),  # one fix off the line
  lambda t, x: None if 5. < t < 7. else (x, 0., 184.),  # a 2 s gap mid-brake
  lambda t, x: (x + 3., 0., 184.) if t > 4. else (x, 0., 184.),  # projected stale fixes, 3 m ahead while braking
])
def test_once_braking_a_bad_missed_or_stale_fix_neither_releases_nor_moves_the_stop(fix):
  ref = drive(StopSpots([SPOT]), -120., 12., 25.)
  out = drive(StopSpots([SPOT]), -120., 12., 25., fix=fix)
  start = next(i for i, r in enumerate(out) if r[2] is not None)
  assert start * DT_MDL < 4., "braking before the bad fixes"
  end = next(i for i, r in enumerate(out) if r[1] == 0.)
  assert all(r[2] is not None for r in out[start:end]), "the brake is never released"
  assert stops(out) == [pytest.approx(stops(ref)[0], abs=0.05)]


@pytest.mark.parametrize("fix", [
  lambda t, x: (x - 3., 2., 0.) if t > 12. else (x, 0., 184.),  # wandering 3 m back, bearing lost at standstill
  lambda t, x: None if 12. < t < 15. else (x - 2.5, 0., 184.) if t >= 15. else (x, 0., 184.),  # a gap, then short
])
def test_a_wandering_or_lost_fix_at_the_line_never_cuts_the_hold_or_stops_twice(fix):
  out = drive(StopSpots([SPOT]), -120., 12., 30., fix=fix, fix_hz=1., model=lambda t, x, v: 0.3 if v < 1. else 0.)
  assert len(stops(out)) == 1
  assert sum(r[3] and r[1] == 0. for r in out) * DT_MDL == pytest.approx(HOLD_TIME, abs=2 * DT_MDL)


@pytest.mark.parametrize("deliver,lag", [(0.8, 0.8), (1., 0.6)])
def test_a_car_braking_less_or_later_than_asked_still_stops_at_the_line(deliver, lag):
  out = drive(StopSpots([SPOT]), -120., 15., 30., deliver=deliver, lag=lag)
  assert stops(out) and stops(out)[0] <= 0.5
  assert min(r[2] for r in out if r[2] is not None) >= -BRAKE_MAX


def test_after_the_hold_the_same_line_never_stops_again():
  ss = StopSpots([SPOT])
  out = drive(ss, -120., 12., 40., fix_hz=1., model=lambda t, x, v: 0.5 if v < 3. else 0.)
  assert len(stops(out)) == 1
  assert out[-1][0] > 30. and ss.done_spot is None, "the latch clears well past the spot"


@pytest.mark.parametrize("accuracy", [6., 12.])
def test_a_poor_fix_never_starts_it(accuracy):
  """Route 0000002b's approach read 12 m (the Jetson fallback): the feature stays off, the car drives as before."""
  assert all(r[2] is None for r in drive(StopSpots([SPOT]), -120., 12., 15., accuracy=accuracy))


def test_one_stray_fix_neither_starts_nor_ends_it():
  """A single good-looking fix 30 m off along the road, early: no brake from it, and the pass carries on."""
  ref = drive(StopSpots([SPOT]), -120., 12., 25.)
  out = drive(StopSpots([SPOT]), -120., 12., 25., fix=lambda t, x: (x + 30., 0., 184.) if 1. < t < 1.06 else (x, 0., 184.))
  assert all(r[2] is None for r in out[:int(1.5 / DT_MDL)])
  assert stops(out) == [pytest.approx(stops(ref)[0], abs=0.05)]


@pytest.mark.parametrize("by", ["steer", "gps"])
def test_turning_off_before_the_line_ends_the_pass(by):
  """Stopped 8 m short (a queue, or the model), then turning right into a side street with openpilot engaged: no
  brake and no hold on the new road. The wheel shows it (with the fix still on the line, as for a driveway inside the
  GPS error), or the fixes do (a wide turn with the wheel under LEAVE_STEER)."""
  state = {"t_stop": None}

  def model(t, x, v):
    if x < -10.5:
      need = v ** 2 / (2. * max(-10. - x, 0.5))
      return -need if need >= 1.5 else 0.  # after the spot has started braking
    if v > 0. and state["t_stop"] is None:
      return -2.
    state["t_stop"] = t if state["t_stop"] is None else state["t_stop"]
    return 1.5 if t - state["t_stop"] > 2. and v < 5. else 0.

  def turning(t):
    return state["t_stop"] is not None and t - state["t_stop"] >= 2.

  def fix(t, x):
    if by == "steer" or not turning(t):
      return (x, 0., 184.)
    # turning: x keeps counting distance driven, the fix moves off to the side at 90 degrees off the heading
    return (-10., 3. * (x + 10.), 274.)

  def steer(t, x):
    return 250. if by == "steer" and turning(t) else 0.

  ss = StopSpots([SPOT])
  out = drive(ss, -120., 12., 30., model=model, fix=fix, steer=steer)
  assert any(r[2] is not None for r in out if r[0] < -40.), "the spot was braking before the model's stop"
  assert len(stops(out)) == 1, "only the model's stop short"
  after = [r for r in out if state["t_stop"] is not None and r[0] > -9.]
  if by == "steer":
    assert after and all(r[2] is None for r in after)
  else:
    # the fixes show the turn some metres in: a light brake until then, never a stop
    assert after and min(r[2] for r in after if r[2] is not None) > -0.5
    assert all(r[2] is None for r in after if r[0] > -3.)
  assert ss.spot is None


def test_a_stale_standstill_fix_after_pulling_away_from_a_queue_keeps_the_stop():
  """Stopped 25 m short in a queue; pulling away, the Jetson fallback republishes the standstill fix (bearing 0, speed 0)
  for 2.5 s: that is not a turn off the road, and the car is still stopped at the line."""
  state = {"t_stop": None}

  def model(t, x, v):
    if x < -25.5:
      need = v ** 2 / (2. * max(-25. - x, 0.5))
      return -need if need >= 1.5 else 0.
    if v > 0. and state["t_stop"] is None:
      return -2.
    state["t_stop"] = t if state["t_stop"] is None else state["t_stop"]
    return 2.5 if t - state["t_stop"] > 2. and v < 5. else 0.

  def stale(t):
    return state["t_stop"] is not None and t - state["t_stop"] < 4.5

  out = drive(StopSpots([SPOT]), -120., 12., 40., model=model,
              fix=lambda t, x: (-25.3, 0., 0.) if stale(t) else (x, 0., 184.),
              fix_speed=lambda t, x, v: 0. if stale(t) else v)
  assert stops(out)[:2] == [pytest.approx(-25.3, abs=0.5), pytest.approx(-2., abs=1.5)]


def test_fixes_that_disagree_never_start_it():
  """Good-accuracy fixes jumping 10 m back and forth (multipath): never ARM_FIXES in a row that agree."""
  out = drive(StopSpots([SPOT]), -120., 12., 15., fix=lambda t, x: (x + (10. if round(t / DT_MDL) % 4 else 0.), 0., 184.))
  assert all(r[2] is None for r in out)


def test_a_braked_for_pass_ends_after_pass_max_time():
  ss = StopSpots([SPOT])
  drive(ss, -120., 12., 9.)
  assert ss.committed
  for _ in range(int(95. / DT_MDL)):
    ss.update(*at(-30.), SPOT["bearing"], 2., False, 0., True, True, False)
  assert ss.spot is None and ss.done_spot is SPOT


def test_no_spots_or_bad_spots_do_nothing(tmp_path):
  assert load_spots(tmp_path / "missing.json") == []
  (tmp_path / "bad.json").write_text("{not json")
  assert load_spots(tmp_path / "bad.json") == []
  (tmp_path / "list.json").write_text(json.dumps({"spots": {"lat": 1.}}))
  assert load_spots(tmp_path / "list.json") == []
  bad = [{"lat": 1.}, {"lat": 42., "lon": -87., "bearing": "180"}, {"lat": float("nan"), "lon": -87., "bearing": 0.},
         {"lat": 42., "lon": -87., "bearing": True}, {"lat": 420., "lon": -87., "bearing": 0.}]
  (tmp_path / "ok.json").write_text(json.dumps({"spots": [SPOT, *bad]}))
  assert load_spots(tmp_path / "ok.json") == [SPOT]
  assert StopSpots([]).update(*at(-50.), SPOT["bearing"], 2., True, 12., False, True, False) is None


def test_along_cross_round_trips():
  for along, cross in ((-70., 0.), (5., -3.), (0., 7.)):
    assert along_cross(SPOT, *at(along, cross)) == pytest.approx((along, cross), abs=1e-3)
