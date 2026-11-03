import json
import math

import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.stop_spots import (BRAKE_MAX, EARTH_M_PER_DEG, HOLD_TIME, START_JERK,
                                                                    START_MAX, STOP_MARGIN, StopSpots, along_cross,
                                                                    load_spots)

SPOT = {"lat": 42.97205, "lon": -87.88467, "bearing": 184., "verified": True}


def at(along: float, cross: float = 0., spot: dict = SPOT) -> tuple[float, float]:
  """lat, lon of a point along (negative = before) and across a spot's heading."""
  b = math.radians(spot["bearing"])
  dn = along * math.cos(b) - cross * math.sin(b)
  de = along * math.sin(b) + cross * math.cos(b)
  return spot["lat"] + dn / EARTH_M_PER_DEG, spot["lon"] + de / (EARTH_M_PER_DEG * math.cos(math.radians(spot["lat"])))


def drive(ss: StopSpots, start: float, v: float, seconds: float, bearing: float = SPOT["bearing"], cross: float = 0.,
          gas_at=None, fix=None, fix_hz: float = 10., deliver: float = 1., lag: float = 0., model=None,
          accuracy: float = 2., steer=None, fix_speed=None, gas=None, lines=None, lead=None):
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
    pressed = (gas_at is not None and t >= gas_at) or (gas is not None and gas(t))
    a = ss.update(lat, lon, f[2] if f is not None else 0., acc, fresh, v, v < 0.01, True, pressed,
                  0. if steer is None else steer(t, x), None if fix_speed is None else fix_speed(t, x, v),
                  lines(t, x) if lines is not None and i % 2 == 0 else None, None if lead is None else lead(t, x))
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
  assert 45. <= -braking[0][0] - STOP_MARGIN <= 58., "brakes about 50-55 m before the stop at 27 mph, where the driver's own stops start"
  assert min(r[2] for r in braking) >= -START_MAX
  steps = [b[2] - a[2] for a, b in zip(braking, braking[1:], strict=False)]
  assert min(steps) >= -START_JERK * DT_MDL - 1e-9, "the brake builds at START_JERK"
  assert stops(out) == [pytest.approx(-STOP_MARGIN, abs=1.)], "one stop, STOP_MARGIN short of the line"
  held = [r for r in out if r[1] == 0. and r[3]]
  assert len(held) * DT_MDL == pytest.approx(HOLD_TIME, abs=2 * DT_MDL)
  assert out[-1][2] is None and ss.done_spot is SPOT, "the model has the go after the hold"


@pytest.mark.parametrize("bearing,cross", [(4., 0.), (184., 12.)])
def test_other_heading_or_a_parallel_road_never_brakes(bearing, cross):
  assert all(r[2] is None for r in drive(StopSpots([SPOT]), -120., 12., 15., bearing=bearing, cross=cross))


def test_a_gas_tap_pauses_the_stop_and_it_resumes():
  """Route 0000002c t=174: a 1 s tap 46 m out, the model slowing 27 -> 22 mph meanwhile, released 36 m out. No brake
  while it is held; on release the stop carries on to the line."""
  def model(t, x, v):
    return -1.2 if 6.2 <= t < 7.2 else 0.
  out = drive(StopSpots([SPOT]), -120., 12., 25., gas=lambda t: 6.2 <= t < 7.2, model=model)
  assert any(r[2] is not None for r in out[:int(6.2 / DT_MDL)]), "was braking before the gas"
  assert all(r[2] is None for r in out[int(6.2 / DT_MDL):int(7.2 / DT_MDL)])
  assert stops(out) == [pytest.approx(-STOP_MARGIN, abs=1.)]


def test_gas_from_the_hold_hands_the_go_to_the_driver():
  """Stopped at the line in the hold, the driver presses the gas to go: no second stop, no hold."""
  ss = StopSpots([SPOT])
  out = drive(ss, -120., 12., 40., gas=lambda t: t >= 17., model=lambda t, x, v: 1.0 if t >= 17. else 0.)
  assert len(stops(out)) == 1
  assert all(r[2] is None for r in out[int(17. / DT_MDL):])


def test_gas_released_at_the_line_is_the_drivers():
  """Pushed off a crawl and lifted at the line to look: no brake back to a stop in the junction."""
  state = {"t": None}

  def gas(t):
    return state["t"] is not None and state["t"] <= t < state["t"] + 1.5

  def model(t, x, v):
    if state["t"] is None and x > -6.:
      state["t"] = t
    return 1.0 if gas(t) else 0.

  out = drive(StopSpots([SPOT]), -12., 0.9, 20., gas=lambda t: state["t"] is not None and state["t"] <= t < state["t"] + 0.6,
              model=model)
  assert stops(out) == [], "lifted about 3 m from the aim: not braked back to a stop"


def test_gas_from_a_queue_stop_short_hands_the_go_to_the_driver():
  """Stopped 20 m short in a queue, the driver creeps up on the gas and lifts: no stop at the line after that."""
  state = {"t_stop": None}

  def model(t, x, v):
    if x < -22.5:
      need = v ** 2 / (2. * max(-22. - x, 0.5))
      return -need if need >= 1.8 else 0.
    if v > 0. and state["t_stop"] is None:
      return -2.5
    state["t_stop"] = t if state["t_stop"] is None else state["t_stop"]
    return 0.8 if 2. <= t - state["t_stop"] < 3. else 0.

  out = drive(StopSpots([SPOT]), -120., 12., 40., model=model,
              gas=lambda t: state["t_stop"] is not None and 2. <= t - state["t_stop"] < 3.)
  assert stops(out)[0] == pytest.approx(-22., abs=1.5) and len(stops(out)) == 1
  assert all(r[2] is None for r in out if r[0] > -21.)


@pytest.mark.parametrize("released_at", [-33., -45.])
def test_gas_released_resumes_only_a_gentle_stop(released_at):
  """Held at 27 mph and released 31 m from the aim (needs 2.3) the stop is the driver's; at 43 m (1.7) as well, over
  RESUME_MAX, as the ramp back would take it near BRAKE_MAX."""
  state = {}

  def gas(t):
    return state.get("released") is None

  def model(t, x, v):
    if x >= released_at:
      state["released"] = True
    return 0.

  out = drive(StopSpots([SPOT]), -120., 12., 25., gas=gas, model=model)
  assert all(r[2] is None for r in out)


def test_gas_released_under_resume_dist_is_the_drivers():
  """Pushed at 7 mph and lifted 7 m from the aim: a resumed ramp there plus actuator lag reached 2-3 m/s2."""
  out = drive(StopSpots([SPOT]), -40., 3., 30., gas=lambda t: 5. <= t < 9.6, lag=0.5)
  assert any(-12. < r[0] < -8. for r in out[int(9.5 / DT_MDL):int(9.7 / DT_MDL)]), "released 6-10 m from the aim"
  assert all(r[2] is None for r in out[int(9.6 / DT_MDL):])


def test_gas_held_through_the_line_ends_the_pass():
  out = drive(StopSpots([SPOT]), -120., 12., 25., gas=lambda t: t >= 6.5)
  assert all(r[2] is None for r in out[int(6.5 / DT_MDL):]) and stops(out) == []


def test_gas_released_too_close_to_stop_gently_is_the_drivers():
  """Released 8 m out at 27 mph: a stop there needs 9 m/s2; no brake from the spot."""
  out = drive(StopSpots([SPOT]), -120., 12., 15., gas=lambda t: 6.5 <= t < 9.)
  assert all(r[2] is None for r in out[int(6.5 / DT_MDL):])


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
  assert stops(out) == [pytest.approx(-STOP_MARGIN, abs=1.)]


def test_a_crawl_the_model_lets_roll_on_is_stopped_at_the_spot():
  """The model slowed to 2 mph well before the line and rolls on (route 0000002b t=157: 2.2 mph, then +0.1)."""
  out = drive(StopSpots([SPOT]), -40., 0.9, 60.)
  assert stops(out) == [pytest.approx(-STOP_MARGIN, abs=1.)]
  assert min(r[2] for r in out if r[2] is not None) >= -1.0, "gently"
  assert not any(r[3] for r in out if r[0] < -STOP_MARGIN - 4.5), "no stop asked short of the spot"


def test_a_crawl_short_of_the_spot_while_braking_is_not_asked_to_stop_there():
  """The model brakes harder and leaves the car rolling at 0.4 m/s 10 m short: no stop is asked until the spot."""
  def model(t, x, v):
    return -2.5 if -45. < x < -15. and v > 0.4 else 0.
  out = drive(StopSpots([SPOT]), -120., 12., 90., model=model)
  assert any(r[1] < 0.5 and r[0] < -8. for r in out), "crawled short of the spot"
  assert not any(r[3] for r in out if r[0] < -STOP_MARGIN - 4.5)
  assert stops(out) == [pytest.approx(-STOP_MARGIN, abs=1.)]


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
  assert stops(out)[:2] == [pytest.approx(-8., abs=1.), pytest.approx(-STOP_MARGIN, abs=1.5)]
  assert sum(r[3] and r[1] == 0. for r in out) * DT_MDL == pytest.approx(HOLD_TIME, abs=2 * DT_MDL)
  assert ss.done_spot is SPOT


@pytest.mark.parametrize("fix", [
  lambda t, x: (x, 9., 184.) if 7. < t < 7.2 else (x, 0., 184.),  # one fix off the line
  lambda t, x: None if 7. < t < 9. else (x, 0., 184.),  # a 2 s gap mid-brake
  lambda t, x: (x + 3., 0., 184.) if t > 6.5 else (x, 0., 184.),  # projected stale fixes, 3 m ahead while braking
])
def test_once_braking_a_bad_missed_or_stale_fix_neither_releases_nor_moves_the_stop(fix):
  ref = drive(StopSpots([SPOT]), -120., 12., 25.)
  out = drive(StopSpots([SPOT]), -120., 12., 25., fix=fix)
  start = next(i for i, r in enumerate(out) if r[2] is not None)
  assert start * DT_MDL < 6.5, "braking before the bad fixes"
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


@pytest.mark.parametrize("deliver,lag", [(0.87, 0.55), (1., 0.6)])  # measured at low speed: 0.87 of the ask, 0.55 s
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
      return -need if need >= 1.8 else 0.  # after the spot has started braking
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
  assert stops(out)[:2] == [pytest.approx(-25.3, abs=0.5), pytest.approx(-STOP_MARGIN, abs=1.5)]


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
         {"lat": 42., "lon": -87., "bearing": True}, {"lat": 420., "lon": -87., "bearing": 0., "verified": True},
         dict(SPOT, verified=False), {k: v for k, v in SPOT.items() if k != "verified"}]  # learned, not checked: no brake
  (tmp_path / "ok.json").write_text(json.dumps({"spots": [SPOT, *bad]}))
  assert load_spots(tmp_path / "ok.json") == [SPOT]
  assert StopSpots([]).update(*at(-50.), SPOT["bearing"], 2., True, 12., False, True, False) is None


def test_along_cross_round_trips():
  for along, cross in ((-70., 0.), (5., -3.), (0., 7.)):
    assert along_cross(SPOT, *at(along, cross)) == pytest.approx((along, cross), abs=1e-3)


def camera(line_x: float, extra: tuple[float, ...] = ()):
  """The camera's bands for a stop line truly at line_x (the spot's GPS frame), as stop_line.py sees them: from 5.4 m
  (the hood) out to 30 m."""
  return lambda t, x: sorted(d for d in [line_x - x, *(e - x for e in extra)] if 5.4 < d < 30.)


@pytest.mark.parametrize("gps_off", [-7., -4., 0., 4., 7.])
def test_the_camera_places_the_stop_at_the_real_line_whatever_the_gps_says(gps_off):
  """Routes 0000002c and 0000002d put the same line 6-7.5 m apart by GPS. The camera's line wins once seen."""
  out = drive(StopSpots([SPOT]), -120., 12., 25., lines=camera(gps_off))
  # a line found nearer than the GPS put it, mid-brake, ends up to 0.6 m past the aim: the brake builds at START_JERK
  assert stops(out) == [pytest.approx(gps_off - STOP_MARGIN, abs=0.75)]


def test_without_the_camera_the_gps_spot_is_used():
  assert stops(drive(StopSpots([SPOT]), -120., 12., 25., lines=lambda t, x: [])) == [pytest.approx(-STOP_MARGIN, abs=1.)]


def test_a_band_far_from_the_spot_or_seen_once_does_not_move_the_stop():
  far = drive(StopSpots([SPOT]), -120., 12., 25., lines=camera(-14.))  # beyond LINE_TOL: some other marking
  assert stops(far) == [pytest.approx(-STOP_MARGIN, abs=1.)]
  ss = StopSpots([SPOT])
  once = drive(ss, -120., 12., 25., lines=lambda t, x: [-5. - x] if 4. < t < 4.06 and 5.4 < -5. - x < 30. else [])
  assert stops(once) == [pytest.approx(-STOP_MARGIN, abs=1.)] and not ss.line_locked


def test_the_crosswalks_near_edge_is_taken_and_later_fixes_do_not_move_it():
  """Both edges of a crosswalk in view: the nearer (upstream) one is the line. Fixes jumping 5 m after the lock do not
  move the stop."""
  ss = StopSpots([SPOT])
  out = drive(ss, -120., 12., 25., lines=camera(-4., extra=(-0.5,)),
              fix=lambda t, x: (x + (5. if t > 7. else 0.), 0., 184.))
  assert stops(out) == [pytest.approx(-4. - STOP_MARGIN, abs=0.75)]


def test_a_camera_line_the_gps_disagrees_with_still_stops_the_car():
  """Route 0000002d northbound: GPS 4.5 m off the line the camera locked, and GPS passing the spot before the line.
  The camera's line is sure on its own, and GPS no longer ends the pass."""
  # locked before any GPS agreement (fixes at 1 Hz): the camera alone is enough to brake
  out = drive(StopSpots([SPOT]), -28., 4.5, 30., lines=camera(0.), fix=lambda t, x: (x + 4.5, 0., 184.), fix_hz=1.)
  assert stops(out) == [pytest.approx(-STOP_MARGIN, abs=0.75)]
  # a crawl GPS puts past the spot 7 m before the line, before the brake starts: the camera keeps the pass
  out = drive(StopSpots([SPOT]), -25., 1., 40., lines=camera(0.), fix=lambda t, x: (x + 7., 0., 184.), fix_hz=1.)
  assert stops(out) == [pytest.approx(-STOP_MARGIN, abs=0.75)]


def test_a_lead_at_the_line_that_pulls_away_does_not_drag_the_stop_into_the_junction():
  """Review of route 0000002d at 3-8 s: a lead reads as a band at about 1.8x its gap. Stopped on the line (hiding it),
  then pulling away at 1.5 m/s2: bands from LEAD_BAND of its distance on are dropped, and the stop stays at the line."""
  def lead_pos(t):  # the lead's rear: stopped at the line until 20 s, then gone
    return -5.5 + (0.75 * (t - 20.) ** 2 if t > 20. else 0.)

  def lead(t, x):
    return lead_pos(t) - x if lead_pos(t) - x < 80. else None

  def lines(t, x):  # the lead hides the line while it sits on it, and reads as a band itself
    gap = lead_pos(t) - x
    hidden = lead_pos(t) < 2.
    return sorted(d for d in ([] if hidden else [-x]) + [1.8 * gap] if 5.4 < d < 30.)

  def model(t, x, v):  # stops 6 m behind the lead as the lead MPC does, then creeps once it has gone
    if t < 20.:
      if x >= lead_pos(t) - 6.5:
        return -3. if v > 0. else 0.
      need = v ** 2 / (2. * max(lead_pos(t) - 6. - x, 0.3))
      return -need if need > 0.3 else 0.
    return 0.6 if v < 2. else 0.

  out = drive(StopSpots([SPOT]), -120., 12., 40., model=model, lines=lines, lead=lead)
  assert len(stops(out)) == 2, "behind the lead, then at the line"
  assert stops(out)[1] == pytest.approx(-STOP_MARGIN, abs=0.75), "at the line, not dragged after the lead"


def test_a_band_drifting_off_after_the_lock_moves_the_stop_at_most_line_drift():
  """An undetected lead or a moving shadow: a band that keeps moving away is followed no more than LINE_DRIFT."""
  def lines(t, x):
    pos = 0. if t < 7. else 0.5 * (t - 7.)
    return [pos - x] if 5.4 < pos - x < 30. else []
  out = drive(StopSpots([SPOT]), -120., 12., 30., lines=lines)
  assert stops(out) and stops(out)[0] <= -STOP_MARGIN + 2. + 1.


def test_no_lock_before_the_gps_has_agreed_on_the_spot():
  ss = StopSpots([SPOT])
  out = drive(ss, -18., 4., 1.2, lines=camera(0.), fix_hz=1.)
  assert not ss.line_locked and all(r[2] is None for r in out)


def test_far_bands_read_off_by_a_nose_dive_are_not_locked_on():
  """At 25 m a 0.5 deg nose-dive puts a band 4 m further; only bands within LOCK_MAX_DIST lock."""
  def lines(t, x):
    d = -x
    return [d + (4. if d > 20. else 0.)] if 5.4 < d < 30. else []
  out = drive(StopSpots([SPOT]), -120., 12., 25., lines=lines)
  assert stops(out) == [pytest.approx(-STOP_MARGIN, abs=0.75)]


@pytest.mark.parametrize("dropout", [False, True])
def test_a_lead_lost_by_the_radar_for_a_moment_is_still_not_a_line(dropout):
  """Radar drops a close lead for up to 1.2 s (route 0000002c t=154.9). A lead parked on the line, its band 1.8x the
  gap, with 0.4 s dropouts every 2 s: the last lead stands for LEAD_HOLD, and the stop stays before the line."""
  def lead(t, x):
    return None if dropout and t % 2. < 0.4 else -5.5 - x

  def lines(t, x):
    return [d for d in [1.8 * (-5.5 - x)] if 5.4 < d < 30.]

  def model(t, x, v):
    if x >= -5.5 - 6.5:
      return -3. if v > 0. else 0.
    need = v ** 2 / (2. * max(-5.5 - 6. - x, 0.3))
    return -need if need > 0.3 else 0.

  ss = StopSpots([SPOT])
  drive(ss, -120., 12., 25., model=model, lines=lines, lead=lead)
  assert not ss.line_locked


def test_in_a_queue_the_line_before_the_lead_is_still_read():
  """A lead through the junction 15 m past the line: the line before it is road, and places the stop despite GPS
  4 m off."""
  def lead(t, x):
    return 15. - x

  def lines(t, x):
    return sorted(d for d in [-x, 1.8 * (15. - x)] if 5.4 < d < 30.)

  out = drive(StopSpots([SPOT]), -120., 12., 25., lines=lines, lead=lead, fix=lambda t, x: (x + 4., 0., 184.))
  assert stops(out) == [pytest.approx(-STOP_MARGIN, abs=0.75)]
