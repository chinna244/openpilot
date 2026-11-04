import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.model_stop_line import GO_TIME, ModelStopLine, model_stop_dist
from openpilot.sunnypilot.selfdrive.controls.lib.stop_spots import BRAKE_MAX, HOLD_ACCEL, START_JERK, START_MAX, STOP_MARGIN

LINE = 60.  # m along the road: the stop line
CROSSWALK = LINE + 4.  # its far edge
MODEL_STOP = LINE + 2.5  # where the model stops: on the crosswalk (route 00000030 northbound)
HOOD = 5.4  # m: the camera sees no road nearer


def drive(ms: ModelStopLine, v: float, seconds: float, x: float = 0., bands=(LINE, CROSSWALK), model_stop=MODEL_STOP,
          sees_from: float = 50., go_at=None, gas=None, lead=None, steer=0., deliver: float = 1.):
  """Drive at v toward a stop the model plans at model_stop (once within sees_from of it), braking at the constant
  decel that stops there; the candidate, when lower, is followed. go_at: s from which the model plans to go. Returns
  per-cycle (x, v, candidate, should_stop)."""
  out = []
  for i in range(int(seconds / DT_MDL)):
    t = i * DT_MDL
    going = go_at is not None and t >= go_at
    to_stop = model_stop - x
    planned = None if going or to_stop > sees_from else max(to_stop, 0.)
    seen = [b - x for b in bands if HOOD <= b - x] if i % 2 == 0 else None
    pressed = gas is not None and gas(t)
    a = ms.update(v, v < 0.01, True, pressed, steer(t) if callable(steer) else steer, planned, seen, None if lead is None else lead(t, x))
    out.append((x, v, a, ms.should_stop))
    a_model = 0.5 if going else (-v ** 2 / (2. * max(to_stop, 0.3)) if planned is not None else 0.)
    cmd = a_model if a is None else min(a_model, a)
    v = max(v + cmd * deliver * DT_MDL, 0.)
    x += v * DT_MDL
  return out


def stopped_at(out):
  return next((r[0] for r in out[1:] if r[1] < 0.1), None)  # the sim's model creeps the last cm in


def test_model_stop_on_the_crosswalk_moves_before_the_line():
  out = drive(ModelStopLine(), 12., 20.)
  x = stopped_at(out)
  assert x is not None
  assert LINE - STOP_MARGIN - 1. <= x <= LINE - STOP_MARGIN + 0.5, "stops STOP_MARGIN short of the line, not on the crosswalk"
  asks = [r[2] for r in out if r[2] is not None]
  assert min(asks) >= -BRAKE_MAX


def test_without_the_candidate_the_model_stops_on_the_crosswalk():
  out = []
  ms = ModelStopLine()
  ms.update = lambda *a, **k: None
  out = drive(ms, 12., 20.)
  assert LINE < stopped_at(out) < CROSSWALK


def test_crosswalk_without_a_stop_line_stops_before_its_near_edge():
  out = drive(ModelStopLine(), 12., 20., bands=(LINE, LINE + 3.5), model_stop=LINE + 1.5)
  assert LINE - STOP_MARGIN - 1. <= stopped_at(out) <= LINE - STOP_MARGIN + 0.5


def test_no_paint_leaves_the_model_stop():
  out = drive(ModelStopLine(), 12., 20., bands=())
  assert all(r[2] is None for r in out)


def test_never_later_than_the_model():
  """A band past where the model stops places nothing after it."""
  out = drive(ModelStopLine(), 12., 20., bands=(MODEL_STOP + STOP_MARGIN + 0.5,))
  assert stopped_at(out) <= MODEL_STOP + 0.1
  out = drive(ModelStopLine(), 12., 20., bands=(MODEL_STOP + STOP_MARGIN + 1.5,))
  assert all(r[2] is None for r in out)


def test_a_stray_band_far_before_the_model_stop_is_ignored():
  out = drive(ModelStopLine(), 12., 20., bands=(MODEL_STOP - 19., LINE, CROSSWALK))
  assert LINE - STOP_MARGIN - 1. <= stopped_at(out) <= LINE - STOP_MARGIN + 0.5


def test_no_stop_planned_reads_nothing():
  out = drive(ModelStopLine(), 12., 6., sees_from=0.)
  assert all(r[2] is None for r in out)


def test_holds_at_the_line_while_the_model_still_stops_then_lets_go_when_it_goes():
  ms = ModelStopLine()
  out = drive(ms, 12., 40., go_at=30.)
  held = [r for r in out if 15. <= out.index(r) * DT_MDL < 30.]
  assert all(r[1] == 0. for r in held[-100:]), "still at the line 25-30 s: a red light"
  assert out[-1][1] > 0. and not ms.locked
  release = [i for i, r in enumerate(out) if i * DT_MDL >= 30. and r[2] is None][0]
  assert (release * DT_MDL - 30.) == pytest.approx(GO_TIME, abs=2 * DT_MDL)


def test_one_odd_plan_does_not_release():
  ms = ModelStopLine()
  drive(ms, 12., 15.)
  assert ms.locked
  ms.update(0., True, True, False, 0., None, None, None)
  assert ms.locked
  assert ms.update(0., True, True, False, 0., 2., None, None) is not None


def test_the_driver_gas_hands_the_stop_back_until_the_model_goes():
  ms = ModelStopLine()
  out = drive(ms, 12., 9., gas=lambda t: 6. <= t < 6.5)
  assert any(r[2] is not None for r in out[:int(6. / DT_MDL)])
  assert ms.locked is False and ms.done
  assert all(r[2] is None for r in out[int(6. / DT_MDL):])


def test_a_lead_leaves_the_stop_to_the_lead_mpc():
  out = drive(ModelStopLine(), 12., 20., lead=lambda t, x: MODEL_STOP + 2. - x)
  assert all(r[2] is None for r in out)


def test_turning_reads_no_band():
  out = drive(ModelStopLine(), 12., 20., steer=90.)
  assert all(r[2] is None for r in out)


def test_found_too_late_to_stop_gently_is_the_models():
  # the model sees the stop only 15 m out at 12 m/s: stopping 3 m short of the line needs well over START_MAX
  ms = ModelStopLine()
  out = drive(ms, 12., 10., sees_from=MODEL_STOP - 45.)
  assert ms.done or all(r[2] is None or r[2] >= -START_MAX for r in out)
  assert all(r[2] is None for r in out)


def test_a_car_that_delivers_less_still_stops_before_the_crosswalk():
  out = drive(ModelStopLine(), 12., 25., deliver=0.8)
  # 80% of the ask from a line seen 16 m out at 8.4 m/s: short of the 3 m margin, still well before the model's stop
  assert stopped_at(out) < MODEL_STOP - 2.5


def plan(v0: float, a: float, stop_at: float | None = None):
  """The model's plan on its own time points: from v0 at a, or braking to stop_at m ahead."""
  t = np.asarray(ModelConstants.T_IDXS)
  if stop_at is None:
    v = np.maximum(v0 + a * t, 0.)
  else:
    decel = v0 ** 2 / (2. * stop_at)
    v = np.maximum(v0 - decel * t, 0.)
  x = np.concatenate([[0.], np.cumsum((v[1:] + v[:-1]) / 2. * np.diff(t))])
  return list(v), list(x)


def creep_plan(dist: float, v_creep: float = 1.2, a: float = 0.6):
  """From a standstill up to v_creep at a, then down at a to a stop dist m on: the model creeping to its own stop."""
  t = np.asarray(ModelConstants.T_IDXS)
  fine = np.arange(0., t[-1] + 0.01, 0.01)
  v, x, vs, xs = 0., 0., [], []
  for _ in fine:
    vs.append(v)
    xs.append(x)
    v = max(0., min(v_creep, v + a * 0.01) if dist - x > v ** 2 / (2. * a) else v - a * 0.01)
    x += v * 0.01
  return list(np.interp(t, fine, vs)), list(np.interp(t, fine, xs))


def test_model_stop_dist():
  assert model_stop_dist(*plan(10., 0., stop_at=30.)) == pytest.approx(30., abs=0.5)
  assert model_stop_dist(*plan(10., 0.)) is None
  assert model_stop_dist(*plan(0., 0.)) == 0., "held at a red light: a stop where the car is"
  assert model_stop_dist([], []) is None


@pytest.mark.parametrize("a", [0.5, 1.0, 2.0])
def test_a_launch_from_standstill_is_no_stop(a):
  """The plan starts at the car's speed: a launch's first points are under the stop speed too (review: green light)."""
  assert model_stop_dist(*plan(0., a)) is None


def test_lets_go_at_a_green_light_on_the_models_launch_plan():
  ms = ModelStopLine()
  drive(ms, 12., 15.)
  assert ms.locked
  for _ in range(int(GO_TIME / DT_MDL) + 1):
    a = ms.update(0., True, True, False, 0., model_stop_dist(*plan(0., 1.)), None, None)
  assert a is None and not ms.locked and not ms.done
  assert ms.update(0., True, True, False, 0., model_stop_dist(*plan(0., 0.)), None, None) is None, "a new stop needs a band again"


def test_the_first_of_two_bands_seen_together_is_the_line():
  # the model stops only 15 m out: both bands are already in view
  out = drive(ModelStopLine(), 6., 20., bands=(LINE, LINE + 1.5), sees_from=15.)
  assert LINE - STOP_MARGIN - 1. <= stopped_at(out) <= LINE - STOP_MARGIN + 0.5


def test_a_band_just_past_the_model_stop_never_moves_it_later():
  out = drive(ModelStopLine(), 12., 20., bands=(MODEL_STOP + STOP_MARGIN - 0.5,))
  assert stopped_at(out) <= MODEL_STOP + 0.1
  # seen late and fast, the shrunk margin would aim past the model's stop: nothing is placed
  out = drive(ModelStopLine(), 8., 20., bands=(MODEL_STOP + STOP_MARGIN - 0.5,), sees_from=15.)
  assert all(r[2] is None for r in out)


def test_turning_off_before_the_line_hands_the_stop_back():
  ms = ModelStopLine()
  out = drive(ms, 12., 20., steer=lambda t: 150. if t >= 5. else 0.)
  assert any(r[2] is not None for r in out[:int(5. / DT_MDL)])
  assert all(r[2] is None for r in out[int(5. / DT_MDL):]) and ms.done


@pytest.mark.parametrize("frames", [1, 3, 5])
def test_a_model_stop_flickering_on_for_a_few_frames_places_nothing(frames):
  """Review 2026-11-03: one frame of model stop braked to -2.2 for GO_TIME, then let go in a step."""
  ms = ModelStopLine()
  x, v = 0., 7.
  for i in range(int(8. / DT_MDL)):
    # from x 44 m: the line 16 m out, in view and firm, a stop before it within BRAKE_MAX
    stop = MODEL_STOP - x if 126 <= i < 126 + frames else None
    a = ms.update(v, False, True, False, 0., stop, [b - x for b in (LINE, CROSSWALK) if HOOD <= b - x] if i % 2 == 0 else None, None)
    assert a is None and not ms.locked
    x += v * DT_MDL


@pytest.mark.parametrize("v0", [5., 7., 9.])
@pytest.mark.parametrize("before", [9., 12., 14.])
def test_a_band_further_back_than_go_dist_never_pulses_the_brake(v0, before):
  """Review 2026-11-03: a band 8-15 m before the model's stop locked, read as the model going, let go, locked again:
  up to 9 pulses of -2.2. It is no line for that stop: the model's own stop stands."""
  ms = ModelStopLine()
  out = drive(ms, v0, 30., bands=(MODEL_STOP - before,))
  assert all(r[2] is None for r in out)
  assert stopped_at(out) == pytest.approx(stopped_at(drive(ModelStopLine(), v0, 30., bands=())))


def test_stopped_at_the_line_a_model_creeping_on_to_its_own_stop_is_still_held():
  """Review 2026-11-03: held 3 m before the line, the model plans to creep to its own stop on the crosswalk; that is a
  stop, not a go: the hold stays."""
  ms = ModelStopLine()
  out = drive(ms, 12., 20.)
  assert ms.locked and out[-1][3]
  creep = creep_plan(MODEL_STOP - out[-1][0])
  assert model_stop_dist(*creep) == pytest.approx(MODEL_STOP - out[-1][0], abs=0.5)
  assert model_stop_dist(*plan(0., 0.6)) is None, "a launch that keeps going is still a go"
  for _ in range(int(5. / DT_MDL)):
    a = ms.update(0., True, True, False, 0., model_stop_dist(*creep), None, None)
    assert a is not None and ms.should_stop


def test_a_bright_spot_that_keeps_its_distance_is_not_the_line():
  """A glare at a fixed distance ahead, in the window before the model's stop: not paint, nothing is placed for it."""
  ms = ModelStopLine()
  x, v = 0., 6.
  for i in range(int(10. / DT_MDL)):
    to_stop = MODEL_STOP - x
    if to_stop < 8.:
      break
    # 10 m ahead every frame: in the window from GO_DIST before the model's stop while it is 7-18 m out
    a = ms.update(v, False, True, False, 0., to_stop if to_stop < 25. else None, [10.] if i % 2 == 0 else None, None)
    assert a is None
    x += v * DT_MDL


def test_the_hold_eases_off_a_hard_stop_without_a_step():
  ms = ModelStopLine()
  ms.locked, ms.dist, ms.anchor, ms.a_target = True, 0.5, 0.5, -BRAKE_MAX
  prev = ms.a_target
  for _ in range(40):
    a = ms.update(0.3, False, True, False, 0., 1., None, None)
    assert abs(a - prev) <= START_JERK * DT_MDL + 1e-9
    prev = a
  assert a == pytest.approx(HOLD_ACCEL)


@pytest.mark.parametrize("gap_at", [1.0, 2.0, 3.0])
def test_a_gap_in_the_plan_while_the_model_still_brakes_keeps_the_stop(gap_at):
  """Review 2026-11-03, route 0000002d t=316.8-318.5: the plan stayed over the stop speed for 1.75 s mid-approach with
  the model asking -2.4: the stop let go and was placed again later, on the crosswalk. Still braking is no go."""
  ms = ModelStopLine()
  x, v, locked_at, out = 30., 9., None, []
  for i in range(int(20. / DT_MDL)):
    t = i * DT_MDL
    to_stop = MODEL_STOP - x
    if ms.locked and locked_at is None:
      locked_at = t
    gap = locked_at is not None and locked_at + gap_at <= t < locked_at + gap_at + 1.75
    seen = [b - x for b in (LINE, CROSSWALK) if HOOD <= b - x] if i % 2 == 0 else None
    a = ms.update(v, v < 0.01, True, False, 0., None if gap else max(to_stop, 0.), seen, None, -2.4)
    if locked_at is not None and v > 0.5:
      assert ms.locked, t
    cmd = -v ** 2 / (2. * max(to_stop, 0.3))
    cmd = cmd if a is None else min(cmd, a)
    out.append((x, v, a, ms.should_stop))
    v = max(v + cmd * DT_MDL, 0.)
    x += v * DT_MDL
  assert locked_at is not None
  assert LINE - STOP_MARGIN - 1. <= stopped_at(out) <= LINE - STOP_MARGIN + 0.5, "before the line, not on the crosswalk"


def test_a_plan_with_no_stop_and_the_model_not_braking_is_still_a_go():
  ms = ModelStopLine()
  drive(ms, 12., 4.)
  assert ms.locked
  for _ in range(int(GO_TIME / DT_MDL) + 1):
    ms.update(8., False, True, False, 0., None, None, None, 0.5)
  assert not ms.locked


def test_just_after_the_gas_a_new_stop_needs_no_more_than_resume_max():
  """Released at 7.5 m/s with the line 11.5 m out: the stop would ask 2.7 on top of the model; the model's then."""
  for gas_until, placed in [(0.3, False), (None, True)]:
    ms = ModelStopLine()
    x, v = 40., 7.5
    for i in range(int(3. / DT_MDL)):
      t = i * DT_MDL
      seen = [b - x for b in (LINE, CROSSWALK) if HOOD <= b - x] if i % 2 == 0 else None
      pressed = gas_until is not None and t < gas_until
      ms.update(v, False, True, pressed, 0., MODEL_STOP - x, seen, None, -2.0)
      x += v * DT_MDL
      if 60. - x < 11.5:
        break
    assert ms.locked == placed or (placed and ms.done is False), (gas_until, ms.locked)


@pytest.mark.parametrize("v0", [0.9, 1.5, 3.0])
def test_a_green_light_in_the_last_roll_lets_go_though_the_models_accel_lags(v0):
  """Review 2026-11-03: the model's accel lags a launch (route 0000002d: -0.5 to -0.7 for up to 2.4 s with the plan
  going); its plan speeding up past the car's is the go."""
  ms = ModelStopLine()
  ms.locked, ms.dist, ms.anchor = True, 2. * v0, 2. * v0
  for _ in range(int(GO_TIME / DT_MDL) + 1):
    ms.update(v0, False, True, False, 0., None, None, None, -0.62, v0 + 1.5)
  assert not ms.locked


def test_slowing_with_a_gap_in_the_plan_is_no_go():
  ms = ModelStopLine()
  ms.locked, ms.dist, ms.anchor = True, 15., 15.
  for _ in range(int(2. / DT_MDL)):
    ms.update(6., False, True, False, 0., None, None, None, -2.4, 3.)
  assert ms.locked
