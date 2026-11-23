"""Route 00000003--941020f467 t=74.8: coming out of a left turn the model's plan was briefly short, the stop-light
detector read it as a stop and Force Stop braked 12->10 mph with nothing to stop for."""
from dataclasses import replace

from openpilot.starpilot.longitudinal.force_stop import POST_TURN_HOLDOFF_NS, ForceStop, StopFrame, vehicle_tune

BASE = 100_000_000_000
TICK = 50_000_000
TUNE = vehicle_tune('MAZDA_CX5_2022')
HOLDOFF_TICKS = POST_TURN_HOLDOFF_NS // TICK


def frame(tick, **changes):
  base = StopFrame(BASE + tick * TICK, 5.4, 25., True, True, False, False, True, False,
                   False, False, 200., 30., False, 0., False, 0.)
  return replace(base, **changes)


def drive(owner, ticks, start=0, turn_ticks=0, **changes):
  plans = []
  for tick in range(start, start + ticks):
    steering = 250. if tick < turn_ticks else 0.
    plans.append(owner.step(frame(tick, steering_angle_deg=steering, **changes), TUNE))
  return plans


def first_forcing(plans):
  return next((i for i, p in enumerate(plans) if p.forcing), None)


def test_without_a_turn_the_model_path_still_forces():
  assert first_forcing(drive(ForceStop(), 40)) is not None


def test_route_74s_post_turn_plan_dip_neither_approaches_nor_forces():
  plans = drive(ForceStop(), 10 + HOLDOFF_TICKS - 1, turn_ticks=10)
  after_turn = plans[10:]
  assert not any(p.forcing for p in after_turn)
  assert all(p.speed_ceiling_mps is None and p.approach_distance_m == 0. for p in after_turn)


def test_holdoff_ends_and_a_real_stop_still_forces():
  owner = ForceStop()
  drive(owner, 10, turn_ticks=10)
  plans = drive(owner, HOLDOFF_TICKS + 40, start=10)
  idx = first_forcing(plans)
  assert idx is not None and idx >= HOLDOFF_TICKS - 1


def test_model_should_stop_is_never_held_off():
  owner = ForceStop()
  drive(owner, 10, turn_ticks=10, model_should_stop=True)
  plans = drive(owner, 20, start=10, model_should_stop=True)
  assert first_forcing(plans) is not None and first_forcing(plans) < HOLDOFF_TICKS


def test_a_stop_already_forcing_continues_through_a_turn():
  owner = ForceStop()
  plans = drive(owner, 40)
  assert plans[-1].forcing
  plans = drive(owner, 10, start=40, turn_ticks=50)
  assert all(p.forcing for p in plans)
