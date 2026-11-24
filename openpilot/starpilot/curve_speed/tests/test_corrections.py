import pytest

from openpilot.starpilot.curve_speed import corrections as C

V_TOWN, V_HWY = 12.0, 32.0


def frame(c, *, v=V_TOWN, lat=0.0, gas=False, brake=False, long_active=True, engaged=True, slowing=False, close_lead=False,
          table=2.8, n=1):
  for _ in range(n):
    c.step(dt=0.05, v_ego=v, lat_accel=lat, gas=gas, brake=brake, long_active=long_active, engaged=engaged, slowing=slowing,
           close_lead=close_lead, table_lat=table)


def fresh():
  c = C.Corrections()
  frame(c)  # pedals released: the next press is an edge
  return c


def gas_correction(c, *, v=V_TOWN, peak=3.2, slowing=True, long_active=True):
  frame(c, v=v, lat=1.5, gas=True, slowing=slowing, long_active=long_active)
  frame(c, v=v, lat=peak, gas=True, long_active=long_active, n=20)
  frame(c, v=v, lat=0.5, long_active=long_active)  # out of the curve, gas released: the watch ends


def test_hand_driven_curves_teach_nothing():
  c = C.Corrections()
  frame(c, long_active=False)
  gas_correction(c, long_active=False)
  frame(c, lat=2.0, brake=True, long_active=False)
  assert c.scales == (1.0, 1.0, 1.0) and not c.dirty


def test_gas_while_the_controller_slows_raises_that_speed():
  c = fresh()
  gas_correction(c)
  assert c.scales[0] == pytest.approx(1.0 + C.UP_FRAC * (3.2 / 2.8 - 1.0))
  assert c.scales[1:] == (1.0, 1.0) and c.dirty


def test_gas_with_the_controller_not_slowing_is_no_correction():
  c = fresh()
  gas_correction(c, slowing=False)
  assert c.scales == (1.0, 1.0, 1.0)


def test_gas_into_a_gentler_curve_than_the_table_does_not_lower():
  c = fresh()
  gas_correction(c, peak=2.0)
  assert c.scales == (1.0, 1.0, 1.0)


def test_brake_in_a_curve_lowers_by_speed():
  c = fresh()
  frame(c, v=V_HWY, lat=1.5, brake=True)
  assert c.scales == pytest.approx((1.0, 1.0, 1.0 - C.DOWN_STEP))


@pytest.mark.parametrize("kwargs", [{"close_lead": True}, {"lat": 0.5}, {"v": 3.0}])
def test_brake_for_a_lead_a_straight_or_a_crawl_is_no_correction(kwargs):
  c = fresh()
  frame(c, **{'lat': 1.5, 'brake': True, **kwargs})
  assert c.scales == (1.0, 1.0, 1.0)


def test_a_pedal_held_from_the_start_is_not_a_press():
  c = C.Corrections()
  frame(c, lat=1.5, brake=True)
  assert c.scales == (1.0, 1.0, 1.0)


def test_scales_stay_within_ten_percent():
  c = fresh()
  for _ in range(20):
    frame(c, lat=1.5, brake=True)
    frame(c)
  assert c.scales[0] == pytest.approx(0.9)
  c = fresh()
  for _ in range(40):
    gas_correction(c, peak=4.5)
  assert c.scales[0] == pytest.approx(1.1)


def test_speed_weights():
  assert C.speed_weights(5.0) == (1.0, 0.0, 0.0)
  assert C.speed_weights(40.0) == (0.0, 0.0, 1.0)
  assert C.speed_weights(20.1) == pytest.approx((0.5, 0.5, 0.0))


@pytest.mark.parametrize("doc", [None, "x", {"version": 2, "scales": [1.1, 1.1, 1.1]}, {"version": 1, "scales": [1.0]},
                                 {"version": 1, "scales": [1.0, "a", 1.0]}, {"version": 1, "scales": [1.0, float('nan'), 1.0]}])
def test_malformed_documents_are_no_correction(doc):
  assert C.load(doc) == (1.0, 1.0, 1.0)


def test_document_round_trip_and_clamp():
  c = fresh()
  gas_correction(c)
  assert C.load(c.document()) == pytest.approx(c.scales, abs=1e-4)
  assert C.load({"version": 1, "scales": [0.5, 1.0, 2.0]}) == (0.9, 1.0, 1.1)


def test_a_gas_press_that_already_released_control_still_counts():
  # the press frame can arrive with long control released and the plan handed back to cruise
  c = fresh()
  frame(c, lat=1.5, slowing=True)
  frame(c, lat=1.5, gas=True, long_active=False, slowing=False)
  frame(c, lat=3.2, gas=True, long_active=False, n=20)
  frame(c, lat=0.5, long_active=False)
  assert c.scales[0] > 1.0


def test_a_brake_press_that_already_disengaged_still_counts():
  c = fresh()
  frame(c, lat=1.6)
  frame(c, lat=1.6, brake=True, long_active=False, engaged=False)
  assert c.scales[0] == pytest.approx(1.0 - C.DOWN_STEP)


def test_control_two_frames_back_does_not_count():
  c = fresh()
  frame(c, lat=1.5, slowing=True)
  frame(c, lat=1.5, long_active=False)
  frame(c, lat=1.6, brake=True, long_active=False)
  assert c.scales == (1.0, 1.0, 1.0)


def test_disengaging_during_the_watch_teaches_nothing():
  c = fresh()
  frame(c, lat=1.5, gas=True, slowing=True)
  frame(c, lat=3.2, gas=True, n=5)
  frame(c, lat=3.4, gas=True, engaged=False, long_active=False, n=5)
  frame(c, lat=0.5, engaged=False, long_active=False)
  assert c.scales == (1.0, 1.0, 1.0)


def test_a_brake_ending_the_watch_does_not_raise():
  c = fresh()
  frame(c, lat=1.5, gas=True, slowing=True)
  frame(c, lat=3.2, gas=True, n=5)
  frame(c, lat=3.4, brake=True)
  frame(c, lat=0.5)  # the curve ends: a surviving watch would raise here
  assert c.scales[0] == pytest.approx(1.0 - C.DOWN_STEP)


def test_idle_drops_the_watch_and_a_held_pedal_is_not_a_press_later():
  c = fresh()
  frame(c, lat=1.5, gas=True, slowing=True)
  c.idle(gas=True, brake=True)
  frame(c, lat=3.2, gas=True, brake=True, slowing=True, n=20)
  frame(c, lat=0.5)
  assert c.scales == (1.0, 1.0, 1.0) and c.watch_left == 0.0


def test_huge_stored_numbers_are_no_correction():
  assert C.load('{"version": 1, "scales": [' + '9' * 401 + ', 1.0, 1.0]}') == (1.0, 1.0, 1.0)
  assert C.load({"version": 1, "scales": [10 ** 400, 1.0, 1.0]}) == (1.0, 1.0, 1.0)


def test_the_watch_ends_with_the_bend_the_press_was_for():
  c = fresh()
  frame(c, lat=1.5, gas=True, slowing=True)
  frame(c, lat=2.0, gas=True, n=5)
  frame(c, lat=0.3, gas=True, n=10)  # straight road, gas still held
  frame(c, lat=3.4, gas=True, n=10)  # a later, tighter bend
  frame(c, lat=0.3)
  assert c.scales == (1.0, 1.0, 1.0)  # only the first bend (2.0, gentler than the table) counted


def test_a_press_no_bend_follows_teaches_nothing():
  c = fresh()
  frame(c, lat=0.5, gas=True, slowing=True)
  frame(c, lat=0.5, gas=True, n=int(C.ENTER_S / 0.05) + 1)
  frame(c, lat=3.4, gas=True, n=10)
  frame(c, lat=0.3)
  assert c.scales == (1.0, 1.0, 1.0)


def test_a_press_at_a_bends_exit_does_not_carry_into_the_next_bend():
  c = fresh()
  frame(c, lat=1.5, slowing=True)
  frame(c, lat=1.2, gas=True, slowing=True)  # pressed on the bend's last frame
  frame(c, lat=0.3, gas=True, n=20)  # one second of straight road
  frame(c, lat=3.4, gas=True, n=10)  # a different bend
  frame(c, lat=0.3)
  assert c.scales == (1.0, 1.0, 1.0)
