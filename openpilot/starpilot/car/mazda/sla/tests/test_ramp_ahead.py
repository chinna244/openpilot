"""An exit ramp's street limit, taken early as the limit ahead, holds to the street (AHEAD_TAKE_PAST_M)."""
from openpilot.common.realtime import DT_MDL
from openpilot.starpilot.car.mazda.sla.resolver import AHEAD_TAKE_PAST_M, Sources, SpeedLimitResolver

MPH = 0.44704
CP = object()  # openpilot longitudinal: the resolver brakes for a lower limit ahead at the coast budget


class Ramp:
  def __init__(self):
    self.r = SpeedLimitResolver(CP)
    self.ns = 1_000_000_000

  def step(self, v_mph, *, own=0., ahead=0., ahead_m=0., car=50., n=1):
    for _ in range(n):
      self.ns += int(DT_MDL * 1e9)
      s = Sources(now_ns=self.ns, car=car * MPH, map_limit=own * MPH, map_ahead=ahead * MPH, map_ahead_distance=ahead_m,
                  map_ns=self.ns, gps_fix=True, gps_ns=self.ns)
      self.r.update(v_mph * MPH, s)
    return round(self.r.speed_limit / MPH)


def on_ramp(car=50.):
  """Off a 50 freeway (car read 50) onto a ramp whose street is 30, 330 m on."""
  t = Ramp()
  assert t.step(57, own=50, car=car, n=80) == 50
  assert t.step(57, ahead=30, ahead_m=330, car=car) == 30
  return t


def test_street_limit_holds_through_a_blank_map_and_a_slowed_car():
  t = on_ramp()
  assert t.step(50, car=50, n=10) == 30  # the ramp lost for 0.5 s
  assert t.step(25, ahead=30, ahead_m=60, car=50, n=10) == 30  # slowed below the street limit
  assert t.step(20, own=30, n=20) == 30  # the street matched


def test_ends_on_a_new_car_reading():
  t = on_ramp()
  assert t.step(40, car=40, n=10) == 40


def test_ends_on_another_limit_ahead():
  t = on_ramp()
  t.step(45, ahead=35, ahead_m=900, n=2)  # too far to brake for yet: not taken, but the street ahead is not the one held
  assert t.r._ahead_take is None


def test_ends_past_the_ramp_end():
  t = on_ramp()
  metres = 330 + AHEAD_TAKE_PAST_M + 20
  t.step(20, n=int(metres / (20 * MPH) / DT_MDL))
  assert t.r._ahead_take is None  # what the map's blank gives from here is the dropout hold's, as without a limit ahead


def test_ends_without_gps():
  t = on_ramp()
  t.ns += int(DT_MDL * 1e9)
  t.r.update(40 * MPH, Sources(now_ns=t.ns, car=50 * MPH, map_ns=t.ns, gps_fix=False, gps_ns=t.ns))
  assert t.r._ahead_take is None


def test_town_street_with_no_limit_ahead_is_unchanged():
  t = Ramp()
  assert t.step(30, own=30, car=30, n=80) == 30
  assert t.step(30, car=30, n=40) == 30
