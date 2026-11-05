import itertools
from openpilot.common.test import OpenpilotTestCase
from openpilot.common.parameterized import parameterized_class

from openpilot.cereal import log

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import get_safe_obstacle_distance, get_stopped_equivalence_factor, get_T_FOLLOW, \
                                                                       get_city_T_FOLLOW, CITY_T_FOLLOW_BP, CITY_T_FOLLOW_FACTOR
from openpilot.selfdrive.test.longitudinal_maneuvers.maneuver import Maneuver


def desired_follow_distance(v_ego, v_lead, t_follow=None):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return get_safe_obstacle_distance(v_ego, t_follow) - get_stopped_equivalence_factor(v_lead)

def run_following_distance_simulation(v_lead, t_end=100.0, personality=0):
  man = Maneuver(
    '',
    duration=t_end,
    initial_speed=float(v_lead),
    lead_relevancy=True,
    initial_distance_lead=100,
    speed_lead_values=[v_lead],
    breakpoints=[0.],
    personality=personality,
  )
  valid, output = man.evaluate()
  assert valid
  return output[-1,2] - output[-1,1]


@parameterized_class(("personality", "speed"), itertools.product(
                      [log.LongitudinalPersonality.relaxed, # personality
                       log.LongitudinalPersonality.standard,
                       log.LongitudinalPersonality.aggressive],
                      [0,10,13.4,35])) # speed
class TestFollowingDistance(OpenpilotTestCase):
  def test_following_distance(self):
    v_lead = float(self.speed)
    simulation_steady_state = run_following_distance_simulation(v_lead, personality=self.personality)
    correct_steady_state = desired_follow_distance(v_lead, v_lead, get_city_T_FOLLOW(get_T_FOLLOW(self.personality), v_lead))
    abs_err_margin = 0.5 if v_lead > 0.0 else 1.15
    self.assertAlmostEqual(simulation_steady_state, correct_steady_state,
                           delta=0.2 * correct_steady_state + abs_err_margin)


class TestCityFollowingDistance(OpenpilotTestCase):
  def test_city_t_follow(self):
    for personality in (log.LongitudinalPersonality.relaxed, log.LongitudinalPersonality.standard, log.LongitudinalPersonality.aggressive):
      t = get_T_FOLLOW(personality)
      for v in (0., 5., CITY_T_FOLLOW_BP[0]):
        self.assertAlmostEqual(get_city_T_FOLLOW(t, v), CITY_T_FOLLOW_FACTOR * t)
      self.assertAlmostEqual(get_city_T_FOLLOW(t, sum(CITY_T_FOLLOW_BP) / 2), (1 + CITY_T_FOLLOW_FACTOR) / 2 * t)
      for v in (CITY_T_FOLLOW_BP[1], 25., 35.):
        self.assertAlmostEqual(get_city_T_FOLLOW(t, v), t)
    self.assertAlmostEqual(get_city_T_FOLLOW(get_T_FOLLOW(log.LongitudinalPersonality.aggressive), 10.), 1.0)

  def test_city_gap_closer_than_full_gap(self):
    # aggressive at 22 mph settles at the city gap, not the full 1.25 s one
    v = 10.
    simulation_steady_state = run_following_distance_simulation(v, personality=log.LongitudinalPersonality.aggressive)
    city = desired_follow_distance(v, v, 1.0)
    full = desired_follow_distance(v, v, get_T_FOLLOW(log.LongitudinalPersonality.aggressive))
    self.assertLess(abs(simulation_steady_state - city), abs(simulation_steady_state - full))
    self.assertAlmostEqual(simulation_steady_state, city, delta=1.0)
