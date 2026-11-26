"""
learn_min_speed: bins centered below it keep their seed. They take no points, never fit and
never restore a cached value; the bins above it learn and restore as before.
"""
import numpy as np
import pytest

from openpilot.selfdrive.locationd.torqued import TorqueEstimator, VERSION, POINTS_PER_BUCKET, STEER_BUCKET_BOUNDS
from openpilot.sunnypilot.selfdrive.locationd.speed_bin_learner import LIVE_TORQUE_PARAMETERS_SP_SERVICE
from openpilot.sunnypilot.selfdrive.locationd.tests.speed_dep_helpers import (
  SPEED_DEP_CARS, FakePubMaster, make_cache, make_cache_sp, make_cp, seed_values,
)

CAR = 'MAZDA_CX5_2022'
pytestmark = pytest.mark.skipif('learn_min_speed' not in SPEED_DEP_CARS.get(CAR, {}), reason="no learn_min_speed entry")


def _fill(bucket, laf, rng):
  """A full, valid bucket of points on lat accel = laf * steer."""
  for lo, hi in STEER_BUCKET_BOUNDS:
    steer = rng.uniform(lo, hi, POINTS_PER_BUCKET)
    bucket.load_points(np.c_[steer, laf * steer + rng.normal(0.0, 0.02, len(steer))].tolist())
  assert bucket.is_valid()


def _frozen_and_learning(est):
  frozen = [i for i, f in enumerate(est.speed_bin_frozen) if f]
  learning = [i for i, f in enumerate(est.speed_bin_frozen) if not f]
  return frozen, learning


class TestLearnMinSpeed:
  def test_bins_below_learn_min_speed_are_frozen(self, fake_params, real_speed_dep_config):
    est = TorqueEstimator(make_cp(fingerprint=CAR))
    learn_min_speed = SPEED_DEP_CARS[CAR]['learn_min_speed']
    assert est.speed_bin_frozen == [c < learn_min_speed for c in est.speed_bin_centers]
    frozen, learning = _frozen_and_learning(est)
    assert frozen and learning

  def test_entry_without_it_learns_every_bin(self, fake_params):
    # every_bin_learns strips the key, as for an entry that never set it
    est = TorqueEstimator(make_cp(fingerprint=CAR))
    assert not any(est.speed_bin_frozen)

  def test_frozen_bin_takes_no_points(self, fake_params, real_speed_dep_config):
    est = TorqueEstimator(make_cp(fingerprint=CAR))
    frozen, learning = _frozen_and_learning(est)
    for lo, hi in est.speed_bin_bounds:
      est._on_torque_point(0.1, 0.3, (lo + hi) / 2)
    assert all(len(est.speed_bin_points[i]) == 0 for i in frozen)
    assert all(len(est.speed_bin_points[i]) == 1 for i in learning)

  def test_frozen_bin_never_fits(self, fake_params, real_speed_dep_config):
    est = TorqueEstimator(make_cp(fingerprint=CAR))
    seeds = seed_values(est)
    frozen, learning = _frozen_and_learning(est)
    rng = np.random.default_rng(0)
    for i in (frozen[0], learning[0]):
      _fill(est.speed_bin_points[i], 1.0, rng)
    results = dict(est._estimate_params_speed_binned())
    assert results[frozen[0]] is False
    assert est.speed_bin_learned[frozen[0]] is False
    assert est.speed_bin_filtered[frozen[0]]['latAccelFactor'].x == seeds[0][frozen[0]]
    assert est.speed_bin_filtered[frozen[0]]['frictionCoefficient'].x == seeds[1][frozen[0]]
    assert results[learning[0]] is True
    assert est.speed_bin_learned[learning[0]] is True
    assert est.speed_bin_filtered[learning[0]]['latAccelFactor'].x != seeds[0][learning[0]]

  def test_restore_keeps_seed_below_and_learned_values_above(self, fake_params, real_speed_dep_config):
    est = TorqueEstimator(make_cp(fingerprint=CAR))
    seeds = seed_values(est)
    frozen, learning = _frozen_and_learning(est)
    n = len(est.speed_bin_centers)
    # the CX-5 2022 cache of 2026-11-21: 0.87/0.203 learned at 3.5 m/s
    lafs = [0.87, 1.74, 1.83, 1.62, 1.58, 1.95, 2.23, 2.53, 2.31][:n]
    frictions = [0.203, 0.176, 0.142, 0.123, 0.117, 0.111, 0.106, 0.095, 0.085][:n]
    lafs = [float(np.float32(x)) for x in lafs]
    frictions = [float(np.float32(x)) for x in frictions]
    points = [[[0.11, 0.3]] * 2 for _ in range(n)]
    cache_sp = make_cache_sp(est.speed_bin_centers, lafs, frictions, points=points, seed_version=est.speed_dep_seed_version,
                             learned=[True] * n, decays=[120.0] * n)
    est._restore_ext_cache(make_cache(decay=120.0, version=VERSION).lateralTorqueParameters, cache_CP=est.CP,
                           cache_sp=getattr(cache_sp, LIVE_TORQUE_PARAMETERS_SP_SERVICE))
    for i in frozen:
      assert est.speed_bin_filtered[i]['latAccelFactor'].x == seeds[0][i]
      assert est.speed_bin_filtered[i]['frictionCoefficient'].x == seeds[1][i]
      assert est.speed_bin_learned[i] is False
      assert len(est.speed_bin_points[i]) == 0
    for i in learning:
      assert est.speed_bin_filtered[i]['latAccelFactor'].x == lafs[i]
      assert est.speed_bin_filtered[i]['frictionCoefficient'].x == frictions[i]
      assert est.speed_bin_learned[i] is True
      assert len(est.speed_bin_points[i]) == 2
      assert est.speed_bin_decays[i] == 120.0

  def test_published_and_cache_round_trip_keep_the_seed(self, fake_params, real_speed_dep_config):
    """A drifted cache restored, published, written and restored again on the next boot: the
    frozen bins read their seed throughout, the learning bins their learned values."""
    CP = make_cp(fingerprint=CAR)
    est = TorqueEstimator(CP)
    seeds = seed_values(est)
    frozen, learning = _frozen_and_learning(est)
    n = len(est.speed_bin_centers)
    lafs = [float(np.float32(x)) for x in [0.87, 1.74, 1.83, 1.62, 1.58, 1.95, 2.23, 2.53, 2.31][:n]]
    frictions = [float(np.float32(x)) for x in [0.203, 0.176, 0.142, 0.123, 0.117, 0.111, 0.106, 0.095, 0.085][:n]]
    cache_sp = make_cache_sp(est.speed_bin_centers, lafs, frictions, points=[[[0.11, 0.3]] * 2 for _ in range(n)],
                             seed_version=est.speed_dep_seed_version, learned=[True] * n, decays=[120.0] * n)
    est._restore_ext_cache(make_cache(decay=120.0, version=VERSION).lateralTorqueParameters, cache_CP=CP,
                           cache_sp=getattr(cache_sp, LIVE_TORQUE_PARAMETERS_SP_SERVICE))

    est._pm = FakePubMaster()
    cache = est.get_msg(with_points=True).to_bytes()
    live = est._pm.last()
    for i in frozen:
      assert live.speedBinLatAccelFactors[i] == pytest.approx(seeds[0][i])
      assert live.speedBinFrictions[i] == pytest.approx(seeds[1][i])
    for i in learning:
      assert live.speedBinLatAccelFactors[i] == pytest.approx(lafs[i])

    fake_params.put("LiveTorqueParameters", cache)
    fake_params.put("CarParamsPrevRoute", CP.to_bytes())
    nxt = TorqueEstimator(CP)
    for i in frozen:
      assert nxt.speed_bin_filtered[i]['latAccelFactor'].x == pytest.approx(seeds[0][i])
      assert nxt.speed_bin_learned[i] is False
      assert len(nxt.speed_bin_points[i]) == 0
    for i in learning:
      assert nxt.speed_bin_filtered[i]['latAccelFactor'].x == pytest.approx(lafs[i])
      assert nxt.speed_bin_learned[i] is True
