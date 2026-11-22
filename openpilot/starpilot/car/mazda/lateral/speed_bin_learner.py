"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ported from zoompilot develop 197598c0f6 (sunnypilot/selfdrive/locationd/speed_bin_learner.py).
Port differences, both forced by StarPilot's frozen cereal (tools/ci/schema_policy.py: log.capnp stays
upstream's and every custom Event slot is taken): the per-bin values ride on the published
lateralTorqueParameters' points list as 4-wide rows (speed_bin_rows / bins_from_points; torqued's own
points are 2-wide and only ever on its cache copy), at develop's cadence and validity; and the bins'
cache (LiveTorqueParametersSP) is a JSON document with develop's liveTorqueParametersSP fields
(SpeedBinsCache). torqued's own caches are read through StarPilot's schema_cache.
"""
import numpy as np

import json
from types import SimpleNamespace

from opendbc.car.structs import car
from openpilot.cereal import log

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog

# Speed bins for cars without a speed_dependent.toml entry
DEFAULT_SPEED_BIN_BOUNDS = [(5, 8), (8, 12), (12, 18), (18, 24), (24, 29), (29, 35), (35, 40)]
DEFAULT_SPEED_BIN_CENTERS = [6.5, 10.0, 15.0, 21.0, 26.5, 32.0, 37.5]

# torqued's 60 s cache write serializes the bins, plus the per-bin point buckets, decays and learned
# flags, into LiveTorqueParametersSP; the buckets are thousands of points and only the restore path
# reads them, so the wire copy carries only the per-bin values.
LIVE_TORQUE_PARAMETERS_SP_KEY = "LiveTorqueParametersSP"
SPEED_BIN_ROW_WIDTH = 4  # [center, latAccelFactor, friction, valid]; torqued's own points are [steer, lat accel]


def speed_bin_rows(centers, lat_factors, frictions, valid_flags):
  return [[float(c), float(f), float(fr), 1.0 if v else 0.0] for c, f, fr, v in zip(centers, lat_factors, frictions, valid_flags, strict=True)]


def bins_from_points(points):
  """The bins torqued put on a published lateralTorqueParameters, as develop's liveTorqueParametersSP
  fields, or None when it carries none (develop: no bins)."""
  rows = [list(row) for row in points if len(row) == SPEED_BIN_ROW_WIDTH]
  if not rows:
    return None
  return SimpleNamespace(speedBinCenters=[r[0] for r in rows], speedBinLatAccelFactors=[r[1] for r in rows],
                         speedBinFrictions=[r[2] for r in rows], speedBinValid=[r[3] > 0.5 for r in rows])


class SpeedBinsCache:
  """develop's LiveTorqueParametersSP cache struct (liveTorqueParametersSP with the buckets), as JSON."""
  FIELDS = ('version', 'seedVersion', 'speedBinCenters', 'speedBinLatAccelFactors', 'speedBinFrictions', 'speedBinValid',
            'speedBinPoints', 'speedBinDecays', 'speedBinLearned')

  def __init__(self, **values):
    self.version = 0
    self.seedVersion = 0
    for name in self.FIELDS[2:]:
      setattr(self, name, [])
    for name, value in values.items():
      if name not in self.FIELDS:
        raise KeyError(name)
      setattr(self, name, value)

  def to_bytes(self) -> bytes:
    return json.dumps({name: getattr(self, name) for name in self.FIELDS}, separators=(',', ':')).encode()

  @classmethod
  def from_bytes(cls, raw: bytes) -> 'SpeedBinsCache':
    data = json.loads(raw)
    if not isinstance(data, dict):
      raise ValueError("speed-bin cache is not an object")
    return cls(**{name: data[name] for name in cls.FIELDS if name in data})


def _torqued():
  # torqued imports this module through torqued_ext, so its names resolve at call time
  from openpilot.selfdrive.locationd import torqued
  return torqued


def _get_cache(params, key):
  from openpilot.starpilot.schema_cache import get_cache
  return get_cache(params, key)


def fit_torque_points(points):
  """Upstream's estimate_params on one point set ([steer, 1, lateral accel] rows): the TLS slope
  (latAccelFactor), its offset and the spread friction. Raises LinAlgError as the SVD does."""
  torqued = _torqued()
  _, _, v = np.linalg.svd(points, full_matrices=False)
  slope, offset = -v.T[0:2, 2] / v.T[2, 2]
  _, spread = np.matmul(points[:, [0, 2]], torqued.slope2rot(slope)).T
  return slope, offset, np.std(spread) * torqued.FRICTION_FACTOR


class SpeedBinLearner:
  """Per-speed-bin torque learning, mixed into TorqueEstimator through TorqueEstimatorExt.

  Each bin runs upstream's total-least-squares fit on the quality-filtered points that fall
  in its speed range and publishes its own latAccelFactor and friction; the controller
  interpolates them by speed. Bins come from speed_dependent.toml, or the defaults above
  seeded with the car's global offline values. Runs wherever self-tune does.
  """
  speed_binned = False  # set by initialize_custom_params, once torqued has set use_params

  @staticmethod
  def _centers_to_bounds(centers, min_speed=None):
    """Bin bounds at the midpoints between consecutive centers; the outer edges take min_speed
    (the table's own, or the one the config sets when it drops bins below a steering floor) or
    the default range's 5 m/s, and the default range's 40 m/s."""
    bounds = []
    for i, c in enumerate(centers):
      lo = (DEFAULT_SPEED_BIN_BOUNDS[0][0] if min_speed is None else min_speed) if i == 0 else (centers[i - 1] + c) / 2
      hi = DEFAULT_SPEED_BIN_BOUNDS[-1][1] if i == len(centers) - 1 else (c + centers[i + 1]) / 2
      bounds.append((lo, hi))
    return bounds

  def _init_speed_bins(self):
    """Builds the per-bin buckets, filters and sanity bounds, then restores the cache. Runs
    after factor_sanity and the offline values are set, before the first get_msg."""
    if not self.speed_binned:
      return

    from openpilot.starpilot.car.mazda.lateral.lateral_tune import get_speed_dep_config_for_car

    cfg = get_speed_dep_config_for_car(self.CP)
    # the TOML entry's seed_version, bumped with a seed refresh to retire every cache learned
    # under the old seeds; 0 for an entry without one and for the defaults
    self.speed_dep_seed_version = int(cfg.get('seed_version', 0))

    if 'speed_bp' in cfg:
      self.speed_bin_centers = list(cfg['speed_bp'])
      self.speed_bin_bounds = self._centers_to_bounds(self.speed_bin_centers, cfg.get('min_speed'))
    else:
      self.speed_bin_bounds = list(DEFAULT_SPEED_BIN_BOUNDS)
      self.speed_bin_centers = list(DEFAULT_SPEED_BIN_CENTERS)

    n_bins = len(self.speed_bin_bounds)

    self.speed_bin_points = [self._make_speed_bin_bucket() for _ in range(n_bins)]
    # a bin with points since its last fit: a full bin's length stops changing while its ring
    # buffers keep turning over, so the length cannot mark new data
    self._speed_bin_dirty = [True] * n_bins

    # seeds from the TOML entry, else the global offline values for every bin
    ref_lafs = cfg.get('laf_bp', [self.offline_latAccelFactor] * n_bins)
    ref_frictions = cfg.get('friction_bp', [self.offline_friction] * n_bins)
    self.speed_bin_decays = [_torqued().MIN_FILTER_DECAY] * n_bins
    # a bin fit since its last reseed: what the restore keys on, since a learned bin can drop
    # below its point minimums (the minimum rose, or the points cache failed) and keep its value
    self.speed_bin_learned = [False] * n_bins
    self.speed_bin_filtered = [
      {'latAccelFactor': FirstOrderFilter(ref_lafs[i], self.speed_bin_decays[i], DT_MDL),
       'frictionCoefficient': FirstOrderFilter(ref_frictions[i], self.speed_bin_decays[i], DT_MDL)}
      for i in range(n_bins)
    ]
    # the fits are clipped to +-sanity of the seed, as upstream clips its global fit
    self.speed_bin_lat_accel_factor_bounds = [
      ((1.0 - self.factor_sanity) * factor, (1.0 + self.factor_sanity) * factor)
      for factor in ref_lafs
    ]
    self.speed_bin_friction_bounds = [
      ((1.0 - self.friction_sanity) * f, (1.0 + self.friction_sanity) * f)
      for f in ref_frictions
    ]
    self._restore_ext_cache()

  def _make_speed_bin_bucket(self):
    """One speed bin's buckets. Each bin sees a fraction of the data, so the per-bucket
    minimums are the global learner's divided by the bin count. The total is a full fit sample
    (fit_points), so a bin is not fit on less than upstream's fit draws: points come at 20 Hz
    and neighbours are near duplicates, so the bucket minimums alone are a curve or two."""
    torqued = _torqued()
    # min_bucket_points is a plain list upstream; coerce before the integer divide
    scaled_min = np.maximum(np.asarray(self.min_bucket_points) // len(self.speed_bin_bounds), 1)
    return torqued.TorqueBuckets(x_bounds=torqued.STEER_BUCKET_BOUNDS,
                                 min_points=scaled_min,
                                 min_points_total=max(int(scaled_min.sum()), int(self.fit_points)),
                                 points_per_bucket=torqued.POINTS_PER_BUCKET,
                                 rowsize=3)

  def _on_torque_point(self, steer, lateral_acc, vego):
    """Routes a quality-filtered point from handle_log to its speed bin."""
    if not self.speed_binned:
      return
    for i, (lo, hi) in enumerate(self.speed_bin_bounds):
      if lo <= vego < hi:
        self.speed_bin_points[i].add_point(steer, lateral_acc)
        self._speed_bin_dirty[i] = True
        break

  @staticmethod
  def _within_bounds(vals, bounds):
    """Per bin, whether its value is finite and inside its (lo, hi) clip range. The wire is
    Float32, so a filter sitting on a bound can read back a rounding step past it."""
    vals = np.asarray(vals, dtype=float)
    lo, hi = np.asarray(bounds, dtype=float).T
    tol = 1e-5 * np.maximum(np.abs(hi), 1.0)
    return np.isfinite(vals) & (vals >= lo - tol) & (vals <= hi + tol)

  def _centers_match(self, cached_centers) -> bool:
    # a legacy cache has no centers; length first so allclose never sees a shape mismatch
    cached = list(cached_centers)
    return len(cached) == len(self.speed_bin_centers) and bool(np.allclose(cached, self.speed_bin_centers, atol=0.01))

  def _restore_ext_cache(self, cache_ltp=None, cache_CP=None, cache_sp=None):
    """Restores the per-bin filters, decay and point buckets from the two caches: upstream's
    LiveTorqueParameters supplies the restore key (and the decay of a cache written before the
    per-bin decays); the fork's LiveTorqueParametersSP supplies its own VERSION, the seed
    version, the bin centers, the values, the per-bin learned flags and decays, and the points.
    Both must carry this car's restore key (fingerprint, tuning type, offline seeds, VERSION)
    and the fork cache this config's seed_version and bin centers. Each bin is judged on its
    own, not on upstream's global valid flag (which needs data above 15 m/s): its value and
    decay come back only when it was written learned (a cache from before the flag: written
    valid), so a bin that never learned takes its current seed, and when its value and decay
    are finite and its value inside its clip range (bounds that moved under it: a seed refresh
    that kept seed_version, or less-restrict switched off). Any other bin starts at its seed at
    MIN decay. Points are restored on top when they pass their own checks, and are simply
    skipped when they fail them. Reads from Params for whichever argument is None."""
    if not self.speed_binned:
      return
    try:
      if cache_ltp is None:
        cache = _get_cache(self._params, "LiveTorqueParameters")
        if not cache:
          return
        with log.Event.from_bytes(cache) as evt:
          cache_ltp = evt.lateralTorqueParameters
      if cache_CP is None:
        params_cache = _get_cache(self._params, "CarParamsPrevRoute")
        if not params_cache:
          cloudlog.info("speed-dep: no CarParamsPrevRoute, restarting learning")
          return
        with car.CarParams.from_bytes(params_cache) as msg:
          cache_CP = msg
      torqued = _torqued()
      restore_key = torqued.TorqueEstimator.get_restore_key

      if restore_key(cache_CP, cache_ltp.version) != restore_key(self.CP, torqued.VERSION):
        cloudlog.info("speed-dep: cache restore key mismatch, restarting learning")
        return
      if cache_sp is None:
        cache = self._params.get(LIVE_TORQUE_PARAMETERS_SP_KEY)
        if not cache:
          cloudlog.info("speed-dep: no LiveTorqueParametersSP cache, restarting learning")
          return
        cache_sp = SpeedBinsCache.from_bytes(cache)
      if restore_key(cache_CP, cache_sp.version) != restore_key(self.CP, torqued.VERSION):
        cloudlog.info("speed-dep: fork cache restore key mismatch, restarting learning")
        return
      if cache_sp.seedVersion != self.speed_dep_seed_version:
        cloudlog.info(f"speed-dep: seed version {cache_sp.seedVersion} -> {self.speed_dep_seed_version}, restarting learning")
        return
      n_bins = len(self.speed_bin_bounds)
      if not self._centers_match(cache_sp.speedBinCenters):
        cloudlog.info("speed-dep: config changed, restarting learning")
        return
      cached_lafs = list(cache_sp.speedBinLatAccelFactors)
      cached_frictions = list(cache_sp.speedBinFrictions)
      if len(cached_lafs) != n_bins or len(cached_frictions) != n_bins:
        cloudlog.info("speed-dep: cache bin count mismatch, restarting learning")
        return
      # per-bin flags and decays; a cache from before them has the valid flags and upstream's one decay
      learned = list(cache_sp.speedBinLearned)
      if len(learned) != n_bins:
        learned = list(cache_sp.speedBinValid)
      learned = np.array(learned, dtype=bool) if len(learned) == n_bins else np.zeros(n_bins, dtype=bool)
      decays = np.asarray(list(cache_sp.speedBinDecays), dtype=float)
      if len(decays) != n_bins:
        decays = np.full(n_bins, float(cache_ltp.decay))

      bin_ok = (self._within_bounds(cached_lafs, self.speed_bin_lat_accel_factor_bounds)
                & self._within_bounds(cached_frictions, self.speed_bin_friction_bounds)
                & np.isfinite(decays))
      restored = learned & bin_ok
      if (learned & ~bin_ok).any():
        cloudlog.warning(f"speed-dep: cached bins {np.flatnonzero(learned & ~bin_ok).tolist()} non-finite or outside sanity bounds, reseeding them")
      # torqued only writes a decay inside this range; the clip guards a hand-edited cache
      decays = np.clip(np.nan_to_num(decays, nan=torqued.MIN_FILTER_DECAY), torqued.MIN_FILTER_DECAY, torqued.MAX_FILTER_DECAY)
      self.speed_bin_learned = restored.tolist()
      for i in np.flatnonzero(restored):
        self.speed_bin_filtered[i]['latAccelFactor'].x = cached_lafs[i]
        self.speed_bin_filtered[i]['frictionCoefficient'].x = cached_frictions[i]
      cached_points = self._load_points_cache(cache_sp, n_bins)
      if cached_points is not None:
        for i in range(n_bins):
          self.speed_bin_points[i].load_points(cached_points[i])
      # a bin resumes at its own decay; one on its seed learns from MIN like a fresh one
      self.speed_bin_decays = [float(d) if ok else torqued.MIN_FILTER_DECAY for d, ok in zip(decays, restored, strict=True)]
      for filters, bin_decay in zip(self.speed_bin_filtered, self.speed_bin_decays, strict=True):
        filters['latAccelFactor'].update_alpha(bin_decay)
        filters['frictionCoefficient'].update_alpha(bin_decay)
      cloudlog.info(f"restored speed-bin torque params from cache: bins {np.flatnonzero(restored).tolist()} learned")
    except Exception:
      cloudlog.exception("speed-dep: failed to restore cache")

  def _load_points_cache(self, cache_sp, n_bins):
    """The per-bin points from the fork cache, or None when they fail the bin-count or
    finiteness checks (the restore key and bin centers were checked on the same struct
    already). A failure here never touches the values restore: the learner just starts its
    buckets empty."""
    try:
      points = [[list(point) for point in bin_points] for bin_points in cache_sp.speedBinPoints]
      if len(points) != n_bins:
        cloudlog.info("speed-dep: points cache bin count mismatch, points not restored")
        return None
      if not all(np.all(np.isfinite(np.asarray(bin_points, dtype=float))) for bin_points in points if bin_points):
        cloudlog.warning("speed-dep: cached bin points non-finite, points not restored")
        return None
      return points
    except Exception:
      cloudlog.exception("speed-dep: failed to read points cache, points not restored")
      return None

  def _sp_cache(self, values):
    """develop's cache copy of liveTorqueParametersSP: VERSION, the seed version, the bin centers,
    the per-bin values, decays and learned flags, and the point buckets."""
    lat_factors, frictions, valid_flags = values
    # Float32, as develop's capnp cache stores them, so a restore reads back the same values
    f32 = lambda xs: np.asarray(xs, dtype=np.float32).tolist()  # noqa: E731
    return SpeedBinsCache(version=_torqued().VERSION, seedVersion=self.speed_dep_seed_version,
                          speedBinCenters=f32(self.speed_bin_centers), speedBinLatAccelFactors=f32(lat_factors),
                          speedBinFrictions=f32(frictions), speedBinValid=[bool(v) for v in valid_flags],
                          speedBinDecays=f32(self.speed_bin_decays), speedBinLearned=[bool(v) for v in self.speed_bin_learned],
                          speedBinPoints=[f32(bucket.get_points()[:, [0, 2]]) for bucket in self.speed_bin_points])

  def _estimate_params_speed_binned(self):
    """Independent total-least-squares fit per bin, upstream's estimate_params() per bucket
    set. As upstream filters its global fit, a bin is fit and filtered only once it is valid,
    so until then it publishes its seed (or the value restored from its cache). A bin that
    goes NaN is reset, as upstream resets its global fit."""
    torqued = _torqued()

    results = []
    for i, bucket in enumerate(self.speed_bin_points):
      if not bucket.is_valid():
        results.append((i, False))
        continue

      # nothing new since the last fit
      if not self._speed_bin_dirty[i]:
        results.append((i, True))
        continue

      # self.fit_points honors the decimated (qlog) point count
      points = bucket.get_points(self.fit_points)
      try:
        slope, _, friction_coeff = fit_torque_points(points)  # slope = latAccelFactor
        if not any(np.isnan(val) for val in [slope, friction_coeff]):
          factor_lo, factor_hi = self.speed_bin_lat_accel_factor_bounds[i]
          fric_lo, fric_hi = self.speed_bin_friction_bounds[i]
          self.speed_bin_decays[i] = min(self.speed_bin_decays[i] + DT_MDL, torqued.MAX_FILTER_DECAY)  # slow down filter over time
          self.speed_bin_filtered[i]['latAccelFactor'].update(np.clip(slope, factor_lo, factor_hi))
          self.speed_bin_filtered[i]['latAccelFactor'].update_alpha(self.speed_bin_decays[i])
          self.speed_bin_filtered[i]['frictionCoefficient'].update(np.clip(friction_coeff, fric_lo, fric_hi))
          self.speed_bin_filtered[i]['frictionCoefficient'].update_alpha(self.speed_bin_decays[i])
          self.speed_bin_learned[i] = True
          self._speed_bin_dirty[i] = False
          results.append((i, True))
          continue
      except np.linalg.LinAlgError:
        pass

      cloudlog.warning(f"speed-dep: bin {i} produced NaN with valid data, resetting bin")
      self.speed_bin_points[i] = self._make_speed_bin_bucket()
      self.speed_bin_decays[i] = torqued.MIN_FILTER_DECAY
      self.speed_bin_learned[i] = False
      self._speed_bin_dirty[i] = True
      results.append((i, False))
    return results

  def _extend_msg(self, msg, with_points):
    """torqued's get_msg hook. Puts the bins on the published message (never their points), and on
    torqued's cache write (with_points, which carries torqued's own points instead) persists them with
    the point buckets under their own key."""
    values = None
    if self.speed_binned:
      bin_results = self._estimate_params_speed_binned()
      n_bins = len(self.speed_bin_bounds)
      lat_factors, frictions, valid_flags = [], [], []
      for i in range(n_bins):
        lat_factors.append(float(self.speed_bin_filtered[i]['latAccelFactor'].x))
        frictions.append(float(self.speed_bin_filtered[i]['frictionCoefficient'].x))
        valid_flags.append(bin_results[i][1])
      values = (lat_factors, frictions, valid_flags)

    if values is None:
      return
    if with_points:
      self._params.put(LIVE_TORQUE_PARAMETERS_SP_KEY, self._sp_cache(values).to_bytes())
    else:
      msg.lateralTorqueParameters.points = speed_bin_rows(self.speed_bin_centers, *values)
