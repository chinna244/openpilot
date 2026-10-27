from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowReasonFilter, slow_reason

# capnp enum values, pinned: longitudinalPlan cruise 0 lead0 1 lead1 2 lead2 3 e2e 4; SP cruise 0 sccVision 1 sccMap 2 SLA 3


class TestSlowReason:
  def test_mpc_lead(self):
    for src in (1, 2, 3):
      assert slow_reason(src, 0, False, 0., 0., 20.) == 'lead'

  def test_model_alone(self):
    assert slow_reason(4, 0, False, 0., 0., 20.) == 'model'

  def test_model_behind_close_slower_lead(self):
    assert slow_reason(4, 0, True, 30., 15., 20.) == 'lead'
    # within 3 s of travel above 40 m
    assert slow_reason(4, 0, True, 59., 15., 20.) == 'lead'

  def test_model_lead_far_or_faster(self):
    assert slow_reason(4, 0, True, 61., 15., 20.) == 'model'
    assert slow_reason(4, 0, True, 41., 15., 10.) == 'model'
    assert slow_reason(4, 0, True, 30., 21.5, 20.) == 'model'

  def test_cruise_targets(self):
    assert slow_reason(0, 1, False, 0., 0., 20.) == 'curve'
    assert slow_reason(0, 2, False, 0., 0., 20.) == 'map'
    assert slow_reason(0, 3, False, 0., 0., 20.) == 'limit'
    assert slow_reason(0, 0, False, 0., 0., 20.) == 'cruise'

  def test_sp_source_ignored_when_lead_brakes(self):
    assert slow_reason(1, 2, False, 0., 0., 20.) == 'lead'


def run(f, frames, t0=0.):
  out = []
  for i, (active, braking, reason) in enumerate(frames):
    out.append(f.update(t0 + i * 0.05, active, braking, reason))
  return out


class TestSlowReasonFilter:
  def test_shows_after_one_second(self):
    out = run(SlowReasonFilter(), [(True, True, 'lead')] * 22)
    assert out[19] is None  # 0.95 s
    assert out[20] == 'lead'  # 1.0 s

  def test_short_brake_never_shows(self):
    f = SlowReasonFilter()
    out = run(f, [(True, True, 'lead')] * 19 + [(True, False, 'lead')] + [(True, True, 'lead')] * 19)
    assert all(o is None for o in out)

  def test_inactive_never_shows(self):
    assert all(o is None for o in run(SlowReasonFilter(), [(False, True, 'lead')] * 40))

  def test_hides_two_and_a_half_seconds_after_braking(self):
    f = SlowReasonFilter()
    run(f, [(True, True, 'lead')] * 21)  # last braking at 1.0 s
    assert f.update(1.0 + 2.45, True, False, 'lead') == 'lead'
    assert f.update(1.0 + 2.5, True, False, 'lead') is None

  def test_hides_at_once_when_inactive(self):
    f = SlowReasonFilter()
    run(f, [(True, True, 'lead')] * 21)
    assert f.update(1.05, False, False, 'lead') is None

  def test_reason_held_two_seconds(self):
    f = SlowReasonFilter()
    run(f, [(True, True, 'lead')] * 21)  # shown at 1.0 s
    assert f.update(2.95, True, True, 'model') == 'lead'
    assert f.update(3.0, True, True, 'model') == 'model'

  def test_reason_not_changed_by_non_braking_frames(self):
    f = SlowReasonFilter()
    run(f, [(True, True, 'lead')] * 21)
    assert f.update(3.0, True, False, 'curve') == 'lead'

  def test_stays_while_braking(self):
    f = SlowReasonFilter()
    assert all(o == 'lead' for o in run(f, [(True, True, 'lead')] * 100)[20:])  # 5 s of braking, no blink


class TestSlowReasonEdges:
  def test_lead_floor_forty_metres(self):
    assert slow_reason(4, 0, True, 35., 5., 10.) == 'lead'

  def test_lead_slightly_faster_still_lead(self):
    assert slow_reason(4, 0, True, 30., 20.5, 20.) == 'lead'


class TestBannerText:
  def test_lead_distance(self):
    from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowBanner
    t = SlowBanner._text_for
    assert t('lead', True, 36.6, 0., False) == 'Car ahead 120 ft'
    assert t('lead', True, 36.6, 0., True) == 'Car ahead 37 m'
    assert t('lead', True, 37.5, 0., False) == 'Car ahead 120 ft'  # 123 ft, to 10 ft

  def test_lead_lost_or_bad(self):
    from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowBanner
    t = SlowBanner._text_for
    assert t('lead', False, 36.6, 0., False) == 'Car ahead'
    assert t('lead', True, 0., 0., False) == 'Car ahead'
    assert t('lead', True, float('nan'), 0., False) == 'Car ahead'

  def test_speed_limit(self):
    from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowBanner
    t = SlowBanner._text_for
    assert t('limit', False, 0., 15.6464, False) == 'Speed limit 35'
    assert t('limit', False, 0., 15.6464, True) == 'Speed limit 56'
    assert t('limit', False, 0., 0., False) == 'Speed limit'
    assert t('limit', False, 0., 0.3, False) == 'Speed limit'  # 0.7 mph
    assert t('limit', False, 0., float('nan'), False) == 'Speed limit'

  def test_other_labels(self):
    from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowBanner
    t = SlowBanner._text_for
    assert [t(r, False, 0., 0., False) for r in ('model', 'curve', 'map', 'cruise')] == ['Slowing for road', 'Curve ahead', 'Map curve', 'Slowing']
