from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowReasonFilter, slow_reason

# capnp enum values, pinned: longitudinalPlan cruise 0 lead0 1 lead1 2 lead2 3 e2e 4; SP cruise 0 sccVision 1 sccMap 2 SLA 3


class TestSlowReason:
  def test_mpc_lead(self):
    for src in (1, 2, 3):
      assert slow_reason(src, 0, False, 0., 0., 20.) == 'lead'

  def test_model_alone(self):
    assert slow_reason(4, 0, False, 0., 0., 20.) == 'model'

  def test_model_behind_close_slower_lead(self):
    assert slow_reason(4, 0, True, 30., 15., 20.) == 'model_lead'
    # within 3 s of travel above 40 m
    assert slow_reason(4, 0, True, 59., 15., 20.) == 'model_lead'

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
    assert slow_reason(4, 0, True, 35., 5., 10.) == 'model_lead'

  def test_lead_slightly_faster_still_lead(self):
    assert slow_reason(4, 0, True, 30., 20.5, 20.) == 'model_lead'


class TestBannerText:
  def test_lead_distance(self):
    from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowBanner
    t = SlowBanner._text_for
    assert t('lead', True, 36.6, 0., False) == 'Lead 120 ft'
    assert t('lead', True, 36.6, 0., True) == 'Lead 37 m'
    assert t('lead', True, 37.5, 0., False) == 'Lead 120 ft'  # 123 ft, to 10 ft

  def test_lead_lost_or_bad(self):
    from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowBanner
    t = SlowBanner._text_for
    assert t('lead', False, 36.6, 0., False) == 'Lead'
    assert t('lead', True, 0., 0., False) == 'Lead'
    assert t('lead', True, float('nan'), 0., False) == 'Lead'

  def test_speed_limit(self):
    from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowBanner
    t = SlowBanner._text_for
    assert t('limit', False, 0., 15.6464, False) == 'SLA 35'
    assert t('limit', False, 0., 15.6464, True) == 'SLA 56'
    assert t('limit', False, 0., 0., False) == 'SLA'
    assert t('limit', False, 0., 0.3, False) == 'SLA'  # 0.7 mph
    assert t('limit', False, 0., float('nan'), False) == 'SLA'

  def test_other_labels(self):
    from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowBanner
    t = SlowBanner._text_for
    assert [t(r, False, 0., 0., False) for r in ('model', 'curve', 'map', 'cruise')] == ['Model', 'SCC-V curve', 'SCC-M map curve', 'Set speed']


class TestModelReasons:
  def test_lead_closing_within_eight_seconds(self):
    # 70 m back, 10 m/s slower: 7 s
    assert slow_reason(4, 0, True, 70., 5., 15.) == 'model_lead'
    assert slow_reason(4, 0, True, 90., 5., 15.) == 'model'

  def test_blinker_is_turn(self):
    assert slow_reason(4, 0, False, 0., 0., 10., True) == 'turn'
    # before the plan check
    assert slow_reason(4, 0, False, 0., 0., 10., True, 1., 0.) == 'turn'

  def test_blinker_fast_is_not_turn(self):
    assert slow_reason(4, 0, False, 0., 0., 15.1, True) == 'model'
    assert slow_reason(4, 0, False, 0., 0., 14.9, True) == 'turn'

  def test_plan_slow_is_stop(self):
    assert slow_reason(4, 0, False, 0., 0., 10., False, 3.9, 8.) == 'stop'
    assert slow_reason(4, 0, False, 0., 0., 10., False, 4.1, 8.) == 'model'
    assert slow_reason(4, 0, False, 0., 0., 10., False, 8., 0.9) == 'stop'
    assert slow_reason(4, 0, False, 0., 0., 10., False, 8., 1.1) == 'model'

  def test_lead_before_turn(self):
    assert slow_reason(4, 0, True, 20., 5., 10., True, 1., 0.) == 'model_lead'

  def test_cruise_sources_ignore_model_inputs(self):
    assert slow_reason(0, 1, False, 0., 0., 10., True, 1., 0.) == 'curve'

  def test_labels(self):
    from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowBanner
    assert SlowBanner._text_for('stop', False, 0., 0., False) == 'Model: stop/turn'
    assert SlowBanner._text_for('turn', False, 0., 0., False) == 'Model: turning'


class TestPlacement:
  def test_top_by_alert_size(self):
    import pyray as rl
    from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowBanner
    rect = rl.Rectangle(0, 0, 476, 240)
    # alertSize none 0, small 1, mid 2, full 3; a small alert is 140 px tall on this screen
    assert SlowBanner._top(0, rect) == 12
    assert SlowBanner._top(1, rect) == 144
    assert SlowBanner._top(2, rect) is None
    assert SlowBanner._top(3, rect) is None

  def test_model_car_label(self):
    from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowBanner
    assert SlowBanner._text_for('model_lead', True, 45.7, 0., False) == 'Model: car 150 ft'
    assert SlowBanner._text_for('model_lead', False, 45.7, 0., False) == 'Model: car'

  def test_box_centered_unless_icon(self):
    import pyray as rl
    from openpilot.selfdrive.ui.mici.onroad.slow_banner import SlowBanner
    rect = rl.Rectangle(0, 0, 476, 240)
    assert SlowBanner._box_x(rect, 200) == 124  # (476 - 200) / 2 - 14
    assert SlowBanner._box_x(rect, 330) == 84  # would reach the icon: pushed right
