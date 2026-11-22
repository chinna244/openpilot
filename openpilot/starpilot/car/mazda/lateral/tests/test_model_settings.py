"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

develop's test_model_settings.py for the port's ModelSettings (Blendful and big model smoothing;
camera offset and the steer delay stay StarPilot's), plus the modeld join the port runs them from.
"""
import ast
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import pytest

from openpilot.cereal import log
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value
from openpilot.starpilot.car.mazda.lateral import model_settings
from openpilot.starpilot.car.mazda.lateral.model_settings import BIG_MODEL_LAT_SMOOTH_SECONDS, PARAMS_FRAMES, ModelSettings
from openpilot.starpilot.car.mazda.lateral.tests.test_blendful import frame as lane_frame
from openpilot.starpilot.car.subaru.smoothing import filter_action

MODELD = Path(__file__).resolve().parents[5] / "selfdrive" / "modeld" / "modeld.py"


def _params(values: dict):
  params = mock.Mock()
  params.get_bool.side_effect = lambda k: bool(values.get(k, False))
  return params


class TestModelSettings:
  def test_refresh_reads_params_every_params_frames(self):
    params = _params({"Blendful": True, "BigModelSmoothing": True})
    settings = ModelSettings(params)
    assert settings.refresh()
    assert (settings.blendful_enabled, settings.big_model_smoothing) == (True, True)
    params.get_bool.side_effect = lambda k: False
    for _ in range(PARAMS_FRAMES - 1):
      assert not settings.refresh()
    assert settings.blendful_enabled
    assert settings.refresh()
    assert not settings.blendful_enabled and not settings.big_model_smoothing

  def test_blendful_is_inactive_during_a_lane_change(self):
    settings = ModelSettings(_params({"Blendful": True}))
    settings.blendful.update = mock.Mock(return_value=0.0)
    for desire, lane_change, active in ((log.Desire.none, log.LaneChangeState.off, True),
                                        (log.Desire.laneChangeLeft, log.LaneChangeState.laneChangeStarting, False)):
      action = log.ModelDataV2.Action(desiredCurvature=2 ** -10)  # exact in Float32
      settings.apply(action, log.ModelDataV2.Action(), {}, SimpleNamespace(chestnut=False), 20.0, desire, lane_change)
      settings.blendful.update.assert_called_with({}, 20.0, active, 2 ** -10, enabled=True)

  def test_big_model_smoothing_only_for_a_big_model_that_smooths_none(self):
    settings = ModelSettings()
    big, small = SimpleNamespace(chestnut=True, LAT_SMOOTH_SECONDS=0.0), SimpleNamespace(chestnut=False, LAT_SMOOTH_SECONDS=0.0)
    assert settings.big_lat_smooth(big) == 0.0, "off until the param is read"
    settings.big_model_smoothing = True
    assert settings.big_lat_smooth(big) == BIG_MODEL_LAT_SMOOTH_SECONDS
    assert settings.big_lat_smooth(SimpleNamespace(chestnut=True)) == BIG_MODEL_LAT_SMOOTH_SECONDS, "jetlink's stock face may lack the attribute"
    assert settings.big_lat_smooth(small) == 0.0
    assert settings.big_lat_smooth(SimpleNamespace(chestnut=True, LAT_SMOOTH_SECONDS=0.2)) == 0.0, "a bundle's own smoothing is kept"

  def test_smooth_curvature_matches_a_bundles_smoothing(self):
    prev = log.ModelDataV2.Action(desiredCurvature=0.0)
    action = log.ModelDataV2.Action(desiredCurvature=0.01)
    ModelSettings.smooth_curvature(action, prev, 0.1)
    assert action.desiredCurvature == pytest.approx(smooth_value(0.01, 0.0, 0.1))
    assert 0.0 < action.desiredCurvature < 0.01

  def test_smooth_curvature_keeps_a_held_curvature(self):
    prev = log.ModelDataV2.Action(desiredCurvature=0.004)
    action = log.ModelDataV2.Action(desiredCurvature=0.004)
    ModelSettings.smooth_curvature(action, prev, 0.1)
    assert action.desiredCurvature == pytest.approx(0.004)

  def test_develops_order_blendful_then_smoothing(self):
    """develop: action_from_model adds Blendful's correction (above 0.3 m/s), then
    smooth_curvature runs against prev_action."""
    settings = ModelSettings(_params({"Blendful": True, "BigModelSmoothing": True}))
    settings.blendful.update = mock.Mock(return_value=0.002)
    prev = log.ModelDataV2.Action(desiredCurvature=0.0)
    action = log.ModelDataV2.Action(desiredCurvature=0.001)
    settings.apply(action, prev, {}, SimpleNamespace(chestnut=True), 20.0, log.Desire.none, log.LaneChangeState.off)
    assert action.desiredCurvature == pytest.approx(smooth_value(0.003, 0.0, BIG_MODEL_LAT_SMOOTH_SECONDS))
    action = log.ModelDataV2.Action(desiredCurvature=0.001)
    settings.apply(action, prev, {}, SimpleNamespace(chestnut=True), 0.2, log.Desire.none, log.LaneChangeState.off)
    assert action.desiredCurvature == pytest.approx(smooth_value(0.001, 0.0, BIG_MODEL_LAT_SMOOTH_SECONDS)), "no correction below 0.3 m/s"


def _modeld_join(CP, model, model_output, action, prev_action, v_ego, desire=log.Desire.none):
  """modeld's main-loop names at its filter_action call (the Mazda's taus are 0)."""
  DH = SimpleNamespace(lane_change_state=log.LaneChangeState.off)  # noqa: F841 - read from this frame
  filter_action(action, prev_action, v_ego, 0.)
  return action


class TestModeldJoin:
  @pytest.fixture(autouse=True)
  def fresh(self, monkeypatch):
    monkeypatch.setattr(model_settings, "_settings", None)
    monkeypatch.setattr(model_settings, "Params", lambda: _params({"Blendful": True, "BigModelSmoothing": True}))

  def test_mazda_big_model_matches_develops_steps(self):
    CP = SimpleNamespace(brand='mazda')
    out = lane_frame(center=0.4)
    prev = log.ModelDataV2.Action(desiredCurvature=0.0)
    got = [_modeld_join(CP, SimpleNamespace(chestnut=True), out, log.ModelDataV2.Action(desiredCurvature=0.001), prev, 20.0).desiredCurvature
           for _ in range(40)]
    assert got[-1] != pytest.approx(0.001, abs=1e-6)
    want = ModelSettings(_params({"Blendful": True, "BigModelSmoothing": True}))
    for g in got:
      action = log.ModelDataV2.Action(desiredCurvature=0.001)
      want.apply(action, prev, out, SimpleNamespace(chestnut=True), 20.0, log.Desire.none, log.LaneChangeState.off)
      assert g == action.desiredCurvature

  @pytest.mark.parametrize("brand", ["toyota", "subaru"])
  def test_other_brands_untouched(self, brand):
    action = _modeld_join(SimpleNamespace(brand=brand), SimpleNamespace(chestnut=True), lane_frame(center=0.4),
                          log.ModelDataV2.Action(desiredCurvature=0.001), log.ModelDataV2.Action(), 20.0)
    assert action.desiredCurvature == pytest.approx(0.001)

  def test_a_caller_without_modelds_names_is_untouched(self):
    action = log.ModelDataV2.Action(desiredCurvature=0.001)
    filter_action(action, log.ModelDataV2.Action(), 20.0, 0.)
    assert action.desiredCurvature == pytest.approx(0.001)

  def test_a_failing_step_never_raises_and_keeps_the_models_curvature(self):
    action = _modeld_join(SimpleNamespace(brand='mazda'), SimpleNamespace(chestnut=True), {},
                          log.ModelDataV2.Action(desiredCurvature=0.001), log.ModelDataV2.Action(), 20.0)  # no lane heads
    assert action.desiredCurvature == pytest.approx(0.001)

  def test_modeld_names_still_exist(self):
    """The join reads these names from modeld's main(); a rename there must fail here, not
    silently drop Blendful and the smoothing on the car."""
    main = next(n for n in ast.walk(ast.parse(MODELD.read_text())) if isinstance(n, ast.FunctionDef) and n.name == 'main')
    assigned = {t.id for n in ast.walk(main) if isinstance(n, ast.Assign) for t in n.targets if isinstance(t, ast.Name)}
    assert {'CP', 'model', 'model_output', 'DH', 'desire', 'v_ego'} <= assigned

  def test_modeld_joins_between_the_action_and_prev_action(self):
    """develop's step runs on the model's action before it becomes prev_action and the message."""
    src = MODELD.read_text().splitlines()
    call = next(i for i, line in enumerate(src) if line.strip().startswith('filter_action(action, prev_action, v_ego'))
    built = max(i for i, line in enumerate(src[:call]) if 'action = model.get_action_from_model(' in line)
    handed = next(i for i, line in enumerate(src[call:], call) if line.strip() == 'prev_action = action')
    assert built < call < handed == call + 1
