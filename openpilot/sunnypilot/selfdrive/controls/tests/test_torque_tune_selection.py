"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

V1 = "v1"  # stands in for the `lac` upstream controller controlsd passes in
V2 = "v2"


@pytest.fixture
def ctx(monkeypatch):
  monkeypatch.setattr(controlsd_ext, "LatControlTorqueV0", lambda *a, **k: V0)
  monkeypatch.setattr(controlsd_ext, "LatControlTorqueV2", lambda *a, **k: V2)
  with OpenpilotPrefix():
    params = Params()
    CP = car.CarParams.new_message(steerControlType="torque")
    CP.lateralTuning.init('torque')
    controls = SimpleNamespace(params=params, CP=CP.as_reader(),
                               CP_SP=custom.CarParamsSP.new_message().as_reader())
    yield params, controls


def select(controls):
  return ControlsExt.initialize_lateral_control(controls, V1, MagicMock(), 0.01)


class TestTorqueTuneSelection:
  def test_unset_selects_v2(self, ctx):
    """The declared default in params_keys.h is 2.0 — an unset param must honor it, so a
    fresh install (and every car seeded into torque control) drives on the v2 tune."""
    params, controls = ctx
    params.put_bool("EnforceTorqueControl", True, block=True)
    params.remove("TorqueControlTune")
    assert select(controls) == V2

  @pytest.mark.parametrize(("version", "expected"), [(0.0, V0), (1.0, V1), (2.0, V2)])
  def test_explicit_version_is_honored(self, ctx, version, expected):
    params, controls = ctx
    params.put_bool("EnforceTorqueControl", True, block=True)
    params.put("TorqueControlTune", version, block=True)
    assert select(controls) == expected

  def test_every_declared_version_is_wired(self, ctx):
    """The versions file is what the UI selectors and the sunnylink schema offer, while
    initialize_lateral_control decides what is constructible. A version added to the file
    but not wired here would surface in every selector and silently run v1."""
    from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import load_versions

    wired = {0.0: V0, 1.0: V1, 2.0: V2}
    declared = {float(info["version"]) for info in load_versions().values()}
    assert declared == set(wired), "declared tune versions must match the wired controllers"

    params, controls = ctx
    params.put_bool("EnforceTorqueControl", True, block=True)
    for version, expected in wired.items():
      params.put("TorqueControlTune", version, block=True)
      assert select(controls) == expected

  @pytest.mark.parametrize("version", [1.0, 2.0])
  def test_torque_control_not_enforced_still_uses_v0_for_torque_cars(self, ctx, version):
    """Pre-existing behavior worth pinning: torque-tuned cars get v0 even with the toggle off.
    For 2.0 this is also the structural NNLC exclusion: enabling NNLC disables
    EnforceTorqueControl (ui_state/_cleanup_unsupported_params), so a stored v2 selection can
    never construct the v2 controller alongside NNLC."""
    params, controls = ctx
    params.put_bool("EnforceTorqueControl", False, block=True)
    params.put("TorqueControlTune", version, block=True)
    assert select(controls) == V0

  def test_ui_default_matches_what_controls_runs(self, ctx):
    """For an unset param the MICI selector lights up the declared default (the widget itself
    is pinned by test_torque_tune_unset_is_v2) — that version must be the one
    initialize_lateral_control picks, or the UI claims a tune the car isn't running."""
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici

    params, controls = ctx
    params.put_bool("EnforceTorqueControl", True, block=True)
    params.remove("TorqueControlTune")

    shown = float(params.get("TorqueControlTune", return_default=True))
    assert shown in set(SteeringLayoutMici._load_torque_versions().values()), \
      "the declared default must be a version the selectors offer"
    assert {0.0: V0, 1.0: V1, 2.0: V2}[shown] == select(controls)
