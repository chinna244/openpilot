"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Regression tests for the SP MICI settings widgets. These cover the param<->display contracts
# that are easy to break silently: nothing renders differently when a scaling factor or a value
# map is wrong, the setting just quietly writes the wrong number.

import os

import pytest

os.environ["BIG"] = "0"
os.environ.setdefault("SCALE", "1")


@pytest.fixture(scope="module")
def gui():
  """Hidden raylib window + isolated params dir. Widgets need textures, so a window is required."""
  import pyray as rl
  from openpilot.common.prefix import OpenpilotPrefix

  with OpenpilotPrefix():
    rl.set_config_flags(rl.FLAG_WINDOW_HIDDEN)
    from openpilot.system.ui.lib.application import gui_app
    gui_app.init_window("test_mici_settings", fps=30)
    yield gui_app
    gui_app.close()


@pytest.fixture
def params(gui):
  from openpilot.common.params import Params
  from openpilot.selfdrive.ui.ui_state import ui_state

  p = Params()
  ui_state.params = p
  return p


def render(widget):
  """Drive one frame the way gui_app does — Widget.render() is what calls _update_state()."""
  import pyray as rl
  widget.render(rl.Rectangle(0, 0, 800, 600))


def wait_for_param(params, key, timeout=2.0):
  """Widgets write with a non-blocking put(), which lands on a background thread."""
  import time
  deadline = time.monotonic() + timeout
  last = params.get(key)
  while time.monotonic() < deadline:
    time.sleep(0.005)
    val = params.get(key)
    if val != last:
      return val
    last = val
  return last


class TestFloatParamScaling:
  """Float params store the physical value; the picker works in an x100 integer domain.

  Getting this wrong is a 100x error in a steering gain, and nothing about the UI looks broken:
  the label just reads 0.02 instead of 2.5. Mirrors OptionControlSP.use_float_scaling on TICI.
  """

  @pytest.mark.parametrize(("param", "stored", "expected_ui"), [
    ("TorqueParamsOverrideLatAccelFactor", 2.5, 250),   # params_keys.h default
    ("TorqueParamsOverrideLatAccelFactor", 1.0, 100),
    ("TorqueParamsOverrideFriction", 0.1, 10),          # params_keys.h default
  ])
  def test_reads_scaled_up(self, params, param, stored, expected_ui):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamOption

    params.put(param, stored, block=True)
    opt = BigParamOption("t", param, min_value=1, max_value=500, float_param=True,
                         label_callback=lambda x: f"{x / 100}")
    assert opt._read_value() == expected_ui
    assert opt.value == str(stored)  # label divides back down

  def test_writes_scaled_down(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.number_picker import NumberPickerScreen

    param = "TorqueParamsOverrideLatAccelFactor"
    params.put(param, 1.0, block=True)
    picker = NumberPickerScreen(title="t", param=param, min_value=1, max_value=500, float_param=True)
    idx = next(i for i, item in enumerate(picker._picker_items) if item.raw_value == 250)
    picker._center_index = lambda: idx
    picker._commit_value()
    assert wait_for_param(params, param) == pytest.approx(2.5), "picker must divide by 100 on write"

  def test_round_trip_is_lossless(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.number_picker import NumberPickerScreen

    param = "TorqueParamsOverrideFriction"
    for physical in (0.05, 0.1, 0.5, 1.0):
      params.put(param, 0.99, block=True)  # something else, so the write below is observable
      picker = NumberPickerScreen(title="t", param=param, min_value=1, max_value=100, float_param=True)
      target = int(physical * 100)
      idx = next(i for i, item in enumerate(picker._picker_items) if item.raw_value == target)
      picker._center_index = lambda i=idx: i
      picker._commit_value()
      assert wait_for_param(params, param) == pytest.approx(physical), f"{physical} did not survive"

      # and the value we just wrote reads back as the same picker position
      assert NumberPickerScreen(title="t", param=param, min_value=1, max_value=100,
                                float_param=True)._read_value() == target

  def test_int_params_are_not_scaled(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamOption

    params.put("BlinkerMinLateralControlSpeed", 25, block=True)
    opt = BigParamOption("t", "BlinkerMinLateralControlSpeed", min_value=0, max_value=255)
    assert opt._read_value() == 25


class TestMultiParamValueMapping:
  """BigMultiParamToggleSP stores the option index by default, or a mapped value with `values=`."""

  def test_alc_modes_map_to_stored_value_not_index(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import ALC_LABELS
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigMultiParamToggleSP

    w = BigMultiParamToggleSP("t", "AutoLaneChangeTimer", list(ALC_LABELS.values()), values=list(ALC_LABELS))
    for mode, label in ALC_LABELS.items():
      params.put("AutoLaneChangeTimer", mode, block=True)
      w.refresh()
      assert w.value == label, f"mode {mode} showed {w.value}"

  def test_unset_resolves_to_declared_default(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import ALC_LABELS
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigMultiParamToggleSP
    from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode

    # AutoLaneChangeTimer defaults to "0" (nudge) in params_keys.h — an unset param must not
    # read as "off" (-1), which is a different index AND a different stored value
    params.remove("AutoLaneChangeTimer")
    w = BigMultiParamToggleSP("t", "AutoLaneChangeTimer", list(ALC_LABELS.values()), values=list(ALC_LABELS))
    assert w.value == ALC_LABELS[AutoLaneChangeMode.NUDGE]

  def test_tap_writes_mapped_value_and_wraps(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import ALC_LABELS
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigMultiParamToggleSP
    from openpilot.system.ui.lib.application import MousePos
    from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode

    modes = list(ALC_LABELS)
    params.put("AutoLaneChangeTimer", modes[-1], block=True)
    w = BigMultiParamToggleSP("t", "AutoLaneChangeTimer", list(ALC_LABELS.values()), values=modes)
    w.refresh()
    w._handle_mouse_release(MousePos(0, 0))
    # wraps to the first option, and stores -1 (the mode) rather than 0 (the index)
    assert w.value == ALC_LABELS[AutoLaneChangeMode.OFF]
    assert params.get("AutoLaneChangeTimer") == AutoLaneChangeMode.OFF

  def test_torque_tune_unset_is_v0(self, params):
    """params_keys.h declares 0.0 (v0); controlsd_ext reads it with return_default, so the
    selector must agree. If these drift, the UI claims a tune the car isn't running."""
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigMultiParamToggleSP

    versions = SteeringLayoutMici._load_torque_versions()
    assert list(versions.values()) == sorted(versions.values()), "must be oldest-first"

    params.remove("TorqueControlTune")
    w = BigMultiParamToggleSP("t", "TorqueControlTune", list(versions), values=list(versions.values()))
    assert versions[w.value] == pytest.approx(0.0)

    for label, version in versions.items():
      params.put("TorqueControlTune", version, block=True)
      w.refresh()
      assert w.value == label


class TestDependentSettings:
  """A setting whose parent makes it inert must read off without losing the user's value."""

  def test_reads_off_while_dependency_unmet_but_keeps_param(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamControlSP

    applies = True
    params.put_bool("AutoLaneChangeBsmDelay", True, block=True)
    w = BigParamControlSP("t", "AutoLaneChangeBsmDelay", depends_on=lambda: applies)

    w.refresh()
    assert w._checked and w.enabled

    applies = False
    w.refresh()
    assert not w._checked, "must display off while inert"
    assert not w.enabled, "must not accept input, or the forced-off display gets written back"
    assert params.get_bool("AutoLaneChangeBsmDelay"), "user's choice must survive"

    applies = True
    w.refresh()
    assert w._checked, "setting must come back when the dependency is met again"

  def test_no_dependency_behaves_like_upstream(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamControlSP

    params.put_bool("AutoLaneChangeBsmDelay", True, block=True)
    w = BigParamControlSP("t", "AutoLaneChangeBsmDelay")
    w.refresh()
    assert w._checked and w.enabled


class TestSubPanelSelfRefresh:
  """gui_app renders only the top 2 nav-stack widgets, so a layout cannot drive a sub-panel
  nested under another sub-panel. Panels refresh themselves instead."""

  def test_rendering_the_panel_refreshes_its_items(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamControlSP, SubPanelSP

    params.put_bool("AutoLaneChangeBsmDelay", False, block=True)
    toggle = BigParamControlSP("t", "AutoLaneChangeBsmDelay")
    panel = SubPanelSP([toggle])
    assert not toggle._checked

    # an external writer (sunnylink, another panel) changes the param
    params.put_bool("AutoLaneChangeBsmDelay", True, block=True)
    render(panel)
    assert toggle._checked, "panel must pick up param changes without a parent driving it"

  def test_nested_panel_still_gates_itself(self, params):
    """The depth-3 case: self-tune sub-panel under the torque sub-panel."""
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici

    params.put_bool("LiveTorqueParamsToggle", False, block=True)
    params.put_bool("SpeedDependentTorqueToggle", True, block=True)
    layout = SteeringLayoutMici()

    render(layout._tq_self_tune_view)
    assert not layout._tq_speed_dep._checked, "inert child must read off"
    assert not layout._tq_speed_dep.enabled
    assert params.get_bool("SpeedDependentTorqueToggle"), "and must keep its value"

    params.put_bool("LiveTorqueParamsToggle", True, block=True)
    render(layout._tq_self_tune_view)
    assert layout._tq_speed_dep._checked, "comes back when self-tune returns"


class TestSteeringLayoutBadges:
  def test_bsm_badge_hidden_when_auto_lane_change_cannot_feed_it(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici
    from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode

    layout = SteeringLayoutMici()
    params.put_bool("AutoLaneChangeBsmDelay", True, block=True)

    for mode, expected in [(AutoLaneChangeMode.NUDGELESS, True), (AutoLaneChangeMode.NUDGE, False),
                           (AutoLaneChangeMode.OFF, False), (AutoLaneChangeMode.THREE_SECONDS, True)]:
      params.put("AutoLaneChangeTimer", mode, block=True)
      layout._update_state()
      shown = "bsm-delay" in (layout._lane_change_btn._badge_labels or [])
      assert shown is expected, f"mode {mode}: badge shown={shown}"
      assert params.get_bool("AutoLaneChangeBsmDelay"), "badge suppression must not clear the param"


class TestJerkAwareToggle:
  """LateralJerkTorqueController and NNLC are mutually exclusive (ui_state and the car interface
  both force-disable the pair); the layout must gate the toggles the same way or a tap on one
  while the other is on re-creates the conflict and gets both silently wiped at the next init."""

  def test_jerk_aware_locked_while_nnlc_on(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici

    params.put_bool("NeuralNetworkLateralControl", True, block=True)
    layout = SteeringLayoutMici()
    render(layout._tq_view)
    assert not layout._jerk_aware_toggle.enabled

    params.put_bool("NeuralNetworkLateralControl", False, block=True)
    render(layout._tq_view)
    assert layout._jerk_aware_toggle.enabled

  def test_nnlc_locked_while_jerk_aware_on(self, params):
    from opendbc.car.structs import car
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici
    from openpilot.selfdrive.ui.ui_state import ui_state

    class _CP:
      steerControlType = car.CarParams.SteerControlType.torque
      enableBsm = False

    old_cp = ui_state.CP
    ui_state.CP = _CP()
    try:
      params.put_bool("LateralJerkTorqueController", True, block=True)
      layout = SteeringLayoutMici()
      layout._update_state()
      assert not layout._nnlc_toggle.enabled

      params.put_bool("LateralJerkTorqueController", False, block=True)
      layout._update_state()
      assert layout._nnlc_toggle.enabled
    finally:
      ui_state.CP = old_cp

  def test_torque_button_reflects_jerk_aware_without_enforce(self, params):
    """Jerk-aware works without EnforceTorqueControl, so the entry button must not read
    'disabled' while it is the only thing on."""
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici

    layout = SteeringLayoutMici()
    params.put_bool("EnforceTorqueControl", False, block=True)
    params.put_bool("LateralJerkTorqueController", True, block=True)
    layout._update_state()
    assert "jerk-aware" in (layout._torque_settings_btn._badge_labels or [])
    assert not layout._torque_settings_btn._disabled

    params.put_bool("LateralJerkTorqueController", False, block=True)
    layout._update_state()
    assert layout._torque_settings_btn._disabled
