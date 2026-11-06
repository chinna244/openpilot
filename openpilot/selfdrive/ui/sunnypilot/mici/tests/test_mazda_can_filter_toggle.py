"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# The Smart CAN Filter toggle in Cruise settings: shown only for a CX-5 2022 with alpha long on,
# locked onroad, and saved to MazdaCanFilter on both device UIs.

import os

import pytest

os.environ["BIG"] = "0"
os.environ.setdefault("SCALE", "1")

from openpilot.selfdrive.ui.sunnypilot.tests.helpers import render

CX5_2022 = {"brand": "mazda", "platform": "MAZDA_CX5_2022", "name": "Mazda CX-5 2022"}
CX9_2021 = {"brand": "mazda", "platform": "MAZDA_CX9_2021", "name": "Mazda CX-9 2021"}


def _mici():
  from openpilot.selfdrive.ui.sunnypilot.mici.layouts.cruise import CruiseLayoutMici
  layout = CruiseLayoutMici()
  return layout, lambda: layout._can_filter_toggle


def _tici():
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.cruise import CruiseLayout
  layout = CruiseLayout()
  return layout, lambda: layout.can_filter_toggle


@pytest.fixture(autouse=True)
def offroad_no_car():
  from openpilot.selfdrive.ui.ui_state import ui_state
  ui_state.CP, ui_state.started = None, False
  yield
  ui_state.started = False


def _setup(params, bundle, alpha_long):
  # blocking: the default put lands on a background thread, and the next render would race it
  params.put("CarPlatformBundle", bundle, block=True)
  params.put_bool("AlphaLongitudinalEnabled", alpha_long, block=True)
  # the prefix lives for the module: start every case from the key's default
  params.remove("MazdaCanFilter")


@pytest.mark.parametrize("make", [_mici, _tici])
class TestMazdaCanFilterToggle:
  @pytest.mark.parametrize("bundle, alpha_long, shown", [
    (CX5_2022, True, True),
    (CX5_2022, False, False),
    (CX9_2021, True, False),
    ({"brand": "toyota", "platform": "TOYOTA_RAV4_TSS2", "name": "X"}, True, False),
  ])
  def test_visibility(self, params, make, bundle, alpha_long, shown):
    _setup(params, bundle, alpha_long)
    layout, toggle = make()
    render(layout)
    assert toggle().is_visible == shown

  def test_default_off_and_locked_onroad(self, params, make):
    from openpilot.selfdrive.ui.ui_state import ui_state
    _setup(params, CX5_2022, True)
    assert not params.get_bool("MazdaCanFilter")
    layout, toggle = make()

    def enabled():
      # the list row's lock sits on its action item, as on the TJA toggle
      return toggle().enabled if make is _mici else toggle().action_item.enabled

    render(layout)
    assert enabled()
    ui_state.started = True
    render(layout)
    assert not enabled()

  def test_saves_and_reloads(self, params, make):
    from openpilot.system.ui.lib.application import MousePos
    from openpilot.selfdrive.ui.sunnypilot.mici.tests.test_mici_settings import wait_for_param
    _setup(params, CX5_2022, True)
    layout, toggle = make()
    render(layout)
    widget = toggle() if make is _mici else toggle().action_item.toggle
    widget._handle_mouse_release(MousePos(0, 0))
    assert wait_for_param(params, "MazdaCanFilter") is True
    # a fresh panel, as after a restart, reads the saved value back
    layout, toggle = make()
    render(layout)
    reloaded = toggle()._checked if make is _mici else toggle().action_item.toggle.get_state()
    assert reloaded
