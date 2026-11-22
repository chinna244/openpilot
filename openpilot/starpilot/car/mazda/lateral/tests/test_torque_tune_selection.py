"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Which torque controller an unset TorqueControlTune selects. This is easy to get wrong by
# dropping `return_default=True` from the params read: params_keys.h declares a default, but
# a bare params.get() returns None for an unset param, and float(None) raises, or, guarded,
# can otherwise fall through to the upstream controller without an explicit error.
# The small model's declared default is upstream's 0.0; the steer-to-zero Mazdas are seeded to
# 2.0 by seed_mazda_torque_defaults instead, so other brands never inherit a tune fitted to
# that EPS. A big model's is v1 for every brand.
#
# The constructors are patched out: these tests pin the branch that gets taken, not the
# controller's behavior. Port of develop's test_torque_tune_selection.py onto MazdaLateral; the
# UI selector and versions-file cases are dropped (the port has neither).

import re
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from opendbc.car.structs import car
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.starpilot.car.mazda.lateral import controls as controls_lateral_zp
from openpilot.starpilot.car.mazda.lateral.controls import TUNE_SWAP_INACTIVE_FRAMES, MazdaLateral

class FakeLaC(str):
  """A controller stand-in that compares as its version label and counts resets."""
  resets = 0

  def reset(self):
    self.resets += 1


V0 = "v0"
V1 = "v1"
V2 = "v2"
BY_VERSION = {0.0: V0, 1.0: V1, 2.0: V2}
PARAMS_KEYS_H = Path(__file__).resolve().parents[5] / "common" / "params_keys.h"


def declared_default(key: str = "TorqueControlTune") -> float:
  """The default params_keys.h declares, read from the header text so the test does not
  depend on the compiled params library being current."""
  m = re.search(r'\{"' + key + r'", \{[^}]*, "([0-9.]+)"\}\}', PARAMS_KEYS_H.read_text())
  assert m, f"{key} must declare a default in params_keys.h"
  return float(m.group(1))


def require_current_libparams(params, *keys: str) -> None:
  for key in keys:
    if float(params.get(key, return_default=True)) != declared_default(key):
      pytest.skip("libparams_c is stale against params_keys.h; rebuild with scons openpilot/common")


@pytest.fixture
def ctx(monkeypatch):
  monkeypatch.setattr(controls_lateral_zp, "LatControlTorqueV0", lambda *a, **k: FakeLaC(V0))
  monkeypatch.setattr(controls_lateral_zp, "LatControlTorqueV1", lambda *a, **k: FakeLaC(V1))
  monkeypatch.setattr(controls_lateral_zp, "LatControlTorqueV2", lambda *a, **k: FakeLaC(V2))
  with OpenpilotPrefix():
    params = Params()
    params.put_bool("EnforceTorqueControl", True, block=True)  # the enforce-off tests flip it
    CP = car.CarParams.new_message(steerControlType="torque")
    CP.lateralTuning.init('torque')
    controls = SimpleNamespace(params=params, CP=CP.as_reader(), ml=None, LaC=None)
    yield params, controls


def select(controls):
  """What controlsd does at startup: build and install the small model's controller."""
  controls.ml = MazdaLateral(controls.CP, MagicMock(), controls.params, 0.01)
  controls.ml._inactive_frames = TUNE_SWAP_INACTIVE_FRAMES
  controls.LaC = controls.ml.LaC
  return controls.LaC


def swap(controls, big: bool):
  controls.LaC = controls.ml.select_lateral_control({'modelV2': SimpleNamespace(big=big)})


class TestTorqueTuneSelection:
  def test_declared_defaults(self):
    """Upstream's v0 for the small model, so other brands never inherit the Mazda tune through
    the default (the Mazdas are seeded, test_torque_defaults_seed.py); v1 for a big one."""
    assert declared_default("TorqueControlTune") == 0.0
    assert declared_default("TorqueControlTuneBig") == 1.0

  def test_unset_selects_the_declared_default(self, ctx):
    """An unset param must resolve through params_keys.h (v0 today), not through None."""
    params, controls = ctx
    params.remove("TorqueControlTune")
    require_current_libparams(params, "TorqueControlTune")
    assert select(controls) == BY_VERSION[declared_default()]

  @pytest.mark.parametrize(("version", "expected"), [(0.0, V0), (1.0, V1), (2.0, V2)])
  def test_explicit_version_is_honored(self, ctx, version, expected):
    params, controls = ctx
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

  def test_unset_big_tune_selects_its_declared_default(self, ctx):
    """An unset TorqueControlTuneBig resolves through params_keys.h (v1), not through the
    small tune and not through None."""
    params, controls = ctx
    params.put("TorqueControlTune", 2.0, block=True)
    params.remove("TorqueControlTuneBig")
    require_current_libparams(params, "TorqueControlTuneBig")
    select(controls)
    swap(controls, big=True)
    assert controls.LaC == BY_VERSION[declared_default("TorqueControlTuneBig")]

  def test_same_tune_for_both_sizes_never_swaps(self, ctx):
    params, controls = ctx
    params.put("TorqueControlTune", 2.0, block=True)
    params.put("TorqueControlTuneBig", 2.0, block=True)
    select(controls)
    swap(controls, big=True)
    assert controls.LaC == V2 and controls.LaC.resets == 0

  @pytest.mark.parametrize(("small", "big"), [(2.0, 1.0), (0.0, 2.0), (1.0, 0.0)])
  def test_lateral_control_follows_the_model_size(self, ctx, small, big):
    """Every drive starts on the small model's tune; the frame after modelV2.big flips runs
    the other size's controller, reset once, and a repeated frame does not churn it."""
    params, controls = ctx
    params.put("TorqueControlTune", small, block=True)
    params.put("TorqueControlTuneBig", big, block=True)
    assert select(controls) == BY_VERSION[small]

    swap(controls, big=True)
    assert controls.LaC == BY_VERSION[big] and controls.LaC.resets == 1
    swap(controls, big=True)
    assert controls.LaC.resets == 1

    swap(controls, big=False)
    assert controls.LaC == BY_VERSION[small] and controls.LaC.resets == 1

  def test_a_hand_back_while_steering_keeps_the_tune_that_steers(self, ctx):
    """2026-09-29: every big-to-small hand-back while steering stepped the commanded torque
    by 0.09 to 0.19 in one tick, from switching to the idle controller's stale state. The
    controller that steers carries the small model until lateral has been inactive for
    TUNE_SWAP_INACTIVE_FRAMES in a row: a one-frame drop or a 0.3 s one keeps it."""
    params, controls = ctx
    params.put("TorqueControlTune", 2.0, block=True)
    params.put("TorqueControlTuneBig", 1.0, block=True)
    select(controls)
    swap(controls, big=True)            # swapped in with nothing in control
    big = controls.LaC

    def frame(lat_active):
      controls.ml.note_lat_active(lat_active)
      swap(controls, big=False)

    assert big == V1 and big.resets == 1
    for gap in (0, 1, 30, TUNE_SWAP_INACTIVE_FRAMES - 1):
      for _ in range(20):               # handed back while steering
        frame(True)
      for _ in range(gap):              # a steer fault flicker, a short blinker pause
        frame(False)
      frame(True)
      assert controls.LaC is big and big.resets == 1, f"swapped after a {gap}-frame gap"
    for _ in range(TUNE_SWAP_INACTIVE_FRAMES):   # half a second off: a disengage, a stop
      frame(False)
    assert controls.LaC == V2 and controls.LaC.resets == 1
    for _ in range(10):                 # and the next engagement is on the small tune
      frame(True)
    assert controls.LaC == V2 and controls.LaC.resets == 1

  def test_switching_while_inactive_is_unchanged(self, ctx):
    params, controls = ctx
    params.put("TorqueControlTune", 2.0, block=True)
    params.put("TorqueControlTuneBig", 1.0, block=True)
    select(controls)
    for big, tune in ((True, V1), (False, V2), (True, V1)):
      swap(controls, big=big)
      assert controls.LaC == tune

  def test_enforce_off_ignores_the_big_tune(self, ctx):
    """The enforce-off v0 forcing applies to both sizes."""
    params, controls = ctx
    params.put_bool("EnforceTorqueControl", False, block=True)
    params.put("TorqueControlTune", 1.0, block=True)
    params.put("TorqueControlTuneBig", 2.0, block=True)
    assert select(controls) == V0
    swap(controls, big=True)
    assert controls.LaC == V0 and controls.LaC.resets == 0
