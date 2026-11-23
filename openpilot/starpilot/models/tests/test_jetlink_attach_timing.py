"""Startup diagnostic: jetlink's attach is timed (GPU bring-up, join, the rest) without changing what it returns."""
import sys
import types
from unittest import mock

import pytest

from openpilot.starpilot.models import jetlink_adapter


@pytest.fixture
def fake_jetlink(monkeypatch):
  pkg, op = types.ModuleType('jetlink'), types.ModuleType('jetlink.openpilot')
  warp, joining = types.ModuleType('jetlink.openpilot.warp'), types.ModuleType('jetlink.openpilot.joining')
  warp.init_device = lambda log: None
  joining.join = lambda *a, **k: 'joined'
  op.warp, op.joining, pkg.openpilot = warp, joining, op
  for name, module in (('jetlink', pkg), ('jetlink.openpilot', op), ('jetlink.openpilot.warp', warp),
                       ('jetlink.openpilot.joining', joining)):
    monkeypatch.setitem(sys.modules, name, module)
  return warp, joining


def real_like_attach(model, small, cam_w, cam_h):
  from jetlink.openpilot.warp import init_device
  from jetlink.openpilot.joining import join
  init_device(None)
  return join(None, cam_w, cam_h, small)


def test_attach_is_timed_and_unchanged(fake_jetlink):
  warp, joining = fake_jetlink
  original = warp.init_device, joining.join
  with mock.patch.object(jetlink_adapter, '_cloudlog') as log:
    assert jetlink_adapter._timed_attach(real_like_attach, 'm', 's', 1, 2) == 'joined'
  name, fields = log.return_value.event.call_args.args[0], log.return_value.event.call_args.kwargs
  assert name == 'jetlink_attach_timing' and {'total_s', 'other_s', 'gpu_init_s', 'join_s'} <= set(fields)
  assert (warp.init_device, joining.join) == original


def test_originals_restored_when_attach_raises(fake_jetlink):
  warp, joining = fake_jetlink
  original = warp.init_device, joining.join

  def broken(*a):
    raise RuntimeError('boom')
  with mock.patch.object(jetlink_adapter, '_cloudlog'), pytest.raises(RuntimeError):
    jetlink_adapter._timed_attach(broken, 'm', 's', 1, 2)
  assert (warp.init_device, joining.join) == original


def test_no_jetlink_means_plain_attach(monkeypatch):
  monkeypatch.setitem(sys.modules, 'jetlink', None)
  assert jetlink_adapter._timed_attach(lambda *a: 'plain', 'm', 's', 1, 2) == 'plain'
