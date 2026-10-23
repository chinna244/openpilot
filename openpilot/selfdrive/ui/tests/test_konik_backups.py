from types import SimpleNamespace

import pytest

from openpilot.selfdrive.ui.sunnypilot.mici.layouts import software
from openpilot.selfdrive.ui.widgets import konik
from openpilot.system.manager.process_config import konik_uploads_ready


@pytest.mark.parametrize('offroad', [False, True])
def test_backup_toggle_only_changes_own_parameter_offroad(monkeypatch, offroad):
  writes = []
  params = SimpleNamespace(put_bool=lambda *args, **kwargs: writes.append((args, kwargs)))
  monkeypatch.setattr(software, 'ui_state', SimpleNamespace(params=params, is_offroad=lambda: offroad))
  button = object.__new__(software.KonikBackupButton)
  button._apply(True)
  assert writes == ([(('KonikUploadsEnabled', True), {'block': True})] if offroad else [])


def test_pairing_page_never_falls_back_to_comma(monkeypatch):
  monkeypatch.setattr(konik, 'pairing_url', lambda _: None)
  panel = object.__new__(konik.KonikPairingDialog)
  panel.params = object()
  panel.qr_texture = None
  assert panel._get_pairing_url() is None


@pytest.mark.parametrize('started, enabled, expected', [(False, True, True), (True, True, False), (False, False, False)])
def test_backup_process_lifecycle(started, enabled, expected):
  params = SimpleNamespace(get_bool=lambda key: key == 'KonikUploadsEnabled' and enabled)
  assert konik_uploads_ready(started, params, None) is expected


@pytest.mark.parametrize('offroad', [False, True])
def test_standard_display_toggle_preserves_comma(monkeypatch, offroad):
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings import software as standard
  writes, restored = [], []
  params = SimpleNamespace(put_bool=lambda *args, **kwargs: writes.append(args), get_bool=lambda _: False)
  monkeypatch.setattr(standard, 'ui_state', SimpleNamespace(params=params, is_offroad=lambda: offroad))
  panel = object.__new__(standard.SoftwareLayoutSP)
  panel.konik_toggle = SimpleNamespace(action_item=SimpleNamespace(set_state=restored.append))
  panel._on_konik_toggled(True)
  assert writes == ([('KonikUploadsEnabled', True)] if offroad else [])
  assert restored == ([] if offroad else [False])


def test_c4_pairing_appears_after_registration_and_refreshes(monkeypatch):
  from openpilot.selfdrive.ui.mici.widgets import konik as c4
  urls, items, unloaded = ['https://stable.konik.ai/?pair=first'], [], []
  params = SimpleNamespace(get=lambda _: 'registered')
  monkeypatch.setattr(c4, 'pairing_url', lambda _: urls[0])
  monkeypatch.setattr(c4, 'QR', lambda url: SimpleNamespace(_url=url, _texture=None, _generate_qr_code=lambda: 'new texture'))
  monkeypatch.setattr(c4.rl, 'unload_texture', unloaded.append)
  page = object.__new__(c4.KonikBackupPage)
  page._params = params
  page._qr = None
  page._last_refresh = float('-inf')
  page._status = SimpleNamespace(set_value=lambda _: None)
  page._online = SimpleNamespace(set_value=lambda _: None)
  page._scroller = SimpleNamespace(add_widget=items.append)
  page._refresh()
  assert len(items) == 1
  assert page._qr._url == urls[0]
  texture = SimpleNamespace(id=123)
  page._qr._texture = texture
  page._last_refresh = float('-inf')
  urls[0] = 'https://stable.konik.ai/?pair=renewed'
  page._refresh()
  assert unloaded == [texture]
  assert len(items) == 1
  assert page._qr._url == urls[0]


def test_standard_pairing_discards_stale_qr_when_identity_unavailable(monkeypatch):
  unloaded = []
  monkeypatch.setattr(konik, 'pairing_url', lambda _: None)
  monkeypatch.setattr(konik.rl, 'unload_texture', unloaded.append)
  panel = object.__new__(konik.KonikPairingDialog)
  panel.params = object()
  texture = SimpleNamespace(id=123)
  panel.qr_texture = texture
  panel._generate_qr_code()
  assert panel.qr_texture is None
  assert unloaded == [texture]


@pytest.mark.parametrize('started, enabled, identity, expected', [
  (False, True, 'konik-id', True), (True, True, 'konik-id', True),
  (False, False, 'konik-id', False), (False, True, None, False),
  (False, True, 'UnregisteredDevice', False),
])
def test_online_process_requires_separate_registered_identity(started, enabled, identity, expected):
  from openpilot.system.manager.process_config import konik_connect_ready
  params = SimpleNamespace(get_bool=lambda _: enabled, get=lambda _: identity)
  assert konik_connect_ready(started, params, None) is expected


def test_c4_account_displays_independent_online_status():
  from openpilot.selfdrive.ui.mici.widgets import konik as c4
  values = []
  page = object.__new__(c4.KonikBackupPage)
  page._params = SimpleNamespace(get=lambda key: 'connected' if key == 'KonikConnectionStatus' else 'uploading')
  page._status = SimpleNamespace(set_value=lambda _: None)
  page._online = SimpleNamespace(set_value=values.append)
  page._qr = None
  page._last_refresh = float('inf')
  page._refresh()
  assert values == ['connected']
