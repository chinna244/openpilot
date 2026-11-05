import time

import pyray as rl

from openpilot.common.api.konik import pairing_url
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.mici.widgets.button import GreyBigButton
from openpilot.selfdrive.ui.mici.widgets.qr import QR
from openpilot.system.ui.widgets.scroller import NavScroller


class KonikBackupPage(NavScroller):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._qr = None
    self._last_refresh = float('-inf')
    self._status = GreyBigButton('Konik backup', '')
    self._online = GreyBigButton('Konik connection', '')
    self._scroller.add_widgets([
      self._status,
      self._online,
      GreyBigButton('pair Konik', 'Open stable.konik.ai, add a device and scan the QR below.'),
      GreyBigButton('', 'Backups need parked Wi-Fi. Konik online status and parked live view are also enabled.'),
      GreyBigButton('', 'comma stays connected. Only one live-view session can use the cameras at a time.'),
      GreyBigButton('', 'Pending drives can be deleted if local storage fills before they upload.'),
    ])

  def _refresh(self):
    self._status.set_value(self._params.get('KonikUploadStatus') or 'enable backups and park on Wi-Fi')
    self._online.set_value(self._params.get('KonikConnectionStatus') or 'waiting for connection')
    now = time.monotonic()
    if now - self._last_refresh < (300 if self._qr else 5):
      return
    self._last_refresh = now
    try:
      url = pairing_url(self._params)
      if url:
        if self._qr is None:
          self._qr = QR(url)
          self._scroller.add_widget(self._qr)
        else:
          if self._qr._texture and self._qr._texture.id:
            rl.unload_texture(self._qr._texture)
          self._qr._texture = None
          self._qr._url = url
          self._qr._texture = self._qr._generate_qr_code()
    except Exception:
      cloudlog.exception('Konik pairing QR unavailable')

  def _update_state(self):
    super()._update_state()
    self._refresh()
