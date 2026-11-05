import pyray as rl

from openpilot.common.api.konik import pairing_url
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.ui.widgets.pairing_dialog import PairingDialog
from openpilot.system.ui.lib.application import FontWeight, gui_app


class KonikPairingDialog(PairingDialog):
  QR_REFRESH_INTERVAL = 5
  def _get_pairing_url(self) -> str | None:
    try:
      return pairing_url(self.params)
    except Exception:
      cloudlog.exception('Konik pairing token unavailable')
      return None

  def _generate_qr_code(self):
    # Do not generate a scannable empty QR or silently use comma's pairing token.
    if self._get_pairing_url() is None:
      if self.qr_texture and self.qr_texture.id:
        rl.unload_texture(self.qr_texture)
      self.qr_texture = None
      return
    super()._generate_qr_code()

  def _update_state(self):
    pass  # comma pairing status must not close this independent pairing dialog.

  def _render(self, rect):
    rl.clear_background(rl.Color(224, 224, 224, 255))
    self._check_qr_refresh()
    self._close_btn.render(rl.Rectangle(rect.x + 25, rect.y + 25, 80, 80))
    font = gui_app.font(FontWeight.NORMAL)
    rl.draw_text_ex(font, 'Konik Stable backup', rl.Vector2(rect.x + 130, rect.y + 40), 55, 0, rl.BLACK)
    text = 'Enable backups, then park on Wi-Fi.\nOpen stable.konik.ai, add a device\nand scan this code.\ncomma Connect stays active.'
    rl.draw_text_ex(font, text, rl.Vector2(rect.x + 40, rect.y + 160), 35, 0, rl.BLACK)
    status = self.params.get('KonikUploadStatus') or 'waiting for registration'
    rl.draw_text_ex(font, status, rl.Vector2(rect.x + 40, rect.y + 350), 30, 0, rl.BLACK)
    online = self.params.get('KonikConnectionStatus') or 'waiting for connection'
    rl.draw_text_ex(font, 'Konik connection: ' + online, rl.Vector2(rect.x + 40, rect.y + 400), 30, 0, rl.BLACK)
    self._render_qr_code(rl.Rectangle(rect.x + rect.width * 0.55, rect.y + 130, rect.height - 170, rect.height - 170))
    return -1
