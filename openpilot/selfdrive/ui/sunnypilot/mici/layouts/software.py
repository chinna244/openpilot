"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import threading

import raylib_nb as rl

from openpilot.selfdrive.ui.mici.layouts.settings.software import (CheckUpdateButton, InstallUpdateButton, SoftwareLayoutMici,
                                                                    TargetBranchButton, UpdaterState, _split_description)
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog, BigDialog
from openpilot.selfdrive.ui.mici.widgets.konik import KonikBackupPage
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.html_render import LIST_INDENT_PX, ElementType, HtmlRenderer
from openpilot.system.ui.widgets.scroller import NavRawScrollPanel, NavScroller
from openpilot.system.ui.widgets.slider import LargerSlider

NOTES_FONT_SIZE = 28  # the renderer defaults to the tici screen, mici is 536x240
SLIDER_MARGIN = 24


def update_pending() -> bool:
  return ui_state.params.get_bool("UpdateAvailable")


def no_notes() -> str:
  return f"<h2>{tr('No release notes available.')}</h2>"


class ReleaseNotesPage(NavRawScrollPanel):
  """Scrolling page for the notes updated writes to params: the top CHANGELOG.md entry, as html."""

  def __init__(self):
    super().__init__()
    self._content = HtmlRenderer(text=no_notes(), text_size={ElementType.P: NOTES_FONT_SIZE})
    # installing is a reboot: offer it under the notes of the release waiting to be installed
    self._install = self._child(LargerSlider("slide to install", confirm_callback=self._on_install))
    self._install.set_visible(lambda: ui_state.is_offroad() and update_pending())
    self._install.set_enabled(lambda: self.enabled and not self.is_dismissing)

  def show_notes(self, pending: bool):
    """Open on the staged release's notes, or on the running release's."""
    notes = ui_state.params.get("UpdaterNewReleaseNotes" if pending else "UpdaterCurrentReleaseNotes")
    self._content.parse_html_content((notes or b"").decode("utf-8", "replace").strip() or no_notes())
    gui_app.push_widget(self)

  def _on_install(self):
    threading.Thread(target=lambda: ui_state.params.put_bool("DoReboot", True, block=True), daemon=True).start()

  def _render(self, rect: rl.Rectangle):
    # the renderer indents a nested list without narrowing its wrap width, so leave the room here
    width = rect.width - LIST_INDENT_PX
    content_height = self._content.get_total_height(int(width))

    installable = self._install.is_visible
    total_height = content_height + (SLIDER_MARGIN + self._install.rect.height if installable else 0)
    scroll_offset = round(self._scroll_panel.update(rect, total_height))

    # the renderer draws every line inside the rect it is handed, so hand it the screen, not the notes
    self._content.render(rl.Rectangle(rect.x, rect.y + scroll_offset, width, rect.height - scroll_offset))

    if installable:
      self._install.render(rl.Rectangle(rect.x + (rect.width - self._install.rect.width) / 2,
                                        rect.y + scroll_offset + content_height + SLIDER_MARGIN,
                                        self._install.rect.width, self._install.rect.height))


class ReleaseNotesButton(BigButton):
  def __init__(self):
    super().__init__("release notes", "", gui_app.texture("icons_mici/settings/device/info.png", 64, 64))
    self._page = ReleaseNotesPage()
    self.set_click_callback(lambda: self._page.show_notes(update_pending()))

  def _update_state(self):
    super()._update_state()

    desc = _split_description(ui_state.params.get("UpdaterNewDescription" if update_pending() else "UpdaterCurrentDescription") or "")
    value = desc[0] if desc is not None else ""
    if self.get_value() != value:
      self.set_value(value)


class CheckUpdateButtonSP(CheckUpdateButton):
  """A downloaded update says so. Upstream's button goes back to "download update" once the
  updater is idle, because UpdaterFetchAvailable (the remote differs from what is installed)
  stays set until the install, and "install now" is the next item down: a user switching
  branches read the download as having failed (2026-10-04). A tap still re-downloads."""
  READY = "update ready\nscroll to install"

  def _update_state(self):
    super()._update_state()
    # with updates disabled the updater answers "updates disabled" instead of going back to idle
    if self._state != UpdaterState.IDLE and ui_state.params.get("UpdaterState") == "updates disabled":
      self.set_rotate_icon(False)
      self.set_value("updates disabled")
      self._state = UpdaterState.IDLE
      self._hide_value_t = rl.get_time()
    if self._state == UpdaterState.IDLE and self.get_value() == "download update" and update_pending():
      self.set_value(self.READY)
      self.set_icon(self._txt_up_to_date_icon)

  def _handle_mouse_release(self, mouse_pos):
    if self.get_value() == self.READY:
      self.set_value("download update")   # what upstream's handler reads to pick the download signal
    super()._handle_mouse_release(mouse_pos)


class RollbackButton(BigButton):
  """Reboots into the build kept before the last update (launch_chffrplus.sh swaps it in)."""

  def __init__(self):
    self._icon = gui_app.texture("icons_mici/settings/device/reboot.png", 64, 70)
    super().__init__("roll back", "", self._icon)
    self.set_visible(lambda: ui_state.is_offroad() and (ui_state.params.get("UpdaterRollbackDescription") or "none") != "none")
    self.set_click_callback(lambda: gui_app.push_widget(BigConfirmationDialog("roll back + reboot", self._icon, self._roll_back, red=True)))

  def _roll_back(self):
    def run():
      if ui_state.is_offroad():
        ui_state.params.put_bool("UpdaterRollback", True, block=True)
        ui_state.params.put_bool("DoReboot", True, block=True)

    threading.Thread(target=run, daemon=True).start()

  def _update_state(self):
    super()._update_state()
    # "commit date (proven good)" of the kept build, or why the boot script would refuse it
    rollback = ui_state.params.get("UpdaterRollbackDescription") or ""
    blocked = " - not possible: " in rollback
    self.set_enabled(not blocked)
    desc = _split_description(rollback)
    value = rollback.split(" - ", 1)[1] if blocked else f"{desc[2]} {desc[3]}" if desc is not None else ""
    if self.get_value() != value:
      self.set_value(value)


class VersionsPage(NavScroller):
  """Recent commits of the update branch (updated lists them on every check), newest first."""

  def __init__(self, on_select):
    super().__init__()
    params = ui_state.params
    running = params.get("GitCommit") or ""
    pinned = params.get("UpdaterTargetCommit") or ""
    check_icon = gui_app.texture("icons_mici/settings/device/up_to_date.png", 64, 64)

    buttons = []
    for i, c in enumerate(params.get("UpdaterCommitList") or []):
      tags = [c["sha"][:7], c["date"]]
      tags += ["running"] if c["sha"] == running else []
      tags += ["pinned"] if c["sha"] == pinned else []
      tags += [] if c.get("update_controls") else ["no update menu"]
      btn = BigButton(c["title"], " · ".join(tags), check_icon if c["sha"] == running else None, scroll=True)
      btn.set_click_callback(lambda c=c, latest=(i == 0): self.dismiss(lambda: on_select(c, latest)))
      buttons.append(btn)
    self._scroller.add_widgets(buttons)


class VersionsButton(BigButton):
  """Go to any recent commit: it becomes the update target (pinned) and downloads; picking the newest follows the branch again."""

  def __init__(self, check_update_btn: CheckUpdateButton):
    self._icon = gui_app.texture("icons_mici/settings/device/update.png", 64, 75)
    super().__init__("versions", "", description="Go to any recent commit. It downloads now and installs like an update (rebuild 10-30 min).")
    self._check_update_btn = check_update_btn
    self.set_click_callback(self._on_click)
    self.set_enabled(lambda: ui_state.is_offroad())

  def _update_state(self):
    super()._update_state()
    pinned = ui_state.params.get("UpdaterTargetCommit") or ""
    value = f"pinned {pinned[:7]}" if pinned else "latest"
    if self.get_value() != value:
      self.set_value(value)

  def _on_click(self):
    if not ui_state.params.get("UpdaterCommitList"):
      gui_app.push_widget(BigDialog("", tr("No version list yet. Connect to Wi-Fi and check for update first.")))
      return
    gui_app.push_widget(VersionsPage(self._on_select))

  def _on_select(self, commit: dict, latest: bool):
    title = "follow latest" if latest else f"go to {commit['sha'][:7]}"
    gui_app.push_widget(BigConfirmationDialog(title, self._icon, lambda: self._go(commit, latest)))

  def _go(self, commit: dict, latest: bool):
    # written before the download signal, so updated already sees the new target
    if latest:
      ui_state.params.remove("UpdaterTargetCommit")
    else:
      ui_state.params.put("UpdaterTargetCommit", commit["sha"], block=True)
    # download it now; the check button shows the progress, install now appears once it is ready.
    # Already running it (e.g. back to latest): only check, a copy of the running version is no update
    running = commit["sha"] == (ui_state.params.get("GitCommit") or "")
    self._check_update_btn._signal_updater(CheckUpdateButton.CHECK_FOR_UPDATE if running else CheckUpdateButton.DOWNLOAD_UPDATE)


class KonikBackupButton(BigButton):
  def __init__(self):
    self._icon = gui_app.texture("icons_mici/settings/device/green_settings.png", 64, 64)
    super().__init__("Konik backups", "", self._icon,
                     description="Enable Konik online status and parked live view; back up drives on parked Wi-Fi. comma stays connected.")
    self.set_click_callback(self._confirm)

  def _confirm(self):
    if ui_state.is_offroad():
      enabled = not ui_state.params.get_bool("KonikUploadsEnabled")
      title = "enable Konik backups" if enabled else "disable Konik backups"
      gui_app.push_widget(BigConfirmationDialog(title, self._icon, lambda: self._apply(enabled)))

  def _apply(self, enabled):
    if ui_state.is_offroad():
      ui_state.params.put_bool("KonikUploadsEnabled", enabled, block=True)

  def _update_state(self):
    super()._update_state()
    self.set_enabled(ui_state.is_offroad())
    self.set_value("enabled" if ui_state.params.get_bool("KonikUploadsEnabled") else "disabled")


class SoftwareLayoutSP(SoftwareLayoutMici):
  def __init__(self):
    super().__init__()
    self._scroller.add_widget(KonikBackupButton())
    pair_button = BigButton("Konik account", "pair and view backup status")
    pair_button.set_click_callback(lambda: gui_app.push_widget(KonikBackupPage()))
    self._scroller.add_widget(pair_button)

    # the check button says when the update is downloaded; the branch button rechecks through it
    items = self._scroller.items
    check = next(i for i, item in enumerate(items) if type(item) is CheckUpdateButton)
    self._scroller.add_widget(CheckUpdateButtonSP())
    items[check] = items.pop()
    next(item for item in items if isinstance(item, TargetBranchButton))._check_update_btn = items[check]

    # versions sits after target branch
    self._scroller.add_widget(VersionsButton(items[check]))
    items = self._scroller.items
    branch = next(i for i, item in enumerate(items) if isinstance(item, TargetBranchButton))
    items.insert(branch + 1, items.pop())

    # add_widget wraps the touch callback, so append and then move it up behind "install now"
    self._scroller.add_widget(ReleaseNotesButton())
    items = self._scroller.items
    install = next(i for i, item in enumerate(items) if isinstance(item, InstallUpdateButton))
    items.insert(install + 1, items.pop())

    # roll back sits after the release notes, below install
    self._scroller.add_widget(RollbackButton())
    items = self._scroller.items
    items.insert(install + 2, items.pop())
