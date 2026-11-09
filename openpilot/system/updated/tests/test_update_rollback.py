import subprocess

from openpilot.system.updated import updated
from openpilot.system.updated.proven import DriveProof, ENGAGED_MIN_S
from openpilot.system.updated.updated import UpdateRequest, handle_update_request


def drive(proof, seconds, enabled=True, alert="", procs=(), mads=False):
  for _ in range(int(seconds / 0.5)):
    proof.update(0.5, enabled, mads, alert, list(procs))


class TestDriveProof:
  def test_clean_drive_is_proven(self):
    p = DriveProof()
    drive(p, ENGAGED_MIN_S)
    assert p.proven()

  def test_short_drive_is_not(self):
    p = DriveProof()
    drive(p, ENGAGED_MIN_S - 1)
    drive(p, 600, enabled=False)
    assert not p.proven()

  def test_error_disable_fails(self):
    for alert in ("controlsMismatch/immediateDisable", "commIssue/softDisable"):
      p = DriveProof()
      drive(p, 300)
      drive(p, 3 if "soft" in alert else 0.5, alert=alert)
      drive(p, 0.5, enabled=False, alert=alert)
      assert not p.proven() and p.error == alert

  def test_soft_disable_that_clears_is_not_an_error(self):
    """Routes 4c, 4d: one dropped camera frame, softDisabling 0.5 s, enabled again, the driver disengages later."""
    p = DriveProof()
    drive(p, 300)
    drive(p, 0.5, alert="commIssue/softDisable")
    drive(p, 1.5, alert="locationdTemporaryError/softDisable")
    drive(p, 60, alert="bigModelLinkLost/warning")
    drive(p, 600, enabled=False, alert="buttonCancel/userDisable")
    assert p.proven()

  def test_error_disable_with_mads_still_steering_fails(self):
    p = DriveProof()
    drive(p, 300, mads=True)
    drive(p, 3, alert="commIssue/softDisable", mads=True)
    drive(p, 10, enabled=False, mads=True)
    assert not p.proven() and p.error == "commIssue/softDisable"

  def test_driver_and_user_disables_are_not_errors(self):
    p = DriveProof()
    drive(p, 300)
    for alert in ("doorOpen/softDisable", "seatbeltNotLatched/softDisable", "buttonCancel/userDisable", "steerSaturated/warning"):
      drive(p, 1, alert=alert)
    assert p.proven()

  def test_process_down(self):
    p = DriveProof()
    drive(p, 300, procs=[("modeld", True, True), ("updated", False, False)])
    drive(p, 4, procs=[("modeld", False, True)])
    assert p.proven()  # a short restart is fine
    drive(p, 6, procs=[("modeld", False, True)])
    assert not p.proven() and p.error == "modeld not running"

  def test_process_down_before_drive_or_optional(self):
    p = DriveProof()
    drive(p, 300, procs=[("statsd", False, True)])  # never ran this drive
    drive(p, 10, procs=[("mapd", True, True)])
    drive(p, 10, procs=[("mapd", False, True)])  # optional, selfdrived ignores it too
    assert p.proven()

  def test_reset(self):
    p = DriveProof()
    drive(p, 300)
    drive(p, 0.5, enabled=False, alert="controlsMismatch/immediateDisable")
    assert p.error
    p.reset()
    drive(p, ENGAGED_MIN_S)
    assert p.proven()


def make_repo(path):
  if not path.is_dir():
    path.mkdir()
    (path / "f").write_text("a")
    for cmd in (["init", "-q"], ["add", "f"], ["-c", "user.name=t", "-c", "user.email=t", "commit", "-qm", "a"]):
      subprocess.check_call(["git", *cmd], cwd=path)
  return path


class FakeParams:
  def __init__(self, **values):
    self.values = values

  def get(self, key, return_default=False):
    return self.values.get(key, 0 if key == "UpdateRequest" else None)

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put(self, key, value, block=False):
    self.values[key] = value

  def put_bool(self, key, value, block=False):
    self.values[key] = value

  def remove(self, key):
    self.values.pop(key, None)


class FakeWait:
  def __init__(self):
    self.calls = []

  def check_now(self, signum, frame):
    self.calls.append("check")

  def update_now(self, signum, frame):
    self.calls.append("download")


class TestUpdateRequest:
  def run(self, request, tmp_path, monkeypatch, previous=True, confirm=False, **values):
    prev = tmp_path / "previous"
    if previous:
      prev.mkdir(exist_ok=True)
    monkeypatch.setattr(updated, "PREVIOUS", str(prev))
    monkeypatch.setattr(updated, "BASEDIR", str(make_repo(tmp_path / "cur")))
    monkeypatch.setattr(updated, "armed", {})
    monkeypatch.setattr(updated, "seen_request", UpdateRequest.NONE)
    params, wait = FakeParams(**values), FakeWait()
    for r in (request, UpdateRequest.CONFIRM) if confirm else (request,):
      params.values["UpdateRequest"] = r
      handle_update_request(params, wait)
      assert params.values["UpdateRequest"] == r  # the device never writes it back (sunnylink would reject the next choice)
    return params.values, wait.calls

  def test_none_does_nothing(self, tmp_path, monkeypatch):
    values, calls = self.run(UpdateRequest.NONE, tmp_path, monkeypatch, IsOffroad=True)
    assert calls == [] and "DoReboot" not in values

  def test_acts_on_change_only(self, tmp_path, monkeypatch):
    monkeypatch.setattr(updated, "seen_request", UpdateRequest.NONE)
    params, wait = FakeParams(UpdateRequest=UpdateRequest.CHECK), FakeWait()
    for _ in range(3):
      handle_update_request(params, wait)
    assert wait.calls == ["check"]  # the choice stays shown, it acts once
    params.values["UpdateRequest"] = UpdateRequest.NONE
    handle_update_request(params, wait)
    params.values["UpdateRequest"] = UpdateRequest.CHECK  # Ready, then Check again
    handle_update_request(params, wait)
    assert wait.calls == ["check", "check"]

  def test_choice_from_before_start_is_ignored(self, monkeypatch):
    params, wait = FakeParams(UpdateRequest=UpdateRequest.ROLLBACK, IsOffroad=True), FakeWait()
    def stop(t):
      raise StopLoop
    monkeypatch.setattr(updated.time, "sleep", stop)
    try:
      updated.watch_update_requests(params, wait)
    except StopLoop:
      pass
    assert "UpdaterRollback" not in params.values and "UpdaterCheckResult" not in params.values

  def test_check_and_download(self, tmp_path, monkeypatch):
    assert self.run(UpdateRequest.CHECK, tmp_path, monkeypatch)[1] == ["check"]
    assert self.run(UpdateRequest.DOWNLOAD, tmp_path, monkeypatch)[1] == ["download"]

  def test_install(self, tmp_path, monkeypatch):
    values, _ = self.run(UpdateRequest.INSTALL, tmp_path, monkeypatch, IsOffroad=True, UpdateAvailable=True)
    assert "DoReboot" not in values and "select Confirm within 60 s" in values["UpdaterCheckResult"]  # first press only asks
    values, _ = self.run(UpdateRequest.INSTALL, tmp_path, monkeypatch, confirm=True, IsOffroad=True, UpdateAvailable=True)
    assert values["DoReboot"] and "UpdaterRollback" not in values

  def test_install_needs_update_and_offroad(self, tmp_path, monkeypatch):
    assert "nothing downloaded" in self.run(UpdateRequest.INSTALL, tmp_path, monkeypatch, IsOffroad=True)[0]["UpdaterCheckResult"]
    assert "DoReboot" not in self.run(UpdateRequest.INSTALL, tmp_path, monkeypatch, confirm=True, IsOffroad=True)[0]
    assert "DoReboot" not in self.run(UpdateRequest.INSTALL, tmp_path, monkeypatch, confirm=True, IsOffroad=False, UpdateAvailable=True)[0]

  def test_rollback(self, tmp_path, monkeypatch):
    values, _ = self.run(UpdateRequest.ROLLBACK, tmp_path, monkeypatch, IsOffroad=True)
    assert "DoReboot" not in values and "UpdaterRollback" not in values
    values, _ = self.run(UpdateRequest.ROLLBACK, tmp_path, monkeypatch, confirm=True, IsOffroad=True)
    assert values["UpdaterRollback"] and values["DoReboot"]

  def test_confirm_expires_and_needs_a_request(self, tmp_path, monkeypatch):
    t = [1000.]
    monkeypatch.setattr(updated.time, "monotonic", lambda: t[0])
    prev = tmp_path / "previous"
    prev.mkdir()
    monkeypatch.setattr(updated, "PREVIOUS", str(prev))
    monkeypatch.setattr(updated, "BASEDIR", str(make_repo(tmp_path / "cur")))
    monkeypatch.setattr(updated, "armed", {})
    monkeypatch.setattr(updated, "seen_request", UpdateRequest.NONE)
    params = FakeParams(IsOffroad=True)
    def press(r, dt=0.):
      t[0] += dt
      params.values["UpdateRequest"] = r
      handle_update_request(params, FakeWait())
    press(UpdateRequest.CONFIRM)  # nothing armed
    assert "nothing to confirm" in params.values["UpdaterCheckResult"]
    press(UpdateRequest.ROLLBACK)
    press(UpdateRequest.CONFIRM, 61.)  # too late
    assert "DoReboot" not in params.values
    press(UpdateRequest.ROLLBACK)
    press(UpdateRequest.ROLLBACK)  # selecting it again only re-arms
    assert "DoReboot" not in params.values
    press(UpdateRequest.CONFIRM, 30.)
    assert params.values["DoReboot"] and params.values["UpdaterRollback"]

  def test_confirm_rechecks_state(self, tmp_path, monkeypatch):
    monkeypatch.setattr(updated, "armed", {})
    monkeypatch.setattr(updated, "seen_request", UpdateRequest.NONE)
    params = FakeParams(IsOffroad=True, UpdateAvailable=True)
    params.values["UpdateRequest"] = UpdateRequest.INSTALL
    handle_update_request(params, FakeWait())
    params.values["IsOffroad"] = False  # car started before Confirm
    params.values["UpdateRequest"] = UpdateRequest.CONFIRM
    handle_update_request(params, FakeWait())
    assert "DoReboot" not in params.values

  def test_rollback_needs_previous_and_offroad(self, tmp_path, monkeypatch):
    for kwargs in ({"previous": False, "IsOffroad": True}, {"IsOffroad": False}):
      values, _ = self.run(UpdateRequest.ROLLBACK, tmp_path, monkeypatch, confirm=True, **kwargs)
      assert "UpdaterRollback" not in values and "DoReboot" not in values

  def test_rollback_refused_with_local_changes(self, tmp_path, monkeypatch):
    (make_repo(tmp_path / "cur") / "f").write_text("patched")  # run() reuses this repo as BASEDIR
    values, _ = self.run(UpdateRequest.ROLLBACK, tmp_path, monkeypatch, confirm=True, IsOffroad=True)
    assert "UpdaterRollback" not in values and "DoReboot" not in values
    assert "not possible: local changes" in self.run(UpdateRequest.ROLLBACK, tmp_path, monkeypatch, IsOffroad=True)[0]["UpdaterCheckResult"]


class TestRollbackBlocker:
  def test_blockers(self, tmp_path, monkeypatch):
    prev, cur = tmp_path / "previous", make_repo(tmp_path / "cur")
    monkeypatch.setattr(updated, "PREVIOUS", str(prev))
    monkeypatch.setattr(updated, "BASEDIR", str(cur))
    monkeypatch.setattr(updated, "AGNOS", False)
    assert updated.rollback_blocker() == "no previous version"
    prev.mkdir()
    assert updated.rollback_blocker() == ""
    (cur / "f").write_text("patched")
    assert updated.rollback_blocker() == "local changes on the device"
    (cur / "f").write_text("a")

    monkeypatch.setattr(updated, "AGNOS", True)
    monkeypatch.setattr(updated.HARDWARE, "get_os_version", lambda: "20.3")
    (prev / "launch_env.sh").write_text('if [ -z "$AGNOS_VERSION" ]; then\n  export AGNOS_VERSION="19.9"\nfi\n')
    assert updated.rollback_blocker() == "needs AGNOS 19.9"
    (prev / "launch_env.sh").write_text('export AGNOS_VERSION="20.3"\n')
    assert updated.rollback_blocker() == ""


class TestStagedUpdate:
  """A staged copy of the running version is no update: it would reboot into the same code and rebuild it."""

  def setup(self, tmp_path, monkeypatch, remote_commit=None, staged_branch="develop", change_staged=False):
    cur = make_repo(tmp_path / "cur")
    subprocess.check_call(["git", "branch", "-qM", "develop"], cwd=cur)
    fin = tmp_path / "finalized"
    subprocess.check_call(["git", "clone", "-q", str(cur), str(fin)])
    if staged_branch != "develop":
      subprocess.check_call(["git", "checkout", "-qb", staged_branch], cwd=fin)
    if change_staged:
      (fin / "f").write_text("b")
      subprocess.check_call(["git", "-c", "user.name=t", "-c", "user.email=t", "commit", "-qam", "b"], cwd=fin)
    (fin / ".overlay_consistent").touch()
    monkeypatch.setattr(updated, "BASEDIR", str(cur))
    monkeypatch.setattr(updated, "FINALIZED", str(fin))
    monkeypatch.setattr(updated.Updater, "target_branch", property(lambda self: "develop"))
    u = updated.Updater()
    u.branches["develop"] = remote_commit or u.get_commit_hash(str(cur))
    return u, cur, fin

  def test_same_commit_staged_is_not_ready(self, tmp_path, monkeypatch):
    u, _, _ = self.setup(tmp_path, monkeypatch, remote_commit="f" * 40)  # remote moved on, staged copy is still the running one
    assert not u.update_ready

  def test_new_commit_staged_is_ready(self, tmp_path, monkeypatch):
    u, _, fin = self.setup(tmp_path, monkeypatch, change_staged=True)
    assert u.update_ready
    (fin / ".overlay_consistent").unlink()
    assert not u.update_ready

  def test_staged_on_another_branch(self, tmp_path, monkeypatch):
    u, _, _ = self.setup(tmp_path, monkeypatch, staged_branch="other")
    assert not u.update_ready  # not the target branch

  def test_up_to_date(self, tmp_path, monkeypatch):
    u, cur, _ = self.setup(tmp_path, monkeypatch)
    assert u.up_to_date
    u.branches["develop"] = "f" * 40
    assert not u.up_to_date
    u.branches["develop"] = u.get_commit_hash(str(cur))
    monkeypatch.setattr(updated.Updater, "target_branch", property(lambda self: "release"))
    assert not u.up_to_date


class StopLoop(Exception):
  pass


class TestDisableUpdatesLive:
  """Turning DisableUpdates off resumes updating without a reboot (updated used to exit at startup)."""

  def test_toggle_without_reboot(self, tmp_path, monkeypatch):
    params = FakeParams(DisableUpdates=True)
    states, calls = [], []
    orig_put = params.put
    def put(key, value, block=False):
      if key == "UpdaterState":
        states.append(value)
      orig_put(key, value, block)
    params.put = put

    class Wait:
      def __init__(self):
        import threading
        self.ready_event = threading.Event()
        self.user_request = updated.UserRequest.NONE
        self.sleeps = 0
      def sleep(self, t):
        self.sleeps += 1
        calls.append(("sleep", t))
        if self.sleeps == 2:
          params.values["DisableUpdates"] = False  # user turns updates back on
        if self.sleeps == 3:
          raise StopLoop

    class Updater:
      branches = {"develop": "a" * 40}
      target_branch = "develop"
      pinned_commit = ""
      target_commit = "a" * 40
      up_to_date = True
      def set_params(self, *args):
        pass
      def check_for_update(self):
        calls.append("check")
      def fetch_update(self):
        calls.append("fetch")

    monkeypatch.setattr(updated, "Params", lambda: params)
    monkeypatch.setattr(updated, "WaitTimeHelper", Wait)
    monkeypatch.setattr(updated, "Updater", Updater)
    monkeypatch.setattr(updated, "init_overlay", lambda: calls.append("overlay"))
    monkeypatch.setattr(updated, "system_time_valid", lambda: True)
    monkeypatch.setattr(updated, "watch_update_requests", lambda *a: None)
    monkeypatch.setattr(updated, "LOCK_FILE", str(tmp_path / "lock"))
    monkeypatch.setattr(updated, "STAGING_ROOT", str(tmp_path / "staging"))
    monkeypatch.setattr(updated, "FINALIZED", str(tmp_path / "finalized"))

    try:
      updated.main()
    except StopLoop:
      pass

    # disabled: no overlay, no check, only short waits; then on again: checks without a reboot
    assert calls[:2] == [("sleep", 10), ("sleep", 10)]
    assert calls[2:] == ["overlay", "check", ("sleep", 1.5 * 60 * 60)]
    assert states[:3] == ["idle", "updates disabled", "idle"] and states[-1] == "idle"
    assert "up to date" in params.values["UpdaterCheckResult"]


class TestCheckResult:
  def make(self, up_to_date, commit):
    class U:
      branches = {"develop": commit}
      target_branch = "develop"
      pinned_commit = ""
      target_commit = commit
    u = U()
    u.up_to_date = up_to_date
    return u

  def test_results(self):
    assert updated.check_result(self.make(True, "a" * 40), None).endswith("UTC: up to date")
    assert updated.check_result(self.make(False, "b" * 40), None).endswith("new version bbbbbbb available")
    assert updated.check_result(self.make(False, "b" * 40), "b" * 40).endswith("bbbbbbb available but rolled back, skipped")
    assert updated.check_result(self.make(False, ""), None).endswith("branch develop not found")


class TestDisableWakesLoop:
  def test_toggle_wakes_sleeping_loop(self, monkeypatch):
    import threading
    params = FakeParams(DisableUpdates=False)
    wait = FakeWait()
    wait.ready_event = threading.Event()
    sleeps = []
    def fake_sleep(t):
      sleeps.append(t)
      if len(sleeps) == 1:
        params.values["DisableUpdates"] = True  # user disables while the loop sleeps 1.5 h
      if len(sleeps) == 3:
        raise StopLoop
    monkeypatch.setattr(updated.time, "sleep", fake_sleep)
    try:
      updated.watch_update_requests(params, wait)
    except StopLoop:
      pass
    assert wait.ready_event.is_set()


class TestDriveInputs:
  """Route 4c (2026-11-08): the drive start read a controlsMismatch kept from before selfdrived was up."""

  class SM(dict):
    def __init__(self, alive, enabled=False, mads=False, alert=""):
      from types import SimpleNamespace
      super().__init__(selfdriveState=SimpleNamespace(enabled=enabled, alertType=alert),
                       selfdriveStateSP=SimpleNamespace(mads=SimpleNamespace(enabled=mads)))
      self.alive = {"selfdriveState": alive, "selfdriveStateSP": alive}

  def test_stale_message_ignored(self):
    from openpilot.system.updated.proven import drive_inputs
    assert drive_inputs(self.SM(False, enabled=True, alert="controlsMismatch/immediateDisable")) == (False, False, "")

  def test_live_message_used(self):
    from openpilot.system.updated.proven import drive_inputs
    assert drive_inputs(self.SM(True, enabled=True, alert="commIssue/softDisable")) == (True, False, "commIssue/softDisable")
    assert drive_inputs(self.SM(True, mads=True)) == (False, True, "")


def test_rollback_blocker_leaves_git_untouched(tmp_path, monkeypatch):
  """init_overlay and the boot script treat any change under .git as local edits: the check must not write there."""
  import os
  cur = make_repo(tmp_path / "cur")
  (tmp_path / "previous").mkdir()
  monkeypatch.setattr(updated, "PREVIOUS", str(tmp_path / "previous"))
  monkeypatch.setattr(updated, "BASEDIR", str(cur))
  monkeypatch.setattr(updated, "AGNOS", False)
  past = os.stat(cur / "f").st_mtime - 100
  (cur / "f").touch()  # stale stat info: a locking git status would rewrite the index
  for p in [cur / ".git", cur / ".git" / "index"]:
    os.utime(p, (past, past))
  before = [os.stat(p).st_mtime for p in (cur / ".git", cur / ".git" / "index")]
  updated.rollback_blocker()
  assert [os.stat(p).st_mtime for p in (cur / ".git", cur / ".git" / "index")] == before


class TestVersions:
  """The Versions page: recent commits of the branch, any of which can be pinned as the update target."""

  def repos(self, tmp_path, n=5):
    origin = tmp_path / "origin"
    origin.mkdir()
    def git(*a):
      return subprocess.check_output(["git", "-c", "user.name=t", "-c", "user.email=t", *a], cwd=origin, text=True).strip()
    git("init", "-q", "-b", "develop")
    shas = []
    for i in range(n):
      (origin / "f").write_text(str(i))
      git("add", "f")
      git("commit", "-qm", f"change {i}")
      shas.append(git("rev-parse", "HEAD"))
    merged = tmp_path / "merged"
    subprocess.check_call(["git", "clone", "-q", str(origin), str(merged)])
    return origin, merged, shas

  def updater(self, monkeypatch, merged, **values):
    monkeypatch.setattr(updated, "OVERLAY_MERGED", str(merged))
    monkeypatch.setattr(updated.Updater, "target_branch", property(lambda self: "develop"))
    u = updated.Updater()
    u.params = FakeParams(**values)
    return u

  def test_commit_list(self, tmp_path, monkeypatch):
    _, merged, shas = self.repos(tmp_path)
    monkeypatch.setattr(updated, "COMMIT_LIST_LEN", 3)
    monkeypatch.setattr(updated, "UPDATE_CONTROLS_COMMIT", shas[3])
    commits = self.updater(monkeypatch, merged).commit_list()
    assert [c["sha"] for c in commits] == shas[::-1][:3]  # newest first, capped
    assert [c["title"] for c in commits] == ["change 4", "change 3", "change 2"]
    assert [c["update_controls"] for c in commits] == [True, True, False]  # change 2 predates the controls

  def test_pin_from_another_branch_is_dropped(self, tmp_path, monkeypatch):
    _, merged, shas = self.repos(tmp_path)
    u = self.updater(monkeypatch, merged, UpdaterTargetCommit=shas[1])
    u.commit_list()
    assert u.params.values["UpdaterTargetCommit"] == shas[1]  # on the branch: kept
    u = self.updater(monkeypatch, merged, UpdaterTargetCommit="f" * 40)
    u.commit_list()
    assert "UpdaterTargetCommit" not in u.params.values

  def test_target_follows_pin(self, tmp_path, monkeypatch):
    _, merged, shas = self.repos(tmp_path)
    u = self.updater(monkeypatch, merged)
    u.branches["develop"] = shas[-1]
    assert u.target_commit == shas[-1] and u.pinned_commit == ""
    u.params.values["UpdaterTargetCommit"] = shas[1]
    assert u.target_commit == shas[1]
    monkeypatch.setattr(updated, "BASEDIR", str(merged))  # running the latest, pinned to an older one: not up to date
    assert not u.up_to_date
    assert "pinned to " + shas[1][:7] in updated.check_result(u, None) and "latest " + shas[-1][:7] in updated.check_result(u, None)

  def test_fetch_checks_out_the_pin(self, monkeypatch):
    cmds = []
    monkeypatch.setattr(updated, "run", lambda cmd, cwd=None: cmds.append(cmd) or "")
    monkeypatch.setattr(updated, "set_consistent_flag", lambda c: None)
    monkeypatch.setattr(updated, "finalize_update", lambda: None)
    monkeypatch.setattr(updated, "AGNOS", False)
    monkeypatch.setattr(updated.Updater, "target_branch", property(lambda self: "develop"))
    for pin, expected in (("a" * 40, "a" * 40), (None, "FETCH_HEAD")):
      cmds.clear()
      u = updated.Updater()
      u.params = FakeParams(UpdaterTargetCommit=pin)
      u.fetch_update()
      checkout = next(c for c in cmds if "checkout" in c)
      assert checkout[-1] == expected and checkout[-2] == "develop"
