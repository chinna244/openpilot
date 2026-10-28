"""A source build must never let SCons trust tracked build outputs it did not leave: updated.py's
copytree + `git reset --hard`, a checkout or a copy restores them while .sconsign still vouches."""
import os
import shutil
import subprocess
import time

import pytest

from openpilot.system.manager.build import SCONSIGN, STAMP, invalidate_stale_outputs, record_build

OUTPUTS = ("openpilot/common/libparams_c.so", "openpilot/selfdrive/pandad/pandad", "panda/board/obj/panda_h7.bin.signed",
           "openpilot/system/camerad/camerad", "third_party/acados/lib/libqpOASES_e.so.3.1")
EXECUTABLES = ("openpilot/selfdrive/pandad/pandad", "openpilot/system/camerad/camerad")


def git(repo, *args):
  return subprocess.check_output(["git", "-c", "user.name=t", "-c", "user.email=t@t", *args], cwd=repo, text=True)


@pytest.fixture
def repo(tmp_path):
  root = tmp_path / "op"
  root.mkdir()
  git(root, "init", "-q")
  (root / ".gitignore").write_text(".sconsign.dblite\n.build_stamp.json\n")
  for name in OUTPUTS:
    (root / name).parent.mkdir(parents=True, exist_ok=True)
    (root / name).write_bytes(b"committed, built from older source")
  (root / "SConstruct").write_text("# source\n")
  for name in EXECUTABLES:
    os.chmod(root / name, 0o755)
  git(root, "add", "-A")
  git(root, "commit", "-qm", "base")
  return root


def scons_build(root):
  """What a successful on-device build leaves: rewritten outputs, a .sconsign, then the stamp."""
  time.sleep(0.01)
  for name in OUTPUTS:
    (root / name).write_bytes(b"built from this source " + name.encode())
  (root / SCONSIGN).write_bytes(b"signatures")
  record_build(str(root))


def test_first_boot_without_a_stamp_rebuilds(repo):
  (repo / SCONSIGN).write_bytes(b"left by someone else")
  assert invalidate_stale_outputs(str(repo))
  assert not (repo / SCONSIGN).exists()


def test_untouched_build_is_trusted(repo):
  scons_build(repo)
  assert not invalidate_stale_outputs(str(repo))
  assert (repo / SCONSIGN).exists()


def test_reset_hard_restoring_committed_outputs_forces_a_rebuild(repo):
  scons_build(repo)
  git(repo, "reset", "-q", "--hard")
  assert (repo / OUTPUTS[0]).read_bytes() == b"committed, built from older source"
  assert invalidate_stale_outputs(str(repo))
  assert not (repo / SCONSIGN).exists() and not (repo / STAMP).exists()


def test_updater_copytree_then_reset_to_the_new_revision_forces_a_rebuild(repo, tmp_path):
  scons_build(repo)
  (repo / "SConstruct").write_text("# newer source\n")
  git(repo, "commit", "-qam", "update")
  target = git(repo, "rev-parse", "HEAD").strip()
  git(repo, "checkout", "-q", "HEAD~1")
  scons_build(repo)
  staging = tmp_path / "finalized"
  shutil.copytree(repo, staging, symlinks=True)  # updated.py: .sconsign and the stamp come along
  git(staging, "reset", "-q", "--hard", target)
  assert invalidate_stale_outputs(str(staging))
  assert not (staging / SCONSIGN).exists()


@pytest.mark.parametrize("name", OUTPUTS)
def test_restoring_any_kind_of_output_at_the_same_head_forces_a_rebuild(repo, name):
  # `git checkout -- <path>` (or stash) without moving HEAD: extensionless executables and .so.N included
  scons_build(repo)
  git(repo, "checkout", "--", name)
  assert (repo / name).read_bytes() == b"committed, built from older source"
  assert invalidate_stale_outputs(str(repo))


def test_a_missing_tracked_output_forces_a_rebuild(repo):
  scons_build(repo)
  os.remove(repo / OUTPUTS[1])
  assert invalidate_stale_outputs(str(repo))


def test_no_git_means_no_trust(repo):
  scons_build(repo)
  shutil.rmtree(repo / ".git")
  assert invalidate_stale_outputs(str(repo))
