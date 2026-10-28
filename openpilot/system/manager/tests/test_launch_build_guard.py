"""launch_chffrplus.sh's manager start, run as written with harmless build/manager stubs: a failed
source build must never start manager; a successful one, or a prebuilt tree, must."""
import os
import subprocess
from pathlib import Path

import pytest

from openpilot.common.basedir import BASEDIR

START = "  cd openpilot/system/manager\n"
END = "  # if broken, keep on screen error\n"


def launcher_block() -> str:
  source = Path(BASEDIR, "launch_chffrplus.sh").read_text()
  block = source[source.index(START):source.index(END)]
  assert "./build.py" in block and "./manager.py" in block
  return block


def run_block(tmp_path, build_exit: int, prebuilt: bool = False, timeout: float = 3.0):
  manager_dir = tmp_path / "openpilot/system/manager"
  manager_dir.mkdir(parents=True)
  stubs = {"build.py": f"#!/bin/sh\ntouch {tmp_path}/build-ran\nexit {build_exit}\n",
           "manager.py": f"#!/bin/sh\ntouch {tmp_path}/manager-started\n"}
  for name, body in stubs.items():
    (manager_dir / name).write_text(body)
    (manager_dir / name).chmod(0o755)
  if prebuilt:
    (tmp_path / "prebuilt").touch()
  script = f"DIR={tmp_path}\nfunction launch {{\n{launcher_block()}}}\nlaunch\n"
  try:
    result = subprocess.run(["bash", "-c", script], cwd=tmp_path, text=True, capture_output=True, timeout=timeout,
                            env={**os.environ, "PYTHONPATH": ""})
    return result.returncode, result.stdout, False
  except subprocess.TimeoutExpired as e:
    return None, (e.stdout or b"").decode() if isinstance(e.stdout, bytes) else (e.stdout or ""), True


@pytest.mark.parametrize("build_exit", [1, 2, 137])
def test_failed_build_never_starts_manager(tmp_path, build_exit):
  _, stdout, held = run_block(tmp_path, build_exit)
  assert (tmp_path / "build-ran").exists()
  assert not (tmp_path / "manager-started").exists()
  assert held, "a failed build keeps the error on screen instead of exiting into a restart"
  assert "build failed" in stdout


def test_successful_build_starts_manager(tmp_path):
  code, _, held = run_block(tmp_path, 0)
  assert not held and code == 0
  assert (tmp_path / "build-ran").exists() and (tmp_path / "manager-started").exists()


def test_prebuilt_tree_skips_the_build_and_starts_manager(tmp_path):
  code, _, held = run_block(tmp_path, 1, prebuilt=True)
  assert not held and code == 0
  assert not (tmp_path / "build-ran").exists() and (tmp_path / "manager-started").exists()
