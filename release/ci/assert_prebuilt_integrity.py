#!/usr/bin/env python3
"""Mandatory integrity checks for a staged zoompilot prebuilt/release tree."""
from __future__ import annotations

import argparse
import os
import pathlib
import stat
import subprocess
import sys


MANDATORY_PATHS = (
  ".git",
  "build.json",
  "openpilot/sunnypilot/common/version.h",
  "launch_openpilot.sh",
  "openpilot/system/manager/manager.py",
  "prebuilt",
)

REQUIRED_SYMLINKS = (
  "msgq",
  "opendbc",
  "rednose",
  "teleoprtc",
  "tinygrad",
)


def fail(msg: str) -> None:
  print(f"ERROR: {msg}", file=sys.stderr)
  raise SystemExit(1)


def check_mandatory(root: pathlib.Path) -> None:
  for rel in MANDATORY_PATHS:
    path = root / rel
    if not path.exists():
      fail(f"missing mandatory path: {rel}")
  launch = root / "launch_openpilot.sh"
  mode = launch.stat().st_mode
  if not (mode & (stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)):
    fail("launch_openpilot.sh is not executable")


def check_symlinks(root: pathlib.Path) -> None:
  for rel in REQUIRED_SYMLINKS:
    path = root / rel
    if not path.is_symlink():
      # published trees sometimes materialize as dirs; require target exists either way
      if not path.exists():
        fail(f"required link/path missing: {rel}")
      continue
    target = path.resolve()
    if not target.exists():
      fail(f"symlink {rel} -> {os.readlink(path)} does not resolve")


def check_release_files(root: pathlib.Path, workspace: pathlib.Path | None) -> None:
  """Verify workspace release_files paths were staged into root (typically BUILD_DIR)."""
  if workspace is None:
    return
  script = workspace / "tools/release/release_files.py"
  if not script.is_file():
    fail(f"release_files.py not found at {script}")
  proc = subprocess.run(
    [sys.executable, str(script)],
    cwd=str(workspace),
    check=True,
    capture_output=True,
    text=True,
  )
  missing = []
  checked = 0
  for line in proc.stdout.splitlines():
    rel = line.strip()
    if not rel:
      continue
    src = workspace / rel
    if not src.exists():
      continue
    checked += 1
    dst = root / rel
    if not dst.exists():
      missing.append(rel)
  if missing:
    preview = "\n  ".join(missing[:30])
    fail(f"{len(missing)}/{checked} release_files entries missing from staged tree, e.g.:\n  {preview}")
  print(f"release_files coverage OK ({checked} existing source paths present in staged tree)")


def check_build_metadata(root: pathlib.Path) -> None:
  env = os.environ.copy()
  env["PYTHONPATH"] = str(root) + (os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else "")
  code = r"""
from openpilot.common.version import get_build_metadata, get_version
print(get_version())
print(get_build_metadata())
"""
  proc = subprocess.run(
    [sys.executable, "-c", code],
    cwd=str(root),
    env=env,
    capture_output=True,
    text=True,
  )
  if proc.returncode != 0:
    fail(
      "build metadata parse failed:\n"
      + proc.stdout
      + proc.stderr
    )
  print(proc.stdout.strip())


def check_core_files(root: pathlib.Path) -> None:
  for rel in (
    "openpilot/sunnypilot/common/version.h",
    "launch_openpilot.sh",
    "openpilot/system/manager/manager.py",
    "prebuilt",
  ):
    if not (root / rel).exists():
      fail(f"missing core path: {rel}")
  launch = root / "launch_openpilot.sh"
  mode = launch.stat().st_mode
  if not (mode & (stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)):
    fail("launch_openpilot.sh is not executable")


def main() -> int:
  p = argparse.ArgumentParser()
  p.add_argument("staged_dir", type=pathlib.Path)
  p.add_argument("--workspace", type=pathlib.Path, default=None,
                 help="source checkout used to validate release_files coverage")
  p.add_argument("--skip-release-files", action="store_true")
  p.add_argument("--skip-mandatory", action="store_true",
                 help="skip .git/build.json requirements (use for BUILD_DIR)")
  p.add_argument("--skip-metadata", action="store_true",
                 help="skip get_build_metadata() import check")
  args = p.parse_args()

  root = args.staged_dir.resolve()
  if not root.is_dir():
    fail(f"staged dir missing: {root}")

  if args.skip_mandatory:
    check_core_files(root)
  else:
    check_mandatory(root)
  check_symlinks(root)
  if not args.skip_release_files:
    check_release_files(root, args.workspace.resolve() if args.workspace else None)
  if not args.skip_metadata:
    check_build_metadata(root)
  print(f"integrity OK: {root}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
