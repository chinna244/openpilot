#!/usr/bin/env python3
import json
import os
import re
import subprocess

# NOTE: Do NOT import anything here that needs be built (e.g. params)
from openpilot.common.basedir import BASEDIR
from openpilot.common.spinner import Spinner
from openpilot.common.text_window import TextWindow
from openpilot.common.hardware import HARDWARE, AGNOS

# A source build tracks some build outputs in git. SCons (Decider MD5-timestamp) never re-checks a
# target's own content, so once an update's `git reset --hard`, a checkout or a copy restores those
# files, .sconsign still vouches for them. The stamp records what the last successful build left; any
# other state discards .sconsign and every target is rebuilt (the CacheDir still serves unchanged ones).
STAMP = ".build_stamp.json"
SCONSIGN = ".sconsign.dblite"
OUTPUT_SUFFIXES = (".so", ".a", ".o", ".bin", ".signed", ".elf")
VERSIONED_SO = re.compile(r"\.so(\.\d+)+$")


def build_state(basedir: str = BASEDIR) -> dict | None:
  """HEAD and every tracked build output as (size, mtime); None when git cannot say."""
  try:
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=basedir, text=True).strip()
    staged = subprocess.check_output(["git", "ls-files", "-s", "-z"], cwd=basedir, text=True).split("\0")
  except (OSError, subprocess.CalledProcessError):
    return None
  outputs = {}
  for entry in filter(None, staged):
    meta, name = entry.split("\t", 1)
    mode = meta.split(" ", 1)[0]
    base = os.path.basename(name)
    # libraries (incl. versioned .so.N), objects, firmware, and extensionless executables (camerad, pandad, ...)
    if name.endswith(OUTPUT_SUFFIXES) or VERSIONED_SO.search(base) or (mode == "100755" and "." not in base):
      try:
        st = os.stat(os.path.join(basedir, name))
        outputs[name] = [st.st_size, st.st_mtime_ns]
      except FileNotFoundError:
        outputs[name] = None
  return {"head": head, "outputs": outputs}


def invalidate_stale_outputs(basedir: str = BASEDIR) -> bool:
  """Drop .sconsign unless the tree is exactly what the last successful build left. True if dropped."""
  try:
    with open(os.path.join(basedir, STAMP)) as f:
      recorded = json.load(f)
  except (OSError, ValueError):
    recorded = None
  state = build_state(basedir)
  if state is not None and recorded == state:
    return False
  for name in (SCONSIGN, STAMP):
    try:
      os.remove(os.path.join(basedir, name))
    except FileNotFoundError:
      pass
  return True


def record_build(basedir: str = BASEDIR) -> None:
  state = build_state(basedir)
  if state is not None:
    tmp = os.path.join(basedir, STAMP + ".tmp")
    with open(tmp, "w") as f:
      json.dump(state, f)
    os.replace(tmp, os.path.join(basedir, STAMP))


def build() -> None:
  spinner = Spinner()
  spinner.update_progress(0, 100)

  if invalidate_stale_outputs():
    print("build: tracked outputs differ from the last build of this tree; rebuilding every target")

  HARDWARE.set_power_save(False)
  if AGNOS:
    os.sched_setaffinity(0, range(8))  # ensure we can use the isolcpus cores

  # building with all cores can result in using too much memory, so retry serially
  compile_output: list[bytes] = []
  for parallelism in ([], ["-j4"], ["-j1"]):
    compile_output.clear()
    with subprocess.Popen(["scons", *parallelism], cwd=BASEDIR, env={**os.environ, "PWD": BASEDIR}, stderr=subprocess.PIPE) as scons:
      assert scons.stderr is not None

      # Read progress from stderr and update spinner
      while scons.poll() is None:
        try:
          line = scons.stderr.readline()
          if line is None:
            continue
          line = line.rstrip()

          prefix = b'progress: '
          if line.startswith(prefix):
            progress = float(line[len(prefix):])
            spinner.update_progress(100 * min(1., progress / 100.), 100.)
          elif len(line):
            compile_output.append(line)
            print(line.decode('utf8', 'replace'))
        except Exception:
          pass

      # Drain and close the pipe before retrying or returning.
      for line in scons.stderr.read().split(b'\n'):
        line = line.rstrip()
        if len(line):
          compile_output.append(line)

    if scons.returncode == 0:
      break

  if scons.returncode == 0:
    record_build()
  os.sync()

  if scons.returncode != 0:
    # Build failed log errors
    error_s = b"\n".join(compile_output).decode('utf8', 'replace')

    # Show TextWindow
    spinner.close()
    if not os.getenv("CI"):
      with TextWindow("openpilot failed to build\n \n" + error_s) as t:
        t.wait_for_exit()
    exit(1)

if __name__ == "__main__":
  build()
