#!/usr/bin/env python3
"""Write build.json for a staged/published prebuilt tree."""
from __future__ import annotations

import argparse
import json
import pathlib
import sys


def main() -> int:
  p = argparse.ArgumentParser()
  p.add_argument("output_dir", type=pathlib.Path)
  p.add_argument("--channel", required=True)
  p.add_argument("--version", required=True)
  p.add_argument("--git-commit", required=True)
  p.add_argument("--git-origin", required=True)
  p.add_argument("--git-commit-date", required=True)
  p.add_argument("--build-style", default="prebuilt")
  args = p.parse_args()

  root = args.output_dir.resolve()
  changelog = root / "CHANGELOG.md"
  release_notes = "unknown"
  if changelog.is_file():
    release_notes = changelog.read_text(errors="replace").split("\n\n", 1)[0]

  meta = {
    "channel": args.channel,
    "openpilot": {
      "version": args.version,
      "release_notes": release_notes,
      "git_commit": args.git_commit,
      "git_origin": args.git_origin,
      "git_commit_date": args.git_commit_date,
      "build_style": args.build_style,
    },
  }
  out = root / "build.json"
  out.write_text(json.dumps(meta, indent=2) + "\n")
  print(f"wrote {out}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
