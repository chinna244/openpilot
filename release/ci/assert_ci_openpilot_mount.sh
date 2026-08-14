#!/usr/bin/env bash
# Refuse to touch /data/openpilot unless it is the CI bind-mount staging dir.
# Protects the live device install when the runner mount namespace is missing/broken.
set -euo pipefail

BUILD_DIR="${1:-/data/openpilot}"
MARKER_NAME=".ci_build_dir"

candidates=(
  "${CI_OPENPILOT_DIR:-}"
  "/data/github/openpilot"
  "/data/media/0/github/openpilot"
)

resolve_ci_dir() {
  local c
  for c in "${candidates[@]}"; do
    [ -n "$c" ] || continue
    if [ -d "$c" ] && [ -f "$c/$MARKER_NAME" ]; then
      printf '%s\n' "$c"
      return 0
    fi
  done
  for c in "${candidates[@]}"; do
    [ -n "$c" ] || continue
    if [ -d "$c" ]; then
      printf '%s\n' "$c"
      return 0
    fi
  done
  return 1
}

if [ ! -d "$BUILD_DIR" ]; then
  echo "::error::BUILD_DIR does not exist: $BUILD_DIR"
  exit 1
fi

CI_DIR="$(resolve_ci_dir)" || {
  echo "::error::Could not locate CI openpilot staging directory"
  exit 1
}

build_id="$(stat -c '%d:%i' "$BUILD_DIR")"
ci_id="$(stat -c '%d:%i' "$CI_DIR")"

if [ "$build_id" != "$ci_id" ]; then
  echo "::error::Refusing to modify $BUILD_DIR — it is NOT the CI bind mount."
  echo "::error::BUILD_DIR inode=$build_id owner=$(stat -c '%U:%G' "$BUILD_DIR")"
  echo "::error::CI_DIR    inode=$ci_id path=$CI_DIR owner=$(stat -c '%U:%G' "$CI_DIR")"
  echo "::error::The github-runner service must unshare+bind $CI_DIR over $BUILD_DIR."
  exit 1
fi

if [ ! -f "$BUILD_DIR/$MARKER_NAME" ]; then
  # Recoverable on a correct mount: plant marker for future runs.
  if [ -w "$BUILD_DIR" ]; then
    touch "$BUILD_DIR/$MARKER_NAME"
  else
    echo "::error::CI mount looks correct by inode but $BUILD_DIR is not writable and lacks $MARKER_NAME"
    exit 1
  fi
fi

if [ ! -w "$BUILD_DIR" ]; then
  echo "::error::$BUILD_DIR is not writable by $(id -un); refusing staging"
  exit 1
fi

echo "CI bind mount OK: $BUILD_DIR -> $CI_DIR (inode $build_id) writable by $(id -un)"
