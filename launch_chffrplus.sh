#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

# the last build that drove clean, kept for Roll back (system/updated/updated.py reads the same path)
PREVIOUS="/data/openpilot_previous"
ROLLED_BACK="/data/openpilot_rolled_back"

# Identity of the current source tree: HEAD commit + uncommitted diffs
# (including submodule commits/diffs). Used to skip rebuilds when nothing changed.
# --no-optional-locks: reading must not write to .git.
function source_fingerprint {
  (
    set -o pipefail
    {
      git --no-optional-locks -C "$DIR" rev-parse HEAD &&
      git --no-optional-locks -C "$DIR" diff HEAD &&
      git --no-optional-locks -C "$DIR" submodule foreach --recursive --quiet \
        'echo "$displaypath $(git rev-parse HEAD)"; git --no-optional-locks diff HEAD'
    } | sha256sum
  ) 2>/dev/null
}

function agnos_init {
  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta
  rm -f /data/scons_cache/config.lock

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ "$(< /VERSION)" != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/openpilot/common/hardware/comma/agnos.py"
    MANIFEST="$DIR/openpilot/system/hardware/comma/agnos.json"
    if "$AGNOS_PY" --verify "$MANIFEST"; then
      sudo reboot
    fi
    while true; do
      "$DIR/openpilot/common/hardware/comma/updater" "$AGNOS_PY" "$MANIFEST"
    done
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f "$DIR/.git/index.lock"

  # Roll back (settings or sunnylink sets UpdaterRollback): swap the kept previous build back in
  # and never install the build we rolled back from again
  if [ "$(cat /data/params/d/UpdaterRollback 2>/dev/null)" == "1" ]; then
    rm -f /data/params/d/UpdaterRollback
    if [ ! -d "$PREVIOUS" ]; then
      echo "no previous build, not rolling back"
    elif [ -n "$(git --no-optional-locks -C "$DIR" status --porcelain --untracked-files=no)" ]; then
      echo "${DIR} has local changes, not rolling back"
    elif [ -f /AGNOS ] && [ "$(unset AGNOS_VERSION; source "$PREVIOUS/launch_env.sh"; echo -n "$AGNOS_VERSION")" != "$(< /VERSION)" ]; then
      # the AGNOS swap back would need a download, which may not be possible where the car is
      echo "previous build needs another AGNOS, not rolling back"
    else
      echo "Rolling back to the previous build"
      LAUNCHER_LOCATION="${BASH_SOURCE[0]}"
      ROLLED_BACK_COMMIT="$(git --no-optional-locks -C "$DIR" rev-parse HEAD)"
      sudo rm -rf "$STAGING_ROOT" "$ROLLED_BACK"
      if mv "$DIR" "$ROLLED_BACK"; then
        if mv "$PREVIOUS" "$DIR"; then
          echo -n "$ROLLED_BACK_COMMIT" > /data/params/d/UpdaterSkipCommit
          rm -f "${PREVIOUS}_unproven"
          # a build from before Roll back can't skip that commit, so it must not update at all
          grep -q UpdaterSkipCommit "$DIR/openpilot/system/updated/updated.py" || echo -n "1" > /data/params/d/DisableUpdates
          cd "$DIR"
          sudo rm -rf "$ROLLED_BACK" &
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        fi
        mv "$ROLLED_BACK" "$DIR"
      fi
    fi
  fi

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find "${DIR}/.git" -newer "${DIR}/.overlay_init" | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv "$DIR" /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" "$DIR"
          cd "$DIR"
          # releases before the openpilot/ move can't swap AGNOS on newer AGNOS, use ours
          [ ! -L system/hardware/tici/agnos.json ] && /data/safe_staging/old_openpilot/openpilot/common/hardware/comma/agnos.py --verify system/hardware/tici/agnos.json && sudo reboot

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # Keep the build an update replaced as the rollback copy, but only one that drove clean
  # (system/updated/proven.py), so two bad updates in a row still roll back to a working build.
  # With no copy yet, keep it marked unproven.
  OLD="${STAGING_ROOT}/old_openpilot"
  if [ -d "$OLD" ]; then
    OLD_COMMIT="$(git --no-optional-locks -C "$OLD" rev-parse HEAD 2>/dev/null)"
    if [ -n "$OLD_COMMIT" ] && [ "$OLD_COMMIT" == "$(cat /data/params/d/UpdaterProvenCommit 2>/dev/null)" ]; then
      sudo rm -rf "$PREVIOUS" && mv "$OLD" "$PREVIOUS" && rm -f "${PREVIOUS}_unproven"
    elif [ ! -d "$PREVIOUS" ]; then
      mv "$OLD" "$PREVIOUS" && touch "${PREVIOUS}_unproven"
    fi
  fi

  # handle pythonpath
  ln -sfn "$(pwd)" /data/pythonpath
  export PYTHONPATH="$PWD"

  # submodule package symlinks for PYTHONPATH imports on device.
  # on PC these come from editable installs via pyproject.toml / uv.
  ln -sfn msgq_repo/msgq msgq
  ln -sfn opendbc_repo/opendbc opendbc
  ln -sfn rednose_repo/rednose rednose
  ln -sfn teleoprtc_repo/teleoprtc teleoprtc
  ln -sfn tinygrad_repo/tinygrad tinygrad
  ln -sfn jetlink_repo/jetlink jetlink

  # hardware specific init
  if [ -f /AGNOS ]; then
    # with Jetlink on, draw nothing from its USB-C port from boot (docs/porting.md in jetlink)
    sudo -n bash jetlink/comma/jetlink-root.sh boot /data/params/d/JetlinkLink &
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # start manager
  cd openpilot/system/manager
  # Skip build.py when this exact source tree was already built here. The `prebuilt` file
  # alone skips it only where no build was ever recorded (or git cannot be read): once a
  # build is recorded, a pull or an edit rebuilds even with `prebuilt` present, so quick
  # boot cannot leave stale binaries or panda firmware.
  BUILT_SOURCE="$DIR/.built_source"
  if ! CURRENT_SOURCE="$(source_fingerprint)"; then
    CURRENT_SOURCE=""
  fi
  SKIP_BUILD=""
  if [ -n "$CURRENT_SOURCE" ] && [ "$(cat "$BUILT_SOURCE" 2>/dev/null)" == "$CURRENT_SOURCE" ]; then
    SKIP_BUILD=1
  elif [ -f "$DIR/prebuilt" ] && { [ -z "$CURRENT_SOURCE" ] || [ ! -f "$BUILT_SOURCE" ]; }; then
    SKIP_BUILD=1
  fi
  if [ -z "$SKIP_BUILD" ]; then
    ./build.py
    if [ $? -eq 0 ] && [ -n "$CURRENT_SOURCE" ]; then
      echo "$CURRENT_SOURCE" > "$BUILT_SOURCE"
    fi
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
