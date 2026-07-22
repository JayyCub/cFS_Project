#!/usr/bin/env bash
# Push the cFS source tree + Docker build files into the WSL Ubuntu filesystem
# so cfs-dev.sh can build/run there instead of over a Windows bind mount.
#
# Bind-mounting a Windows path into Docker Desktop's Linux VM goes through a
# slow cross-OS filesystem translation layer; a WSL-native path avoids that
# entirely. This only copies what's needed to build (cFS/, Dockerfile,
# cfs-dev.sh) — everything else (Unity sim, git history, docs) stays on the
# Windows side, where you keep editing.
#
# Usage: ./sync-to-wsl.sh
# Then, from a WSL Ubuntu shell:
#   cd ~/cFS_Project && ./cfs-dev.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DISTRO="Ubuntu"
WSL_DEST="cFS_Project"
WSL_SRC="/mnt${SCRIPT_DIR}"

echo "==> Syncing cFS/ + Dockerfile + cfs-dev.sh into WSL ($DISTRO:~/$WSL_DEST)"

MSYS_NO_PATHCONV=1 wsl.exe -d "$DISTRO" -e bash -c "
    set -euo pipefail
    mkdir -p ~/$WSL_DEST
    rsync -a --delete --exclude 'build-*' '$WSL_SRC/cFS/' ~/$WSL_DEST/cFS/
    cp '$WSL_SRC/Dockerfile' '$WSL_SRC/cfs-dev.sh' ~/$WSL_DEST/
    sed -i 's/\r$//' ~/$WSL_DEST/cfs-dev.sh ~/$WSL_DEST/Dockerfile
    chmod +x ~/$WSL_DEST/cfs-dev.sh
"

echo "==> Done. From a WSL Ubuntu shell, run:"
echo "      cd ~/$WSL_DEST && ./cfs-dev.sh"
