#!/usr/bin/env bash
# One-shot cFS dev entry point: sync cFS/ + Docker files into WSL, then drop
# into the cFS dev container shell running there (fast WSL-native build/run,
# instead of the slow Windows bind mount — see sync-to-wsl.sh for why).
#
# Usage:
#   ./wsl-dev.sh          — sync, then open a bash shell in the container
#   ./wsl-dev.sh <cmd>    — sync, then run <cmd> in the container
#                           (e.g. ./wsl-dev.sh make native_std.install)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DISTRO="Ubuntu"
WSL_DEST="cFS_Project"

"$SCRIPT_DIR/sync-to-wsl.sh"

remote_cmd="cd ~/$WSL_DEST && ./cfs-dev.sh"
for arg in "$@"; do
    remote_cmd+=" $(printf '%q' "$arg")"
done

echo "==> Entering WSL ($DISTRO) dev container"
MSYS_NO_PATHCONV=1 exec wsl.exe -d "$DISTRO" -e bash -lc "$remote_cmd"
