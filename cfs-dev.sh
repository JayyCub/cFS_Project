#!/usr/bin/env bash
# Launch the cFS development container.
#
# Usage:
#   ./cfs-dev.sh          — drops you into a bash shell inside the container
#   ./cfs-dev.sh <cmd>    — runs <cmd> instead (e.g. ./cfs-dev.sh make native_std.install)
#
# The cFS source tree is mounted at /cfs inside the container so edits made
# on the Mac side are immediately visible without rebuilding the image.
#
# UDP ports:
#   5005 (host → container) — Unity sends telemetry to cFS here
#   5006 (container → host) — cFS sends commands to Unity via host.docker.internal
#   1234 (host → container) — CI_LAB command uplink
#   2234 (container → host) — TO_LAB telemetry downlink; no -p needed, goes via host.docker.internal

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
IMAGE="cfs-dev"

# Build the image if it doesn't exist yet
if ! docker image inspect "$IMAGE" &>/dev/null; then
    echo "==> Building $IMAGE image..."
    docker build -t "$IMAGE" "$SCRIPT_DIR"
fi

echo "==> Starting cFS dev container (cFS source mounted at /cfs)"

# MSYS_NO_PATHCONV avoids Git Bash mangling the /cfs container-side path (and
# the /bin/bash default command) into a Windows path before docker sees it.
MSYS_NO_PATHCONV=1 docker run --rm -it \
    --name cfs-dev \
    -v "$SCRIPT_DIR/cFS:/cfs" \
    -p 5005:5005/udp \
    -p 1234:1234/udp \
    --sysctl fs.mqueue.msg_max=256 \
    --add-host=host.docker.internal:host-gateway \
    "$IMAGE" \
    "${@:-/bin/bash}"
