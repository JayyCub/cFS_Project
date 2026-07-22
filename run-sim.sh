#!/usr/bin/env bash
# One-shot build+run for the standard native_std dev loop:
#   ./wsl-dev.sh
#   make native_std.install
#   cd build-native_std/exe/cpu1
#   ./core-cpu1
# combined into a single command, with the full run captured to run_logs/
# so it can be grepped/reviewed afterward instead of scrolling terminal
# history.
#
# Usage: ./run-sim.sh
# Ctrl+C stops the sim, same as running it manually.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="$SCRIPT_DIR/run_logs"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/run_$(date +%Y%m%d_%H%M%S).log"

echo "==> Logging full run to $LOG"

"$SCRIPT_DIR/wsl-dev.sh" bash -c \
    'make native_std.install && cd build-native_std/exe/cpu1 && exec ./core-cpu1' \
    2>&1 | tee "$LOG"
