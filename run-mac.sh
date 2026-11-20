#!/usr/bin/env bash
# One-command simulator launch on macOS: cFS in Docker, the ground console, and
# the SimulatorScene.app Unity build. Ctrl+C (or quitting the app) stops it all.
#
# Usage: ./run-mac.sh
#
# Each stage waits for a real signal from the one before it, because the order
# matters:
#   1. cFS first. Unity free-runs when nothing answers it, so a sim started early
#      drifts away from its start pose before cFS takes over.
#   2. Ground console once CI_LAB is listening. It sends the TO_LAB enable command
#      1.5 s after it starts, and if cFS isn't up yet that UDP packet is just lost.
#   3. Unity once telemetry is flowing, then wait for lock-step and a NAV solution.
#
# Guidance stays inhibited until you press GO on the console.
# Logs for each run go to run_logs/.

set -uo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
APP="$ROOT/cFS_DockingSim/SimulatorScene.app"
CONTAINER="cfs-dev"
APP_BUNDLE_ID="com.DefaultCompany.3D-Project"
CONSOLE_URL="http://localhost:8080"

BUILD_TIMEOUT_S=1800    # a from-scratch cFS build takes 10+ minutes
DOCKER_TIMEOUT_S=120
TELEMETRY_TIMEOUT_S=30
ENGAGE_TIMEOUT_S=120    # the app loads a ~200 MB station model
NAV_TIMEOUT_S=60

LOG_DIR="$ROOT/run_logs"
STAMP="$(date +%Y%m%d_%H%M%S)"
CFS_LOG="$LOG_DIR/run_${STAMP}_cfs.log"
CONSOLE_LOG="$LOG_DIR/run_${STAMP}_console.log"
UNITY_LOG="$LOG_DIR/run_${STAMP}_unity.log"

CFS_PID=""
CONSOLE_PID=""
APP_PID=""
CLEANED_UP=0

say()  { printf '==> %s\n' "$*"; }
warn() { printf 'WARNING: %s\n' "$*" >&2; }

# fail <message> [log file to show the end of]
fail() {
    printf '\nERROR: %s\n' "$1" >&2
    if [ -n "${2:-}" ] && [ -f "$2" ]; then
        printf '\n--- last lines of %s ---\n' "$2" >&2
        tail -n 25 "$2" >&2
    fi
    exit 1
}

cleanup() {
    [ "$CLEANED_UP" = 1 ] && return
    CLEANED_UP=1
    [ -n "$CFS_PID$CONSOLE_PID$APP_PID" ] || return
    trap '' INT TERM
    echo
    say "Shutting down"
    [ -n "$APP_PID" ] && kill "$APP_PID" 2>/dev/null
    [ -n "$CONSOLE_PID" ] && kill "$CONSOLE_PID" 2>/dev/null
    if [ -n "$CFS_PID" ]; then
        docker stop -t 5 "$CONTAINER" >/dev/null 2>&1
        wait "$CFS_PID" 2>/dev/null
    fi
    say "Stopped. Logs: $LOG_DIR/run_${STAMP}_*.log"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Fails the run as soon as something that should be up has died.
check_alive() {
    if [ -n "$CFS_PID" ] && ! kill -0 "$CFS_PID" 2>/dev/null; then
        CFS_PID=""
        fail "cFS stopped unexpectedly." "$CFS_LOG"
    fi
    if [ -n "$CONSOLE_PID" ] && ! kill -0 "$CONSOLE_PID" 2>/dev/null; then
        CONSOLE_PID=""
        fail "The ground console stopped unexpectedly." "$CONSOLE_LOG"
    fi
}

# wait_until <description> <timeout s> <log for errors> <command...>
wait_until() {
    local what="$1" timeout="$2" log="$3" start=$SECONDS next_note=30
    shift 3
    until "$@"; do
        check_alive
        if [ $((SECONDS - start)) -ge "$timeout" ]; then
            fail "Timed out after ${timeout}s waiting for $what." "$log"
        fi
        if [ $((SECONDS - start)) -ge "$next_note" ]; then
            say "Still waiting for $what ($((SECONDS - start))s)"
            next_note=$((next_note + 30))
        fi
        sleep 1
    done
}

# Only counts a socket bound to the port. lsof also lists sockets aimed at it, and
# Docker Desktop leaves one of those behind on 5006 after every run.
port_owner() {  # port_owner <TCP|UDP> <port>
    lsof -nP "-i$1:$2" 2>/dev/null | awk -v port=":$2" '
        NR > 1 {
            split($9, ends, "->")
            if (substr(ends[1], length(ends[1]) - length(port) + 1) == port) {
                print $1 " (PID " $2 ")"
                exit
            }
        }'
}

require_free() {  # require_free <TCP|UDP> <port> <what needs it>
    local owner
    owner="$(port_owner "$1" "$2")"
    [ -z "$owner" ] || fail "$1 port $2 is in use by $owner, and $3 needs it. Close that first (Unity Editor in Play mode, or an earlier console, are the usual causes)."
}

docker_up()  { docker info >/dev/null 2>&1; }
cfs_ready()  {
    grep -q 'entering OPERATIONAL state' "$CFS_LOG" &&
    grep -q 'CI_LAB listening' "$CFS_LOG" &&
    grep -q 'SIM_IO: listening' "$CFS_LOG" &&
    grep -q 'Set LC state command: new state = 1' "$CFS_LOG"   # fault protection armed
}
console_up() { curl -fsS -o /dev/null --max-time 2 "$CONSOLE_URL/state" 2>/dev/null; }
engaged()    { grep -q 'lock-step ENGAGED' "$UNITY_LOG" 2>/dev/null; }
nav_valid()  { curl -fsS --max-time 2 "$CONSOLE_URL/state" 2>/dev/null | grep -q '"RELNAV"'; }
app_pid()    { pgrep -f "$APP/Contents/MacOS/" | head -n 1; }
app_running() { [ -n "$(app_pid)" ]; }

# The console's own enable can still be dropped, so resend until telemetry appears.
telemetry_flowing() {
    curl -fsS --max-time 2 "$CONSOLE_URL/state" 2>/dev/null | grep -q '"last_rx": "'
}

# --- Preflight -------------------------------------------------------------

[ "$(uname -s)" = "Darwin" ] || fail "This script is for macOS. On Windows, use ./run-sim.sh."
[ -d "$APP" ] || fail "No Unity build at $APP. Build it from Unity first (File > Build Profiles)."
[ -f "$ROOT/cFS/Makefile" ] || fail "cFS/ is empty. Run: git submodule update --init --recursive"
python3 -c 'import sys; sys.exit(sys.version_info < (3, 10))' 2>/dev/null ||
    fail "ground_console.py needs Python 3.10 or newer."
[ -z "$(app_pid)" ] || fail "SimulatorScene.app is already running. Quit it first."
require_free UDP 5006 "SimulatorScene.app"
require_free UDP 2234 "the ground console"
require_free TCP 8080 "the ground console"

mkdir -p "$LOG_DIR"

if ! docker_up; then
    say "Starting Docker"
    open -a Docker || fail "Could not start Docker Desktop. Is it installed?"
    wait_until "Docker to start" "$DOCKER_TIMEOUT_S" "" docker_up
fi

if docker container inspect "$CONTAINER" >/dev/null 2>&1; then
    say "Replacing the existing $CONTAINER container"
    docker rm -f "$CONTAINER" >/dev/null || fail "Could not remove the old $CONTAINER container."
fi

# --- 1. cFS ----------------------------------------------------------------

say "Building and starting cFS (a first build takes 10+ minutes)"
say "Follow along with: tail -f $CFS_LOG"
"$ROOT/cfs-dev.sh" bash -c \
    'make native_std.install && cd build-native_std/exe/cpu1 && exec ./core-cpu1' \
    </dev/null >"$CFS_LOG" 2>&1 &
CFS_PID=$!
wait_until "cFS to boot" "$BUILD_TIMEOUT_S" "$CFS_LOG" cfs_ready
say "cFS is up"

# --- 2. Ground console -----------------------------------------------------

say "Starting the ground console"
(cd "$ROOT" && exec python3 -u ground_console.py) </dev/null >"$CONSOLE_LOG" 2>&1 &
CONSOLE_PID=$!
wait_until "the ground console" 15 "$CONSOLE_LOG" console_up

start=$SECONDS
next_enable=$((SECONDS + 5))
until telemetry_flowing; do
    check_alive
    [ $((SECONDS - start)) -lt "$TELEMETRY_TIMEOUT_S" ] ||
        fail "No telemetry from cFS after ${TELEMETRY_TIMEOUT_S}s." "$CONSOLE_LOG"
    if [ "$SECONDS" -ge "$next_enable" ]; then
        curl -fsS -o /dev/null --max-time 2 -H 'Content-Type: application/json' \
            -d '{"cmd":"to_enable"}' "$CONSOLE_URL/cmd"
        next_enable=$((SECONDS + 5))
    fi
    sleep 1
done
say "Telemetry is flowing"
open "$CONSOLE_URL"

# --- 3. Unity --------------------------------------------------------------

# Unity saves the window size on quit and reuses it, so tiling the app beside the
# browser once would leave it stuck at that shape. Always start at native resolution.
for key in "Screenmanager Resolution Width" "Screenmanager Resolution Height" \
           "Screenmanager Resolution Use Native"; do
    defaults delete "$APP_BUNDLE_ID" "$key" 2>/dev/null
done

say "Opening SimulatorScene.app"
open -n "$APP" --args -logFile "$UNITY_LOG" || fail "Could not open $APP."
wait_until "the app to launch" 15 "" app_running
APP_PID="$(app_pid)"

wait_until "Unity lock-step" "$ENGAGE_TIMEOUT_S" "$UNITY_LOG" engaged
say "Lock-step engaged"

# NAV can legitimately take longer from far away, so this one only warns.
start=$SECONDS
until nav_valid || [ $((SECONDS - start)) -ge "$NAV_TIMEOUT_S" ]; do
    check_alive
    sleep 1
done
if nav_valid; then
    say "NAV is valid"
else
    warn "NAV not valid after ${NAV_TIMEOUT_S}s. Check the NAV panel before pressing GO."
fi

echo
say "Ready. Press GO at $CONSOLE_URL (or run: python3 gnc_cmd.py go)"
say "Press Ctrl+C here, or quit the app, to stop everything."

# --- Run until something ends ----------------------------------------------

while kill -0 "$APP_PID" 2>/dev/null; do
    check_alive
    sleep 1
done
APP_PID=""
say "SimulatorScene.app closed"
