#!/usr/bin/env bash
# Rebuild SimulatorScene.app from the terminal, without opening the Unity Editor.
#
# Usage: ./build-mac.sh
#
# Builds into cFS_DockingSim/Builds/ first, so a failed build leaves the current
# app alone. If the app is running when the build finishes, this waits for you to
# quit it before swapping the new one in. The Unity Editor must be closed.

set -uo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
PROJECT="$ROOT/cFS_DockingSim"
APP="$PROJECT/SimulatorScene.app"
NEW_APP="$PROJECT/Builds/SimulatorScene.app"
OLD_APP="$PROJECT/Builds/SimulatorScene.previous.app"
VERSION="$(awk '/^m_EditorVersion:/ { print $2 }' "$PROJECT/ProjectSettings/ProjectVersion.txt")"
UNITY="/Applications/Unity/Hub/Editor/$VERSION/Unity.app/Contents/MacOS/Unity"
LOG="$ROOT/run_logs/unity_build_$(date +%Y%m%d_%H%M%S).log"

say()  { printf '==> %s\n' "$*"; }
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

[ -x "$UNITY" ] || fail "Unity $VERSION is not installed at $UNITY."
if pgrep -f "MacOS/Unity .*cFS_DockingSim" >/dev/null; then
    fail "The Unity Editor has this project open. Close it first."
fi

mkdir -p "$ROOT/run_logs"
rm -rf "$NEW_APP"

say "Building with Unity $VERSION at low priority, so a running sim isn't disturbed"
say "Follow along with: tail -f $LOG"
start=$SECONDS
nice -n 19 "$UNITY" -batchmode -quit -projectPath "$PROJECT" -buildTarget OSXUniversal \
    -executeMethod BuildSimulator.Mac -logFile "$LOG"
status=$?
if [ "$status" -ne 0 ] || [ ! -d "$NEW_APP" ]; then
    grep -nE 'error CS|\[BuildSimulator\]|Error building' "$LOG" | tail -n 15 >&2
    fail "Build failed (exit $status). Your current app is unchanged. Log: $LOG"
fi
say "Build finished in $((SECONDS - start))s"

if pgrep -f "$APP/Contents/MacOS/" >/dev/null; then
    say "SimulatorScene.app is running. The new build installs when you quit it."
    while pgrep -f "$APP/Contents/MacOS/" >/dev/null; do sleep 2; done
fi

# The app writes its telemetry CSVs inside the bundle, so keep them across the swap.
if [ -d "$APP" ]; then
    mkdir -p "$ROOT/run_logs/unity_telemetry"
    find "$APP" -maxdepth 1 -name 'TelemetryLog_*.csv' -exec mv {} "$ROOT/run_logs/unity_telemetry/" \;
    rm -rf "$OLD_APP"
    mv "$APP" "$OLD_APP" || fail "Could not move the old app aside."
fi
mv "$NEW_APP" "$APP" || fail "Could not install the new app. The old one is at $OLD_APP."
rm -rf "$OLD_APP"
say "Installed. Run ./run-mac.sh to use it."
