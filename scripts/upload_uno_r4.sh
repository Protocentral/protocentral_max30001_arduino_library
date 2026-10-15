#!/usr/bin/env bash
#
# Compile and upload an example to a connected Arduino UNO R4 Minima.
#
# Defaults to example 01.OpenView. Auto-detects the board's serial port, and
# auto-installs the required core (arduino:renesas_uno) if it is missing.
#
# Usage:
#   scripts/upload_uno_r4.sh                 # build + upload 01.OpenView
#   scripts/upload_uno_r4.sh 02.HeartRateRtoR
#   scripts/upload_uno_r4.sh 01.OpenView /dev/cu.usbmodem31201
#   PORT=/dev/cu.usbmodem31201 scripts/upload_uno_r4.sh
#   MONITOR=1 scripts/upload_uno_r4.sh       # open the serial monitor after upload
#
# Requires: arduino-cli (https://arduino.github.io/arduino-cli/)
#
# Copyright (c) 2026 ProtoCentral - MIT License
#
# 01.OpenView sends binary packets only; use MONITOR=1 with a text example.

set -euo pipefail

FQBN="arduino:renesas_uno:minima"
CORE="arduino:renesas_uno"
BAUD="${BAUD:-115200}"

# Repo root = parent of this script's directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

EXAMPLE="${1:-01.OpenView}"
SKETCH_DIR="$REPO_ROOT/examples/$EXAMPLE"

# --- checks --------------------------------------------------------------------

if ! command -v arduino-cli >/dev/null 2>&1; then
    echo "ERROR: arduino-cli not found. Install it: https://arduino.github.io/arduino-cli/" >&2
    exit 1
fi

if [ ! -d "$SKETCH_DIR" ]; then
    echo "ERROR: example '$EXAMPLE' not found at $SKETCH_DIR" >&2
    echo "Available examples:" >&2
    ls -1 "$REPO_ROOT/examples" >&2
    exit 1
fi

# --- core ----------------------------------------------------------------------

if ! arduino-cli core list 2>/dev/null | grep -q "^${CORE}"; then
    echo ">> Installing core $CORE ..."
    arduino-cli core update-index
    arduino-cli core install "$CORE"
fi

# --- port detection ------------------------------------------------------------
# Priority: $2 arg > $PORT env > auto-detect a board reporting the R4 Minima FQBN.

PORT="${2:-${PORT:-}}"

if [ -z "$PORT" ]; then
    PORT="$(arduino-cli board list 2>/dev/null | awk -v fqbn="$FQBN" '$0 ~ fqbn {print $1; exit}')"
fi

if [ -z "$PORT" ]; then
    echo "ERROR: no UNO R4 Minima detected. Connect the board, or pass the port:" >&2
    echo "       $0 $EXAMPLE /dev/cu.usbmodemXXXX" >&2
    echo "Detected serial ports:" >&2
    arduino-cli board list 2>/dev/null >&2
    exit 1
fi

echo ">> Example : $EXAMPLE"
echo ">> Board   : $FQBN"
echo ">> Port    : $PORT"

# --- compile -------------------------------------------------------------------

echo ">> Compiling ..."
arduino-cli compile \
    --fqbn "$FQBN" \
    --library "$REPO_ROOT" \
    "$SKETCH_DIR"

# --- upload --------------------------------------------------------------------

echo ">> Uploading ..."
arduino-cli upload \
    --fqbn "$FQBN" \
    --port "$PORT" \
    "$SKETCH_DIR"

echo ">> Done."

# --- optional serial monitor ---------------------------------------------------

if [ "${MONITOR:-0}" = "1" ]; then
    echo ">> Opening serial monitor at ${BAUD} baud (Ctrl-C to exit) ..."
    arduino-cli monitor --port "$PORT" --config "baudrate=${BAUD}"
fi
