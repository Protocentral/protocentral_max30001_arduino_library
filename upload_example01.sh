#!/bin/bash

# Compile and upload Example01_BasicECGBioZ to an attached Arduino Uno R4 Minima
#
# Usage: ./upload_example01.sh [port]
#   port  Serial port of the board (default: auto-detected via arduino-cli)
#
# The sketch is compiled against this checkout of the library (--library),
# so local changes are tested even if another copy is installed.

set -e

FQBN="arduino:renesas_uno:minima"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH="${REPO_ROOT}/examples/Example01_BasicECGBioZ"

PORT="${1:-}"
if [ -z "${PORT}" ]; then
    PORT=$(arduino-cli board list | awk -v fqbn="${FQBN}" '$0 ~ fqbn {print $1; exit}')
fi

if [ -z "${PORT}" ]; then
    echo "✗ No Uno R4 Minima (${FQBN}) found. Connect the board or pass the port:"
    echo "    $0 /dev/cu.usbmodemXXXX"
    exit 1
fi

echo "=========================================="
echo "  MAX30001 Library - Upload Example01"
echo "=========================================="
echo "Board:  Arduino Uno R4 Minima (${FQBN})"
echo "Port:   ${PORT}"
echo "Sketch: ${SKETCH}"
echo ""

echo "Compiling..."
arduino-cli compile --fqbn "${FQBN}" --library "${REPO_ROOT}" "${SKETCH}"

echo ""
echo "Uploading..."
arduino-cli upload --fqbn "${FQBN}" --port "${PORT}" "${SKETCH}"

echo ""
echo "✓ Uploaded. Open ProtoCentral OpenView at 57600 baud on ${PORT}."
