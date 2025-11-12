#!/bin/bash

# Compile All Examples Script for MAX30001 Arduino Library
# This script compiles all examples to verify they build without errors

set -e

FQBN="arduino:renesas_uno:minima"
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "  MAX30001 Library - Compile All Examples"
echo "=========================================="
echo ""
echo "Board: Arduino Uno R4 Minima (${FQBN})"
echo "Repository: ${REPO_ROOT}"
echo ""

# Array of example directories
EXAMPLES=(
    "examples/Example01_BasicECGBioZ/Example01_BasicECGBioZ.ino"
    "examples/Example02_RtoRDetection/Example02_RtoRDetection.ino"
    "examples/Example03_LeadOff/Example03_LeadOff.ino"
    "examples/Example04_InterruptDriven/Example04_InterruptDriven.ino"
    "examples/Example05_AdvancedConfig/Example05_AdvancedConfig.ino"
)

TOTAL=${#EXAMPLES[@]}
PASSED=0
FAILED=0
FAILED_EXAMPLES=()

echo "Found ${TOTAL} examples to compile:"
for example in "${EXAMPLES[@]}"; do
    echo "  - ${example}"
done
echo ""

# Compile each example
for example in "${EXAMPLES[@]}"; do
    echo "────────────────────────────────────────"
    echo "Compiling: ${example}"
    echo "────────────────────────────────────────"
    
    if arduino-cli compile --fqbn "${FQBN}" "${REPO_ROOT}/${example}" 2>&1; then
        echo "✓ SUCCESS: ${example}"
        ((PASSED++))
    else
        echo "✗ FAILED: ${example}"
        ((FAILED++))
        FAILED_EXAMPLES+=("${example}")
    fi
    echo ""
done

# Summary
echo "=========================================="
echo "  Compilation Summary"
echo "=========================================="
echo "Total Examples:    ${TOTAL}"
echo "Passed:            ${PASSED}"
echo "Failed:            ${FAILED}"
echo ""

if [ ${FAILED} -eq 0 ]; then
    echo "✓ All examples compiled successfully!"
    exit 0
else
    echo "✗ Some examples failed to compile:"
    for example in "${FAILED_EXAMPLES[@]}"; do
        echo "  - ${example}"
    done
    exit 1
fi
