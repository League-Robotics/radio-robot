#!/usr/bin/env bash
# coverage.sh — Sprint 038 coverage harness
#
# Builds a coverage-instrumented libfirmware_host, runs the simulation tier
# against it, and prints the overall source/ line coverage via gcovr.
#
# Usage:
#   bash tests/_infra/coverage.sh
#
# The script uses a separate build_coverage/ directory so the standard
# build/ directory (used by the normal pytest suite) is never touched.
#
# Prerequisites (auto-installed via uv):
#   cmake (system), gcovr (uv --with gcovr)
#
# Confirmed baseline (Sprint 038 Phase 0): ~73% line coverage over source/.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SIM_DIR="$SCRIPT_DIR/sim"
COV_DIR="$SIM_DIR/build_coverage"

echo "=== Sprint 038 coverage harness ==="
echo "Repo root:          $REPO_ROOT"
echo "Coverage build dir: $COV_DIR"
echo ""

# ---------------------------------------------------------------------------
# Step 1: Configure coverage-instrumented build
# ---------------------------------------------------------------------------
echo "--- cmake configure (coverage) ---"
cmake -S "$SIM_DIR" -B "$COV_DIR" \
    -DCMAKE_CXX_FLAGS="--coverage -O0 -g" \
    -DCMAKE_SHARED_LINKER_FLAGS="--coverage"

# ---------------------------------------------------------------------------
# Step 2: Build
# ---------------------------------------------------------------------------
echo ""
echo "--- cmake build (coverage) ---"
cmake --build "$COV_DIR" -- -j4

# ---------------------------------------------------------------------------
# Step 3: Determine platform lib name
# ---------------------------------------------------------------------------
if [[ "$(uname)" == "Darwin" ]]; then
    LIBNAME="libfirmware_host.dylib"
else
    LIBNAME="libfirmware_host.so"
fi
FIRMWARE_LIB="$COV_DIR/$LIBNAME"

if [[ ! -f "$FIRMWARE_LIB" ]]; then
    echo "ERROR: coverage lib not found at $FIRMWARE_LIB" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# Step 4: Run simulation tier against the instrumented lib
# ---------------------------------------------------------------------------
echo ""
echo "--- running simulation tier with coverage-instrumented lib ---"
echo "    FIRMWARE_HOST_LIB=$FIRMWARE_LIB"
cd "$REPO_ROOT"
FIRMWARE_HOST_LIB="$FIRMWARE_LIB" \
    uv run --with pytest python -m pytest tests/simulation -q

# ---------------------------------------------------------------------------
# Step 5: Report coverage
# ---------------------------------------------------------------------------
echo ""
echo "--- gcovr coverage report ---"
uv run --with gcovr gcovr \
    --root source \
    --print-summary \
    "$COV_DIR"

echo ""
echo "=== Coverage harness complete ==="
