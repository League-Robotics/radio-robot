#!/usr/bin/env bash
# coverage.sh — Sprint 045 coverage harness
#
# Builds a coverage-instrumented libfirmware_host, runs the simulation tier
# against it, prints an overall source/ line coverage report (per-file table),
# and prints a second "simulatable-code" percentage that excludes CODAL-only
# files from the denominator.
#
# Usage:
#   bash tests/_infra/coverage.sh [--fail-under N]
#
#   --fail-under N   Exit non-zero if simulatable-code coverage < N%.
#
# The script uses a separate build_coverage/ directory so the standard
# build/ directory (used by the normal pytest suite) is never touched.
# build_coverage/ is matched by .gitignore's build_*/ glob.
#
# Prerequisites (auto-installed via uv):
#   cmake (system), gcovr (uv --with gcovr), pytest (uv --with pytest)
#
# Final baseline (Sprint 045, ticket 045-005): 81.5% overall / 86.2% simulatable-code
# (simulatable-code excludes CODAL-only + RatioPidController dead-code; see exclusion set below).
#
# CODAL-only exclusion set (simulatable-code denominator exclusions):
#   source/app/DebugCommandable.cpp     — HOST_BUILD stubs only; I2C handlers guarded
#   source/control/PortController.cpp   — NezhaHAL hardware I/O, not sim-exercisable
#   source/control/ServoController.cpp  — hardware PWM output, same rationale
#   source/io/real/*                    — real device drivers, absent from host lib
#   source/app/WedgeTest.cpp            — CODAL-only diagnostic (#ifndef HOST_BUILD)
#   source/control/LoopScheduler.cpp    — CODAL scheduler (MicroBit fiber APIs)
#   source/main.cpp                     — CODAL entry point, not in host lib
#   source/io/real/BenchOtosSensor.cpp  — bench-only, physical OTOS over I2C
#
# Note: source/app/SystemCommands.cpp has mixed coverage — testable paths are
# included in the simulatable denominator (only the RESET/#ifndef HOST_BUILD
# paths are unreachable, but file-granularity exclusion cannot split them).
#
# (045-002) source/control/RatioPidController.cpp — CONFIRMED DEAD CODE in the
#   live control loop and excluded from the simulatable denominator.  N13/030-010
#   removed RatioPidController::update() from MotorController::controlTick (the
#   sync-gain coupling replaced it).  A repo-wide grep finds NO call site: the
#   only references are the pid.* config keys in ConfigRegistry.cpp (kept for
#   host SET/GET compatibility), the class's own .h/.cpp, and the removal note in
#   MotorController.h.  It is therefore unreachable through the sim and excluded
#   here rather than covered by a synthetic isolation test (per ticket OQ-1 (a)).

set -euo pipefail

# ---------------------------------------------------------------------------
# Argument parsing
# ---------------------------------------------------------------------------
FAIL_UNDER=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --fail-under)
            FAIL_UNDER="$2"
            shift 2
            ;;
        *)
            echo "Unknown argument: $1" >&2
            echo "Usage: bash tests/_infra/coverage.sh [--fail-under N]" >&2
            exit 1
            ;;
    esac
done

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SIM_DIR="$SCRIPT_DIR/sim"
COV_DIR="$SIM_DIR/build_coverage"

echo "=== Sprint 045 coverage harness ==="
echo "Repo root:          $REPO_ROOT"
echo "Coverage build dir: $COV_DIR"
echo ""

# ---------------------------------------------------------------------------
# Step 1: Configure coverage-instrumented build (fresh/clean each run)
# ---------------------------------------------------------------------------
echo "--- cmake configure (coverage) ---"
rm -rf "$COV_DIR"
cmake -S "$SIM_DIR" -B "$COV_DIR" \
    -DCMAKE_CXX_FLAGS="--coverage -O0 -g" \
    -DCMAKE_SHARED_LINKER_FLAGS="--coverage"

# ---------------------------------------------------------------------------
# Step 2: Build
# ---------------------------------------------------------------------------
echo ""
echo "--- cmake build (coverage) ---"
cmake --build "$COV_DIR" --parallel

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
# Step 5a: Overall source/ coverage report (per-file table + summary)
# ---------------------------------------------------------------------------
echo ""
echo "=== Overall source/ coverage ==="
uv run --with gcovr gcovr \
    --root . \
    --filter 'source/' \
    --gcov-ignore-errors=source_not_found \
    --print-summary \
    "$COV_DIR"

# ---------------------------------------------------------------------------
# Step 5b: Simulatable-code coverage (CODAL-only files excluded)
# ---------------------------------------------------------------------------
echo ""
echo "=== Simulatable-code coverage (CODAL-only / dead-code files excluded) ==="
echo "Excluded: DebugCommandable.cpp, PortController.cpp, ServoController.cpp,"
echo "          io/real/*, WedgeTest.cpp, LoopScheduler.cpp, main.cpp, BenchOtosSensor.cpp,"
echo "          RatioPidController.cpp (045-002: confirmed dead code, no call sites)"
echo ""

SIM_SUMMARY="$(uv run --with gcovr gcovr \
    --root . \
    --filter 'source/' \
    --exclude 'source/app/DebugCommandable\.cpp' \
    --exclude 'source/control/PortController\.cpp' \
    --exclude 'source/control/ServoController\.cpp' \
    --exclude 'source/control/RatioPidController\.cpp' \
    --exclude 'source/io/real/.*' \
    --exclude 'source/app/WedgeTest\.cpp' \
    --exclude 'source/control/LoopScheduler\.cpp' \
    --exclude 'source/main\.cpp' \
    --gcov-ignore-errors=source_not_found \
    --print-summary \
    "$COV_DIR")"

echo "$SIM_SUMMARY"

# ---------------------------------------------------------------------------
# Step 6: Extract simulatable-code line % and apply --fail-under check
# ---------------------------------------------------------------------------
if [[ -n "$FAIL_UNDER" ]]; then
    # Parse "lines: XX.X% ..." from the gcovr --print-summary output
    SIM_LINE_PCT="$(echo "$SIM_SUMMARY" | grep -i '^lines:' | grep -oE '[0-9]+(\.[0-9]+)?%' | head -1 | tr -d '%')"
    if [[ -z "$SIM_LINE_PCT" ]]; then
        echo "ERROR: could not parse simulatable-code line % from gcovr output" >&2
        exit 1
    fi
    echo ""
    echo "Simulatable-code line coverage: ${SIM_LINE_PCT}%  (threshold: ${FAIL_UNDER}%)"
    # Use awk for floating-point comparison (bash arithmetic is integer-only).
    # awk exits 0 (success) when the expression is true; bash if-body runs on success.
    # "Is coverage below threshold?" — if yes, fail.
    if awk "BEGIN { exit !(${SIM_LINE_PCT} < ${FAIL_UNDER}) }"; then
        echo "FAIL: simulatable-code coverage ${SIM_LINE_PCT}% is below --fail-under ${FAIL_UNDER}%" >&2
        exit 1
    else
        echo "PASS: simulatable-code coverage ${SIM_LINE_PCT}% meets --fail-under ${FAIL_UNDER}%"
    fi
fi

echo ""
echo "=== Coverage harness complete ==="
