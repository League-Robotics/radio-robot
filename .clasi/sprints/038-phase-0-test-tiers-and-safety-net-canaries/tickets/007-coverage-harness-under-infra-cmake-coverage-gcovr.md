---
id: '007'
title: Coverage harness under _infra/ (cmake --coverage + gcovr)
status: done
use-cases:
- SUC-006
depends-on:
- '003'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Coverage harness under _infra/ (cmake --coverage + gcovr)

## Description

Create `tests/_infra/coverage.sh` — a shell script (and optionally a `justfile` target)
that builds a coverage-instrumented `libfirmware_host`, runs the simulation tier against
it, and prints overall `source/` line coverage via gcovr. This makes the 85% coverage
goal measurable and reproducible from a single command.

No hard threshold is enforced yet (source/ is not reorganized). The goal is a working,
repeatable measurement.

## Proven-working recipe (from sprint brief)

The following has been confirmed to work:

```bash
# Step 1: Configure coverage build (separate dir to avoid tainting standard build)
cmake -S tests/_infra/sim \
      -B tests/_infra/sim/build_coverage \
      -DCMAKE_CXX_FLAGS="--coverage -O0 -g" \
      -DCMAKE_SHARED_LINKER_FLAGS="--coverage"

# Step 2: Build coverage-instrumented lib
cmake --build tests/_infra/sim/build_coverage

# Step 3: Run simulation tier against the instrumented lib
# The firmware.py Sim wrapper reads FIRMWARE_HOST_LIB env var (or equivalent)
# to locate the shared lib. Check how firmware.py selects the lib path:
# - If it uses a fixed relative path, pass via env var override.
# - If it uses a pytest fixture arg, pass via conftest env.
FIRMWARE_HOST_LIB=tests/_infra/sim/build_coverage/libfirmware_host.so \
    uv run --with pytest python -m pytest tests/simulation -q

# Step 4: Report coverage
uv run --with gcovr gcovr \
    --root source \
    --print-summary \
    tests/_infra/sim/build_coverage

# (gcovr 8.6 is confirmed installable via uv --with gcovr)
```

The programmer must verify the exact env var or mechanism that `firmware.py`'s `Sim`
class uses to locate `libfirmware_host`. If it uses a hard-coded relative path rather
than an env var, add `FIRMWARE_HOST_LIB` env var support to `firmware.py`'s ctypes
`cdll.LoadLibrary` call (guarded: if env var is set, use it; else use the default path).

## `coverage.sh` script structure

```bash
#!/usr/bin/env bash
set -euo pipefail

# Repo root is two levels up from tests/_infra/
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
SIM_DIR="$SCRIPT_DIR/sim"
COV_DIR="$SIM_DIR/build_coverage"

echo "=== Sprint 038 coverage harness ==="
echo "Repo root: $REPO_ROOT"
echo "Coverage build dir: $COV_DIR"

# Configure
cmake -S "$SIM_DIR" -B "$COV_DIR" \
    -DCMAKE_CXX_FLAGS="--coverage -O0 -g" \
    -DCMAKE_SHARED_LINKER_FLAGS="--coverage"

# Build
cmake --build "$COV_DIR" -- -j4

# Run simulation tier
cd "$REPO_ROOT"
FIRMWARE_HOST_LIB="$COV_DIR/libfirmware_host.$(uname | grep -qi darwin && echo dylib || echo so)" \
    uv run --with pytest python -m pytest tests/simulation -q

# Report
uv run --with gcovr gcovr \
    --root source \
    --print-summary \
    "$COV_DIR"
```

## firmware.py lib-path override

Check `tests/_infra/sim/firmware.py`. If `Sim.__init__` or `ctypes.CDLL` uses a
hardcoded path (e.g., `pathlib.Path(__file__).parent / "build" / "libfirmware_host.*"`),
add an env var override:

```python
import os
_DEFAULT_LIB_PATH = pathlib.Path(__file__).parent / "build" / ...
_LIB_PATH = pathlib.Path(os.environ.get("FIRMWARE_HOST_LIB", str(_DEFAULT_LIB_PATH)))
```

This change is in `tests/_infra/sim/firmware.py` (not in `source/`) — it is an infra
change, not a behavior change.

## Files to Create

- `tests/_infra/coverage.sh` (executable: `chmod +x`)
- Optional: `justfile` target `coverage` at repo root that calls `tests/_infra/coverage.sh`
- Optionally update `tests/_infra/sim/firmware.py` to support `FIRMWARE_HOST_LIB` env var

## Files to Modify

- `tests/_infra/sim/firmware.py` — add `FIRMWARE_HOST_LIB` env var support (if needed)

## Files NOT Modified

- `source/` — zero changes
- Standard `tests/_infra/sim/build/` — the coverage build uses `build_coverage/`,
  never the standard build dir (so `build_lib` fixture is unaffected)

## Acceptance Criteria

- [x] `tests/_infra/coverage.sh` exists and is executable.
- [x] Running `bash tests/_infra/coverage.sh` from the repo root completes without errors.
- [x] The script prints a gcovr summary line with an overall line coverage percentage.
      Measured baseline: 71.6% lines (3489/4871) at Phase 0.
- [x] The coverage build uses a separate `build_coverage/` dir — the standard `build/`
      dir is unaffected (already covered by gitignore `build_*/` rule).
- [x] The standard `uv run --with pytest python -m pytest -q` suite still passes ≥ 1954
      tests after this ticket (the firmware.py env var change is backward compatible).
- [x] `git diff source/` is empty.

## Testing Plan

```bash
# Run the coverage harness end-to-end:
bash tests/_infra/coverage.sh
# Expected: prints gcovr summary with a line-% number, exits 0

# Confirm standard suite still green:
uv run --with pytest python -m pytest -q

# Confirm build dirs are separate:
ls tests/_infra/sim/build/          # standard build (libfirmware_host.*)
ls tests/_infra/sim/build_coverage/ # coverage build (libfirmware_host.* + .gcno/.gcda files)
```
