---
id: '001'
title: 'Fix coverage.sh harness: correct gcovr invocation, per-file table, simulatable-code
  report'
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 045-001: Fix coverage.sh harness: correct gcovr invocation, per-file table, simulatable-code report

## Description

`tests/_infra/coverage.sh` (Sprint 038 vintage) is broken: it uses `--root source`
which causes gcovr to look for `.gcno` files relative to `source/` as root. After
the Phase C EKF move (`source/control/EKF.cpp` → `source/state/EKF.cpp`), gcovr
cannot find the EKF object files and aborts with `source_not_found` errors.

Additionally the script uses `cmake --build <dir> -- -j4` (non-portable), references
the old path in comments, and has no per-file table, no `--fail-under` flag, and no
"simulatable-code" percentage (overall minus CODAL-only files).

Rewrite `coverage.sh` so it:

1. Uses a fresh build directory (`tests/_infra/sim/build_coverage/`).
2. Configures and builds with the confirmed-working cmake flags.
3. Runs the simulation pytest tier against the coverage-instrumented lib.
4. Calls gcovr with `--root .` (repo root), `--filter 'source/'`, `--gcov-ignore-errors=source_not_found`.
5. Prints overall `source/` line coverage percentage and a per-file table.
6. Prints a second "simulatable-code" coverage percentage by re-running gcovr with
   `--exclude` flags for each CODAL-only file in the documented exclusion set (see
   architecture-update.md §"CODAL-only exclusion set").
7. Accepts optional `--fail-under N` argument: exits non-zero if overall coverage < N%.
8. Updates header comments to reflect Sprint 045 and drop Sprint 038 references.

The confirmed-working invocation (Sprint 045 baseline):
```bash
cmake -S tests/_infra/sim -B <covdir> \
    -DCMAKE_CXX_FLAGS="--coverage -O0 -g" \
    -DCMAKE_SHARED_LINKER_FLAGS="--coverage"
cmake --build <covdir> --parallel
FIRMWARE_HOST_LIB=<covdir>/libfirmware_host.dylib \
    uv run --with pytest python -m pytest tests/simulation -q
uv run --with gcovr gcovr \
    --root . --filter 'source/' \
    --gcov-ignore-errors=source_not_found \
    --print-summary <covdir>
```

## Acceptance Criteria

- [x] `bash tests/_infra/coverage.sh` runs to completion with exit code 0 and no gcovr errors.
- [x] Overall `source/` line coverage percentage is printed to stdout.
- [x] Per-file coverage table is printed (one row per source file).
- [x] Simulatable-code coverage percentage is printed on a clearly labelled line, with the exclusion set documented in the script comments.
- [x] `bash tests/_infra/coverage.sh --fail-under 85` exits 1 when coverage is below 85% (test by temporarily passing a high threshold like 99).
- [x] `bash tests/_infra/coverage.sh --fail-under 0` exits 0.
- [x] The full simulation test suite still passes: `uv run --with pytest python -m pytest tests/simulation -q` exits 0.

## Implementation Plan

### Approach

Replace the body of `tests/_infra/coverage.sh` while preserving the shebang and
set -euo pipefail header. The script is self-contained; no other files reference it.

### Files to modify

- `tests/_infra/coverage.sh` — full rewrite of body, preserving path constants.

### Implementation notes

- Parse `--fail-under` from `$@` using a simple loop (no getopts needed).
- Run gcovr twice: once for overall, once with `--exclude` flags for the
  simulatable percentage. Or use a single gcovr run with `--json` output and
  compute both numbers from it if gcovr supports it cleanly. Simpler: two runs.
- CODAL-only `--exclude` patterns (regex, anchored to `source/`):
  - `source/app/DebugCommandable\.cpp`
  - `source/control/PortController\.cpp`
  - `source/control/ServoController\.cpp`
  - `source/io/real/.*`
  - `source/app/WedgeTest\.cpp`
- `source/control/LoopScheduler.cpp` and `main.cpp` are already absent from the
  host lib (not compiled in); gcovr will not find them unless `source_not_found`
  suppression is active — that's fine.
- Print format example:
  ```
  === Overall source/ coverage ===
  Lines: 74.6% (3879/5200)
  <per-file table from gcovr>

  === Simulatable-code coverage (CODAL-only files excluded) ===
  Lines: XX.X% (NNNN/MMMM)
  Excluded: DebugCommandable.cpp, PortController.cpp, ServoController.cpp, io/real/*, WedgeTest.cpp
  ```

### Testing plan

- Run `bash tests/_infra/coverage.sh` manually and confirm no errors.
- Run `bash tests/_infra/coverage.sh --fail-under 99` and confirm exit code 1.
- Run `bash tests/_infra/coverage.sh --fail-under 0` and confirm exit code 0.
- Confirm `uv run --with pytest python -m pytest tests/simulation -q` still passes after harness run.

### Documentation updates

- Update header comment in `coverage.sh` to reference Sprint 045 and the confirmed-working baseline.
