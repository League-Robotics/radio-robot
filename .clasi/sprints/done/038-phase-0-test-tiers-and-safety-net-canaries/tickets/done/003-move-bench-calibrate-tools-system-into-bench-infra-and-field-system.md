---
id: '003'
title: Move bench/calibrate/tools/system into bench/ _infra/ and field/system/
status: done
use-cases:
- SUC-001
depends-on:
- '002'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Move bench/calibrate/tools/system into bench/ _infra/ and field/system/

## Description

Complete the §7 tier layout by moving the remaining non-simulation directories into their
target locations: `tests/calibrate/` → `tests/_infra/calibrate/`; `tests/tools/` →
`tests/_infra/tools/`; `tests/system/` scripts → `tests/field/system/`; and the flat
`tests/bench/` scripts into `tests/bench/` (which now has `unit/` and `system/` subdirs
from ticket 001). Verify the simulation tier stays green and bench/field remain excluded.

These scripts are NOT pytest files (no `test_` prefix) so they are not collected.
The moves are organizational — no script content changes.

## Implementation Plan

### Step 1: Move calibrate/ into _infra/

```bash
git mv tests/calibrate/calibrate_bench.py   tests/_infra/calibrate/calibrate_bench.py
git mv tests/calibrate/calibrate_linear.py  tests/_infra/calibrate/calibrate_linear.py
git mv tests/calibrate/odom_check.py        tests/_infra/calibrate/odom_check.py
git mv tests/calibrate/README.md            tests/_infra/calibrate/README.md
git mv tests/calibrate/rotation_cal.py      tests/_infra/calibrate/rotation_cal.py
git mv tests/calibrate/rotation_sweep.py    tests/_infra/calibrate/rotation_sweep.py
git mv tests/calibrate/set_linear_cal.py    tests/_infra/calibrate/set_linear_cal.py
git mv tests/calibrate/set_param.py         tests/_infra/calibrate/set_param.py
git mv tests/calibrate/turn_once.py         tests/_infra/calibrate/turn_once.py
git mv tests/calibrate/turn_test.py         tests/_infra/calibrate/turn_test.py
```

(Or `git mv tests/calibrate/* tests/_infra/calibrate/` if git supports it for the
directory contents. Verify the list matches actual files at move time.)

### Step 2: Move tools/ into _infra/

```bash
git mv tests/tools/playfield_tour.py  tests/_infra/tools/playfield_tour.py
git mv tests/tools/__init__.py        tests/_infra/tools/__init__.py
```

(Move all files found in `tests/tools/`.)

### Step 3: Move system/ scripts into field/system/

```bash
git mv tests/system/goto_system.py       tests/field/system/goto_system.py
git mv tests/system/goto_world.py        tests/field/system/goto_world.py
git mv tests/system/run_tour.py          tests/field/system/run_tour.py
git mv tests/system/square_targets.json  tests/field/system/square_targets.json
git mv tests/system/world_tour.py        tests/field/system/world_tour.py
# Move tests/system/out/ if present
```

### Step 4: Flatten bench/ scripts into bench/ parent dir

The current `tests/bench/` already holds all scripts at the top level. The `unit/` and
`system/` subdirs were created empty in ticket 001. The existing bench scripts stay
at `tests/bench/` (top level) — they are hardware runner scripts, not pytest tests,
and do not need to be moved into `unit/` or `system/` subdirs until Phase A/B adds
real bench pytest files. No `git mv` needed here.

If any bench script imports from `bench_safety` (the shim), confirm the import still
resolves from the new `tests/bench/` location. The shim `tests/bench/bench_safety.py`
is unchanged.

### Step 5: Verify `pyproject.toml` norecursedirs is consistent

After this ticket, `tests/calibrate` and `tests/tools` no longer exist. The `norecursedirs`
from ticket 002 already uses `tests/_infra/calibrate` and `tests/_infra/tools`. Confirm
no stale entries remain for the old paths.

### Step 6: Verify suite green

```
uv run --with pytest python -m pytest -q
```

Confirm ≥ 1954 passed, 0 errors. The simulation tier should be completely unaffected
(none of the moved files are pytest-collected).

## Files to Move (`git mv`)

- `tests/calibrate/*` → `tests/_infra/calibrate/`
- `tests/tools/playfield_tour.py`, `tests/tools/__init__.py` → `tests/_infra/tools/`
- `tests/system/*.py`, `tests/system/*.json` → `tests/field/system/`

## Files NOT Moved (bench stays at bench/)

- `tests/bench/` scripts remain at `tests/bench/` top level. No moves needed.
  `bench/unit/.gitkeep` and `bench/system/.gitkeep` already exist from ticket 001.

## Acceptance Criteria

- [x] `tests/calibrate/` is empty and removed (or only `.gitkeep`).
- [x] `tests/tools/` is empty and removed (or only `.gitkeep`).
- [x] `tests/system/` is empty and removed (or only `.gitkeep`).
- [x] `tests/_infra/calibrate/` contains all moved calibrate scripts.
- [x] `tests/_infra/tools/` contains `playfield_tour.py` (and `__init__.py` if it existed).
- [x] `tests/field/system/` contains all moved system scripts.
- [x] `uv run --with pytest python -m pytest -q` passes ≥ 1954 tests, 0 errors.
- [x] `git diff source/` is empty.

## Testing Plan

```bash
# Primary verification:
uv run --with pytest python -m pytest -q

# Confirm moved tools are importable from their new paths (smoke):
python3 -c "import sys; sys.path.insert(0, 'host'); import robot_radio.testkit"
```

No new tests are written in this ticket.
