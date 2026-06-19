---
id: '002'
title: Move simulation tests to simulation/unit/ and repoint pyproject testpaths
status: in-progress
use-cases:
- SUC-001
depends-on:
- '001'
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Move simulation tests to simulation/unit/ and repoint pyproject testpaths

## Description

Move all 73 `test_*.py` files (and non-test helper modules) from `tests/unit/` into
`tests/simulation/unit/`. Update `pyproject.toml` so `testpaths = ["tests/simulation"]`
and `norecursedirs` excludes `bench/` and `field/`. Verify the suite still reports
≥ 1954 passed with the new collection root.

Optionally, the programmer may move clear whole-robot scenario tests
(e.g., `test_incident_scenarios.py`, `test_goto_bounds.py`, `test_033_005_wedge_hardening.py`)
to `tests/simulation/system/`. This is organizational only; both subdirs are collected
under `testpaths = ["tests/simulation"]`.

## Implementation Plan

### Step 1: Move all files from `tests/unit/`

```bash
# Move all test_*.py files and helper modules:
git mv tests/unit/test_*.py   tests/simulation/unit/
git mv tests/unit/rogo.py     tests/simulation/unit/
# (move any other non-test helpers alongside their tests)
```

If the programmer elects to split `simulation/unit/` vs `simulation/system/`, move the
whole-robot scenario test files to `tests/simulation/system/` instead.

### Step 2: Update `pyproject.toml` `[tool.pytest.ini_options]`

```toml
# Before:
testpaths = ["tests"]
norecursedirs = [
    "tests/old",
    "tests/sim/build",
    "tests/bench",
    "tests/calibrate",
    "tests/tools",
    "tests/system",
    ...
]

# After:
testpaths = ["tests/simulation"]
norecursedirs = [
    "tests/old",
    "tests/_infra/sim/build",
    "tests/bench",
    "tests/field",
    "tests/_infra/calibrate",
    "tests/_infra/tools",
    "vendor", "build", ".venv", "node_modules",
    "*.egg", ".*", "dist", "{arch}", "__pycache__",
]
```

Note: `tests/_infra/` is automatically skipped by pytest because of the `_` prefix, but
`tests/_infra/sim/build` is explicitly excluded to protect against the CMake build output
directory. `tests/calibrate` and `tests/tools` are replaced by their `_infra/` counterparts
(tickets 003 will have moved them by the time this matters in execution, but these tickets
may be sequential — update the list to match the target state).

### Step 3: Update `tests/CLAUDE.md`

Rewrite the layout section to describe the new tier structure:
- `simulation/unit/` — the maintained pytest suite (always-run CI gate)
- `simulation/system/` — whole-robot sim scenarios
- `bench/unit/`, `bench/system/` — real-hardware tests (opt-in)
- `field/unit/`, `field/system/` — playfield tests (opt-in, deferred)
- `_infra/` — sim build, calibrate/, tools/, baseline artifacts

Update the `## Run` section:
```
uv run --with pytest python -m pytest -q
```
(This now collects `tests/simulation/` only, which is the intended default.)

Update `## RULES` section to reference the new tier paths.

### Step 4: Verify green suite

```
uv run --with pytest python -m pytest -q
```
With `testpaths = ["tests/simulation"]`, this collects only `simulation/unit/` and
`simulation/system/`. Confirm ≥ 1954 passed, 0 errors.

Also verify that bench/field are NOT collected by default:
```
uv run --with pytest python -m pytest --collect-only -q 2>&1 | grep "bench\|field"
# Should return no output
```

## Files to Move (`git mv`)

- All 73 `tests/unit/test_*.py` → `tests/simulation/unit/` (or some to `simulation/system/`)
- `tests/unit/rogo.py` → `tests/simulation/unit/rogo.py`
- Any other non-test helpers in `tests/unit/` → alongside their tests in `simulation/unit/`

## Files to Modify

- `pyproject.toml` — `testpaths` and `norecursedirs`
- `tests/CLAUDE.md` — layout and rules sections

## Acceptance Criteria

- [x] All 73 `test_*.py` files exist under `tests/simulation/` (unit/ or system/).
- [x] `tests/unit/` is empty and removed (or only contains `.gitkeep`).
- [x] `pyproject.toml` `testpaths = ["tests/simulation"]`.
- [x] `pyproject.toml` `norecursedirs` excludes `tests/bench`, `tests/field`,
      `tests/_infra/sim/build`.
- [x] `uv run --with pytest python -m pytest -q` collects ≥ 1954 tests and passes.
- [x] Running `pytest --collect-only` does NOT collect any file from `tests/bench/`
      or `tests/field/`.
- [x] `git diff source/` is empty.

## Testing Plan

```bash
# Primary verification:
uv run --with pytest python -m pytest -q

# Confirm collection scope (bench/field must not appear):
uv run --with pytest python -m pytest --collect-only -q 2>&1 | grep -E "bench|field"
# expected: no output

# Confirm count at least matches baseline:
uv run --with pytest python -m pytest -q 2>&1 | tail -3
```

No new tests are written in this ticket.
