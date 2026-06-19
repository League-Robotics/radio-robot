# tests/ — tiered test tree

Tests are organized into tiers by execution environment (038 reorganization).
The default `uv run --with pytest python -m pytest -q` command collects ONLY the
`simulation/` tier — the always-run CI gate. Other tiers require opt-in.

## Layout

- `simulation/unit/` — **the maintained pytest suite** (CI gate): `robot_radio`
  library tests, firmware-sim tests (`from firmware import Sim`), firmware-logic
  tests, and `testkit`/tools tests. Non-test helper modules that unit tests import
  (e.g. `rogo.py`) live here as siblings.
- `simulation/system/` — **whole-robot scenario tests** (also CI gate): multi-step
  robot behaviour tests and scenario simulations (`test_incident_scenarios.py`,
  `test_goto_bounds.py`, `test_033_005_wedge_hardening.py`).
- `bench/unit/`, `bench/system/` — **real-hardware bench tests** (opt-in, not
  collected by default).
- `field/unit/`, `field/system/` — **playfield tests** (opt-in, deferred).
- `_infra/` — sim build infrastructure, calibration routines, and interactive tools:
  - `_infra/sim/` — firmware simulation: `CMakeLists.txt`, `sim_api.cpp`,
    `firmware.py` (the `Sim` ctypes wrapper). Builds into `_infra/sim/build/`
    via `python3 build.py` or the `build_lib` fixture.
  - `_infra/calibrate/` — calibration routines.
  - `_infra/tools/` — target-switchable interactive tools (`playfield_tour.py`).
- `old/` — retired one-off / probe / superseded scripts, kept for reference.
  Not maintained, not collected.
- `conftest.py` — shared fixtures (`build_lib`, `sim`, `sim_field_profile`) and
  `sys.path` setup so `simulation/unit/` can import `firmware` (the `Sim` wrapper)
  and `robot_radio.testkit`.

## Run

```
uv run --with pytest python -m pytest -q
```

This collects `tests/simulation/` only (both `unit/` and `system/` subdirs),
which is the intended CI default.

(Bare `uv run pytest` fails on a missing `serial` import — always use the
`--with pytest python -m pytest` form.)

To run a specific tier explicitly:
```
uv run --with pytest python -m pytest tests/simulation/ -q        # simulation tier (default)
uv run --with pytest python -m pytest tests/bench/ -q            # bench tier (opt-in)
```

## RULES

- A maintained simulation pytest test → `tests/simulation/unit/`. A non-test
  module a unit test imports goes alongside it in `tests/simulation/unit/`
  (it won't be collected). Whole-robot scenario tests → `tests/simulation/system/`.
- A target-switchable interactive tool → `tests/_infra/tools/`; build it on
  `robot_radio.testkit.make_target` so it runs on every target.
- A real-robot bench script → `tests/bench/`; a calibration routine →
  `tests/_infra/calibrate/`.
- A reusable test helper → the **`robot_radio.testkit`** package
  (`target`, `pose`, `safety`, `camera`, `dash`) — do NOT scatter helpers here.
- A one-off / probe / throwaway / superseded script → `tests/old/`.
- Keep the **root of `tests/` clean** — no loose scripts (only `conftest.py`,
  this file, and the subdirectories above).
