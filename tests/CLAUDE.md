# tests/ — single consolidated test tree

All tests, sim infrastructure, and interactive tools live under `tests/`
(sprint 037 merged the former `host_tests/` and `host/tests/` into here).
Common, reusable test helpers ship in the package as **`robot_radio.testkit`**,
not as loose scripts in this tree.

## Layout

- `sim/` — firmware **simulation** infrastructure: `CMakeLists.txt`,
  `sim_api.cpp`, `firmware.py` (the `Sim` ctypes wrapper). The host-sim library
  `libfirmware_host` builds here into `sim/build/` (via `python3 build.py`'s
  default-both, or the `build_lib` fixture). `-DHOST_BUILD=1`.
- `unit/` — **the maintained pytest suite** (`test_*.py`): the `robot_radio`
  library tests, the firmware-sim tests (which `from firmware import Sim`), the
  firmware-logic tests, and the `testkit`/tools tests. This is what CI runs.
  Non-test helper modules a unit test imports (e.g. `rogo.py`) live here as
  siblings.
- `tools/` — **target-switchable interactive tools**: `playfield_tour.py`.
  Each runs against any target via
  `--target {sim,bench,production}` (and `--real-time/--full-speed`,
  `--pose {firmware,camera}`), built on `robot_radio.testkit.make_target`.
- `bench/` — real-robot **hardware** bench scripts and helpers
  (`bench_safety.py` is a thin shim over `robot_radio.testkit.safety`);
  `velocity_chart.py`, the rich real-robot live velocity dashboard, lives here.
- `calibrate/` — calibration routines.
- `old/` — **retired** one-off / probe / superseded scripts and demo notebooks,
  kept for reference. Not maintained, not collected.
- `conftest.py` — shared fixtures (`build_lib`, `sim`, `sim_field_profile`) and
  the `sys.path` setup so `unit/` can import `firmware` (the `Sim` wrapper) and
  `robot_radio.testkit`.

## Run

```
uv run --with pytest python -m pytest tests/ -q
```
(Bare `uv run pytest` fails on a missing `serial` import — always use the
`--with pytest python -m pytest` form.)

## RULES

- A maintained pytest test → `tests/unit/`. A non-test module a unit test
  imports goes alongside it in `tests/unit/` (it won't be collected).
- A target-switchable tool → `tests/tools/`; build it on
  `robot_radio.testkit.make_target` so it runs on every target.
- A real-robot bench script → `tests/bench/`; a calibration routine →
  `tests/calibrate/`.
- A reusable test helper → the **`robot_radio.testkit`** package
  (`target`, `pose`, `safety`, `camera`, `dash`) — do NOT scatter helpers here.
- A one-off / probe / throwaway / superseded script → `tests/old/`.
- Keep the **root of `tests/` clean** — no loose scripts (only `conftest.py`,
  this file, and the subdirectories above).
