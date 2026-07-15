# tests/sim/system/

Whole-robot scenario tests against the new `source/` tree's simulator —
composed via `TestSim::SimApi` (`tests/sim/support/sim_api.h`, 105-004),
which wires the REAL `App::RobotLoop` against the REAL wheel/OTOS plant
(`tests/sim/plant/`) and a scripted `Devices::I2CBus`, no ARM hardware
involved. See `tests/CLAUDE.md` for how this domain fits alongside
`bench/`/`playfield/`.

Every file here follows the same convention every other `tests/sim/*`
harness already uses (`plant/test_plant.py`'s own precedent): a pytest file
compiles its own throwaway C++ harness binary + the shared support/plant
sources via `subprocess` into a per-test `tmp_path`, runs it, and asserts
exit 0. There is no shared Python fixture (see `tests/sim/conftest.py`'s own
file header for that ticket-time call) — `TestSim::SimApi` is a C++ class
linked directly into each harness binary.

## Contents

- **`sim_api_harness.cpp` / `test_sim_api.py`** (105-004, SUC-021) — the
  off-hardware acceptance proof for `SimApi` itself: boot, a twist-driven
  ramp, an explicit STOP, deadman expiry, and the virtual-cycle-timing
  diagnostic.
- **`faults/fault_knobs_harness.cpp` / `faults/test_fault_knobs.py`**
  (105-005, SUC-022) — the three plant-level fault-injection knobs (motor
  disconnect, encoder wedge, encoder dropout) driven through `SimApi` and
  asserted against the firmware's own observable reaction in decoded
  telemetry.
- **`scripted_twist_demo_harness.cpp` / `test_scripted_twist_demo.py`**
  (105-006, SUC-023) — this sprint's own Definition of Done: a readable,
  narrated, end-to-end story built entirely on the two primitives above —
  boot, twist forward, watch the plant's real first-order velocity ramp,
  stop, watch velocity reverse the ramp and head back toward zero — with a
  human-readable cycle-by-cycle trace printed to stdout. See that file's own
  header comment for why the post-STOP observation window is bounded to 4
  cycles (a verified limit of `SimApi`'s current bus-scripting design, not
  an arbitrary choice — `clasi/issues/sim-api-multi-write-decay-window.md`
  tracks the deferred fix).

## Running

Everything here is already inside `pyproject.toml`'s `testpaths = ["tests/
sim", "tests/unit"]` — no special flag or marker needed:

```
uv run python -m pytest tests/sim/system/            # everything in this domain
uv run python -m pytest tests/sim/system/ -k fault    # just the fault-injection scenarios
```

The scripted-twist demo is also runnable standalone — this IS the sprint's
own "run one command and see the sim loop move" proof (SUC-023), with the
harness's own printed trace visible (pytest normally captures stdout; `-s`
disables that):

```
uv run python tests/sim/system/test_scripted_twist_demo.py
```

or compile-and-run the harness binary directly with no pytest involved at
all — see `test_scripted_twist_demo.py`'s own docstring for the exact
compiler invocation (it's the same sources every sibling `test_*.py` file
in this directory already compiles).

## Adding a new scenario

Copy `test_sim_api.py`'s shape: a `_HARNESS_SRC` pointing at a new
`*_harness.cpp` in this directory (or a `faults/`-style subdirectory), the
same `_APP_SOURCES`/`_DEVICE_SOURCES`/`_MESSAGE_SOURCES`/
`_KINEMATICS_SOURCES` lists, compiled with `-DHOST_BUILD` against `source/`
+ `tests/sim/support/` + `tests/sim/plant/`. Inside the harness, link
against `tests/sim/support/sim_api.h`'s `TestSim::SimApi` rather than
re-deriving the `RobotLoop`+plant+`FakeTransport` composition — see
`sim_api.h`'s own file header for what it does and does not do.
