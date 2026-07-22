# src/tests/sim/system/

Whole-robot scenario tests against the new `src/firm/` tree's simulator —
composed via `TestSim::SimHarness` (`tests/_infra/sim/sim_harness.h`,
108-003), which wires the REAL `App::RobotLoop` against `TestSim::SimPlant`
(`tests/_infra/sim/sim_plant.{h,cpp}`, 108-002) -- a REAL `Devices::I2CBus`
implementation that parses the actual Nezha/OTOS wire protocol and
integrates real wheel/OTOS physics (`src/tests/sim/plant/`), no ARM hardware
involved. See `tests/CLAUDE.md` for how this domain fits alongside
`bench/`/`playfield/`.

Every file here follows the same convention every other `src/tests/sim/*`
harness already uses (`plant/test_plant.py`'s own precedent): a pytest file
compiles its own throwaway C++ harness binary + the shared support/plant/
`tests/_infra/sim` sources via `subprocess` into a per-test `tmp_path`, runs
it, and asserts exit 0. There is no shared Python fixture (see
`src/tests/sim/conftest.py`'s own file header for that ticket-time call) --
`TestSim::SimHarness` is a C++ class linked directly into each harness
binary.

Ticket 108-004 migrated every scenario below off the deleted
`TestSim::SimApi`/`SimApi::DutyPredictor` (`src/tests/sim/support/sim_api.
{h,cpp}`, removed 108-003) onto `SimHarness`/`SimPlant` -- SimPlant responds
LIVE to whatever bytes firmware actually puts on the wire instead of
predicting them from a write count, closing the desync class of bug that
motivated the whole sprint (a twist stream could drift the old predictor
and the firmware's real write sequence apart; a live-responding bus cannot
desync). See each `*_harness.cpp` file's own header for its own migration
notes.

## Contents

- **`sim_api_harness.cpp` / `test_sim_api.py`** (originally 105-004,
  SUC-021) — the off-hardware acceptance proof for the SimHarness/SimPlant
  composition itself: boot, a twist-driven ramp, an explicit STOP, deadman
  expiry, and the virtual-cycle-timing diagnostic.
- **`faults/fault_knobs_harness.cpp` / `faults/test_fault_knobs.py`**
  (originally 105-005, SUC-022) — the three plant-level fault-injection
  knobs (motor disconnect, encoder wedge, encoder dropout), now surfaced
  per-port on `SimPlant` via `SimHarness::plant()`, driven through the real
  RobotLoop and asserted against the firmware's own observable reaction in
  decoded telemetry.
- **`scripted_twist_demo_harness.cpp` / `test_scripted_twist_demo.py`**
  (originally 105-006, SUC-023; STOP phase strengthened 106-003, SUC-026) —
  a readable, narrated, end-to-end story built entirely on the two
  primitives above — boot, twist forward, watch the plant's real
  first-order velocity ramp, stop, watch velocity converge to
  (approximately) zero over a 12-cycle post-STOP window — with a
  human-readable cycle-by-cycle trace printed to stdout.
- **`straight_twist_harness.cpp` / `test_straight_twist.py`** (108-004,
  SUC-041) — the sprint's own headline regression test: a straight twist
  (v_x only, omega=0) at a realistic ~150mm/s tour cruise speed, held for a
  tour-leg-scale duration, asserting BOTH wheels track together (no
  frozen/runaway divergence) and heading stays within a small, documented
  tolerance of zero for the ENTIRE run, not just the final sample -- the
  direct proof the divergence bug (left encoder freezes, right runs away
  under an arbitrary twist stream) that motivated this whole sprint is
  gone.
- **`behavior_lock_harness.cpp` / `test_behavior_lock.py`** (111-001,
  SUC-001) — DELETED (115-002, gut-to-minimal-firmware S1 motion-stack
  excision): this was the motion-control terminal-blips arc's own Step 0
  numeric behavior-lock acceptance instrument for `Motion::Executor`'s
  accel/jerk profile, driven by `msg::PlannerConfig` bounds -- both gone
  wholesale along with the rest of the motion stack. The
  `pre-gut-motion-stack` tag preserves the full pre-deletion file for
  recovery if this design work is ever revisited.

## Running

Everything here is already inside `pyproject.toml`'s `testpaths = ["tests/
sim", "tests/unit"]` — no special flag or marker needed:

```
uv run python -m pytest src/tests/sim/system/            # everything in this domain
uv run python -m pytest src/tests/sim/system/ -k fault    # just the fault-injection scenarios
```

The scripted-twist demo is also runnable standalone — this IS the sprint's
own "run one command and see the sim loop move" proof (SUC-023), with the
harness's own printed trace visible (pytest normally captures stdout; `-s`
disables that):

```
uv run python src/tests/sim/system/test_scripted_twist_demo.py
```

or compile-and-run the harness binary directly with no pytest involved at
all — see `test_scripted_twist_demo.py`'s own docstring for the exact
compiler invocation (it's the same sources every sibling `test_*.py` file
in this directory already compiles).

## Adding a new scenario

Copy `test_sim_api.py`'s shape: a `_HARNESS_SRC` pointing at a new
`*_harness.cpp` in this directory (or a `faults/`-style subdirectory), the
same `_APP_SOURCES`/`_DEVICE_SOURCES`/`_MESSAGE_SOURCES`/
`_KINEMATICS_SOURCES` lists, compiled with `-DHOST_BUILD` against `src/firm/`
+ `src/tests/sim/support/` + `src/tests/sim/plant/` + `tests/_infra/sim/`. Inside
the harness, link against `tests/_infra/sim/sim_harness.h`'s
`TestSim::SimHarness` rather than re-deriving the `RobotLoop`+plant+
`FakeTransport` composition — see `sim_harness.h`'s own file header for what
it does and does not do.
