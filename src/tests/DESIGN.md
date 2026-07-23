# Tests (`src/tests`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-22 · **Status:** in-flux

---

## 1. Purpose

`src/tests/` is every test and HITL (human-in-the-loop) verification
tool for this project, organized around **where the test runs**, not
around a CI-gate tiering scheme. It was rebuilt from scratch alongside
`src/firm/` in the sprint 077 greenfield rebuild — `src/archive/
source_old`/`src/archive/tests_old` are the parked pre-rebuild trees,
kept for history, never touched by anything under this directory. The
seam this subsystem draws is "three independent test regimes running on
three different machines, never combined": collapsing them into one
suite would either force sim-only assertions onto real hardware (flaky,
unsafe) or force HITL/camera setup onto the unattended CI gate
(impossible to run headless). See this directory's own `CLAUDE.md` for
the full historical rationale — this doc is the design-doc-set summary
of the same material, kept in sync with it.

## 2. Orientation

Three never-combined domains, plus two flat "kept categories":

- **`sim/`** — runs on a developer laptop against `src/sim/`'s host-build
  firmware simulator, no hardware. `unit/` (per-module `App::`/
  `Devices::` host-build harnesses, one `*_harness.cpp` + `test_*.py`
  pair per module), `plant/` (`TestSim::WheelPlant`/`OtosPlant` — a
  seeded duty→velocity→position first-order model plus fault-injection
  knobs), `support/` (`TestSim::SimApi`/`FakeTransport`/
  `wire_test_codec.h` — wires the real `App::RobotLoop` against the real
  plant), `system/` (whole-robot scenario tests built on `SimApi`, the
  scripted-twist demo, fault-injection scenarios). `uv run python -m
  pytest` collects this domain only (`pyproject.toml`'s `testpaths`) —
  the always-run, no-hardware gate.
- **`bench/`** — Python CLI tools (not pytest-collected) that drive a
  real robot on the bench/stand over USB or the radio relay, with a
  human present to hand-load wheels and watch for runaways. Drives the
  robot via the `DEV` command family
  ([`docs/protocol-v2.md`](../../docs/protocol-v2.md) §16). Includes
  `tlm_log.py` (TLM-stream-to-CSV capture, sprint 115) and
  `estimator_capture.py` (sprint 117: drives a scripted, varied MOVE
  pattern — steps both directions, a reversal, pivots, chained legs —
  while capturing via `tlm_log.py`'s own `stream_to_csv()`; works against
  a real `NezhaProtocol` connection or a `SimLoop` instance).
- **`playfield/`** — Python CLI tools (not pytest-collected) that drive a
  real robot on the camera-covered playfield (never "the floor" — see
  `.clasi/knowledge/playfield-not-floor.md`). **Currently parked**: both
  scripts here need motion/odometry that only existed in the pre-rebuild
  firmware — see each file's own header for what has to come back before
  they run again. This is a DIFFERENT kind of dormancy than
  `src/host/robot_radio/planner`'s (see
  [`../host/robot_radio/DESIGN.md`](../host/robot_radio/DESIGN.md)): the
  scripts themselves are simple, it is the motion capability they call
  into that no longer exists on the current minimal firmware.
- **`unit/`** — host-side unit tests that are not scenario/domain
  specific (protocol parsing, config-sync checks, calibration kwargs).
  Some files here (`test_planner_heading.py`, `test_planner_model.py`,
  `test_planner_profile.py`) exercise the DORMANT
  `src/host/robot_radio/planner/` package — confirm current pass/fail
  status before trusting them as a live regression gate; see that
  subsystem's own doc for the live/dormant split. `test_one_step_ahead.py`
  (sprint 117) unit-tests `tools/one_step_ahead.py`'s pure ZOH prediction
  math, imported via the same cross-directory flat-import `sys.path` shim
  `test_gen_boot_config_otos.py`/`test_pose_fix_convergence_pure.py`
  already use.
- **`tools/`** — test tooling/helpers shared across domains. Includes
  `one_step_ahead.py` (sprint 117: a pure-Python, dependency-free
  reference implementation of `App::StateEstimator`'s zero-order-hold
  one-step-ahead prediction math — a genuinely separate reimplementation
  of ticket 002's C++ formula, not a wrapper around it — plus a
  leave-one-out walk and RMS-by-pattern-phase grouping helpers).
- **`notebooks/`** — Jupyter notebooks for interactive analysis (drive
  planning, drivetrain stress, sensor characterization, turn-accuracy
  results) — exploratory artifacts, not a pytest-collected suite. Includes
  `estimator_validation.ipynb` (sprint 117 ticket 007): the stakeholder's
  own leave-one-out one-step-ahead validation methodology run against an
  `estimator_capture.py`-shaped CSV (a fresh sim capture by default, a
  bench capture once ticket 008 lands) — per-stream/per-phase RMS tables,
  the ZOH lag-signature check on ramp phases, residual plots, and a
  leg-level error projection, ending in a PROPOSED (not self-ratified)
  accept-threshold table. All prediction/RMS math is delegated to
  `tools/one_step_ahead.py`; this notebook is presentation/orchestration
  only.
- **`testgui/`** — pytest tests for the `src/host/robot_radio/testgui/`
  Qt application itself (widget behavior, command dispatch, calibration
  push-on-connect), distinct from `sim/system/`'s firmware-level
  scenario tests. **`test_gui_button_acceptance.py`** (stakeholder
  directive, 2026-07-22) is this directory's standing acceptance gate for
  the GUI's own motion-button surface: it builds the real window headless,
  clicks (or, for the one entry point with no widget — `SEG 0 <cdeg>` —
  calls directly) every distance/angle preset in both the Unmanaged and
  Managed columns, the four Test buttons (rebuild+hot-reload+drive), Tour 1
  and Tour 2 end to end, STOP mid-tour, and the (currently dormant) GOTO
  button, against the REAL Sim stack (`SimTransport` → `SimLoop` → the
  compiled firmware simulator) — including the connect-time
  calibration/config push exactly as an operator's Connect click performs
  it. Every button asserts ground-truth pose lands within a stated
  tolerance, an encoder excursion actually occurred, and the move completes
  (no hang) — printing a per-button trace table and writing it to a CSV
  under pytest's own tmp dir. **GUI motion work is not considered done
  until this suite passes** — see this directory's own Constraints section
  below.

## 3. Constraints and Invariants

- **The three HITL/sim domains (`sim/`, `bench/`, `playfield/`) are
  never combined into one run or one CI gate.** A sim run proves control
  logic against an idealized plant; a bench run proves the real motor/
  encoder/PID under load with no camera or floor risk; a playfield run
  proves the whole robot holds a world-frame task in the one environment
  that can actually fail badly. Merging them either contaminates the
  unattended CI gate with hardware/camera dependencies or dilutes the
  hardware runs with sim-only assumptions.
- **`bench/` and `playfield/` are HITL CLI tools, not pytest tests** —
  nothing under either directory is pytest-collected; run them directly
  (`uv run python src/tests/bench/dev_exercise.py --port ...`).
- **`tests_old/`/`source_old/` (now under `src/archive/`) are excluded
  from collection (`norecursedirs`) and must never be touched by
  anything under `src/tests/`.** They are historical reference only.
- **Bench/playfield scripts must be resilient to the `DEV` serial-silence
  watchdog** (default 1000 ms, `docs/protocol-v2.md` §16): widen it
  (`DEV WD 3000`) at session start and always restore it (`DEV WD 1000`)
  plus send `DEV STOP` in a `finally` block — motors must never be left
  running on an exception or Ctrl-C. This is a hardware-safety
  invariant, not a style preference.
- **`sim/conftest.py` has no fixtures by design** (105-006 removed the
  stale `build_lib`/`sim` fixtures that referenced a deleted
  `tests/_infra/sim`). Every harness compiles its own throwaway binary
  ad hoc via `subprocess`; do not reintroduce a shared Python-level
  fixture without re-justifying it against that removal's reasoning.
- **GUI work is not done until `testgui/test_gui_button_acceptance.py`
  passes.** A ticket that adds, rewires, or retunes a TestGUI motion
  button (a preset, a Test button, a tour, STOP, GOTO) is not complete on
  unit tests alone — this suite is the standing proof that the button was
  actually clicked (or its exact entry point exercised) against the real
  Sim stack and the robot moved where it was supposed to, within its
  stated tolerance, without hanging. A button whose semantics are
  known-broken/dormant still gets a row here (`xfail`/`skip`, reason
  stated) rather than being silently dropped from the suite — the button
  surface must stay fully enumerated so a future change can't quietly
  regress an untested control.
- **This suite's tolerance bands ARE the user-visible quality bar, not
  just a test-passing convenience.** Stakeholder directive (2026-07-22,
  wire-testgui-live-push-of-estimator-stop-lead fix): "you're running
  1,300 tests and not testing the thing I want: the tour to look good."
  `MANAGED_ANGLE_90_ABS_MARGIN_DEG`/`TOUR_TURN_ERROR_MAX_DEG`
  (`test_gui_button_acceptance.py`) are deliberately tight enough to FAIL
  if the connect-time `EstimatorConfigPatch` push
  (`__main__.py`'s `_push_estimator_config()`) ever regresses — that is
  their entire purpose. **Widening any of these bands requires explicit
  stakeholder sign-off**, recorded as a comment on the constant itself
  (measured old value, new value, why) — the same discipline this
  module's other tolerance constants already follow. A band may be
  ADDED alongside an existing one for a mechanism-specific reason (e.g.
  Tour 1/Tour 2's per-leg bound running at 1x sim speed instead of this
  suite's default 10x, to avoid a documented `SimLoop` real-tick-thread
  polling artifact at high speed-up factors — see `_run_tour()`'s own
  docstring) without that counting as "widening."

## 4. Design

**Why organized by "where it runs," not by CI tier.** The pre-rebuild
`tests_old/` split `sim/` and `simulation/` as separate directories that
both wrapped the same simulated firmware — an artifact of history, not a
real domain boundary. The 077 rebuild collapsed that into the current
three-domain split specifically because "which machine does this need"
is the actual constraint that determines whether a test can run
unattended in CI (`sim/`) or requires a human at a bench (`bench/`) or a
camera-covered table (`playfield/`) — a tiering scheme (unit/integration/
e2e) would cut across that constraint rather than track it.

**One sim object, shared by tests and the TestGUI.** `sim/support`'s
`SimApi`/`FakeTransport` and `src/sim/`'s `SimHarness`/`SimPlant` are the
same command-in/telemetry-out path the TestGUI drives when connected via
its Sim transport (see [`../sim/DESIGN.md`](../sim/DESIGN.md)) — a test
in `sim/system/` therefore exercises exactly what a developer watching
the TestGUI would see, not a divergent, test-only code path.

## 5. Interfaces

### Exposes

- **`uv run python -m pytest`** — collects `src/tests/sim/` only (see
  `pyproject.toml`'s `testpaths`); the always-run, no-hardware gate.
- **`src/tests/bench/*.py`, `src/tests/playfield/*.py`** — standalone
  HITL CLI entry points, invoked directly with `uv run python`.
- **`src/tests/sim/plant/`'s `WheelPlant`/`OtosPlant`** — the physics
  model `src/sim/sim_plant.cpp` delegates to; see
  [`../sim/DESIGN.md`](../sim/DESIGN.md) §2.
- **`src/tests/sim/support/fake_transport.h`** — the `App::Transport`
  HOST_BUILD double used by both the sim harness and host-build unit
  harnesses.

### Consumes

- **`src/firm/` (HOST_BUILD)** — every `sim/unit/*_harness.cpp` and
  `sim/system/*.cpp` links the real firmware modules compiled under
  `-DHOST_BUILD`; see [`../firm/DESIGN.md`](../firm/DESIGN.md) §4.
- **`src/sim/`** — the composition root (`SimHarness`) `sim/system/`'s
  Python-level tests drive over ctypes; see
  [`../sim/DESIGN.md`](../sim/DESIGN.md).
- **`src/host/robot_radio/`** — `testgui/` and `unit/`'s protocol tests
  exercise the host package directly; see
  [`../host/robot_radio/DESIGN.md`](../host/robot_radio/DESIGN.md).

## 6. Open Questions / Known Limitations

- **`playfield/` is parked**, waiting on motion/odometry capability that
  existed pre-rebuild and has not yet returned in the current minimal
  firmware (no MOVE command yet — see
  [`../firm/DESIGN.md`](../firm/DESIGN.md) §6). Each script's own header
  names its specific missing dependency.
- **`unit/test_planner_*.py`'s current pass/fail status against the
  DORMANT `src/host/robot_radio/planner/` package was not re-verified as
  part of this review** — see
  [`../host/robot_radio/DESIGN.md`](../host/robot_radio/DESIGN.md) for
  the live/dormant split those tests exercise.
