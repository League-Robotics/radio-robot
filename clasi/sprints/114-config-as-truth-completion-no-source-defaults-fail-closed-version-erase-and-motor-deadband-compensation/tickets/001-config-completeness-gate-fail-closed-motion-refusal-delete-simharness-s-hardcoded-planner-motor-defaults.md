---
id: '001'
title: 'Config-completeness gate: fail-closed motion refusal + delete SimHarness''s
  hardcoded planner/motor defaults'
status: open
use-cases: [SUC-001, SUC-004]
depends-on: []
github-issue: ''
issue: config-as-truth-completion-no-defaults-fail-closed-version-erase.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Config-completeness gate: fail-closed motion refusal + delete SimHarness's hardcoded planner/motor defaults

## Description

Build the configuration-completeness gate that makes "unconfigured" a real,
refusable state, and delete the hardcoded planner/motor defaults from
`TestSim::SimHarness` that currently make it impossible to ever observe that
state. These two are one unit of work: building the gate without removing the
always-configured default would leave it structurally unreachable, and
removing the default without the gate would leave motion undefined when
nothing has been configured.

## Context

See sprint 114's `sprint.md` Architecture section (Design Rationale Decisions
1-3) for the full reasoning. Summary: `App::RobotLoop` gets one boolean, set
exactly once by whichever atomic boot path configured the whole graph
(`main.cpp`'s existing `Config::default*()` sequence, or `SimHarness`'s
`configurePlanner()`+`configureMotor()` pair). Real firmware is unaffected in
practice — its boot bake is already complete before `RobotLoop::run()`
starts, so `markConfigured()` is an unconditional, always-immediate call. The
sim is where this gate has teeth: today `SimHarness`'s constructor calls
`makeExecutorConfig()`/`makeMotorConfig()` (both private static methods in
`src/sim/sim_harness.h`) unconditionally, so it is *always* pre-configured
with hardcoded stand-in values — this is exactly the divergence that let
sprint 113's own `vel_kp` bug (0.003 hardcoded vs 0.002 configured) exist.
Deleting those two methods with nothing else would silently zero out roughly
40 existing `src/tests/sim/**` C++ harnesses that construct a bare
`SimHarness` and immediately drive it. This ticket fixes that by moving the
SAME values (byte-for-byte) into a new, explicitly test-only header, and
adding one line to every affected harness.

## Approach

1. **New `ErrCode`**: add `ERR_NOT_CONFIGURED = 8;` to the `ErrCode` enum in
   `src/protos/envelope.proto` (after the existing `ERR_OVERSIZE = 7`).
   Regenerate with `python3 src/scripts/gen_messages.py` (or however
   `build.py`'s codegen step invokes it) — this regenerates
   `src/firm/messages/envelope.h`. Never hand-edit the generated header.

2. **`App::RobotLoop` gate** (`src/firm/app/robot_loop.h`/`.cpp`):
   - Add `private: bool configured_ = false;` and two public methods:
     `void markConfigured();` and `bool isConfigured() const;`.
   - In `handleTwist()` and `handleMove()`, as the very first statement: if
     `!configured_`, ack `msg::AckStatus::ACK_STATUS_ERR` with
     `static_cast<uint32_t>(msg::ErrCode::ERR_NOT_CONFIGURED)` and `return`
     immediately — do not touch `drive_`, `pilot_`, or `deadman_`.
   - Do **not** gate `handleStop()` or `handleConfig()` — both must remain
     unconditional (already always-live; this is existing, correct behavior,
     do not change it).
   - Verify (do not need to change) that PING/ECHO/ID/HELLO/DEVICE-banner
     handling lives entirely in `App::Comms`, not in `RobotLoop`'s
     `processMessage()` switch (confirmed during sprint planning via
     `grep -rln "PING\b" src/firm`) — these are unaffected by this gate by
     construction.

3. **`main.cpp`**: immediately after the existing
   `pilot.configureHeading(plannerConfig);` call (the last of the
   boot-configure sequence) and before `robotLoop.run();`, add
   `robotLoop.markConfigured();`. This is the only change to `main.cpp`.

4. **Delete `src/sim/sim_harness.h`'s hardcoded defaults**:
   - Delete the private static methods `makeMotorConfig(uint32_t port)` and
     `makeExecutorConfig()` in full.
   - In the constructor: construct `motorL_`/`motorR_` with a
     default-constructed `Devices::MotorConfig{}` instead of
     `makeMotorConfig(1)`/`makeMotorConfig(2)`. Remove the constructor body's
     `armorL_.configure(makeMotorConfig(1));` /
     `armorR_.configure(makeMotorConfig(2));` and the
     `msg::PlannerConfig cfg = makeExecutorConfig(); executor_.configure(cfg); ...`
     block entirely — `SimHarness` now leaves `executor_`/`headingSource_`/
     `drive_`/`pilot_` at their own default-constructed state after
     construction.
   - `SimHarness` must report `isConfigured() == false` (via
     `robotLoop_.isConfigured()` — add a thin passthrough accessor if
     `robotLoop_` isn't already reachable) immediately after construction,
     and `== true` only after both `configurePlanner()` and both
     `configureMotor(1, ...)`/`configureMotor(2, ...)` calls have landed.
     `configurePlanner()`/`configureMotor()` (113-002) already exist as the
     additive config-load surface — this ticket makes them load-bearing for
     the first time. Track completion with `hasConfiguredPlanner_` (already
     exists) plus new `hasConfiguredMotorL_`/`hasConfiguredMotorR_`; call
     `robotLoop_.markConfigured()` from whichever of the three calls
     completes the set. `markConfigured()` itself is idempotent (a plain
     `configured_ = true;`), so double-calling is harmless.

5. **New test-support bench-config header**:
   `src/tests/sim/support/bench_test_config.h` + `.cpp` (new files),
   namespace `TestSupport`:
   - `msg::PlannerConfig benchTestPlannerConfig();` — byte-for-byte the
     deleted `SimHarness::makeExecutorConfig()` body (copy every field and
     every explanatory comment verbatim; only the enclosing
     function/namespace changes).
   - `Devices::MotorConfig benchTestMotorConfig(uint32_t port);` —
     byte-for-byte the deleted `SimHarness::makeMotorConfig(uint32_t port)`
     body.
   - `void configureSimForBenchTest(TestSim::SimHarness& sim);` —
     convenience wrapper calling `sim.configurePlanner(benchTestPlannerConfig())`
     then `sim.configureMotor(1, benchTestMotorConfig(1))` then
     `sim.configureMotor(2, benchTestMotorConfig(2))`.

6. **Batch-migrate every existing caller**: enumerate every `.cpp` under
   `src/tests/sim/**` that constructs a bare `SimHarness` (grep for every
   construction spelling in use). For each, add
   `TestSupport::configureSimForBenchTest(sim);` (plus the `#include
   "bench_test_config.h"`) immediately after construction and before the
   first `injectTwist()`/`injectMove()`/`step()`/`boot()` call. Do **not**
   touch `src/tests/sim/unit/sim_harness_configure_harness.cpp` (113-002's
   own test) or any harness that already calls `configurePlanner()`/
   `configureMotor()` explicitly with its own values — those intentionally
   test the unconfigured-then-configured transition or a specific override.
   This convention (call `configureSimForBenchTest()` right after
   constructing a bare `SimHarness`) is now the required pattern for any
   *new* sim harness written by tickets 002-006 too.

## Files to Touch

- `src/protos/envelope.proto` (new `ErrCode` value)
- `src/firm/messages/envelope.h` (regenerated, not hand-edited)
- `src/firm/app/robot_loop.h`, `src/firm/app/robot_loop.cpp` (gate)
- `src/firm/main.cpp` (one new call)
- `src/sim/sim_harness.h` (delete two methods, change construction, add
  configured-tracking)
- `src/tests/sim/support/bench_test_config.h`, `.cpp` (new)
- Every `src/tests/sim/**/*.cpp` that constructs a bare `SimHarness` without
  configuring it (enumerate via grep; expect on the order of ~35-40 files)

## Acceptance Criteria

- [ ] `ErrCode::ERR_NOT_CONFIGURED` exists in the regenerated `envelope.h`.
- [ ] A freshly-constructed `SimHarness`, before any `configurePlanner()`/
      `configureMotor()` call, has `isConfigured() == false`.
- [ ] Injecting a TWIST or MOVE against an unconfigured `SimHarness` yields
      `ACK_STATUS_ERR` / `ERR_NOT_CONFIGURED`; `drive_`/`pilot_`/`deadman_`
      state is unchanged (zero motor writes — assert via the plant's own
      write-count/duty-history hook).
- [ ] Injecting STOP or a CONFIG patch against an unconfigured `SimHarness`
      still acks `ACK_STATUS_OK`.
- [ ] After `configurePlanner()` + both `configureMotor()` calls,
      `isConfigured() == true` and a subsequent TWIST/MOVE is accepted
      normally.
- [ ] `grep -n "makeExecutorConfig\|makeMotorConfig" src/sim/sim_harness.h`
      finds nothing.
- [ ] Every one of the ~40 pre-existing `src/tests/sim/**` harnesses compiles
      and passes unchanged (behaviorally) after adding its one
      `configureSimForBenchTest()` line — verified by a full targeted
      re-run, not a sample.
- [ ] `main.cpp`'s new `markConfigured()` call does not change any observable
      real-firmware behavior (it was always immediately true in practice).

## Testing

- **Existing tests to run**: the full `src/tests/sim` suite (unit + system) —
  this ticket's own migration touches most of it.
  `uv run python -m pytest src/tests/sim -v` as a first pass, then the full
  suite.
- **New tests to write**: a targeted `App::RobotLoop`/`SimHarness` gate test
  (new file, e.g. `src/tests/sim/unit/config_gate_harness.cpp` + its
  `test_config_gate.py` pytest wrapper, mirroring
  `sim_harness_configure_harness.cpp`'s existing shape) covering:
  unconfigured refusal (TWIST/MOVE → `ERR_NOT_CONFIGURED`, zero writes),
  unconfigured pass-through (STOP/CONFIG → OK), and the
  configured-then-accepted transition.
- **Verification command**: `uv run python -m pytest src/tests/sim -v`
  (targeted), then full suite `uv run python -m pytest` before marking done.
