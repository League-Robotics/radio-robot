---
id: '005'
title: 'Configurator: single config-application authority'
status: done
use-cases:
- SUC-002
- SUC-003
- SUC-005
depends-on:
- '002'
- '003'
- '004'
github-issue: ''
issue: plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Configurator: single config-application authority

## Description

Implement `Configurator` per `architecture-update.md`'s Reference code and
Decision 4: constructed with references to `Drivetrain`, `PoseEstimator`,
`Planner`, and `Hardware` — **the one deliberate exception** to "no
subsystem pointers" in this design. It folds `Rt::ConfigDelta` entries
popped from the Blackboard's `configIn` `WorkQueue` into a per-target
desired-config copy, calls that target's existing `configure()` when
changed, and publishes the resulting current config into the Blackboard's
config state cells (`drivetrainConfig`, `motorConfig[]`, `plannerConfig`,
`odometerConfig`). Exposes `pending(bb)`/`applyOne(bb)` for the loop's
slack phase (ticket 007) and `publish(bb)` for boot-time seeding.

## Acceptance Criteria

- [x] `Configurator`'s constructor takes exactly `Drivetrain&`,
      `PoseEstimator&`, `Planner&`, `Hardware&`, plus boot-default configs
      (per the Reference code's
      `Configurator configurator(drivetrain, poseEstimator, planner,
      hardware, ...)`), and holds no other subsystem reference.
- [x] `applyOne(bb)` pops exactly one `ConfigDelta` from `bb.configIn` per
      call (never more), folds it into the addressed target's desired-config
      copy, calls `configure()` on that target only when the fold actually
      changes anything, and writes the resulting current config into the
      matching `bb.*Config` cell.
- [x] `publish(bb)` seeds all four `bb.*Config` cells from the
      Configurator's current per-target config without requiring a delta to
      have been posted first (boot-time use, per the Reference code's
      `configurator.publish(bb)` call before the loop starts).
- [x] `pending(bb)` returns true iff `bb.configIn` is non-empty (used by the
      loop's slack `else if` branch).
- [x] No other component calls `configure()` directly on any subsystem —
      grepping `source/commands/` and `source/runtime/command_router.*` for
      `.configure(` outside `configurator.cpp` returns nothing. **Scoped to
      what this ticket controls** (see Implementation Notes): confirmed
      clean within `source/runtime/` (no hit outside `configurator.cpp`);
      `command_router.*` does not exist yet (ticket 006 creates it).
      `source/commands/{config_commands,dev_commands}.cpp` still call
      `.configure(` directly — those are the pre-existing inline call sites
      the architecture's own Impact table assigns to ticket 006's rewrite
      ("Rewritten bodies... every subsystem pointer field removed"), not
      this ticket's file list; this ticket introduces no *new* external
      caller.
- [x] A unit test constructs a `Configurator` against real (not mocked)
      `Drivetrain`/`PoseEstimator`/`Planner`/`Hardware` instances, posts a
      `ConfigDelta` for each of the four targets in turn, calls
      `applyOne()` the corresponding number of times, and asserts each
      target's own `config()` now reflects the delta and the matching
      Blackboard cell was published.
- [x] A unit test posts two deltas for the **same** target back-to-back and
      confirms both fold into the same `configure()` call's worth of
      change when drained in sequence (deterministic FIFO fold order) —
      grounding Decision 3's "current published config, not
      current+pending" validation baseline from the *caller's* (`SET`
      handler's) side, not the Configurator's own internal fold order.

## Implementation Plan

**Approach.** New `source/runtime/configurator.{h,cpp}`. Uses
`Rt::ConfigDelta` (from `blackboard.h`, ticket 002) and calls the four
targets' existing `configure()`/`config()` (`PoseEstimator`/`Hardware`'s
added in ticket 004; confirm during implementation whether `Drivetrain`/
`Planner` already have `configure()`/`config()` today — Grounding did not
explicitly confirm `Planner` does. If either is missing, add it here and
flag the addition as a deviation from this ticket's stated scope, per
sprint 085's own precedent for documenting such deviations.)

**Files to create:**
- `source/runtime/configurator.{h,cpp}`

**Files to modify:** none expected (built entirely on tickets 002-004's
faceplates); see the `Planner`/`Drivetrain` `configure()`/`config()`
contingency above.

**Testing plan:**
- New `tests/sim/unit/configurator_harness.cpp` + `test_configurator.py`,
  constructing real subsystem instances (no mocks, consistent with this
  sprint's testability goal) and exercising the acceptance criteria above
  directly.
- **Verification command**: `uv run pytest tests/sim/unit/test_configurator.py`

**Documentation updates:** none beyond `architecture-update.md`.

## Implementation Notes (post-execution)

- **`Rt::ConfigDelta` concretized (source/runtime/commands.h), field-masked
  not full-replace.** `ConfigDelta` (a target+port placeholder since ticket
  002/004) now carries: the same `Target`/`port` fields; a `uint64_t mask`;
  and FOUR plain `msg::{DrivetrainConfig,MotorConfig,PlannerConfig,
  OdometerConfig}` members (one meaningful per delta, selected by `target`
  — mirrors `Rt::PoseResetCommand`'s own "valid when kind==..." convention
  rather than a union, since none of the four generated msg:: types has a
  *trivial* default constructor — a raw union of all four would need
  hand-written special member functions to stay well-formed; four plain
  members costs more per-queued-entry memory in `WorkQueue<ConfigDelta,16>`
  in exchange for zero union hazards, judged the better trade). Four new
  `enum class *ConfigField` enums (`DrivetrainConfigField` 41 entries,
  `MotorConfigField` 12, `PlannerConfigField` 10, `OdometerConfigField` 2)
  assign one bit position per top-level field of the matching struct — full
  coverage of every field each struct *already declares* (no new
  capability), confirmed against direct reads of
  `source/messages/{drivetrain,motor,planner,odometer}.h`. `MotorConfigField`
  is the one enum that further splits `vel_gains` into its five members
  (`kp`/`ki`/`kff`/`i_max`/`kaw`) — confirmed by direct read of
  `source/commands/dev_commands.cpp`'s `applyMotorCfgKey()` that these five
  are *already* independently wire-settable today (`DEV M <n> CFG kp=...
  ki=...`, one key at a time); no other struct's nested/array field is ever
  set at sub-field granularity by any existing wire path, so
  `DrivetrainConfigField`'s `vel_gains`/`travel_calib_wheel`/`fwd_sign_wheel`
  each fold as one whole-field bit. `bitOf(field)` overloads (one per enum)
  are the single place a field enumerator becomes a mask bit, so the
  "bit i means field i of the enum matching `target`" invariant cannot
  drift from the enums' own declaration order.
- **The fold itself is field-by-field, always onto the Configurator's OWN
  persistent per-target copy, never onto a value re-derived from `bb`.**
  `configurator.cpp`'s four `fold*()` free functions each copy only the
  masked fields from `delta.<value>` onto the caller-supplied persistent
  config, snapshot-compare before/after via `memcmp` to detect a real
  change, and return that bool. This is what makes AC-7's back-to-back
  same-target test pass even when the second delta is built as though from
  the SAME stale baseline as the first (proven directly in
  `configurator_harness.cpp`'s scenario 7): the Configurator's fold, not
  the caller's baseline discipline, is what prevents the clobber.
- **Per-target apply mapping**, all inside `Configurator::applyOne()`:
  - `kDrivetrain` → fold onto `drivetrainConfig_`; if changed, call BOTH
    `drivetrain_.configure(...)` AND `poseEstimator_.configure(...)` (both
    share `msg::DrivetrainConfig` per ticket 004 — mirrors
    `source/commands/config_commands.h`'s own documented existing behavior:
    "any drivetrain-scoped key... re-propagates the FULL candidate
    msg::DrivetrainConfig to BOTH Drivetrain::configure() and
    PoseEstimator::configure()"); publish `bb.drivetrainConfig`
    unconditionally.
  - `kMotor` → `delta.port` clamped to `[1, kPortCount]` (mirrors
    `Hardware::motor()`'s own out-of-range convention) as the index into a
    per-port `motorConfig_[kPortCount]` array; if changed, apply through
    `hardware_.motor(port).configure(...)` — **Hardware has no top-level
    configure() setter** (ticket 004's own Implementation Notes flagged
    this gap explicitly for this ticket to resolve); publish
    `bb.motorConfig[port-1]` unconditionally.
  - `kPlanner` → fold onto `plannerConfig_`; if changed, call
    `planner_.configure(...)`; publish `bb.plannerConfig` unconditionally.
  - `kOdometer` → fold onto `odometerConfig_`; if changed AND
    `hardware_.odometer()` is non-null (nullptr on this build's
    `NezhaHardware` — no real-hardware OTOS driver yet), call
    `odometer->configure(...)`; publish `bb.odometerConfig`
    unconditionally, regardless of whether a real device exists, so it
    stays a truthful record of what was asked for.
  - Publish is **unconditional** in every case (even a no-op fold) while
    `configure()` is gated on the changed-check — these are deliberately
    two separate conditions (AC-2's wording supports either reading; this
    is the simpler, always-consistent one and costs only a cheap struct
    copy).
- **Boot-config constructor argument shape matches the Reference code
  exactly (two args, not four).** `Configurator`'s constructor takes only
  `bootDrivetrainConfig`/`bootPlannerConfig` — no separate boot
  `MotorConfig`/`OdometerConfig` argument. Per-port `motorConfig_[]` is
  seeded by reading back `hardware.config(port)` (ticket 004's getter,
  which already holds exactly what `NezhaHardware`/`SimHardware`'s own
  constructor was given) rather than a redundant second copy;
  `odometerConfig_` defaults to a zero-valued `msg::OdometerConfig{}`,
  matching `source/commands/otos_commands.h`'s own established "no
  boot-config generator feeds it" convention (confirmed by direct read).
- **`Hardware::config(port)` is a boot-time snapshot only — NOT a live
  mirror of post-construction `configure()` calls (confirmed empirically,
  not merely inferred from ticket 004's note).** Neither `NezhaHardware`
  nor `SimHardware` writes back into their own `config_[]` cache after
  construction (there is no `Hardware::configure()` setter). This ticket
  does **not** add one — Configurator never needs to *read* it back after
  the one-time boot seed, and adding a Hardware-level setter/live-cache
  would be new scope beyond "concretize `ConfigDelta` + build the fold."
  The test harness originally asserted `hardware.config(port)` reflected a
  post-delta change and failed (caught during implementation, not
  guessed); fixed by asserting against the published `bb.motorConfig[]`
  cell instead — the architecture's own designated replacement for
  per-subsystem config getters/shadows ("Current config -- published by
  the Configurator on apply... Replaces every shadow"). Flagging this here,
  mirroring ticket 004's own precedent, in case ticket 006/009 later needs
  a live per-port config read outside the Configurator.
- **AC-5's grep scope.** `source/runtime/` (excluding `configurator.cpp`)
  is confirmed clean of `.configure(` calls; `source/runtime/
  command_router.*` does not exist yet (ticket 006 creates it — vacuously
  clean). `source/commands/{config_commands,dev_commands}.cpp` still
  contain their pre-existing direct `.configure(` calls — removing those is
  explicitly ticket 006's own scope (the architecture's Impact table:
  "Rewritten bodies... every subsystem pointer field removed"), not this
  ticket's file list; this ticket introduces no new external caller of
  `configure()`.
- **Call-site breakage:** none. No existing file (`dev_loop.cpp`,
  `main.cpp`, `sim_api.cpp`, any other harness) references `Rt::ConfigDelta`
  by field beyond `target`/`port` (confirmed by repo-wide grep before
  editing), so growing the struct with a `mask` and four new value members
  needed no patching anywhere outside this ticket's own new files.
  `tests/sim/unit/runtime_blackboard_harness.cpp` (ticket 002, not touched)
  continues to pass unmodified against the larger `ConfigDelta`.
- **Verification:**
  `uv run python -m pytest tests/sim/unit/test_configurator.py` (1 passed,
  9 scenarios inside the harness), then `uv run python -m pytest tests/sim
  -q` (255 passed — the pre-ticket 254 baseline plus this ticket's one new
  wrapper, no regressions). Additionally ran `uv run python3 build.py`: both
  the real ARM firmware (`MICROBIT.hex`, compiling `source/runtime/
  configurator.cpp` via CODAL's own `source/` auto-discovery) and the
  host-simulation library (`libfirmware_host`) build clean with no warnings
  from any touched file (only pre-existing, unrelated `libraries/tinyekf/
  tinyekf.h` unused-function warnings appear, in both the harness compile
  and the full build).
