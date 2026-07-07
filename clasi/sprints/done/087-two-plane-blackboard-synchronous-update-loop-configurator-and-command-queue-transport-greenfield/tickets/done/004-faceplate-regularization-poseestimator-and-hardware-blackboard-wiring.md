---
id: '004'
title: 'Faceplate regularization: PoseEstimator and Hardware blackboard wiring'
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-004
- SUC-006
depends-on:
- '002'
github-issue: ''
issue: plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Faceplate regularization: PoseEstimator and Hardware blackboard wiring

## Description

Give `PoseEstimator` and `Hardware` (`NezhaHardware`/`SimHardware`) the
faceplate members `architecture-update.md`'s Step 3/Step 5 identify as
missing today. `PoseEstimator` gains `configure()`/`config()` and a
drainable `poseResetIn` queue consumed inside `tick()`, **reusing** the
existing pending-flag mechanism (`setPose()`/`resetEncoderBaseline()`)
rather than replacing it — the phantom-jump-avoidance logic stays exactly
where it is today (Decision 7: target-drained resets keep the entangled
coherence logic inside the estimator). `Hardware` gains a uniform
`config()`/`state()` (today per-`Hal::Motor`) and consumes a per-port
`Rt::Mailbox<msg::MotorCommand> motorIn[kPortCount]` array (Decision 2)
plus a `bool motorResetIn[kPortCount]` flag array (`ZERO enc`'s
target-drained reset, idempotent, reusing the existing `resetPosition()`
staging) in place of its current addressed-command input path.

## Acceptance Criteria

- [x] `PoseEstimator::configure(...)`/`config()` exist and round-trip a
      config value. Confirm during implementation whether a
      `PoseEstimatorConfig`-equivalent `msg::` struct already exists under
      `source/messages/`; add one if not (Grounding did not confirm one
      exists — this is the one open item this ticket must resolve, not
      predicted in `architecture-update.md`, which stays at module level).
- [x] `PoseEstimator::tick()`'s signature gains a
      `Rt::WorkQueue<Rt::PoseResetCommand,4>& poseResetIn` parameter;
      `tick()` drains it (FIFO, all entries each pass) and dispatches
      `kSetPose` to the existing `setPose()`, `kResetBaseline` to the
      existing `resetEncoderBaseline()` — no change to either method's own
      internals or the pending-flag/phantom-jump-avoidance mechanism.
- [x] `Hardware::tick()`'s signature takes a per-port
      `Rt::Mailbox<msg::MotorCommand>` array and a per-port
      `bool motorResetIn[]` array; consumes each port's mailbox uniformly
      (no addressed-dispatch branch) and applies a pending
      `motorResetIn[i]` flag by calling the existing per-motor
      `resetPosition()`, clearing the flag afterward (idempotent — "reset
      twice = reset once").
- [x] `Hardware` (both `NezhaHardware` and `SimHardware`) exposes a uniform
      `config()`/`state()` at the `Hardware` faceplate level (not only
      per-`Hal::Motor` as today).
- [x] `source/subsystems/pose_estimator.h` and
      `hardware.h`/`nezha_hardware.h`/`sim_hardware.h` include only
      `messages/*.h` and `runtime/queue.h` — never `blackboard.h`.
- [x] Existing `test_pose_estimator.py`, `test_sim_hardware.py`, and
      `test_hardware_seam.py` (and their harnesses) pass with the updated
      signatures, each updated to construct bare `Rt::WorkQueue`/
      `Rt::Mailbox` instances directly — no full `Blackboard` needed
      (SUC-002).
- [x] A new test confirms `SI`'s re-anchor (`kSetPose`) and `ZERO enc`'s
      re-baseline (`kResetBaseline`) each still avoid the phantom jump:
      posting a reset command and ticking produces the same before/after
      pose relationship the existing `setPose()`/`resetEncoderBaseline()`
      tests already assert — this ticket's queue plumbing must not regress
      that guarantee, even though the SI/ZERO *wire-level routing* itself
      is ticket 006's job.

## Implementation Plan

**Approach.** Modify `pose_estimator.{h,cpp}`, `hardware.h`,
`nezha_hardware.{h,cpp}`, `sim_hardware.{h,cpp}`. Reuse `Rt::PoseResetCommand`
from `blackboard.h` (ticket 002) as the reset-queue payload type. Confirm/add
any missing `msg::` config type for `PoseEstimator`.

**Files to modify:**
- `source/subsystems/pose_estimator.{h,cpp}`
- `source/subsystems/hardware.h`
- `source/subsystems/nezha_hardware.{h,cpp}`
- `source/subsystems/sim_hardware.{h,cpp}`
- `source/messages/odometer.h` (or the correct home for a `PoseEstimator`
  config type — confirm during implementation)
- `tests/sim/unit/pose_estimator_harness.cpp`, `sim_hardware_harness.cpp`,
  `hardware_seam_harness.cpp`, and their `.py` drivers

**Testing plan:**
- Update harnesses to construct bare `Rt::WorkQueue<Rt::PoseResetCommand,4>`
  and a `Rt::Mailbox<msg::MotorCommand>` array directly.
- Add drain-order and idempotent-reset-flag test cases.
- Re-run the phantom-jump-avoidance assertions already present in
  `test_pose_estimator.py` against the new queue-driven entry point.
- **Verification command**: `uv run pytest tests/sim/unit/test_pose_estimator.py tests/sim/unit/test_sim_hardware.py tests/sim/unit/test_hardware_seam.py`

**Documentation updates:** none beyond `architecture-update.md`.

## Implementation Notes (post-execution)

- **`Rt::PoseResetCommand`/`Rt::ConfigDelta` header boundary (per team-lead
  guidance, applying Decision 10 directly).** Both structs were moved out of
  `source/runtime/blackboard.h` into a new, lightweight, CODAL-free header
  `source/runtime/commands.h` (`<cstdint>` + `messages/drivetrain.h` only —
  the same pattern ticket 087-002 already established for
  `source/subsystems/statement.h`). `blackboard.h` now `#include`s
  `runtime/commands.h` instead of defining them inline; nothing else in
  `Rt::Blackboard`'s shape changed. This is what lets
  `pose_estimator.h`/`hardware.h`/`nezha_hardware.h`/`sim_hardware.h` name
  `Rt::PoseResetCommand`/`Rt::Mailbox<msg::MotorCommand>` without including
  `blackboard.h` — satisfying AC5's boundary rule (which pre-dates
  `commands.h`'s existence and only names `messages/*.h`/`runtime/queue.h`
  explicitly; `runtime/commands.h` is the same class of lightweight,
  non-blackboard header as `runtime/queue.h` itself).
- **`PoseEstimator::config()`.** Reused `msg::DrivetrainConfig` exactly as
  directed — no new message type. Added a `msg::DrivetrainConfig config_`
  member (mirrors `Subsystems::Drivetrain`'s own `config_` member),
  populated verbatim (`config_ = config;`) at the top of `configure()`,
  before the existing trackwidth_/rotationalSlip_/EKF-sentinel logic (all
  unchanged). `config()` returns `config_` as-is — the raw value last passed
  to `configure()`, NOT the EKF-noise-sentinel-substituted values (those
  substitutions only ever fed `EkfTiny::init()` internally, matching the
  ticket's "round-trip a config value" framing).
- **`Hardware::config(port)`/`state(port)`.** Added as new pure virtuals on
  `Subsystems::Hardware` (port-indexed, same `[1, kPortCount]` convention and
  out-of-range clamp-to-port-4 behavior as `motor()`). `NezhaHardware` and
  `SimHardware` each gained a private `msg::MotorConfig config_[kPortCount]`
  member, populated verbatim from the constructor's `configs[]` argument
  (the same array each port's own `Hal::Motor` leaf is already constructed
  with) — `config()` is a plain read of this array. `state(port)` forwards
  to that port's `Hal::Motor::state()` unchanged. Neither class gained a way
  to CHANGE a port's config after construction (no `Hardware::configure()`
  exists yet) — this ticket's own acceptance criteria only ask for
  `config()`/`state()` getters, not a setter; ticket 005 (`Configurator`)
  will need to add one when it wires per-target `configure()` calls for the
  `kMotor` delta target, since it constructs `Configurator` with a
  `Hardware&` and its own plan text expects `Hardware`'s `configure()`/
  `config()` "added in ticket 004" for all four targets — flagging this gap
  explicitly here (mirroring 005's own stated contingency for
  `Drivetrain`/`Planner`) rather than silently leaving it for that ticket's
  implementer to rediscover.
- **`motorResetIn[]` does not mark a port in-use (`NezhaHardware`).** Mirrors
  today's exact behavior: `source/commands/pose_commands.cpp`'s `ZERO`
  handler already calls `hardware->motor(port).resetPosition()` directly,
  never marking the port in-use either. A reset-only request on a port that
  has never otherwise been addressed will not be serviced by the flip-flop
  scheduler until/unless that port becomes in-use some other way — a
  pre-existing characteristic, not a regression introduced here.
  `motorIn[]`, by contrast, DOES mark the addressed port in-use (mirrors the
  existing `apply()` overloads' side effect) so a motor command posted
  through the new queue path is scheduled exactly like one delivered through
  `apply()` today.
- **Call-site breakage beyond this ticket's own file list, patched
  minimally (mirrors ticket 087-003's precedent for the SAME reason —
  `dev_loop.cpp`/`main.cpp`'s eventual deletion in ticket 007):**
  - `source/dev_loop.cpp` — both `hardware.tick(now)` slices and the
    `poseEstimator->tick(...)` call now pass an always-empty/all-false local
    `Rt::Mailbox<msg::MotorCommand> noMotorInYet[kPortCount]` /
    `bool noMotorResetInYet[kPortCount]` pair and an always-empty local
    `Rt::WorkQueue<Rt::PoseResetCommand,4> noPoseResetInYet`, respectively.
    Compile-fix only: SI/ZERO still land via the existing direct
    `setPose()`/`resetEncoderBaseline()` calls in
    `source/commands/pose_commands.cpp`, and motor commands still route via
    the existing `apply()` overloads — none of that changes this ticket.
    Ticket 007 replaces this call site for real.
  - `tests/sim/unit/dev_loop_pose_estimator_harness.cpp`'s
    `oneReferencePass()` — the same mechanical patch (mirrors its own
    existing `noDriveInYet` precedent from ticket 003).
  - `tests/sim/unit/nezha_flipflop_harness.cpp` — not in this ticket's
    original file list, but calls `Subsystems::NezhaHardware::tick()`
    directly (bypassing the abstract `Hardware*` seam) in several scenarios
    and its shared `runOneCycle()` helper; patched with the same
    always-empty/all-false local pair at each call site (or once inside
    `runOneCycle()`), preserving every existing scenario's behavior exactly
    (none of them exercise `motorIn[]`/`motorResetIn[]`).
  - `source/main.cpp` and `tests/_infra/sim/sim_api.cpp` needed NO changes —
    both call the shared `devLoopTick()` (unchanged signature), never
    `Hardware::tick()`/`PoseEstimator::tick()` directly.
- **Test additions beyond the ticket's own scope, to make `config()`/
  `state()` and the new queue-driven entry points independently testable**
  (all three explicitly-listed harnesses gained new scenarios, on top of the
  minimal signature-migration edits to every pre-existing scenario):
  - `pose_estimator_harness.cpp`: `configure()`/`config()` round-trip
    (AC1); `kSetPose` via `poseResetIn` re-anchoring both readings with no
    phantom jump, compared against the direct `setPose()` contract; and
    `kResetBaseline` via `poseResetIn` preserving the existing deferred
    `dt>0` phantom-jump guard exactly (a same-pass `dt==0` tick leaves the
    reset armed but unapplied; the next genuinely-time-advancing tick — where
    a staged hardware zero has landed — produces zero delta, not a large
    negative jump) (AC7).
  - `sim_hardware_harness.cpp` / `hardware_seam_harness.cpp`: new scenarios
    proving `config(port)`/`state(port)` match the constructor/`motor(port)`
    values, and that `motorIn[]`/`motorResetIn[]` are consumed uniformly (no
    addressed-dispatch branch) with idempotent reset clearing — the
    `hardware_seam_harness.cpp` version additionally proves this entirely
    through the abstract `Hardware*` base pointer, and that a reset really
    landed on hardware (`softResetCount() >= 1`), not just that the flag
    cleared.
- **Full verification:** `uv run python -m pytest tests/sim/unit/test_pose_estimator.py
  tests/sim/unit/test_sim_hardware.py tests/sim/unit/test_hardware_seam.py`
  (3 passed), then `tests/sim -q` (254 passed — identical to the
  pre-ticket baseline, no regressions). Additionally, as a non-required
  courtesy check (this ticket's own internal wiring doesn't touch
  `main.cpp`/`sim_api.cpp`'s construction, so neither is gated on it), ran
  `uv run python3 build.py`: both the real ARM firmware (`MICROBIT.hex`) and
  the host-simulation library (`libfirmware_host`, used by TestGUI) build
  clean with no warnings from any touched file.
