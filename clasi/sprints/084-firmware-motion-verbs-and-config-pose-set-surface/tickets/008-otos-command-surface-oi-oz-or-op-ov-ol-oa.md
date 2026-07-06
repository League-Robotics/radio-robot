---
id: '008'
title: 'OTOS command surface: OI OZ OR OP OV OL OA'
status: done
use-cases: [SUC-007]
depends-on: ['007']
github-issue: ''
issue: firmware-config-and-pose-set-surface.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OTOS command surface: OI OZ OR OP OV OL OA

## Description

Register the seven OTOS verbs (`OI`/`OZ`/`OR`/`OP`/`OV`/`OL`/`OA`) —
**already fully specified** in `docs/protocol-v2.md` §11 (grammar, reply
shapes, `ERR nodev` behavior are all already documented and unchanged) —
resolving them uniformly against whichever `Hal::Odometer` the active
`Subsystems::Hardware` owner has, via the `hardware.odometer()` seam
sprint 082 already built (`nullptr` on `Subsystems::NezhaHardware` today).

Per architecture-update.md Decision 5 (approved as-is), this ticket adds
**`protos/odometer.proto`** (`OdometerCommand{oneof: init | zero |
reset_tracking | set_pose}`, `OdometerConfig{linear_scalar,
angular_scalar}`) rather than bolting ad hoc, non-message virtual methods
onto `Hal::Odometer` — matching the same `apply()`/`configure()` faceplate
discipline `Hal::Motor` already has. This fills the gap `hal/capability/
odometer.h`'s own file header (081) explicitly flagged as future work.

`Hal::SimOdometer` implements the new `apply()`/`configure()` against two
**new** fields (`linearScalar_`/`angularScalar_`), kept independent of its
existing error-injection knobs (`linearNoiseSigma_` etc. — 081's "two
error models never share state" acceptance criterion, extended here to
"the calibration surface and the error-injection surface never share
state" either). `OL`/`OA`'s sim implementation is a deliberate,
documented **store-and-echo, no physical effect** (there is no scale
error modeled in sim that a calibration scalar would meaningfully correct
— mirrors sprint 083 Decision 4's precedent for `sim_prefs` fields with
no ctypes-backed physical effect).

**No real-hardware OTOS driver exists this program** (deferred to
`clasi/issues/nezha-hardware-otos-driver-for-new-source-tree.md`) — every
one of these seven verbs returns `ERR nodev <verb>` against
`Subsystems::NezhaHardware` (whose `odometer()` still returns `nullptr`,
unchanged since 082). This is the same OTOS-gap caveat sprint 082 already
carried forward honestly; ticket 009's bench report must record it
explicitly again, not silently.

**Wire keys stay stable.** All seven verbs' grammar, reply shapes, and
`ERR nodev` behavior are exactly as already documented in
`docs/protocol-v2.md` §11 — this ticket implements that existing contract
without renaming or reshaping it.

## Acceptance Criteria

- [x] New `protos/odometer.proto`: `OdometerCommand` (oneof: `init` |
      `zero` | `reset_tracking` | `set_pose(SetPose)`), `OdometerConfig`
      (`linear_scalar`, `angular_scalar`); `source/messages/odometer.h`
      regenerated. **Deviation (documented in `protos/odometer.proto`
      itself):** `set_pose`'s payload type is `common.proto`'s `Pose2D`,
      not a new/reused `SetPose` message — `scripts/gen_messages.py` only
      ever auto-includes `messages/common.h` for a non-common proto file
      (no generator support for a second cross-file include), and protoc
      itself rejects a second `message SetPose` in the same `package
      robot;` (it would collide with `drivetrain.proto`'s existing
      definition). `Pose2D` is the identical shape, already
      generator-visible, and already the established reused-not-duplicated
      value type (`PoseEstimate.pose` does the same).
- [x] `source/hal/capability/odometer.h` gains
      `apply(const msg::OdometerCommand&)` and
      `configure(const msg::OdometerConfig&)` — concrete (defined once in
      the header), dispatching onto five new primitive virtuals
      (`init()`/`resetTracking()`/`setPose()`/`setLinearScalar()`/
      `setAngularScalar()`), mirroring `capability/motor.h`'s own
      apply()/configure()-over-primitives split.
- [x] `source/hal/sim/sim_odometer.{h,cpp}` implements both: `init`/
      `zero`/`reset_tracking`/`set_pose` act on `SimOdometer`'s own
      accumulator (`odomX_`/`odomY_`/`odomH_`); `configure()` stores
      `linear_scalar`/`angular_scalar` in two new fields, independent of
      the existing error-injection knobs, with a documented no-physical-
      effect this sprint (Decision 5's Consequences).
- [x] New `source/commands/otos_commands.{h,cpp}` registers `OI`/`OZ`/
      `OR`/`OP`/`OV`/`OL`/`OA`, matching `docs/protocol-v2.md` §11's
      existing wire shape exactly, resolving the odometer via
      `hardware.odometer()` each dispatch (not a construction-time-bound
      pointer — mirrors `source_old/commands/OtosCommands.h`'s own
      documented rationale for live resolution).
- [x] All seven verbs ack (`OK ...`) against the sim
      (`Subsystems::SimHardware`) — `tests/sim/unit/test_otos_commands.py`.
- [x] All seven verbs return `ERR nodev <verb>` against
      `Subsystems::NezhaHardware` (`odometer()` still `nullptr`) — no
      crash, verified by an explicit test for every one of the seven, not
      just a subset — `tests/sim/unit/test_otos_commands_nodev.py` +
      `otos_commands_harness.cpp` (an ad hoc host harness, since
      `tests/_infra/sim/CMakeLists.txt` never compiles
      `subsystems/nezha_hardware.cpp` into `libfirmware_host`).
- [x] `OP` reads cached `HardwareState`/telemetry-sampled pose (matching
      `source_old/commands/OtosCommands.h`'s `OtosCtx::hwState` precedent
      — no direct device call on every `OP` poll). **Deviation (documented
      in `otos_commands.h`/`.cpp`):** the new tree has no equivalent cached
      `HardwareState` struct — `OP` instead calls `Hal::Odometer::pose()`
      directly, a cheap accessor (not `tick()`), the SAME call
      `telemetry_commands.cpp`'s SNAP handler already makes for its `otos=`
      field (also flagged `CMD_NONE` there) — a read, not a hardware write,
      which is the substance of source_old's own distinction.

## Implementation Plan

**Approach:** Extend the `Hal::Odometer` faceplate first (message +
interface + `SimOdometer` implementation), then the thin command layer
that resolves it live each dispatch.

**Files to create:**
- `protos/odometer.proto`
- `source/commands/otos_commands.h`, `source/commands/
  otos_commands.cpp`

**Files to modify:**
- `source/hal/capability/odometer.h` (new `apply()`/`configure()`)
- `source/hal/sim/sim_odometer.h`, `source/hal/sim/sim_odometer.cpp`
  (implement them; two new fields)
- `source/main.cpp` (construct the OTOS command state, concatenate
  `otosCommands()`'s table)
- `docs/protocol-v2.md` §11 (no wire-shape change needed — already
  accurate; confirm during implementation, note any discrepancy found
  explicitly rather than silently reconciling)

**Testing plan:**
- Sim-level tests: all seven verbs ack against `SimHardware`; all seven
  return `ERR nodev` against `NezhaHardware`; `OZ`/`OV` visibly move
  `SimOdometer`'s accumulator; `OL`/`OA` store-and-echo with no physical
  effect (explicitly asserted, not just unasserted); `OR` resets tracking
  state.
- Existing suites stay green.

**Documentation updates:** None expected beyond confirming §11's existing
text still matches (see above) — this is the rare ticket in this sprint
where the wire contract was already fully specified before implementation
began.
