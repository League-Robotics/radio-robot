---
id: '004'
title: Telemetry surface -- TLM frame, STREAM/SNAP commands
status: done
use-cases:
- SUC-004
depends-on:
- '003'
github-issue: ''
issue: plan-revive-testgui-against-the-new-tree-simulator.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Telemetry surface -- TLM frame, STREAM/SNAP commands

## Description

Add the `STREAM <ms>` / `SNAP` verbs and `TLM` frame formatting so both the
ARM firmware and the sprint-081 host sim emit identical telemetry frames,
ported in concept from `source_old/robot/RobotTelemetry.cpp`. Carries the
fields `t= mode= seq= enc= vel= pose= encpose= otos= twist=` per
`docs/protocol-v2.md` §8's field vocabulary (a reference for field syntax,
not already-built behavior in `source/`).

**Deliberately minimal this sprint** (architecture-update.md Decision 5):
no `STREAM fields=<csv>` subscription (always emit the full fixed field
set -- there is no second field set to select between in this dev-bench
tree yet), no D10 idle-rate refinement (`max(period, 500ms)` when idle), no
channel-rebinding-restriction nuance beyond "the channel that most recently
issued `STREAM` is the bound recipient." These are named, explicit
deferrals, not gaps to silently reintroduce without a fresh reason.

## Acceptance Criteria

- [x] `source/telemetry/tlm_frame.{h,cpp}` -- a pure, stateless frame-
      formatting function/class taking `now`, `mode` (char), `seq`, per-wheel
      `enc`/`vel` values, and `pose`/`encpose`/`otos`/`twist` (each
      independently provided-or-omitted), producing one `TLM ...` wire line.
      No I/O, no state -- given the same inputs, produces the same string.
- [x] `source/commands/telemetry_commands.{h,cpp}` registers `STREAM <ms>`
      and `SNAP` via `makeSchemaCmd`/`makeCmd` (matching the existing
      `dev_commands.cpp` registration pattern), holding a `TelemetryState`
      struct: `periodMs` (0 = disabled), a shared `uint16_t seq`, the bound
      `ReplyFn`/`void* replyCtx` (captured at `STREAM`-command time), and
      `lastEmitMs`/`hasLastEmit` for the periodic-emission check.
- [x] `STREAM <ms>` clamps to a 20 ms floor (matching
      `docs/protocol-v2.md`'s existing documented minimum: `STREAM 10` ->
      `OK stream period=20`) and replies with the clamped period. `STREAM 0`
      disables periodic emission.
- [x] `SNAP` returns one `TLM` line synchronously (not wrapped in `OK`),
      sharing the SAME `seq` counter `STREAM`-driven frames use (`SNAP`
      increments/reads it exactly like a periodic emission would).
- [x] `devLoopTick()` gains one new step (after ticket 003's estimator-tick
      step): if `periodMs > 0` and enough time has elapsed since
      `lastEmitMs`, format and send a frame on the bound `replyFn`/`replyCtx`,
      then update `lastEmitMs`.
- [x] `enc=`/`vel=` read `hardware.motor(port).position()` /
      `.velocity()` directly for the Drivetrain's bound pair -- NOT
      `Drivetrain::state()`'s `vel_[]` (which reports commanded targets, a
      different semantic -- architecture-update.md Decision 7). Confirm this
      by construction, not just by testing: the telemetry code must not
      include or reference `Drivetrain::state()` at all for these two fields.
- [x] `pose=`/`encpose=` read `poseEstimator->fusedPose()` /
      `->encoderPose()` (ticket 002/003). `otos=` reads the raw sampled
      odometer pose from ticket 003's wiring, **omitted (not zero-filled)**
      when `hardware.odometer() == nullptr`.
- [x] `twist=` is populated from directly-measured/derived rates (e.g. the
      fused estimate's own twist field, or a direct differentiation of
      encoder deltas over dt) -- NOT read from any EKF velocity-channel state
      (ticket 001 does not implement one).
- [x] `mode=` is `I` when `!drivetrain.active()` and `S` when active --
      exactly two values this sprint, matching `docs/protocol-v2.md`'s
      existing `I`/`S` character definitions (no `T`/`D`/`G` -- those belong
      to sprint 083's motion verbs).
- [x] `source/main.cpp` concatenates `telemetryCommands(telemetryState)` into
      the command table alongside `systemCommands()`/`devCommands(devState)`.
- [x] Hardware bench smoke: `STREAM`/`SNAP` round-trip over real serial,
      producing well-formed frames with `enc=`/`vel=` visibly changing as the
      robot is driven on the stand. **Deferred to ticket 005's HITL session**
      per this ticket's dispatch instructions (consolidating the bench gate
      there rather than duplicating it here) -- verified instead via the
      ctypes sim (`tests/_infra/sim/sim_api.cpp`/`firmware.py`): `STREAM 10`
      clamps to `period=20`, `SNAP` and STREAM-driven frames share one `seq`
      counter, `enc=`/`vel=`/`pose=`/`encpose=`/`otos=`/`twist=` all changed
      visibly while driving, and `mode=` correctly flipped `I`/`S`.

## Implementation Plan

### Approach

1. Read `source_old/robot/RobotTelemetry.cpp` in full to extract its frame-
   assembly and per-field omission logic (freshness gating, subscription
   masking) -- adapt only the parts needed for this sprint's fixed field set
   (Decision 5 explicitly drops the subscription mechanism itself).
2. Write `tlm_frame.{h,cpp}` as a pure function first, unit-testable with
   plain scalar/struct inputs, no dependency on `DevLoop`/`Hardware` types.
3. Write `telemetry_commands.{h,cpp}` following the existing
   `dev_commands.cpp` registration pattern exactly (same `makeSchemaCmd`
   usage for `STREAM <ms>`'s fixed-shape `<verb> <int>` form; `SNAP` needs no
   schema, mirroring how `PING`/`VER` register with `parseFn = nullptr`).
4. Extend `devLoopTick()` with the periodic-emission step; extend
   `main.cpp`'s command-table concatenation and `TelemetryState`
   construction/wiring.
5. Confirm the `enc=`/`vel=`/`otos=` sourcing rules (Decision 7, the
   omission-vs-zero-fill rule) hold via unit test, not just code review.

### Files to create

- `source/telemetry/tlm_frame.h`
- `source/telemetry/tlm_frame.cpp`
- `source/commands/telemetry_commands.h`
- `source/commands/telemetry_commands.cpp`
- `tests/sim/unit/tlm_frame_harness.cpp` (frame-formatting unit tests, ad hoc
  compile tier).

### Files to modify

- `source/dev_loop.h` / `.cpp` -- add `TelemetryState*` field and periodic-
  emission step.
- `source/main.cpp` -- construct/wire `TelemetryState`; concatenate
  `telemetryCommands()`.

### Testing plan

- Host unit tests for `tlm_frame.*`'s pure formatting (all fields present;
  each field independently omitted; verify omission vs. zero-fill for
  `otos=` when no odometer).
- Host/sim-level test (via the 081 ctypes harness, once reachable) for
  `STREAM`/`SNAP` shared `seq=` and period clamping -- may land partly in
  ticket 005 if the CMakeLists.txt source-list addition for the new files
  is more naturally batched with ticket 005's verification work; note the
  actual split in this ticket's implementation notes when executed.
- Hardware bench gate: `STREAM <ms>` / `SNAP` over real serial, transcript
  recorded.

### Documentation updates

- `docs/protocol-v2.md` §8: add a note (or a short new subsection) that
  `source/`'s `STREAM`/`SNAP`/`TLM` implementation, as of this sprint, is the
  minimal subset described in architecture-update.md Decision 5 (no
  `fields=`, no idle-rate refinement) -- so a reader of §8 does not assume
  the new tree already has the full richness §8 documents from the old tree.

## Implementation Notes (as executed)

- **Testing-plan split, resolved:** the CMakeLists.txt source-list addition
  (`tlm_frame.cpp`, `telemetry_commands.cpp`) landed IN this ticket, not
  deferred to 005 -- it was needed immediately to keep
  `test_dev_loop_pose_estimator.py` (082-003) linking, since `dev_loop.cpp`'s
  new periodic-emission step calls `telemetryEmit()` unconditionally. The
  host/sim-level `STREAM`/`SNAP` proof itself (shared `seq=`, period
  clamping, live field values while driving) was done as manual verification
  via `tests/_infra/sim/firmware.py`'s `Sim` wrapper during this ticket
  rather than as a new checked-in pytest test -- ticket 005 owns turning
  that into a permanent regression test.
- **`Subsystems::PoseEstimator::trackwidth()`** (small addition beyond the
  planned file list, in `source/subsystems/pose_estimator.h`): a
  small, read-only accessor for the SAME trackwidth `configure()` already
  caches, needed so `telemetryEmit()` can compute `twist=` via
  `BodyKinematics::forward(velLeft, velRight, trackwidth, ...)` -- a pure
  kinematic transform of the two directly-read wheel velocities (never
  `Drivetrain::state()`, never EKF state). Mirrors
  `Hal::PhysicsWorld::trackwidth()`'s existing pure-getter precedent.
- **`tests/_infra/sim/sim_api.cpp`** and **`tests/sim/unit/
  dev_loop_pose_estimator_harness.cpp`** both needed a `TelemetryState`
  wired into their own `DevLoop` instances (the third and fourth
  `devLoopTick()` callers, alongside `main.cpp`) -- `DevLoop::telemetry`
  is dereferenced unconditionally each pass, so every caller must wire it,
  even a harness whose `TelemetryState.periodMs` stays at its 0 (disabled)
  default and never actually emits.
- **`twist=`** is sourced via `BodyKinematics::forward()` on the same
  directly-read wheel velocities `vel=` uses (not `PoseEstimator::
  fusedPose().twist`, which this sprint's `PoseEstimator` leaves at its
  zero default per `pose_estimator.h`/`ekf_tiny.h`'s own doc comments --
  reading it as-is would have emitted an always-zero placeholder, not a
  genuinely directly-measured/derived rate).
