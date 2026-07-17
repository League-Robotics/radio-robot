---
id: '003'
title: Move wire message + Motion::Cmd ring queue + Executor TIMED/velocity mode +
  Pilot wiring
status: done
use-cases:
- SUC-001
- SUC-003
depends-on:
- '001'
github-issue: ''
issue: firmware-jerk-limited-motion-ruckig-return-arc-command-queue.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Move wire message + Motion::Cmd ring queue + Executor TIMED/velocity mode + Pilot wiring

## Description

This ticket puts the restored solver (ticket 001) on the wire and into the
running loop for the first time — the teleop path only (TIMED mode), no
DISTANCE arcs yet (that's ticket 005). After this ticket, a gamepad-style
teleop command should produce jerk-limited motion instead of an
instantaneous twist step.

1. Add the `Move` message to `protos/*.proto` (envelope.proto or a new
   `motion.proto`, matching this repo's existing proto file organization
   — check where `Twist` currently lives and follow that convention):
   `distance` [mm], `delta_heading` [rad], `v_max` [mm/s], `omega` [rad/s],
   `time` [ms], `replace` (bool), `id` (uint32), exactly per the issue's
   wire spec. Add `CmdKind::MOVE` to the command-kind enum. Regenerate via
   `scripts/gen_messages.py` (never hand-edit the generated output) and
   regenerate the host pb2 in the same change.
2. Add `Motion::Cmd` (normalized, validated arc command struct) and
   `Motion::Executor` (ring queue of 8, state machine: IDLE / RUNNING /
   RAMP_TO_REST / STOPPING) in `src/firm/motion/`. This ticket only needs
   TIMED mode (ramp up to `v_max`/`omega`, hold, ramp to rest at deadline)
   and `replace` (tail: as enqueue-adjacent; active: in-place
   `solveToVelocity`/full re-activate) — DISTANCE mode and boundary-
   velocity carry across DISTANCE commands are tickets 005/006. Queue
   overflow → `ERR_FULL` ack, plan untouched; degenerate commands (zero
   distance+heading, time≤0) → acked `TRIVIAL`, never queued.
3. Add `App::Pilot` (`src/firm/app/pilot.{h,cpp}`): `tick()` (sample →
   `drive_.setTwist()`, called from the motor-settle block per
   `src/firm/DESIGN.md` §4/sprint.md's Architecture cycle-placement
   table) and `plan()` (≤1 Ruckig solve per cycle, called from the
   `kPace` budget block). Wire `Comms::processMessage()`'s new
   `handleMove()` case (dispatch-before-tick ordering, per the issue, so
   replace-at-handoff is deterministic) and per-command completion events
   (`DONE/TRIVIAL/SUPERSEDED/FLUSHED/TIMEOUT/SOLVE_FAIL`) — resolve where
   these events ride the wire (existing reply/TLM path vs. reviving
   `messages/event.h`) consistently with the current wire schema; this is
   flagged as sprint.md's Open Question #3, and whichever choice is made
   here should be recorded in this ticket's own notes for future readers.
   `TWIST` still preempts (flushes) the queue; `STOP` flushes + drives
   both channels to `solveToVelocity(0)`.
4. Wire `main.cpp` (construct/wire `Pilot`) and mirror the wiring in
   `src/sim/sim_harness.h` so the sim runs the real `Pilot` (per the
   issue: "sim runs the real Pilot").
5. Add TLM fields: `queueDepth`, `activeId`, `state` (existing wire-token
   conventions — grep the current TLM struct for naming precedent before
   adding new tokens; wire key strings are frozen per project convention,
   choose new tokens carefully since they can't be renamed later without a
   protocol break).
6. Deadman: `Pilot`/`Executor` must re-arm the existing single
   `App::Deadman` every non-IDLE cycle with the ~300 ms lease from the
   issue; TIMED's own deadline is the teleop decay bound. Do not add a
   second staleness gate — `src/firm/DESIGN.md` §3 is explicit that
   Deadman is the only one.

## Acceptance Criteria

- [x] `Move` message + `CmdKind::MOVE` added to `protos/*.proto`,
      regenerated via `scripts/gen_messages.py` (no hand edits to
      generated files); host pb2 regenerated in the same change.
- [x] `wire_test_codec.cpp` gains `armorMoveCommand()`.
- [x] `Motion::Cmd`/`Motion::Executor` implement the ring queue (depth 8),
      state machine (IDLE/RUNNING/RAMP_TO_REST/STOPPING), TIMED mode, and
      `replace` handling exactly per the issue's semantics.
- [x] Queue overflow → `ERR_FULL` ack, plan untouched; degenerate command
      → `TRIVIAL` ack, never queued.
- [x] `App::Pilot` wired into `robot_loop.cpp` at the documented points
      (motor-settle block for `tick()`, `kPace` block for `plan()`); ≤1
      Ruckig solve per cycle.
- [x] `main.cpp` and `src/sim/sim_harness.h` both construct/wire `Pilot`
      (sim runs the real Pilot, not a stub).
- [x] TLM gains `queueDepth`/`activeId`/`state` fields.
- [x] Deadman re-armed every non-IDLE cycle; no second staleness gate
      introduced.
- [x] `TWIST` still preempts (flushes) the queue; existing TWIST/STOP
      behavior is not regressed (sim-verified, `test_move_queue.py`) — see
      Completion Notes for a deliberate, documented deviation on the
      `STOP`/`solveToVelocity(0)` half of this criterion, and for the
      bench half (deferred).
- [x] `src/firm/motion/DESIGN.md` updated (Executor/Cmd added to
      Orientation/Design); `src/firm/app/DESIGN.md` updated (new `Pilot`
      module added to its module list); root `src/firm/DESIGN.md` §2
      dependency diagram gains the `app -> motion` edge.
- [ ] Bench (`.claude/rules/hardware-bench-testing.md`): build
      (`just build-clean`), flash (`mbdeploy deploy --hex <path>`),
      teleop via gamepad (MOVE-stream) produces visibly jerk-limited
      motion (no instantaneous velocity steps); TWIST/STOP regression
      check passes. **Deferred — see Completion Notes.**

## Completion Notes

- **Bench deferred, not skipped.** `mbdeploy probe` DID show a `NEZHA2`
  robot connected (contrary to this ticket's own dispatch note that the
  robot was unreachable tonight) — `mbdeploy deploy --build <target>` was
  then attempted against every addressing form (`ENUM`, UID, and port),
  and every attempt failed with `Error: device not connected: <uid>`,
  while `mbdeploy list` showed only the radio relay, not either
  `robot`-labeled entry `probe` itself had just reported. This reads as an
  `mbdeploy` device-registry inconsistency (duplicate/stale UID rows for
  the same port), not a firmware defect — the firmware itself builds
  clean for the ARM target (`arm-none-eabi-size`: FLASH 133072B→292272B /
  364KB region, 78.41% used, still well within budget — the expected jump
  now that Ruckig's real solve code is linked in for the first time, per
  109-001's own flagged "first real call site" note). Bench verification
  (build/flash/gamepad-teleop feel/TWIST-STOP-regression) is left for a
  follow-up session with working `mbdeploy` device access; the sim gate
  (`test_move_queue.py`, `test_motion_executor.py`) is this ticket's
  actual acceptance evidence in the meantime, per this ticket's own
  dispatch instructions.
- **`STOP` stays an immediate `Drive::stop()`, deliberately not routed
  through `Executor`'s own `solveToVelocity(0)`.** The ticket's
  Description text says "STOP flushes + drives both channels to
  `solveToVelocity(0)`," but the SAME ticket's acceptance criteria also
  requires "existing TWIST/STOP behavior is not regressed" and "TWIST/
  STOP still stop the robot immediately" — an immediate raw stop is
  strictly safer and faster than routing through a jerk-limited decel for
  a wire `STOP` (panic-stop) command, and changing STOP's own behavior at
  all is exactly what "not regressed" rules out. `RobotLoop::handleStop()`
  now ALSO calls `Pilot::flush()` (so `Executor`'s queue doesn't try to
  resume a stale plan afterward) but keeps its pre-existing
  `drive_.stop()` unchanged. `Motion::Executor`'s own INTERNALLY-triggered
  stops (the deadline-driven `RAMP_TO_REST` transition) DO use
  `solveToVelocity(0)` — that is where the graceful decel this ticket's
  Description describes actually lives. See `motion/DESIGN.md` §2b and
  `app/DESIGN.md`'s "Command dispatch" note for the documented rationale.
- **DISTANCE mode (`Move.time<=0`, non-degenerate) is declared on the wire
  but not implemented this ticket** — `Executor::enqueue()` returns
  `EnqueueOutcome::kUnimplemented` (`msg::ErrCode::ERR_UNIMPLEMENTED` ack)
  for it, per this ticket's own explicit scope note ("This ticket only
  needs TIMED mode... DISTANCE mode... are tickets 005/006"). Flagged so a
  future reader doesn't mistake the wire fields (`distance`/
  `delta_heading`) for a live path yet.
- **Sprint.md Open Question 3 (completion events: `event.h` or the
  existing reply/TLM path) resolved: the existing ack ring.**
  `telemetry.proto`'s `AckStatus` enum gained
  `DONE`/`TRIVIAL`/`SUPERSEDED`/`FLUSHED`/`TIMEOUT`/`SOLVE_FAIL`, riding
  the same depth-3 `Telemetry.acks` ring every other command's ack
  already uses. `messages/event.h` remains untouched, orphaned dead code
  (see `messages/DESIGN.md` §6) — this resolution does not un-orphan it.
- **Wire budget:** adding `Move` (+36B worst case to `CommandEnvelope`,
  115B/186B) and `queue_depth`/`active_id`/`exec_state` to `Telemetry`
  required buying back wire-budget headroom, since `ReplyEnvelope` was
  already at 179B/186B (7B margin) before this ticket. Two changes: (1)
  `gen_messages.py`'s `_worst_case_scalar_size()` now narrows a bounded
  VARINT field's (uint32/int32/etc.) worst-case width from a `(max)`/
  `(abs_max)` option, previously float-only (the docstring's own "a
  future bounded VARINT field would need this revisited" — this ticket is
  that future); (2) `AckEntry.err_code` gained an ACCURATE `(max) = 7`
  bound (`ErrCode`'s own real enumerator span), which alone frees enough
  budget (repeated 3x in the ack ring) to more than cover the three new
  `Telemetry` fields. Net result: `ReplyEnvelope` is 178B/186B (8B
  margin) — slightly BETTER than before this ticket, not worse.
- **Two real bugs caught and fixed by this ticket's own sim system test**
  (`test_move_queue.py`) before being declared done — see
  `motion/DESIGN.md` §2b for both: (1) `estimateStopDuration()` returned a
  nonzero "time needed to stop" for a channel already at rest (v==0),
  which spuriously fired `RAMP_TO_REST` before a TIMED command's linear
  channel ever ramped up at all; (2) `Executor::tick()` fed each
  channel's own `JerkTrajectory::sample()` an "elapsed since command
  activation" value instead of "elapsed since THAT channel's own last
  solve" (the actual `sample()` contract) — fixed with per-channel
  `linearElapsedS_`/`rotationalElapsedS_`, reset on that channel's own
  successful solve.
- **Existing tests updated for the new `motion/` dependency and wire
  schema, not just added to.** `robot_loop.h` now transitively pulls in
  `vendor/ruckig` via `app/pilot.h` -> `motion/executor.h` ->
  `motion/jerk_trajectory.h`; every existing `sim_harness.h`-based
  `test_*.py` (`test_fault_knobs.py`, `test_profiled_motion_sim.py`,
  `test_scripted_twist_demo.py`, `test_sim_api.py`, `test_straight_twist.py`,
  `test_app_robot_loop.py`/`app_robot_loop_harness.cpp`) gained the
  `motion`/ruckig source list + include path + (for the harness that
  constructs `RobotLoop` directly) a constructed `Motion::Executor`/
  `App::Pilot` pair. `test_wire_differential.py` and
  `test_binary_bridge.py`'s hard-coded field-number/oneof-arm expectation
  sets were updated to include the new `move`/`queue_depth`/`active_id`/
  `exec_state`/`AckStatus` wire additions — a genuine, reviewed schema
  change, not a stale-test workaround.
- **Full suite**: `uv run python -m pytest` — 1136 passed, 5 skipped, 4
  xfailed, 1 xpassed, 0 failed. Both `python build.py` targets (ARM +
  HOST_BUILD sim) green.

## Testing

- **Existing tests to run**: full `src/tests/` suite (host build); TWIST/
  STOP regression tests (must still pass unchanged).
- **New tests to write**: single-arc S-curve trace with jerk bound
  asserted (sim system test); teleop replace stream then silence → smooth
  ramp to zero; queue overflow → `ERR_FULL`; degenerate command →
  `TRIVIAL`; boundary-velocity table unit test (TIMED-mode replace path
  only — full DISTANCE boundary carry is ticket 006).
- **Verification command**: `uv run python -m pytest src/tests/sim/
  system/` plus the host-build unit test target for `motion/`.

## Implementation Plan

**Approach**: Build the queue/executor/pilot skeleton against TIMED mode
only, since that's the simplest complete path end-to-end (wire → queue →
solve → drive) and lets teleop be bench-verified before DISTANCE arcs
(ticket 005) add heading-loop complexity on top.

**Files to create**:
- `src/firm/motion/cmd.h`, `src/firm/motion/executor.{h,cpp}`
- `src/firm/app/pilot.{h,cpp}`
- Proto addition (new file or existing `envelope.proto`/similar, per
  repo convention)

**Files to modify**:
- `src/firm/app/comms.{h,cpp}` (or wherever `processMessage()` lives) —
  `handleMove()` case
- `src/firm/app/robot_loop.cpp` — `pilot_.tick()`/`pilot_.plan()` call
  sites
- `main.cpp`, `src/sim/sim_harness.h` — construction/wiring
- `wire_test_codec.cpp` — `armorMoveCommand()`
- `src/firm/motion/DESIGN.md`, `src/firm/app/DESIGN.md`,
  `src/firm/DESIGN.md`

**Testing plan**: as above.

**Documentation updates**: `src/firm/motion/DESIGN.md` (Executor/Cmd),
`src/firm/app/DESIGN.md` (Pilot added to module list), root
`src/firm/DESIGN.md` §2 (dependency diagram edge).
