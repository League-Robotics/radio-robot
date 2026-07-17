---
id: '003'
title: Move wire message + Motion::Cmd ring queue + Executor TIMED/velocity mode +
  Pilot wiring
status: open
use-cases: [SUC-001, SUC-003]
depends-on: ['001']
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

- [ ] `Move` message + `CmdKind::MOVE` added to `protos/*.proto`,
      regenerated via `scripts/gen_messages.py` (no hand edits to
      generated files); host pb2 regenerated in the same change.
- [ ] `wire_test_codec.cpp` gains `armorMoveCommand()`.
- [ ] `Motion::Cmd`/`Motion::Executor` implement the ring queue (depth 8),
      state machine (IDLE/RUNNING/RAMP_TO_REST/STOPPING), TIMED mode, and
      `replace` handling exactly per the issue's semantics.
- [ ] Queue overflow → `ERR_FULL` ack, plan untouched; degenerate command
      → `TRIVIAL` ack, never queued.
- [ ] `App::Pilot` wired into `robot_loop.cpp` at the documented points
      (motor-settle block for `tick()`, `kPace` block for `plan()`); ≤1
      Ruckig solve per cycle.
- [ ] `main.cpp` and `src/sim/sim_harness.h` both construct/wire `Pilot`
      (sim runs the real Pilot, not a stub).
- [ ] TLM gains `queueDepth`/`activeId`/`state` fields.
- [ ] Deadman re-armed every non-IDLE cycle; no second staleness gate
      introduced.
- [ ] `TWIST` still preempts (flushes) the queue; `STOP` flushes + drives
      to `solveToVelocity(0)` on both channels — existing TWIST/STOP
      behavior is not regressed (bench: jerk-limited gamepad teleop feels
      smooth; TWIST/STOP still stop the robot immediately).
- [ ] `src/firm/motion/DESIGN.md` updated (Executor/Cmd added to
      Orientation/Design); `src/firm/app/DESIGN.md` updated (new `Pilot`
      module added to its module list); root `src/firm/DESIGN.md` §2
      dependency diagram gains the `app -> motion` edge.
- [ ] Bench (`.claude/rules/hardware-bench-testing.md`): build
      (`just build-clean`), flash (`mbdeploy deploy --hex <path>`),
      teleop via gamepad (MOVE-stream) produces visibly jerk-limited
      motion (no instantaneous velocity steps); TWIST/STOP regression
      check passes.

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
