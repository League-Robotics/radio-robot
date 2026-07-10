---
id: '007'
title: BinaryChannel + * discriminator + CommandProcessor/CommandRouter wiring (drive/segment/replace/stop/ping/echo/id)
status: open
use-cases: [SUC-006]
depends-on: ['005', '006']
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# BinaryChannel + * discriminator + CommandProcessor/CommandRouter wiring (drive/segment/replace/stop/ping/echo/id)

## Description

Land the binary command plane end-to-end for this sprint's seven
implemented arms, on a codec already proven correct (ticket 006) — the
last ticket in the sprint, and the only one that touches the existing
text-dispatch files (in exactly two small, localized places).

1. **New `source/commands/binary_channel.{h,cpp}`** (~250 lines per the
   issue's own estimate): `BinaryChannel::handle(const char* line,
   ReplyFn replyFn, void* replyCtx, void* routerCtx)`:
   - Dearmor: strip the `*B` prefix and trailing newline, base64-decode
     via `wire_runtime`'s functions (ticket 004).
   - Decode: `msg::wire::decode(env, buf, len)` (ticket 005). On failure,
     encode+armor+send a `ReplyEnvelope{Error{code, field}}` and return.
   - Cast `routerCtx` to `Rt::CommandRouter*` and call `.blackboard()` —
     the SAME idiom every `commands/*.cpp` handler already uses for its
     `handlerCtx` (see Decision 1 in `architecture-update.md` — do not
     invent a second access mechanism).
   - `switch (env.cmd_kind)`:
     - `DRIVE`: post `env.cmd.drive` (already a `msg::DrivetrainCommand`,
       no translation needed) straight to `bb.driveIn`, mirroring
       `handleS()`'s/`handleStop()`'s own posts (`motion_commands.cpp`).
     - `SEGMENT`: translate the decoded `msg::MotionSegment` into a local
       `Motion::Segment` field-by-field (Decision 2 — distance, direction,
       final_heading, speed_max->speedMax, accel_max->accelMax,
       jerk_max->jerkMax, yaw_rate_max->yawRateMax,
       yaw_accel_max->yawAccelMax, yaw_jerk_max->yawJerkMax, time, v,
       omega, stream), post to `bb.segmentIn`. If the queue is full, reply
       `Error{ERR_FULL}` (mirrors `handleMove()`'s existing `ERR full`
       text behavior).
     - `REPLACE`: same translation, post to `bb.replaceIn` (Mailbox,
       latest-wins — mirrors `handleMover()`).
     - `STOP`: construct `msg::DrivetrainCommand{NEUTRAL=BRAKE}` directly
       (Decision 3 — byte-identical to `handleStop()`'s own construction,
       NOT derived from any caller-supplied field since `Stop{}` has
       none), post to `bb.driveIn`.
     - `PING`: reply inline with `Ack{}` or a dedicated timestamp field if
       the schema needs one for clock-sync parity with text `PING`'s
       `t=<ms>` — check ticket 001's `Ping`/`Ack` shape and use whichever
       already carries a timestamp; if neither does, flag this as a gap
       against text `PING`'s clock-sync use case and either extend `Ack`
       or note the deferral explicitly (do not silently drop the
       clock-sync capability without saying so).
     - `ECHO`: reply inline, echoing `env.cmd.echo.payload` back — mirrors
       `handleEcho()`'s text behavior (reassemble and echo payload).
     - `ID`: reply inline with `ReplyEnvelope{DeviceId{model, name,
       serial, fw_version, proto_version}}`, sourced from the SAME
       `deviceIdentity()` helper `system_commands.cpp` already exposes
       (reuse it, do not duplicate the `#ifdef HOST_BUILD` identity
       branch a second time).
     - `MOTION`/`CONFIG`/`POSE`/`OTOS`/`GET`/`STREAM` (declared-only
       arms): reply `Error{ERR_UNIMPLEMENTED, fieldNumber}` — never
       silently drop, never crash.
   - Encode+armor+send a `ReplyEnvelope{Ack{q, rem}}` on success for
     `drive`/`segment`/`replace`/`stop` — `q`/`rem` sourced the same way
     `handleMove()`/`handleMover()` already compute them
     (`bb.segmentIn.size() + bb.drivetrain.queue`, `bb.drivetrain.rem`).
2. **`source/commands/command_processor.{h,cpp}`**: add a private `void*
   _binaryCtx = nullptr` member and a public `setBinaryContext(void* ctx)`
   setter (mirrors `setSerialReply()`'s existing shape exactly). At the
   TOP of `process()` (before `parseTokens()` runs — base64 must never be
   tokenized/uppercased), add: `if (line[0] == '*') { BinaryChannel::handle(line,
   replyFn, ctx, _binaryCtx); return; }`. Zero other changes to
   `process()`, `dispatchTable()`, or any reply-builder helper.
3. **`source/runtime/command_router.cpp`**: in `CommandRouter`'s
   constructor, add one line: `processor_.setBinaryContext(this);`
   (mirrors how every `motionCommands(router)`/`systemCommands(router)`
   call already threads `&router` through as `handlerCtx`). `route()`
   itself is unchanged.

## Acceptance Criteria

- [ ] A binary `drive` command posts the exact `msg::DrivetrainCommand`
      payload decoded from the wire to `bb.driveIn`, unmodified.
- [ ] A binary `segment`/`replace` command translates the decoded
      `msg::MotionSegment` into a `Motion::Segment` with every field
      correctly mapped (verify each of the 13 fields individually, not
      just "it compiles") and posts to `bb.segmentIn`/`bb.replaceIn`
      respectively, matching `handleMove()`'s/`handleMover()`'s posted
      shape for equivalent input.
- [ ] A binary `stop` command posts `msg::DrivetrainCommand{NEUTRAL=BRAKE}`
      to `bb.driveIn`, byte-identical to `handleStop()`'s own
      construction.
- [ ] A binary `ping`/`echo`/`id` command replies inline (no Blackboard
      post) with information content matching its text counterpart.
- [ ] A binary `motion`/`config`/`pose`/`otos`/`get`/`stream` command
      replies `Error{ERR_UNIMPLEMENTED, ...}` — never crashes, never
      silently drops.
- [ ] A malformed or out-of-range binary command (decode failure or a
      validation-bound violation from ticket 005) yields a typed
      `Error{code, field}` reply, never a crash, never a silent drop.
- [ ] `CommandProcessor::process()`'s text branch (`dispatchTable()` and
      everything it calls) is verified UNCHANGED — diff
      `command_processor.cpp` and confirm the only edits are the new
      member/setter/one branch at the top of `process()`.
- [ ] The full existing text-plane sim suite passes byte-for-byte
      unmodified (58-test baseline, per `sprint.md`).
- [ ] New sim tests cover: each implemented arm posting to the correct
      Blackboard queue with correctly-translated fields; the
      `ERR_UNIMPLEMENTED` arms; malformed/out-of-range rejection; a mixed
      text+binary session in the same test (proves dual-stack coexistence
      at the dispatch level, not just "each plane works alone").
- [ ] `just build` (ARM) and `just build-sim` succeed.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim -q` (full
  58-test baseline, must stay green unmodified), `just build`.
- **New tests to write**: `tests/sim/unit/test_binary_channel.py` (or
  extend the existing `sim_command()`/`sim_command_on()` test harness
  pattern `test_bare_loop_move_and_tlm.py` already established) exercising
  every acceptance criterion above via the sim's command channel — send a
  binary-armored line, assert the correct Blackboard queue received the
  correct payload (or the correct error reply for the unimplemented/
  malformed cases). Include the mixed text+binary session test explicitly.
- **Verification command**: `uv run python -m pytest
  tests/sim/unit/test_binary_channel.py -q` plus the full `uv run python
  -m pytest tests/sim -q`.

Note: the HITL bench gate (binary `MOVE`/`MOVER`/`STOP` over USB serial
AND the radio relay, text-protocol regression pass, flash delta from
`MICROBIT.map`) is run by the team-lead AFTER this ticket (and the whole
sprint) closes, per `.claude/rules/hardware-bench-testing.md` — it is
this sprint's overall acceptance gate (see `sprint.md` Success Criteria),
not a deliverable of this ticket's own dev-time session.
