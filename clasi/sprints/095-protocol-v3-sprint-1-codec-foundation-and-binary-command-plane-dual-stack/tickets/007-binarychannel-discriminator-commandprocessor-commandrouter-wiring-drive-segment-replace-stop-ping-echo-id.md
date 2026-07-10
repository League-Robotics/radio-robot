---
id: '007'
title: BinaryChannel + * discriminator + CommandProcessor/CommandRouter wiring (drive/segment/replace/stop/ping/echo/id)
status: done
use-cases:
- SUC-006
depends-on:
- '005'
- '006'
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

- [x] A binary `drive` command posts the exact `msg::DrivetrainCommand`
      payload decoded from the wire to `bb.driveIn`, unmodified.
- [x] A binary `segment`/`replace` command translates the decoded
      `msg::MotionSegment` into a `Motion::Segment` with every field
      correctly mapped (verify each of the 13 fields individually, not
      just "it compiles") and posts to `bb.segmentIn`/`bb.replaceIn`
      respectively, matching `handleMove()`'s/`handleMover()`'s posted
      shape for equivalent input.
- [x] A binary `stop` command posts `msg::DrivetrainCommand{NEUTRAL=BRAKE}`
      to `bb.driveIn`, byte-identical to `handleStop()`'s own
      construction.
- [x] A binary `ping`/`echo`/`id` command replies inline (no Blackboard
      post) with information content matching its text counterpart.
- [x] A binary `motion`/`config`/`pose`/`otos`/`get`/`stream` command
      replies `Error{ERR_UNIMPLEMENTED, ...}` — never crashes, never
      silently drops.
- [x] A malformed or out-of-range binary command (decode failure or a
      validation-bound violation from ticket 005) yields a typed
      `Error{code, field}` reply, never a crash, never a silent drop.
- [x] `CommandProcessor::process()`'s text branch (`dispatchTable()` and
      everything it calls) is verified UNCHANGED — diff
      `command_processor.cpp` and confirm the only edits are the new
      member/setter/one branch at the top of `process()`.
- [x] The full existing text-plane sim suite passes byte-for-byte
      unmodified (58-test baseline, per `sprint.md`).
- [x] New sim tests cover: each implemented arm posting to the correct
      Blackboard queue with correctly-translated fields; the
      `ERR_UNIMPLEMENTED` arms; malformed/out-of-range rejection; a mixed
      text+binary session in the same test (proves dual-stack coexistence
      at the dispatch level, not just "each plane works alone").
- [x] `just build` (ARM) and `just build-sim` succeed.

## Completion Notes (2026-07-10)

**Files changed** (beyond the ticket's own 3-part plan):
- New: `source/commands/binary_channel.{h,cpp}` (M5), `tests/sim/unit/test_binary_channel.py`.
- Text-dispatch (as scoped): `source/commands/command_processor.{h,cpp}`
  (member+setter+branch), `source/runtime/command_router.cpp` (ctor line).
- **`motion` (field 5) confirmed ALREADY REMOVED** by architecture-update-r1.md
  Decision 6 before this ticket started — `CommandEnvelope::CmdKind` has no
  `MOTION` value; the binary_channel.cpp switch correctly has no case for it
  (matches the ticket description text's own list minus `motion`, which the
  Description still names but Decision 6 supersedes).
- **`system_commands.{h,cpp}`**: `deviceIdentity()` moved out of the
  anonymous namespace (external linkage) and declared in
  `system_commands.h`, per the ticket's own explicit "reuse it, do not
  duplicate" instruction for the `id` arm — zero behavior change to any
  existing caller (`handleId()`/`formatDeviceAnnouncement()` both call the
  exact same function, now just via external rather than internal linkage).
- **Two schema-gap closures** (protos/envelope.proto), same "cheap,
  downstream-critical, closed not silently dropped" treatment the ticket's
  own PING-timestamp instruction authorized, extended by analogy to a
  second, equally real gap found during implementation:
  1. **PING timestamp**: `Ack` gained a `uint32 t = 3` field ([ms]). Binary
     `ping` replies `Ack{q=0,rem=0,t=Types::systemClockNow()}` — parity
     with text PING's `OK pong t=<ms>`, per the ticket's own explicit
     instruction (sprint 098 clock-sync dependency, Decision 6).
  2. **ECHO reply arm**: `ReplyEnvelope.body` had NO `echo` variant at all
     (`CommandEnvelope.cmd.echo` existed request-side only) — the ticket's
     own text ("reply inline, echoing `env.cmd.echo.payload` back") is
     unsatisfiable against the schema as originally written. Added
     `Echo echo = 8;` to `ReplyEnvelope.body`, reusing the existing `Echo`
     message (no new type) — the same "reuse the request-side message on
     the reply side" move Decision 4 already made for `DeviceId`/`id`.
  Both regenerated via `gen_messages.py`/`gen_pb2.py`; both envelopes stay
  at 168B total (well under the 186B cap — see wire.h's own updated
  `kMaxEncodedSize` comment). Ticket 006's differential suite was extended
  (not just kept passing) to cover both new fields byte-for-byte against
  `google.protobuf`: `wire_differential_harness.cpp` gained an optional 5th
  `encode_ok` argv (`t`, backward-compatible default 0) and a new
  `encode_echo_reply` verb; `test_wire_differential.py` gained `t`-bearing
  `Ack` cases and a full `test_direction_b_echo_reply` (+6 tests total).
- **Test-support additions** (`tests/_infra/sim/`, `source/runtime/queue.h`):
  to verify all 13 `Motion::Segment` fields INDIVIDUALLY (not just
  behaviorally) per this ticket's own acceptance criterion, added
  `Mailbox<T>::peek()` (non-destructive read, mirrors `WorkQueue::peek()`'s
  existing precedent), and three `sim_api.cpp` test-only C ABI entry points:
  `sim_route_no_tick()` (routes one command without the trailing
  `MainLoop::tick()` sim_command_on() replays, so a posted segment can be
  peeked before Drivetrain drains it), `sim_peek_segment_in()`,
  `sim_peek_replace_in()`. Wrapped in `firmware.py` as
  `Sim.route_no_tick()`/`peek_segment_in()`/`peek_replace_in()`.
- **`corr_id` echoing bug found and fixed during test-writing**: the first
  draft of `binary_channel.cpp` never set `ReplyEnvelope.corr_id` on any
  reply path (violates envelope.proto's own documented contract — "corr_id
  is echoed back... so a pipelined client can correlate replies out of
  order"). Caught by `test_binary_channel.py`'s own `corr_id` assertions
  before this ticket was called done; fixed by threading `env.corr_id`
  through every `sendAck`/`sendError`/inline-reply call site.

**ARM flash delta** (measured via `git stash` isolating this ticket's
ARM-affecting files, rebuilding both sides with `just build`):
  - Before: `FLASH: 311868 B / 364 KB = 83.67%` (matches architecture-
    update.md's stated baseline), `RAM: 120768 B / 122816 B = 98.33%`.
  - After: `FLASH: 319988 B / 364 KB = 85.85%`, `RAM: 120768 B / 122816 B
    = 98.33%` (unchanged, per `.clasi/knowledge` — never a regression
    signal on its own).
  - **Delta: +8120 B (+2.18 percentage points)** — this is `wire.cpp`'s/
    `wire_runtime.cpp`'s field tables + `BinaryChannel` + base64/envelope
    glue getting their first live caller and surviving `--gc-sections`, per
    the ticket's own prediction. Comfortably under the architecture doc's
    budgeted +12-15 KB estimate.

**Verification**: `just build` (ARM, 85.85% flash) + `just build-sim`
succeed; `uv run python -m pytest tests/sim/unit/test_binary_channel.py -q`
— 22 passed; `uv run python -m pytest tests/sim -q` — 469 passed (441
baseline + 22 new binary-channel tests + 6 differential-suite additions for
the two schema-gap closures). Text-plane diff confirmed minimal (see diffs
above/git history for `command_processor.{h,cpp}`, `command_router.cpp`).

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
