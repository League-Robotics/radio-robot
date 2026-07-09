---
id: "094-006"
title: "Command surface: MOVE + graceful STOP + TLM"
status: open
use-cases: ["SUC-001", "SUC-002", "SUC-003"]
depends-on: ["094-005"]
issue: communicator-drivetrain-motion-command-segment.md
---

# 094-006: Command surface: MOVE + graceful STOP + TLM

## Description

Add the one new wire verb this sprint introduces, `MOVE`, plus a minimal
pull-based `TLM` verb, and update `STOP`'s handler to match 094-004's new
graceful-stop semantics (no code change needed in the handler itself if
094-004 already made `NEUTRAL` graceful at the `Drivetrain` level — this
ticket confirms and tests that end-to-end over the wire, and updates any
stale doc comment in `motion_commands.cpp` that still describes the old
093 instant-brake behavior).

`MOVE <distance_mm> <direction_cdeg> <finalHeading_cdeg> [v=][a=][j=][w=]
[wa=][wj=]` parses into a `Motion::Segment` and posts it to `bb.segmentIn`.
Angles are centidegrees on the wire (matching `TURN`/`RT`'s existing
convention, `motion_commands.cpp:472`), converted to radians internally.
`TLM` is a one-shot, SNAP-style synchronous read of `bb.drivetrain`/
`bb.motors[]`, replying through the command's own `ReplyFn`/`ctx` — the
exact mechanism `PING`/`HELLO`/`S`/`STOP` already use. No loop-originated
output is introduced (consistent with 093 Decision 1).

## Acceptance Criteria

- [ ] `parseMove`/`handleMove` added to `motion_commands.cpp`: parses
      `MOVE <distance_mm> <direction_cdeg> <finalHeading_cdeg>` (required)
      plus optional `v=`/`a=`/`j=`/`w=`/`wa=`/`wj=` key-value overrides;
      out-of-range args reply `ERR range`/`ERR badarg` following the
      existing verbs' own convention (`parseD`/`parseTURN`'s range-check
      shape).
  - [ ] Angles converted centidegrees→radians using the existing
        `kCdegToRad` constant (motion_commands.cpp:472) — no new conversion
        constant introduced.
  - [ ] Builds a `Motion::Segment`, posts to `bb.segmentIn`, replies
        synchronously `OK move dist=<> dir=<> fh=<>` (or the ticket
        executor's chosen exact reply shape — document whatever is chosen
        in the ticket's own completion notes for docs/protocol-v2.md's
        eventual update, Step 7 Open Question 1).
- [ ] `handleStop` (or `Drivetrain::apply()`'s `NEUTRAL` handling, if the
      graceful behavior lives there per 094-004) is confirmed, over the
      wire (`sim_command_on(h, "STOP", ...)`), to trigger a graceful
      decel-to-zero rather than an instant brake — `STOP`'s wire reply text
      is unchanged (`OK stop`) even though its physical effect changed.
  - [ ] Any doc comment in `motion_commands.cpp` still describing 093's
        instant-brake `STOP` rationale (`motion_commands.cpp:710-728`) is
        updated to describe 094's graceful-stop behavior instead.
- [ ] `handleTlm` (new): reads `bb.drivetrain`/`bb.motors[]`, replies
      synchronously with measured `enc=`/`vel=` fields plus the executor's
      active/idle flag (`msg::DrivetrainState.active`). No new loop-output
      queue, no `EVT`, no periodic timer — a single request/reply exchange
      exactly like `PING`.
- [ ] `Rt::CommandRouter`'s `buildTable()` registers `MOVE` and `TLM`
      alongside 093's `PING`/`HELLO`/`S`/`STOP`.
- [ ] Sim end-to-end tests over `sim_command()`/`sim_command_on()`:
  - [ ] `MOVE <mm> 0 0` (straight) executes and settles.
  - [ ] `MOVE 0 0 <heading>` (pure in-place turn) executes and settles.
  - [ ] `MOVE <mm> 0 <heading>` (translate-then-terminal-pivot) executes
        and settles.
  - [ ] Each of the above drains to a graceful stop with **no reverse-creep**
        (assert via ground-truth or reported velocity trace never changing
        sign after the segment's own natural completion).
  - [ ] `S`/`STOP` still work unchanged over the wire (093's four-verb
        suite stays green, extended, not replaced).
  - [ ] `TLM` returns measured `enc=`/`vel=` that track real (simulated)
        wheel motion, not a commanded target.
  - [ ] A `MOVE` sent mid-slack takes effect on the very next mandatory
        tick (no added latency beyond the existing one-pass model).
  - [ ] Two `MOVE`s queued back-to-back (no intervening tick) both execute
        in order — proves `segmentIn`'s WorkQueue shape does not drop the
        first one the way a Mailbox would.
- [ ] `just build-sim` succeeds; `uv run python -m pytest` stays green.

## Implementation Plan

**Approach**: Follow the existing `parseD`/`handleD` and `parseTURN`/
`handleTURN` shapes in `motion_commands.cpp` as templates for `parseMove`/
`handleMove` (arg parsing, range checks, kv-pair overrides via the existing
`packStopKVs`-adjacent kv-parsing helpers or a new small one if the
`v=`/`a=`/`j=`/`w=`/`wa=`/`wj=` shape doesn't fit the existing `stop=`/
`sensor=` kv machinery cleanly). `handleTlm` is new but structurally the
simplest handler in the file — a read-only reply, no blackboard post.

**Files to modify**:
- `source/commands/motion_commands.cpp` — add `parseMove`/`handleMove`,
  add `handleTlm` (and a `parseTlm` if `TLM` takes no args, a `nullptr`
  parse function following `STOP`'s own precedent,
  `motion_commands.cpp:753`), update `motionCommands()`'s registration
  list, update `handleStop`'s doc comment.
- `source/runtime/command_router.cpp` — `buildTable()` registers `MOVE`/
  `TLM`.

**Files to create**: none (all changes land in existing files).

**Testing plan**: `tests/sim/unit/` — new tests using `sim_command()`/
`sim_command_on()` exactly as the existing 093 four-verb focused suite
does, extended to cover `MOVE`'s three shapes (straight / in-place-turn /
translate-then-pivot), `TLM`, and the graceful-`STOP`-over-the-wire check.
Reuse `tests/sim/conftest.py`'s `sim` fixture unchanged (093 already fixed
its `DEV WD` widen call; this ticket adds no new fixture dependency).

**Documentation updates**: `docs/protocol-v2.md` currency is flagged as an
open question in architecture-update.md Step 7 Item 1 — if the team-lead/
stakeholder decides this ticket should update it, add: `MOVE`'s grammar,
`TLM`'s reply shape, and a note on `STOP`'s changed physical semantics.
Otherwise, defer per that open question and note the deferral in this
ticket's completion notes so it isn't silently dropped.
