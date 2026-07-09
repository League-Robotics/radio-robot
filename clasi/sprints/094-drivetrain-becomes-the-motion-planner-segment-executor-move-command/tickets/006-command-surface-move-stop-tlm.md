---
id: "094-006"
title: "Command surface: MOVE + graceful STOP + TLM"
status: done
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

- [x] `parseMove`/`handleMove` added to `motion_commands.cpp`: parses
      `MOVE <distance_mm> <direction_cdeg> <finalHeading_cdeg>` (required)
      plus optional `v=`/`a=`/`j=`/`w=`/`wa=`/`wj=` key-value overrides;
      out-of-range args reply `ERR range`/`ERR badarg` following the
      existing verbs' own convention (`parseD`/`parseTURN`'s range-check
      shape).
  - [x] Angles converted centidegrees→radians using the existing
        `kCdegToRad` constant (motion_commands.cpp:472) — no new conversion
        constant introduced.
  - [x] Builds a `Motion::Segment`, posts to `bb.segmentIn`, replies
        synchronously `OK move dist=<> dir=<> fh=<>` (or the ticket
        executor's chosen exact reply shape — document whatever is chosen
        in the ticket's own completion notes for docs/protocol-v2.md's
        eventual update, Step 7 Open Question 1).
- [x] `handleStop` (or `Drivetrain::apply()`'s `NEUTRAL` handling, if the
      graceful behavior lives there per 094-004) is confirmed, over the
      wire (`sim_command_on(h, "STOP", ...)`), to trigger a graceful
      decel-to-zero rather than an instant brake — `STOP`'s wire reply text
      is unchanged (`OK stop`) even though its physical effect changed.
  - [x] Any doc comment in `motion_commands.cpp` still describing 093's
        instant-brake `STOP` rationale (`motion_commands.cpp:710-728`) is
        updated to describe 094's graceful-stop behavior instead.
- [x] `handleTlm` (new): reads `bb.drivetrain`/`bb.motors[]`, replies
      synchronously with measured `enc=`/`vel=` fields plus the executor's
      active/idle flag (`msg::DrivetrainState.active`). No new loop-output
      queue, no `EVT`, no periodic timer — a single request/reply exchange
      exactly like `PING`.
- [x] `Rt::CommandRouter`'s `buildTable()` registers `MOVE` and `TLM`
      alongside 093's `PING`/`HELLO`/`S`/`STOP`.
- [x] Sim end-to-end tests over `sim_command()`/`sim_command_on()`:
  - [x] `MOVE <mm> 0 0` (straight) executes and settles.
  - [x] `MOVE 0 0 <heading>` (pure in-place turn) executes and settles.
  - [x] `MOVE <mm> 0 <heading>` (translate-then-terminal-pivot) executes
        and settles.
  - [x] Each of the above drains to a graceful stop with **no reverse-creep**
        (assert via ground-truth or reported velocity trace never changing
        sign after the segment's own natural completion).
  - [x] `S`/`STOP` still work unchanged over the wire (093's four-verb
        suite stays green, extended, not replaced).
  - [x] `TLM` returns measured `enc=`/`vel=` that track real (simulated)
        wheel motion, not a commanded target.
  - [x] A `MOVE` sent mid-slack takes effect on the very next mandatory
        tick (no added latency beyond the existing one-pass model).
  - [x] Two `MOVE`s queued back-to-back (no intervening tick) both execute
        in order — proves `segmentIn`'s WorkQueue shape does not drop the
        first one the way a Mailbox would.
- [x] `just build-sim` succeeds; `uv run python -m pytest` stays green.

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

## Completion Notes

**`MOVE` grammar (final)**:
```
MOVE <distance_mm> <direction_cdeg> <finalHeading_cdeg>
     [v=<mm/s>] [a=<mm/s^2>] [j=<mm/s^3>]
     [w=<cdeg/s>] [wa=<cdeg/s^2>] [wj=<cdeg/s^3>]
```
`distance_mm` ∈ [-10000, 10000] (signed, 0 allowed — pure in-place turn).
`direction_cdeg`/`finalHeading_cdeg` ∈ [-180000, 180000] (RT's own wider
relative-angle bound, not TURN's absolute ±18000 — both fields here are
RELATIVE deltas). `v`/`a`/`j`/`w`/`wa`/`wj` are optional per-segment
overrides of the executor's boot config; an absent kv defaults to `0.0`,
matching `Motion::Segment`'s own 0-sentinel ("fall back to configured
default") — no separate "was this supplied" bookkeeping needed. Range
ceilings (`kMoveMaxSpeedMax`=3000 mm/s, `kMoveMaxAccelMax`=6000 mm/s²,
`kMoveMaxJerkMax`=60000 mm/s³, `kMoveMaxYawRateMaxCdeg`=72000 cdeg/s,
`kMoveMaxYawAccelMaxCdeg`=500000 cdeg/s², `kMoveMaxYawJerkMaxCdeg`=2000000
cdeg/s³) are sanity ceilings well above the boot defaults
(`source/main.cpp`'s `defaultMotionConfig()`), not a physics model — same
role `D`'s mm bound / `T`'s ms bound / `TURN`'s eps bound already play.
Reply on success: `OK move dist=<distance_mm> dir=<direction_cdeg>
fh=<finalHeading_cdeg>` (wire-unit echo, ints). Errors: `ERR badarg` (< 3
positional tokens) / `ERR range <field>` (any of `distance`/`direction`/
`finalHeading`/`v`/`a`/`j`/`w`/`wa`/`wj` out of bounds), matching the
existing verbs' own convention.

**`TLM` reply shape (final, new verb — see below)**: `TLM` takes no
arguments (`nullptr` parseFn, same precedent as `STOP`/`QLEN`). Reply:
`OK tlm enc=<encL>,<encR> vel=<velL>,<velR> active=<0|1>` — all four
numeric fields are `bb.drivetrain`'s MEASURED per-wheel values (sourced
from `Subsystems::Drivetrain::state()`, itself reading `hardware_.state(
port)` — never a commanded target, since 094-004's rewrite of `state()`).
`active` is `msg::DrivetrainState.active`, which THIS ticket also widened
at the `Drivetrain::state()` level (see "Fix beyond the ticket's own file
list" below) to OR in the owned `Motion::SegmentExecutor`'s own
active/idle status alongside the pre-079 authority flag — otherwise a
`MOVE`-only session (no `S`/`STOP` ever issued) would report `active=0`
throughout, since only `setTwist()`/`setWheelTargets()`/`setNeutral()`
(the `driveIn` escape-hatch path) ever set the pre-existing `active_`
flag.

**Deliberate deviation from protocol-v2.md's existing `TLM` vocabulary**:
`docs/protocol-v2.md` §8 already documents a DIFFERENT, richer `TLM` wire
shape — the pre-093 `STREAM`/`SNAP` family's own unwrapped, non-`OK`-
prefixed `TLM t=<ms> mode=<c> seq=<n> enc=... vel=... pose=...` frame
(built by the still-on-disk-but-unregistered `source/telemetry/
tlm_frame.{h,cpp}`/`source/commands/telemetry_commands.cpp`, per 093
Decision 1 — that whole family stays parked, not revived this ticket).
This ticket's `TLM` is a NEW, DELIBERATELY MINIMAL verb with the SAME
NAME but a DIFFERENT, simpler reply envelope (`OK tlm ...`, wrapped like
every other verb in this trimmed table) — a stand-in the architecture
update's Decision 2 explicitly calls for ("pull-based TLM ... not a full
drain seam"), not a re-implementation of the old rich frame. The
`docs/protocol-v2.md` update this implies (documenting the NEW `TLM`
verb's `OK tlm ...` shape, `MOVE`'s grammar above, and `STOP`'s changed
physical semantics, and reconciling/superseding the old SNAP-emitted raw
`TLM t=...` line's documentation) is DEFERRED per Step 7 Open Question 1 —
not made in this ticket. A future doc pass has everything it needs in this
completion-notes section.

**Graceful `STOP` over the wire**: confirmed end to end —
`test_bare_loop_move_and_tlm.py::
test_stop_over_wire_mid_move_triggers_graceful_decel_no_reverse_creep`
posts a long `MOVE` over `sim.command()`, confirms genuine driving, issues
`STOP` over `sim.command()`, and asserts the measured velocity trace never
reverses sign while decaying to zero. No code change was needed in
`handleStop` itself (094-004 already made `NEUTRAL` graceful at the
`Drivetrain` level via `dispatchEscapeHatch()`'s segment-in-flight check)
— only its doc comment was updated to describe the new physical semantics;
`STOP`'s DIRECT-mode (no segment ever queued) instant-neutral behavior is
unchanged and still covered by `test_bare_loop_commands.py`'s own
pre-existing test.

**Fix beyond the ticket's own file list**: `Drivetrain::state()`
(`source/subsystems/drivetrain.cpp`) needed a small, ticket-driven
widening — `s.active = active_ || (segmentMode_ && executor_.active())`,
replacing the pre-094-006 `s.active = active_`. Without it, `TLM`'s own
acceptance-criterion `active=` flag (an explicit AC bullet) would report
`active=0` throughout every `MOVE`-only test, since the pre-079 authority
flag (`active_`) is set only by the `driveIn` escape-hatch path
(`setTwist()`/`setWheelTargets()`/`setNeutral()`), never by `segmentIn`.
Confirmed via `git grep` that no live (non-parked) code reads
`Drivetrain::state().active`/`Drivetrain::active()` other than this
ticket's own new `handleTlm` — the widening is additive (any DIRECT-mode
caller still sees the identical `active_` value it always did) and does
not regress anything.

**Build/test results**: `just build` (ARM firmware) — succeeded
(FLASH 81.13%, RAM 98.33% used — tight per architecture-update.md Step 7
Risk 1, but fits, unchanged concern from earlier tickets, not this
ticket's own regression). `just build-sim` — succeeded. `uv run python -m
pytest tests/sim tests/unit` — 52 passed (testgui/PySide6 excluded per
this ticket's own instructions — 2 pre-existing failures noted in prior
sprints' knowledge base). New test file:
`tests/sim/unit/test_bare_loop_move_and_tlm.py` (13 tests: 3 `MOVE`-shape
executes-and-settles-no-reverse-creep, 4 `MOVE` argument-error-convention,
1 `STOP`-over-wire-graceful-decel, 1 `S`/`STOP` smoke, 2 `TLM`
measured-vs-commanded + idle-flag, 1 mid-slack-takes-effect-next-tick, 1
two-`MOVE`s-queued-back-to-back).
