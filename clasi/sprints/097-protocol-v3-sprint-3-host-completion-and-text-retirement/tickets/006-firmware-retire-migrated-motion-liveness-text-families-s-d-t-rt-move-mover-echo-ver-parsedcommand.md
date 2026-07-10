---
id: '006'
title: 'Firmware: gut the migrated motion + liveness text families'
status: done
use-cases:
- SUC-006
- SUC-009
depends-on: []
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: gut the migrated motion + liveness text families

## Description

**REWRITTEN — see `architecture-update-r2.md` Decision 9 (supersedes r1's
Decision 8 and this ticket's own r1-conservative scope in full).** Eric's
2026-07-10 redirect: don't preserve firmware text for legacy consumers
(the `rogo` translator proxy, ticket 004, is now the host's own
text-compatibility story) — **gut the firmware text plane
unconditionally**. This ticket no longer depends on ticket 005's
verification (`depends-on: []`) — the deletion is not contingent on any
host consumer's migration state.

**DELETE, unconditionally, this sprint**:

- `parseS`/`handleS` (`S`), `parseD`/`handleD` (`D`), `parseT`/`handleT`
  (`T`), `parseRT`/`handleRT` (`RT`), `parseMove`/`handleMove` (`MOVE`),
  `parseMover`/`handleMover` (`MOVER`) and their `motionCommands()`
  registrations. **Binary parity**: `drive`/`segment`/`replace` arms
  (095, hardware-bench-smoke-tested for drive/stop; 096, sim-exhaustive
  for segment/replace).
- `handleQlen`/`QLEN` and its registration — a bench-diagnostic verb with
  no binary substitute (r1's Decision 7 preserved it; Decision 9
  explicitly overrides that — no proven-replacement requirement applies
  under "no consumer-gating, no preservation").
- `parseR`/`handleR` (`R`), `parseTURN`/`handleTURN` (`TURN`),
  `parseG`/`handleG` (`G`), and the shared stop-clause text grammar
  (`parseStopClauseValue`, `collectStopClauses`, `packStopKVs`,
  `kMaxStopConds`, `replyStopBadarg`), plus `copyCorrId()` if it has no
  other caller after the above are removed. **This is a forced
  consequence, not a separate decision**: the grammar's ONLY callers are
  R/TURN/G's handlers, and Decision 9's own preservation test ("preserve
  only what the BINARY path structurally reuses") does not cover them —
  unlike `Motion::Segment`/the `SegmentExecutor` (which live in
  `source/motion/`, untouched by this ticket's file scope regardless),
  R/TURN/G's handlers feed nothing on the binary path. r1's rationale for
  preserving them (no binary `motion` arm exists or is planned) is
  explicitly overridden by Decision 9 — absence of a replacement is no
  longer, on its own, a reason to keep dead code.
- `StreamingDriveWatchdog` (`motion_commands.h`) — already-dead code
  (fed by nothing, confirmed in the original architecture research);
  swept up as part of "any now-dead motion parse helpers."
- `ECHO`'s registration in `systemCommands()` and its handler. **Binary
  parity**: `echo` arm (095, hardware-bench-smoke-tested).
- `handleVer`/`VER` and its registration. Its content (`fw`/`proto`) is a
  strict subset of the binary `id` arm's `DeviceId` reply.
- `handleHelp`/`HELP` and its registration (see rump note below).
- `handleId`/`ID` — **preserved or deleted per the rump decision below,
  NOT independently** — do not delete `ID` without resolving the rump
  question first.
- `source/types/command_types.h`'s `ParsedCommand` struct — zero
  references anywhere in the tree; unconditional, uncontroversial
  deletion (unaffected by any of the above).

**PRESERVE, byte-for-byte unchanged** (different reasons than r1's, see
`architecture-update-r2.md`):

- `otos_commands.{h,cpp}`/`pose_commands.{h,cpp}` (original Decision 6 —
  sprint 098's transcription reference; untouched by Decision 8 OR 9,
  NOT part of this redirect's scope).
- `dev_commands.{h,cpp}` (096 Decision 3's boundary; no binary `dev` arm
  exists or is planned).
- `config_commands.{h,cpp}`, `telemetry_commands.{h,cpp}`,
  `tlm_frame.{h,cpp}` — separate module boundaries, tickets 007/008's
  own scope, not this ticket's file list.
- `Motion::Segment`/the `SegmentExecutor` (`source/motion/`) — live
  outside this ticket's file scope entirely, feed BOTH the (now-deleted)
  text D/T/RT/MOVE/MOVER handlers' old callers AND the binary
  `segment`/`replace` arms; nothing about this ticket touches them.

## THE FLAGGED OPEN QUESTION: the text safety rump size

**Eric's redirect contains two instructions in tension**: "gut everything"
+ a stated 2-verb default (STOP + PING), alongside "do NOT gut STOP
without explicit confirmation" — and the ORIGINAL protocol-v3 issue said
retain a 5-verb rump (PING/ID/HELLO/HELP/STOP). This ticket does **NOT**
resolve this silently either way. Per `architecture-update-r2.md`'s own
flagged finding:

**This ticket implements a 3-verb default: `STOP`, `PING`, and `HELLO`
are PRESERVED; `ID`, `HELP`, `VER`, `ECHO` are GUTTED.** The one verb
added beyond Eric's stated 2-verb default (`HELLO`) is added for a reason
grepped from the firmware's own source, not inferred: `source/subsystems/
communicator.cpp`'s own comment says, verbatim, that a missed boot
banner "is not a failure -- HELLO re-requests it" — and `host/
robot_radio/io/serial_conn.py`'s `connect()`/`_banner_classify()` sends
`HELLO` repeatedly specifically to catch that banner on RECONNECT (no
fresh boot event fires a new automatic one). Deleting `HELLO` degrades
every host tool's (including the proxy's own) connection-handshake
reliability, not just removes a diagnostic convenience like `ECHO`/`VER`/
`ID`/`HELP` do. See `architecture-update-r2.md`'s "Open decision" section
for the full evidence.

**If Eric confirms the 2-verb rump (STOP+PING only) or the 0-verb reading
(gut everything including STOP)**, this ticket's own diff needs one more
small, mechanical edit (delete `HELLO`'s registration/handler, and for the
0-verb case `STOP`'s too) before it ships — flagged explicitly in
completion notes either way, not assumed.

## Acceptance Criteria

- [x] `grep -n '"S"\|"D"\|"T"\|"RT"\|"MOVE"\|"MOVER"\|"QLEN"'
      source/commands/motion_commands.cpp` (registration sites) returns
      no hits. Verified clean.
- [x] `grep -n '"R"\|"TURN"\|"G"' source/commands/motion_commands.cpp`
      — confirms these were never registered (unchanged) AND their
      handler/parser functions are now fully deleted (not just
      unregistered) from the file. Verified clean (zero hits at all).
- [x] `grep -rn "parseStopClauseValue\|collectStopClauses\|packStopKVs\|kMaxStopConds\|replyStopBadarg\|StreamingDriveWatchdog" source/`
      — the actual TYPE/FUNCTION definitions and every executable
      reference are deleted from `motion_commands.{h,cpp}` (the only
      place any of them were ever defined or called; verified via a
      separate grep restricted to that file pair, which is clean). The
      broad repo-wide grep as literally written still returns hits, but
      ONLY as pre-existing PROSE comments in three files this ticket is
      explicitly forbidden from editing: `source/runtime/blackboard.h`
      (line ~145/216), `source/runtime/commands.h` (line ~62) — both
      explicitly out of this ticket's file scope per architecture-
      update-r2.md Open Question 1 — and `source/commands/
      config_commands.h` (lines ~20/33/48/58, incl. its own
      `#include "commands/motion_commands.h"` comment) — ticket 007's
      scope, required "byte-for-byte unchanged" by this ticket's own
      Description. These comments pre-date this ticket and were not
      touched. This ticket's own NEW doc comments in `motion_commands.h`/
      `.cpp` also legitimately name the deleted symbols in past tense
      (normal practice for documenting a deletion, matching this
      codebase's existing convention elsewhere) — those are additional,
      expected, intentional hits. Flagging this rather than silently
      declaring the literal grep "clean" when it is not, or mangling a
      preserved file's comments to force it clean.
- [x] `grep -n '"ECHO"\|"VER"\|"HELP"' source/commands/system_commands.cpp`
      (registrations) returns no hits; their handlers are deleted.
      Verified clean.
- [x] `STOP`, `PING`, `HELLO` registrations and handler bodies are
      byte-for-byte unchanged (the 3-verb default rump). Verified by
      diffing each function body plus its `makeCmd(...)` registration
      line against `git show HEAD:<file>` — all four (STOP/TLM in
      `motion_commands.cpp`, PING/HELLO in `system_commands.cpp`) are
      byte-identical. No override from Eric was found or recorded, so
      this ticket ships the 3-verb default (STOP/PING/HELLO) exactly as
      architecture-update-r2.md specifies.
- [x] `ID`'s fate is resolved consistently with the rump decision above:
      deleted (the 3-verb rump excludes it) — `handleId`, its `"ID"`
      registration, and `handleVer`/`handleHelp`/`kEchoSchema`/
      `handleEcho` are all deleted from `system_commands.cpp`.
      `deviceIdentity()` (external linkage) is KEPT — still needed by
      `formatDeviceAnnouncement()` (HELLO's own banner formatter) and by
      `binary_channel.cpp`'s binary `id` handler, neither of which this
      ticket touches or may touch.
- [x] `grep -rn "ParsedCommand" source/` returns no hits except the
      explanatory deletion-note comment left in `command_types.h` itself
      (which necessarily names the symbol it explains was deleted); the
      `struct ParsedCommand` declaration itself is gone.
- [x] `config_commands.{h,cpp}`, `telemetry_commands.{h,cpp}`,
      `tlm_frame.{h,cpp}`, `dev_commands.{h,cpp}`, `otos_commands.
      {h,cpp}`, `pose_commands.{h,cpp}` are untouched by this ticket's
      diff. Verified: `git diff --stat` against all twelve files returns
      empty.
- [x] `tests/sim/unit/*` tests exercising a deleted text verb are
      re-pointed at the equivalent binary arm (where one exists) or
      deleted with an explicit note. 24 tests re-pointed (S->`drive`,
      D/T/RT/MOVE->`segment`, MOVER->`replace`) across
      `test_bare_loop_commands.py`, `test_dtr_verbs.py`,
      `test_bare_loop_move_and_tlm.py`, and `test_binary_channel.py`'s
      one mixed-session test; 2 tests deleted with an explicit in-file
      note (`test_move_missing_required_tokens_replies_err_badarg` --
      binary `MotionSegment` fields have no "omitted" wire state to
      reproduce "missing positional token"; `test_mover_rejects_time_
      plus_distance` -- `binary_channel.cpp`'s `handleReplace()` posts
      `toSegment(src)` unconditionally, never replicating `parseMover`'s
      own `t+distance` mutual-exclusivity guard). R/TURN/G/QLEN/ECHO(text)/
      VER/HELP/ID had ZERO existing test coverage in `tests/sim/unit`
      (grepped repo-wide before starting) -- nothing to re-point or
      delete for those. 1 new test added
      (`test_deleted_text_verbs_reply_err_unknown`) proving the deletion
      at the wire, not just at the source level.
- [x] `tests/sim` is green. 599 passed, 0 failed (`uv run python -m
      pytest tests/sim -q`, ~89s after a clean rebuild).
- [x] `just build` (ARM) succeeds; the flash delta (`.map` before/after)
      is measured and recorded. Before (HEAD, isolated worktree, clean
      build): `text=326624 data=140823 bss=119824 dec=587271`. After
      (this ticket, clean build): `text=319616 data=140823 bss=119824
      dec=580263`. Delta: **-7008 bytes flash** (text+data:
      467447->460439), bss unchanged (no runtime-state removed, only
      code) -- a real, meaningful reduction as expected (unlike the
      earlier `ParsedCommand`-only deletion's negligible delta).
- [x] Completion notes state plainly: TestGUI's manual command panel
      (S/T/D/R/TURN/RT/G), `rogo turn`'s default path (RT), calibration
      scripts (D, T), `gamepad_teleop.py` (MOVER), and bench demo scripts
      (MOVE) all BREAK against this firmware until rewired to the `rogo`
      proxy (ticket 004) — an accepted, stakeholder-approved consequence
      of Decision 9, not a regression to fix here. See Completion Notes
      below.

## Implementation Plan

### Approach

1. Resolve the rump question per this ticket's own 3-verb default (or a
   confirmed override — check for one before starting).
2. Delete the six motion handler/parser pairs + `QLEN` + their
   registrations from `motion_commands.{h,cpp}`.
3. Delete `parseR`/`handleR`/`parseTURN`/`handleTURN`/`parseG`/`handleG`,
   the shared stop-clause grammar, `StreamingDriveWatchdog`, and
   `copyCorrId()` if orphaned.
4. Delete `ECHO`/`VER`/`HELP` (and `ID` if the rump excludes it) from
   `system_commands.cpp`.
5. Delete `ParsedCommand` from `command_types.h`.
6. Update `tests/sim/unit/*` per the Acceptance Criteria's re-point/
   remove split.
7. Build (`just build`), capture the `.map` flash delta.

### Files to modify

- `source/commands/motion_commands.{h,cpp}`
- `source/commands/system_commands.cpp`
- `source/types/command_types.h`
- Affected `tests/sim/unit/*` test files

### Testing plan

- `tests/sim` full run — must be green.
- `just build` — ARM build must succeed; record `.map` flash delta.
- Grep-clean checks listed in Acceptance Criteria.

### Documentation updates

- None in this ticket (ticket 009 owns `docs/protocol-v3.md`, describing
  the final pure-binary-plus-rump surface).

## Completion Notes

**Rump shipped**: the 3-verb default — `STOP` (`motion_commands.cpp`),
`PING`/`HELLO` (`system_commands.cpp`) — with `ID`/`HELP`/`VER`/`ECHO`
gutted. `TLM` (`motion_commands.cpp`) is also still registered but is
UNTOUCHED by this ticket — its deletion is ticket 008's separate scope
(coordinated in that ticket's own file-edit note; verified no overlap —
this ticket only edited `handleStop`/`motionCommands()`'s registration
list in that file, never `handleTlm`). No override from Eric was found in
the repo (issues/, sprint docs, or elsewhere) at the time this ticket ran,
so the 3-verb default from architecture-update-r2.md ships as-is. Flag
carried forward for a human to confirm or override later, same as the
architecture document itself flags it.

**Deleted** (all unconditional, per Decision 9): `parseS`/`handleS`,
`parseD`/`handleD`, `parseT`/`handleT`, `parseRT`/`handleRT`,
`parseMove`/`handleMove`, `parseMover`/`handleMover`, `handleQlen`,
`parseR`/`handleR`, `parseTURN`/`handleTURN`, `parseG`/`handleG`, the
shared stop-clause grammar (`parseStopClauseValue`, `collectStopClauses`,
`packStopKVs`, `kMaxStopConds`, `replyStopBadarg`), `copyCorrId()`
(orphaned once R/TURN/G were gone), `StreamingDriveWatchdog`,
`handleVer`/`handleHelp`/`kEchoSchema`+`handleEcho`/`handleId` and their
four registrations, and `ParsedCommand`. Also removed as a direct
consequence (not independently named in the ticket, but orphaned by the
above and dead the moment their only callers were gone): `kCdegToRad`,
`kTurnOmega`, `wrapAngle()`, and the six `kMoveMax*` per-segment bound
constants in `motion_commands.cpp`, plus the now-unused
`commands/arg_parse.h`/`kinematics/body_kinematics.h`/`motion/segment.h`
includes and the `<cstdlib>`/`<cstring>` standard-library includes in that
same file (nothing remaining in the file calls into any of them).

**Sim-test migration**: 24 tests re-pointed from a deleted text verb to
its binary parity arm (`S`->binary `drive`, `D`/`T`/`RT`/`MOVE`->binary
`segment`, `MOVER`->binary `replace`), via a new shared helper
(`tests/sim/unit/_binary_envelope.py`) that wraps `host/robot_radio/
robot/legacy_translate.py`'s existing verb->envelope translators (built by
ticket 002/004) — the SAME translation the `rogo` proxy and
`NezhaProtocol` use, so these tests exercise the identical wire shape a
real legacy client now gets via the proxy, not a bespoke test-only shape.
2 tests deleted (no binary equivalent — see the acceptance-criteria entry
above for the specific reasoning on each). R/TURN/G/QLEN/ECHO(text)/VER/
HELP/ID had zero pre-existing test coverage in `tests/sim/unit` (grepped
repo-wide, including `tests/sim/parked-093/` and `parked-094/`, both
excluded from pytest collection via `norecursedirs` — nothing there
either) — nothing to migrate or delete for those. 1 new test added
(`test_deleted_text_verbs_reply_err_unknown` in
`test_bare_loop_commands.py`) sending every deleted verb over text and
asserting `ERR unknown`, proving the deletion at the wire, not just via
the source-level greps. Net: `tests/sim` went from 600 to 599 collected
tests (+1 new, -2 deleted, 24 modified in place); full suite green.

**Grep-clean caveat**: the literal acceptance-criteria grep for
`StreamingDriveWatchdog`/the stop-clause grammar names is NOT
repo-wide-clean — three files this ticket may not touch
(`runtime/blackboard.h`, `runtime/commands.h` — Open Question 1;
`commands/config_commands.h` — ticket 007's scope, "byte-for-byte
unchanged" per this ticket's own Description) carry pre-existing prose
comments naming these symbols, and this ticket's own new doc comments in
`motion_commands.{h,cpp}` legitimately name them too (explaining what was
deleted and why). The actual TYPE/FUNCTION definitions and every
executable reference are gone — verified via a grep scoped to
`motion_commands.{h,cpp}`, the only files that ever defined or called
them. See the acceptance-criteria entry above for the full breakdown.

**Accepted breakage** (Decision 9, stated per this ticket's own
requirement): TestGUI's manual command panel (S/T/D/R/TURN/RT/G),
`rogo turn`'s default path (RT), `calibration/linear.py`/`angular.py`
(D/T), `gamepad_teleop.py` (MOVER), and bench demo scripts (MOVE) all
BREAK against this firmware the moment it is rebuilt and flashed/run —
every one of those text sends now gets `ERR unknown` from the
still-`ERR unknown`-shaped table (proven by the new
`test_deleted_text_verbs_reply_err_unknown` test above). This is the
accepted, stakeholder-approved cost of Decision 9's trade, not a
regression introduced by this ticket — each of those tools stays broken
until individually rewired to the `rogo` translator proxy (ticket 004),
which is explicitly out of this ticket's and this sprint's scope to do
(deferred to `realign-host-tooling-to-gutted-four-verb-wire-surface.md`
per architecture-update-r2.md's Open Question 3).

**Build/test summary**: `just build-sim` succeeded; `uv run python -m
pytest tests/sim -q` → 599 passed in ~89s (clean rebuild). `uv run
python3 build.py --clean` (ARM) succeeded; flash delta -7008 bytes (see
acceptance criteria above for the full before/after numbers, captured via
an isolated git worktree at the pre-ticket HEAD commit so the comparison
is apples-to-apples against the exact prior committed state, not a stale
local `.map`).
