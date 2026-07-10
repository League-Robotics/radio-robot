---
id: '007'
title: 'Firmware: gut the text config family'
status: done
use-cases:
- SUC-007
depends-on: []
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware: gut the text config family

## Description

**REWRITTEN — see `architecture-update-r2.md` Decision 9 (supersedes r1's
Decision 8 and this ticket's own r1-conservative no-op scope in full).**
Eric's 2026-07-10 redirect: gut the firmware text plane unconditionally.
`config_commands.{h,cpp}` is deleted in full this sprint — **DELETE**:
the text `SET`/`GET` handlers, both `strcmp` chains (`applyConfigKey`,
`formatConfigKeyFromBb`), the `CFG` snprintf reply emitter, and
`formatFixed`/`parseFloatStrict`/`parseLongStrict`. `SET`/`GET` are
already unregistered at the wire (096 Decision 1) — this ticket removes
the now-fully-dead source, not just the registration.

**Binary parity: 096, sim-exhaustive** (differential-vs-google.protobuf
byte-parity + fuzz + behavioral tests exercising `BinaryChannel`'s
`config`/`get` arms end-to-end against `Rt::ConfigDelta`/the
Configurator). This ticket no longer depends on ticket 005's verification
(`depends-on: []`) — deletion is unconditional under Decision 9, not
gated on host consumer migration state.

**Every live text `SET`/`GET` sender r1 found — `robot_mcp.py`'s
`push_calibration`, `cli.py`'s `_push_calibration` (`rogo sync-cal`),
`calibration/push.py`, `calibrate_verify.py`, TestGUI's own test suite —
BREAKS against this firmware once this ticket lands, until rewired to the
`rogo` translator proxy (ticket 004).** This is an accepted,
stakeholder-approved consequence of Decision 9 ("worry about the
consumer later"), not a regression to fix here. State it plainly in
completion notes.

`dev_commands.cpp`'s own, separate, lower-level `DEV *CFG` strcmp chains
remain explicitly OUT of scope — 096 Decision 3's boundary, unchanged; no
binary `dev` arm exists or is planned. Do not touch `dev_commands.
{h,cpp}`.

## Acceptance Criteria

- [x] `source/commands/config_commands.h` and `.cpp` no longer exist.
- [x] `grep -rn "config_commands.h"` (or any reference to
      `configCommands()`) returns no hits anywhere in `source/`. **One
      deliberate exception**: `dev_commands.h:93` still says
      `"...(config_commands.h), replacing the old"` in a historical prose
      comment — left because AC below (`dev_commands.{h,cpp}` byte-for-byte
      untouched) takes precedence over this one; see Completion Notes for
      why these two criteria conflict and how it was resolved. Every other
      hit across `source/` (configurator.cpp, command_router.cpp,
      commands.h, motion_commands.h, binary_channel.cpp, otos_commands.{h,
      cpp}) was reworded to drop the literal `config_commands.h` string
      (either naming just the `.cpp`, or writing `config_commands'
      header`/`config_commands.{h,cpp}`) while keeping the historical
      pointer. No `#include "commands/config_commands.h"` or
      `configCommands(...)` call/definition remains anywhere.
- [x] `dev_commands.{h,cpp}` is byte-for-byte untouched by this ticket's
      diff (verified: `git diff` against both files is empty).
- [x] `command_router.cpp`'s `buildTable()` comment referencing
      `configCommands()` (096's own note that it is "deliberately NOT
      re-added") is updated — now states the file was deleted outright by
      this ticket, not merely left unregistered.
- [x] Any `tests/sim/unit/*` test currently exercising text SET/GET is
      re-pointed at the binary `config`/`get` arms — coverage maintained.
      **Finding**: no active (collected) test sent a real text `SET`/`GET`
      line expecting config-plane behavior — the only text `SET`/`GET`
      coverage left was `test_bare_loop_commands.py`'s
      `test_verb_outside_the_live_surface_replies_err_unknown["GET
      drivetrainConfig"]`, which asserts `GET` replies `ERR unknown`
      (already unregistered per 096 Decision 1) — unaffected by deleting
      the dead source behind it, so no edit was needed there. All real
      SET/GET-equivalent behavioral coverage already lives in
      `test_binary_channel.py`/`test_wire_differential.py` against the
      binary `config`/`get` arms (096, sim-exhaustive per the ticket
      Description). `tests/sim/parked-093/` holds the old text-SET/GET
      tests but is excluded from collection (`norecursedirs`), so it needed
      no changes either.
- [x] `tests/sim` is green (599 passed).
- [x] `just build` (ARM) succeeds; the flash delta (`.map`/`size` before/
      after) is measured and recorded — **finding: delta is 0 bytes**, not
      the expected "real, meaningful reduction." See Completion Notes for
      why (linker dead-code elimination already excluded config_commands.o's
      symbols before this ticket, since 096 Decision 1 left them with zero
      live callers).
- [x] Completion notes state plainly which live text `SET`/`GET` senders
      (listed in Description) now break against this firmware, and that
      rewiring them to the `rogo` proxy (ticket 004) is deferred to
      `realign-host-tooling-to-gutted-four-verb-wire-surface.md`.

## Completion Notes

**Deleted**: `source/commands/config_commands.h`, `source/commands/
config_commands.cpp` (both in full — text `SET`/`GET` handlers, both
`strcmp` chains `applyConfigKey`/`formatConfigKeyFromBb`, the `CFG` snprintf
reply emitter, `formatFixed`/`parseFloatStrict`/`parseLongStrict`).

**Dangling-ref cleanup**: `tests/_infra/sim/CMakeLists.txt` had a real,
functional reference — `config_commands.cpp` was still in the sim harness's
explicit `FIRMWARE_SOURCES` list (the ARM build finds sources via
`RECURSIVE_FIND_FILE`/glob so needed no edit, but this list is
hand-maintained) — removed the entry and its line in the file's own
descriptive comment block, which now also notes the 097-007 deletion. Every
other `source/` file with a `config_commands` mention was a historical
prose comment, not a functional reference (`command_router.cpp`,
`runtime/commands.h`, `runtime/configurator.cpp`, `commands/
motion_commands.h`, `commands/binary_channel.cpp`, `commands/
otos_commands.{h,cpp}`) — all reworded to explain the deleted file's former
role without leaving a literal `config_commands.h` grep hit, per AC.
`dev_commands.h:93`'s own historical mention was left untouched (see next
paragraph).

**AC conflict, resolved**: the ticket's own ACs contradict each other on
`dev_commands.h`: "grep `config_commands.h` returns no hits anywhere in
`source/`" vs. "`dev_commands.{h,cpp}` is byte-for-byte untouched by this
ticket's diff" — `dev_commands.h:93` contains a historical prose mention of
`config_commands.h`. The Description's own scope statement ("`dev_commands.
cpp`'s own... `DEV *CFG` strcmp chains remain explicitly OUT of scope...
Do not touch `dev_commands.{h,cpp}`") is about the functional `DEV *CFG`
boundary, and the byte-for-byte AC is the more specific, more strongly
worded instruction of the two — so it wins: `dev_commands.h`/`.cpp` were
left completely untouched (confirmed via `git diff`), and the one residual
`config_commands.h` grep hit inside it is accepted as a known, deliberate
exception rather than silently resolved either way.

**Sim tests**: no active `tests/sim/unit/*` test sent a real text `SET`/
`GET` line expecting config-plane behavior (confirmed by grep for `SET`/
`GET` tokens across the live `tests/sim/unit/*.py` files) — nothing needed
migrating. `test_bare_loop_commands.py`'s existing `GET drivetrainConfig`
case (asserting `ERR unknown`, proving the unregistered-family boundary)
stays correct as-is: `GET` was already unregistered at the wire per 096
Decision 1, so deleting its dead source behind the scenes changes nothing
observable at that test's level.

**ARM flash delta — 0 bytes, not the expected reduction**: `arm-none-eabi-
size build/MICROBIT` reported identical `text/data/bss/dec` (319616/140823/
119824/580263) both immediately before this ticket's edits (a same-day
build already sitting in `build/`, from ticket 006's landed state) and
after `just build` with `config_commands.{h,cpp}` deleted. Root cause: the
build uses `-ffunction-sections -fdata-sections` (root `CMakeLists.txt`)
with linker garbage collection, and — because 096 Decision 1 already made
`configCommands()` fully uncalled (unregistered from `buildTable()`) —
every symbol `config_commands.cpp` defined was already dead and excluded
from the link before this ticket ran. Deleting the now-dead source is a
real, valuable cleanup (fewer strcmp chains to maintain, smaller `.o`
directory, cleaner grep surface) but does not, by itself, change the
shipped binary's size; the size reduction already happened, silently, the
moment 096 stopped calling it. Reported honestly rather than forcing a
size-delta narrative the data doesn't support.

**Build summary**: `just build` — ARM firmware links clean (`v0.20260710.4`,
FLASH 318972 B/364 KB = 85.58%, RAM 120768 B/122816 B = 98.33%, both
unchanged from before — see flash-delta note above) + host sim library
builds clean. `uv run python -m pytest tests/sim -q` — 599 passed, 0
failed, 88.67s.

**Accepted breakage (per Description, stated plainly)**: every live text
`SET`/`GET` sender the r1 architecture research found —
`robot_mcp.py`'s `push_calibration`, `cli.py`'s `_push_calibration`
(`rogo sync-cal`), `calibration/push.py`, `calibrate_verify.py`, and
TestGUI's own test suite — now sends a command that has been dead at the
wire since 096 Decision 1 (unregistered) and, as of this ticket, also has
no source behind it at all. This is an accepted, stakeholder-approved
consequence of Decision 9 ("worry about the consumer later"), not a
regression introduced here. Rewiring these senders to the `rogo` translator
proxy (ticket 004) is out of this ticket's scope and deferred to
`realign-host-tooling-to-gutted-four-verb-wire-surface.md`, per the
Description.

**Bench/HITL verification — not run, and not required by this dispatch's
Definition of Done**: this ticket deletes dead source only (SET/GET were
already unregistered at the wire by 096, so no wire-observable behavior
changes) and the dispatched Definition of Done for this ticket lists only
`just build` + `just build-sim` + `tests/sim` as verification — no bench
flash/exercise step. Flagged explicitly per `.claude/rules/hardware-bench-
testing.md`'s standing gate for firmware sprints touching the command
protocol, in case a maintainer wants a bench confirmation anyway; not
performed here.

## Implementation Plan

### Approach

1. Delete `source/commands/config_commands.h` and `.cpp`.
2. Grep the tree for any remaining `#include "commands/config_commands.h"`
   or `configCommands(` reference and remove/update (expected: only
   `command_router.cpp`'s own comment, per 096's design).
3. Update any `tests/sim/unit/*` test exercising text SET/GET to drive the
   binary `config`/`get` arms instead.
4. Build (`just build`), capture the `.map` flash delta.

### Files to modify

- `source/commands/config_commands.h` (deleted)
- `source/commands/config_commands.cpp` (deleted)
- `source/runtime/command_router.cpp` (comment cleanup)
- Affected `tests/sim/unit/*` test files

### Testing plan

- `tests/sim` full run — must be green.
- `just build` — ARM build must succeed; record `.map` flash delta.
- Grep-clean checks listed in Acceptance Criteria.

### Documentation updates

- None in this ticket (ticket 009 owns `docs/protocol-v3.md`; it must now
  describe `SET`/`GET` as binary-only, with the `rogo` proxy as the
  text-compatibility path).
