---
id: '008'
title: 'Firmware atomic cutover: ASCII-only dispatch, case-direction, motion verbs'
status: open
use-cases: [SUC-003, SUC-005]
depends-on: ['007']
github-issue: ''
issue:
- protocol-v6-spec.md
- protocol-and-implementation-simplification-review.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Firmware atomic cutover: ASCII-only dispatch, case-direction, motion verbs

## Description

Execute the atomic cutover on firmware: switch `Core::Comms` dispatch to
ASCII-only line grammar for the motion verbs (`MOVE`/`WHEELS`/`GOTO`/
`STOP`/`ESTOP`/`SEED`/`CAL`, spec §5), enforce case-sensitive direction
(commands UPPERCASE, replies lowercase, spec §2.1), and drop binary framing
for these verbs — along with `HELLO`/`PING`/`ID`/`VER`/`STATUS`/`HELP`
(spec §4), which share `Core::Comms`'s dispatch table. This is spec §13
migration stage 4, the one step the spec calls "atomic": firmware (this
ticket) and host (ticket 009) cut over together, since after this ticket the
host cannot talk to a not-yet-cut-over firmware and vice versa.
**Nothing else should land between tickets 008 and 009** (`sprint.md`
Migration Concerns).

## Acceptance Criteria

- [ ] `MOVE <kind> <a> <b> <c> <stop> <limit> <timeout> #<id>` parses and
      dispatches per spec §5.1 (both `t` twist and `w` wheels kinds);
      `WHEELS <left> <right> <duration> [#<id>]` per §5.2 (duration required,
      ceiling 5000 ms); `GOTO <x> <y> <frame> <speed> <arrive> <timeout>
      #<id>` per §5.3; `STOP #<id>` and `ESTOP` per §5.4; `SEED <x> <y> <h>
      [#<id>]` per §5.5; `CAL [<samples>] [#<id>]` per §5.6.
- [ ] `ESTOP` halts now: zeroes wheel targets and clears the planner's
      active + pending queue in the same cycle, discarded entries get no
      `done`, carries no id, is never acked.
- [ ] `STOP #<id>` is a planned stop: an ordinary queue entry, `ok #<id>` on
      enqueue, `done #<id> stop` when actually at rest.
- [ ] A lowercase reply line heard on a shared channel (e.g. another
      robot's `dbg`/`t`) is silently dropped and does NOT increment
      `malformedCount_`/the malformed flag bit (spec §2.1, SUC-005) — verb
      lookup is case-sensitive.
- [ ] An uppercase line with an unknown verb still increments the malformed
      counter — the distinction that matters is direction (case), not "any
      bad line."
- [ ] v5's `replace` flag is dropped; preemption is `ESTOP` then a new
      `MOVE` (spec §5.1).
- [ ] The banner reply is rewritten per spec §4.1 — lowercase verb, space
      separators (`device NEZHA2 robot <name> <serial>`) — the one
      byte-frozen string v6 breaks, called out explicitly.
- [ ] `HELLO`/`PING`/`ID`/`VER`/`STATUS`/`HELP` (spec §4) all move to the
      ASCII grammar in this same ticket, since they share `Core::Comms`'s
      dispatch table with the motion verbs.
- [ ] `grep -rn "<an existing sibling .cpp>" src --include=*.txt
      --include=*.py` is run BEFORE this ticket's new/changed firmware
      `.cpp` files are considered done, to catch every hand-maintained
      source list (CMake plus every `src/tests/sim/**/test_*.py` file's own
      `_APP_SOURCES` list) — a missed one produces a link error far from
      the change.
- [ ] The binary command plane for these verbs is disabled/dead-ended in
      this ticket (superseded), not yet physically deleted — deletion is
      ticket 013.

## Testing

- **Existing tests to run**: sim/unit Comms dispatch suites (expect
  breaking changes to binary-plane-dependent tests for these specific
  verbs — update, don't skip).
- **New tests to write**: sim/unit tests for every motion verb's ASCII
  grammar, the case-sensitivity/malformed-counter distinction, `ESTOP` vs.
  `STOP` semantics, and the banner case break.
- **Verification command**: `uv run pytest src/tests/sim/unit -k "comms or
  motion or dispatch"`, plus a full firmware `--clean` build.
