---
id: '005'
title: 'Mode machine: extend TLM mode= to I/S/T/D/G'
status: open
use-cases: [SUC-004]
depends-on: ['004']
github-issue: ''
issue: firmware-closed-loop-motion-verbs.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Mode machine: extend TLM mode= to I/S/T/D/G

## Description

Extend `TLM`'s `mode=` field (`source/commands/telemetry_commands.cpp`)
from sprint 082's minimal `I`/`S` (a direct read of
`drivetrain.active()`) to the full `I`/`S`/`T`/`D`/`G` vocabulary,
reading `Subsystems::Planner::state().mode` (`msg::DriveMode`) now that
every verb family (tickets 002-004) exists and its mode-mapping needs are
known. Sequenced last among the motion tickets deliberately (architecture-
update.md's Migration Concerns) — mapping `mode=` once, with every verb
shape settled, avoids redesigning it three times across tickets 002-004.

**Approved mode mapping (architecture-update.md Decision 6, team-lead/
stakeholder-approved as-is):**

| `msg::DriveMode` | Wire char | Verbs that produce it |
|---|---|---|
| `IDLE` | `I` | No active `Planner` command (also true at boot and after any `EVT done`/`STOP`) |
| `STREAMING` | `S` | `S`, `VW` (082), a bare `R` with no `stop=` clause |
| `TIMED` (new, fills the reserved `DriveMode` gap) | `T` | `T`, an `R` with a `stop=` clause, `TURN`, `RT` |
| `DISTANCE` | `D` | `D` |
| `GO_TO` | `G` | `G` |

`TURN`/`RT`/a bounded `R` sharing `'T'` with plain timed drives is an
**approved, deliberate scope decision** (Decision 6) — TestGUI's tour
runner only needs `mode=I` for idle/completion detection (per the issue);
no present consumer needs to distinguish "turning" from "driving a timed
straight" over the wire. Do not invent a new `mode=` character for
`TURN`/`RT` in this ticket.

**Wire keys stay stable.** `mode=`'s field name and every value already
documented (`I`/`S`) are unchanged; this ticket only adds new values to an
existing field per its own documented intent (`docs/protocol-v2.md` §8
already lists `T`/`D`/`G` as the target vocabulary, just not yet
implemented in `source/`).

## Acceptance Criteria

- [ ] `telemetry_commands.cpp`'s `mode=` derivation reads
      `Subsystems::Planner::state().mode`, mapped per the table above,
      instead of `drivetrain.active() ? 'S' : 'I'`.
- [ ] `mode=` is `I` if and only if `Subsystems::Planner::
      hasActiveCommand()` is false.
- [ ] `mode=` returns to `I` at completion of every verb family (`S`/`T`/
      `D`/`R`/`TURN`/`RT`/`G`), independent of whether the corresponding
      `EVT done`/`safety_stop` line was actually received by the host
      (i.e. polling `SNAP` alone, with no `EVT` listener, must show the
      `I` transition).
- [ ] `docs/protocol-v2.md` §8's `mode=` field table is updated to
      document the full `I`/`S`/`T`/`D`/`G` vocabulary and explicitly
      states the `TURN`/`RT`/bounded-`R` share `'T'` decision (with a
      forward pointer to this ticket/architecture-update.md Decision 6 —
      not silently unexplained).
- [ ] No change to `msg::DriveMode`'s wire values beyond `TIMED = 2`
      (already landed in ticket 001) — this ticket is pure consumption of
      an already-extended enum, not a further schema change.

## Implementation Plan

**Approach:** A single, focused change to `telemetryEmit()`'s `mode=`
sourcing rule (082's Decision 7 precedent: `enc=`/`vel=` read live
hardware directly, never a stale commanded-target field — this ticket
applies the same "read the authoritative live state, once, in one place"
discipline to `mode=`).

**Files to modify:**
- `source/commands/telemetry_commands.cpp` (mode= derivation)
- `docs/protocol-v2.md` §8 (mode= table)

**Testing plan:**
- Sim-level tests: `SNAP`/`STREAM` polling through a full `S`/`T`/`D`/`R`/
  `TURN`/`RT`/`G` cycle each, asserting the expected `mode=` value
  throughout and the return to `I` at completion (polled, not
  `EVT`-triggered).
- Existing `tests/sim/` mode= assertions (082's own tests, `I`/`S` only)
  must still pass unchanged.

**Documentation updates:** `docs/protocol-v2.md` §8's `mode=` table, per
Acceptance Criteria.
