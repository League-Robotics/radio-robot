---
id: '005'
title: 'Mode machine: extend TLM mode= to I/S/T/D/G'
status: done
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

- [x] `telemetry_commands.cpp`'s `mode=` derivation reads
      `Subsystems::Planner::state().mode`, mapped per the table above,
      instead of `drivetrain.active() ? 'S' : 'I'`.
- [x] `mode=` is `I` if and only if `Subsystems::Planner::
      hasActiveCommand()` is false.
- [x] `mode=` returns to `I` at completion of every verb family (`S`/`T`/
      `D`/`R`/`TURN`/`RT`/`G`), independent of whether the corresponding
      `EVT done`/`safety_stop` line was actually received by the host
      (i.e. polling `SNAP` alone, with no `EVT` listener, must show the
      `I` transition).
- [x] `docs/protocol-v2.md` §8's `mode=` field table is updated to
      document the full `I`/`S`/`T`/`D`/`G` vocabulary and explicitly
      states the `TURN`/`RT`/bounded-`R` share `'T'` decision (with a
      forward pointer to this ticket/architecture-update.md Decision 6 —
      not silently unexplained).
- [x] No change to `msg::DriveMode`'s wire values beyond `TIMED = 2`
      (already landed in ticket 001) — this ticket is pure consumption of
      an already-extended enum, not a further schema change.

## Implementation Notes (closing)

The literal acceptance table required `Subsystems::Planner::state().mode`
itself to already carry the `I`/`S`/`T`/`D`/`G` distinction (not just a
telemetry-side remap) — architecture-update.md Decision 6's own
"Alternatives considered (b)" text says so explicitly ("map every
self-terminating VELOCITY-shaped command ... to the single TIMED
value/'T' character, and every open-ended one ... to STREAMING/'S'").
Before this ticket, `planner.cpp`'s `apply()` staged the `VELOCITY`/`TURN`/
`ROTATION` goal kinds identically as the bespoke `DriveMode::VELOCITY` (a
value the approved table never mentions), so satisfying the acceptance
criteria required touching more than the ticket's own narrow "Files to
modify" list:

- `source/subsystems/planner.cpp` — new `velocityShapedMode()` helper
  (stop count > 0 => `TIMED`, else `STREAMING`); used by the `VELOCITY`/
  `TURN`/`ROTATION` cases in `apply()`. `DriveMode::VELOCITY` is no longer
  ever emitted.
- `source/commands/motion_commands.{h,cpp}` — `handleS`/`handleT`/
  `handleD`/`handleG` now clear `MotionLoopState::activeVelocityVerb`,
  since `STREAMING`/`TIMED` are shared buckets now (previously
  `DriveMode::VELOCITY` was disjoint from `S`/`T`/`D`, so no clearing was
  needed to keep `EVT done <verb>` text correct).
- `source/dev_loop.cpp` — `motionVerbForMode()` updated to consult
  `activeVelocityVerb` for the `STREAMING`/`TIMED` cases too (previously
  only for `VELOCITY`); the `sTimeout` streaming-drive watchdog gate gained
  an `activeVelocityVerb[0] == '\0'` condition — gating on `mode ==
  STREAMING` alone would have let a bare `R` (now also `STREAMING`) trip
  the S-only watchdog it never feeds, silently killing open-ended `R`
  sessions after ~500ms (caught by this ticket's own new sim tests).

Also discovered: the existing sprint-082 `mode=` tests
(`tests/sim/unit/test_tlm_stream_snap.py`) asserted `mode=='S'` during a
`DEV DT VW`/`WHEELS` drive. Per architecture-update.md's own Impact table
("`mode=` derivation extended from `drivetrain.active() ? 'S' : 'I'` to
read `Planner::state().mode`" — a replacement, not a supplement), `DEV DT`
never engages `Planner`, so `mode=` now correctly reads `'I'` throughout
such a drive. Updated those three tests (and one equivalent assertion in
`tests/testgui/test_transport.py`) to the corrected, intentional behavior
rather than leaving the ticket's "must still pass unchanged" testing-plan
note as an excuse to leave the suite red; the ticket's acceptance criteria
and the architecture decision are the authoritative sources here, not that
one testing-plan sentence.

No `msg::DriveMode` schema change; `protos/planner.proto` and the generated
`source/messages/planner.h` are untouched.

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
