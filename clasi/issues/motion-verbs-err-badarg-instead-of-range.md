---
status: pending
sprint: '054'
---

# Motion verbs reply `ERR badarg <field>` instead of `ERR range <field>` for out-of-range args

## Problem

Bench testing on **tovez** (firmware `0.20260628.44`) found that out-of-range
arguments to the motion verbs **S / T / D / R** now produce
`ERR badarg <field>` instead of the original `ERR range <field>`. Observed live:

- `S 99999 0` → `ERR badarg l`  (expected `ERR range l`)
- `T 0 0 0`   → `ERR badarg ms` (expected `ERR range ms`)

The field detail token (`l` / `r` / `ms` / `mm`) is correct; only the error
**code** changed from `range` to `badarg`. This is a (minor) wire-protocol
behavior change vs the pre-sprint-051 firmware.

## Root cause

During the sprint **051** ArgSchema migration and the sprint **052/053**
stop-condition work, S/T/D/R were converted to custom `parseFn`s (to forward
`stop=` clauses) and registered via `makeCmd(..., errFmt="badarg")`. On a
ranged-value failure the parser sets `res.err.code = nullptr` and
`res.err.detail = def.name`; the dispatcher then formats the code from
`desc.errFmt`, which defaults to `"badarg"`. The range-specific `"range"` code
was lost. (See `source/commands/MotionCommands.cpp` `parseS/parseT/parseD/parseR`
and the registration table; and `source/commands/CommandProcessor.cpp` error
formatting.)

## Why it wasn't caught

The simulation tests (e.g. `tests/simulation/unit/test_motion_verbs_v2.py`)
assert loosely and do not distinguish `range` vs `badarg`, so the regression
passed CI.

## Proposed fix

- Register S/T/D/R (and any other ranged motion verbs) so a ranged-value
  failure emits `ERR range <field>`, while genuine arg-count failures still
  emit `ERR badarg`. Likely approach: have the ranged-fail path set
  `res.err.code = "range"` and make the dispatcher honor `res.err.code` when
  present, falling back to `desc.errFmt` otherwise.
- Tighten the sim tests (`test_motion_verbs_v2.py` and the protocol /
  stop-condition coverage suites) to assert the **exact** error code+field
  string so this can't regress silently again.

## Acceptance

- `S 99999 0` → `ERR range l`; `S 0 99999` → `ERR range r`
- `T 0 0 0` → `ERR range ms`; `D 0 0 0` → `ERR range mm`
- Arg-count errors still → `ERR badarg`
- Sim suite green; ideally bench-verify on tovez.

## Notes

Severity: minor (protocol nuance; does not affect normal operation). Found
during post-roadmap bench validation of sprints 048–053.
