---
id: '010'
title: Tier-1 fault-knob matrix + lag-on validation
status: open
use-cases: [SUC-012]
depends-on: ['007', '008']
github-issue: ''
issue: motion-stack-v2-a-self-contained-stateless-motion-control-subsystem.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Tier-1 fault-knob matrix + lag-on validation

## Description

Exercise the sim's existing fault knobs (`motor_lag`, `enc_slip`,
`stiction`, `trackwidth`, `scrub`) against `source/drive/` through the
now-live adapter (tickets 007/008), with `motor_lag` at 120-140ms as the
default for every tracker/replan scenario — the zero-lag path is reserved
for golden-TLM bit-exactness only.

## Acceptance Criteria

- [ ] Sim test matrix covers `motor_lag`(120-140ms)/`enc_slip`/
      `stiction`/`trackwidth`-error/`scrub`, each run against at least
      one arc and one pivot segment through the live adapter.
- [ ] `enc_slip`/scale faults are checked against `true_pose` convergence
      (NEVER `bb.fusedPose`, per the plant-model convention — grep/
      review-verifiable in the new test file).
- [ ] `stiction` is checked for terminal walk-in with no premature
      `DONE_STOP` and no reversal (a dedicated no-reversal assertion,
      mirroring ticket 005's own terminal-machine regression test, now
      through the full adapter+plant stack).
- [ ] `trackwidth` error is checked for cross-gain (`k_c`) correction of
      the resulting radius error.
- [ ] An infeasible ask under fault conditions produces a typed `ERR`
      with the queue untouched (no hang, no silent wrong answer).
- [ ] The zero-lag sim path is explicitly EXCLUDED from this matrix's
      tracker/replan scenarios — a comment/assertion in the test file
      documents why (reserved for golden-TLM bit-exactness only).
- [ ] No scenario in the matrix reproduces the 2026-07-11 false-green
      (zero-lag-only validation) failure class — an explicit note in
      completion notes cross-references that incident.
- [ ] `uv run python -m pytest` passes.

## Testing

- **Existing tests to run**: `uv run python -m pytest`.
- **New tests to write**: the fault-knob matrix itself (see Acceptance
  Criteria).
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: the sim's fault knobs already exist (`motor_lag`/
`enc_slip`/`stiction`/`trackwidth`/`scrub` on the `Sim` class) — this
ticket is pure test-writing against the now-live adapter; no production
code change is expected. If a fault-knob combination surfaces a real
defect, do NOT fix it inline here — file it and reopen the relevant
ticket (004/005/007), matching this sprint's own acceptance-tickets-
don't-silently-patch-code precedent (M11/M12).

**Files to create**: fault-matrix test file(s) under `tests/sim/system/`
or `tests/sim/unit/` (programmer's judgment on exact location, matching
`tests/CLAUDE.md`'s domain split).

**Testing plan**: the matrix itself is the testing plan.

**Documentation updates**: none.
