---
id: '013'
title: App::Drive takes motor ownership; named bus-phase methods (requestLeft/collectLeft/requestRight/collectRight)
status: open
use-cases:
- SUC-006
depends-on:
- '007'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# App::Drive takes motor ownership; named bus-phase methods (requestLeft/collectLeft/requestRight/collectRight)

## Description

**Valve line — first ticket of the deferrable tail** (sprint Design
Rationale Decision 6). Carried forward from 124's own explicit deferral.
Purely internal wiring: no wire-visible or user-visible behavior change,
which is exactly why this tail is safe to split to a 125b if the core
(tickets 001-012) ran long or bench access became unavailable again. If
you are picking this ticket up, the core is presumed done/accepted first.

`App::Drive` takes ownership of both `Devices::Motor` (moved from being
`RobotLoop`-owned back onto `Drive`). Exposes named phase methods —
`requestLeft()`/`collectLeft(nowUs)`/`requestRight()`/`collectRight(nowUs)`
— self-evidently ordered (never `tick1`/`tick2`/`tick3`), replacing
`RobotLoop`'s direct `motorL_.requestSample()`/`motorL_.tick()` calls.
Bus timing must be BYTE-FOR-BYTE unchanged — this is a pure mechanical
relocation of WHO calls the existing protocol methods, not a change to
WHEN or how. See sprint architecture Step 3 (`App::Drive`) and Use Case
SUC-006.

## Acceptance Criteria

- [ ] The four phase methods exist on `App::Drive` with those exact
      names.
- [ ] `RobotLoop` no longer calls `Devices::Motor` methods directly for
      wheel sensing (still may hold other device references until
      ticket 015 finishes the reshuffle).
- [ ] **[off-hardware]** A sim regression re-run of the existing pairing-
      skew/straight-leg-crab suite (121-005-class same-generation L/R
      telemetry) passes UNCHANGED — proves the ownership move is
      mechanical, not a behavior change.
- [ ] Bus request/settle/collect timing (measured in sim under the
      virtual clock) is identical before/after this ticket.

## Testing

- **Existing tests to run**: the full pairing-skew/straight-leg-crab
  regression suite; `app_robot_loop_harness.cpp`.
- **New tests to write**: none expected — if this ticket needs new tests
  beyond confirming the existing suite is unchanged, that's a signal the
  "pure mechanical relocation" framing was wrong.
- **Verification command**: `uv run pytest`.
