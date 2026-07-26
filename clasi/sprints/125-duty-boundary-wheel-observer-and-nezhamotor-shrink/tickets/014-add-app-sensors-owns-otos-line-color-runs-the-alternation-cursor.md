---
id: '014'
title: 'Add App::Sensors: owns OTOS/line/color, runs the alternation cursor'
status: open
use-cases:
- SUC-006
depends-on:
- '013'
github-issue: ''
issue: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add App::Sensors: owns OTOS/line/color, runs the alternation cursor

## Description

**Valve-line tail** (Design Rationale Decision 6). New `App::Sensors`
(`src/firm/app/sensors.{h,cpp}`): owns the OTOS leaf and the line/color
leaves, runs the alternation cursor internally (moved verbatim from
`RobotLoop`'s own inline logic — exactly one of {line, color} ticks per
call, alternating which on the next call, UNCHANGED cadence). One
`update(RobotState&, nowUs)` entry point publishes the otos/perception
sections. See sprint architecture Step 3 (`App::Sensors`) and Use Case
SUC-006.

## Acceptance Criteria

- [ ] `App::Sensors::update()` exists and internally alternates line/
      color at the same cadence as today (unit-tested, not just eyeballed).
- [ ] `RobotLoop` no longer directly ticks `Devices::Otos`/line/color
      leaves.
- [ ] **[off-hardware]** A sim regression re-run of the existing
      pairing-skew/straight-leg-crab suite passes UNCHANGED.
- [ ] `App::Sensors`'s own alternation-cursor unit test matches the
      pre-move cadence exactly (e.g. line/color fresh flags alternate on
      the expected schedule across N calls).

## Testing

- **Existing tests to run**: whatever currently exercises the line/color
  alternation cursor inline in `RobotLoop`; the pairing-skew/straight-
  leg-crab suite.
- **New tests to write**: `App::Sensors`'s own alternation-cadence unit
  test (ported from wherever it currently lives, if anywhere, or newly
  authored if the cadence was previously only implicitly tested).
- **Verification command**: `uv run pytest`.
