---
id: '005'
title: Move the unified controller into Drive for every writer; wire-key repoint +
  observability
status: open
use-cases: [SUC-001]
depends-on: ['004']
github-issue: ''
issue:
- wheel-speed-controller-moves-into-drive.md
- 04-continuous-duty-per-speed-calibration.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Move the unified controller into Drive for every writer; wire-key repoint + observability

## Description

Per `wheel-speed-controller-moves-into-drive.md` Phase 3 (DECIDED,
stakeholder 2026-08-01): make the Stage A/B/C controller from ticket
004 the actuation path for EVERY `cmdVelocity` writer — WHEELS teleop
and planner Moves alike. `Motion::Planner` sheds `stageTrim()`/
`WheelTrim` entirely (deleted outright, not redirect-stubbed — sprint
Architecture Design Rationale Decision 2: the code moves with the
responsibility, `src/firm` still imports nothing from `src/motion` for
this feature). Repoint `pid.*` CONFIG wire keys from `applyVelGains()`
(the parked duty stage) to the new controller's gains, closing the
silent no-op. Ship observability with the feature (folded from issue
04's mandate: "ships with the feature, not after it") — live
`dutyPerSpeedLeft/Right`, `bias`, and both trim integrals in telemetry
and the TestGUI.

## Acceptance Criteria

- [ ] `Motion::WheelTrim` and its planner-side ownership (`stageTrim()`,
      trim state) deleted from `src/motion` — no redirect stub, no
      future caller (git preserves history).
- [ ] A WHEELS teleop command and a planner Move both reach the SAME
      `Drive` controller path — verified by a test asserting identical
      Drive behavior for the same commanded velocity regardless of
      which subsystem issued it.
- [ ] `pid.*` CONFIG wire keys route to the new controller's live gains
      (`kp`/`ki`/`iMax`/`kaff`/`pidMax`) — `SET pid.kp` visibly tunes
      the controller or returns an explicit error; no silent no-op
      remains anywhere on the wire surface.
- [ ] Live `dutyPerSpeedLeft/Right`, `bias` (both wheels), and the PID
      integrator are published in telemetry and surfaced in the
      TestGUI.
- [ ] `drive.h`'s header comment is corrected: "there is no controller
      here" and "closed-loop control lives in Motion::Planner's own duty
      stage" (already false — the duty stage is parked) are both
      replaced with an accurate description of the unified controller.

## Testing

- **Existing tests to run**: full `app_drive`/`planner` unit and sim
  suites; wire-protocol CONFIG tests for `pid.*` keys.
- **New tests to write**: a WHEELS-vs-Move actuation-parity test; a
  CONFIG round-trip test confirming `pid.*` reaches the controller's
  live gains; a telemetry/TestGUI test confirming `bias`/`dutyPerSpeed`
  values are visible.
- **Verification command**: `uv run pytest`

## Implementation Plan

**Approach**: this is the "move" ticket — ticket 004 already built the
algorithm; this ticket relocates ownership so every caller reaches it,
and finishes the observability/wire-key work issue 04 called for.

**Files to create/modify**:
- `src/firm/app/drive.{h,cpp}` (own the full controller + WHEELS +
  Move actuation paths)
- `src/motion/planner/planner.{h,cpp}` (remove `stageTrim()`/`WheelTrim`
  ownership)
- `src/motion/planner/wheel_trim.{h,cpp}` (deleted)
- `src/firm/app/configurator.cpp` (`pid.*` repoint)
- Telemetry assembly (`App::Telemetry`) + TestGUI panel (live
  `dutyPerSpeed`/`bias`/trim observability)

**Testing plan**: WHEELS-vs-Move parity test; CONFIG round-trip test;
telemetry/TestGUI observability check. Bench A/B acceptance is ticket
006's job, not this ticket's.

**Documentation updates**: `src/motion/DESIGN.md`'s "wheel control
generations" note updated (the `WheelTrim` generation retired here,
ahead of ticket 007's duty-stage deletion); `drive.h` header comment
rewrite (above).
