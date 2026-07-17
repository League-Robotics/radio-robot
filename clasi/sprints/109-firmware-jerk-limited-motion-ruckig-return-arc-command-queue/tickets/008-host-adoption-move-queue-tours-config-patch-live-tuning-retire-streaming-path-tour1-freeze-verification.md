---
id: 008
title: 'Host adoption: MOVE-queue tours, config-patch live tuning, retire streaming
  path, tour1-freeze verification'
status: in-progress
use-cases:
- SUC-003
- SUC-005
depends-on:
- '007'
github-issue: ''
issue: tour1-freeze-investigation-2026-07-15.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Host adoption: MOVE-queue tours, config-patch live tuning, retire streaming path, tour1-freeze verification

## Description

The last host-side ticket before the decisive sim gate (ticket 009): move
TestGUI/tours off the streamed-twist path (host planner streaming plain
trapezoid twists at ~6.7 Hz) and onto the new `MOVE` command queue. This
is also where the host planner is demoted to teleop input shaping only —
DISTANCE/pivot planning now lives entirely in firmware (tickets 003-006).

1. TestGUI/tour runner sends `MOVE` command sequences (one per tour leg)
   instead of streamed twists. `host/robot_radio/planner/tour.py`
   translates each leg into a `Move` (distance + delta_heading + v_max +
   time=0) and enqueues via the wire, relying on firmware's own queue
   (ticket 003) and boundary-velocity carry (ticket 006) instead of host-
   side leg sequencing/settling.
2. Host planner demoted: `host/robot_radio/planner/` keeps only the
   teleop input-shaping role (gamepad → TIMED `Move` commands via ticket
   003's teleop path) — remove or clearly mark dead the DISTANCE/pivot
   planning logic that MOVE now replaces.
3. Retire the dead host streaming path for tours specifically (leave the
   teleop streaming path alone — it's still needed for gamepad input
   shaping, per ticket 003's TIMED mode).
4. Un-stub live `PlannerConfig` gain patches for runtime tuning (the issue
   notes these were previously stubbed) — this is what lets
   `heading_kp`/`heading_kd` (ticket 005) be tuned from the TestGUI
   without a reflash, matching the existing "Velocity + heading gains
   live-tunable on stand" precedent from sprint 098 (per project memory).
   Note (2026-07-17, Architecture Revision 1 on ticket 002): construct
   and send `PlannerConfigPatch` directly via the same direct-patch-send
   mechanism tickets 002/004 establish and reuse — never via
   `binary_bridge.translate_command()`, which is dead and is not being
   resurrected this sprint (see sprint.md's Architecture Revision 1).
5. Verify `tour1-freeze-investigation-2026-07-15.md`'s freeze symptom
   cannot recur on the new MOVE-queue path: that investigation's verdict
   was a real `kFaultWedgeLatch` firmware fault at the straight->turn
   boundary + a too-short `DEFAULT_INTER_LEG_SETTLE` (already fixed to
   1.0s in `tour.py`), not a Qt deadlock. On the new path, leg-to-leg
   transitions are firmware-driven (boundary-velocity carry, ticket 006)
   rather than host-timed settle delays — confirm the wedge-latch
   condition specifically (not just "it didn't visibly freeze") is
   exercised and does not reproduce under the new command model, then
   close the issue with this verdict recorded.

## Acceptance Criteria

- [ ] TestGUI/tour runner sends `MOVE` sequences for Tour 1 and Tour 2
      instead of streamed twists.
- [ ] `host/robot_radio/planner/`'s DISTANCE/pivot planning logic is
      removed or clearly marked dead (superseded by firmware planning);
      teleop input-shaping role is retained and unchanged.
- [ ] Dead host streaming path for tours is retired (not just
      unreferenced — actually removed, per the project's greenfield-
      rebuild preference of parking/removing rather than leaving dead
      code live).
- [ ] Live `PlannerConfig` gain patches (including `heading_kp`/
      `heading_kd`) are un-stubbed and settable from the TestGUI without
      a reflash.
- [ ] `tour1-freeze-investigation-2026-07-15.md`'s specific failure mode
      (`kFaultWedgeLatch` at a straight->turn boundary) is exercised
      against the new MOVE-queue path and does not reproduce; the issue
      is moved to done with this verdict recorded (or, if it DOES
      reproduce, this ticket does not close the issue and instead records
      the reproduction for the team-lead/stakeholder to triage before
      ticket 009 proceeds).
- [ ] Bench: TestGUI → hardware → Tour 1 runs via the new MOVE-queue path
      without the wedge-latch/freeze symptom (stretch goal per sprint.md,
      not a blocker — the decisive gate is Sim, ticket 009).

## Testing

- **Existing tests to run**: existing TestGUI tour-runner tests (must be
  updated to reflect MOVE-based sending, not deleted wholesale); existing
  `PlannerConfig` patch tests.
- **New tests to write**: MOVE-sequence-per-leg encoding test for Tour 1/
  Tour 2; live gain-patch round-trip test (`heading_kp` set via TestGUI,
  confirmed applied); a targeted wedge-latch-at-boundary reproduction
  test against the new path (sim or bench) per the investigation's own
  "what would pin it down" section.
- **Verification command**: `uv run python -m pytest tests/testgui/ -k
  "tour or planner_config"`.

## Implementation Plan

**Approach**: This is the host-side "flip the switch" ticket — by this
point in the sprint, firmware fully supports MOVE-queue tours (tickets
003-006) and the sim can validate it faithfully (ticket 007); this ticket
just needs the host to actually use the new path instead of the old one,
and to clean up what it replaces rather than leaving both live.

**Files to modify**:
- `host/robot_radio/planner/tour.py` (MOVE-sequence sending)
- `host/robot_radio/planner/` (demote/remove DISTANCE/pivot planning)
- `testgui/` (PlannerConfig gain-patch UI un-stub, if a UI stub currently
  exists)
- `clasi/issues/tour1-freeze-investigation-2026-07-15.md` (close with
  verdict, or leave open with reproduction notes if it recurs)

**Testing plan**: as above.

**Documentation updates**: none required in `src/firm/` (host-only
ticket); update any host-side planner/tour documentation that describes
the now-retired streaming path.
