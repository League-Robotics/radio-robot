---
status: pending
filed: 2026-07-24
filed_by: team-lead (sprint 121 close disposition, stakeholder-directed autonomous run)
related:
- replan-sprints-122-plus-to-close-goal-exact-tours.md
- tour-1-final-leg-completes-only-on-stop.md
- land-at-zero-at-orthogonal-chain-boundaries.md
tickets: []
---

# Bench-verify: 121-002 tour-1 ack-ring fix + the post-122 land-at-zero firmware, on the stand

## Why this exists

Sprint 121 was closed (stakeholder-directed autonomous run, 2026-07-24) with
its two firmware/host tickets fully Sim-verified but their **on-stand bench
halves deferred** — captured here so they are not lost. The bench gate is a
hard project rule (`.claude/rules/hardware-bench-testing.md`); this issue is
the explicit deferral record, to be run in the first hardware session the robot
is on the stand.

## What to verify on the stand

1. **121-002 (ack-ring completion fix, `planner/tour.py`)** — NOT superseded by
   any later sprint; verify directly:
   - Run a full **TOUR_1 over the real serial link** (`/dev/cu.usbmodem2121102`)
     and confirm it **retires its final leg and reports closure WITHOUT a STOP
     press** (the exact bug from `tour-1-final-leg-completes-only-on-stop.md`).
   - Capture the telemetry frames/acks around the final leg's completion: a
     frame should carry `ack_corr == <final Move.id>` in its `acks` ring with
     `kFlagActive` dropping, retiring the leg with no STOP. This closes the two
     bench-half acceptance boxes (1 & 3) left open on ticket 121-002.

2. **Land-at-zero standing bench gate** — DEFERRED to **after sprint 122**, not
   run against 121-003's firmware. 121-003's orthogonal margin machinery
   (`kStoppingMarginFactorOrthogonal = 0.67`) is a swept interim that sprint 122
   **deletes** (per `replan-sprints-122-plus-to-close-goal-exact-tours.md`);
   bench-verifying it would be moot. Run the standing verification gate
   (sensors alive, wheels drive both directions with encoders incrementing, a
   managed turn→straight sequence observed over the real link) against the
   **post-122** firmware instead, folding into sprint 122's / the bench
   campaign's own standing gate.

## Disposition of the 121 close (for the record)

- 121-001 (encpose): fully done, no bench dependency.
- 121-002: code + Sim complete and committed (0e06cf79); item 1 above is its
  only open verification.
- 121-003: structural orthogonal/same-axis boundary split + the definitive
  finding that **margin tuning cannot close S1 accuracy** (root cause: real
  plant post-reset momentum decay; true fix = analytic
  `|omega_measured|·(kCycle/2 + tauPlant)`). Accuracy targets NOT achieved in
  121; **re-scoped to sprint 122** per the replan. Committed 81fa7858.
