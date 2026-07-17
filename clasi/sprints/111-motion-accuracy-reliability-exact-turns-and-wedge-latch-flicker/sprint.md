---
id: '111'
title: 'Motion accuracy & reliability: exact turns and wedge-latch flicker'
status: roadmap
branch: sprint/111-motion-accuracy-reliability-exact-turns-and-wedge-latch-flicker
worktree: false
use-cases: []
issues:
- turn-lead-compensation-gain-cotuning.md
- wedge-latch-flickers-during-active-motion.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 111: Motion accuracy & reliability: exact turns and wedge-latch flicker

## Goals

Deliver exact turns and stop healthy motion from being spuriously
fault-stopped. This is the highest functional priority of the four
roadmap sprints planned this round: both member issues directly gate
reliable bench tour closure.

(a) Co-tune the heading-loop PD gains jointly with the three lead loci
(`heading_lead_bias` / `plan_lead` / `terminal_lead`) — a raw lead at
`heading_kp = 6` regresses tours (109-010 found no combination that both
avoids regression and beats the ticket-009 baseline). Also consider
whether an architectural fix to the cycle ordering (sample OTOS before
`Pilot::tick()` reads heading — a `robot_loop.cpp` change with bus-timing
implications) beats compensating for staleness in software.

(b) Root-cause and/or debounce `kFaultWedgeLatch` (bit `0x2`) flickering
DURING active continuous motion (not just at idle/direction-reversal
boundaries), which occasionally lands as a "new" bit relative to
`StreamingExecutor`'s baseline and fault-stops an otherwise-healthy run.

Sim characterizes turn error (rate-sweep harness, fitted error model) but
cannot validate absolute turn accuracy — bench re-tune on the real robot
is required for (a). Detail-planning for this sprint will decide ticket
split and whether the `robot_loop.cpp` cycle-ordering change is in scope
alongside the gain co-tune, or deferred.

## Scope

### In Scope

- Joint co-tuning of heading PD gains (kp, kd) with the three
  `PlannerConfig` lead loci for exact turns under an ideal OTOS.
- Investigation and fix (debounce and/or root cause) for `kFaultWedgeLatch`
  flickering during active continuous motion.
- Persisted `PlannerConfig` defaults (per-robot JSON) for whatever gain/lead
  values the co-tune settles on; bench re-tune before trusting hardware
  turns.

### Out of Scope

- Any comms/device robustness work (ack-ring, relay handshake, device
  re-probe) — sprint 112.
- Host P4 mid-layer rewrite (Nezha facade, nav, calibrate) — sprint 113.
- Repo hygiene / naming sweep / comment audit / vendor symlink — sprint 114.

## Acceptance Sketch (at-a-glance)

- Ideal-OTOS turns exact to plant epsilon in sim; error-vs-rate slope ≈ 0
  post-compensation.
- TOUR_2 leg-14 outlier (4.9°, tied to the same latency mechanism) collapses
  under the realistic profile.
- Tours remain 100% reliable; full test suite green.
- `kFaultWedgeLatch` no longer fault-stops a healthy, continuously-moving
  `StreamingExecutor` run on a transient single-frame flicker.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [ ] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
