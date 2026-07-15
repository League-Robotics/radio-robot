---
id: '106'
title: 'Host trajectory planner: profiled twists, straights and turns'
status: done
branch: sprint/106-host-trajectory-planner-profiled-twists-straights-and-turns
use-cases: []
issues:
- host-planner-design-lessons-from-drive-v2-review.md
- heading-loop-output-clamp-and-velocity-resonance.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 106: Host trajectory planner: profiled twists, straights and turns

## Goals

ROADMAP-STAGE ENTRY (not yet detailed — no architecture-update.md content,
no tickets). This sprint is the first sprint AFTER the single-loop
firmware arc (103-105) that adds real capability on top of the proven
loop: sprint 103 made the robot a pure velocity/yaw follower with no
on-robot trajectory planning (stakeholder decision, sprint 102); this
sprint builds that planning HOST-side for the first time under the new
architecture:

- **Trapezoidal/jerk-limited velocity profiles**, generated host-side,
  streamed to the robot as a sequence of `twist` commands at the
  telemetry-validated cadence (spike-001's ~25 Hz target — re-confirm the
  actual achieved cadence against sprint 104's own soak-gate measurement,
  since 103-010 observed 15.62 Hz in a short capture, below that target,
  and flagged it as worth a closer look).
- **Heading feedback from streamed telemetry** for straight-tracking and
  turn accuracy — closing the loop host-side against the robot's reported
  pose (raw encoder pose this sprint; OTOS/host-fusion is 106+ scope only
  to the extent this sprint's own profiling needs it — full sensor fusion
  is not committed here).
- **The ten binding requirements from
  `clasi/issues/host-planner-design-lessons-from-drive-v2-review.md`**
  are non-negotiable design inputs, not suggestions — every one traces to
  a real defect or bench-observed failure mode from the deleted `drive/`
  v2 code, and the host planner will face every one of those failure
  modes again in this new problem domain. Detail-mode planning for this
  sprint must show each requirement addressed (a design decision or an
  explicit "not applicable, because X"), not silently dropped.
- **Taming the ~140 mm/s inner-velocity-PID resonance**
  (`clasi/issues/heading-loop-output-clamp-and-velocity-resonance.md`,
  Part 2 — Part 1's on-robot clamp is obsolete/deleted, but Part 2
  "SURVIVES and matters MORE" per that issue's own scope-reduction note:
  the rebuilt robot is a pure velocity follower, so the inner loop is now
  the ONLY loop, and its resonance will show up directly as ringing in
  exactly the acceleration/deceleration charts the stakeholder's end goal
  asks for). This is not a nice-to-have — an un-tamed resonance makes this
  sprint's own acceptance bar (clean accel/decel traces) unreachable.

## Problem

Under the pre-102 architecture, trajectory planning (segments, coast
anticipation, terminal completion) lived on-robot. Sprint 102's
stakeholder decision deleted all of that: the robot now only follows a
commanded twist for a duration and reports raw telemetry. Nothing today
can command a multi-leg path with smooth acceleration/deceleration —
sprint 103/104 only prove a single constant-velocity twist works. The
project's stated end goal ("charts... show nice acceleration and
deceleration on straights and turns") requires host-side motion profiling
that does not exist yet, built on lessons the project already paid for
once (the `drive/` v2 review's eleven findings) and against a known,
already-diagnosed resonance that will otherwise show up as visible
ringing in the very charts being asked for.

## Solution

Design deferred to this sprint's own detail-mode planning pass. Steer:

1. A host-side trapezoidal (or jerk-limited, if detail-mode planning finds
   trapezoidal insufficient for clean accel/decel charts) velocity
   profiler that decomposes a commanded straight-line distance or turn
   angle into a sequence of `twist` commands at the validated streaming
   cadence.
2. An outer heading-correction loop, host-side, closing against streamed
   telemetry (encoder pose this sprint; the achievable correction
   bandwidth over the telemetry link must be verified empirically before
   committing to a control split, per binding requirement #10).
3. Direct application of all ten binding requirements from the
   drive-review lessons issue (direction-aware completion checks, no
   silent drops, clock discipline across replans, preemption invalidating
   chain state, wire-input validation, bounded overshoot, terminal-phase
   care with no zero-dwell reversals, latency as an explicit model
   parameter, everything gain/threshold tunable live, and the heading-loop
   bandwidth verification).
4. A resonance-taming pass against the ~140 mm/s inner-loop peak
   (filter/feedforward/notch — candidates named in the issue; acceptance
   per that issue's own bar: <~10% step overshoot across the speed range
   with rise time preserved) — this may land as this sprint's OWN first
   ticket, ahead of profiling work, since clean profiles are unreachable
   without it (a scope-split call for this sprint's own detail-mode
   planning, not pre-decided here).

## Success Criteria

Notebook-quality (see sprint 107's own deliverable notebook — this
sprint's success criteria are the DATA that notebook will need, not the
notebook itself, which sprint 107 assembles) commanded-vs-measured
acceleration and deceleration traces on a straight and on a turn on the
bench rig, captured via streamed telemetry, showing no visible resonance
ringing (matching the tamed-resonance issue's own <~10% overshoot bar) and
heading correction holding a straight/turn within a stated tolerance. This
sprint's own bench-runnable proof: a scripted profiled-twist run (one
straight leg, one turn) executes on the real rig and produces a captured
telemetry trace a human can plot and judge by eye — the actual chart
production is sprint 107's, but this sprint proves the underlying motion
is clean enough to chart.

## Scope

### In Scope

- Host-side trapezoidal/jerk-limited velocity profiler.
- Host-side heading-feedback loop against streamed telemetry.
- Resonance-taming pass on the inner velocity PID (filter/feedforward/
  notch — implementation TBD by detail-mode planning).
- Application of all ten drive-review binding requirements.
- A scripted single-straight-leg and single-turn profiled run, on the
  bench rig, with captured telemetry.

### Out of Scope

- TestGUI integration and tour execution (sprint 107).
- Full host-side sensor fusion (OTOS + encoder) — this sprint uses raw
  encoder pose for heading feedback unless detail-mode planning finds a
  hard blocker that requires pulling forward a minimal OTOS contribution;
  full fusion remains 106+/undecided scope, not committed here.
- Jupyter notebook production itself (sprint 107's deliverable; this
  sprint produces the clean underlying motion/telemetry the notebook will
  visualize).

## Test Strategy

(Deferred to detail-mode planning for this sprint.)

## Architecture Notes

Depends on sprint 104 (fully realigned, soak-proven host tooling — the
profiler streams twists over the SAME command surface 104 hardens) and
benefits from, but does not strictly require, sprint 105 (a sim tier would
let profile logic be iterated without bench time, but this sprint's own
acceptance is real-hardware bench proof either way). Full
architecture-update.md is written when this sprint is detailed — it must
explicitly address each of the ten drive-review binding requirements as
part of Step 6 (Design Rationale) or Step 7 (Open Questions), not leave
any unaddressed.

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Firmware loop-cadence fix | — |
| 002 | ConfigDelta live-apply (motor gains) + inner velocity-PID resonance taming | 001 |
| 003 | Sim decay-window generalization (SimApi duty-changed scripting) | — |
| 004 | Pure trapezoidal profile generator (straight distance + in-place turn) | — |
| 005 | Streaming twist executor + heading-correction loop + binding-requirement safety | 002, 003, 004 |
| 006 | Bench gate: sim-validated then real profiled straight + turn, captured traces | 001, 002, 003, 004, 005 |

Tickets execute serially in the order listed. 001, 003, and 004 have no
dependency on each other or on any other ticket in this sprint and may be
built in parallel if a future execution pass chooses to; 002 depends on 001
(a trustworthy, internally-consistent telemetry cadence before
characterizing the inner-loop step response); 005 is the integration point
(needs tamed gains, the sim decay-window fix, and the profiler); 006 is
strictly last, matching 103/104/105's own precedent of ordering the
Definition-of-Done gate last.
