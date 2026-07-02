---
id: '066'
title: 'Sim fidelity and host cleanups: sim-OTOS ground truth and lever arm, landmine
  batch, CR-15 batch, TestGUI trace correctness'
status: done
branch: sprint/066-sim-fidelity-and-host-cleanups-sim-otos-ground-truth-and-lever-arm-landmine-batch-cr-15-batch-testgui-trace-correctness
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
- SUC-007
- SUC-008
- SUC-009
issues:
- sim-otos-fidelity-ground-truth-and-lever-arm.md
- landmine-cleanups-planner-apply-now0-sim-abi-buffers.md
- small-cleanups-from-2026-07-01-review.md
- testgui-trace-correctness-slow-tlm-and-anchor-rotation.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 066: Sim fidelity and host cleanups: sim-OTOS ground truth and lever arm, landmine batch, CR-15 batch, TestGUI trace correctness

## Goals

Close out the remaining review findings: make the simulator's OTOS a real
independent ground-truth sensor (with lever arm) so OTOS bug classes are
testable in sim; defuse four medium landmines; land the CR-15 small-cleanup
batch; and fix the two TestGUI trace-correctness defects.

## Problem

- CR-07/08: sim OTOS re-integrates commanded wheel speeds (can never disagree
  with encoders) and models no lever arm — the two most painful hardware OTOS
  bug classes have zero sim coverage.
- CR-11..14: `Planner::apply` hard-codes now=0 (instant TIME-stop landmine),
  OdomTracker's world transform is an untested convention stack, the sim
  C-ABI global clock breaks on double SimHandle, SimConnection buffer growth.
- CR-15: eight small independent cleanups (heading wrap, retired probe path,
  relay_info surfacing, SimTransport connect ordering, traces midpoint
  integration, stop-slot verify, rgbToHSV placement, KeyboardDriver
  multi-key).
- CR-09/10: TestGUI encoder-reset heuristic misses resets on slow TLM
  (re-breaking the "encoder track ignores turns" fix on the relay), and
  otos/fused traces are rotated when anchored mid-session.

## Solution

- SimOdometer samples plant ground truth (+ noise/quantization) and models
  the sensor at `odomOffX/Y`; slip then produces genuine encoder/OTOS
  disagreement in sim.
- Thread a real timestamp through `Planner::apply` (or baseline-on-first-
  tick); add the OdomTracker convention test or retire the class; scope the
  sim clock per-handle; bound the SimConnection buffer.
- CR-15 items as one maintenance ticket.
- Traces: rebaseline on command boundaries (GUI knows when it sends D/ZERO)
  and rotate otos/fused deltas by (anchor_yaw − firmware_heading_at_baseline).

## Success Criteria

- Sim tests demonstrate encoder/OTOS disagreement under slip and lever-arm
  compensation coverage (a db11b7c-style regression now fails in sim).
- Landmine tests: PlannerCommand TIME stop uses real now; double-SimHandle
  clock isolation; convention test green.
- TestGUI: slow-TLM reset scenario integrates correctly (no spurious reverse
  motion); mid-session anchor leaves traces aligned.
- Full default suite + testgui tier green; ARM firmware builds clean.

## Scope

### In Scope

`source/hal/sim/SimOdometer.*`, `SimHardware.*`, `PhysicsWorld.*`,
`tests/_infra/sim/sim_api.cpp`, `source/superstructure/Planner.cpp`,
`host/robot_radio/sensors/odom_tracker.py`, `host/robot_radio/io/
serial_conn.py`, `host/robot_radio/testgui/{traces.py,transport.py,drive.py}`,
`source/control/StopCondition.cpp` (rgbToHSV move), matching tests.

### Out of Scope

Encoder pipeline (064), stop/watchdog/fusion-gate (065), hardware validation.

## Test Strategy

Sim-tier and testgui-tier pytest. New sim fidelity tests are the core
deliverable of the OTOS work. Full default suite green before close.

## Architecture Notes

- SimOdometer's contract changes from "integrator" to "ground-truth sampler
  with sensor pose"; document in architecture-update. Existing tests that
  relied on OTOS==encoders may need updating — that agreement was the bug.
- KeyboardDriver multi-key (CR-15.8) touches the same file as 065's STOP
  work; 066 runs after 065 so rebase conflicts are resolved here.

## GitHub Issues

(none)

## Definition of Ready

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan (auto-approve session)

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Sim OTOS ground-truth sampling and lever-arm compensation | (none) |
| 002 | Landmine defusal batch: Planner::apply timestamp, OdomTracker convention test, sim-ABI clock isolation, SimConnection buffer | (none) |
| 003 | CR-15 maintenance batch: eight small independent cleanups | 001 |
| 004 | TestGUI trace correctness: command-boundary rebaseline and anchor-heading rotation | 003 |

Tickets execute serially in the order listed. 003 depends on 001 (item 1,
`PhysicsWorld._truePoseH` wrapping, is verify-only in 003 because it is
resolved by 001). 004 depends on 003 (both edit
`TraceModel._feed_encoder()` in `traces.py`; 003's midpoint-integration fix
should land before 004 rewrites the reset-detection logic in the same
function).
