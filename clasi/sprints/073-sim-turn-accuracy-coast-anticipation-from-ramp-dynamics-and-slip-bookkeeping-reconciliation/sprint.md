---
id: '073'
title: 'Sim turn accuracy: coast anticipation from ramp dynamics and slip bookkeeping
  reconciliation'
status: planning-docs
branch: sprint/073-sim-turn-accuracy-coast-anticipation-from-ramp-dynamics-and-slip-bookkeeping-reconciliation
use-cases: []
issues:
- sim-turn-undershoot.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 073: Sim turn accuracy: coast anticipation from ramp dynamics and slip bookkeeping reconciliation

## Goals

(Describe what this sprint aims to accomplish.)

## Problem

(What problem does this sprint address?)

## Solution

(High-level description of the approach.)

## Success Criteria

A fresh, ZERO-configuration `Sim()` (default `RobotConfig`, no injected sim
error) issuing a bare `RT <cdeg>` lands within ~1° of the commanded angle,
measured from plant ground truth (`sim.get_true_pose()`), across a
45°–300° sweep — the combined effect of Ticket 001 (coast anticipation
derived live from ramp-down dynamics, replacing the stale
`kRtCoastArc = 8.0mm` constant) and Ticket 002 (the sim plant's body-
rotational scrub seeded from `RobotConfig.rotationalSlip` at construction,
closing the "clean sim over-rotates" gap). No wire-protocol grammar
change; no `RobotConfig`/`SIMSET` field added or renamed. Confirmed,
final: `tests/simulation/system/test_073_rt_angle_sweep.py`'s 4-angle
sweep passes at a documented `_TOL_DEG = 1.25°` bound (measured worst
case 45° → +1.10°; see Regression Sweep Results below). See
`architecture-update.md`'s Sprint Changes Summary for the full five-item
breakdown and Step 6 for the design rationale behind each fix.

## Scope

### In Scope

(List what is included in this sprint.)

### Out of Scope

(List what is explicitly excluded.)

## Test Strategy

Sim-only (Ticket 001's coast-anticipation fix has real-hardware impact,
but HIL validation is explicitly deferred to a follow-up — see
`architecture-update.md` Open Questions). Ticket 001 updates
`tests/simulation/unit/test_rt_slip.py`'s hardcoded coast-arc constant to
derive from the new ramp-dynamics formula instead. Ticket 002 adds a
direct `PhysicsWorld::setSlip()` decoupling unit test and confirms three
existing `PhysicsWorld`/`sim_set_motor_slip` callers are unaffected
(`test_sim_otos_lever_arm.py`, `test_physics_world_basic.py`,
`test_physics_world_body_scrub.py`). Ticket 003 updates the TestGUI
default-profile tests (`test_sim_prefs.py`, `test_transport.py`,
`test_070_004_sim_errors_from_cal.py`) for the reconciled
`DEFAULT_PROFILE` values. Ticket 004 (this ticket) is the acceptance
vehicle: a new angle-sweep regression test
(`test_073_rt_angle_sweep.py`) proving Tickets 001+002 combine correctly
end to end, a rewrite of `test_069_rt_90deg_body_scrub.py`'s identity
test (whose "default is a no-op" premise Ticket 002 deliberately
invalidated), and a contingent xfail-removal attempt on the
pre-existing, independently-root-caused
`tests/testgui/test_tour1_geometry.py` Tour-1 GUI test.

### Regression Sweep Results (Ticket 004)

**Full-suite counts** (`uv run python -m pytest`, the `tests/simulation/`
CI gate). Pre-sprint baseline (`architecture-update.md` Step 1): **2655
passed, 0 failed**. Entering ticket 004 (post 001–003, confirmed by
ticket 003's own final run and reproduced by direct observation):
**2667 passed, 1 failed** (`test_069_rt_90deg_body_scrub.py::test_rt_90deg_identity_no_scrub`,
owned by this ticket). Final, post-004, confirmed on two consecutive
runs: **2672 passed, 0 failed** — delta +1 (identity test fixed) +4 (new
sweep tests), exact arithmetic match, no unexplained regressions. No
`--clean` sim rebuild was required (no C++/sim source changed by this
ticket).

**Deliberately-updated tests, before/after:**

| Test | Before | After |
|---|---|---|
| `test_rt_slip.py`'s coast constant | Hardcoded `kRtCoastArcMm = 8.0` | `_coast_mm()` helper derives the value live from `rate²/(2·yawAccMax)·(π/180)·(tw/2)` (ticket 001; already reconciled, no residual found, no ticket-004 edit needed) |
| `test_069_rt_90deg_body_scrub.py::test_rt_90deg_identity_no_scrub` | `body_rot_scrub=None` ("PhysicsWorld's neutral default is a no-op") → failed post-002 at 83.50° (6.50° miss) | `body_rot_scrub=1.0` explicit (setter's neutral value; preserves the identity intent) → passes at ~90° |
| `test_sim_prefs.py` / `test_transport.py` / `test_070_004_sim_errors_from_cal.py` | `slip_turn_extra: 0.26`; `body_rot_scrub` hardcoded `1.0` | `slip_turn_extra: 0.0`; `body_rot_scrub` resolved dynamically from the active robot's `rotational_slip` (ticket 003) |
| `tests/testgui/test_tour1_geometry.py::test_tour1_traces_the_tour_at_zero_error` | `xfail(strict=True)` | **Unchanged — still XFAILs.** Root cause: this test's own `_ZERO_ERROR_SPINS["sim_err_body_rot_scrub"] = 1.0` overrides ticket 002's calibration-seeded plant scrub back to neutral via an explicit `SIMSET`, while the test's baked config still pushes `rotational_slip=0.92` to firmware on Connect — reproducing the original "baked-0.92-exposed" over-rotation (+5.11°/leg measured vs. a clean `Sim()`'s +1.10°/leg), not a residual of tickets 001/002. See ticket 004's Implementation Notes for the full root-cause probe and a flagged (not actioned) follow-up. |

**New test.** `tests/simulation/system/test_073_rt_angle_sweep.py` — 4
parametrized cases (45°/90°/180°/300°), headline acceptance for the
sprint. Measured misses: 45°→+1.10°, 90°→+1.01°, 180°→+0.59°,
300°→+0.93°; documented bound `_TOL_DEG = 1.25`°.

## Architecture Notes

(Key design decisions and constraints.)

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | RT coast anticipation from ramp dynamics | none |
| 002 | Sim plant scrub reconciliation | none |
| 003 | TestGUI default-profile reconciliation | 002 |
| 004 | Regression sweep + Tour-1 xfail removal | 001, 002, 003 |

Tickets execute serially in the order listed.
