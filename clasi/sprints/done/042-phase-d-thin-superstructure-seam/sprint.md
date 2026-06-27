---
id: '042'
title: "Phase D \u2014 Thin Superstructure seam"
status: done
branch: sprint/042-phase-d-thin-superstructure-seam
use-cases: []
issues:
- migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 042: Phase D — Thin Superstructure seam

## Goals

Introduce a thin `Superstructure` seam (§4): a `Goal` enum, a guarded `requestGoal`
entry point that verb handlers route through, centralized keepalive/SAFE/ESTOP logic, and
a pre-cut `goalAllowed()` world-bounds hook. `MotionController` moves under
`source/superstructure/`. Foundation tier only — a `switch`-over-`Goal`; no state-graph.
Bodies moved verbatim; no behavior changes.

Depends on: Sprint 041 (Phase C) — `PhysicalStateEstimate` seam must exist before
`Superstructure` can reference the fused belief cleanly.

## Problem

Today, keepalive-watchdog decisions, SAFE re-arm, and ESTOP/X paths are scattered across
`loopTickOnce`. Verb handlers call `motionController.beginX()` directly — there is no
single guarded entry point for goal transitions. The off-table fence and world-bounds
logic, when added later, will need to touch multiple call sites. There is no canonical
place for "one guarded transition."

## Solution

Following §4 of the issue:

- Create `source/superstructure/Superstructure.{h,cpp}`:
  - `enum class Goal { IDLE, STREAM, TIMED, DISTANCE, GOTO, TURN, ROTATE, VELOCITY,
    ARC, ESTOP }`.
  - `requestGoal(GoalRequest)` — the single transition function verb handlers route
    through (replacing direct `motionController.beginX()` calls).
  - Pre-cut `goalAllowed()` hook — no new behavior now, just the seam, so the off-table
    fence can later live in one place.
- Move `MotionController.{h,cpp}` under `source/superstructure/`.
- Centralize in `Superstructure`: the keepalive-needs-watchdog decision, the SAFE one-
  shot re-arm, and the ESTOP/X path (driven by `HaltController`). These currently live
  spread across `loopTickOnce`; move bodies verbatim.
- `loopTickOnce` calls `superstructure.periodic()` rather than individual controllers.
- Foundation tier only: a `switch`-over-`Goal`. Do NOT build a state-graph/transition-
  table (D2 L3–4 of the spec).

## Key Deliverables

- `source/superstructure/Superstructure.{h,cpp}` with `Goal` enum + `requestGoal`.
- `source/superstructure/MotionController.{h,cpp}` (moved; bodies verbatim).
- `source/superstructure/HaltController.{h,cpp}` (moved or referenced from here).
- `goalAllowed()` hook stub in place (returns `true` for now — no fence logic yet).
- Keepalive/SAFE/ESTOP centralized in `Superstructure`; `loopTickOnce` simplified.
- All verb handlers route goal requests through `requestGoal` (not direct `beginX()`).
- `test_watchdog_exemption.py`, `test_goto_bounds.py`, `test_incident_scenarios.py`,
  behavior fences all still green.

## Scope

### In Scope

- `Superstructure` class with `Goal` enum + `requestGoal` transition function.
- `MotionController` move to `source/superstructure/`.
- `HaltController` move / reference from `source/superstructure/`.
- Keepalive/SAFE/ESTOP centralization (bodies verbatim from `loopTickOnce`).
- `goalAllowed()` stub.
- Verb handler repoint from `beginX()` to `requestGoal`.
- `loopTickOnce` simplification.

### Out of Scope

- State-graph or transition-table (D2 L3–4 — explicitly deferred by the issue).
- Off-table fence logic (just the seam is pre-cut; implementation deferred).
- Subsystem wrapping with `periodic()` / `updateInputs()` (Phase E).
- TLM reader repoint / cleanup (Phase F).
- Any new behavior (safety improvements, EKF tuning, etc.) — structural seam only.

## Architecture Notes

- Honestly thin (diff-drive + one optional gripper, no mechanism-vs-mechanism interlock).
  The value of this seam is one guarded entry point + consolidated safety, not a complex
  state machine.
- `goalAllowed()` is pre-cut only: `return true;` until the off-table fence is
  implemented in a later (post-migration) sprint.
- `loopTickOnce` stays as the shared firmware↔sim periodic orchestrator; this phase
  simplifies it but does not restructure it further.
- Moving bodies verbatim ensures behavior is unchanged; canaries catch any regression.

## Definition of Done (Phase D — from issue §4 / Migration sequence)

- [ ] `source/superstructure/Superstructure.{h,cpp}` exists with `Goal` enum +
      `requestGoal(GoalRequest)`.
- [ ] All verb handlers route goal requests through `requestGoal` (no direct `beginX()`
      calls from outside `Superstructure`).
- [ ] `goalAllowed()` stub in place (returns `true`).
- [ ] Keepalive/SAFE/ESTOP logic centralized in `Superstructure`; removed from
      `loopTickOnce` scatter sites.
- [ ] `MotionController` at `source/superstructure/MotionController.{h,cpp}`.
- [ ] `test_watchdog_exemption.py`, `test_goto_bounds.py`, `test_incident_scenarios.py`
      and all behavior-preservation fences still green.
- [ ] Simulation tier green (≥ 1954 tests): `uv run --with pytest python -m pytest -q`
- [ ] `defaultRobotConfig()` field-pin diff empty.
- [ ] Golden-TLM frame canary unchanged.
- [ ] Vendor-confinement grep gate passes (Phase D scope).
- [ ] No new heap allocation or fibers introduced.
- [ ] No state-graph / transition-table added (out of scope by design).

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 042-001 | Create Superstructure skeleton with Goal enum and requestGoal routing to existing MotionController.beginX() | — |
| 042-002 | Move MotionController under source/superstructure/ and fix include paths and build system | 042-001 |
| 042-003 | Centralize keepalive/SAFE/ESTOP safety into Superstructure.evaluateSafety() and pre-cut goalAllowed() hook | 042-001, 042-002 |

Tickets execute serially in the order listed.
