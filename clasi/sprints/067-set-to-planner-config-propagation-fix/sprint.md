---
id: '067'
title: SET-to-Planner config propagation fix
status: roadmap
branch: sprint/067-set-to-planner-config-propagation-fix
use-cases: []
issues:
- set-config-not-propagated-to-planner.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 067: SET-to-Planner config propagation fix

## Goals

Make a runtime `SET` of a plain config key (`rotSlip`, `tw`, and peers)
actually reach the subsystem that consumes it, so per-robot calibration takes
effect without a recompile. This is a foundational correctness fix: it
unblocks the sim-to-hardware fitting workflow in sprint 069 (fitted parameters
must actually take effect when pushed to the robot) and finally makes turn-slip
recalibration meaningful.

## Problem

`SET rotSlip=1.0` replies `OK` and `GET rotSlip` reads back `1.0`, but turn
behavior is bit-identical to the default (verified in sim 2026-07-02: RT
4500/9000 produce the same true rotation at rotSlip 0.92 and 1.0).

The config registry only pushes a `configure()` delta into a subsystem for
entries that carry a subsystem annotation. Plain `CFG_F` entries have
`subsystem = nullptr`, and the Planner holds a **boot-time private copy** of
`RobotConfig` (`RobotConfig _cfg`) that is never re-invoked for these keys. The
struct is updated, the reply says OK, and the consumer never sees the new
value. The per-robot `rotational_slip: 0.92` in `data/robots/tovez.json` only
"works" because it coincidentally equals the compiled-in default — recalibrating
it (camera evidence says real playfield scrub is ≈0, not 8%) would silently
change nothing.

## Solution

- Audit **all** plain `CFG_F`/`CFG_I`/`CFG_FI` registry entries and determine,
  for each, whether any consumer caches a config copy (Planner is the known
  offender; check trackwidth `tw`, rotation gains/offsets, odom offsets, EKF
  noise keys, etc.).
- Fix by either annotating each key with its owning subsystem(s) so the
  existing post-commit `configure()` push fires, or converting the Planner to
  read the live `RobotConfig` by reference (`const RobotConfig& _cfg`) the way
  Superstructure already does.
- Add a recurrence guard: a regression test that `SET`s every registered
  motion-critical key and asserts the owning consumer observes the new value.

## Success Criteria

- `SET rotSlip=<x>` measurably changes RT arc targets on the next turn (sim
  test: RT 9000 true rotation differs between rotSlip 0.92 and 1.0).
- Audit results recorded; every stale-copy key is either annotated or its
  consumer converted to live-reference reads.
- Regression test covering `SET`→consumer propagation for motion-critical keys.

## Scope

### In Scope

- Firmware `ConfigRegistry` / Planner (and any other caching consumer) audit
  and fix.
- SET→consumer propagation regression test.

### Out of Scope

- Adding new sim error knobs or the wire-settable sim param surface (sprint 069).
- Encoder-only pose / `encpose=` telemetry (sprint 068).

## Test Strategy

Sim-level regression: SET each motion-critical key, then exercise the consuming
motion (e.g. RT turn) and assert observable behavior changes. Add a
propagation-coverage test that sweeps every registered key against its consumer.

## Architecture Notes

Two candidate fixes (subsystem annotation vs. live-reference read) — the
sprint-planner/architecture pass will choose; annotation is the smaller change,
live-reference is the more robust one. Decide per-consumer.

## Dependencies

None — foundational. Unblocks sprint 068 (encpose integrator consumes the
now-live scrub/trackwidth calibration) and sprint 069 (fitted parameters must
take effect on the robot).

## GitHub Issues

(none)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|

Tickets execute serially in the order listed.
