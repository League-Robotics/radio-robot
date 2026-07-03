---
id: 069
title: 'Sim error model: runtime-settable, plant-complete, hardware-fit'
status: planning-docs
branch: sprint/069-sim-error-model-runtime-settable-plant-complete-hardware-fit
use-cases: []
issues:
- expose-sim-error-model-knobs-in-testgui.md
- sim-error-model-runtime-settable-hardware-fit.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 069: Sim error model — runtime-settable, plant-complete, hardware-fit

## Goals

**The simulator and the real robot must be tunable to behave identically.**
Make every simulator error/plant parameter settable and readable over the wire
at runtime, give the plant a physically complete error model (a body that
genuinely scrubs), expose the full knob set in the TestGUI, and build the
host-side regression that fits sim parameters to a recorded hardware run so a
sim replay reproduces the hardware trajectory. This sprint folds in the
"expose existing knobs in the TestGUI" work as part of the wire-command surface,
rather than plumbing it through the legacy ctypes path first and reworking it.

## Problem

1. **Error parameters are not uniformly runtime-settable.** Some knobs exist
   only behind the ctypes C-ABI (`sim_set_motor_slip`, `sim_set_encoder_noise`,
   OTOS noise setters), reachable only by a host process linked to the sim
   library; others are compile-time constants. Nothing conceptually a simulator
   parameter should require a recompile — the sim needs a `SET`/`GET`-equivalent
   command surface for plant/error values.
2. **The error model is physically incomplete.** "Turn slip" corrupts only the
   *reported* encoders; the plant body never scrubs (`effectiveSlip(<=0)` maps
   to 1.0). Meanwhile firmware turn control compensates for 8% scrub
   (`rotationalSlip=0.92`), so a "zero-error" sim structurally over-rotates
   (RT 9000 → 95.2° true; the 45° case only looks right because two errors
   cancel). A plant that cannot scrub can never be fitted to a robot that does.
3. **The TestGUI exposes only 4 of the existing knobs** (encoder noise, turn
   slip, OTOS linear/yaw noise); scale-error and drift setters already exist in
   the plant but are not surfaced.

## Solution

- **Wire-settable surface.** Give the sim a `SET`/`GET`-equivalent for
  plant/error values (a sim-build-only `SIMSET`/`SIMGET` or a reserved key
  namespace in the existing ConfigRegistry). The ctypes setters remain as a
  thin wrapper over the same registry.
- **Full plant error knobs**, each a parameter of a defined error function so it
  can be a regression variable: per-wheel encoder scale/slip/Gaussian noise
  (exist), **body rotational scrub (NEW — currently impossible)**, straight-line
  slip, trackwidth error, OTOS linear/yaw noise + scale, motor/actuation
  asymmetry (per-side gain), response lag / coast.
- **Expose all knobs in the TestGUI Sim Errors panel** (issue C folded in):
  encoder scale L/R, OTOS linear/angular scale, OTOS linear/yaw drift, and
  optionally motor offset factor and trackwidth error — surfaced through the
  wire-command surface, with defaults of 0.0 (no behavior change until opted in).
- **Host-side fit tooling.** A script that takes a recorded hardware run (wire
  log + camera truth + the three TLM poses) and regresses the sim error
  parameters to minimize trajectory disagreement, emitting a parameter file the
  sim loads.

## Success Criteria

- Every documented sim error parameter can be set/read over the wire at runtime;
  a test sweeps each knob and observes the corresponding telemetry change.
- With body-scrub set to match `rotationalSlip=0.92`, RT 9000 lands on 90° true
  in sim (closing today's 95.2° gap); with all knobs zero AND `rotationalSlip`
  effectively 1.0, RT is exact.
- The TestGUI Sim Errors panel exposes the full knob set (existing 4 + the newly
  surfaced knobs).
- End-to-end demo: record a hardware Tour 1, fit parameters, replay Tour 1 in
  sim with fitted parameters, and show trajectory agreement within a stated
  tolerance.

## Scope

### In Scope

- Wire-command surface for sim plant/error parameters (SIMSET/SIMGET or reserved
  namespace); ctypes setters rebased onto it.
- Body rotational scrub in the plant + the remaining plant error functions.
- Full knob exposure in the TestGUI Sim Errors panel.
- Host-side regression fit tooling + end-to-end Tour 1 demo.

### Out of Scope

- **Sim-OTOS ground-truth + lever-arm** — delivered by sprint 066; this sprint
  builds on it and must not re-cover it.
- Config propagation to firmware consumers (sprint 067) and `encpose=` telemetry
  (sprint 068) — both consumed here, not re-done.
- Future model extensions noted in the issues (encoder additive bias `b`,
  Brownian yaw drift, encoder gain noise).

## Test Strategy

Per-knob sweep tests asserting the telemetry response. The two RT-rotation
acceptance points (scrub=0.92 → 90°; all-zero → exact). An end-to-end
record → fit → replay regression demonstrating trajectory agreement within
tolerance, using the three TLM poses from sprint 068.

## Architecture Notes

The wire-command surface is the pivotal decision — a sim-build-only verb pair vs.
a reserved ConfigRegistry namespace. Body rotational scrub is a genuine plant
model addition (the currently-impossible piece), distinct from the existing
reported-encoder slip. Depends on the three-pose telemetry from 068 as the
fitting signal.

## Dependencies

- **Sprint 067** — fitted calibration values must actually take effect on the
  robot (SET→consumer propagation).
- **Sprint 068** — the three TLM world poses provide the signal-by-signal
  trajectory comparison the fit tooling regresses against.
- **Builds on sprint 066** — sim-OTOS ground-truth and lever-arm (out of scope
  here).

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
| 001 | Expose seven EKF noise fields via SET/GET (close 067 Open Question 5) | none |
| 002 | PhysicsWorld body-rotational and body-linear scrub (new, independent, multiplicative) | none |
| 003 | SIMSET/SIMGET wire-command surface (SimCommands, sim-build-only) | 002 |
| 004 | Surface remaining sim-error knobs through SIMSET/SIMGET (encoder, OTOS) | 003 |
| 005 | Rebase ctypes sim setters as thin wrappers over shared SIMSET setter functions | 003, 004 |
| 006 | Comprehensive per-knob telemetry sweep test for the SIMSET registry | 004 |
| 007 | TestGUI Sim Errors panel: expose full knob set via SIMSET | 004, 005 |
| 008 | Host-side fit tooling: fit_sim_error_model.py (scipy least_squares, sim-to-sim validated) | 004, 006 |

Tickets execute serially in the order listed.
