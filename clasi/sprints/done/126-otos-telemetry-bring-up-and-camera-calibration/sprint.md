---
id: '126'
title: OTOS Telemetry Bring-Up and Camera Calibration
status: closed
branch: sprint/126-otos-telemetry-bring-up-and-camera-calibration
worktree: false
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
issues:
- otos-telemetry-bring-up-and-camera-calibration.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 126: OTOS Telemetry Bring-Up and Camera Calibration

## Goals

Turn the OTOS chip into a **legible, camera-verified instrument**: confirm
what the telemetry frame actually reports (units, robot-centre vs. chip
frame, lever-arm sign), and calibrate the two committed scale factors
(`otos_linear_scale`, `otos_angular_scale`) against overhead-camera ground
truth for distance and for heading/turns. Leave a repeatable bench/playfield
script behind. Do **not** fuse OTOS into the state estimate or use it for
heading control — `estimator.weight_heading_otos` / `weight_omega_otos`
stay 0.0 for the whole sprint.

## Problem

`Devices::RealOtos` is a real driver, already wired into `main.cpp`
(`FAKE_OTOS` off), the telemetry frame already carries `otos`/`otos_reading`/
`otos_present`/`otos_connected`/`otos_health`, and `data/robots/tovez.json`
already carries a lever arm (`geometry.odometry_offset_mm` = {x:-47.7, y:3.5,
yaw_rad:0.0}) and two scale factors (`otos_linear_scale` 1.067,
`otos_angular_scale` 0.987) that `gen_boot_config.py` bakes into
`OtosBootConfig` at boot. None of that has been checked against ground
truth: the units of the reported tuple are unconfirmed, whether the pose is
robot-centre or chip-frame is unconfirmed (a single at-rest sample after a
power cycle — `(48, -4, 0)`, suspiciously close to the negated configured
offset — is consistent with the centre-frame transform already being
applied correctly, but a single sample cannot rule out coincidence), and the
two scale factors are committed values of unknown provenance. Bringing the
chip up as a trusted instrument requires measuring all four against the
overhead AprilCam, not re-deriving them from the source.

## Solution

Six small, independently-runnable hardware/playfield tickets, each
producing one measured, printed result, building on one shared bench
script (`src/tests/bench/otos_calibration_bench.py`) that grows one mode
per ticket rather than being rewritten per ticket:

1. Confirm presence, units, and liveness (single known straight move).
2. Confirm lever-arm frame and sign (single in-place rotation).
3. Calibrate `otos_linear_scale` against camera distance (straight runs,
   both directions, varying length).
4. Calibrate `otos_angular_scale` against camera heading (in-place turns,
   both directions, varying magnitude, tether-respecting).
5. Fix the `test_calibration_kwargs.py` snapshot that pins the exact
   `tovez.json` calibration values these tickets may change.
6. Demonstrate all seven of the issue's acceptance criteria together, on
   the playfield, with a printed/charted report.

Tickets 1-2 gate 3-4 (frame/units must be known before a scale
regression means anything). Any `tovez.json` edit (3, 4) is boot-baked and
needs a reflash before it takes effect — the tether makes this possible.

## Success Criteria

The issue's own seven acceptance criteria, demonstrated on the playfield
against camera truth (tag 100 registered as the robot's centre of
rotation), results printed and charted:

1. `STATUS` reports `otos=1` and telemetry flag bit 0 is set, on a robot at
   `READY` with `connL=1`/`connR=1`.
2. OTOS pose changes when the robot moves and holds steady at rest; units
   and frame stated explicitly with the measurement that established them.
3. Whether the reported pose is robot-centre or chip is shown; the
   configured `odometry_offset_mm` lever arm is confirmed correct in
   magnitude AND sign.
4. Distance calibration: OTOS vs. camera distance over straight runs of
   differing lengths, both directions; linear scale reported;
   `otos_linear_scale` corrected if measurement disagrees with 1.067.
5. Heading/turn calibration: OTOS vs. camera heading change over turns of
   differing magnitude, both directions (tether-respecting); angular scale
   reported; `otos_angular_scale` corrected if measurement disagrees with
   0.987.
6. Residuals (mean, spread) stated for distance and heading, not hidden.
7. Sim suite still passes (`uv run python -m pytest src/tests/sim -q`);
   nothing in the estimator's fusion weights changed.

## Scope

### In Scope

- Bench/playfield measurement scripts (new `otos_calibration_bench.py`
  under `src/tests/bench/`, reusing `square_tour.py`'s lights-preflight and
  per-segment camera-fix pattern and `tlm_log.py`'s `otos_*` telemetry
  capture).
- Editing `data/robots/tovez.json`'s `calibration.otos_linear_scale` /
  `calibration.otos_angular_scale` **only if measurement disagrees** with
  the committed values, plus the reflash each edit requires.
- Fixing `src/tests/unit/test_calibration_kwargs.py`'s
  `test_calibration_commands_tovez_json_snapshot` (and its `tovez_nocal`
  sibling if touched), which pins the exact `tovez.json` calibration
  values these tickets may edit — already stale independent of this
  sprint (`rotSlip=0.92` pinned vs. the file's current `0.9117`).
- The final acceptance demonstration and report/chart.

### Out of Scope

- Fusing OTOS into the state estimate, or any change to
  `estimator.weight_heading_otos` / `weight_omega_otos` (stakeholder
  directive, 2026-07-29 — a later issue decides whether to trust it).
- Any change to `Devices::RealOtos`, the telemetry frame layout, or the
  OTOS wire/register protocol — the driver and frame are already correct
  and complete per the issue; this sprint measures, it does not rebuild.
  If a measurement ticket finds the firmware transform itself wrong (not
  expected — code reading shows `RealOtos::tick()` already applies the
  lever-arm and mounting-yaw transform correctly), that is a new issue,
  not an in-sprint fix.
- Correcting `odometry_offset_mm` if the lever-arm check (ticket 002)
  disagrees — the issue's acceptance criterion 3 asks only that magnitude
  and sign be *confirmed*, not corrected.
- Wiring `rotation_gain`/`rotation_offset_deg` (the affine turn-response
  correction) into firmware — unrelated pre-existing dead data, not part
  of this issue.

## Test Strategy

All new coverage here is a hardware/playfield measurement, not a unit
test — there is nothing to unit-test about a camera-vs-sensor comparison.
The sim suite (`uv run python -m pytest src/tests/sim -q`) has 11 known
pre-existing failures unrelated to this sprint (10 in `testgui`, sprint-108
sim-mode tour-1 fault-baseline; 1 in `test_calibration_kwargs`, a stale
snapshot). Ticket 005 fixes the `test_calibration_kwargs` one (bringing the
known-failure count to 10, all `testgui`); ticket 006's acceptance run
re-confirms the suite shows only that reduced known set. Every hardware
script follows the existing bench-script safety contract: lights
preflight, `estop()` (never `stop()`) in a `finally` block, tether-safe
turn sequencing (east→west through north, alternating direction).

## Architecture

**Compact** — adds one new module (an OTOS calibration bench-script suite
under `src/tests/bench/`), reusing already-wired telemetry/config/camera
plumbing (`RealOtos`, `TLMFrame`, `NezhaProtocol`, the AprilCam daemon);
no new cross-module dependency, no dependency-direction change, and no
data-model change (`tovez.json`'s two calibration scalars change in
*value*, not in schema).

### Architecture Overview

**What changed**: one module — `src/tests/bench/otos_calibration_bench.py`
— a bench/playfield script that turns the already-existing OTOS telemetry
channel into a measured, camera-verified calibration result. It grows one
mode per ticket (units/liveness, lever-arm, distance-scale, heading-scale,
acceptance-report) rather than being rewritten per ticket.

Boundary: it *reads* `TLMFrame`'s `otos_*` fields over the existing
`NezhaProtocol` connection (exactly as `tlm_log.py` already does) and reads
world poses from the existing AprilCam daemon (exactly as `square_tour.py`'s
per-segment camera-fix already does). It writes nothing to firmware, the
wire protocol, or the telemetry frame. Its only mutation is to
`data/robots/tovez.json`'s two calibration scalars (`otos_linear_scale`,
`otos_angular_scale`), and, downstream of that, to the stale
`test_calibration_kwargs.py` snapshot that pins their exact wire-command
encoding (`OL`/`OA`).

Use cases served: SUC-001 through SUC-005 (below).

**Why**: the OTOS driver, telemetry frame, and boot-config plumbing are
already built and already wired (verified on hardware 2026-07-29, see the
issue). What is missing is entirely empirical — is the reported pose
robot-centre or chip-frame, what are its units, and are the two committed
scale factors still correct — and empirical questions are answered by
measurement scripts and a data-value correction, not by new components.

**Impact on Existing Components**: None — additive and read-only against
every existing component except two artifacts this sprint may edit by
value: `data/robots/tovez.json`'s two calibration scalars, and
`test_calibration_kwargs.py`'s snapshot of their wire encoding. Neither
edit changes any interface, schema, or code path.

### Design Rationale

**Decision: one script with growing modes, not one script per ticket.**
- *Context*: the issue explicitly asks that a "repeatable bench/playfield
  script" be left behind, and each of tickets 1-4 needs its own
  independently-runnable hardware session (per the ticket guidance: a
  failed hardware run must not block the rest).
- *Alternatives considered*: (a) one script,
  `otos_calibration_bench.py`, that each ticket adds one mode to,
  reusing `square_tour.py`'s lights-preflight/camera-fix helpers and
  `tlm_log.py`'s telemetry-capture pattern (chosen); (b) four independent
  one-off scripts, one per measurement ticket.
- *Why this choice*: (b) would duplicate the lights-preflight,
  camera-bring-up, and `estop()`-in-`finally` boilerplate every existing
  bench script already factors out, and would leave four scripts behind
  instead of the one re-runnable artifact the issue asks for. (a) keeps
  each ticket's hardware run independently invokable (`--mode units`,
  `--mode lever-arm`, etc.) while landing in one place.
- *Consequences*: ticket 001 creates the script skeleton plus its first
  mode; tickets 002-004 each add one mode to the same file; ticket 006's
  acceptance run invokes all modes and assembles the final report.

### Migration Concerns

Any `tovez.json` edit this sprint makes (tickets 003/004, only if
measurement disagrees with the committed values) requires a reflash before
it takes effect — `otos_linear_scale`/`otos_angular_scale` are boot-baked
via `gen_boot_config.py`, not runtime `SET`-table keys (confirmed by the
issue: `App::Configurator` accepts only OTOS/ESTIMATOR/MOTOR patches, and
these two keys are not in the host's config key map). No wire/protocol
change. No estimator fusion-weight change — `weight_heading_otos`/
`weight_omega_otos` stay 0.0, checked explicitly in ticket 006.

## Use Cases

Sized to the compact tier — brief, a couple of sentences per SUC.

### SUC-001: OTOS Presence, Units, and Liveness Confirmed in Telemetry
Parent: UC-012

- **Actor**: Bench operator (via `otos_calibration_bench.py --mode units`)
- **Preconditions**: Robot on the playfield, tethered, at `READY` with
  `connL=1`/`connR=1`; lights confirmed on; camera bring-up complete.
- **Main Flow**: Confirm `STATUS` reports `otos=1` and telemetry flag bit 0
  is set; command one known straight-line move; compare the OTOS-reported
  delta against the camera-measured delta to settle units (mm vs. cm) and
  confirm the pose tracks motion and holds steady at rest.
- **Postconditions**: Units and liveness are stated explicitly, with the
  measurement that established them, printed by the script.
- **Acceptance Criteria**:
  - [ ] `otos=1` and flag bit 0 set are confirmed on a `READY` robot.
  - [ ] Units (mm vs. cm) are stated, derived from a measured delta, not
        from reading source.
  - [ ] Pose is shown to change under motion and hold steady at rest.

### SUC-002: OTOS Lever-Arm Frame and Sign Confirmed
Parent: UC-012

- **Actor**: Bench operator (via `otos_calibration_bench.py --mode
  lever-arm`)
- **Preconditions**: SUC-001 complete (units known). Robot at rest,
  tether-safe orientation.
- **Main Flow**: Command a single in-place rotation; capture camera-truth
  poses at rest before/after; compare the OTOS pose's motion during the
  turn against the two hypotheses (a stationary centre vs. a chip tracing
  a circle of radius `|odometry_offset_mm|`) to determine which frame the
  telemetry reports, and confirm the configured offset's magnitude and
  sign against whichever frame it is.
- **Postconditions**: Frame (centre vs. chip) and lever-arm magnitude/sign
  are stated explicitly, with the measurement that established them.
- **Acceptance Criteria**:
  - [ ] Frame (robot-centre vs. chip) is determined from the rotation
        measurement, not asserted from code reading.
  - [ ] `odometry_offset_mm` magnitude and sign are confirmed correct
        against that measurement, or a mismatch is reported (not
        corrected — out of scope, see sprint Scope).

### SUC-003: OTOS Distance Scale Calibrated Against Camera
Parent: UC-013

- **Actor**: Bench operator (via `otos_calibration_bench.py --mode
  distance`)
- **Preconditions**: SUC-001/SUC-002 complete.
- **Main Flow**: Drive a set of straight runs of differing lengths, both
  directions; compare OTOS-reported distance to camera-measured distance
  at each run; fit the linear scale and its residual (mean, spread).
- **Postconditions**: `otos_linear_scale` is reported and, if the
  measured scale disagrees with the committed 1.067, corrected in
  `tovez.json` and the robot reflashed.
- **Acceptance Criteria**:
  - [ ] Distance comparison covers multiple lengths, both directions.
  - [ ] Linear scale and its residual (mean, spread) are printed.
  - [ ] `tovez.json` is updated and the robot reflashed if the measured
        scale disagrees with 1.067.

### SUC-004: OTOS Heading/Turn Scale Calibrated Against Camera
Parent: UC-013

- **Actor**: Bench operator (via `otos_calibration_bench.py --mode
  heading`)
- **Preconditions**: SUC-001/SUC-002 complete. Tether rule in force:
  turns go east→west through north only, alternating direction across
  successive turns.
- **Main Flow**: Drive a set of in-place turns of differing magnitudes,
  both directions, respecting the tether rule; compare OTOS-reported
  heading change to camera-measured heading change at each turn; fit the
  angular scale and its residual (mean, spread).
- **Postconditions**: `otos_angular_scale` is reported and, if the
  measured scale disagrees with the committed 0.987, corrected in
  `tovez.json` and the robot reflashed.
- **Acceptance Criteria**:
  - [ ] Turn comparison covers multiple magnitudes, both directions,
        tether rule respected throughout.
  - [ ] Angular scale and its residual (mean, spread) are printed.
  - [ ] `tovez.json` is updated and the robot reflashed if the measured
        scale disagrees with 0.987.

### SUC-005: OTOS Bring-Up Acceptance Demonstrated, Fusion Untouched
Parent: UC-012

- **Actor**: Bench operator (via `otos_calibration_bench.py --mode
  acceptance`), CI (`pytest`)
- **Preconditions**: SUC-001 through SUC-004 complete; `tovez.json`
  calibration snapshot fixed (ticket 005).
- **Main Flow**: Run all prior modes' checks together on the playfield
  (including a full-tour liveness check, not just a single leg); assemble
  the printed/charted report against the issue's seven acceptance
  criteria; run the sim suite and confirm only the known, reduced
  pre-existing-failure set remains; grep-confirm
  `weight_heading_otos`/`weight_omega_otos` are still 0.0.
- **Postconditions**: All seven acceptance criteria are demonstrated
  together with one printed/charted report; sim suite passes at the known
  baseline; fusion weights unchanged.
- **Acceptance Criteria**:
  - [ ] All seven of the issue's acceptance criteria are demonstrated in
        one run, with printed results and a chart.
  - [ ] Sim suite passes with only the known, reduced pre-existing-failure
        set (10, `testgui`-only).
  - [ ] `estimator.weight_heading_otos` / `weight_omega_otos` confirmed
        still 0.0.

## GitHub Issues

(None — this sprint is driven by `clasi/issues/otos-telemetry-bring-up-and-camera-calibration.md`, not a GitHub issue.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning document is complete (sprint.md, including its
      Architecture and Use Cases sections)
- [x] Architecture review passed (or skipped, for changes with no
      architectural impact)
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | OTOS presence, units, and liveness bring-up | — |
| 002 | OTOS lever-arm frame and sign confirmation | 001 |
| 003 | OTOS distance-scale calibration against camera | 001, 002 |
| 004 | OTOS heading/turn-scale calibration against camera | 001, 002 |
| 005 | Fix stale `test_calibration_kwargs` snapshot | 003, 004 |
| 006 | OTOS bring-up acceptance demonstration | 001, 002, 003, 004, 005 |

Tickets execute serially in the order listed.
