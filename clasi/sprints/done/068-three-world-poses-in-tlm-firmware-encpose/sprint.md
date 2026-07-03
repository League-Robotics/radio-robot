---
id: 068
title: Three world poses in TLM (firmware encpose)
status: done
branch: sprint/068-three-world-poses-in-tlm-firmware-encpose
use-cases: []
issues:
- tlm-three-world-poses-encoder-only-pose.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 068: Three world poses in TLM (firmware encpose)

## Goals

Have the firmware compute and transmit an encoder-only dead-reckoned world pose
(`encpose=`) so telemetry carries **three world poses** — encoder-only, OTOS,
and EKF — and delete the fragile host-side encoder integrator. Hosts become
dumb plotters of the three poses; the hardware-fit tooling in sprint 069 gets
the signal-by-signal comparison it needs.

## Problem

Telemetry today carries `otos=` (raw OTOS pose) and `pose=` (fused EKF), plus
raw cumulative wheel distances (`enc=`), but **no encoder-only pose**. The robot
is the only party that can integrate the encoders correctly (it owns trackwidth
and slip calibration and sees every sample at tick rate), yet the integration is
reconstructed host-side in `TraceModel._feed_encoder` with its own trackwidth
constant, a reset-detection heuristic, and a turn-scrub knob synced to the
*simulator's* injected error. That host integrator is a defect factory:
missed firmware resets cancelled turn headings (2026-07-01); the reset heuristic
still misses resets on slow TLM (CR-09, ~1–2 Hz relay); the GUI scrub knob
desynced from the sim's injected slip and produced a wildly rotated encoder
trace with all sim errors zero (2026-07-02).

## Solution

- Firmware computes an encoder-only world pose via an encoder-only variant of
  the existing `Odometry::predict` machinery, using the firmware's own
  geometry/slip calibration (this is exposing existing math, not new math).
- The pose is NOT EKF/OTOS-corrected, accumulates continuously across the
  per-drive-command encoder zeroing, and is re-referenced only by explicit
  pose-set commands (`SI`/`ZERO`).
- Emit `encpose=` in TLM. Weigh relay bandwidth (~12 msg/s radio): candidate
  options are STREAM-TLM only, alternating fields between frames, or a TLM
  verbosity flag. `parse_tlm` exposes the new field; sim golden-TLM / protocol
  tests updated.
- TestGUI plots `encpose=` directly; **delete** the host-side integrator —
  `TraceModel._feed_encoder`'s raw-count re-integration, its private trackwidth
  copy, its `set_turn_scrub_factor` compensation, and the reset heuristic.
  These are display-layer internals, not user controls.
- **Retained (stakeholder requirement):** the Sim Errors panel and all
  robot-config calibration values (`tw`, `rotational_slip`) are untouched and
  remain user-settable; they feed the firmware integrator that now produces
  `encpose=`.

## Success Criteria

- TLM (at least in streamed form) carries encoder-only, OTOS, and EKF poses.
- Sim golden-TLM / protocol tests updated; `parse_tlm` exposes the new field.
- TestGUI encoder trace plots `encpose=` directly; the host-side integration
  code (re-integration, private trackwidth/scrub, reset heuristic) is removed;
  the Sim Errors panel and robot-config calibration values are unaffected.
- With zero injected sim error, all three wire poses and plant ground truth
  agree over Tour 1 (sim regression test).

## Scope

### In Scope

- Firmware encoder-only pose integration + `encpose=` TLM field (with bandwidth
  strategy).
- `parse_tlm` field + protocol/golden-TLM test updates.
- TestGUI: plot `encpose=`; delete the host-side integrator internals.

### Out of Scope

- The config-propagation fix itself (sprint 067; consumed here).
- Expanding the Sim Errors panel / sim error model / fit tooling (sprint 069).

## Test Strategy

Sim regression: zero-error Tour 1 → three wire poses + plant ground truth agree.
Protocol/golden-TLM tests for the new field. Slow-TLM (relay-rate) case to
confirm the reset heuristic is genuinely gone (no host reconstruction left to
break).

## Architecture Notes

Bandwidth is the main design decision — a third pose on every TLM line is costly
on the relay. Prefer STREAM-only or a verbosity gate unless measurement shows
headroom.

## Dependencies

- **Sprint 067** — the firmware `encpose=` integrator consumes the now-live
  scrub/trackwidth calibration; without 067, `SET tw`/`SET rotSlip` wouldn't
  reach it.
- **Supersedes** sprint 066 ticket-004's host-side trace fix (the CR-09
  host-side direction): with `encpose=` on the wire, the reset heuristic ceases
  to exist.

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
| 001 | Firmware: expose encpose= in TLM | — |
| 002 | Host protocol: parse_tlm() + TLMFrame.encpose | 001 |
| 003 | TestGUI: delete host-side encoder integrator, plot encpose= directly | 002 |
| 004 | Regression: zero-error three-pose agreement | 002 |

Tickets execute serially in the order listed.
