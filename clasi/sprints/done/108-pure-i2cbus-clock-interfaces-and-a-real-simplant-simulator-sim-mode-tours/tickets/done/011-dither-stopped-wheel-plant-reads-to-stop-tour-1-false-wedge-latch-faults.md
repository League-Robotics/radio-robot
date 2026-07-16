---
id: '011'
title: Dither stopped-wheel plant reads to stop Tour 1 false wedge-latch faults
status: done
use-cases:
- SUC-042
depends-on: []
github-issue: ''
issue: sim-mode-tour-1-fault-baseline-exclusion-mismatch.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Dither stopped-wheel plant reads to stop Tour 1 false wedge-latch faults

## Description

Closes `clasi/issues/sim-mode-tour-1-fault-baseline-exclusion-mismatch.md`,
discovered during ticket 108-007. Sim-mode Tour 1 (`run_tour(parse_tour
(TOUR_1), SimLoop)`, `host/robot_radio/io/sim_loop.py` +
`planner.executor.StreamingExecutor`) reliably fault-stops on
`kFaultWedgeLatch` at leg boundaries, though the same tour completes on
real hardware and a raw `SimLoop` twist/stop sequence never faults.

**Root cause (already confirmed by a read-only investigation, not
re-derived by this ticket)**: the firmware wedge detector
(`source/devices/motor_armor.h::updateWedgeDetector()`, `kWedgeThreshold
=10`) latches after 10 consecutive byte-identical encoder reads (~500ms
at 50ms/cycle) — intentionally not motion-gated (064-004 hardening; do
not touch firmware). `SimPlant`/`WheelPlant::reportedPosition()`
(`tests/sim/plant/wheel_plant.cpp`) reports a stopped wheel's position
with ZERO noise, quantized to 0.1mm tenths — a stopped wheel therefore
emits byte-identical tenths every cycle, unlike a real encoder's natural
jitter, so the wedge latches on every idle/leg-boundary window. Because
the bit is absent when `executor.py`'s `begin()` snapshots its
fault-bit baseline and appears a few ticks later, the baseline-exclusion
logic (107-001) reads it as a NEW fault rather than pre-existing. The
firmware behavior is correct; the defect is the plant being
unrealistically noise-free at rest. The fix belongs in the plant, not
the firmware or the executor.

**Verified fix**: add a per-wheel, rest-gated, seeded alternating ±1 LSB
(0.1mm) dither to `WheelPlant::reportedPosition()`'s nominal branch only,
confirmed to take Tour 1 from 4/4 faults to 13/13 legs / 0 faults.

## Acceptance Criteria

- [x] `tests/sim/plant/wheel_plant.h`/`.cpp` add a rest-gated, per-wheel
      alternating ±1 LSB (0.1mm) dither to `reportedPosition()`'s
      NOMINAL branch only. `freezePosition_` (the fault-knob freeze
      scenario) and the dropout branch are untouched.
- [x] The dither is gated on rest (e.g. `fabsf(velocity_) < ~1 mm/s`, the
      sub-LSB-per-cycle regime) — a moving wheel's `reportedPosition()`
      stays exactly `position_` (unchanged, deterministic); only a
      stopped wheel's read is dithered.
- [x] The dither source is a stateful, seeded alternating phase bit (a
      `ditherPhase_`-style field set at construction and flipped each
      dithered read) — no RNG. `position()`/`velocity()` (plant truth)
      are unaffected; only the reported/wire value changes.
- [x] The dither is keyed per wheel port (left and right dither
      independently) — not a single global toggle, which only relocates
      the fault to turn boundaries where L/R reads alternate.
- [x] `tests/testgui/test_sim_transport_tour1.py::
      test_tour_1_runs_to_completion_with_finite_small_closure` is
      un-marked `xfail` and passes for real: Tour 1 runs all 13 legs to
      completion with 0 faults and a finite/small closure.
- [x] The investigation's noted caveat — headless `closure.position_delta`
      reading `0.0` in that test's path (possible `TLMFrame.pose`
      population quirk) — is either fixed as part of this ticket if it's
      in-scope for the same code path, or is explicitly re-filed as a
      separate, clearly-described follow-up issue if it isn't. Do not
      leave it silently unmentioned in the ticket's completion notes.
      **Disposition**: not reproduced. With jitter enabled on the ctypes
      path, `test_tour_1_runs_to_completion_with_finite_small_closure`
      passes with a real, finite, non-zero `closure.position_delta` (see
      Completion Notes below) — the earlier `0.0` reading was a symptom
      of the wedge-latch fault-stop cutting the tour short before a
      closure could be computed from real motion, not an independent
      `TLMFrame.pose` bug. No follow-up issue filed.
- [x] Fault-knob scenario tests under `tests/sim/system/faults/` still
      pass: the FREEZE scenario still latches the wedge (proving the
      dither is confined to the nominal branch), and the DROPOUT scenario
      still asserts no false latch.
- [x] `plant_harness.cpp`'s determinism check (run-A == run-B, same seed)
      still passes.
- [x] `straight_twist_harness.cpp`'s tolerances still pass (the dither
      only perturbs `reportedPosition()` at rest, not the moving case).
- [x] `uv run python -m pytest` is fully green with no new failures
      (beyond the intentional xfail flip above).
- [x] `python build.py --fw-only` is unaffected — no firmware file is
      touched by this ticket.
- [x] `clasi/issues/sim-mode-tour-1-fault-baseline-exclusion-mismatch.md`
      is resolved by this ticket's `completes_issue: true`.

## Completion Notes

**Regression found and fixed mid-ticket**: the first cut of this fix
applied the rest-dither unconditionally to every `WheelPlant`, which is a
GLOBAL change — it broke every C++ scenario test that asserts an exact,
byte-stable stopped-wheel `reportedPosition()`
(`tests/sim/system/test_scripted_twist_demo.py` and others), violating
`WheelPlant`'s own documented "seeded, deterministic, run-A==run-B"
contract for those direct-C++-harness callers.

**Fix: opt-in jitter, default OFF.** `WheelPlant` gained a per-instance
`encoderJitter_` flag (default `false`) plus `setEncoderJitter(bool)` /
`encoderJitter()`; `reportedPosition()`'s rest branch only dithers when
the flag is on (`tests/sim/plant/wheel_plant.{h,cpp}`).
`TestSim::SimPlant::setEncoderJitter(bool)` (`tests/_infra/sim/
sim_plant.{h,cpp}`) fans the flag out to both the left and right
`WheelPlant`s. Jitter is turned on in exactly one place:
`tests/_infra/sim/sim_ctypes.cpp`'s `sim_create()`, immediately after
constructing the `SimHarness` and before `boot()` — every ctypes/Python
consumer of the sim (the tour runner, TestGUI's sim-mode transport, `host/
robot_radio/io/sim_loop.py`'s `SimLoop`) therefore gets hardware-like
encoders by default, with zero host-side (`sim_loop.py`) churn needed.
The plain C++ `SimHarness`/`SimPlant` construction path used directly by
`tests/sim/system/*.cpp` scenario tests and `plant_harness.cpp` never
calls `sim_ctypes.cpp`, so it stays on the deterministic default (jitter
off) — `test_scripted_twist_demo`, `test_straight_twist`, the fault-knob
scenarios under `tests/sim/system/faults/`, and `plant_harness.cpp`'s
run-A==run-B determinism check all pass unchanged.

**Full-suite verification** (`uv run python -m pytest -q`, foreground,
~4m18s): **1111 passed, 5 skipped, 4 xfailed, 1 xpassed, 0 failed** — the
xfail count dropped from 5 (pre-ticket) to 4 now that Tour 1's `xfail`
marker is removed and the test passes for real. Spot-confirmed
individually:
- `tests/testgui/test_sim_transport_tour1.py::
  test_tour_1_runs_to_completion_with_finite_small_closure` — PASSES
  (jitter on via `sim_create()` → Tour 1 clears the wedge-latch, all 13
  legs complete, finite non-zero closure).
- `tests/sim/system/test_scripted_twist_demo.py` and
  `tests/sim/system/test_straight_twist.py` — PASS unchanged (jitter off
  on the direct-C++-harness path → fully deterministic).
- `tests/testgui/test_sim_loop.py::
  test_write_hook_can_swallow_a_command` — PASSES (the hold-then-flip
  `kDitherPeriod=3` refinement, still gated by the same opt-in flag,
  keeps the PID's proportional term at zero on held reads).

## Implementation Plan

**Approach**: Small, targeted fix confined to the host-side sim plant.
Add a per-wheel dither field and rest-gate check inside
`WheelPlant::reportedPosition()`'s existing nominal branch; no new
classes, no interface changes, no firmware changes. This is a sim-fidelity
correction, not an architecture change — no architecture-update.md
revision is needed for this ticket.

**Files to modify**:
- `tests/sim/plant/wheel_plant.h` — add a `ditherPhase_`-style per-wheel
  field (or equivalent), reset at construction.
- `tests/sim/plant/wheel_plant.cpp` — `reportedPosition()`: in the
  nominal branch, when `fabsf(velocity_)` is below the rest threshold,
  add the current dither phase (±0.1mm) to the quantized read and flip
  the phase; leave the `freezePosition_` and dropout branches untouched.
- `tests/testgui/test_sim_transport_tour1.py` — remove the `xfail(strict
  =False)` marker on `test_tour_1_runs_to_completion_with_finite_small_
  closure`; address or re-file the `closure.position_delta == 0.0`
  caveat noted in the issue.

**Files to create**: none expected; if the `position_delta` caveat turns
out to be a separate, unrelated bug outside this code path, file a new
`clasi/issues/*.md` for it rather than expanding this ticket's scope.

**Testing plan**:
1. `uv run python -m pytest tests/sim/plant/` and
   `tests/sim/system/faults/` — plant determinism + fault-knob scenarios
   green.
2. `uv run python -m pytest tests/testgui/test_sim_transport_tour1.py -v`
   — Tour 1 completion test passes (no xfail), 0 faults, 13/13 legs.
3. `uv run python -m pytest` (full suite) — fully green, no regressions.
4. Manual sanity (optional, not gating): a short repro loop of headless
   Tour 1 runs (the same shape as the investigation's repro) confirms 0
   faults across multiple runs, not just one lucky seed ordering.

**Documentation updates**: none required beyond this ticket's own
completion notes recording the before/after fault counts and confirming
the caveat's disposition; `clasi/issues/sim-mode-tour-1-fault-baseline-
exclusion-mismatch.md` is closed via `completes_issue: true` at sprint
close.
