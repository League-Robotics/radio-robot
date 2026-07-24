---
id: '001'
title: 'Analytic completion: measured-speed coast rule, tau_plant characterization
  + plumbing, margin-constant deletion'
status: open
use-cases:
- SUC-075
depends-on: []
github-issue: ''
issue: chain-advance-completion-margin-narrow-pocket.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Analytic completion: measured-speed coast rule, tau_plant characterization + plumbing, margin-constant deletion

## Description

Delete `kStoppingMarginFactorChain`/`Final`/`Orthogonal` and
`kDiscretizationCyclesChain` from `move_queue.cpp` (grep-clean). Replace
`MoveQueue::landAtZero()`'s completion predicate with
`remaining <= |measuredSpeed| * (kCycle-or-dt/2 + tauPlant)`, applied
uniformly to final, orthogonal-chain, and same-axis-chain boundaries — state
explicitly what "terminal condition under carry" means operationally (the
predicate still governs same-axis completion even though ticket 002 stops
resetting the shaper there). `measuredSpeed` is the same-cycle
`frame_.twist.v_x`/`omega` `RobotLoop::cycle()` already computes via
`BodyKinematics::forward()` — thread it into a widened `MoveQueue::tick()`
signature. Do NOT give `MoveQueue` its own motor references, and do NOT
resurrect `StateEstimator::bodyAt()` (stays quarantined).

`tauPlant` is the ONE new physically-derived constant this sprint permits.
Characterize it via a dedicated step-response test against the same sim
plant the closure gate runs (mirroring `velocity_step_response.py`'s bench
method / `tune_velocity_pid.py`'s sim-side precedent) — measuring
commanded-to-achieved speed lag directly, independent of any tour or
turn-accuracy assertion. **Never sweep `tauPlant` against
`test_tour_closure_gate.py`** — that is exactly the mistake this sprint
exists to undo (see 121-003's own "HONEST RESIDUAL" comment in
`move_queue.cpp` for why margin-fraction tuning cannot work). Commit the
measured value to `data/robots/*.json` as `control.tau_plant` with a
derivation note (mirroring the `_shaper_note` convention already used for
`a_max`/`a_decel`/etc).

Full design context: `clasi/sprints/122-same-axis-carry-through-chain-margin-cleanup/sprint.md`'s
Architecture section (Decisions 1-3) and Open Questions 1-2.

## Approach

1. Characterize `tauPlant` (step-response test, sim plant) — produce a
   citable, reviewable measurement, not a guessed number.
2. Commit `tauPlant` to every committed robot JSON's `control.tau_plant`.
3. Wire schema: add field 11 `tau_plant` to `EstimatorConfigPatch`
   (`config.proto`), regenerate `config.h`/`envelope_pb2.py`.
4. `Config::ShaperBootConfig` gains `tauPlant`; `gen_boot_config.py`'s
   `shaper_config_for_config()` reads `control.tau_plant`.
5. Host: `robot_config.py`'s `ControlConfig` gains `tau_plant`;
   `NezhaProtocol.estimator_config()` gains a `tau_plant` kwarg.
6. `App::ShaperLimits` gains `tauPlant`; `RobotLoop::handleConfig()`'s
   existing `ESTIMATOR` branch applies it (never persisted, same contract as
   its five siblings).
7. Widen `MoveQueue::tick()`'s signature to accept measured `v_x`/`omega`;
   update its one call site in `RobotLoop::cycle()` to pass
   `frame_.twist.v_x`/`omega`.
8. Rewrite `landAtZero()` per the analytic formula; delete the four margin
   constants and their comment archaeology (relocate historical sweep
   narrative into `DESIGN.md` as a closed chapter, per the sprint's Solution
   section).
9. Re-measure (do not assume) TOUR_1/TOUR_2 ideal-chip numbers against sim
   ground truth; state the achieved numbers honestly against the S1 bar.
10. Update `move_queue.h`/`.cpp`'s doc comments, `messages/DESIGN.md`,
    `config/DESIGN.md` (direct edits), and the sprint's `design/DESIGN.md`
    overlay (§4 land-at-zero paragraph).

## Acceptance Criteria

- [ ] `tauPlant`'s value is traceable to an independent step-response
      characterization (named method/script) — never fitted against
      `test_tour_closure_gate.py` or any tour/turn assertion.
- [ ] `kStoppingMarginFactorChain`/`Final`/`Orthogonal`/
      `kDiscretizationCyclesChain` no longer exist anywhere in
      `move_queue.cpp` (grep-clean; historical derivation relocated to
      `DESIGN.md`/comment archaeology, not deleted outright).
- [ ] `landAtZero()` fires uniformly off measured speed for Distance (v_x)
      and Angle (omega) kinds, for final/orthogonal-chain/same-axis-chain
      boundaries alike; the half-cycle term's source (local `dt` vs. a
      hypothetical `RobotLoop::kCycle`) is decided and stated with its
      dependency-direction justification (recommend local `dt` — avoids
      `App::MoveQueue` depending on `App::RobotLoop`).
- [ ] `App::ShaperLimits` gains `tauPlant`; `EstimatorConfigPatch`/
      `config.proto` gain field 11 `tau_plant`, applied by the existing
      `ESTIMATOR` branch of `RobotLoop::handleConfig()`, never persisted
      (same volatile contract as its five siblings).
- [ ] `gen_boot_config.py`'s `shaper_config_for_config()` reads
      `control.tau_plant`; `robot_config.schema.json` requires it;
      `robot_config.py`'s `ControlConfig` and `NezhaProtocol.
      estimator_config()` carry it end to end.
- [ ] Deterministic sim, ideal chip, TOUR_1/TOUR_2 re-measured (not assumed)
      against the new predicate: straights-following-turns gain and turn
      |error| are stated honestly against the S1 bar
      (`docs/design/goal-exact-tours.md`: per-motion heading ≤0.1°/position
      ≤1mm; tour net ≤0.5°, closure ≤5mm, per-leg straight gain ≤0.1°). If
      not met, the specific residual mechanism and magnitude are named
      (feeds ticket 003's floor-naming decision) — no silent gap.
- [ ] `tauPlant` is the only new constant introduced; its physical
      derivation is named in the implementation (replan non-negotiable #3 —
      no tuned compensation constants).
- [ ] Full suite green (`uv run python -m pytest`), including
      `test_gui_button_acceptance.py`'s managed-turn presets, re-verified
      against the new predicate (not assumed carried over);
      `test_app_move_queue.py`/`app_move_queue_harness.cpp`/
      `test_move_queue.py`'s `tick()` call sites updated for the widened
      signature.
- [ ] STANDING VERIFICATION GATE (`.claude/rules/hardware-bench-testing.md`):
      built + flashed to the robot on the stand and exercised on real
      hardware — sensors alive and changing, wheels drive both directions
      with encoders incrementing, and a tour (or the relevant managed
      turn/straight sequence) driven and observed over the real link — NOT
      tests alone. Record the bench results in this ticket.
      (pending team-lead bench run on the stand)

## Files to modify

- `src/firm/app/move_queue.{h,cpp}` — the completion predicate rewrite,
  `tick()` signature widening, margin-constant deletion.
- `src/firm/app/robot_loop.{h,cpp}` — the one `moveQueue_.tick(...)` call
  site widens to pass `frame_.twist.v_x`/`omega`.
- `src/firm/messages/config.h`, `src/protos/config.proto` — new field 11
  `tau_plant` on `EstimatorConfigPatch` (+ regenerated `config.h`/
  `envelope_pb2.py`).
- `src/firm/config/boot_config.{h,cpp}` — `Config::ShaperBootConfig` gains
  `tauPlant`.
- `src/scripts/gen_boot_config.py` — reads `control.tau_plant`.
- `data/robots/*.json`, `data/robots/robot_config.schema.json` — new
  required `control.tau_plant` key + derivation note.
- `src/host/robot_radio/config/robot_config.py`, `src/host/robot_radio/robot/protocol.py`
  — `ControlConfig.tau_plant`, `NezhaProtocol.estimator_config()` kwarg.
- `src/firm/messages/DESIGN.md`, `src/firm/config/DESIGN.md` — direct edits
  (not overlaid).
- `clasi/sprints/122-same-axis-carry-through-chain-margin-cleanup/design/DESIGN.md`
  — overlay edit (§4 land-at-zero paragraph); diff via
  `clasi.design.overlay.generate_diffs`, validate via
  `clasi design validate --overlay` before close.
- Possibly a new step-response characterization script (sim-side).

## Testing

- **Existing tests to run**: `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py -v`,
  `test_gui_button_acceptance.py`'s managed-turn presets,
  `src/tests/sim/unit/test_app_move_queue.py`,
  `src/tests/sim/system/test_move_queue.py`, full
  `uv run python -m pytest`.
- **New tests to write**: the `tauPlant` step-response characterization test
  itself; a `landAtZero()` measured-speed unit scenario in the `MoveQueue`
  harness.
- **Verification command**: `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py -v`
  for the sim gate, THEN `mbdeploy deploy --build` + a bench tour/turn run
  over `/dev/cu.usbmodem2121102` on the stand (bench gate) with results
  recorded here.
