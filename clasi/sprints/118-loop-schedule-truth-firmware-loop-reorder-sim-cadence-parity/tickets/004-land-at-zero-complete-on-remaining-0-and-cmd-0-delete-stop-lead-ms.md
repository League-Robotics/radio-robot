---
id: '004'
title: "Land at zero: complete on remaining\u22480 AND \u03C9_cmd\u22480; delete stop_lead_ms"
status: open
use-cases:
- SUC-066
depends-on:
- '002'
github-issue: ''
issue:
- land-at-zero-completion-delete-stop-lead.md
- turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Land at zero: complete on remaining≈0 AND ω_cmd≈0; delete stop_lead_ms

## Description

**Pulled forward from sprint 119 into 118 (2026-07-23), mid-execution,
by team-lead decision.** Ticket 002 landed the odometry-freshness fix
(`moveQueue_.tick()` relocated into the pace block after
`odom_.integrate()`/`stateEstimator_.update()`) — but the closure gates
went RED for exactly the reason `land-at-zero-completion-delete-stop-lead.md`
predicted: with fresh same-cycle odometry, the stale-tuned
`stop_lead_ms=45` now OVERcompensates. Full data (a 0-120ms sweep against
the closure gate's own exact path) is recorded as a dated addendum in
that issue and in ticket 002's own report; the short version: TOUR_1
worst-case 4.39°, the ±90° preset tests ~93.7° (previously clean, now
failing their own tightened ±3.0° band), and the sweep found only a
fragile ~1ms-wide passing window at ~62ms — not the broad 30-54ms
plateau the shipped 45ms value was originally chosen from. No value in
the bracket has real margin. Per the turn-execution review's own R6 rule
("a change that adds or retunes a numeric constant must name the
physical quantity it models and derive it from named constants; if
adding a stage forces retuning an existing constant, the default action
is to delete the constant, not retune it") and the project's
sprint-end-must-be-testable convention, retuning is not an option here —
this ticket deletes `stop_lead_ms` and lands the taper-to-zero completion
predicate instead, so 118 ends on a green, testable closure gate rather
than handing a known-red gate to a not-yet-detailed sprint 119.

Turn completion today is an open-loop time-lead guess: the stop fires
when a *predicted* heading (`stateEstimator_.bodyAt(nowMs + stopLead_)`,
`move_queue.cpp:232-233`) crosses the threshold. The decel taper already
commands `ω = √(2·α_decel·(remaining − jerkMargin))`
(`velocity_shaper.cpp:101-109`) — the robot is DESIGNED to arrive at the
goal at ~zero speed. Let it finish: declare completion when
`remaining ≤ ε AND |ω_cmd| ≤ ε_ω`, keep the `StopCondition`
threshold/timeout as the always-armed backstop, and DELETE `stop_lead_ms`
+ the anticipation block rather than re-deriving it. There is then no
tail to predict.

## Binding Design Constraints (from `land-at-zero-completion-delete-stop-lead.md`, verified against code 2026-07-22/23)

1. **The predicate lives in `MoveQueue::tick()`**, not `StopCondition`:
   `Motion::StopCondition` is pure and dependency-free with no access to
   shaper state (`stop_condition.h:5-16`). No new `StopCondition` Kind.
   The threshold/timeout outcome remains the backstop path, ALWAYS
   evaluated (not bypassed when the land-at-zero gate is available).
2. **Shaping-off keeps threshold semantics.** With all-zero
   `ShaperLimits`, `shapeAndStage()` early-returns (`move_queue.cpp:143`)
   — no taper exists and `ω_cmd` never bleeds, so a land-at-zero gate
   would never fire. The backstop is the ONLY completion path in that
   regime (exactly today's behavior — do not change it).
3. **Scope: TWIST moves with Angle (ω axis) and Distance (v_x axis)
   stops only.** TIME stops have no spatial `remaining`
   (`move_queue.cpp:149-160`); WHEELS moves never taper the stop axis
   (`:111-137`). Both keep pure threshold/timeout semantics — byte-
   identical behavior, regression-tested.
4. **`ε_ω` must clear the deadband floor.** Sub-deadband nonzero targets
   are boosted to ~15 mm/s per wheel (`nezha_motor.cpp:559-566`; ≈0.23
   rad/s ≈ 13°/s equivalent on the ω axis at 128 mm trackwidth), so
   commanded ω never settles below that while nonzero. Set `ε_ω` just
   above the deadband-equivalent floor; on completion `Drive::stop()`
   stages exact zero, bypassing the boost and engaging the rest gate.
   Residual coast from the floor is bounded by `τ·ω_floor ≈ 0.13s ·
   13°/s ≈ 1.7°` worst-case — budget it in the acceptance band.
5. **Delete list** (in one commit, schema + all consumers together):
   - `stopLead_` member + ctor param (`move_queue.{h,cpp}`), the
     anticipation block (`move_queue.cpp:230-240`).
   - `stop_lead_ms` from the `EstimatorConfigPatch` wire arm, the
     `estimator_kwargs()` push, the pydantic model
     (`robot_config.py`/`ControlConfig` or equivalent estimator schema).
   - `stop_lead_ms` (+ each `_estimator_note` archaeology block) from
     all three `data/robots/*.json`.
   - `gen_boot_config.py`'s bake of the field.
   - The `config_sync_allowlist.json` entry for `stop_lead_ms`, if one
     exists (verify — this key is part of "schema deletion" per the
     project's config-attic discipline).
   - **The `StateEstimator`'s `bodyAt()` then has no firmware production
     consumer — QUARANTINE it (keep the module, `update()`, and its
     tests; it is the planned consumer for fake-OTOS/fusion bench work).
     Do NOT delete `StateEstimator` or `bodyAt()` itself.**
6. **`test_turn_error_characterization.py` disposition folds into this
   ticket** (per `turn-error-characterization-postcompensation-tests-need-rewrite-after-lead-deletion.md`):
   the `test_postcompensation_*` tests characterize a lead-compensation
   gain-tuning approach that no longer exists once `stop_lead_ms` is
   deleted. Do not just flip the `xfail` back to a plain assertion — the
   whole approach the module validates is being replaced. Likely
   disposition: delete the module (or the parts assuming lead-
   compensation exists); if any of its coverage is still meaningful
   against the land-at-zero predicate, replace it with acceptance
   coverage for what THIS ticket actually ships.
7. **Sequencing satisfied**: this ticket depends on 002 (already landed
   — `remaining` is computed from this-cycle odometry). Ticket 003 (sim
   cadence parity + gate re-baselining) now depends on THIS ticket and
   runs last, so its own re-baseline reflects the final regime (40ms +
   land-at-zero) in one pass rather than two.

## Acceptance Criteria

- [ ] Land-at-zero completion predicate implemented in `MoveQueue::tick()`
      per constraints 1-4 above.
- [ ] `stop_lead_ms` and the anticipation block fully deleted per the
      delete list (constraint 5) — schema, all three robot JSONs,
      `gen_boot_config.py`, `config_sync_allowlist.json` entry, in one
      commit.
- [ ] `StateEstimator`/`bodyAt()` QUARANTINED, not deleted — module,
      `update()`, and its own tests remain; only its `MoveQueue`
      consumer (the anticipation block) is removed.
- [ ] `test_turn_error_characterization.py` disposition resolved per
      constraint 6 (not a bare xfail flip).
- [ ] **No `stop_lead` string survives anywhere in `src/` or `data/`**
      (grep gate).
- [ ] Sim tour-closure gate green at CURRENT (unchanged) bands with
      `stop_lead_ms` deleted — TOUR_1 and TOUR_2, ideal and realistic,
      all pass (this is the exact gate ticket 002's addendum measured
      red; must go green here).
- [ ] The two regressed preset tests from ticket 002's addendum —
      `test_managed_angle_preset[±90]` and
      `test_managed_seg_0_cdeg_turn[±90]` — pass within their existing
      (tightened) ±3.0° band.
- [ ] Isolated 90° twist turn lands within ±2° sim-deterministic.
- [ ] Distance stops (v_x axis) land within current bands.
- [ ] TIME/WHEELS moves byte-identical behavior (regression tests pass
      unchanged).
- [ ] Full `uv run python -m pytest` suite green, EXCEPT any
      cadence-parity item that is explicitly ticket 003's own scope
      (e.g. the five hardcoded-0.05s cadence assumptions, the
      `kCycleDtUs`/throttle-margin work) — those remain ticket 003's
      responsibility and are not blocking for this ticket's own close.
- [ ] Design overlay (`src/firm/app/DESIGN.md` via this sprint's
      `design/DESIGN.md` overlay) updated to describe the land-at-zero
      completion semantics (taper-to-zero + threshold backstop, no
      lead) in place of the "anticipation lead, deleted later" framing
      written before this pull-forward — coordinate with the sprint-
      level overlay edit already made for this amendment.
- [ ] Bench verification is DEFERRED to the phase-B bench session per
      this sprint's stated mandate — not required to close this ticket.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite);
  sim tour-closure gate (TOUR_1 + TOUR_2, ideal + realistic); button-
  acceptance suite (`test_managed_angle_preset`,
  `test_managed_seg_0_cdeg_turn`, and the rest); TIME/WHEELS regression
  tests.
- **New tests to write**: unit coverage for the land-at-zero predicate
  in `MoveQueue::tick()` — Angle and Distance axes, shaping-on and
  shaping-off (backstop-only) regimes, the `ε_ω`-above-deadband-floor
  boundary case. Whatever replaces `test_turn_error_characterization.py`'s
  postcompensation tests per constraint 6.
- **Verification command**: `uv run python -m pytest`, plus
  `grep -rn "stop_lead" src/ data/` (must return nothing), plus the sim
  tour-closure gate and button-acceptance suite runs.
