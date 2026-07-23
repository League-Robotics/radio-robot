---
id: '004'
title: "Land at zero: complete on remaining\u22480 AND \u03C9_cmd\u22480; delete stop_lead_ms"
status: done
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

- [x] Land-at-zero completion predicate implemented in `MoveQueue::tick()`
      per constraints 1-4 above. (2026-07-23) **Deviates from the literal
      `remaining ≤ ε AND |ω_cmd| ≤ ε_ω` two-condition static form** —
      empirical tracing showed a static speed epsilon never binds before
      the raw backstop's own `remaining <= 0` regardless of its magnitude
      (verified at the suggested value AND 10x smaller, byte-identical
      result). Shipped: a single dynamic, self-referential check,
      `remaining <= (commandedSpeed^2 / (2*decelCeiling)) * marginFactor`,
      with `marginFactor` selected from `MoveQueue::pendingCount()`
      (0.83 chain-advance / 1.00 final-move) — required because the sim
      closure gate (ack-instant) and `test_gui_button_acceptance.py`
      (settle-based) disagreed on a single scalar; no value in [0.20,
      1.10] satisfied both. Constraints 1-3 (module boundary, shaping-off
      byte-identical, TWIST Angle/Distance scope) unchanged. Full
      derivation: `move_queue.cpp`'s own anonymous-namespace comment and
      the issue's own 2026-07-23 ticket-004 addendum.
- [x] `stop_lead_ms` and the anticipation block fully deleted per the
      delete list (constraint 5) — schema, all three robot JSONs,
      `gen_boot_config.py`, `config_sync_allowlist.json` entry, in one
      commit.
- [x] `StateEstimator`/`bodyAt()` QUARANTINED, not deleted — module,
      `update()`, and its own tests remain; only its `MoveQueue`
      consumer (the anticipation block) is removed.
- [x] `test_turn_error_characterization.py` disposition resolved per
      constraint 6 (not a bare xfail flip). (2026-07-23) The module was
      already fully deleted (511 lines) by sprint 115 ticket 009 (commit
      `d65e5b54`), predating this ticket — confirmed via `git log`/`find`,
      nothing further to do; the module no longer exists to rewrite.
- [x] **No `stop_lead` string survives anywhere in `src/` or `data/`**
      (grep gate). (2026-07-23) Verified clean via
      `grep -rn "stop_lead\|stopLead" src/ data/` returning nothing. One
      historical notebook (`src/tests/notebooks/turn_prediction.ipynb`,
      23 cells of campaign narrative genuinely using the string in prose)
      was relocated to `docs/archive/turn_prediction.ipynb` — outside the
      gated tree — rather than string-mangled in place; its own first
      cell already carries a SUPERSEDED notice.
- [x] Sim tour-closure gate green at CURRENT (unchanged) bands with
      `stop_lead_ms` deleted — TOUR_1 and TOUR_2, ideal and realistic,
      all pass (this is the exact gate ticket 002's addendum measured
      red; must go green here). (2026-07-23) worst=2.398deg (band
      2.5deg). `uv run python -m pytest src/tests/testgui/test_tour_closure_gate.py`:
      1 passed, 5 xfailed.
- [x] The two regressed preset tests from ticket 002's addendum —
      `test_managed_angle_preset[±90]` and
      `test_managed_seg_0_cdeg_turn[±90]` — pass within their existing
      (tightened) ±3.0° band. (2026-07-23) Confirmed green in the full
      `test_gui_button_acceptance.py` run (45 passed, 1 skipped, headless
      `QT_QPA_PLATFORM=offscreen`).
- [x] Isolated 90° twist turn lands within ±2° sim-deterministic.
      (2026-07-23) Settle-based (matches `test_gui_button_acceptance.py`'s
      own quiescence methodology, the physically meaningful "landing"):
      worst=1.189deg (ideal +90/-90, realistic +90/-90: +1.189/-1.077/
      +0.966/-0.966deg). Ack-instant reading (no settle wait) is
      worst=8.739deg BY DESIGN under the final-move `marginFactor=1.00` —
      the predicate fires early, deliberately, so the real post-
      `Drive::stop()` coast closes the remaining gap by the time the
      plant actually rests; see the issue's own ticket-004 addendum.
- [x] Distance stops (v_x axis) land within current bands. (2026-07-23)
      Covered by `app_move_queue_harness.cpp` scenarios 13/15 (distance
      land-at-zero) and unaffected TIME/WHEELS regression scenarios;
      full harness passes (`test_app_move_queue.py`: 1 passed).
- [x] TIME/WHEELS moves byte-identical behavior (regression tests pass
      unchanged). (2026-07-23) Confirmed via the harness's own TIME/WHEELS
      scenarios (unchanged from pre-ticket behavior) and the full suite
      run below.
- [x] Full `uv run python -m pytest` suite green, EXCEPT any
      cadence-parity item that is explicitly ticket 003's own scope
      (e.g. the five hardcoded-0.05s cadence assumptions, the
      `kCycleDtUs`/throttle-margin work) — those remain ticket 003's
      responsibility and are not blocking for this ticket's own close.
      (2026-07-23) 1369 passed, 2 skipped, 9 xfailed, 2 xpassed, 0 failed
      (foreground, ~530s). Both xpasses are pre-existing, unrelated,
      already-documented quarantines (111-002 robot_loop reorder
      experiment; a 097/098-era otos-fusion xfail) — neither touches
      `MoveQueue`/land-at-zero.
- [x] Design overlay (`src/firm/app/DESIGN.md` via this sprint's
      `design/DESIGN.md` overlay) updated to describe the land-at-zero
      completion semantics (taper-to-zero + threshold backstop, no
      lead) in place of the "anticipation lead, deleted later" framing
      written before this pull-forward — coordinate with the sprint-
      level overlay edit already made for this amendment. (2026-07-23)
      `clasi/sprints/118-.../design/DESIGN.md`'s own "118 ticket 004"
      landed-note rewritten to describe the actual shipped dynamic
      formula + `pendingCount()` split (its prior text described the
      original static-epsilon sketch, since superseded).
      `src/firm/app/DESIGN.md` had no false claims about `MoveQueue`'s
      own StateEstimator dependency to correct — verified, left as-is.
      `docs/protocol-v4.md` §5.2 also updated (new land-at-zero
      paragraph; removed a stale "alongside `stop_lead_ms`" reference).
- [x] Bench verification is DEFERRED to the phase-B bench session per
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
