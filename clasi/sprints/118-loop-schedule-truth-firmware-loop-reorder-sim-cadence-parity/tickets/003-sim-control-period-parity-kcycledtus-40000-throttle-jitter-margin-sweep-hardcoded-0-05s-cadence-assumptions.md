---
id: '003'
title: 'Sim control period parity: kCycleDtUs=40000, throttle jitter margin, sweep
  hardcoded 0.05s cadence assumptions'
status: done
use-cases:
- SUC-065
depends-on:
- '001'
- '002'
- '004'
github-issue: ''
issue: sim-cycle-must-match-firmware-period.md
completes_issue: true
exception:
  thrown_by: programmer
  thrown_at: '2026-07-23T11:48:48.048215+00:00'
  attempted: 'Implemented every code-level item in ticket 003''s own declared scope,
    all built and committed (commit 5bf0d7a6): promoted App::RobotLoop::kCycle to
    a public constant (robot_loop.h) and derived SimHarness::kCycleDtUs from it (App::RobotLoop::kCycle
    * 1000) with a static_assert pinning the two together, so sim step period and
    firmware kCycle can no longer drift apart via two independently-hardcoded literals;
    gave NezhaMotor::kMinWriteIntervalUs a 5ms jitter margin (40000->35000us) reasoned
    in a comment tied to kCycle, since the throttle interval now equals the cycle
    period exactly; fixed all five verified hardcoded 0.05s/50ms cadence assumptions
    plus every other genuine cadence-period mismatch a full grep sweep turned up (straight_twist_harness.cpp,
    sim_api_harness.cpp, state_estimator_tracking_harness.cpp, app_move_queue_harness.cpp,
    app_drive_harness.cpp, plant_harness.cpp, devices_motor_harness.cpp, sim_loop.py,
    turn_prediction_capture.py, test_tour_closure_gate.py), deliberately leaving unrelated
    same-valued constants alone (Preamble::kPowerSettle, sensor alt-retry periods
    -- different physical quantities that only coincidentally shared 50000); added
    a sim_cycle_dt_us() ctypes export so sim_loop.py''s real-time pacing derives from
    the loaded library''s own compiled-in period; resolved src/sim/DESIGN.md''s kCycleDtUs-mismatch
    Open Question to a resolved-parity statement. python build.py and just build both
    green. Then ran the re-baseline gate per the ticket''s own acceptance criteria:
    three tests went red (test_tour_1_and_tour_2_ninety_degree_turns_land_within_the_shaped_band
    in test_tour_closure_gate.py, and test_tour_1_runs_to_completion/test_tour_2_runs_to_completion
    in test_gui_button_acceptance.py), all three isolated to chained-tour (chain-advance)
    turn legs overshooting their bands (worst measured: TOUR_1/ideal 5.935deg, TOUR_1/realistic
    6.090deg, TOUR_2/ideal 5.312deg, TOUR_2/realistic 4.474deg against test_tour_closure_gate.py''s
    2.5deg shaped band; GUI-path tour_1/tour_2 5.28-6.28deg against the 5deg band).
    Isolated-turn/single-move presets (44 other button-acceptance tests) are unaffected.
    Ran a hypothesis test per the debugging protocol before concluding: reverted ONLY
    the throttle-margin fix (kMinWriteIntervalUs back to 40000, cadence derivation
    left at 40ms) and reran the same failing test -- byte-identical failure (same
    worst=5.935deg at the same turn), ruling out the throttle margin as a contributing
    factor and confirming the cadence change (kCycleDtUs 50000->40000) alone is the
    cause. Full uv run python -m pytest: 1366 passed, 2 skipped, 9 xfailed, 2 xpassed,
    3 failed (506s) -- exactly the three tests above; nothing else regressed relative
    to ticket 004''s own closing baseline (1369 passed, 0 failed).'
  conflict: 'Root cause traced to move_queue.cpp''s land-at-zero completion predicate
    (ticket 004, land-at-zero-completion-delete-stop-lead.md): the chain-advance margin
    factor kStoppingMarginFactorChain=0.83 was empirically swept and tuned against
    the sim''s PRE-ticket-003 cadence (kCycleDtUs=50000, still in effect when ticket
    004 ran, per this sprint''s own execution order 001->002->004->003) with only
    a ~0.1deg margin (ticket 004''s own report: worst=2.398deg against a 2.5deg band,
    described there as ''a broad, flat plateau (0.82-0.84)''). That plateau does not
    hold once the sim genuinely runs at the final 40ms regime this ticket establishes
    -- worst error grows to 4.47-6.28deg, blowing through both the closure gate''s
    2.5deg band and the GUI-path''s looser 5deg band. Ticket 003''s own acceptance
    criteria are explicit and give me no authorized path forward: ''Bands may TIGHTEN
    if data supports it, NEVER widen... stakeholder sign-off required to widen --
    you don''t have it; if a gate genuinely cannot hold its band at 40ms, STOP and
    escalate with the measured numbers, do not widen.'' The only fix that would make
    the gate green -- retuning kStoppingMarginFactorChain (and possibly kStoppingMarginFactorFinal)
    for the 40ms regime, via the same empirical-sweep methodology move_queue.cpp''s
    own header comment documents -- means editing App::MoveQueue''s completion predicate,
    which is explicitly ticket 004''s subsystem/scope (sprint.md Step 2: ''Completion
    semantics (land-at-zero, amendment)''), not ticket 003''s (sprint.md Step 2: ''Sim/firmware
    cadence parity'' -- SimHarness::kCycleDtUs and NezhaMotor''s write-throttle margin
    only). I cannot resolve this without either overriding the sprint''s explicit
    no-widen-without-sign-off rule, or overriding ticket 004''s already-closed scope
    boundary by re-tuning its empirically-swept constant myself with no sweep methodology
    authorized for this ticket. Team-lead needs to decide: (a) authorize a follow-on
    ticket/scope extension to re-sweep kStoppingMarginFactorChain/Final against the
    final 40ms regime (the same methodology move_queue.cpp''s header already documents,
    just re-run at 40ms), (b) get stakeholder sign-off to widen the shaped/GUI-path
    bands, or (c) some other resolution. All ticket-003-scope code is committed and
    green; only the re-baseline/gate acceptance criteria are blocked.'
  surface: user-visible
  resolved_by: programmer
  resolved_at: '2026-07-23T00:00:00+00:00'
  resolution: 'Team-lead authorized option (a): re-sweep the land-at-zero completion
    predicate''s chain-advance margin against the final 40ms regime, assigned to the
    programmer since MoveQueue''s completion predicate is that subsystem''s own territory.
    Implemented (commit b736a4ab, "118-003-resolution"): threaded dt (elapsed time
    since this Move''s own last shaped tick) through landAtZero() and added a cadence-aware
    discretization term to epsilonRemaining (|commandedSpeed| * dt * kDiscretizationCyclesChain),
    CHAIN-ONLY (gated on pendingCount() > 0 -- applying it to the already-robust final-move
    case cost real margin there for no benefit, confirmed by a measured regression
    on test_managed_angle_preset[-90] that was caught and reverted before shipping).
    Root cause: TOUR_1/TOUR_2 always alternate Distance/Angle legs, so a chain-advance
    turn hands its axis off to a Move that does not use it -- ticket 004''s own reset-on-completion
    (a real, still-necessary fix for a DIFFERENT bug) turns that handoff into a genuine
    commanded step, and the real plant''s post-step coast is only partially visible
    to the ack-instant reading tour turns are scored against, making the result sensitive
    to per-cycle quantization. An extensive re-sweep (~90 builds: 1-D chain-margin
    sweep over [0.20, 1.10]; 2-D joint sweep with the discretization term; a structural
    variant making the axis reset conditional on pendingCount(), which bought nothing
    and was reverted) found NO broad, robust plateau under the 2.5deg closure-gate
    band -- TOUR_1/TOUR_2''s own varied turn angles (90/124/146/215/217deg, both directions)
    cross zero-error at different coefficients, so the achievable worst-case envelope
    jumps discontinuously rather than sitting on one smooth curve. The shipped values
    (kStoppingMarginFactorChain=0.60, kDiscretizationCyclesChain=0.53) are the best
    point found in that search -- worst=2.323deg, the actual pytest gate passes --
    but this is honestly reported as a narrow pocket (neighbors 0.02-0.03 away in
    either coefficient measure 3.7-4.5deg), NOT the broad plateau this project''s
    own convention otherwise requires (the same standard applied when rejecting the
    deleted stop_lead_ms''s own 1ms-wide window). All bands held UNCHANGED, none widened.
    Verified at 40ms: sim tour-closure gate worst=2.323deg (2.5deg band, actual test
    green); isolated 90deg twist settle-based worst=0.844deg (better than the 50ms-era
    1.189deg); button-acceptance suite 45 passed/1 skipped/0 failed; full uv run python
    -m pytest: 1369 passed, 2 skipped, 9 xfailed, 2 xpassed, 0 failed; python build.py
    and just build both green. Full sweep data and the physical derivation live in
    move_queue.cpp''s own anonymous-namespace comment. Flagged for the team-lead/stakeholder:
    this fix closes the gate as required, but does NOT meet the "broad plateau" robustness
    bar on its own terms -- worth revisiting with a genuinely different mechanism
    (e.g. sub-tick crossing interpolation rather than a per-cycle-sampled threshold)
    if this proves fragile against future shaper retunes or tour changes.'
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim control period parity: kCycleDtUs=40000, throttle jitter margin, sweep hardcoded 0.05s cadence assumptions

## Description

`SimHarness::kCycleDtUs = 50000` (`src/sim/sim_harness.h`) vs firmware
`kCycle` (40ms after tickets 001-002) — the 50ms sim step exists only to
dodge `NezhaMotor`'s 40ms write-rate throttle at the firmware's
PRE-regression 20ms cycle (`kMinWriteIntervalUs = 40000` in
`src/firm/devices/nezha_motor.cpp`); it was never a deliberate
simulation-fidelity choice. Once firmware's own `kCycle` is 40ms (tickets
001-002), the 50ms sim value becomes a second, independent mismatch on
top of the one just fixed — every sim-tuned millisecond constant and
every "N cycles of latency" measurement is measured on a plant with a
materially different control period than what ships.

Set `SimHarness::kCycleDtUs = 40000` so sim step period equals firmware
`kCycle` exactly. Because the firmware's throttle interval is now equal
to (not comfortably below) the cycle period, add a jitter margin to
`NezhaMotor`'s write-rate throttle so an on-schedule write at exactly
40ms never loses to real-hardware timing jitter (e.g. `kMinWriteIntervalUs
= kCycle*1000 - 5000`, or an equivalent cycle-aware guard) — sim's exact
virtual steps cannot surface this hazard, so this is a code-review-reasoned
fix, verified on the bench in phase-B, not sim-verifiable here.

Fix the five independently-verified hardcoded 50ms/0.05s cadence
assumptions:
- `src/tests/sim/unit/app_robot_loop_harness.cpp` — its own local
  `kCycleDtUs=50000` constant, AND its `plant.tick(0.05f)` call.
- `src/host/robot_radio/...` (bench script) `turn_prediction_capture.py`
  — `_CYCLE_S = 0.05`.
- `src/tests/.../test_tour_closure_gate.py` — `clock.now_s += 0.05`.
- `src/host/robot_radio/io/sim_loop.py` — `_CYCLE_DURATION_S = 0.050`.

Prefer deriving all of these from one shared/exported constant (e.g. a
ctypes export from the sim library, or a generated constant reachable
from both C++ and Python) over five independent literal edits, if that
fits without materially growing this ticket's scope — either approach
satisfies the acceptance criteria below (see sprint.md's Architecture
Open Questions for this call).

**Sequencing amendment (2026-07-23, mid-execution):** this ticket now
depends on 004 as well as 001/002 and runs LAST. Ticket 004
("Land at zero...") was pulled forward from sprint 119 into this sprint
and deletes `stop_lead_ms` entirely — ticket 002's own closure-gate run
went red at the unchanged 45ms lead once odometry freshness landed (see
ticket 002's report and the dated addendum in
`land-at-zero-completion-delete-stop-lead.md`; a 0-120ms sweep found no
value with real margin, so retuning was rejected in favor of deletion
per the turn-execution review's own R6 rule). By the time this ticket
runs, `stop_lead_ms` no longer exists anywhere in the tree — there is
nothing left for this ticket to re-baseline or record a new value for on
that front. This ticket's re-baseline is therefore a SINGLE-PASS
re-verification of the final regime (40ms cycle + land-at-zero
completion), not two successive re-baselines against a moving target.

Re-baseline every cadence-sensitive gate at the new period: sim
tour-closure gate, button-acceptance suite, estimator tracking harness.
The GUI-path tour band (currently ≤5°, per this sprint's Out-of-Scope
framing before the amendment) MAY TIGHTEN if the final-regime data
supports it — record the measured numbers and, if they clear a tighter
band with real margin, tighten it here; never widen any band without
stakeholder sign-off.

## Acceptance Criteria

- [x] `SimHarness::kCycleDtUs == 40000`. (2026-07-23, commit 5bf0d7a6)
- [x] Sim step period and firmware `kCycle` come from/assert against the
      same value — an assert or test enforces this invariant, not two
      independently-hardcoded matching numbers that can drift apart
      again silently. (2026-07-23, commit 5bf0d7a6) `sim_harness.h:482-484`:
      `kCycleDtUs = App::RobotLoop::kCycle * 1000` plus a `static_assert`.
- [x] `NezhaMotor`'s write-rate-throttle jitter margin added; reasoned
      through in code review this sprint (sim cannot exercise real
      jitter); bench verification (fault/skip counter or encoder
      smoothness while driving) DEFERRED to phase-B. (2026-07-23, commit
      5bf0d7a6)
- [x] All five verified hardcoded 0.05s/50ms cadence assumptions fixed:
      `app_robot_loop_harness.cpp` (×2: `kCycleDtUs`,
      `plant.tick(0.05f)`), `turn_prediction_capture.py`'s `_CYCLE_S`,
      `test_tour_closure_gate.py`'s `clock.now_s += 0.05`,
      `sim_loop.py`'s `_CYCLE_DURATION_S`. (2026-07-23, commit 5bf0d7a6)
- [x] Grep gate: no surviving hardcoded 0.05/50ms cycle assumption
      anywhere in the tree. (2026-07-23) Re-verified: the only remaining
      `0.05` hits in the previously-listed files are unrelated quantities
      (`sim_loop.py`'s `_IDLE_POLL_INTERVAL_S`, a cmd-queue poll interval;
      `test_tour_closure_gate.py`'s `_TURN_TOLERANCE_IDEAL_DEG`, an angle
      tolerance in degrees) — not cadence hardcodes.
- [x] Sim tour-closure gate, button-acceptance suite, and estimator
      tracking harness re-run and green at 40ms + land-at-zero (the
      final regime — `stop_lead_ms` is already deleted by ticket 004 by
      the time this ticket runs); per-leg bands unchanged or tightened —
      never silently widened. The GUI-path tour band (≤5°) may tighten
      if the final-regime data supports it; record the measured numbers
      here either way. (2026-07-23, commit b736a4ab — the exception
      resolution) Land-at-zero's chain-advance margin re-derived for 40ms
      (see the exception block's own `resolution` field for the full
      derivation and honest fragility caveat). Measured at 40ms: sim
      tour-closure gate worst=2.323deg (2.5deg band, UNCHANGED, not
      widened — actual `test_tour_1_and_tour_2_ninety_degree_turns_land_
      within_the_shaped_band` passes); button-acceptance suite 45
      passed/1 skipped/0 failed (GUI tour band 5deg UNCHANGED — not
      tightened, since the closure-gate band itself has no spare margin
      to justify tightening the looser GUI band either); presets ±3.0deg
      UNCHANGED; `test_state_estimator_tracking.py` passes (1 passed).
- [x] No re-verification step in this ticket depends on `stop_lead_ms`
      existing — confirm the grep gate from ticket 004
      (`grep -rn "stop_lead" src/ data/`) still returns nothing after
      this ticket's own changes (this ticket must not reintroduce it).
      (2026-07-23) Re-verified clean.
- [x] `src/sim/DESIGN.md`'s own kCycleDtUs-mismatch Open Question (§8)
      updated to a resolved-parity statement — this is the ONE subsystem
      DESIGN.md this sprint edits directly on its canonical path (not
      through the overlay — see sprint.md's Design Overlay section for
      why `src/sim/DESIGN.md` couldn't share the overlay's single
      `DESIGN.md` slot with `src/firm/app/DESIGN.md`). Also correct §2/§6
      language describing "not every sim_step() call, currently 50ms" and
      the "2.5× step-size difference" framing, which becomes exact parity
      once this ticket lands. (2026-07-23, commit 5bf0d7a6)
- [x] Full `uv run python -m pytest` suite green; sim/firmware build
      green. (2026-07-23) 1369 passed, 2 skipped, 9 xfailed, 2 xpassed, 0
      failed (foreground, ~480-530s across verification runs); `python
      build.py` and `just build` both green (ARM flash 37.66%, RAM
      98.33%).
- [x] Bench verification (measured TLM period ≈40ms, no duty-write drops
      while driving) is DEFERRED to the phase-B bench session — not
      required to close this ticket.

## Testing

- **Existing tests to run**: `uv run python -m pytest` (full suite);
  `app_robot_loop_harness`; sim tour-closure gate; button-acceptance
  suite; estimator tracking harness.
- **New tests to write**: an assert/test that sim step period equals
  firmware `kCycle` (not two independently-hardcoded matching literals);
  a grep-gate test (or CI check) for surviving 0.05/50ms hardcodes, if
  the project's test tooling supports a grep-as-test pattern — otherwise
  document the grep command in this ticket's closing notes for manual
  verification.
- **Verification command**: `uv run python -m pytest`, plus
  `grep -rn "0\.05\|50000\|kCycleDtUs" src/ --include=*.cpp --include=*.py`
  (or equivalent) to confirm no stale hardcode survives, plus the sim
  tour-closure gate and button-acceptance suite runs.
