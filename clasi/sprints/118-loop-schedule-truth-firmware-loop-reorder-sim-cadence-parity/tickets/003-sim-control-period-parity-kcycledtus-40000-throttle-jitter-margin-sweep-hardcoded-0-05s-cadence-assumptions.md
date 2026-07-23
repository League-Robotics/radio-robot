---
id: '003'
title: 'Sim control period parity: kCycleDtUs=40000, throttle jitter margin, sweep
  hardcoded 0.05s cadence assumptions'
status: in-progress
use-cases:
- SUC-065
depends-on:
- '001'
- '002'
- '004'
github-issue: ''
issue: sim-cycle-must-match-firmware-period.md
completes_issue: true
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

- [ ] `SimHarness::kCycleDtUs == 40000`.
- [ ] Sim step period and firmware `kCycle` come from/assert against the
      same value — an assert or test enforces this invariant, not two
      independently-hardcoded matching numbers that can drift apart
      again silently.
- [ ] `NezhaMotor`'s write-rate-throttle jitter margin added; reasoned
      through in code review this sprint (sim cannot exercise real
      jitter); bench verification (fault/skip counter or encoder
      smoothness while driving) DEFERRED to phase-B.
- [ ] All five verified hardcoded 0.05s/50ms cadence assumptions fixed:
      `app_robot_loop_harness.cpp` (×2: `kCycleDtUs`,
      `plant.tick(0.05f)`), `turn_prediction_capture.py`'s `_CYCLE_S`,
      `test_tour_closure_gate.py`'s `clock.now_s += 0.05`,
      `sim_loop.py`'s `_CYCLE_DURATION_S`.
- [ ] Grep gate: no surviving hardcoded 0.05/50ms cycle assumption
      anywhere in the tree.
- [ ] Sim tour-closure gate, button-acceptance suite, and estimator
      tracking harness re-run and green at 40ms + land-at-zero (the
      final regime — `stop_lead_ms` is already deleted by ticket 004 by
      the time this ticket runs); per-leg bands unchanged or tightened —
      never silently widened. The GUI-path tour band (≤5°) may tighten
      if the final-regime data supports it; record the measured numbers
      here either way.
- [ ] No re-verification step in this ticket depends on `stop_lead_ms`
      existing — confirm the grep gate from ticket 004
      (`grep -rn "stop_lead" src/ data/`) still returns nothing after
      this ticket's own changes (this ticket must not reintroduce it).
- [ ] `src/sim/DESIGN.md`'s own kCycleDtUs-mismatch Open Question (§8)
      updated to a resolved-parity statement — this is the ONE subsystem
      DESIGN.md this sprint edits directly on its canonical path (not
      through the overlay — see sprint.md's Design Overlay section for
      why `src/sim/DESIGN.md` couldn't share the overlay's single
      `DESIGN.md` slot with `src/firm/app/DESIGN.md`). Also correct §2/§6
      language describing "not every sim_step() call, currently 50ms" and
      the "2.5× step-size difference" framing, which becomes exact parity
      once this ticket lands.
- [ ] Full `uv run python -m pytest` suite green; sim/firmware build
      green.
- [ ] Bench verification (measured TLM period ≈40ms, no duty-write drops
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
