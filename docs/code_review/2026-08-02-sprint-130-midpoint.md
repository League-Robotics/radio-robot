# Sprint 130 Mid-Sprint Review — Consolidated

**Date:** 2026-08-02 · **Scope:** the sprint-130 diff (`45cd56df..b7ed4663`,
tickets 001–007 done, 008 in progress, 009–012 open) · three parallel
review passes: (A) firmware base + composition roots, (B) planner honesty
pass, (C) host/config chain + bench evidence. Standard:
`docs/code_review/GUIDELINES.md`. Requested focus (stakeholder): hard-coded
config, config sprawl, non-cohesive/wrong-place code, hack residue, and
whether the target bugs are genuinely getting fixed.

## Verdict

**Architecturally on track; behaviorally half-delivered; four items to fix
before the remaining gates run.** The sprint's three structural bets all
landed real: `main.cpp` is a thin 177-line shell and `boot_wiring.cpp` did
NOT become the new literal block (every value flows JSON → gen_boot_config
→ boot_config → installers); the unified controller in Drive is cohesive
and legibly staged (A/B/C separately named and separately unit-tested); the
duty-stage deletion is decisive, with callers walked outward loudly — the
prior review's core complaint was heard. The code reads as design-shaped,
not hack-shaped. What keeps this from a clean bill: one MAJOR behavioral
coupling that silently defeats Stage C for exactly the planner-writer class
the sprint exists to serve, adaptation bounds shipping live under a
provenance note that says they don't, the 40→50 ms cutover missing three
live bench scripts (including the script ticket 011's own gate runs), and a
config surface that is currently net **+11 keys / −0** until ticket 009
lands.

## Top findings (all three passes, severity-ranked)

| # | Sev | Finding | Where |
|---|---|---|---|
| 1 | MAJOR | Every accepted MOVE calls `drive_.estop()` (pre-130 takeover idiom), which 130-004 extended to reset Stage C bias + PID integrators — streamed Moves (goto-mode, +500, chained tours) can never converge the bias; the wheels-solid contract is defeated for the planner writer | `robot_loop.cpp:227`, `drive.cpp:43-50` |
| 2 | MAJOR | `wheel_v_min: 99.7` / `wheel_bias_max: 23.8` ship LIVE (the floor actively rounds every sub-100 mm/s command up, incl. planner decel tapers) while the same file's `_drive_calibration_note` declares those numbers "report-only, NOT applied", n=3, LOW CONFIDENCE | `data/robots/tovez.json`, `drive.cpp` `applySpeedFloor()` |
| 3 | MAJOR | 40→50 ms cutover missed three host bench scripts that hard-code `0.04` with comments claiming it equals `kCycle` — all cycle math off 25%; `square_tour.py` is the vehicle for ticket 011's bench gate | `square_tour.py:105`, `curve_stream.py:79`, `turn_prediction_capture.py:94` |
| 4 | MAJOR | `applySpeedFloor()` and `updateDeficit()` (both live/latent policies) have zero unit tests, despite the +422-line harness investment covering A/B/C | `app_drive_harness.cpp` (absent scenarios) |
| 5 | MAJOR | `duty_per_speed` is STILL three numbers for one quantity: firmware bakes 0.001182, JSONs/schema/generator require 0.00187 (58% off, still fed to the sim via `sim_configure_drive()`), SimHarness defaults 0.002; `sim_harness.h:117-121`'s "same values as hardware" comment is false | `drive.h:138`, `sim_loop.py:716-719`, `sim_harness.h:130` |
| 6 | MAJOR | Committed `planner_types.h:137-149` still documents deleted WheelTrim as "the closed loop that actually reaches the wheels" — in the honesty-pass sprint | `planner_types.h` |
| 7 | MAJOR (small blast) | Swapped `snprintf` args (`%s` ← double) in the parity harness's failure diagnostic — UB exactly when parity fails and tries to explain itself | `composition_root_parity_harness.cpp:52-57` |
| 8 | MINOR | `kDutyPerSpeed`'s provenance comment tells the superseded 853.6 story and cites superseded issue 04; the sprint's own newer data disputes the value by ~23% (left) with an unexplained ~20-25% session plant-gain drop on top | `drive.h:120-137` |
| 9 | MINOR | Parity test claims "drive-calibration" parity but checks only PlannerLimits; `WheelControllerBootConfig` and the sim's `setDutyPerSpeed()` override are invisible to it | parity harness header |
| 10 | MINOR | Stage B silently dead unless Stage C's `aSteady` is nonzero (cross-stage gate documented nowhere); Stage B P-term not freshness-gated | `drive.cpp:301-302`, `fastPid()` |
| 11 | MINOR | Stale-comment cluster: three pointers into deleted `wheel_trim.h`, "47 ms" claims in `main.cpp:80`/`boot_wiring.h:36`/`boot_config.h:272`, `ratio_lock_test.cpp:169` citing the deleted test, `test_sim_wire_loopback.py:322` tolerance justified by the deleted trim, tovez `_settle_note`/`_wheel_controller_note` self-contradictions | various |
| 12 | MINOR | `plannerTrim()` FFI symbol contains no trim; rename inside ticket 009's ABI break | `capi.cpp:56` |
| 13 | MINOR | No test covers `gen_boot_config.py::wheel_controller_config_for_config()` (fail-closed path or emitted C++); `population_spread([])` ZeroDivisionError; `_load_bench_module()` duplicated-and-diverged across two test files | host tests |
| 14 | NOTE | `pid.kff`/`pid.kaw` wire keys semantically repurposed (now kaff [s] / pidMax [mm/s]) — consistently plumbed, but a live-tuning trap; real keys deserve filing for the next wire rev | `configurator.cpp`, `push.py:184-188` |
| 15 | NOTE | Ticket 009's 34→23 target is off by one: 12 fields verified dead ⇒ 22 remain; reconcile before executing | dead-field inventory in pass B |
| 16 | NOTE | Ticket 007 sits in done/ with every acceptance box unchecked and no completion notes (work verifiably done; artifact hygiene) | 007 ticket file |
| 17 | NOTE | Config sprawl accounting: +11 keys, 0 removed; 7 of 11 are 0.0 on every robot (Stage B, deficit); `tauAdapt`/`aSteady` are design constants wearing config clothing; all removals parked in 009 | JSONs/schema |

## Evidence check — are the target bugs fixed?

- **Circular calibration (SUC-002): structurally dead.** Baked constant +
  identity corrections + constant-free saturation cross-check + report-only
  candidate values. Durable.
- **"Wheel holds its commanded speed" (SUC-001): proven only in steady
  state.** The 90 s bias-convergence probe is real committed evidence
  (bias → +14/−4, velocity within ~5 mm/s by t≈15-20 s). But: cold starts
  run 20-30% slow; the +500 transient criteria FAIL at shipped Stage-B-zero
  gains (rise 0.59 s vs 0.3, ripple 24 vs 10, split 21 vs 10);
  WHEELS-under-drag was never measured (no rig); and finding #1 means the
  planner writer currently gets a bias reset on every enqueue.
- **Population grounding (SUC-002): incomplete by hardware fault.** Left
  wheel fully characterized; right wheel n=3; simultaneous grid zero rows;
  power-ceiling verdict honestly deferred. The parallel-lines result so far
  CONTRADICTS the intercept-only hypothesis Stage C is built on (low
  confidence, correctly flagged, unresolved).
- **One composition root (SUC-003): real.** One `composeRobot()`,
  `boot_config.cpp` linked host-side, one period constant, a genuine
  divergence-tripwire parity test, `simPlannerLimits()` gone. Gaps: the
  parity check's scope (finding 9) and the duty_per_speed triple (finding 5).
- **Planner honesty (SUC-004): substantially honest**, pending 008/009 —
  duty stage fully gone, `pid.*` keys verifiably reach Drive, DESIGN.md
  tells the truth; blemishes are findings 3 and 6.
- **Bookkeeping honesty: exemplary.** Unchecked boxes with reasons, FAILs
  reported as FAILs, a wrong first conclusion corrected in the same notes,
  the plant-gain drop flagged without a battery narrative.

## Recommended sequencing before sprint close

1. **Fix finding 1 (takeover-vs-panic verb split) before ticket 012's
   playfield gate** — goto-mode streaming is its exact regime. Design
   conversation: takeover should disarm WHEELS + zero targets; the Stage
   B/C reset belongs to real estop only.
2. **Fix finding 3 before ticket 011 runs** — the bench re-verification
   gate would otherwise score against 25%-wrong cycle math in its own
   vehicle script.
3. Fold into 008/009 (already open): findings 6, 12, 15, the
   duty_per_speed_* removal lockstep (finding 5), and the stale-47 ms
   comment sweep (finding 11).
4. Quick standalone fixes: swapped snprintf (7), floor/deficit harness
   scenarios (4), provenance reconciliation on vMin/biasMax (2), ticket
   007 checkboxes (16).
5. Stakeholder bench session (already flagged in the issue pool): complete
   the population sweep (right wheel + simultaneous grid + fresh battery),
   re-run saturation to resolve the ~25% plant-gain drop, then a real
   Stage B gain sweep against the +500 transient spec.

## What's good (consolidated)

- `boot_wiring.cpp`/`main.cpp`: the feared "new literal block" did not
  materialize; `BootOverrides` is a model explicit-divergence seam;
  `RobotGraph` is textbook composition-root C++.
- Drive's Stage A/B/C: separately named, separately tested, bumpless
  transfer and anti-windup pinned by the harness; estop armor survived the
  churn (write-throttle correctly re-derived 35→45 ms with kCycle).
- The deletion discipline: zero stale includes, tombstones instead of
  residue, `hil_drive.py --duty` rejected loudly, no silent zeros.
- Config lockstep executed correctly across all four surfaces, fail-closed
  end to end; no test code leaked into `src/host`; naming/units clean in
  all new files.
- `duty_sweep.py`'s abort-keeps-rows redesign and report-only candidate
  values; both new unit-test files are substantive (24 real tests), not
  import-smoke.
