# Issue triage, 2026-08-02: 25 open → 10

**Tree:** `master` @ `a5f7b06b` (v0.20260802.2, sprint 130 merged and closed)
**Requested by:** stakeholder — "review all the open issues and figure out which
are still valid and open... be aggressive... anything we could probably move out
of the mainstream, let's do it."

Every disposition below was checked against the tree, not taken from the issue
text. Nothing was deleted; closed issues moved to `clasi/issues/done/`, deferred
issues to `clasi/issues/later/`. All of it is reversible in git.

## The finding that drove the triage

`docs/code_review/2026-08-02-post-130-wheels-solid-review.md` — written the same
day by six parallel review agents and spot-verified by the team-lead — had
already independently re-verified most of this list against the tree. Several
issues turned out to be one bullet inside a better-analyzed finding, and two
(the +500 spec, the ESTOP bound) were *answered* by it rather than merely
restated. The review is now the master analysis; the issue pool is the subset of
it that is ready to become tickets.

## Closed (11)

| Issue | Why |
|---|---|
| `systest-firmware-changes-dbg-inbound-...` | The work **landed** at `d08264dc`. Verified: `src/tests/system/` with `systest.py`/`recorder.py`/`goldens.py` and `fault_wedge.tour` all present. Issue self-described as "now the review record"; the review's Part 5 covered it. |
| `otos-frozen-at-a-constant-on-tovez` | The issue's own 2026-08-02 correction says it was **not a fault** — every observation was on the stand, where a ground-tracking optical sensor reading constant is correct. The residual liveness-flag ask lives in review F6. |
| `misc-changes-from-2026-07-31-session` | Items 4 (`wheel_gain_*` keys) and 5 (`sim_configure_drivetrain`) verified **landed**. Items 1, 2, 3 are now recorded in review H6 and in the still-open rotation-calibration issue. |
| `host-side-ack-drop-fault-injection-...` | Self-declared "low priority, explicitly not blocking anything." The condition it would test became unreachable by design when the IRQ guard was removed. `DBG drop <n>` is a cheap add if it is ever wanted — already a parked open question in the systest issue. |
| `sensors-subsystem-owns-line-color-tick-flipflop` | Merged into the line-sensor issue. Whether the line sensor exists at all determines what the Sensors subsystem paces — one decision, not two. |
| `surface-i2c-error-counts-through-the-bus-interface` | One bullet inside review F6's observability package (which also covers `ConfigSnapshot`, setpoint-on-wire, deficit decoupling, bits 10/12, probeSlot). Will be done as part of it or not at all. |
| `plus500-transient-criteria-and-plant-gain-drift-followup` | **Answered, not deferred.** Review Part 6 shows two of the three failing bounds (ripple ≤10 mm/s, split ≤10 mm/s) sit *below the actuator floor* — 1 duty LSB ≈ 8–11 mm/s. The issue's recommendation (run a Stage B gain sweep) would chase a bound the hardware cannot hold. This is a spec decision, and the one Stage B trial making ripple *worse* (33.7) is consistent with a quantizer limit cycle. The gain-drift half is review F2. |
| `bench-reverify-residuals-and-the-4ms-...` | Review F7 covers the period comprehensively and structurally (the four `runAndWait` gaps sum to `kCycle` but real work runs *between* the marks, plus per-block whole-ms round-up). The unverified bench criteria moved to the bench-session checklist. |
| `tovez-hard-silent-i2c-wedge-...` | → consolidated into `A-next-physical-bench-session-checklist` (item 1, the gate). |
| `hitl-confirm-wheel-frozen-flag-...` | → consolidated (item 2). |
| `playfield-actuation-floor-measurement-...` | → consolidated (item 4). |

## Moved to `later/` (7)

Deferred, not judged wrong. Each would be rediscovered quickly if it mattered.

| Issue | Why out of the mainstream |
|---|---|
| `path-following-hardware-gaps` | Sprint-127 era (curvature feed-forward, cloverleaf streaming). Off the current G1/G2 road, and `curve_stream.py` is on the systest retirement list anyway. |
| `testgui-decompose-main-into-controller-classes` | ~2,935-line `_build_main_window()`, confirmed not started (review H3). A large refactor with no bearing on firmware-solid or characterization. The STOP path it was sequenced behind is already fixed and acceptance-tested. |
| `planner-profile-py-dormancy-candidate` | Dormancy re-confirmed by the review. It is a one-line delete/keep decision that belongs in a cruft sweep, not a standing issue. |
| `estop-settle-time-floor-...` | Review Part 1 already re-derives it: drained in block 3, first duty write next cycle, ≈54 ms + reissue + spin-down ≈ the measured 0.19 s. The issue's own arithmetic still says 40 ms — stale in the wrong direction. |
| `radio-bench-gate-fault-latch-...` | A gate-semantics decision (budgeted count vs latch). Rediscovered the instant anyone runs the gate — it fails 31/35 today for this reason. |
| `system-test-square-tour-...` (the charter) | Its operative ask — record the no-new-system-tests rule where it will be read — is **already done** in `src/tests/system/CLAUDE.md`. Kept in `later/` as the readable reference for the three-tier definition, golden-trace process rules, and deletion list. *(Flagging explicitly: this is a stakeholder directive being moved, not discarded. Its mechanism issue stays active.)* |
| `system-test-tests-outside-...-taxonomy-and-tiers` | The keep-list, the `src/tools/` relocation, and the deletion sweep. `src/tools/` does not exist yet; the sweep is not the current priority and the lists stay valid whenever it becomes one. |

## Created (2)

| Issue | Why |
|---|---|
| `A-speed-floor-snaps-the-planner-differential` | **Referenced by three documents and never created** — the post-mortem, the knowledge doc, and the actuation-floor issue all cite a file a repo- and history-wide search does not find. Review F12 calls creating it "the highest-value single file in this review." Mechanism verified at `drive.cpp:150-156,277-278`: the floor runs on each wheel's already-summed `cmdVelocity`, so it cannot distinguish a 3 mm/s differential trim from a 3 mm/s travel command. |
| `A-next-physical-bench-session-checklist` | Consolidates the three issues that all waited on the same thing — a person at the bench — plus the two sprint-130 criteria never re-measured because nobody was there to power-cycle a wedged robot. They were four files describing one session. |

**Not created, contrary to review F12:** `make-irq-guard-off-permanent-and-reconcile-the-docs.md`.
The review flagged it as a second missing file, but the guard is not merely
"off" — `setIrqGuard()`/`irqGuard()` were **deleted entirely** in 128-013, with
the rationale and the ~0% measured inbound loss recorded in
`src/firm/devices/DESIGN.md:222-235` and a removal note in `i2c_bus.h:9-11`.
Its own tracking issue was resolved by stakeholder decision at `79ca4d26`.
Nothing remains to make permanent. The stale references in older docs
(`docs/architecture/*`, `docs/post-mortem/*`) are ordinary doc rot, not a
tracked decision.

## Still open (9)

All re-verified unfixed in the tree on 2026-08-02.

| Issue | Verification |
|---|---|
| `A-rebuild-nezha-facade-on-the-v5-binary-surface` | Review F11: ~29 of ~36 public methods call deleted protocol methods; 11 `_proto._conn` reach-arounds confirmed present; three pose owners with disagreeing unit conversions. Blocks every bench session. |
| `B-rewrite-io-calibrate-on-the-v5-binary-surface` | Review H4: six deleted protocol calls confirmed at `calibrate.py:401-421,747-854`; `calibration/linear.py`/`angular.py` fail *silently* at the wire. Needs the fold-or-delete decision. |
| `B-vision-config-robot-tag-id-must-fail-closed` | Confirmed: `robot_config.py:77` is still `robot_tag_id: int = 1`, with hardcoded `100` fallbacks at `cli.py:166,645,747` and `calibrate.py:315,651`. Small, real, nasty failure mode. |
| `C-sensors-line-sensor-dead-and-perception-pacing-owner` | Confirmed: `cycleCount_` declared `robot_loop.h:201`, tested `robot_loop.cpp:544`, incremented nowhere. Merged with the flip-flop-ownership issue. |
| `A-tour2-146-degree-turn-still-undershoots-after-130-010` | Updated with review F10's specific hypothesis: `decelLatched` is a one-way trap whose failure is additive and angle-independent — matching the residual's signature. Now a testable claim with a named fingerprint. |
| `B-rotation-calibration-vs-live-heading-hold-gain` | Live: `tovez_nocal.json` still carries the sim-fitted `rotation_gain` 1.006 / offset 12.1, and `heading_hold_gain` is still 2.0 there and in `togov.json` (zeroed only in `tovez.json`). Two tests xfail on it. |
| `B-system-test-minimal-remaining-phases` | Trimmed to the unlanded phases (2, 4, 5, 6). Phases 1 and 3 verified landed. |
| `A-next-physical-bench-session-checklist` | New. Gates most hardware work. |
| `A-speed-floor-snaps-the-planner-differential` | New. |

## What this does not cover

The review's twelve ranked findings are mostly **not** in the issue pool, by
design — F1 (position rebaseline destroys the pose), F2 (adapt the slope, not
the intercept), F3 (the sim's error knobs bias belief, not motion), F5 (every
MOVE wipes the controller's learned state), F6, F7, F8, F9 are all
sprint-shaped, not issue-shaped. They should be planned straight off the review
per its own Part 8 sequencing, not decomposed into issue files first. The pool
above is the residue that is genuinely independent of that planning.
