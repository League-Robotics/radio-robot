# Exactness Review — system state after sprint 120, and what closes the goal

**Reviewer:** Claude (team-lead session, stakeholder-directed)
**Date:** 2026-07-23 · **Tree:** v0.20260723.3 (sprints 118/119/120 merged)
**Companion:** [`docs/design/goal-exact-tours.md`](../design/goal-exact-tours.md) — the goal
this review validates against. Prior review: `2026-07-22-turn-execution-review.md`.
**Method:** all sim numbers re-measured on this merge (deterministic sim, ideal chip,
`tovez_nocal`); bench numbers from sprint 120's own ticket records; two independent
code-survey passes (comment/naming, design-doc audit) run against HEAD.

---

## 1. Verdict

**The sim is no longer mysteriously wrong — it is precisely, reproducibly wrong in
exactly one place, and that place is scheduled.** Three identical runs today produced
identical per-leg errors to the hundredth of a degree. 100% of the remaining sim error
enters at the chain-advance completion instant; nothing else contributes above the
0.01° noise floor. Sprints 121–123 as planned address it, and if executed to their
issues' acceptance numbers they reach the goal document's S1 bar (per-motion ≤0.1°,
tour ≤0.5°).

**Two things the plan does NOT cover, and the goal cannot close without them:**

- **G1 — OTOS adoption is on no sprint.** Fusion weights are 0.0 everywhere; the wire
  schema has no position-fusion arm at all; sprint 120's FAKE_OTOS synthesizes OTOS
  *from the encoders* (a plumbing exerciser — by construction it can never add
  information). The stakeholder's stated purpose for the hardware — heading and
  distance from optical flow — has no scheduled work anywhere in sprints 121–127.
- **G2 — Bench accuracy is on no sprint.** The bench tour now *completes* 13/13 but
  closes at 750–1370 mm / 120–155° on an uncalibrated robot. Sprint 125 fixes the
  transport, nothing fixes the accuracy: no calibration campaign, no bench gate with
  numbers, and the known hardware error sources (pure-P velocity droop → the
  `later/nocal-straight-terminal-wedge-needs-velocity-integrator.md` issue, deadband,
  reversal dwell, encoder wedge) all sit unscheduled in `later/`.

Two smaller gaps: **G3** — no sprint ratchets the accuracy gates to the goal bars as
fixes land (the "exact" gate is still an aspirational xfail); **G4** — the playfield
stage has no defined sprint. §8 gives the corrected sprint sequence.

---

## 2. Measured state (sim, this merge)

TOUR_1, deterministic step, ideal chip, per-leg TRUE heading change vs commanded:

| Leg | Kind | Δheading | Error |
|----:|------|---------:|------:|
| 1 | distance | +0.00° | **+0.00°** |
| 2 | turn 90 | +87.80° | −2.20° |
| 3 | distance | +4.23° | **+4.23°** |
| 4 | turn 90 | +90.73° | +0.73° |
| 5 | distance | +2.63° | +2.63° |
| 6 | turn 90 | +87.81° | −2.19° |
| 7 | distance | +4.24° | +4.24° |
| 8 | turn 90 | +90.80° | +0.80° |
| 9 | distance | +2.76° | +2.76° |
| 10 | turn 90 | +92.06° | +2.06° |
| 11 | distance | +1.34° | +1.34° |
| 12 | turn 90 | +90.81° | +0.81° |
| 13 | distance | +2.72° | +2.72° |

Tour net: **+17.9° over 540° commanded**; closure position ~36 mm. Isolated (final)
moves: 90° turn +0.3–1.0°, 360° turn ±0.3°, straight-from-rest exact. Estimation
agreement (post-119-005): truth vs firmware pose vs OTOS all within 0.1° / 4 mm over a
full tour; telemetry stream gap-free (599/599 frames); body never reverses.

### 2.1 Error budget — every remaining degree named

| Mechanism | Where (code) | Magnitude | Fix (scheduled?) |
|---|---|---|---|
| Chain completion fires inside the braking envelope with residual ω | `MoveQueue::landAtZero()`, `kStoppingMarginFactorChain=0.48` (`move_queue.cpp`) — completes when remaining ≤ 0.48× the stopping envelope, i.e. *deliberately while still turning*; the residual decays into a leg that doesn't command ω | +1.3–4.2° added to each following straight (mean +2.9/boundary); the dominant term: ~+17.4° of the tour's +17.9° | **Yes — sprint 121** (`land-at-zero-at-orthogonal-chain-boundaries.md`): orthogonal boundaries use the final-move regime; margin sweeping ends |
| Chain-margin pocket scatter on the turns themselves | same predicate + 40 ms grid: crossing at residual speed makes the firing cycle quantize ±1 cycle | ±2.2° per chained turn (signed scatter, mean ~0) | **Yes — 121/122**: at ω→0 crossing the quantization cost collapses; 122 cleans up the margin machinery |
| Final-move margin conservatism | `kStoppingMarginFactorFinal=0.92` + coast from residual creep | ~0.3° per final move | Acceptable inside S1's 0.1°? Borderline — if not, the terminal creep-servo option (§8, R-121b) closes it without tuning |
| No heading hold on straights | `shapeAndStage()` commands constant ω=0; entry error persists forever | 0 in sim once 121 lands; dominant on hardware (slip/asymmetry) | **Yes — sprint 123** (`heading-hold-during-distance-moves.md`; note: `heading_kp` was deleted with the 119-003 attic — the issue re-adds it with a real consumer, not resurrects) |
| Float32 pose accumulation, midpoint-arc integration | `odometry.cpp` | <0.01° / <0.1 mm per tour | Below every bar; ignore |

**Why sim isn't exact, in one sentence:** the completion predicate is a swept
*fraction* of the physical stopping envelope (0.48 chained / 0.92 final), so every
chained motion is defined-complete while still moving — the error is not noise, drift,
or latency any more; it is the specified semantics of "done," and sprint 121 changes
that specification.

**On "exact to the micron":** in sim it is genuinely available — the plant is exact
and deterministic; the only irreducible quantities are the termination epsilons we
choose and float32 (≈10⁻³ ° class). The goal doc pins S1 at 0.1°/1 mm because pushing
epsilons below that trades settle time for digits with no engineering value; if you
want another decimal, it is an epsilon constant, not a redesign.

---

## 3. Where error enters a tour today (the 30-second trace)

Host sends one bounded `Move` per leg, one leg queued ahead (`run_tour()`,
`planner/tour.py`) → firmware activates it (`MoveQueue::activate()`), shaper S-curves
the commanded axis (`VelocityShaper::next()`), odometry integrates same-cycle
(`robot_loop.cpp` pace block, post-118 order) → **[exact through all of that]** →
`landAtZero()` declares completion *inside* the braking envelope → chain-advance
activates the next leg with the ending axis still moving → the residual decays during
the next leg's first ~0.3 s → **[all 17.9° enter here, 6 times]** → final leg drains
to a real stop (0.92 margin) → host sees the completion ack. Since 119-005 the
estimation/telemetry side is faithful (truth ≡ pose ≡ OTOS ≡ host view); since 119-001
the config can't silently disable any of it. What remains is purely the completion
semantics at boundaries.

Two live functional defects also sit on this path, both scheduled for 121:
`tour-1-final-leg-completes-only-on-stop.md` (final-leg completion ack not consumed —
suspected host-side interaction with 120-001's ack ring) and
`encpose-active-gate-freezes-dead-reckoner-before-motion-ends.md` (display-side).

---

## 4. Bench state and why it isn't exact

Sprint 120 got the bench tour *completing* (13/13, twice, with the FAKE_OTOS build) —
real progress; completion was the four-month blocker. Accuracy is untouched:
closure 750–1370 mm / 120–155° per the ticket's own record, on an **uncalibrated**
robot (neutral `tovez_nocal` gains: pure-P velocity loop with `vel_ki=0` — droop and
terminal wedge are already documented in `later/nocal-straight-terminal-wedge-…`),
with a 15 mm/s deadband, 100 ms reversal dwell, τ≈0.13 s coast, real encoder
quantization, and an intermittent transport defect (envelope loss — sprint 125) that
forced enqueue retries mid-tour.

Bench error sources, in expected order of magnitude: (1) uncalibrated velocity loop
(droop scales every distance and both wheels asymmetrically — plausibly the bulk of
the meters-scale closure); (2) wheel-geometry calibration (trackwidth / travel-calib
per wheel — workflows exist under `calibration/`); (3) deadband/stiction at taper
tails (land-at-zero's low-speed approach will interact with the 15 mm/s deadband —
expect the S3 campaign to need the velocity integrator issue); (4) transport drops
(125); (5) everything sim already fixed, which transfers for free since hardware now
runs the same 40 ms schedule sim validates (118-003 parity).

The bench does not need new theory; it needs the S1-fixed firmware plus a
calibration-then-gate campaign that does not exist in the plan yet (§8, S-B).

---

## 5. OTOS: bought, mounted, wired — and unused

Current facts: `weight_heading_otos = weight_omega_otos = 0.0` (committed,
"encoder-only v1"); the estimator blends heading/ω only — **there is no position
(x/y) fusion arm in the wire schema at all** (`EstimatorConfigPatch` has no
weight_x/weight_y; `BodyEstimate.x/y` documented "never OTOS-blended"); `MoveQueue`'s
stop conditions and `Odometry`'s pose consume encoders exclusively; tovez's real OTOS
is flagged `otos_untrusted: true` ("mechanically decoupled from the wheels on this
robot's current physical mount"); FAKE_OTOS (120-002) proves the *plumbing* on the
stand but deliberately reproduces encoder dead-reckoning (0.00 mm/0.00° deviation from
`pose`, by construction).

Why it matters to the goal: encoders measure wheel rotation, not body motion — on the
playfield, slip/scrub decouples them, and nothing in the current stack can see that.
The OTOS is the independent body-frame measurement. In sim, OTOS ≡ truth, so fusion
can be validated against a known answer before hardware ever depends on it — that is
the correct on-ramp and it costs no hardware time.

What adoption actually requires, in order:
1. **Sim, heading first (cheap):** set the two existing weights nonzero in sim runs;
   verify gates unchanged/improved. This exercises code that already exists.
2. **Estimator v2 — position fusion (new work):** schema arm (weight_x/weight_y or a
   single position weight + staleness), `StateEstimator` blend, and a decision with
   architectural weight: *where fused pose is consumed* — recommend fusing into the
   estimator and having `MoveQueue` consume estimator pose for stop conditions
   (odometry stays the raw integrator), rather than corrupting `Odometry` itself.
   Wrap/unwrap discipline for OTOS heading (chip reports wrapped; odometry is
   unwrapped) is the one known trap — design it, don't discover it.
3. **Realistic-profile sim gate (S2):** error knobs on, calibration push active,
   fused; this is the existing `test_tour_closure_gate.py` realistic branch promoted
   from xfail to a hard gate at the S2 bar.
4. **Bench prerequisite (physical, stakeholder):** remount the OTOS rigidly, clear
   `otos_untrusted`, run the existing OL/OA scale calibration; only then enable
   weights on hardware.
5. **Bench + playfield fused gates (S3/S4).**

---

## 6. Fit and finish (code-survey pass, post-119-004)

The 119-004 relocation did most of what the last review asked; what remains is
specific and small:

**Comment bloat.** The high-ratio firmware headers (`velocity_shaper.h` 6.5:1,
`stop_condition.h` 5.3:1) are *healthy* — contract/units/invariants prose; leave them.
The genuine remaining archaeology concentrates in: `robot/protocol.py` (69
sprint-history lines in the module docstring), `testgui/__main__.py` (63),
`testgui/transport.py` (57), `io/serial_conn.py` (40), `robot_loop.cpp` (36 —
kCycle/kPace derivation history), `io/sim_loop.py` (36), `sim_prefs.py` (25/364 lines,
worst density), `calibration/push.py` (22), `boot_config.h` / `persisted_tuning.h`
(deleted-field eulogies). Disposition: one cleanup ticket, same recipe as 119-004
(narrative → DESIGN.md/git; contracts stay).

**Comment/UI lies (4, worth fixing this week):** `io/sim_loop.py:995` says 50 ms sim
cycles (it's 40); `telemetry.h:2/6` file header still says "single ack slot" while the
same file implements the 120-001 ring; `testgui/__main__.py:803/809` — **user-facing
tooltips** tell the operator the managed path runs "Motion::Executor profile + Pilot
heading loop," both deleted in 115; `telemetry_panel.py:15` names `App::HeadingSource`
as currently-active (deleted; only the wire enum survives).

**Naming (residue past the 071/076 sweeps):** C++ — `Devices::Otos::writePoseMm()`
(API-surface violation; → `writePose` + `// [mm]`), plus three stray locals (`nowMs`
`nezha_motor.cpp:310`, `ageMs` `state_estimator.cpp:77,91`, `shortfallMs`
`microbit_i2c_bus.cpp:161`). Python — a `_cm` parameter cluster that escaped 076:
`path/catmull_rom.py` (`radius_cm`), `path/obstacle.py` (`clearance_cm`),
`path/bezier.py` (`spacing_cm`), `path/sampled_path.py` (`total_length_cm` field),
`calibration/linear.py` (`target_cm`), `field/playfield.py:400` (`x_cm, y_cm`),
`io/cli.py` (`width_cm/height_cm/arrive_cm`). All rename-with-unit-tag mechanical.

**Monoliths (>1500 lines), which are also the top archaeology carriers:**
`testgui/__main__.py` 3325, `testgui/transport.py` 2260, `io/cli.py` 1831,
`io/serial_conn.py` 1616, `robot/protocol.py` 1504. Splitting is real work; at minimum
stop growing them (rule: new TestGUI features land as modules, `__main__.py` only
wires).

---

## 7. Design-document audit (claims vs code at HEAD)

Bodies of `app/DESIGN.md`, `motion/DESIGN.md`, and `docs/protocol-v4.md` were kept
current through 120 — genuinely good hygiene. The stale set:

| Doc | Stale claim | Reality | Action |
|---|---|---|---|
| `docs/protocol-v4.md` §8.2 bit 6 | I2C safety-net bit is a "boot-time one-shot, not actionable" | 120-003 proved it a live, per-Otos-read counter (`app/DESIGN.md` §4 has it right) | Rewrite the bit-6 row (fold into sprint 127, whose issue owns this bit's semantics) |
| `src/sim/DESIGN.md` §2/3 | Whole round-trip narrated as TWIST + deadman + single ack slot | MOVE/StopCondition/ack-ring since 116/120 (its *cadence* text is current) | Rewrite the walkthrough — worst doc in the tree right now |
| `.claude/rules/hardware-bench-testing.md` | Banner `DEVICE:NEZHA2:<name>:microbit:<serial>`; "ack slot"; `mbdeploy deploy --build` | Banner is `DEVICE:NEZHA2:robot:<name>:<serial>` (`main.cpp:34`); ack ring; in-tree record shows `build.py --fw-only` + `mbdeploy deploy <uid> --hex` | Fix before the next bench session uses it as a checklist |
| `app/DESIGN.md:512,519,722` + `:219-222` | Three "single ack slot" cross-refs; pre-119-005 sweep numbers (0.82–0.84/2.398°) beside the current ones (0.48/2.218°) | Ring section exists in the same doc; 119-005 table supersedes | Delete the stale parenthetical, re-point the cross-refs |
| `sim_plant.h:4-8` | `source/devices/…` paths | Tree is `src/firm/…` | Mechanical |
| `test_tour_closure_gate.py` header | "firmware's heading PD", "Executor/Pilot" orientation prose | Deleted in 115 (the xfail reason strings themselves are current — they cite land-at-zero and live issues) | Reword two sentences |
| `docs/design/design.md:27,308` | "single ack slot" ×2 | Ring | One-line notes |

"Last reviewed" stamps on `motion/`, `app/`, `sim/`, `design.md` all predate 118–120;
`app/` and `motion/` bodies were updated anyway — restamp when the above lands.

---

## 8. Sprint-plan validation and the corrected sequence

*(Re-assessed 2026-07-23 evening against the DETAILED sprint plans — 121 is in detail
mode with three tickets; 122–127 carry written Goals/Success Criteria. The plans are
materially better than this morning's roadmap stubs: 121's SUC-072/073/074 embed this
review's own acceptance numbers — ≤0.3°/straight, ≤0.5°/turn, 540°±1° tour, the
per-leg TRUE-heading gate assertion, bench-gated; 123/124 adopt their issues'
acceptance verbatim; 122 pins the 90% no-dip floor with an explicit
stakeholder-decision escape hatch; 125 is diagnosis-first with hardware evidence
required; 127 requires a stakeholder pick from named candidates.)*

Planned: **121** tour completion + trace fidelity + land-at-zero boundaries ·
**122** same-axis carry + chain-margin cleanup · **123** heading hold ·
**124** TOUR_3/TOUR_4 · **125** bench transport reliability · **126** dead-legacy
cleanup · **127** I2C bit semantics.

**As-written coverage of §9's once-and-for-all list:** items 1–3 fully scheduled with
real numbers (121–124); item 5's transport half scheduled (125); item 6 partially
(126 predates this review's §6/§7 findings — fold in the four comment lies, the
naming residue, the monolith growth-freeze, `src/sim/DESIGN.md`'s stale walkthrough,
and the bench-rules doc's wrong banner/commands, which is otherwise UNOWNED and is
used as a live bench checklist). **Still absent as before: item 4 (G1, OTOS
estimator v2 — S2), item 5's accuracy half (G2, bench calibration + S3 gate), item 7
(G4, playfield S4), and the G3 ratchet policy** — 121/124's numbers sit at or near
the S1 bar but no sprint commits to converting the aspirational xfail gates into
permanent hard gates at the goal-document bars as each stage lands.

Bottom line, updated: executed as written, 121–124 deliver S1 (sim exact) and
125–127 deliver bench *reliability* and hygiene — roughly the entire sim half of the
goal and none of the OTOS/bench-accuracy/playfield half. The two campaign insertions
(G1 after 124, G2 after 125 + OTOS remount) remain the difference between "sim
finally exact" and the stated goal.

| Sprint | Advances the goal? | Notes |
|---|---|---|
| 121 | **Yes — the S1 keystone.** | Land-at-zero-at-orthogonal is the +17.4°; tour-1-final-leg is a live blocker; encpose restores trace trust. Add to acceptance: per-leg TRUE-heading assertion in the gate, and ratchet the gate to S1 numbers (G3) |
| 122 | Yes (S1 finish + deletes the pocket machinery) | Success = `kStoppingMarginFactorChain`/`kDiscretizationCyclesChain` cease to exist, not get re-swept |
| 123 | Yes (S1 straights; the big S3 payoff) | Issue notes `heading_kp` must be *re-added* (119-003 deleted it) |
| 124 | Yes (S1 evidence + arc coverage) | Fine at position 124 |
| 125 | Bench completion reliability — necessary, not sufficient | Envelope loss also corrupts accuracy runs via retries |
| 126, 127 | Hygiene — fine, low priority | Fold §6's four comment lies + §7's doc fixes into 126 |
| **missing** | **G1: OTOS estimator v2** | Insert as **~124.5/125.5**: (a) sim heading-weights-on, (b) position-fusion arm + estimator-pose consumption, (c) S2 realistic-profile hard gate. Before S3 depends on it |
| **missing** | **G2: bench accuracy campaign (S3)** | After 125 + OTOS: calibrate (vel gains incl. the `vel_ki` droop issue from `later/`, wheel/track calib, OTOS scales), remount OTOS (stakeholder/physical), then a numeric bench gate mirroring the sim gate |
| **missing** | **G3: gate ratchet** | One ticket inside 121/122: aspirational-xfail → hard gates at the goal-doc bars as each lands |
| **missing** | **G4: playfield (S4)** | Define after S3; camera-truth tooling already exists |

**Answer to "will the planned sprints close this?":** 121–124 close S1 (sim exact) if
held to their issues' numbers — validate by the gate ratchet, not by eyeball. 125–127
keep the bench alive and the tree clean. Nothing planned closes S2 (OTOS) or S3/S4
(bench/playfield accuracy); without inserting G1 and G2 the four-month frustration
recurs on hardware with the sim finally innocent.

## 9. The once-and-for-all list

1. Execute 121 exactly to its issues; ratchet the sim gate to S1 (0.1°/motion, 0.5°
   tour) in the same sprint. Measure TOUR_1 net ≤0.5° or name the physical floor.
2. 122: delete the margin/pocket machinery; same-axis carry per the reset-defeats
   issue; assert SUC-003 un-xfailed.
3. 123 heading hold (re-add `heading_kp` with a consumer); 124 TOUR_3/4 as the
   multiplier evidence.
4. **New — estimator v2 (G1):** sim heading fusion on → position-fusion arm →
   S2 hard gate on the realistic profile.
5. 125 transport; **new — bench accuracy campaign (G2):** calibration battery
   (velocity loop incl. `vel_ki`, wheel geometry, OTOS scales), OTOS remount
   (physical, stakeholder), then the S3 bench gate at ≤3°/≤50 mm.
6. 126 folds in §6's comment lies, §7's doc fixes, the naming residue, and the
   monolith growth-freeze rule; 127 as planned.
7. S4 playfield gate; camera-verified tours inside S4 bars; goal document flips to
   "met" only when all four stage gates are green and permanent.

Everything on this list is either already scheduled, a bounded insertion (G1, G2), or
mechanical. There is no unknown left in the sim error budget, the bench error budget
is enumerated with existing issues covering its biggest terms, and the estimator
design (v1) was explicitly built with v2's consumption point in mind. The goal is
reachable with the machinery you already have — what was missing was the two absent
campaigns and a numeric definition of done, which the goal document now supplies.
