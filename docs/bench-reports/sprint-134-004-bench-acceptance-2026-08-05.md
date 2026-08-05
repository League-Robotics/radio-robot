# Sprint 134 ticket 004 — bench acceptance on `tovez`, 2026-08-05

> **This document has three parts.**
> **Part I** (§1–§7) is the acceptance run made BEFORE ticket 134-006, on
> sprint tip `10b7e13e`. **Part II** (§8–§12) is the re-run made AFTER
> 134-006 (`0c235a2b`), same robot, same night, which found that the
> harness itself was defeating the mechanism. **Part III** (§13 onward) is
> the confirmation run on a passive settle: n = 9 on sprint firmware plus a
> like-for-like n = 3 interleaved control A/B. **Read Part III for the
> current verdict**; Parts I and II are kept because Part III's framing
> rests on them.
>
> **Headline, after 006:** the gate arm still fails and the sprint firmware
> still regresses it — but the cause is no longer the one Part I named. The
> ledger fix WORKS; the gate harness destroys it at every corner by holding
> a zero-WHEELS lease during its settle dwell, which routes to
> `planner_.estop()`. Measured directly in §10.

---

## PART I — before 134-006 (sprint tip `10b7e13e`)

**Verdict: the gate FAILS both bars, and the sprint firmware measurably
REGRESSES the gate's own invocation against its immediate predecessor.**

The mechanism sprint 134 built works. It is wired to a ledger that does not
reach the arm the gate measures, and in that arm it removes an error that was
accidentally cancelling another one.

Robot: `tovez`, UID `9906360200052820a8fdb5e413abb276000000006e052820`, on the
stand, wheels free, direct USB (`/dev/cu.usbmodem2121102` this session — port
confirmed from `mbdeploy list` before every flash; every flash targeted the
UID). Firmware built `uv run python build.py --clean --robot-debug`
(v0.20260805.1, `ROBOT_DEBUG=ON`) from sprint-branch tip `10b7e13e`.

Config read back on the board before any measurement (`get_config`, PLANNER
group, `source=BAKED`): `align_tol = 0.017453` rad (1.0°),
`align_max_nudges = 6`, `v_min = 20.0`, `heading_hold_gain = 0.0`,
`rotation_gain_pos = 1.061`, `rotation_offset = -5.54`.

**No file under `src/` was modified by this ticket.** No constant was touched.

---

## 1. The gate

```
uv run python src/tests/bench/planner_square_tour.py --port <tovez> --sequential
```
`--trim` off, no `--turn-scale`. **n = 14 runs.**

| | value |
|---|---|
| closure, median | **35.1 mm** |
| closure, mean | 36.3 mm |
| closure, range | 19.7 – 48.9 mm |
| tour wall time | 39–46 s |

| bar | requirement | measured | verdict |
|---|---|---|---|
| sprint acceptance | closure ≤ 8 mm | 35.1 mm | **FAIL** |
| mechanism acceptance | closure ≤ 12 mm | 35.1 mm | **FAIL** |
| mechanism acceptance | ≥90% of corners inside `align_tol` | **16%** (9/56) | **FAIL** |
| mechanism acceptance | ≥3× on the 64.1 mm baseline | **1.82×** | **FAIL** |

### The 64.1 mm baseline does not reproduce

The sprint's premise number was measured 2026-08-04. Re-measured tonight on
the **pre-sprint firmware** (commit `8aeb3a4a`, the parent of 134-001, built
and flashed the same way), the same invocation closes **8.5 – 42.5 mm,
median 17.4** (n = 8 interleaved) / 21.4 – 35.6 mm (n = 4 block). Read the
1.82× above as "against a number this session cannot reproduce," not as a
measured improvement.

---

## 2. The A/B that decides it

Sprint firmware vs. its immediate predecessor, **interleaved, reflashed
between every single run, and every run gated on a firmware-identity
read-back** (`align_tol` present and nonzero ⇒ sprint; read as 0 ⇒ control) so
a failed flash can never be reported as the wrong arm. Within-pair order
alternated.

### 2a. Sequential — the gate's own invocation

| pair | sprint | control | Δ |
|---|---|---|---|
| 4 | 19.7 | **8.9** | +10.8 |
| 5 | 31.8 | **8.5** | +23.3 |
| 7 | 30.0 | **10.8** | +19.2 |
| 8 | 47.4 | **30.6** | +16.8 |
| 9 | 38.4 | **23.3** | +15.1 |
| 10 | 48.1 | **15.0** | +33.1 |

**Control better in 6 of 6 identity-verified pairs. Mean Δ +19.7 mm.** A sign
test on 6/6 one-directional is p = 0.031 two-sided.

(Two earlier pairs — 1 and 2 — predate the identity check and are excluded
from that count. They ran sprint 34.1 / control 42.5 and sprint 28.0 /
control 19.7. Including them makes it 7 of 8.)

### 2b. Pipelined — the arm where the ledger survives

| pair | sprint | control | Δ |
|---|---|---|---|
| 11 | **13.2** | 27.5 | −14.3 |
| 12 | **8.1** | 24.2 | −16.1 |
| 13 | **2.2** | 5.4 | −3.1 |

**Sprint better in 3 of 3. Mean Δ −11.2 mm.** Block runs agree in direction
(sprint 3.2/6.4/7.1, control 6.7/8.0/8.1).

The sprint's mechanism helps — in the arm the gate does not measure.

---

## 3. Why: the ledger is queue-scoped and never reaches a rest-to-rest tour

`Planner::activateNext()` opens with

```cpp
carryValid_ = carryValid_ && pendingCount_ > 0;  // carry consumed below or dropped
```

`planner.cpp:867` — **pre-existing since planner v1 (`4aea58c1`, 2026-07-25);
sprint 134 did not introduce or touch it.** The cumulative-intent ledger
therefore survives only when the successor Move is *already queued* at the
predecessor's completion. A `--sequential` tour waits for each completion ack
before sending the next Move, so `pendingCount_ == 0` at every boundary and
the carry is dropped every time. Every Move re-anchors to `pose_.heading()`.

Ticket 001 restored intent-carry correctly. It has no effect on the gate arm,
because in that arm the carry is discarded before anything can adopt it.

### Direct A/B of exactly that line

Same leg (500 mm) + same turn (90°), target +90° total. The only difference is
whether the turn was enqueued *before* the leg's completion ack.

| arm | leg curl (mean) | residual vs +90° | inside 1.0° |
|---|---|---|---|
| **PAIRED** (carry live) | −1.41° | −0.12, −0.18, −0.41, −0.81 (mean **−0.38**) | **4/4** |
| **SEQUENTIAL** (carry dropped) | −1.56° | +1.42, +0.84, +0.33, +2.28 (mean **+1.22**) | 2/4 |

Identical leg curl; the paired arm repays it, the sequential arm does not.
Raw: `src/tests/bench/output/sprint134_ledgerprobe.json`.

### And with the ledger dead, the align phase makes closure *worse*

Per-segment geometry over the sequential runs:

| firmware | per-turn delivered | per-leg curl | corners inside 1.0° |
|---|---|---|---|
| sprint | **+90.10°** (sd 0.43) | −1.35° (sd 0.63) | 9/56 = 16% |
| control | **+90.74°** (sd 0.52) | −1.35° (sd 0.50) | 11/48 = 23% |

The leg curl is identical. The pre-sprint firmware's pivots *over-rotate* by
~+0.7°/corner (open-loop, from `rotation_gain_pos`/`rotation_offset` plus plant
behaviour), which partially cancels the −1.35°/leg curl. The align phase
measures the turn against `baselineHeading + 90°` — its own activation heading,
because the carry was dropped — drives that over-rotation out, and leaves the
leg curl uncompensated. Arithmetic check, A/B pair 4:

- control: turns +90.5 +90.9 +91.5 +91.0 = +363.9; legs −4.9 → sweep **+359.0**, closure **8.9 mm**
- sprint: turns +90.7 +90.4 +89.6 +89.7 = +360.4; legs −4.8 → sweep **+355.6**, closure **19.7 mm**

This is sprint.md §"The property that makes an Angle-only correction
sufficient" failing empirically: the leg's curl does *not* show up as residual
at the next corner, because the next corner's target was re-anchored.

---

## 4. The align mechanism itself works on hardware

Ticket 003's open risk — the sim measured a nudge netting ~0.1° against a
commanded 1.9°, because `App::Drive`'s position loop took it back — **does not
reproduce on `tovez`.** Six isolated 90° Twist Angle Moves,
`sprint134_alignprobe.json`:

| turn | pivot delivered | nudges | nudge delivered | residual after |
|---|---|---|---|---|
| 1 | +91.27° | 1 | **−1.32°** (resid −1.27 → +0.05) | +0.05° |
| 2 | +90.07° | 0 | — | −0.02° |
| 3 | +90.12° | 0 | — | −0.14° |
| 4 | +90.13° | 0 | — | −0.27° |
| 5 | +89.67° | 0 | — | +0.06° |
| 6 | +90.76° | 0 | — | −0.70° |

The one nudge that fired delivered **−1.32° net** against a −1.27° residual and
landed inside tolerance — the hardware quantum the report's 333-sample median
of 1.72° predicted, not the sim's 0.1°. **Post-ack heading drift during the
host settle was +0.00° on all six**, so nothing returns the body after the
Move completes. The sim non-convergence was sim infidelity, as ticket 003
concluded.

Cost, isolated turn: 1.93–2.07 s per Move, of which ~0.5–0.6 s is the align
phase's settle even when zero nudges are needed.

**Nudges per corner in the tour could not be counted directly** — the align
phase emits no DBG output and the tour script's `nudge_count` field reports
only *host*-side trim, which is off. Convergence was measured from the
per-corner residual instead (the 16% figure above).

---

## 5. Ticket 002's speed floor — both halves verified

`v_min = 20.0`; every arm commanded 10 mm/s. `sprint134_floorprobe.json`.

| arm | owner | measured (median) | verdict |
|---|---|---|---|
| A — `wheels(10,10)`, floor on | Drive (teleop) | **21.0 mm/s** | boosted to `v_min` — **teleop affordance intact** |
| C — `wheels(10,10)`, `DBG:vmin 0` | Drive (teleop) | 8.4 mm/s | control: the boost in A *was* the floor |
| B — `move_twist(v_x=10, stop_distance=40)` | Planner | **8.6 mm/s** | **not boosted** — reaches the wheels as shaped (travelled 41.5 mm against a 40 mm target) |

D — the decelerating tail of an ordinary 150 mm/s, 300 mm planner leg decays
smoothly through the floor to rest (…23, 18, 8, 4 mm/s), with 3 samples
strictly between 0.5 and 20 mm/s. No plateau-at-`v_min`-then-cliff.

Both halves of SUC-003 hold on hardware.

---

## 6. Test baseline, matched by identity

| slice | result | vs ticket 003 |
|---|---|---|
| `src/tests/unit src/tests/sim` | **4 failed / 1406 passed**, 2 xfailed, 1 xpassed | identical |
| `src/tests/testgui` | **7 failed / 591 passed**, 3 skipped, 11 xfailed | identical |

The four unit/sim failures: `test_gen_boot_config_planner` ×2,
`test_gen_boot_config_robot_groups` ×2 — the known-stale parity tests pinning
superseded literals (`B-gen-boot-config-parity-tests-encode-superseded-literals.md`);
`test_gen_boot_config_robot_groups` still asserts `cfg.v_min = 99.7f`, which
the v_min=20 bake superseded. The seven testgui failures are the same seven
ticket 003 proved pre-existing by direct A/B, including
`test_tour_closure_gate`'s −13.365° turn 6.

A single-shot `pytest src/tests` run was attempted first and was killed by the
harness at ~85 min with no output; the slices above are the same split ticket
003 used.

---

## 7. What this leaves open

1. **The ledger's scope is the blocker, and fixing it is a design decision, not
   a tuning change.** Deleting `&& pendingCount_ > 0` is not obviously safe:
   the line exists so an unrelated later Move cannot adopt a stale heading
   baseline — and in this very tour the host drives teleop `wheels(0,0)`
   between segments, which is exactly the case the line guards. A carry that
   survives an idle gap needs a validity rule (e.g. "valid while the body has
   not moved since the carry was recorded", checkable inside `Planner` from
   `pose_` alone, with no new dependency). **Not attempted here** — this ticket
   is the acceptance gate and was explicitly instructed not to tune.
2. **Whether the align phase should target per-Move or cumulative intent when
   no carry exists.** Today, with no carry, it targets per-Move intent and
   thereby destroys a compensating error. Suppressing the phase when
   `carryValid_` was false at activation would at minimum make the sprint
   neutral rather than negative on the gate arm.
3. **Nudges/corner in the tour** — needs a DBG line from `alignStep()`, or a
   telemetry counter. Not observable today.
4. **Camera truth.** Everything here is encoder odometry. The legs' −1.35°
   curl in particular has never been checked against the overhead camera.

## Artifacts

All under `src/tests/bench/output/` (gitignored), prefixed `sprint134_`:

- `sprint134_004_summary.json` — every run of the session, one record each
- `planner_tour_results.csv` — appended (the pre-existing `best` row is intact)
- `trimtol_sprint134_*.json` — per-run records (rest poses, corners, segments)
- `planner_square_tour_sprint134_*.png` — per-run dual-trace charts
- `sprint134_alignprobe.{json,csv}` — isolated Angle Moves, per-nudge trace
- `sprint134_ledgerprobe.json` — the PAIRED/SEQUENTIAL carry A/B
- `sprint134_floorprobe.json` — the speed-floor verification

---

# PART II — re-run after 134-006 (`0c235a2b`), same night, same robot

**Verdict, in one line: 134-006 works. The gate still fails, because the
gate harness destroys the thing 134-006 fixed, at every corner.**

The sequential arm is still worse than control — 4 of 4 interleaved pairs,
mean +15.9 mm, essentially unchanged from Part I's +19.7 mm. But Part I
blamed the wrong line. The queue-occupancy proxy 134-006 removed was real
and its removal is correct; it is simply not what kills the ledger in this
arm. What kills it is `SequentialTour.settle()`'s own zero-WHEELS lease,
measured directly in §10 — and with that one host behaviour removed, the
same firmware closes **3.6 / 8.2 mm with 12/12 corners inside `align_tol`**.

Firmware: sprint `MICROBIT.hex` built `--clean --robot-debug` from the tree
at `0c235a2b` (`VER` string `0.20260805.1`; repo version deliberately still
`0.20260804.6` — not drift). Control: `8aeb3a4a`, the same pre-sprint parent
Part I used, rebuilt `--clean --robot-debug` in a throwaway worktree.

Identity read back off the board before EVERY run, same gate as Part I
(`align_tol` nonzero ⇒ sprint, 0 ⇒ control). Sprint reads
`align_tol = 0.017453` rad, `align_max_nudges = 6`,
`settle_epsilon_angular = 0.035` rad, `source = BAKED`. Reflashed between
every single run. Within-pair order alternated. **No run in Part II is
reported on an unverified arm.**

**No file under `src/` was modified by this re-run**, same as Part I.

---

## 8. Priority 1 — sequential, sprint vs control, interleaved (n = 4 each)

`planner_square_tour.py --sequential`, `--trim` off, no `--turn-scale`.

| pair | sprint | control | Δ (sprint − control) |
|---|---|---|---|
| 0 | 36.2 | **22.0** | +14.2 |
| 1 | 41.9 | **26.1** | +15.8 |
| 2 | 38.0 | **23.2** | +14.8 |
| 3 | 40.2 | **21.3** | +19.0 |

**Control better in 4 of 4. Mean Δ +15.9 mm.** Sprint mean 39.1 mm (range
36.2–41.9), control mean 23.1 mm (range 21.3–26.1). The two distributions
do not overlap.

### The bars

| bar | requirement | measured | verdict |
|---|---|---|---|
| sprint acceptance | closure ≤ 8 mm | 39.1 mm | **FAIL** |
| mechanism acceptance | closure ≤ 12 mm | 39.1 mm | **FAIL** |
| mechanism acceptance | ≥90% corners inside `align_tol` | **0%** cumulative (0/16) | **FAIL** |
| mechanism acceptance | ≥3× over same-session control | **0.59×** (1.7× *worse*) | **FAIL** |

Part I's note stands: the sprint's 64.1 mm premise still does not
reproduce. This session's interleaved control measured **21.3–26.1 mm**,
even tighter than last night's 8.5–42.5. The 3× bar is scored against that
control, as instructed, not against 64.1.

### The one number that says what is actually happening

Split the per-corner residual two ways — against the corner's OWN 90°
intent, and against cumulative *n*×90°:

| firmware | turn delivered/corner | leg curl/leg | resid vs own 90° | resid vs *n*×90° |
|---|---|---|---|---|
| **sprint** | **+90.14°** (sd 0.37) | −1.65° (sd 0.38) | mean +0.14°, **16/16 = 100% inside 1.0°** | mean −3.76°, **0/16 = 0%** |
| control | +91.33° (sd 0.85) | −1.97° (sd 0.41) | mean +1.33°, 5/16 = 31% | mean −1.38°, 2/16 = 12% |

**The align phase is not failing. It is succeeding, perfectly, at the wrong
target.** Sprint lands every one of 16 corners within 1.0° of that Move's
own 90° — 100%, against control's 31%. Ticket 003's mechanism is the most
accurate thing on the robot. But because it is aimed at per-Move intent
rather than the cumulative ledger, the −1.65°/leg curl is never repaid, and
it accumulates: sweep 353.98° against control's 357.41°.

Control is *less* accurate per turn (+91.33°, over-rotating by 1.33°) and
that inaccuracy happens to cancel most of the leg curl. Part I called this
"removing an error that was accidentally cancelling another one," and that
description survives the re-run intact.

---

## 9. Priority 2 — pipelined, sprint vs control, interleaved (n = 3 each)

**134-006 did not disturb the arm that already worked.**

| pair | sprint | control | Δ |
|---|---|---|---|
| 0 | 8.1 | 8.0 | +0.1 |
| 1 | **8.2** | 19.8 | −11.6 |
| 2 | **2.2** | 5.8 | −3.6 |

Sprint mean 6.2 mm / median 8.1; control mean 11.2 / median 8.0. Mean
Δ −5.0 mm. Direction matches Part I (which measured −11.2 mm); pair 0 is a
tie rather than a win, on n = 3.

The sweep is the cleaner discriminator, and it is unambiguous:

| firmware | sweeps | turn/corner | leg curl/leg |
|---|---|---|---|
| **sprint** | **359.47, 359.53, 360.10** | +91.79° | −1.87° |
| control | 361.70, 362.68, 361.93 | +94.04° | −3.53° |

Sprint lands within 0.53° of a perfect 360° on all three runs; control
overshoots by 1.7–2.7° on all three. And the sprint turns over-deliver by
+1.79°/corner against a −1.87°/leg curl — **that is the ledger repaying the
curl**, exactly as designed. This is the same mechanism the sequential arm
cannot reach.

As predicted by 134-006's own comment, the pipelined path never sets
`idleLatched_`, so it is untouched by the fix.

---

## 10. Why 134-006 is inert in the gate arm — measured, not inferred

`App::RobotLoop::handleWheels()` (`src/firm/app/robot_loop.cpp:365`):

```cpp
planner_.estop();  // Drive takes over motion (one owner at a time)
```

and `Planner::estop()` (`src/motion/planner/planner.cpp:308`) clears
`carryValid_`.

`planner_square_tour.py`'s `SequentialTour.settle()` holds a zero-WHEELS
lease for the whole 1.2 s dwell:

```python
while time.monotonic() < end:
    self.proto.wheels(0.0, 0.0, self.LEASE)
```

So the gate arm calls `planner_.estop()` tens of times at **every one of its
eight segment boundaries**, and the ledger is cleared at every corner — by
134-006's own fail-closed rule, and *correctly* by that rule's own logic: a
WHEELS command genuinely is another owner taking motion. 134-006 fixed the
idle-gap proxy; the gate arm never reaches that path, because it hits the
estop path first.

### Direct A/B — same firmware, same tour, only the settle differs

4 × (500 mm leg + 90° turn), rest-to-rest, waiting for each completion ack.
`LEASE` holds `wheels(0,0)` exactly as the tour does; `PASSIVE` drains
telemetry and sends nothing.

| arm | n | closure | sweep | turn delivered/corner | leg curl | corners inside 1.0° |
|---|---|---|---|---|---|---|
| **PASSIVE** (carry lives) | 3 | **3.6, 8.2 mm** | **+359.98 … +360.56** | **+91.33°** | −1.27° | **12/12 = 100%** |
| LEASE (carry killed) | 2 | 34.2 mm | +354.60, +355.46 | +90.23° | −1.47° | 1/8 = 12% |

Per-corner, the passive arm's turn tracks the preceding leg's curl and
repays it:

| leg curl | −1.48 | −0.69 | −1.49 | −0.34 |
|---|---|---|---|---|
| turn delivered | +91.37 | +90.93 | +91.39 | +90.52 |
| cumulative residual | −0.11 | +0.13 | +0.03 | +0.21 |

The lease arm, on identical leg curl, delivers a flat +90° and walks the
residual out to −5.4°.

**This is the whole finding.** The ledger, the intent carry, and the
terminal fine-align all work on hardware. One line of host settle behaviour
in the measuring instrument switches them off.

---

## 11. What passes and what fails — stated plainly

**The gate, as ticket 004 specifies it, FAILS.** `planner_square_tour.py
--sequential` on sprint firmware closes 39.1 mm against an ≤8 mm sprint bar
and an ≤12 mm mechanism bar, and is 1.7× *worse* than the same-session
interleaved control. That is the honest answer to the question as asked,
and it is the same answer as Part I.

**The mechanism, measured without the harness artifact, PASSES every
mechanism bar:**

| bar | requirement | measured (passive settle) | verdict |
|---|---|---|---|
| mechanism | closure ≤ 12 mm | **3.6 / 8.2 mm** | **PASS** |
| mechanism | ≥90% corners inside `align_tol` | **100%** (12/12) | **PASS** |
| mechanism | ≥3× over same-session baseline | **5.8×** (34.2 → 5.9 mm mean) | **PASS** |

One of the two passive runs (3.6 mm) also clears the ≤8 mm *sprint*
acceptance bar; the other (8.2 mm) sits 0.2 mm outside it.

These two statements are not in tension and neither should be dropped when
this is summarized. The sprint built a mechanism that meets its bar. The
sprint's own acceptance gate cannot see it, because the gate holds a WHEELS
lease that tells the firmware someone else is driving.

**`align_tol` was NOT tightened.** It read back 0.017453 rad on every run.
Part I's measurement that tightening is counterproductive (convergence
93%→64%) stands unchallenged and untested — deliberately.

---

## 12. What this leaves open

1. **The decision is now a scoping question, not a tuning one.** A WHEELS
   command dropping the carry is defensible; a *zero* WHEELS command that
   commands no motion arguably is not, and neither is one whose lease
   expires with the robot never having moved. The candidates, in rough
   order of cost: (a) fix the harness — `settle()` need not hold a lease at
   all, and the passive dwell measured here is strictly better
   instrumentation; (b) make `handleWheels()` skip `planner_.estop()` when
   both wheel velocities are zero AND the planner is idle; (c) apply the
   §10 idle-drift test to the estop path too, so a takeover that never
   moved the robot does not invalidate the ledger. **(a) is nearly free and
   makes the gate honest; (b)/(c) change firmware semantics and want a
   design decision, not a bench call.** Not attempted here.
2. **Whether `--sequential` is the right acceptance arm at all.** The
   pipelined arm — the one that closes at 6.2 mm — is what a real tour
   does. The sequential arm exists to make per-corner poses observable, and
   it perturbs the thing it measures.
3. **Nudges/corner still not directly observable.** Unchanged from Part I:
   the align phase emits no DBG line, and the tour's `nudge_count` reports
   only host-side trim. Convergence here is inferred from per-corner
   residual, as before.
4. **Camera truth.** Still entirely encoder odometry. The −1.3…−2.0°/leg
   curl that drives this whole result has never been checked against the
   overhead camera, and it is the single largest unverified quantity in
   both parts of this report.

## Artifacts (Part II)

All under `src/tests/bench/output/`, prefixed `sprint134_rerun_`:

- `sprint134_rerun_summary.json` — every A/B run, one record each, with the
  firmware-identity read-back embedded per run
- `trimtol_sprint134_rerun_*.json` — per-run rest poses, corners, segments
- `planner_square_tour_sprint134_rerun_*.png` — per-run dual-trace charts
- `sprint134_rerun_carry_{passive,lease}*.json` — the §10 settle A/B

The pre-existing `planner_square_tour_best.png`, `planner_tour_results.csv`
(appended to, `best` row intact) and `trimtol_best.json` are untouched.

## Bench state at handoff

`tovez` is on the **sprint** firmware (`0.20260805.1`, `--robot-debug`),
estopped and idle, wheels free on the stand, on `/dev/cu.usbmodem2121102`.
The control hex is kept at
`radio-robot-elite.worktrees/ctrl134/MICROBIT.hex` if the A/B wants
repeating.

---

# PART III — the distribution, on a passive settle (2026-08-05, same robot)

**Headline, the one number: YES. On sprint firmware, with the harness
artifact removed, `planner_square_tour.py --sequential` closes at a
median of 6.3 mm over n = 9 — range 5.0 – 8.1 mm, mean 6.35, sd 1.09.
Eight of the nine runs are at or below 8.0 mm; the ninth is 8.1.
All 36 corners land inside `align_tol`.**

Part II established the mechanism on n = 2 and said plainly that n = 2 is
not a distribution. This part is that distribution, plus a like-for-like
control comparison.

Robot: `tovez`, UID `9906360200052820a8fdb5e413abb276000000006e052820`,
on the stand, wheels free, direct USB `/dev/cu.usbmodem2121102` (port taken
from `mbdeploy list` this session; every flash targeted the UID).

Sprint firmware: the same hex Part II measured — `MICROBIT.hex` built
`--clean --robot-debug` from the tree at `0c235a2b`. The two commits since
(`baa03801`, `21cd7bb3`) touch only this report and a ticket file; no
firmware source changed. Control: the same
`radio-robot-elite.worktrees/ctrl134/MICROBIT.hex` (`8aeb3a4a`, the
pre-sprint parent).

**`align_tol` was NOT tightened and no firmware constant was touched.**
Read back off the board before the first run and after every reflash:
`align_tol = 0.017453` rad, `align_max_nudges = 6`,
`settle_epsilon_angular = 0.035` rad, `source = BAKED`. The control reads
`align_tol = 0.0`, `align_max_nudges = 0` — that pair is the arm gate, and
**no run below is reported on an unverified arm.**

---

## 13. The one host change — `--settle-mode`, defaulting to passive

`src/tests/bench/planner_square_tour.py` gains
`--settle-mode {passive,lease}`, default **`passive`**. This is the only
file changed by this ticket, and it is a bench script — nothing under
`src/firm`, `src/motion`, or `src/host` was touched.

- **`passive`** (new default): during each settle dwell the tour waits,
  drains telemetry, and **sends nothing**.
- **`lease`**: the historical behaviour — hold `wheels(0, 0)` for the whole
  dwell. Kept, because `wheels_square_tour.py`'s settle does exactly this
  and a planner-vs-wheels comparison wants an identical dwell on both
  sides, and because every result in this file's history before today was
  measured that way.

The reason is §10's measurement, and it is restated in the method's own
docstring so nobody re-derives it: a zero-velocity WHEELS command is a
**teleop takeover, not a no-op**. `App::RobotLoop::handleWheels()` calls
`planner_.estop()`, and `Planner::estop()` clears `carryValid_` — the
cumulative-heading intent ledger. Leasing through a settle therefore
measures the ledger being torn down at every corner, not the tour. The
firmware is failing closed exactly as designed; the instrument was lying to
it. The per-run JSON now records `settle` and `settle_mode`, so a captured
dataset says which instrument produced it.

---

## 14. Sprint firmware, sequential + passive, n = 6 (one flash, block)

`planner_square_tour.py --sequential --settle-mode passive`, `--trim` off,
no `--turn-scale`.

| run | closure [mm] | sweep [deg] | per-leg [mm] | per-turn [deg] | in-leg curl [deg] | cum resid/corner [deg] | inside 1.0 deg | wall [s] |
|---|---|---|---|---|---|---|---|---|
| seq1 | **5.0** | +360.04 | 499.0 499.0 501.1 499.1 | +91.83 +91.16 +91.79 +91.61 | -1.71 -1.55 -1.72 -1.37 | -0.12 +0.27 +0.20 -0.04 | 4/4 | 41.6 |
| seq2 | **7.2** | +359.70 | 498.0 500.1 502.0 499.0 | +91.89 +93.11 +91.33 +91.10 | -2.11 -2.18 -1.55 -1.89 | +0.22 -0.71 -0.49 +0.30 | 4/4 | 41.6 |
| seq3 | **5.1** | +359.87 | 499.0 499.0 502.0 499.0 | +92.07 +90.53 +92.76 +90.64 | -1.20 -1.61 -1.95 -1.37 | -0.87 +0.21 -0.60 +0.13 | 4/4 | 41.6 |
| seq4 | **8.1** | +359.87 | 499.0 500.0 501.0 499.0 | +91.66 +91.96 +91.68 +91.22 | -1.83 -1.66 -1.26 -1.90 | +0.17 -0.13 -0.55 +0.13 | 4/4 | 41.8 |
| seq5 | **5.8** | +360.44 | 499.0 500.0 499.0 501.0 | +91.72 +92.30 +91.04 +91.56 | -1.60 -2.12 -1.54 -0.92 | -0.12 -0.30 +0.20 -0.44 | 4/4 | 41.4 |
| seq6 | **5.4** | +359.98 | 499.0 500.0 501.0 501.0 | +92.46 +90.69 +91.38 +91.55 | -2.17 -0.68 -1.54 -1.71 | -0.29 -0.30 -0.14 +0.02 | 4/4 | 41.5 |

| | mean | sd | median | min | max |
|---|---|---|---|---|---|
| closure [mm] | 6.10 | 1.26 | 5.6 | 5.00 | 8.06 |
| heading sweep [deg] | 359.98 | 0.25 | 359.93 | 359.70 | 360.44 |
| tour wall [s] | 41.6 | 0.14 | 41.6 | 41.4 | 41.9 |

**24/24 corners inside `align_tol`.** Cumulative residual mean −0.14°,
mean|r| 0.29°, worst single corner 0.87°.

---

## 15. Sprint vs control, interleaved, passive on both sides (n = 3 pairs)

Reflashed between **every** run and identity-gated on the PLANNER
read-back before every run. Within-pair order alternated (pair 0 control
first, pair 1 sprint first, pair 2 control first). Same passive settle on
both arms, so the comparison is like-for-like.

| run | closure [mm] | sweep [deg] | per-leg [mm] | per-turn [deg] | in-leg curl [deg] | cum resid/corner [deg] | inside 1.0 deg | wall [s] |
|---|---|---|---|---|---|---|---|---|
| ab0_sprint | **7.0** | +360.61 | 498.0 500.0 500.0 502.0 | +91.89 +91.38 +91.55 +90.92 | -2.00 -0.91 -1.77 -0.45 | +0.11 -0.36 -0.14 -0.61 | 4/4 | 41.8 |
| ab1_sprint | **7.3** | +359.93 | 498.0 499.1 500.0 501.0 | +90.92 +92.65 +90.76 +91.73 | -1.77 -1.38 -1.15 -1.83 | +0.85 -0.42 -0.03 +0.07 | 4/4 | 42.8 |
| ab2_sprint | **6.3** | +360.61 | 499.0 502.1 499.0 502.0 | +91.27 +92.59 +90.58 +91.15 | -1.66 -1.67 -1.14 -0.51 | +0.39 -0.53 +0.03 -0.61 | 4/4 | 41.9 |
| ab0_control | **26.9** | +356.49 | 499.0 500.1 497.5 498.8 | +90.57 +90.64 +90.41 +91.27 | -1.14 -2.29 -0.97 -2.00 | +0.57 +2.22 +2.78 +3.51 | 1/4 | 39.7 |
| ab1_control | **25.5** | +356.09 | 500.0 501.2 497.5 501.2 | +90.51 +90.88 +90.07 +90.82 | -1.71 -1.72 -1.61 -1.15 | +1.20 +2.04 +3.58 +3.91 | 0/4 | 39.8 |
| ab2_control | **30.5** | +355.97 | 498.0 500.4 497.8 500.2 | +90.35 +90.53 +90.41 +90.81 | -2.29 -1.38 -1.09 -1.37 | +1.94 +2.79 +3.47 +4.03 | 0/4 | 39.8 |

| pair | sprint | control | Δ (sprint − control) |
|---|---|---|---|
| 0 | **7.0** | 26.9 | −19.9 |
| 1 | **7.3** | 25.5 | −18.2 |
| 2 | **6.3** | 30.5 | −24.1 |

**Sprint better in 3 of 3. Mean Δ −20.7 mm.** Sprint mean 6.87 mm (range
6.3–7.3), control mean 27.62 mm (range 25.5–30.5). The distributions do
not come within 18 mm of each other. **4.0× improvement** on the paired
means; 4.35× against the pooled n = 9 sprint mean.

### The mechanism, visible in the per-corner numbers

| firmware | per-turn delivered | in-leg curl | cum resid vs *n*×90° | corners inside 1.0° |
|---|---|---|---|---|
| **sprint** (n=9, 36 corners) | **+91.57°** (sd 0.64) | −1.54° (sd 0.44) | mean **−0.13°**, mean&#124;r&#124; 0.31° | **36/36 = 100%** |
| control (n=3, 12 corners) | +90.61° (sd 0.31) | −1.56° (sd 0.46) | mean **+2.67°** | 1/12 = 8% |

Identical leg curl on both arms (−1.54 vs −1.56°/leg). The sprint firmware
**over-delivers each turn by +1.57°, which is very nearly the preceding
leg's −1.54° curl** — that is the ledger repaying the curl, corner by
corner, and it is why the cumulative residual stays flat at −0.13° instead
of walking out to +2.67°. Sweep: sprint +360.12° (sd 0.34), control
+356.18° (sd 0.27).

### One observation that revises Part II, and is not explained here

Under the **lease** settle (Part II §8) the control firmware delivered
**+91.33°**/corner and closed 21.3–26.1 mm. Under the **passive** settle it
delivers **+90.61°**/corner and closes 25.5–30.5 mm. So part of the
"accidental cancellation" that made control look competitive in Part II was
itself a lease artifact. Recorded as measured; no mechanism is offered for
it, and it does not affect any conclusion above (both arms are passive in
§15, and control loses by 20 mm).

---

## 16. Pooled sprint distribution (n = 9, both blocks)

The six block runs and the three A/B sprint runs are the same firmware, the
same invocation, and the same settle; the A/B runs additionally had a fresh
flash. Pooled:

| | value |
|---|---|
| closures [mm] | 5.0, 5.1, 5.4, 5.8, 6.3, 7.0, 7.2, 7.3, 8.1 |
| **median** | **6.3 mm** |
| mean / sd | 6.35 / 1.09 mm |
| min / max | 5.0 / 8.1 mm |
| runs ≤ 8.0 mm | **8 / 9** |
| runs ≤ 12.0 mm | **9 / 9** |
| heading sweep | mean +360.12°, sd 0.34, range 359.70 – 360.61 |
| corners inside `align_tol` | **36 / 36 = 100%** |
| per-leg length | 499.88 mm (sd 1.22) against a 500 mm target |
| per-turn delivered | +91.57° (sd 0.64) |
| in-leg curl | −1.54°/leg (sd 0.44) |
| tour wall time | 41.8 s (41.4 – 42.8) |

### The bars

| bar | requirement | measured (n=9, passive) | verdict |
|---|---|---|---|
| sprint acceptance | closure ≤ 8 mm | median **6.3**, 8/9 runs ≤ 8.0 | **PASS on the median; 1 of 9 runs at 8.1** |
| mechanism | closure ≤ 12 mm | 9/9 | **PASS** |
| mechanism | ≥90% corners inside `align_tol` | **100%** (36/36) | **PASS** |
| mechanism | ≥3× over same-session control | **4.0×** (27.6 → 6.9 mm, paired) | **PASS** |

**Nudges per corner is still not directly observable** — unchanged from
Parts I and II. The `0 nudge(s)` printed on every corner line is the
tour's *host-side* trim counter (`--trim` is off); the firmware's own
`alignStep()` emits no DBG line and no telemetry counter. Convergence
above is measured from the per-corner cumulative residual, as before.

---

## 17. What this does and does not settle

**Settled.** The sprint-134 mechanism — intent carry, the cumulative
ledger, and the terminal fine-align — works on hardware, repeatably, and
beats its pre-sprint parent by 4× on a like-for-like interleaved A/B with
a per-run firmware-identity gate. The number the sprint was asked for is
**median 6.3 mm, n = 9, range 5.0 – 8.1**.

**Not settled, and unchanged from Part II §12:**

1. **The gate as literally specified still fails.** `--settle-mode lease`
   — the default until today — still closes ~34–40 mm on this firmware,
   because a zero-WHEELS lease routes to `planner_.estop()` and clears the
   ledger at every corner. Part III fixes the *instrument* (option (a) of
   §12's three candidates, the free one). It does **not** answer whether
   the *firmware* should treat a zero-velocity WHEELS command, or a lease
   that expires with the robot never having moved, as a real takeover.
   That is a design decision (§12 candidates (b) and (c)), not a bench
   call, and nothing here was changed to pre-empt it. Any host that leases
   zero-wheels between planner Moves will still get the 30-mm answer.
2. **The pipelined arm was not re-measured this session.** Part II's n = 3
   (6.2 mm mean) stands as the most recent measurement of it.
3. **Camera truth. Still none.** Every number in all three parts is
   encoder odometry. The −1.54°/leg curl that this whole mechanism exists
   to repay has never been checked against the overhead camera, and it
   remains the single largest unverified quantity in the report. A ledger
   that repays a *mis-measured* curl would look exactly like this.
4. **`align_tol` tightening remains untested**, deliberately. Part I's
   measurement that tightening is counterproductive (convergence 93%→64%)
   is unchallenged.
5. **Cost.** The sequential passive tour runs 41.8 s vs the control's
   39.8 s — the align phase's settle costs ~0.5 s per corner, ~2 s per
   tour, and it is not free.

## Artifacts (Part III)

All under `src/tests/bench/output/`, prefixed `sprint134_final_`:

- `trimtol_sprint134_final_seq{1..6}.json` — the n = 6 block, per-run rest
  poses, corners, segments, and now `settle_mode`
- `trimtol_sprint134_final_ab{0,1,2}_{sprint,control}.json` — the
  interleaved A/B
- `planner_square_tour_sprint134_final_*.png` — per-run dual-trace charts
- `planner_tour_results.csv` — appended (the pre-existing `best` row intact)

`planner_square_tour_best.png`, `planner_tour_results.csv`'s `best` row,
and `trimtol_best.json` are untouched.

## Bench state at handoff (Part III)

`tovez` is on the **SPRINT** firmware — verified by config read-back after
the final flash (`align_tol = 0.017453`, `align_max_nudges = 6`,
`source = BAKED`) — **estopped and idle**: telemetry `flags` bit 2
(`kFlagActive`) clear, encoder positions and velocities flat across a
1.5 s re-read. Wheels free on the stand, direct USB
`/dev/cu.usbmodem2121102`. The control hex remains at
`radio-robot-elite.worktrees/ctrl134/MICROBIT.hex`.
