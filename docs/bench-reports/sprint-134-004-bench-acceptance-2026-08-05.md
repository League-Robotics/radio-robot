# Sprint 134 ticket 004 — bench acceptance on `tovez`, 2026-08-05

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
