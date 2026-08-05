# Motion-planning lab — what to build, with the measurements that say so

**2026-08-04, one OOP session. Robots: `tovez` (bench A/B, plant ID),
`gopiv` (boundary timing, motors-no-wheels). Sim: the newly predictive
plant (`motion-lab` worktree, `bb9f96df`). Companion package: the same-day
wheels-only session (`turn-tuning` worktree,
`docs/findings/wheels-motion-primitives-2026-08-04.md`) — this document
builds on it and answers its open question of whether the planner adds
anything.**

Goal, as set by the stakeholder: **improve positioning quality on moves**,
using the square tour as the test, and decide what goes into production
code. This is that plan. Every recommendation carries the measurement that
justifies it; everything rejected carries the measurement that killed it.

## 1. The governing model (confirmed, r = +0.95)

**Square-tour closure is propagated per-corner cumulative-heading
residual.** Across 30 valid runs spanning seven configurations,
`corr(closure, L·√Σ residual²) = +0.95`. Leg-length error, turn-mean
bias that a corrector can see, and calibration constants all trim or
scale out; what survives into closure is the heading residual left at
each corner. At 500 mm legs, 1° of corner residual ≈ 10–20 mm of
closure.

Consequences:

- The reference session's 8.1/5.8/**1.0 mm** headline runs were three
  landings deep inside the trim's ±1.0° deadband (their per-turn sd
  0.12–0.30° vs ~1° elsewhere) — not a different method. Honest typical
  performance of trimmed tours today: **~8–11 mm**.
- Any work that does not reduce per-corner residual will not move
  closure. This killed two plausible-sounding knobs (§4).

## 2. The head-to-head: wheels-only vs the planner

26 interleaved runs, then 30 sequential runs, encoder-odometry closure,
rest-to-rest. Full data: `src/tests/bench/output/headtohead_*` and
`trimtol_*`.

| arm | closure |
|---|---|
| wheels + scales + per-corner trim | 7.9–13.9 mm (arm means) |
| planner as shipped, pipelined | 23.4–25.8 mm |
| planner sequential, no trim | 64.1 mm |
| **planner + the same trim grafted on** | **9.4–11.5 mm** |

Findings that matter:

1. **The planner reaches parity with exactly one added feature** — a
   per-corner heading correction against the cumulative n×90° target.
   Nothing else was changed.
2. The planner's failure mode as shipped is precise and self-cancelling:
   **+1.55°/corner over-rotation, −1.43°/leg curl-back**. Total sweep
   looks right; the square is skewed, not short. Closure-only metrics
   never see this — per-segment truth does.
3. The **pipelined no-trim tour (26 mm) beats the sequential no-trim tour
   (64 mm) by accident**: in-motion boundaries partially cancel pivot
   undershoot against leg curl. Rest-to-rest exposes the true uncorrected
   residual (~2.6°/corner). Do not read the shipped pipelined number as
   "the planner is close"; it is two errors cancelling.
4. The planner's legs are *more* repeatable than the streamed path's
   (sd 1.6 vs 4.0 mm). `profileStep()`'s measured-remaining braking is
   not the problem and needs no redesign.

Why the planner is the right base (all measured this session): 8
retryable enqueues vs ~350–450 unretryable streamed `wheels()` commands
per tour (the relay drops ~20% inbound); host-stall immunity vs a 0.4 s
deadman lease; braking decided onboard at loop rate vs 10 Hz across a
~0.10 s transport+plant delay; 24.7 s of motion vs ~36 s for the same
square.

## 3. The corner-authority floor (the constant that bounds everything)

From 333 individual trim nudges on `tovez`
(`src/tests/bench/output/trimtol_*.json`, per-nudge records):

- **The low-speed corrective pivot is bimodal: 26% deliver <0.25°
  (no breakaway), the rest a median 1.72°** (10th pct 0.63°). Stiction
  bistability, matching the reference session's creep findings.
- Therefore tolerance below the ~1.8° quantum buys nothing: 0.3° vs 1.0°
  closure is indistinguishable (7.9 vs 9.1 mm wheels; 11.5 vs 9.4
  planner), while tripling nudge count, cutting corner convergence from
  ~93% to ~64%, adding ~14 s/tour, and occasionally making corners
  *worse* (a nudge fired at a residual it cannot resolve).
- **Operating point: tolerance 1.0°** — 1.3 nudges/corner, ~2 s/corner,
  94% convergence. This lands tours at ~8–11 mm.
- **The only road below ~8 mm is a finer terminal actuator**, not a
  tighter threshold. The authority exists: the reference session's
  rejected "crawl" configuration (8 mm/s terminal crawl, floor lifted)
  measured **±1.5 mm** landings — the best accuracy anyone measured
  today — and was rejected only on profile shape. Sub-breakaway pivot
  authority is a solvable actuation problem, not a physics wall.

## 4. Measured and rejected — do not rebuild these

| candidate | verdict | evidence |
|---|---|---|
| Tighter trim tolerance | dead | §3; 0.3° ≈ 1.0° in closure, 3× cost |
| `alpha_decel` shaping (5→3.5→2.5) | dead | sim sweep: turn mean +88.55/+88.76/+88.64°, closure 52/45/55 mm — corner residual immune to turn decel |
| Wall-clock trajectory schedules | dead on arrival | `profileStep()` already plans from measured remaining with a discrete-exact staircase; both bench sessions independently confirmed gentle-and-early (decel ~120–150 mm/s²) beats steep (accuracy degrades monotonically 120→400; the velocity loop cannot follow above ~400 mm/s² and its integrator converts the miss into overshoot) |
| Shape-error controller (`e_shape`) as the *accuracy* fix | not now | closed-loop wheel tracking already measures 1.000 ± 0.010 delivered/commanded (30–500 mm/s, both wheels, both directions); an isolated sim straight runs −0.11°; the binding error is the corner, not the ratio. Revisit under load (§7) |

## 5. The production plan, in order

**5.1 One owner for rotation calibration → the cumulative-intent ledger.**
One work item, not two. `RobotLoop::handleMove()` rewrites
`Move.threshold` at ingestion (rotation-calibration inversion), which is
*why* 130-010 had to abandon intent-carry (`carryHeading_ = baseline +
threshold`) for measured-carry — the threshold stops meaning "turn 90°"
before the planner sees it (`af3ca435`'s own comment says so). Move the
calibration to the actuation side, or carry intent as its own field; then
restore the cumulative-intent ledger so corners repay residual instead of
re-anchoring to wherever they stopped. This also absorbs the third
rotation calibration currently living in a bench script's argv
(`--turn-scale 1.0363` ≠ `1/rotational_slip = 1.097` — they are not even
the same correction).

**5.2 Terminal fine-align phase in the Move (the trim, in firmware).**
Proven on hardware by the host-side graft: planner 25.8 → 9.4 mm at
tolerance 1.0°, cost ~2 s/corner, zero other changes. Fires only when
|cumulative residual| > 1.0° (below that it cannot help — §3).
Prerequisite: **5.3**.

**5.3 The `v_min` floor must not boost the planner's own decelerating
tails.** The deferred sprint-131 item; both sessions hit it
independently. `applySpeedFloor` treats a deliberately-dying profile
tail like a standing sub-breakaway command, making terminal authority
undeliverable. This is a precondition for 5.2 and for anything in 5.4.

**5.4 Sub-breakaway terminal authority (the road below 8 mm).** The
±1.5 mm crawl measurement proves the plant can land that precisely once
the floor is out of the way. Design candidates, in cheapness order:
duty-pulse nudges sized from the measured breakaway asymmetry
(d₀ ≈ 0.009–0.030 by wheel and direction); a shaped terminal crawl the
profile enters below ~20 mm/s. Bench-only development — the sim's corner
behaviour sign-flips vs hardware (−1.4° under- vs +1.55° over-rotation)
and cannot validate corner work.

**5.5 Observability: publish commanded wheel velocity.** The telemetry
wire has no commanded-velocity field (`TelemetrySecondary.cmd_vel` was
never wired; died in 124-009). The `gopiv` boundary measurement was
impossible without a debug-build workaround. Cheap, and every future
boundary/tracking question needs it.

## 6. Boundary sequencing (known, priced, second-order today)

`gopiv` (motors, no wheels) confirmed on silicon: the next Move's first
command lands in the **same control cycle** as the previous Move's DONE
ack (+0 ms, sd 0 across trials), plant still moving in 3/5. On an
unloaded rig the cost is ~0.7 points of delivered arc; the cost scales
with coast length (`tovez` closed-loop settle is ~700 ms). The
cumulative-intent ledger (5.1) converts this from a permanent error into
a repaid one, which is why it outranks re-sequencing completion itself.
If later camera data shows boundary bleed still binding, the fix is a
completion gate on measured rest — not a schedule.

## 7. Open questions, owned honestly

- **Camera truth.** Every closure number here is encoder odometry, blind
  to cross-track drift. Arm-vs-arm comparisons are sound (both blind
  identically); absolutes await a playfield run. The planner's −1.43°/leg
  curl in particular needs camera eyes — the ack-lag artifact has the
  opposite sign, so the real curl is at least as reported.
- **Load.** Everything is stand/unloaded. Stiction, breakaway, the
  crawl's ±1.5 mm, and the shape-controller question all move under
  load. The `e_shape` design (`clasi/issues/
  replace-independent-wheel-position-schedules-…`) stays parked until a
  loaded run shows ratio error binding.
- **Sim corner fidelity.** Corners sign-flip vs hardware; corner work is
  bench-only until the sim's turn model is fixed. Legs and loop dynamics
  do transfer.
- **Sim tour wobble.** ~5 mm run-to-run variance on identical config in
  the "deterministic" harness (47.3 vs 52.1 mm) — unexplained; bounds how
  finely sim closure may be read.
- **Post-swap validation: ANSWERED.** The parity level survives the
  clean plant boundary: pooled tolerance-1.0 closure was 9.3 ± 6.8 mm
  before the battery swap + brick power cycle, 11.0 ± 7.7 mm after
  (legs 499–501 mm both sides). The session's numbers were not riding on
  accumulated state. Bonus finding: **the planner's raw pivot delivery
  moved 88.89° → 90.80° across the power cycle** — ~1.9° of plant-state
  sensitivity in the pivot itself. That drift is precisely why the
  terminal correction (5.2) is the right design and a fixed calibration
  (`--turn-scale`) is not: the post-swap planner arm, with less residual
  for the trim to absorb, was the best of the day — **7.3 ± 3.6 mm,
  100% corner convergence, 0.81 nudges/corner.**
- **`tovez` end-of-session fault** was the Nezha brick losing power
  (battery died, then the swapped-in brick was left switched off) — the
  documented dead-external-I2C signature, diagnosed live by pyocd
  backtrace. Not a firmware or bus defect. The progressive travel decay
  (496→425→55→0 mm) is what a dying brick battery looks like from
  odometry; those 6 runs are quarantined in the data.

## 8. Artifacts

- Head-to-head + sweep: `src/tests/bench/output/headtohead_*`,
  `trimtol_*` (39 runs, per-nudge JSON, comparison charts)
- Plant ID (`tovez`): `motionlab_report.json`, `motionlab_{step_response,
  steady_state,ramps,reversal}.png`; sim replica `motionlab_sim_report.json`
- Boundary (`gopiv`): `gopiv_boundary.{png,csv}` + `gopiv_boundary_summary.csv`
- Sim tour harnesses: `src/tests/dev/motionlab_tour.py`,
  `motionlab_trace.py` (motion-lab worktree)
- Predictive plant: `motion-lab` worktree commits `7e48c696`, `bb9f96df`,
  `eaddfcc6`
- Bench script upgrades (main checkout, uncommitted):
  `wheels_square_tour.py` (per-nudge instrumentation, `trim_to_heading()`
  helper), `planner_square_tour.py` (`--sequential`, `--trim`, grafted
  corrections), `analyze_trimtol.py`
