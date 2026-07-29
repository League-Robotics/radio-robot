# Motion profile exploration — deceleration is 2–4× slower than acceleration

Exploration only, 2026-07-28, tovez on the stand. No firmware changed.
Started as "why are pivots asymmetric"; the pivot asymmetry was resolved
separately (pivot rate was inside the dead zone — see
`planner_square_tour.py`'s own `OMEGA` comment). This is what the follow-up
measurement found instead.

## Headline

**Deceleration takes 2–4× longer than acceleration on every move, legs and
turns alike.** Acceleration matches prediction; deceleration does not.
Every move also ends with a ~0.33–0.42 s tail creeping below 30 mm/s.

Turns merely *look* worse than legs because they are shorter, so a roughly
fixed 1.0–1.8 s deceleration is a larger fraction of the move.

## Measurements

Per-move phase durations, measured from telemetry (accel = start until
within 90 % of peak; decel = last sample within 90 % of peak until motion
ceases):

| move | duration [s] | accel [s] | plateau [s] | decel [s] | tail <30 mm/s [s] | decel/accel |
|---|---|---|---|---|---|---|
| turn 45° | 0.84 | 0.33 | 0.00 | 0.50 | 0.33 | 1.50× |
| turn 90° | 1.49 | 0.39 | 0.11 | 0.99 | 0.38 | 2.52× |
| turn 180° | 2.89 | 0.39 | 0.86 | 1.63 | 0.38 | 4.20× |
| turn 360° | 3.56 | 0.39 | 1.59 | 1.59 | 0.38 | 4.06× |
| omega 1.2 (90°) | 1.79 | 0.17 | 0.06 | 1.56 | 0.23 | 9.24× |
| omega 2.4 (90°) | 1.49 | 0.44 | 0.11 | 0.93 | 2.11× | 2.11× |
| omega 3.6 (90°) | 1.39 | 0.44 | 0.06 | 0.88 | 0.42 | 1.99× |
| leg 200 mm | 2.46 | 0.45 | 0.32 | 1.68 | 0.38 | 3.74× |
| leg 500 mm | 4.51 | 0.45 | 2.26 | 1.80 | 0.33 | 4.00× |

Against configured limits (`a_max = a_decel = 800 mm/s²`,
`alpha_max = alpha_decel = 7.0 rad/s²`), acceleration and deceleration
should be symmetric. Measured acceleration (~0.39–0.45 s) is close to the
predicted 0.343 s; measured deceleration is not close to anything.

Total durations run 30–60 % over prediction:

| move | predicted [s] | measured [s] | over |
|---|---|---|---|
| turn 90° @ 2.4 | 1.00 | 1.49 | +49 % |
| leg 200 mm | 1.52 | 2.46 | +62 % |
| leg 500 mm | 3.52 | 4.51 | +28 % |

The profiler **does** produce a cruise phase when there is room — 0.86 s at
180°, 1.59 s at 360°, 2.26 s on a 500 mm leg. Short moves lose it because
the long deceleration eats the budget, not because the profiler declines to
plan one.

## The blocking instrumentation gap

**There is currently no telemetry of commanded wheel velocity**, so
"the planner commanded a bad profile" cannot be distinguished from "the
plant failed to follow a good one".

- `Telemetry.twist` is documented in `telemetry.proto` as *"body twist from
  measured wheel velocities"* and is populated from `state.pose.v_x/omega`
  (`telemetry.cpp:92`). It is a **measured** signal.
- `RobotState::Command::v_x/omega` exist but are **unwired**, permanently
  0.0 — documented as such in `robot_state.h`.
- `Telemetry.cmd_vel` (the real per-wheel setpoint) lives on
  `TelemetrySecondary` and is flagged a "permanent gap" — not decoded by
  the host.

Consequence worth fixing on its own: **`planner_square_tour.py` plots a
dashed trace labelled "commanded" that is actually the measured signal**
(it reconstructs per-wheel speeds from `f.twist`). Every tour chart showing
commanded and measured tracking perfectly is showing one signal twice. That
is not evidence of good tracking and should not be read as such.

## RESOLVED — the cause is `decelPlanFraction = 0.4`

Everything below this heading was written before reading the profiler. The
code answered it outright, and **candidate 1 below is wrong**: the braking
law *is* a proper discrete constant-deceleration ramp, not a P-taper.

`profileStep()` (`profile.cpp`) binary-searches each tick for the highest
velocity that still leaves room to brake to the boundary, and the
feasibility test uses `planDecel(limits)` — which returns
`min(aDecelPlan, aDecel)`. So the commanded ramp is governed by
**`aDecelPlan`, not `aDecel`**.

`aDecelPlan = aDecel * decelPlanFraction` (`shape.cpp`), and
`decelPlanFraction` is hardcoded at **0.4** in `main.cpp:380`. Deceleration
therefore runs at 320 mm/s² while acceleration runs at 800 — 2.5× longer by
construction, against the 2.0–4.2× measured.

That also explains the tail: the ramp follows v ≈ √(2 · 320 · remaining), so
at 1 mm remaining it commands 25 mm/s and at 0.1 mm it commands 8 mm/s. The
sub-30 mm/s creep at the end of every move IS the √-taper approaching zero —
and it lands squarely inside the dead zone, where this plant stops
responding repeatably. Candidate 2 compounds it rather than causing it.

**This is deliberate, not a defect.** `profile.h`'s own comment says the
planner "brakes sooner and rides a gentler ramp, holding
(aDecel - aDecelPlan) in reserve", and `square_tour_velocity.py` records the
sweep that chose it: *"0.4 gave the best closure (16.0 mm vs 23.3 mm at full
authority) because the plant can actually TRACK the gentler ramp down."*

Two caveats on that justification: `decelPlanFraction` is a **config
constant living in `main.cpp`**, which the project's own config-as-truth
rule says belongs in robot JSON; and the sweep that justified it no longer
reproduces (below).

## Sim reproduces it — iteration no longer needs hardware

`src/motion/planner/bench/square_tour_velocity.py` drives the real planner
through ctypes against a simulated plant, logs commanded AND measured per
tick, and closes at 19.70 mm against hardware's 18.6 mm. Its shared library
was stale (missing `plannerEstop`); rebuild with
`cmake -S src/motion/planner -B src/motion/planner/build && cmake --build
src/motion/planner/build`.

Sweeping `decelPlanFraction` there (deterministic, one run each):

| leeway | accel [s] | hold [s] | decel [s] | decel/accel | closure [mm] | duration [s] |
|---|---|---|---|---|---|---|
| 0.0 / 1.0 (full) | 3.95 | 14.57 | 2.91 | 0.74 | 21.24 | 22.42 |
| 0.2 | 3.95 | 8.04 | 13.58 | 3.44 | 22.47 | 26.55 |
| **0.4 (current)** | 3.95 | 12.17 | 6.82 | 1.73 | 19.70 | 23.92 |
| **0.6** | 3.95 | 13.54 | 4.65 | 1.18 | **18.52** | **23.12** |
| 0.8 | 3.95 | 14.29 | 3.57 | 0.90 | 20.21 | 22.79 |

(0.0 and 1.0 are identical: the code treats both as full authority.)

Findings:

- The knob directly sets deceleration time — 13.58 s down to 2.91 s.
- Closure is a shallow **U**, not monotonic: the accuracy argument for
  slowing the ramp is real but weak.
- **0.6 dominates 0.4 on every axis** — better closure, shorter duration,
  less decel drag.
- At full authority deceleration is *faster* than acceleration (0.74×),
  because acceleration is jerk-limited and braking is not.
- The original justification does **not** reproduce: the sweep that picked
  0.4 recorded 16.0 mm vs 23.3 mm at full authority (7.3 mm spread); this
  sweep spans only ~4 mm total, and full authority costs 2.7 mm rather than
  7.3. The tour has changed since (completion epsilon, pivot rate), so that
  tuning was fitted to a configuration that no longer exists.

Caveat: the sim is deterministic and this is one run per setting, so there
is no spread to judge whether 18.52 vs 19.70 is meaningful. Do not treat the
U's minimum as precisely located.

## Candidate causes (superseded by the section above — kept for the record)

1. **The decel law is a taper, not a constant-deceleration ramp.** If the
   commanded speed is driven by remaining distance (a P-like law, or
   `VelocityShaper`'s taper toward the stop threshold), the approach is
   asymptotic and produces exactly this shape: a long slow tail rather than
   a straight ramp landing at zero. `a_decel` would then be a ceiling that
   is never the binding constraint.
2. **The dead zone.** Below ~120 mm/s this plant is not repeatable (four
   identical trials at 76.8 mm/s spread 26–53 mm/s; one wheel sometimes did
   not turn at all). `crawl_pulse` is 0, so the Bresenham pulsing in
   `App::Drive::crawlDuty()` that exists for exactly this region is off. Any
   commanded taper passing through this band will crawl.
3. **Actuation lag.** ~120–140 ms of command-to-wheel latency adds to the
   tail but cannot explain a 1.0–1.8 s deceleration on its own.

(1) and (2) are not exclusive and probably compound: a taper spends its last
phase inside the band where the plant stops responding predictably.

## Options

**A. Wire up commanded-velocity telemetry.** Prerequisite for everything
else, and it fixes the mislabeled tour chart. Either wire
`RobotState::Command::v_x/omega` from the planner's staged command, or decode
the existing `cmd_vel` secondary field. Small, low-risk, and converts this
entire question from inference to measurement.

**B. Inspect the deceleration law directly.** Read what
`Motion::VelocityShaper` / the planner's braking phase actually computes for
commanded speed vs remaining distance. Cheap (no hardware), and would confirm
or kill candidate 1 outright. Best done together with A.

**C. Re-enable `crawl_pulse`.** Addresses the sub-30 mm/s tail, and would
also help the completion-epsilon undershoot found earlier today and low-speed
motion generally. Needs the breakaway amplitude re-measured first — the old
0.20 was sized against a criterion the standalone prober disproved
(`src/tests/firmware/duty_min/RESULTS.md`), which is why it currently ships
at 0.

**D. ~~Raise `a_decel`/`alpha_decel`~~ — WRONG, would do nothing.** Confirmed
by reading the profiler: `a_decel` only sets a non-binding floor; the ramp is
governed by `aDecelPlan`. **The actual lever is `decelPlanFraction`**
(`main.cpp:380`, currently 0.4). Sim says 0.6 is better than 0.4 on closure,
duration, and decel time simultaneously. Raising it is a one-line change and
the single highest-value experiment left — but it should also be *moved out
of `main.cpp` into robot config* while being touched, per the project's
config-as-truth rule.

**E. Accept it and shorten the tail at the completion end.** The completion
epsilon was already widened from 1 µm to 1.0 mm today, which removed the
30 s hangs. Widening the arrival criterion further, or completing on
"profile exhausted AND at rest", would truncate the tail without touching the
profile law. Treats the symptom, but is low-risk and independent of the
others.

Recommended order: **A + B together** (they answer the question), then **C**
(the dead zone is implicated in several unrelated defects and is worth
fixing properly rather than dodging), with **D** as a quick experiment only
if B shows the ramp is decel-limited.

## Not investigated

- Whether the same asymmetry appears in the sim domain. If it does, this is
  reproducible without hardware and much cheaper to iterate on.
- The peak overshoot (measured peaks ran 8–20 % above target at omega 1.2
  and 2.4, but *under* target at omega 3.6 — 206 vs 212). Unexplained.
- Whether the ~0.38 s tail duration is constant by coincidence or because it
  is set by a fixed number of control cycles.
