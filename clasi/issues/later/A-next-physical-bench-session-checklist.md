---
status: pending
priority: high
---

# Next physical bench session: the checklist of everything blocked on a human being present

## Description

Consolidates three issues (2026-08-02 triage) that all wait on the same
thing — a person physically at the bench — plus the two sprint-130 bench
criteria that were never re-measured because nobody was there to power-cycle
a wedged robot. They were three separate open issues describing one session.

**Item 1 gates every other item.** The robot has been unusable since
2026-08-01.

## 1. GATE — get `tovez` responding again (I2C-bus wedge)

Four hard wedges in two days, each needing a physical power cycle
(was: `tovez-hard-silent-i2c-wedge-blocks-completing-the-population-duty-sweep.md`).

Signature: banner + a few telemetry frames, then total silence; PC spinning
in `codal::system_timer_wait_cycles` via `system_timer_current_time`; recurs
on every boot including a hard DTR reset, so it is a persistent physical
condition, not a software glitch. Root-caused previously (2026-07-25) to a
dead motor-brick battery starving the external I2C bus's pull-ups —
`Devices::MicroBitI2CBus::write` → `NRF52I2C::waitForStop` with IRQs masked,
so even serial DMA stops.

- Check / recharge / replace the motor-brick battery; verify the external
  I2C bus is powered.
- Confirm `PING` + telemetry before starting anything below.
- Diagnostics are read-only per `.claude/rules/debugging.md` — `halt`/`reg`
  and `gdb backtrace` only, never register writes.

Related observability follow-up (not blocking): `Preamble::probeSlot` has no
timeout, so the most common bench condition (motor power off) produces total
silence and needed a debugger four times. Bounding and reporting it would
turn this whole class into an error message.

## 2. Positive-case wheel-frozen flag (needs a hand on a wheel)

Sprint 129 ticket 002 verified the NEGATIVE case on hardware (a healthy
700 mm leg raised neither flag over 143 frames). The positive case needs a
person to hold a wheel stationary while it is commanded — an autonomous
session cannot do it, and the agent declined to fabricate a pass.

```bash
uv run python src/tests/bench/wheel_frozen_bench.py \
    --port <tovez port> --stall-wheel left     # then: right
```

Expected: the matching per-wheel flag sets within ~0.5 s and
`wheel_frozen_reason()` names that wheel. Matters beyond visibility — 129-007's
duty-per-speed learner uses this flag as a guard, so if the positive case does
not fire, that guard is silently absent.

## 3. Complete the population duty sweep (right wheel + simultaneous grid)

Cut short by item 1 after 45 good rungs. Captured: a COMPLETE left-wheel
characterization, both directions, full 0.04–1.0 duty range. Missing: the
right wheel's full range (only 3 forward rungs at duty ≤ 0.242, NO reverse
data) and the entire simultaneous-both-wheels grid (zero rungs).

Everything resting on the missing data is flagged LOW CONFIDENCE (n=3) in
`data/robots/tovez.json`'s `control._drive_calibration_note` and must not be
treated as settled: `vMin` 99.7, `biasMax` ±23.8, breakaway band 0.04–0.09,
and — most consequentially — the parallel-lines verdict, which came back
**slope-dominated (fanned)**, the OPPOSITE of the intercept-only hypothesis
Stage C's bias adaptation is built on. That verdict is the input to the
2026-08-02 review's F2 (adapt the slope, not the intercept).

```bash
uv run python src/tests/bench/duty_sweep.py --port <tovez port>
# --from-csv re-runs the fit/population analysis against a combined dataset
# without re-driving the good left-wheel rungs
```

Same session should also cover 129-006's freshly-charged-battery re-run
(separating a hard power-delivery ceiling from a battery-sag artifact) and
the multi-motor population campaign (physically swapping motors).

## 4. Playfield actuation-floor measurement (deferred from 130-012)

Needs the chassis **translating under its own weight** — the stand cannot
substitute, because the actuation floor is a *loaded* property and the OTOS
reports nothing useful with a static scene under it (this caused a false
"frozen sensor" report during sprint 130).

- Actuation floor per wheel, per direction, robot translating.
- Termination tolerance derived from that measurement instead of assumed —
  `settle_epsilon_linear` currently promises 4 mm arrival while vMin 99.7
  delivers ~5.4 mm of travel per 54 ms tick.
- Answer the parallel-lines question with n well above 3.
- The two +500 criteria that need translation (net heading change ≤ 3 deg,
  camera-measured travel 500 ± 25 mm), deferred by 130-006.

Playfield rules apply: camera-supervised, lights checked first, geofence
checked INSIDE the move at ~10 Hz, `estop()` never `stop()` on every halt
path (`.claude/rules/playfield-testing.md`).

## 5. Sprint-130 criteria that were never re-measured on the 50 ms firmware

From the closed `bench-reverify-residuals-and-the-4ms-delivered-period-offset.md`.
Ticket 006's numbers were all taken on the 40 ms build; the longer delivered
period plausibly moves rise time and ripple in either direction, which is the
interesting variable.

- **Bench square-tour closure after the heading-hold fix.** The fix is
  flashed (`v0.20260802.1`, `heading_hold_gain: 2.0 → 0.0` in `tovez.json`);
  its confirming tour never completed. Re-run
  `src/tests/bench/planner_square_tour.py`.
- **+500 / saturation / bias on the 50 ms firmware.** Re-run
  `src/tests/bench/wheel_controller_ab_bench.py`. Prior (40 ms build):
  endpoint 510/502 mm split 8 mm PASS; transients FAIL (rise 0.59 s vs ≤0.3,
  ripple L 24.3 / R 22.5 vs ≤10, |vL−vR| 20.9 vs ≤10); Stage C bias converged
  to +14.0 mm/s right over a 90 s hold; saturation L 571.7 / R 532.5 against
  a historical 760–795 — a directly-measured ~25% drop, unattributed (no
  pack-voltage telemetry exists to confirm or rule out battery state; per
  project convention that is an open finding, not a power narrative).
  Note the 2026-08-02 review's Part 6 argues the ripple and split bounds sit
  BELOW the actuator floor (1 duty LSB ≈ 8–11 mm/s), so this run is input to
  a spec decision, not a tuning target.
- **Live `pid.kp` tuning round-trip** — code-verified only
  (`Configurator::applyMotorConfigPatch` still routes into
  `Drive::setControlGains()`), never re-exercised on the wire.

## Verification

Each numbered item above has its own pass criteria; the session is complete
when items 1–3 are done and 4–5 are either done or explicitly deferred with
a reason. Record measurements in `data/robots/tovez.json`'s calibration note
and the relevant bench CSV, as the existing sweeps do.

## Related

- Consolidated 2026-08-02 from `tovez-hard-silent-i2c-wedge-...`,
  `hitl-confirm-wheel-frozen-flag-...`, and
  `playfield-actuation-floor-measurement-deferred-from-130-012` (all closed
  into this file), plus the bench residuals of
  `bench-reverify-residuals-and-the-4ms-delivered-period-offset` (closed).
- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` — F2 (slope
  vs intercept), Part 6 (what the hardware can actually do), and the
  probeSlot-timeout note.
