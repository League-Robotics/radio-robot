---
date: 2026-07-04
tags: [encoder, wedge, latch, nezha, i2c, motor, reversal, zero-dwell, wedgelab, root-cause]
related-tickets: []
---

# Encoder latch ROOT CAUSE: the reversal write train — and the ≥50 ms zero-dwell fix

**Status: root cause isolated and fix proven on bench (wedgelab campaign,
2026-07-04, commit 5499fa7). This doc SUPERSEDES the causal theories in**
[2026-07-01-encoder-wedge-boundary-latch-flavor.md](2026-07-01-encoder-wedge-boundary-latch-flavor.md)
(write/read-interleave attribution) **and the bus-speed/write-rate theories
referenced by** [encoder-wedge-nrf52-twim-irq-load-errata.md](encoder-wedge-nrf52-twim-irq-load-errata.md).
The July-1 doc's *observations* (boundary-correlated, transient, detector-blind,
heals at at-rest resets) all stand; its *mechanism hypothesis* is replaced.

## Problem

Nezha V2 motor encoder readback (register 0x46) freezes at a constant value
while the wheel keeps spinning — the "wedge/latch" family that has corrupted
odometry, DISTANCE stops, and tours since sprint 015, surviving every prior
mitigation (100 kHz bus, IRQ guard, write-on-change, 40 ms write throttle,
slew cap, soft rebaseline).

## Symptoms

- One wheel's cumulative encoder exactly constant across many reads while
  commanded and physically rotating; I2C transactions all succeed (err=0).
- Strikes at command boundaries (latch value = the decel landing point) and
  on reversals; L-dominant in field data.
- Transient: heals at the next at-rest atomic read burst (D-start reset,
  ZERO enc); escalates to persistent under repeated abuse.

## What Was Tried (and what the lab showed)

The wedgelab campaign (standalone `wedgelab/` project; 4 motors: M1/M2 =
old latch-prone pair, M3/M4 = fresh; dual driver: from-scratch raw wire
functions vs VERBATIM production Motor/I2CBus copies) tested every prior
theory against a chip-confirmed detector (motion-armed exact-constancy +
raw-path cross-reads at each latch + encoder-verified stops):

1. **Bus speed** — 25/100/400 kHz historical claims: NOT causal. (Sprint-015
   logs re-read 2026-07-03: the "~165 ticks at 400 kHz" datum was actually a
   100 kHz run; 25 kHz wedged too. Lab: everything at 100 kHz.)
2. **Write rate / write-read interleave** (sprint-015 theory behind
   write-on-change + 40 ms throttle): NOT sufficient. The raw driver wrote
   every 10 ms tick interleaved with reads for tens of thousands of
   transactions — zero latches, even on the old motors.
3. **Atomic read bursts while moving** (064-003 theory behind
   rebaselineSoft): NOT sufficient ALONE on these articles. Production
   `resetEncoder()` mid-motion with no reversal: 0/20 hot cycles. (The
   2026-07-02 stress-matrix arm-3 result is not reproduced on the current
   motors; burst+reversal DOES latch, and in several runs the burst's
   at-speed reads actually RE-PRIMED the register right after a flip.)
4. **Mixed-bus sensor traffic, IRQ load, thermal** — no effect measurable
   in the lab (OTOS reads every tick: clean; Eric rules out thermal —
   low-power 8 V motors, latches never correlated with heat).
5. **THE REVERSAL WRITE TRAIN — CONFIRMED.** `Motor::setSpeed`'s reversal
   exemption writes a sign flip immediately, slew-stepped ±25 through zero
   across consecutive 10 ms ticks, interleaved with encoder reads. Isolated
   via `rebaselineSoft` arms (ZERO burst I2C): the reversal alone latches
   **every hot +→− flip (5/5, repeatedly)** on the old motors.
   Chip-confirmed at every latch: two raw-path 0x46 reads 60 ms apart,
   independent of the production driver, identical values while the wheel
   spins, zero bus errors (`XCHECK CHIP`, exp10). Not driver error-masking.

## What Worked

Two independent fixes, each validated with hot controls bracketing the run
(controls latched 5/5 immediately before and after the clean fix arms):

- **Zero-dwell reversal (recommended)**: on any commanded sign change,
  write 0 and HOLD ≥50 ms before writing the new direction.
  **0 latches / ~75 hot susceptible flips (soak n=150 cycles).**
  Dose-response: 150 ms clean, 50 ms clean, **20 ms FAILS (12/12 flips
  latched)** — the protective threshold is in (20, 50] ms. Ship 100 ms
  for margin where latency permits.
- **Gentle reversal ramp**: step the command ≤5 PWM-% per 10 ms tick
  through zero (≈130 ms for a ±32 flip). 0/25 hot cycles.

Also clean: stop → verified standstill → hard reset → new direction
(the right D-boundary discipline for hard encoder resets).

## Why It Works

The Nezha brick's firmware cannot tolerate a drive-direction reversal
executed as an immediate H-bridge sign flip under way: the readback
register machinery latches its last value (the counter itself keeps
running internally — at-rest atomic reads later re-prime the readback and
full counts reappear). Giving the brick a commanded-zero window (≥50 ms)
or a slow ramp through zero lets whatever internal state the flip corrupts
drain first. Direction asymmetry (+→− latches, −→+ doesn't, per motor)
and the motor-unit dependence (fresh motors immune at every dose; the
old pair latches deterministically when hot) say the susceptibility is an
electrical/firmware margin in the motor+brick channel — which is why no
host-side bus-timing mitigation ever fixed it.

This also explains the production **boundary latch**: the velocity PID's
sign-dither at every decel/stop emits micro-reversals through the same
exempt-write path — hence "latch value = decel landing point" and why
write-on-change/throttling (which exempt stops and reversals!) never
helped at exactly the moments that mattered.

## Future Guidance

1. **Production fix** (sprint ticket): two-phase reversal in
   `Motor::setSpeed` — on sign change, write 0, hold ≥50 ms (100 ms
   conservative), then the new direction (or ramp ≤5/tick). Keep stop
   (pct==0) immediate. Applies to ALL sign changes including PID dither;
   pair with an output deadband so near-zero dither cannot request flips.
2. **Hard encoder resets only at verified standstill** (keep 064-003
   rebaselineSoft for in-motion rebaselines).
3. **Incoming inspection**: `wedgelab/` `run reset 10` (resetmode 1) on a
   mounted motor answers "latch-prone or clean" in ~15 s per direction.
   Fresh motors: 0 latches across the entire campaign battery.
4. **Do not trust** write-rate/bus-speed/interleave explanations in older
   comments and docs — deleted/superseded 2026-07-04. The trigger is the
   reversal write train; the amplitude matters (±32 latches, ±1 dither
   alone did not in the lab); susceptibility is per-motor-unit and
   state-dependent (hot vs cold runs differ — bracket experiments with
   controls).
5. Lab data: `wedgelab/out/*exp09..14*`; experiment scripts
   `wedgelab/exp/09..14*`; usage `wedgelab/README.md`.
