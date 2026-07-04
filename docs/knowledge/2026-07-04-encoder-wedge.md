---
date: 2026-07-04
tags: [encoder, wedge, latch, nezha, i2c, motor, reversal, zero-dwell, wedgelab,
       twim, irq-guard, odometry, ekf, telemetry, root-cause]
related-tickets: []
---

# The Nezha encoder wedge — consolidated: root cause, both flavors, fixes, diagnostics

**This is the single authoritative encoder-wedge document.** Consolidated
2026-07-04 from three now-deleted docs; any older reference to these
filenames resolves here:

- `encoder-wedge-nrf52-twim-irq-load-errata.md` (2026-06-07 — the
  interrupt-load flavor and the IRQ-guard fix)
- `2026-07-01-encoder-wedge-boundary-latch-flavor.md` (the transient
  boundary-latch flavor, detector blind spots, 07-02 stand stress matrix)
- `2026-07-04-encoder-latch-reversal-write-train.md` (the wedgelab
  root-cause campaign and the proven zero-dwell fix)

---

## Executive summary (state of knowledge, 2026-07-04)

**Terminology:** *wedge* is the family name for "encoder readback freezes
while the wheel keeps spinning"; *latch* is the dominant flavor — the Nezha
V2 brick's 0x46 readback register returns a constant stale value while I2C
transactions all still succeed (`err=0`).

Two independent, real flavors are on record. They have different triggers,
different persistence, and different fixes:

| Flavor | Trigger | Persistence | Fix | Status |
|---|---|---|---|---|
| **Reversal latch** | The reversal write train: an immediate H-bridge sign flip written to 0x60 while the motor is under way — including the velocity PID's sign-dither at every decel/stop | Transient (heals at the next **at-rest** atomic 0x46 read burst: D-start reset, `ZERO enc`); escalates to persistent under repeated abuse | **≥50 ms commanded-zero dwell** on any sign change (ship 100 ms), or ramp ≤5 PWM-%/10 ms tick through zero | Root cause isolated and fix proven on bench 2026-07-04 (wedgelab, commit `5499fa7`). **Not yet in production firmware** — production ships the ±25 ΔPWM slew cap (064-002) only; zero-dwell is a pending sprint ticket |
| **Interrupt-load wedge** | nRF52 TWIM (I2C master) silicon errata: hardware SHORTS fail under high background interrupt load (telemetry UART + radio IRQs), corrupting a transfer to the Nezha | Persistent until the Nezha's power domain cycles | **IRQ guard**: mask interrupts across each I2C transaction (`I2CBus.cpp`, `_irqGuard`, default ON; live toggle `DBG IRQGUARD 1\|0`) | A/B proven 2026-06-07. Necessary for its flavor, **irrelevant to the reversal latch** — "the guard is on" is NOT evidence against a wedge |

**Dead theories** (do not resurrect from older comments/docs — each was
tested and killed in the 2026-07-04 wedgelab campaign): bus speed
(25/100/400 kHz), write rate / write-read interleave (the sprint-015 theory
behind write-on-change + the 40 ms throttle), atomic read bursts while
moving *alone*, mixed-bus sensor traffic, IRQ load (for this flavor),
thermal.

---

## The failure signature

- One wheel's cumulative encoder is **exactly constant** across many reads
  while the wheel is commanded and physically rotating; I2C transactions
  all succeed (`err=0`, `reentry=0`). That is the brick latching its 0x46
  readback register, not a bus fault.
- Strikes at **command boundaries** (latch value = the decel landing point)
  and on reversals; L-dominant in field data. Never both wheels at once in
  field recordings.
- Latches are discrete all-or-nothing events — zero near-miss frozen
  streaks in 1,400+ frames around field episodes.
- Consequences: dead-reckoning heading corrupts (a frozen wheel injects
  phantom rotation through an entire following `RT`/`TURN`), DISTANCE stops
  starve, tours go off course.

---

## Root cause of the latch (2026-07-04): the reversal write train

### The wedgelab campaign

Standalone lab (`wedgelab/`, self-contained CODAL project), 4 motors:
M1/M2 = old latch-prone pair, M3/M4 = fresh. Dual driver: from-scratch raw
wire functions vs VERBATIM production Motor/I2CBus copies. Chip-confirmed
detector: motion-armed exact-constancy + raw-path cross-reads at each latch
+ encoder-verified stops. Every prior theory was tested against it:

1. **Bus speed — NOT causal.** Sprint-015 logs re-read 2026-07-03: the
   "~165 ticks at 400 kHz" datum was actually a 100 kHz run; 25 kHz wedged
   too. Lab ran everything at 100 kHz.
2. **Write rate / write-read interleave — NOT sufficient.** The raw driver
   wrote every 10 ms tick interleaved with reads for tens of thousands of
   transactions — zero latches, even on the old motors.
3. **Atomic read bursts while moving — NOT sufficient alone** (the 064-003
   theory behind `rebaselineSoft`). Production `resetEncoder()` mid-motion
   with no reversal: 0/20 hot cycles on the current motors. (The
   2026-07-02 stress-matrix arm-3 result did not reproduce on these
   motors; burst+reversal DOES latch, and in several runs the burst's
   at-speed reads actually RE-PRIMED the register right after a flip.)
4. **Mixed-bus sensor traffic, IRQ load, thermal — no measurable effect.**
   OTOS reads every tick: clean. Eric rules out thermal (low-power 8 V
   motors; latches never correlated with heat).
5. **THE REVERSAL WRITE TRAIN — CONFIRMED.** `Motor::setSpeed`'s reversal
   exemption writes a sign flip immediately, slew-stepped ±25 through zero
   across consecutive 10 ms ticks, interleaved with encoder reads. Isolated
   via `rebaselineSoft` arms (zero burst I2C): the reversal alone latches
   **every hot +→− flip (5/5, repeatedly)** on the old motors.
   Chip-confirmed at every latch: two raw-path 0x46 reads 60 ms apart,
   independent of the production driver, identical values while the wheel
   spins, zero bus errors (`XCHECK CHIP`, exp10). Not driver error-masking.

### What works (each validated with hot controls bracketing the run)

Controls latched 5/5 immediately before and after each clean fix arm:

- **Zero-dwell reversal (recommended):** on any commanded sign change,
  write 0 and HOLD ≥50 ms before writing the new direction.
  **0 latches / ~75 hot susceptible flips (soak n=150 cycles).**
  Dose-response: 150 ms clean, 50 ms clean, **20 ms FAILS (12/12 flips
  latched)** — the protective threshold is in (20, 50] ms. Ship 100 ms for
  margin where latency permits.
- **Gentle reversal ramp:** step the command ≤5 PWM-% per 10 ms tick
  through zero (≈130 ms for a ±32 flip). 0/25 hot cycles.
- Also clean: stop → verified standstill → hard reset → new direction (the
  right D-boundary discipline for hard encoder resets).

### Why it works (mechanism picture)

The Nezha brick's firmware cannot tolerate a drive-direction reversal
executed as an immediate H-bridge sign flip under way: the readback
register machinery latches its last value. The counter itself keeps running
internally — at-rest atomic reads later re-prime the readback and full
counts reappear, which is why the flavor is transient and why the 0x46
readback must be "primed" by a read transaction at boot (`Motor::begin`).
Giving the brick a commanded-zero window (≥50 ms) or a slow ramp through
zero lets whatever internal state the flip corrupts drain first.

Direction asymmetry (+→− latches, −→+ doesn't, per motor) and motor-unit
dependence (fresh motors immune at every dose; the old pair latches
deterministically when hot) say the susceptibility is an
electrical/firmware margin in the motor+brick channel — which is why no
host-side bus-timing mitigation ever fixed it. Amplitude matters: ±32
latches; ±1 dither alone did not latch in the lab. Susceptibility is
state-dependent (hot vs cold runs differ) — always bracket experiments
with controls.

This also explains the production **boundary latch**: the velocity PID's
sign-dither at every decel/stop emits micro-reversals through the same
throttle-exempt write path (stops and reversals write immediately, exempt
from the 40 ms throttle) — hence "latch value = decel landing point", and
why write-on-change/throttling never helped at exactly the moments that
mattered.

Transient vs persistent is a **continuum**: transient latches heal at any
at-rest atomic reset (next D from idle, `ZERO enc`); repeated abuse
escalates to a persistent latch that no in-band reset clears — only a
Nezha power-domain cycle plus full firmware reboot (`begin()` re-init).
This also cleanly explains the playfield-vs-stand rate gap: loaded
deceleration = larger current/PWM transients at every command boundary.

### Production guidance

1. **Production fix (pending sprint ticket):** two-phase reversal in the
   motor write path — on sign change, write 0, hold ≥50 ms (100 ms
   conservative), then the new direction (or ramp ≤5/tick). Keep stop
   (pct==0) immediate. Applies to ALL sign changes including PID dither;
   pair with an output deadband so near-zero dither cannot request flips.
   Until it ships, production carries the ±25 ΔPWM slew cap (064-002,
   ported to `source/hal/nezha/motor_slew.h` in sprint 077) — a
   mitigation, not the fix.
2. **Hard encoder resets only at verified standstill** (keep 064-003
   `rebaselineSoft` for in-motion rebaselines).
3. **Incoming inspection:** `wedgelab/` `run reset 10` (resetmode 1) on a
   mounted motor answers "latch-prone or clean" in ~15 s per direction.
   Fresh motors: 0 latches across the entire campaign battery.

---

## The second flavor: nRF52 TWIM errata under background interrupt load (2026-06-07)

The historical "driving wedge": mid-drive, persistent until the Nezha's
power domain cycles. Root cause is in CODAL's own driver —
`libraries/codal-nrf52/source/NRF52I2C.cpp`, `NRF52I2C::waitForStop`:

```c
// Test for condition where the SHORTS configuration appears to not trigger TASKS as expected.
// Could be an undocumented silicon errata.
// Appears to only occur under higher levels of background interrupt load.
```

The TWIM peripheral sequences a transfer with hardware SHORTS (event→task
chaining); under heavy interrupt load those SHORTS can fail to fire and a
corrupted transfer leaves the Nezha's readback latched. Production firmware
generates exactly that load: interrupt-driven async telemetry TX at
20–50 ms plus the radio relay. The raw `DBG WEDGE` harness never reproduced
it because it takes over the loop (no telemetry, no relay → low interrupt
load → clean for 10–20 min).

**Fix:** mask interrupts for the duration of each I2C transaction —
`I2CBus::write`/`read` hold `target_disable_irq()` across the underlying
`_bus.write/read` call, gated by `_irqGuard` (default **ON**), live toggle
`DBG IRQGUARD 1|0`. `target_disable_irq/enable_irq` are nest-counted in
CODAL, so this composes safely.

**A/B proof** (same robot, same session, full sensor + telemetry load,
which previously wedged at ~maneuver #20 / ~1 min on every run):

| `DBG IRQGUARD` | Result |
| --- | --- |
| **1 (on)** | 188 maneuvers, 8 min, NO ANOMALY |
| **0 (off)** | wedged at maneuver #12, 0.5 min |

**Trade-off:** each transaction masks IRQs ~hundreds of µs (≈1–1.5 ms per
control tick in short bursts). UART/radio use DMA + FIFOs so brief masking
only delays servicing; an 8-minute soak streamed telemetry cleanly. If
heavier or latency-sensitive interrupt work is ever added, re-verify comms.

**Scope caveats learned later:** the guard never took the field wedge rate
to zero — `DefaultConfig.cpp` (2026-06-17) recorded the wedge persisting at
4–12% in instrumented sweeps at 25/50/100 Hz alike, and the 2026-07-02
stress matrix showed both reversal-latch triggers fire with the guard
explicitly ON. The guard is necessary for *its* flavor only.

**Its red herrings** (each correlated only by changing interrupt load):
sensor enable/disable moved the wedge within stochastic noise; read method
(`readEncoderSettle` vs atomic, `SET encAtomic`) — no effect; write rate
was already throttled to 40 ms, and forcing a write before each read
(`Motor::reassertSpeed`) made it worse; quiet-before-read (`SET encQuiet`)
"helped" only by slowing the loop; the `I2CBus` wrapper was a thin
pass-through — not the cause, but the right *place* for the fix. The
breakthrough: `enc_watch` (production control path, all sensors off) still
wedged while raw `WedgeTest` stayed clean → same motors, same chip → the
difference was the production runtime's interrupt load.

---

## Field history (how the understanding evolved)

### 2026-07-01 recording — the boundary latch, detector-blind

`host/recordings/recording_20260701_210332.jsonl` (42 s D/RT tour, relay):
two independent single-wheel latches in ~5 command boundaries. Episode A:
right encoder latched at 748 mm between `D#9` completing and `RT 9000 #10`,
frozen 14 TLM frames (~3.25 s) through the whole turn while OTOS/pose kept
moving — **no `EVT enc_wedged`**. Episode B: left latched at −501 mm
exactly as `D#11` completed, frozen ~3.7 s, recovered at `D#14`. A third
episode during decel did fire the detector with `err=0 reentry=0` — reads
succeeding, data stale. Both latches self-healed at the next `D` (whose
`resetEncoders()` atomic 0x46 burst re-primes the register); `RT`/`TURN`
never reset encoders, which is exactly why a boundary latch poisons the
entire following turn.

This session established the signature that separates the flavors:
**transient, boundary-correlated, self-healing** vs the historical
mid-drive, persistent, power-cycle-only wedge. It also verified the
message-architecture rebase dropped no mitigation (`Motor.cpp`
byte-identical, per-tick wire order unchanged).

### 2026-07-02 stand stress matrix — triggers isolated

fw 0.20260701.14, tovez on stand. Passive repro over relay: 30×
(`D 200 200 500` → `RT 9000`) produced ONE boundary latch (R at exactly
557 mm, frozen 10.5 s, healed at next D, zero EVT). Exact TOUR_1 replay ×6:
clean. Then a five-arm stress matrix over robot-USB, guard set explicitly
per arm (`D`-preemption = re-issue D mid-flight → `resetEncoders()` atomic
burst while wheels rotate; `S`/`RT` never reset):

| Arm | Stress (every 1.2 s) | Resets while moving | Wheel speed | Guard | Result |
|---|---|---|---|---|---|
| 1 | `D +400` → `D −400` | yes | ±400 | OFF | persistent @ ~8 reversals |
| 2 | same | yes | ±400 | ON | persistent @ ~16 |
| 3 | `D +400` → `D +400` | yes | +400 | ON | 13 transient episodes / 10 cycles, persistent @ ~80 |
| 4 | `RT 9000` → `RT −9000` | no | ~±90 | ON | 12/12 clean (120 reversals) |
| 5 | `S +400` → `S −400` | no | ±400 | ON | persistent @ ~24–32 |

Verdict at the time: two amplitude-dependent triggers (full-speed reversal
transients; atomic resets while rotating), both unaffected by the IRQ
guard; gentle reversals harmless. `EVT enc_wedged` fired for none of ~18
episodes. **2026-07-04 lab correction:** arm 3 (resets-while-moving alone)
did not reproduce on the current motors — the reversal write train is the
confirmed root trigger; resets-while-moving contributes in combination
(arms 1–2 latched 5–10× faster than either alone).

---

## Detector blind spots

The `enc_wedged` detector (015-003 + 033-005d, `MotorController`
`controlTick` era) is blind exactly where the boundary latch lives — it
fired for **none** of the ~18 observed episodes across 07-01/07-02:

- It **resets on target==0** — a latch in the last decel ticks or at stop
  never accumulates its 10 counts before targets zero.
- Its **arming grace** requires the wheel to move *after* the next command
  starts — a wheel frozen *before* the command never arms it. (Episode A
  produced no EVT through a full 3 s turn for exactly this reason.)
- Consequently the odometry wedge gating (033-005e, `est.setWedgeActive`)
  never engages, and the EKF integrates garbage encoder heading through
  the whole turn.

Hardening directions filed from these sessions (count identical raw reads
regardless of target/arming grace; `wheel_wedged` in TLM; auto re-prime at
idle on detection) fed sprint 064's encoder-pipeline hardening. **Do not
trust the EVT as ground truth for "no wedge"** — diagnose from TLM (below).

---

## Diagnostics, recovery, and bench workflow

- **Flavor triage:** does the freeze clear at the next `D` from idle
  (encoder reset)? Transient ⇒ reversal-latch flavor; persistent through
  at-rest resets ⇒ escalated latch or the TWIM/interrupt flavor.
- **Diagnose from TLM, not EVT:** one wheel's `enc` exactly constant
  across ≥8 frames while `mode` is V/D and twist/OTOS move.
- **In-band recovery:** an at-rest atomic-read burst (`Motor::resetEncoder`
  — median-of-3 + readback verify — via D-start reset or `ZERO enc`)
  re-primes a transient latch. Fire it only at rest: bursts butted against
  control traffic while moving are implicated in combination latches.
- **Persistent latch recovery:** cycle the Nezha's power domain. On the
  robot, the power switch also cuts OTOS/sensors and the firmware does
  NOT re-run `begin()` — everything I2C comes back uninitialized (encoders
  read 0, OTOS frozen: looks like a super-wedge but isn't). **Recover with
  a FULL power-cycle including USB unplug** (micro:bit reboot). A robot
  booted with the rail off hangs silent on USB (boot blocks in `begin()`).
  Note a micro:bit reset/reflash alone never clears a latch — it lives in
  the Nezha's (battery-backed) power domain and persists across reflashes.
- **Bus capture:** the `I2CBus` ring buffer logs every transaction and is
  frozen on wedge detection; `DBG I2CLOG ARM` → repro → quiet telemetry
  (`STREAM 0`) → `DBG I2CLOG` dump.
- **`DBG` gotchas (verified 2026-07-02):** every `DBG` reply is
  `ForceReply::SERIAL` — over the relay they return NOTHING; use robot-USB.
  And historically a bare `DBG IRQGUARD` query DISABLED the guard
  (ArgSchema default-fill regression from 051-008, fixed in sprint 064) —
  on affected firmware, always SET the guard explicitly for A/B work.
- **Repro harnesses:** old bench tools live in `tests/old/dev/`
  (`stand_soak.py`, `enc_watch.py`, `wedge_repro.py`); the definitive
  reproducer/inspection rig is `wedgelab/` (see its README). Lab data:
  `wedgelab/out/*exp09..14*`; experiment scripts `wedgelab/exp/09..14*`.
- **mbdeploy "device not connected" while `probe` lists it:** stale device
  registry — delete the registry file and re-`probe`.

---

## Related

- [i2c-sensor-detection-and-bus-wedge.md](i2c-sensor-detection-and-bus-wedge.md)
  — the separate **cold-boot detection** wedge (`begin()` placement,
  battery-backed bus keeping a wedged slave alive across reflashes).
- [encoders-read-zero-i2c-bus-hang.md](encoders-read-zero-i2c-bus-hang.md)
  — encoders/sensors reading zero at boot (sensor-detection placement).
- [2026-07-02-d-drive-terminal-instability-reversal-thrash.md](2026-07-02-d-drive-terminal-instability-reversal-thrash.md)
  — the D-drive terminal reversal thrash (a *control* defect; its
  stop-boundary reversals are also exactly the latch trigger).
- [watchdog-uint32-underflow-velocity-notches.md](watchdog-uint32-underflow-velocity-notches.md)
- Issues: `clasi/issues/encoder-wedge-corrupts-tour-legs.md` (field impact
  + wedgelab campaign log); sprint 064 `issues/done/` for the
  reset-while-moving and IRQGUARD-query issues.
- Recording: `host/recordings/recording_20260701_210332.jsonl` (episodes at
  t+15.28 R@748 and t+25.23 L@−501; EVT at t+22.30).
