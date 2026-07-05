---
status: in-progress
sprint: 078
tickets:
- 078-001
- 078-002
- 078-003
- 078-004
- 078-005
---

# Armor the motor write path against the encoder reversal latch (zero-dwell reversal + guarded resets)

## Context

`docs/knowledge/2026-07-04-encoder-wedge.md` is the authoritative record: the
dominant encoder-wedge flavor (the **reversal latch**) is root-caused to the
**reversal write train** — an immediate H-bridge sign flip written to 0x60
while the motor is under way, including the velocity PID's sign-dither at
every decel/stop. The wedgelab campaign proved the trigger (5/5 hot +→− flips
latch on susceptible motors, chip-confirmed via raw-path cross-reads) and
proved the fix (zero-dwell reversal: **0 latches / ~75 hot susceptible flips**,
soak n=150; dose-response: 150 ms clean, 50 ms clean, **20 ms fails 12/12** —
protective threshold in (20, 50] ms).

The new greenfield tree (sprint 077) ported the OLD write path unchanged, per
that sprint's locked scope. A 2026-07-04 audit of
`source/hal/nezha/nezha_motor.cpp` confirmed the trigger is fully present:

- `writeDuty()` reversal exemption writes sign flips immediately (exempt from
  the 40 ms throttle), slew-stepped ±25 through zero across consecutive ticks
  interleaved with encoder reads — exactly the proven latch train. The ±25
  slew cap is (per the knowledge doc) *a mitigation, not the fix*.
- No zero-dwell, no ≤5 %/tick reversal ramp, no output deadband exists.
- The embedded per-motor velocity PID (new in 077) now generates sign-dither
  micro-reversals inside every leaf at decel/stop — the boundary-latch
  mechanism, now one layer closer to the metal.
- `resetPosition()` → `hardResetEncoder()` executes at the next `tick()`
  regardless of motion, violating the doc's "hard encoder resets only at
  verified standstill" guidance (mid-motion bursts escalate combination
  latches 5–10×). The new tree has no soft-rebaseline equivalent.

Flavor 2 (nRF52 TWIM errata under interrupt load) is already covered in the
new tree: `source/com/i2c_bus.cpp` ports the IRQ guard, default ON. No action
needed for it; do not regress it.

Bench tests on the friction rig are already reporting wedge flags; at-rest
`wedged=1` is benign detector semantics (see item 4), but hot reversals in the
load schedules are genuine trigger exposure.

## Placement (stakeholder correction, 2026-07-04, sprint-078 planning)

The armor is **motor-generic policy, not Nezha-specific code**. The reversal
dwell, output deadband, standstill reset guard, and wedge detection/
qualification go at the **`Hal::Motor` level** (`source/hal/capability/
motor.h`), implemented once in the base the same way `apply()`/`state()`
already are — any leaf (future SimMotor/MockMotor, other vendor motors) gets
them for free. `NezhaMotor` keeps only the device-specific primitives (the
actual duty write to the brick, the atomic 0x46 reset burst, encoder
sampling) plus Nezha-only write shaping (40 ms throttle, ±25 slew). Item
references to `NezhaMotor::writeDuty()` below should be read as "the motor
write path" with the policy layer in the base class.

## What to change (per the knowledge doc's Production guidance)

### 1. Two-phase (zero-dwell) reversal in `NezhaMotor::writeDuty()` — the fix

On any commanded **sign change** (including PID dither and DEV DUTY flips):

1. Write 0 immediately (the stop path stays immediate/unclamped — the doc
   keeps `pct == 0` exempt on purpose).
2. **Hold commanded zero for ≥ 50 ms; ship 100 ms** (`kReversalDwell`,
   conservative per the doc). During the dwell, `writeDuty()` suppresses all
   non-zero writes to that motor; the PID keeps computing, the write layer
   enforces the dwell.
3. After the dwell, write the new direction (slew from 0 as usual).

Notes:
- The dwell is per-motor state in the write path — NOT in the PID, NOT in
  Drivetrain (subsystems must not need to know about it).
- Alternative accepted by the doc if dwell latency is ever unacceptable:
  ramp ≤ 5 PWM-% per 10 ms tick through zero (≈130 ms for a ±32 flip). Default
  to the dwell; it is the stronger-proven arm.
- Consider making the dwell duration a `MotorConfig` field (default 100 ms;
  0 = legacy behavior for A/B bench comparisons only — never ship 0).

### 2. Output deadband so dither cannot request flips

Pair the dwell with a small output deadband in the write path: |pct| below a
threshold (e.g. < 3–5 %, tune on bench) writes 0 instead of a tiny signed
duty. The doc: "pair with an output deadband so near-zero dither cannot
request flips." Amplitude matters (±32 latches; ±1 dither alone did not in
the lab) — the deadband removes the flip *requests*, the dwell armors the
ones that remain.

### 3. Standstill-guarded hard resets + soft rebaseline

- `hardResetEncoder()` (the at-rest atomic 0x46 burst, median-of-3 +
  readback-verify) must run **only at verified standstill**: gate on measured
  velocity ≈ 0 AND commanded output 0 for N consecutive ticks. If reset is
  requested while moving, either (a) defer the hard reset until standstill is
  observed, or (b) perform a **soft rebaseline** immediately (software-only
  zero: fold the current reading into `encOffset_` with no I2C burst — port
  of `source_old`'s `rebaselineSoft`, 064-003) and log which happened.
- `begin()`'s boot-time hard reset stays (at rest by construction; it is also
  the 0x46 readback prime the doc requires).
- `DEV M <n> RESET` semantics follow automatically; document the
  deferred/soft behavior in `docs/protocol-v2.md` §16.

### 4. Motion-qualified wedge REPORTING (keep the detector unconditional)

The ported detector counts identical reads unconditionally — correct per the
064-004 hardening (target-gating and arming-grace were the blind spots that
made the old detector miss every real episode; do NOT reintroduce them). But
its raw latch is trivially true for any motor parked ≥10 ticks, which is what
makes bench output confusing ("wedged=1" on an idle motor).

- Keep the internal unconditional counter exactly as is.
- Add a derived, reported qualification: wedge-SUSPECT = stuck counter over
  threshold **while** |appliedDuty| is above the deadband (commanded to move)
  for the same window. Expose both if cheap (e.g. `wedged=` raw latch,
  `wsus=` motion-qualified) or switch the DEV `wedged=` field to the
  qualified form and document the raw counter elsewhere — decide in the
  sprint; either way `docs/protocol-v2.md` documents the semantics.
- Update bench scripts to assert on the motion-qualified signal; an at-rest
  raw latch is not a failure.

### 5. Regression validation on the friction rig (the acceptance gate)

Port the wedgelab methodology to the new tree's DEV protocol
(`tests/bench/`), following the doc's "always bracket with controls"
discipline and its transient-vs-persistent triage:

- **Hot-flip soak**: repeated commanded sign flips at meaningful amplitude
  (±30–50 % duty) on a motor under friction load (rig ports 3/4), n ≥ 100
  flips. Detect latches motion-armed: commanded + applied duty nonzero,
  position frozen across ≥10 reads (diagnose from state polling, never from
  the raw at-rest flag).
- **A/B**: dwell disabled (legacy config, if the config knob from item 1
  exists) must show the trigger is exercisable on susceptible hardware;
  dwell 100 ms must show 0 latches over the soak. Note the doc's caveat:
  susceptibility is motor-unit- and state-dependent (hot vs cold; fresh
  motors may be immune at every dose) — a clean legacy-arm run on immune
  motors is NOT evidence the armor works; record which motors were used and
  bracket accordingly.
- Reset guard check: request RESET mid-motion → verify deferred/soft path
  taken (no atomic burst while rotating), then hard reset completes at rest.
- Keep every run's CSV + transcript; end all sessions with DEV STOP.

## Out of scope

- The IRQ guard (flavor 2) — already ported, default ON; just don't regress.
- `source_old/` — the old tree keeps its behavior; this armors the new tree
  only. (The knowledge doc's "pending sprint ticket" for production now means
  THIS tree.)
- Wedgelab itself — remains the standalone root-cause lab; incoming-inspection
  workflow (`run reset 10`) unchanged.

## Acceptance sketch

1. `writeDuty()` enforces zero-dwell ≥50 ms (default 100 ms) on every sign
   change; stop stays immediate; deadband suppresses sub-threshold signed
   writes. Unit-testable off-hardware by inspecting the write decisions over
   a scripted command sequence (sim/host harness or a write-log hook).
2. Hard reset never issues an atomic burst while the motor is in motion.
3. `wedged` reporting is motion-qualified (or a qualified sibling field
   exists); protocol doc updated; bench scripts assert the qualified signal.
4. Friction-rig soak: 0 motion-armed latches over ≥100 hot flips with the
   armor on, controls bracketed and recorded.
5. Knowledge doc updated: reversal-latch fix status changes from "not yet in
   production" to shipped-in-new-tree, with the soak evidence linked.
