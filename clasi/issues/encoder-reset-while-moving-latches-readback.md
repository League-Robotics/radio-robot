---
status: pending
---

# High-amplitude motor transients latch the Nezha encoder readback: full-speed reversals AND resets-while-moving (reliable stand repro)

## Evidence (2026-07-02 stand session, tovez, fw 0.20260701.14, USB-direct)

Five controlled arms, all detected host-side from TLM (`enc` exactly constant
while the other wheel counts / robot commanded). `D`-preemption fires
`resetEncoders()` (atomic 0x46 burst) while wheels rotate; `S`/`RT` never reset.

| Arm | Stress (every 1.2 s) | In-motion resets | Wheel speed | Guard | Result |
|---|---|---|---|---|---|
| 1 | `D +400` → `D −400` | yes | ±400 | OFF | **persistent** @ ~8 reversals |
| 2 | same | yes | ±400 | ON | **persistent** @ ~16 |
| 3 | `D +400` → `D +400` | yes | +400 | ON | **13 transient** / 10 cycles, persistent @ ~80 |
| 4 | `RT 9000` → `RT −9000` | no | ~±90 | ON | **12/12 clean** (120 reversals) |
| 5 | `S +400` → `S −400` | no | ±400 | ON | **persistent** @ ~24–32 |

Baselines: plain D/RT cycles 1 transient / 30 cycles; exact tour 0 / 6 passes.

**Conclusion: two independent sufficient triggers, both amplitude-dependent:**
(1) full-speed reversal transients (max-ΔPWM 0x60 slam while 0x46 traffic is
in flight — arm 5, no resets involved); (2) atomic resets while wheels rotate
(arm 3). Combined they latch 5–10× faster. Gentle reversals (~±90 mm/s) are
harmless. This explains the playfield-vs-stand rate gap: loaded deceleration =
larger current/PWM transients at every command boundary.

Key facts:

- **Trigger:** `Robot::distanceDrive()` calls `resetEncoders()` →
  `Motor::resetEncoder()` (3× atomic 0x46 reads + readback verify, busy-waits)
  regardless of motion state. Preempting an active move fires this burst while
  the wheels rotate under active 0x60 traffic → readback latches, typically at
  exactly the reset value (0).
- **Transient vs persistent:** an at-rest atomic reset (next D from idle,
  `ZERO enc`) re-primes and heals a transient latch. Repeated in-motion resets
  escalate to a **persistent** latch that no atomic reset clears — only a Nezha
  power-cycle (with the firmware re-running `begin()`, i.e. full reboot in
  practice).
- **Reversal accelerates ~5-10×** (persistent in 1-2 cycles vs ~10 without).
- **The IRQ guard does not protect against this flavor** (persistent latch with
  guard ON). The guard addresses the separate TWIM interrupt-load errata
  (d6d798d) and must stay ON regardless — see the companion issue
  [dbg-irqguard-query-disables-guard.md](dbg-irqguard-query-disables-guard.md).
- **`EVT enc_wedged` fired for NONE of ~18 episodes** — the detector resets on
  target==0 and its arming grace (033-005d) requires post-command movement, so
  boundary/onset latches are structurally invisible to it. The odometry wedge
  gating (033-005e) therefore never engages either.

## Proposed fixes (in order)

1. **Slew-limit PWM steps in `Motor::setSpeed`** (cap |ΔPWM| per write, e.g.
   ≤40/write, stop exempted): addresses trigger 1 (full-speed reversal slam)
   and softens boundary-decel transients generally. Note normal G/D/RT motion
   is BVC-profiled and never commands instant reversals — the exposure is PID
   dither near stop, command preemption, and raw S usage.
2. **Never fire the atomic reset while wheels may be rotating.** In
   `Robot::distanceDrive` (and any other resetEncoders caller reachable while
   moving): if the drivetrain is not at rest (targets nonzero, |vel| above
   epsilon, or within ~300 ms of a stop command), either (a) command stop and
   defer the reset until velocity ≈ 0 (bounded wait), or (b) skip the hardware
   reset and re-baseline in software only (offset = current cached reading).
3. **Detector:** count identical raw reads independent of target state and
   arming grace (or additionally); expose `wheel_wedged` in TLM.
4. **Recovery:** on detected transient latch at idle, run one at-rest
   resetEncoder re-prime automatically. (Persistent latches need a Nezha
   power-cycle + full reboot — the firmware never re-runs begin(), see KB doc.)

## Repro tool

Session harness `wedge_boundary_repro.py` (scratchpad; worth landing in
`tests/bench/`): `--slam --slam-variant {reversal,samedir,rtrev} --irqguard {0,1}`,
per-cycle alive-check (catches both-wheel persistent latches), episode detector,
JSONL logs. Full narrative:
docs/knowledge/2026-07-01-encoder-wedge-boundary-latch-flavor.md (updated with
this session's results).
