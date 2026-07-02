---
date: 2026-07-01
tags: [encoder, wedge, nezha, i2c, wedge-detector, odometry, ekf, telemetry, motor-throttle]
related-tickets: []
---

# The encoder wedge, revisited: a transient boundary-latch flavor the detector cannot see

**Status:** analysis complete, mechanism hypothesis NOT yet bench-proven (I2CLOG capture
pending). This doc **supersedes parts of**
[encoder-wedge-nrf52-twim-irq-load-errata.md](encoder-wedge-nrf52-twim-irq-load-errata.md):
specifically the claims that the wedge is persistent-until-power-cycle and that the
IRQ guard eliminated it. Those claims are contradicted by July-1 field data and by
the project's own config history.

---

## Problem

After the sprint 047→060 message-based/ordered-tick rebase, "the encoder wedge is
back": during test-GUI tours (D / RT sequences over the radio relay), an encoder
freezes while the robot drives, dead-reckoning heading corrupts, and tours go off
course. The prior knowledge base said this wedge was root-caused (nRF52 TWIM errata
under interrupt load), fixed (IRQ-guarded I2C transactions), and clearable only by
power-cycling the Nezha. An evidence-first re-analysis of the code and of
`host/recordings/recording_20260701_210332.jsonl` shows the current failure does not
match that description.

## Symptoms (from the 2026-07-01 21:03 recording — 42 s, D/RT tour, relay session)

Two independent single-wheel latches in ~5 command boundaries:

- **Episode A:** right encoder latched at 748 mm at t+15.28 — *after* `D#9`
  completed (t+15.13) and *before* `RT 9000 #10` was issued (t+15.38). Frozen for
  14 consecutive TLM frames (~3.25 s) spanning the whole turn. OTOS and pose kept
  updating (robot really rotating); reported right-wheel velocity collapsed to 0.
  **No `EVT enc_wedged` fired.**
- **Episode B:** left encoder latched at −501 mm exactly as `D#11` completed;
  frozen ~3.7 s through `RT 9000 #13`; recovered at `D#14`.
- A third stuck episode during `D#11`'s deceleration *did* fire the detector:
  `EVT enc_wedged wheel=L enc=496 raw=496 n=10 err=0 reentry=0 lastErr=0`
  — I2C reads succeeding (`err=0`, `reentry=0`) but returning stale data. That is
  the Nezha latching its 0x46 readback register, not a bus fault.
- Both latches **self-healed at the next `D` command**. Never both wheels at once.
  No power-cycle, no reconnect, no re-ZERO occurred mid-run.

Key signature vs the historical wedge: **transient, boundary-correlated, and
self-healing** — the historical TWIM wedge was mid-drive, persistent, and cleared
only by Nezha power-cycle.

## What Was Tried (prior beliefs, and what the evidence says now)

1. **"The IRQ guard eliminated the wedge" (2026-06-07).** The guard is intact and
   default-ON (`I2CBus.cpp`, `_irqGuard(true)`, masks IRQs across the full
   transaction). But `source/robot/DefaultConfig.cpp` (commit `db11b7c`,
   2026-06-17 — ten days later) records: *"Instrumented field sweeps showed the
   TWIM encoder wedge is roughly RATE-INDEPENDENT (4-12% at 25/50/100Hz alike)"*.
   The smoke rituals of 06-20 and 06-27 (pre-rebase) failed RT×4 closure and
   G-square. **The wedge never actually went to zero pre-rebase**; "we got away
   from it" was partly exposure/perception.
2. **"The rebase must have dropped a mitigation."** Verified false, item by item.
   Survived intact: IRQ guard (default ON), 100 kHz bus (`main.cpp`),
   both-encoders-every-tick R-before-L (`NezhaHAL::tick`), write-on-change + 40 ms
   write throttle (`Motor::setSpeed`), coast-via-0x60-speed-0 (never 0x5F),
   busy-wait settle windows, wedge detector + arming grace
   (`MotorController::controlTick`), odometry wedge gating
   (`Drive::tickUpdate` → `est.setWedgeActive`). `Motor.cpp` is **byte-identical**
   across the rebase; per-tick wire order (enc reads → PID write → OTOS →
   line/color) unchanged. The only substantive control change in the window is
   049-003 (VelocityController recomposed over cmon-pid) — subtle integrator
   dynamics at decel, worth an A/B, but the sign/clamp law did not change.
3. **"Only a Nezha power-cycle clears it."** Contradicted: both episodes healed at
   the next `D`, which calls `Robot::resetEncoders()` →
   `Motor::resetEncoder()` ×2 → 8+ atomic 0x46 reads. Atomic reads **re-prime**
   the latched readback register — the same mechanism that un-freezes the register
   at boot (`Motor::begin`) and that `velocity_chart` exploited (commit `0897696`
   "zero-encoders on start to clear I2C wedge"). `RT`/`TURN` never reset encoders,
   which is exactly why a boundary latch poisons an entire following turn.

## What the analysis found (the durable facts)

### 1. Latch onset is at D-deceleration/stop, not mid-cruise

All three episodes latched during the final deceleration ticks or at the stop of a
`D` command. This is where the anti-wedge write throttle is systematically
bypassed: in `Motor::setSpeed`, a **stop** (pct==0) and a **reversal** (sign flip)
write immediately, exempt from the 40 ms throttle. Near stop, the PID output
rounds through int8 as +1/0/−1 dither, and the output clamp is bidirectional
(`clamp(rawPwm, ±100)` — the header's "sign follows setpoint" doc is stale), so
decel can emit sign-flipping writes every control tick — recreating the
sprint-015-proven wedge trigger ("high-frequency 0x60-write/0x46-read interleave")
in a corner case. **Hypothesis, not yet proven** — `DBG I2CLOG ARM` + dump at a
captured latch is the proof/refutation.

### 2. The detector is blind exactly where this flavor lives

`MotorController::controlTick` wedge detector (015-003 + 033-005d):

- **Resets on target==0** — a latch in the last decel ticks or at stop never
  accumulates its 10 counts before targets zero.
- **Arming grace** requires the wheel to move *after* the next command starts — a
  wheel frozen *before* the command never arms the detector. Episode A produced no
  EVT through a full 3 s turn for exactly this reason.
- Consequently `wheelWedgedL/R()` stays false, so the **033-005e odometry
  hardening never engages**: the EKF integrated garbage encoder heading through
  the entire RT 9000.
- `wheel_wedged` exists in `msg::DrivetrainState` but is **not emitted in TLM** —
  the host's only signal is the (blind-spotted, relay-droppable) EVT.

### 3. Recovery is in-band and cheap

An atomic-read sequence (`Motor::resetEncoder`, median-of-3 + readback verify)
clears the latch. Any recovery design can use re-priming instead of power-cycle —
with care, since firing atomic reads from the comms path butted against control
reads is itself a documented wedge trigger.

## Why It Works (mechanism picture)

The Nezha's 0x46 encoder readback is a register that must be "primed" by a read
transaction and can latch stale under adversarial write/read timing (vendor's own
4 ms wait comments, boot-freeze behavior). Two distinct latch inducers are now on
record: (a) TWIM-errata transaction corruption under interrupt load (the 2026-06-07
persistent flavor — guarded against, possibly not perfectly), and (b) tight
0x60-write/0x46-read interleave (the sprint-015 flavor — throttled, except at
stop/reversal). The boundary latch is consistent with (b) escaping through the
throttle exemptions, and its transience is consistent with the register-prime
model: the next atomic-read burst re-primes it.

## Future Guidance

1. **Don't trust "wedge = power-cycle".** Check transience first: does the freeze
   clear at the next `D` (encoder reset)? Transient ⇒ boundary-latch flavor;
   persistent ⇒ the TWIM/interrupt flavor.
2. **Don't trust the EVT as ground truth for "no wedge".** The detector cannot see
   latches that begin at command boundaries. Diagnose from TLM: one wheel's `enc`
   exactly constant across ≥8 frames while `mode` is V/D and twist/OTOS move.
3. **Repro/proof workflow:** `DBG I2CLOG ARM` → run D-decel/RT cycles → on latch,
   quiet telemetry (`STREAM 0`) → `DBG I2CLOG` dump → inspect for unthrottled
   stop/reversal 0x60 writes at the latch point. Old harnesses live in
   `tests/old/dev/` (`stand_soak.py`, `enc_watch.py`, `wedge_repro.py`) — the
   knowledge-base bench workflow that references `tests/dev/` is stale.
4. **Mitigation candidates (in test order):** (i) output hysteresis so near-zero
   dither can't produce reversal-exempt writes (latch pct=0 until |pct| exceeds a
   threshold); (ii) don't reset the stuck-counter on target==0 at command
   boundaries, and drop the arming grace when the frozen value equals the last
   pre-command value; (iii) emit `wheel_wedged` in TLM; (iv) re-prime encoders at
   `RT`/`TURN` start or on detection (verify with I2CLOG that the added atomic
   reads don't themselves induce latches).
5. **A/B candidates if frequency rose post-rebase:** 049-003 cmon-pid decel
   dynamics vs the hand-rolled integrator; relay/radio sessions vs USB-direct
   (interrupt load); OTOS lag 10 ms vs 100 ms (bus load — raised 06-17,
   pre-rebase).

## Related

- [encoder-wedge-nrf52-twim-irq-load-errata.md](encoder-wedge-nrf52-twim-irq-load-errata.md)
  — the persistent interrupt-load flavor and the IRQ-guard fix (partially
  superseded by this doc).
- [i2c-sensor-detection-and-bus-wedge.md](i2c-sensor-detection-and-bus-wedge.md) —
  the cold-boot detection wedge.
- Recording analyzed: `host/recordings/recording_20260701_210332.jsonl`
  (episodes at t+15.28 R@748 and t+25.23 L@−501; EVT at t+22.30).
