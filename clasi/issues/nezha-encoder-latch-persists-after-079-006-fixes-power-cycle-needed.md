---
status: pending
sprint: 079
tickets:
- 079-006
---

# Nezha 0x46 readback still frozen after 079-006's fixes — likely a persistent latch requiring a physical power-cycle to verify

## Summary

Ticket 079-006's stand campaign root-caused and fixed two real defects in
the sprint 079 split-phase encoder path (see the ticket's "Root-cause
findings" section and `source/hal/nezha/nezha_motor.cpp`):

1. **A severe TWIM hardware stall**: `requestEncoder()`'s 0x46 write and
   `writeMotorRun()`'s 0x60 write had no clearance around them, so a single
   in-use port's own back-to-back request/collect/write cadence could
   re-issue the next 0x46 request with ~0 µs real gap since the previous
   transaction. Confirmed via pyOCD/gdb backtraces caught mid-stall: the
   firmware parked for several seconds at a time inside vendor CODAL's
   `NRF52I2C::waitForStop()` (`libraries/codal-nrf52/source/NRF52I2C.cpp`),
   spinning toward its own ~10 s internal timeout waiting for a TWIM
   STOPPED event that never arrived — freezing the entire main loop,
   serial included. **Fixed**: `preClear=4000`/`postClear=4000` added to
   both writes, restoring the real ≥4 ms gap around every 0x10 transaction
   the old fused/blocking `readEncoderSettle()` always had. Verified via
   long-duration (60-90 s) hardware PING-availability monitoring:
   multi-second sustained blackouts collapsed to isolated single-poll
   misses consistent with the already-documented ordinary USB-CDC
   transport drop rate.
2. **A wrong-direction first-write bug**: `writeRawDuty()`'s slew clamp fed
   `lastWrittenPct_`'s `-128` "no write yet" sentinel into
   `MotorSlew::clampStep()` unconditionally (ported-as-intentional from
   source_old/077/078). `clampStep(-128, 30, 25)` returns `-103` — a
   wrong-sign, out-of-range (register's speed byte is documented 0-100)
   write, sent as literally the first command to a fresh port. This is
   exactly `docs/knowledge/2026-07-04-encoder-wedge.md`'s confirmed
   reversal-write-train latch trigger. **Fixed**: the first-ever write (like
   a stop) is now exempted from the slew clamp and goes straight to the
   requested value.

## What's still open

Despite both fixes (independently verified — see the host-level regression
scenarios added to `tests/sim/unit/nezha_flipflop_harness.cpp`, scenarios 8
and 9, and the isolated hardware timing tests recorded in ticket 079-006),
**the 0x46 readback on this specific bench unit (NEZHA2 "robot",
`9906360200052820a8fdb5e413abb276000000006e052820`) never showed real
motion during this session** — `pos`/`vel` stayed pinned at exactly the
post-reset baseline (0.0) on every port (1-4) tested, across dozens of
attempts, including:

- Immediately after a fresh, clean reflash (both fixes in place, minimal
  prior traffic to that port).
- On a port individually addressed for the very first time all session
  (port 3 in one pass).
- Immediately after a genuine, verified standstill-guarded **hard** reset
  (`hrc` counter observed incrementing — i.e. the atomic median-of-3
  re-prime burst genuinely ran), not just a soft rebaseline.
- With `travel_calib` amplified 1000x (`DEV M <n> CFG travel_calib=500`) to
  make even a single raw-count change highly visible — still exactly 0.0.

`conn=1`/`err=0` throughout (no I2C error ever reported) and `wsus=1`
(motion-qualified wedge-suspect, not just the raw, always-fires-at-rest
`wedged` flag) fires within ~1 s of any DUTY/VEL command — i.e. the
symptom is a real, hardware-level frozen-readback latch (not a software
display artifact; not a false-positive from the raw `wedged` flag, which
trivially fires for a stationary motor too), matching
`docs/knowledge/2026-07-04-encoder-wedge.md`'s documented "latch" flavor,
including its explicit escalation path: "repeated abuse escalates to a
**persistent** latch that no in-band reset clears — only a Nezha
power-domain cycle plus full firmware reboot (`begin()` re-init) clears
it... a micro:bit reset/reflash alone never clears a latch."

**This session's own testing is the most likely cause of the escalation**:
before the two fixes above landed, dozens of cold-start DUTY tests each hit
the wrong-direction sentinel bug (a hardware-confirmed reversal-latch
trigger) and/or the severe TWIM stall, back to back, on every port, which
is exactly the "repeated abuse" the doc warns escalates a transient latch
into a persistent one. Ticket 079-005's own stand-smoke session (which
first surfaced "pos/vel frozen" as a finding) may have already started
this escalation, since it too exercised `DEV M 1 DUTY 30` through the
newly-activated flip-flop for the first time ever.

## What's needed to close this out

1. **A full physical power-cycle** of the robot (USB unplug/replug, or the
   robot's own power switch if separate from the micro:bit's USB power) —
   per the doc's own recovery guidance. A microcontroller reflash alone is
   **not** sufficient; the latch lives in the Nezha board's own
   (battery-backed) state, independent of the micro:bit's firmware.
2. **After the power-cycle**, re-run this ticket's stand checks (cadence,
   in-use cycling, the lazy-timer A/B gate, the shared-0x10 clobber check,
   `vel_filt_alpha` retune) against a genuinely clean encoder — none of
   these could be conclusively completed this session, since they all
   depend on being able to see the encoder actually move.
3. If the fixes above (clearance timing + sentinel exemption) hold up on a
   clean unit, this issue can close. If pos/vel STILL never move on a
   freshly power-cycled unit, that is new, stronger evidence of a deeper
   defect (possibly still in the request/collect split's interaction with
   this specific brick, or a hardware fault on this unit) warranting
   further investigation — ideally starting from the pyOCD/gdb technique
   this session used to catch the TWIM stall directly (breakpoints on
   `Hal::NezhaMotor::collectEncoder`/`::tick` were unreliable when attaching
   to an already-running target via `pyocd gdbserver -M attach`; the default
   `-M halt` connect mode was more reliable for backtraces but disrupts
   serial for a few seconds on attach — worth a cleaner recipe for whoever
   picks this up).

## Evidence / how to reproduce

See ticket 079-006's own results section for the full session log
(reproduction commands, gdb backtraces, timing measurements). Quick
repro once a clean unit is available:

```
mbdeploy deploy robot
# then, over serial:
DEV WD 5000
DEV M <n> RESET
DEV M <n> DUTY 30
DEV M <n> STATE   # repeat — pos/vel should climb; wedged/wsus should stay 0
```
