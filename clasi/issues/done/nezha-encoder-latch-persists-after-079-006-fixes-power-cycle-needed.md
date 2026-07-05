---
status: done
sprint: 079
tickets:
- 079-006
---

# Nezha 0x46 readback still frozen after 079-006's fixes — RESOLVED: stale incremental build, not a hardware latch

## RESOLVED (2026-07-05, session 3) — it was a stale incremental build, not a persistent latch

**This issue can be closed** (team-lead disposition). A separate debug
pass did a genuine `build.py --clean` + flash and confirmed encoders
track real motion: forward drive on ports 1/2/3 (`pos` climbing, `vel`
tracking), closed-loop `VEL 150` converging at the default
`vel_filt_alpha` (0.3), no split-phase TWIM hang, 8/8 pings after `DUTY`.

**The two fixes below (committed `c729c4db`) are correct and
sufficient.** Sessions 1-2's "still frozen, even after a physical
power-cycle" observations were caused by repeatedly flashing a **stale
hex that still contained the pre-fix sentinel bug** — every cold-start
test against that stale build re-triggered the reversal-write-train
latch (item 2 below), regardless of what the source tree actually said.
The persistent-Nezha-latch hypothesis session 2 appeared to falsify was
never actually tested against a clean build in the first place.

**Verification lesson for future sessions**: `VER`'s `fw=` field reports
`source/types/protocol.h`'s hand-maintained `FIRMWARE_VERSION` constant —
it is NOT the `pyproject.toml`/`dotconfig` version bumped by `dotconfig
version bump`, which is never compiled into the firmware at all and is a
red herring for confirming what's actually running. Always run
`uv run python3 build.py --clean` immediately before a HITL verification
flash (never trust an incremental build banner), and treat "should be
fixed but still shows the old behavior" as a stale-build suspect before
reaching for a new hardware hypothesis. See ticket 079-006's own results
section (all three sessions) for the full measurement record (cadence,
both-directions, `vel_filt_alpha`, lazy-timer A/B — all now passed/
recorded on the clean build).

## Session 2 update (superseded by the resolution above) — power-cycle performed, hypothesis appeared falsified

The stakeholder physically power-cycled the robot (full USB unplug, not
just a reflash) specifically to test this issue's hypothesis. Re-verified
with a clean rebuild + fresh `mbdeploy deploy robot` (removing any doubt
about what was on the chip), then drove `DEV M <n> DUTY` on three ports
(1, 2, 4) across separate fresh-boot sessions, with `travel_calib`
amplified 1000x (`DEV M <n> CFG travel_calib=500`) so even a single raw
encoder count would show as a large `pos` swing, and at duty levels from
20% up to 80% (to rule out mechanical stiction as an innocent
explanation).

**Result: `pos`/`vel` are still pinned at exactly 0.0 on every port, at
every duty level, immediately after the power-cycle and a clean reflash.**
`conn=1`/`err=0` throughout; `wsus=1` fires within ~1 s each time, same as
before. The TWIM-stall fix (root cause 1 below) is confirmed still
holding — no more multi-second blackouts, just ordinary single-poll
transport misses — so the firmware itself is not stalled this time.

**This falsifies the persistent-latch hypothesis** below: a genuine
power-cycle should clear a persistent Nezha-side latch per
`docs/knowledge/2026-07-04-encoder-wedge.md`'s own recovery guidance, and
it did not restore any encoder motion. **The root cause of the frozen
`pos`/`vel` reading remains open and unexplained.** Neither of ticket
079-006's two confirmed, real, hardware-verified fixes (the TWIM stall,
the sentinel-write bug) turns out to be *the* explanation for the frozen
encoder value — both are kept regardless (they fix real, independently-
verified defects), but this issue's core question is still unanswered.
This needs a rethink, not another power-cycle. See "Updated next steps"
below (replacing the original "What's needed to close this out").

## Original summary (session 1, superseded by the update above)

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

## Next steps (session 2 — superseded; kept for the historical record)

*Written when the power-cycle appeared to falsify the persistent-latch
hypothesis, before session 3 identified the actual cause (a stale build).
Not acted on further since — kept here only so a future reader can see the
reasoning trail, not as an active task list.*

1. Physically/visually confirm the wheel actually rotates during a `DEV M
   <n> DUTY 80` command.
2. A working gdb/pyOCD inspection of `collectEncoder()`'s `resp[]` bytes
   live (breakpoints were unreliable when attaching to an already-running
   target via `pyocd gdbserver -M attach` in session 1).
3. Check whether `hardReset()`'s own atomic path
   (`readEncoderAtomicRaw()`) ever computes a nonzero snapshot at all.
4. Compare against a different Nezha board/unit.

None of these were needed — a plain `build.py --clean` before the next
flash resolved it (session 3).

## Evidence / how to reproduce

See ticket 079-006's own results section for the full session log (all
three sessions — reproduction commands, gdb backtraces, timing
measurements, and session 3's cadence/A-B/alpha measurements on the
working, clean build). The bug (both fixes reverted) reproduces reliably;
on a genuinely clean build with both fixes in place, it does not:

```
uv run python3 build.py --clean   # ALWAYS do this before a verification flash
mbdeploy deploy robot
# then, over serial:
DEV WD 5000
DEV M <n> RESET
DEV M <n> VEL 150
DEV M <n> STATE   # repeat — pos/vel should climb and converge; wedged/wsus should stay 0
```
