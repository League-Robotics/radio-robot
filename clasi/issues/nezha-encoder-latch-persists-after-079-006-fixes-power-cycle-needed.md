---
status: pending
sprint: 079
tickets:
- 079-006
---

# Nezha 0x46 readback still frozen after 079-006's fixes — power-cycle done, persistent-latch hypothesis FALSIFIED, root cause still open

## UPDATE (2026-07-05, session 2) — power-cycle performed, hypothesis falsified

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

## Updated next steps (session 2 — the power-cycle path is exhausted)

Since a genuine power-cycle did not help, the remaining candidate
explanations are all either a deeper software defect or a hardware fault,
neither of which more blind clearance-timing/firmware guessing is likely
to resolve efficiently. Ranked by how cheap they are to try:

1. **Physically/visually confirm the wheel actually rotates** during a
   `DEV M <n> DUTY 80` command. This agent has no way to observe the stand
   directly and has been trusting ticket 005's original "duty writes work
   (motors spin)" claim at face value. If the wheel does NOT actually turn
   (motor stalled, gearbox/coupling issue, wrong motor wired to that
   channel), the frozen encoder is *correct* behavior and the real bug is
   elsewhere (or there is no bug — a bench wiring issue). This is the
   single highest-value, cheapest thing to check next and should happen
   before any more firmware changes.
2. **A working gdb/pyOCD inspection of `collectEncoder()`'s `resp[]` bytes
   live.** Session 1 tried this and could not get breakpoints to reliably
   fire when attaching to an already-running target via
   `pyocd gdbserver -M attach`; the default `-M halt` connect mode was more
   reliable for one-shot backtraces (used successfully to catch the TWIM
   stall) but halts the target on connect, disrupting serial for a few
   seconds. Worth trying: `-M halt` + immediately `continue` before setting
   breakpoints (giving the target a chance to resume normal scheduling
   first), or a completely different technique (e.g. `DBG I2CLOG`-style
   instrumentation added to the firmware temporarily, exposing raw
   transaction bytes over the wire instead of relying on a live debugger
   attach at all).
3. **Check whether `hardReset()`'s own atomic path (`readEncoderAtomicRaw()`)
   ever computes a nonzero snapshot at all**, independent of the split-phase
   design entirely — if `encOffset_` is *always* 0 across repeated hard
   resets (not just "unchanged from before," but genuinely always
   recomputed as 0), that points at the shared atomic-read primitive
   itself (used at boot and by every hard reset, unmodified since sprint
   077) rather than anything specific to 079's request/collect split. Not
   directly observable over the wire today (no verb exposes the raw
   register or `encOffset_`); would need either a temporary debug verb or
   the gdb approach above.
4. Compare directly against a **different Nezha board/unit** if one is
   available, to separate "this specific unit's hardware" from "the
   current firmware."

## Evidence / how to reproduce

See ticket 079-006's own results section for the full session log (both
sessions — reproduction commands, gdb backtraces, timing measurements).
This reproduces reliably, immediately, on a freshly power-cycled and
freshly reflashed unit — no special setup or waiting needed:

```
mbdeploy deploy robot
# then, over serial:
DEV WD 5000
DEV M <n> RESET
DEV M <n> DUTY 30
DEV M <n> STATE   # repeat — pos/vel should climb; wedged/wsus should stay 0
```
