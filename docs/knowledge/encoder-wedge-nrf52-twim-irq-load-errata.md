# The driving encoder wedge: nRF52 TWIM I2C errata under background interrupt load

**Date:** 2026-06-07
**Sprint:** post-015 (out-of-process bench debugging)
**Status:** RESOLVED — root cause confirmed, fix implemented and A/B-verified

> **Note (2026-07-01): partially superseded.** A second, distinct wedge flavor is
> now on record — a **transient boundary latch** that begins at D-command
> deceleration/stop, is invisible to the enc_wedged detector, and **self-heals at
> the next encoder reset** (atomic reads re-prime the register; no power-cycle
> needed). Also: the config history (`DefaultConfig.cpp`, 2026-06-17) documents
> the wedge persisting at 4-12% in field sweeps *after* the IRQ-guard fix below,
> so "eliminated" overstates it. See
> [2026-07-01-encoder-wedge-boundary-latch-flavor.md](2026-07-01-encoder-wedge-boundary-latch-flavor.md).
> The bench tools referenced below now live in `tests/old/dev/`, not `tests/dev/`.

This is the root cause of the long-standing "encoder wedge": while driving, the
Nezha motor controller (I2C `0x10`) freezes its encoder readback — it returns a
constant value forever while the wheels keep spinning — and only a power-cycle of
the Nezha clears it. This doc is the authoritative explanation; it supersedes the
earlier *correlation*-based theories (sensor bus traffic, read method, write rate,
loop structure, the `I2CBus` wrapper). Those were all symptoms of the real cause.

---

## The root cause

**It is the nRF52 TWIM (I2C master) silicon errata, triggered by background
interrupt load.** This is documented in CODAL's own driver —
`libraries/codal-nrf52/source/NRF52I2C.cpp`, in `NRF52I2C::waitForStop`:

```c
// Test for condition where the SHORTS configuration appears to not trigger TASKS as expected.
// Could be an undocumented silicon errata.
// Appears to only occur under higher levels of background interrupt load.
```

The nRF52 TWIM peripheral uses hardware SHORTS (event→task chaining) to sequence a
transfer. Under heavy interrupt load, those SHORTS can fail to fire, so the
transfer doesn't complete/stop as expected. Codal has partial recovery
(SUSPEND/STOP/`redirect`), but it is not enough here: a corrupted transfer to the
Nezha leaves the controller's encoder readback latched frozen.

The production firmware generates exactly the "higher levels of background
interrupt load" the errata needs:
- **Async serial telemetry TX is interrupt-driven** — streaming TLM frames at
  20-50 ms means a steady stream of UART interrupts.
- The **radio** (relay link) adds more.
- These fire *around* the I2C transactions to the Nezha on the shared bus.

The raw `WedgeTest` harness (`DBG WEDGE`) never reproduced it because it **takes
over the loop**: no telemetry stream, minimal serial, no radio relay → low
interrupt load → the errata effectively never fires (clean for 10-20 min).

---

## The fix

**Mask interrupts for the *duration* of each I2C transaction** so there is no
background interrupt load while the TWIM transfer runs. In
[source/hal/I2CBus.cpp](../../source/hal/I2CBus.cpp), `I2CBus::write` / `read` hold
`target_disable_irq()` across the underlying `_bus.write/read` call (not just the
`_inUse` flag), re-enabling after:

```cpp
target_disable_irq();          // mask for the flag AND the transaction
... _inUse flag check/set ...
int status = _bus.write(address, data, len, repeated);   // runs with IRQs masked
... clear flag ...
target_enable_irq();
```

Gated by `_irqGuard` (default **ON**). Live toggle for A/B testing without
reflashing: **`DBG IRQGUARD 1|0`** (no arg = report state).

`target_disable_irq/enable_irq` are nest-counted in codal
(`codal_target_hal_base.cpp`), so this composes safely.

### Trade-off to keep in mind
Each transaction masks IRQs for ~hundreds of µs (≈1-1.5 ms total per control tick,
in short bursts). The nRF UART/radio use DMA + FIFOs, so brief masking only delays
interrupt servicing rather than dropping data — an 8-minute soak streamed telemetry
cleanly throughout. Still: if you ever add much heavier or more latency-sensitive
interrupt work, re-verify comms reliability.

---

## The proof (A/B, same robot, same session, full load)

Full load = all sensors enabled + heavy telemetry streaming (the condition that
wedged at ~maneuver #20 / ~1 min on **every** prior run):

| `DBG IRQGUARD` | Result |
| --- | --- |
| **1 (on)**  | 188 maneuvers, **8 min, NO ANOMALY** |
| **0 (off)** | **wedged at maneuver #12, 0.5 min** |

Toggling the single flag flips the bug on and off. That is the definitive test.

---

## Why it took so long (red herrings, and why each was wrong)

Everything that *correlated* did so because it changed the **interrupt load**, not
because it was the mechanism:

- **"Sensors cause it."** More sensor reads = more bus traffic *and* the firmware
  was busier, but the real link is interrupt load. Disabling individual sensors
  (`SET lag.color=0`, `lag.line=0`) only moved the wedge within the stochastic
  noise (#55-#112).
- **Read method** (`readEncoderSettle` vs atomic, `SET encAtomic`): no effect —
  atomic wedged too.
- **Write rate**: already throttled to 40 ms in `Motor::setSpeed`
  (`kMinWriteIntervalUs`), same as the clean harness. Forcing a write right before
  each read (`Motor::reassertSpeed`) made it *worse* (#1) — a non-issue once the
  real cause was known.
- **Quiet-before-read** (`SET encQuiet`): "helped" only by slowing the loop (fewer
  transactions/min); the frozen read still had 50 ms of quiet before it.
- **The `I2CBus` wrapper**: a thin pass-through to the *same* `NRF52I2C` call, so
  it wasn't the cause by itself — it only correlated because production uses it
  *and* runs hot. (It did turn out to be the right *place* for the fix.)

The breakthrough was: `enc_watch` (production control path, **all sensors off**)
still wedged, while the raw `WedgeTest` (which takes over the loop) stayed clean.
Same motors, same chip → the difference is the production runtime's interrupt load,
which led to the codal errata comment.

---

## Recovery & diagnostics (bench workflow)

- **Clearing a wedge:** power-cycle **only the Nezha motor controller** (its own
  switch). The micro:bit/USB stay up — no re-enumerate, no reflash, no reconnect.
  The latch lives in the Nezha's power domain. (A micro:bit reset / reflash does
  NOT clear it.)
- **Detect a wedge:** `uv run python tests/dev/enc_watch.py --secs 4` — drives
  briefly, prints `ENCODERS COUNTED ✓` / `FROZEN ✗` and `stuck=L:n,R:n`.
- **Reproduce under load:** `uv run python tests/dev/stand_soak.py --minutes N`
  (random maneuvers + telemetry + OV writes). `--dbg "IRQGUARD 0"` / `--set k=v`
  apply commands after connect (the soak resets the µbit on connect, so set knobs
  there, not before).
- **See the bus at the wedge:** the `I2CBus` ring buffer logs every transaction;
  `MotorController` freezes it on the wedge; `DBG I2CLOG` dumps it (one line, after
  telemetry is quiet so it doesn't garble the async TX).
- **mbdeploy "device not connected" while `probe` lists it:** stale device
  registry — delete the probe/registry file and re-`probe`.

---

## Related

- [i2c-sensor-detection-and-bus-wedge.md](i2c-sensor-detection-and-bus-wedge.md) —
  the *cold-boot detection* wedge (a separate issue: `begin()` placement, battery-
  backed bus). That doc's "wedged bus persists across reflash" note is the same
  power-domain fact seen from the boot side.
- [encoders-read-zero-i2c-bus-hang.md](encoders-read-zero-i2c-bus-hang.md)
- [watchdog-uint32-underflow-velocity-notches.md](watchdog-uint32-underflow-velocity-notches.md)
