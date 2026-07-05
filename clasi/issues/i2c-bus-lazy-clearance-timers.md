---
status: pending
---

# I2CBus lazy per-device pre/post clearance timers — reclaim the ~8 ms/tick encoder settle spins

## Idea (stakeholder, 2026-07-04)

Move the encoder-path busy-waits out of the callers and into the bus
object as *lazy* deadlines. `I2CBus::write`/`read` gain optional
`preClear`/`postClear` parameters — `// [us]` — and the bus keeps
per-device timestamps: after each transaction it records
`lastEnd = now` and `readyAt = now + postClear`; on entry it spins only
until `max(slot.readyAt, slot.lastEnd + preClear)`. If the deadline has
already passed (because the control loop did other work in between),
the call fires immediately and the wait costs nothing. Defaults of 0
leave every existing call site untouched.

## What it replaces

Unconditional spins in `source/hal/nezha/nezha_motor.cpp`:

- `readEncoderSettle()` — 0x46 request write → 4 ms spin → read, every
  tick, per motor. Two motors on a 10 ms tick ≈ **8 ms of every control
  tick burned spinning**.
- `readEncoderAtomicRaw()` — 4 ms pre-idle spin + 4 ms settle spin
  (~8 ms, hard-reset path only).
- `writePositionMove()` — 4 ms post-write spin (BUG-CRITICAL vendor
  lore: "no task/fiber may interleave").

Under the new scheme the 0x46 encoder request write carries
`postClear = 4000` instead; the paired read waits out only the
remainder. The atomic path's pre-idle becomes `preClear = 4000`, which
is typically already elapsed.

## The catch — the timer alone saves nothing

Today the read follows the request write by microseconds, so the
deadline never expires early; restructured call sites are where the
time comes from. The split-phase `requestEncoder()`/`collectEncoder()`
API already exists in `NezhaMotor` (ported, deliberately unwired) —
wiring it up lets other-device traffic (OTOS 0x17, line 0x1A, color)
and control math fill the settle windows, with the bus timer as the
safety net: any path that collects too early waits out the remainder
instead of reading garbage.

Structural constraint: **both motors are the same device (0x10)** — one
Nezha brick, one readback register — so L/R 0x46 requests cannot
pipeline. The sequence stays reqL → collectL → reqR → collectR; only
other devices and math can hide in the windows.

## Design constraints

1. **Per-device timers, not bus-global.** Extend the existing 7-bit
   address `DeviceSlot` table in `source/com/i2c_bus.h` with
   `lastEnd`/`readyAt` `// [us]` fields (`system_timer_current_time_us`,
   uint64, no wrap concern). A bus-global timer would block the very
   OTOS read that is supposed to fill the settle window.
2. **Clearance spin stays OUTSIDE the IRQ-guard masked window** — never
   mask interrupts for 4 ms + transaction. Keep the wait a spin (not
   `fiber_sleep`) to preserve the vendor no-interleave property.
3. **HOST_BUILD stub** gets the same API surface.
4. Attach the settle to the *request write's* `postClear` (the writer
   owns the constraint) so any subsequent transaction to 0x10 —
   including a stray 0x60 velocity write — is held off too.

## Acceptance gate — stand A/B required

Traffic *inside* the settle window is untested territory. Wedgelab
killed "mixed-bus sensor traffic" as a latch cause, but nobody has run
an OTOS read between a 0x46 request and its collect, and the vendor
timing is cargo-cult ("ported verbatim"). Per
`docs/knowledge/2026-07-04-encoder-wedge.md`, gate on a stand A/B:
latch rate (diagnose from TLM enc-constancy, not EVT) + encoder sanity
under the interleaved tick, bracketed with controls.

## Explicit non-goals

- **NOT the ≥50 ms zero-dwell reversal fix.** That is a control-*state*
  requirement (command zero and hold), belongs in a non-blocking motor
  driver state machine, and remains its own pending ticket. A blocking
  `preClear = 50000` on the sign-flip write would stall the control
  loop ~40 ms mid-tick — do not implement the dwell this way (backstop
  enforcement at most).
- **NOT a wedge fix.** The knowledge doc's conclusion stands: no
  bus-timing change fixes the latch. This issue is tick-budget recovery
  and wait-centralization only.
