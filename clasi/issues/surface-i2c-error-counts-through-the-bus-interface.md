---
status: pending
---

# Surface I2C error/re-entrancy counts through Devices::I2CBus so kFlagFaultI2CNak can go live

**Source:** code review 2026-07-30, `01-firm.md` MINOR §6.
**Priority:** P2 — small, pure diagnosability win.
**Goal served:** this campaign's premise is making bugs findable. A run of
NAK'd duty writes or a bus re-entrancy violation is currently invisible
outside a debugger, even though `MicroBitI2CBus` already counts both —
the counts are just unreachable through the abstract interface `RobotLoop`
holds. Wedge/bus investigations start blind because of this.

## What is wrong

- `MicroBitI2CBus::record()` tracks per-device `errCount`/`lastErr`;
  `reentryViolations_` tracks re-entrancy — but the abstract
  `Devices::I2CBus` (`i2c_bus.h`) exposes only `clearanceSafetyNetCount()`.
- `telemetry.h:126-128` admits bit 8 (`kFlagFaultI2CNak`) is "declared, not
  yet wired live (no per-transaction NAK aggregate exists yet)" — which is
  wrong: the aggregate exists, it is just unreachable.

## What to do

Widen the interface with a single rollup (per-device breakdown stays on the
concrete class, per its own design note):

```cpp
// i2c_bus.h -- Devices::I2CBus
// Running totals across ALL devices on this bus. A fault bit needs only
// the rollup; the per-device breakdown stays MicroBitI2CBus-only.
virtual uint32_t errCount() const = 0;            // NAK/timeout transactions
virtual uint32_t reentryViolations() const = 0;
```

Wire `RobotLoop`/`App::Telemetry` to set `kFlagFaultI2CNak` when `errCount()`
has advanced since the previous frame (an Event-category bit per
telemetry.h's own bit-layout discipline — compare against the documented
freshness/event rules there), and correct the telemetry.h comment.
Update `SimPlant`'s bus implementation with the two trivial overrides.

## Acceptance

- Firmware test: injecting a NAK through the sim bus advances `errCount()`
  and raises bit 8 on the next frame exactly once.
- Bench: unplug the OTOS mid-session (known NAK generator) — bit 8 appears
  in telemetry; `tlm_log.py` capture shows it.
- telemetry.h's bit-8 comment matches reality.
