# Encoders read zero — I2C bus wedged (color↔OTOS chip conflict)

**Diagnosed:** 2026-06-04 (sprint 014 bench)

## Symptom

Encoder **position and velocity both read 0** while the **motors still run normally**
(often *fast*, because the velocity loop sees 0 and saturates PWM open-loop). On the wire
(`rogo drive ... stream`, or TLM): `ENC 0 0  VEL 0 0` throughout a drive, but `S`/`drive`
commands are accepted and wheels spin. Hand-spinning a wheel while idle also fails to move
the count. Motor `setSpeed` (I2C *writes* to 0x10) work; only the encoder *read* (0x46) is dead.

## Root cause (bisected)

A **specific pair of I2C devices conflicts on the shared bus**: the **PlanetX color sensor
(addr 0x43)** and the **SparkFun OTOS (addr 0x17)**. With *both* connected, encoder reads
return zeros. Bisection matrix:

| On the bus | Encoders |
|---|---|
| color only | ✅ |
| OTOS only | ✅ |
| color + line (0x1A) | ✅ |
| line + OTOS | ✅ |
| **color + OTOS** | ❌ `0 0` |
| all three | ❌ `0 0` |

Each device works alone; only the **0x43 + 0x17 combination** wedges the bus. It is **not**
bus loading (line+OTOS, a pair, is fine) and **not** our firmware's reads — proven from BOTH
sides with compile-time toggles, all with both modules physically plugged:
- Rewriting the color read to match upstream (single-byte + init/settle/retry) — **still 0**.
- `DISABLE_COLOR_SENSOR` (zero I2C to 0x43, OTOS still read) — **still 0**.
- `DISABLE_OTOS_SENSOR` (zero I2C to 0x17, color still read) — **still 0**.

Disabling *either* chip's firmware access does not help; only their **physical coexistence**
on the bus matters. Definitively an **electrical / chip-level I2C conflict** between those two
specific boards (bus contention / pull-up / clock-stretch) — firmware cannot resolve it.

**Per-unit:** a *different* robot build ran all three sensors fine under stock MakeCode (which,
note, never reads the OTOS). So this is specific to **this unit's bus wiring** (pull-ups / lead
length / module quality), not universal — another build may carry all three.

## Recovery (the important operational habit)

The wedge **persists across a reflash** — flashing resets only the micro:bit (nRF52); the
motor board + sensors stay powered (battery backup), so the wedged device never cold-boots.

To recover encoder reads: **fully power down everything** — disconnect **battery AND USB AND
the conflicting peripheral(s)**, wait, reconnect. A micro:bit-only reset/reflash is NOT enough.

**Rule of thumb:** if encoders suddenly read 0 while motors still run, suspect a wedged I2C
bus from a peripheral *before* the firmware. Full peripheral power-down to clear it.

## Diagnosis technique (firmware-independent oracle)

Flash a **stock MakeCode program** (PlanetX/Nezha extension) that drives motors and reads
`readAngle`/`readSpeed`. If it *also* reads 0 → it's the bus/hardware, not our firmware.
Then bisect peripherals one at a time (each change needs a **full power cycle**) to find the
conflicting set. Our C encoder read (`Motor::readEncoderAtomic`) is byte-identical to the
vendor `readAngle()` (4 ms → write `[FF F9 motor 00 46 00 F5 00]` → 4 ms → read 4 bytes), so
when it reads 0 on a healthy bus, suspect the bus, not the bytes.

## Mitigations (open — hardware domain)

color↔OTOS cannot share the bus as wired. Options: put one on a separate I2C bus/mux, tune
pull-ups / shorten leads, or run only one of the two. Firmware can't resolve a chip-level
electrical conflict.

## Aggravating factor to avoid

Hammering the bus with rapid back-to-back diagnostic reads (scanning many motor IDs, tight
version+encoder loops) can itself lock the bus. Keep on-device diagnostics gentle and spaced.
