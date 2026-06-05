# Encoders/sensors read zero — RESOLVED (it was sensor-detection placement)

**Status: SOLVED.** All three sensors (OTOS 0x17, line 0x1A, color 0x43) + encoders read
reliably together. Verified telemetry:
`TLM enc=58,57 pose=... line=51,98,155,218 color=216,392,351,1099`.

## What it actually was

NOT an electrical/"marginal bus" problem (the HAL/I2C code was correct all along), and NOT a
color↔OTOS device conflict. The root cause was **where sensor detection (`begin()`) ran**:

1. **Detecting in the boot constructor** read the line/color chips **before they had powered
   up** (the chips need ~seconds after a cold power-on). Those failed reads marked the sensors
   absent and could leave the bus stuck.
2. **Detecting inside the cooperative loop** froze the loop during each sensor's retry, so the
   encoder task didn't run between attempts and the reads failed.
3. **Color detection must re-assert its wake registers each retry.** The old PlanetX
   `initColor` writes `0x81=0xCA, 0x80=0x17` *inside* every retry iteration (20×, 50 ms apart)
   so a chip that wasn't ready on the first try gets re-woken once it powers up. A wake-once
   version fails to detect a cold chip.

## The fix (current architecture)

- **Devices are constructed in `main()`** on `uBit.i2c`/`uBit.io` (Motor×2, OtosSensor,
  LineSensor, ColorSensor, Servo, PortIO), and `Robot` is built **from those objects + a
  `Communicator`** — `Robot` no longer takes `i2c`/serial/radio/`MicroBit`.
- **`begin()` is called explicitly, straight-line, in `main()` before the loop starts**, after
  a short settle (`uBit.sleep(2500)`) so the chips are powered. Comment out a `begin()` to
  disable that device (its reads then skip via `is_initialized()`).
- Color/line `begin()` retry internally (color re-wakes each retry, exact port of `initColor`).
- Reads are gated on `is_initialized()`; `begin()` is the only thing that sets it.

## The red herring that cost the most time

A wedged bus (a slave holding SDA) **persists across micro:bit reflashes** because the robot's
battery keeps the peripheral side powered. Once any early test wedged the bus, *every*
subsequent reflash-and-test ran on the still-wedged bus and failed identically — which looked
exactly like a deterministic code bug. **Rule: when debugging I2C, do a FULL power-down
(battery + USB) to guarantee a clean bus; a reflash is not enough.** A minimal bare-metal
bring-up `main` (construct sensors, `begin()`, read in a plain loop — no scheduler/Robot) on a
freshly power-cycled bus is what finally proved the HAL was correct.
