# Encoders read zero — I2C bus hang from a misbehaving peripheral

**First hit:** 2026-06-04 (sprint 014 bench)

## Symptom

Encoder **position and velocity both read 0** while the **motors still run normally**.
On the wire (TLM / `rogo enc`): `enc=0,0 vel=0,0` throughout a drive, but `S`/`drive`
commands are accepted and the wheels physically spin (often *fast*, because the
velocity loop sees 0 and saturates PWM open-loop). A hand-spin of a wheel while idle
also fails to change the count.

## Root cause

A **misbehaving (or unneeded) I2C peripheral on the shared bus wedges the bus**, so the
Nezha V2 motor controller's encoder register (`0x46`) read returns all zeros — even
though **writes still work** (`0x60` setSpeed reaches the chip, hence motors run). The
motor chip and encoder are fine; the **bus is hung**. On this robot the likely culprit
was an extra **joystick module** on the bus (not needed — remove it).

This is NOT a firmware problem when it reproduces under a stock vendor program (see below).

## How to diagnose (fast, definitive)

Flash a **stock MakeCode program** using the PlanetX/Nezha extension that drives the
motors and reads `readAngle` / `readSpeed`. This is the known-good oracle:
- MakeCode **also reads 0** → it's the **bus/hardware**, not our firmware.
- Put the same program on a **different device + motor**: if it works there, the original
  unit's bus is wedged.

(Our C read path in `Motor::readEncoderAtomic` is byte-for-byte the vendor `readAngle()`:
4 ms delay → write `[FF F9 motor 00 46 00 F5 00]` → 4 ms delay → read 4 bytes. If that
returns 0 but MakeCode on the same healthy bus does not, then it IS a firmware
difference — investigate I2C init/clocking, not the read bytes.)

## Recovery (the important habit)

**A micro:bit-only reset is not enough.** Reflashing or USB-power-cycling the micro:bit
does NOT clear it, because the motor board's **battery backup keeps the I2C peripherals
powered** across a micro:bit reset — the wedged device never cold-boots.

To recover: **fully power down everything** — disconnect the **battery AND USB AND all
I2C peripherals**, wait, then reconnect. Removing the unneeded peripheral (joystick)
prevents recurrence.

**Rule of thumb:** if encoders suddenly read 0 while motors still run, suspect a hung
I2C bus *before* the firmware. Do the full peripheral power-down first, and confirm with
a stock MakeCode readAngle program.

## Aggravating factor to avoid

Hammering the bus with rapid back-to-back diagnostic reads (e.g. scanning many motor IDs,
or tight version+encoder read loops) can itself **lock up the bus** and make things worse.
Keep on-device diagnostics gentle and spaced.
