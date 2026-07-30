---
status: in-progress
sprint: '126'
tickets:
- 126-001
- 126-002
- 126-003
- 126-004
- 126-005
- 126-006
---

# Bring the OTOS up: read it, report it in telemetry, calibrate it against the camera

Turn the OTOS (optical tracking odometry sensor) on as a **readable, trusted
instrument**. Read the chip, get its pose into telemetry, and prove — against
camera truth — that what it reports is correct for **heading, turns, and
distance**.

**Explicitly out of scope** (stakeholder, 2026-07-29): fusing OTOS into the
state estimate, or using it to control heading. `estimator.weight_heading_otos`
and `weight_omega_otos` stay **0.0**. This issue makes the OTOS *legible*, not
*authoritative*. A later issue can decide whether to trust it.

## What already exists (verified on hardware 2026-07-29, do not redo)

Most of this is already built. The gap is smaller than it looks:

- `Devices::RealOtos` (`src/firm/devices/otos.cpp`) is a real driver, and
  `main.cpp` binds the loop's `Devices::Otos&` to it. `FAKE_OTOS` defaults
  **OFF** in both `CMakeLists.txt` and `build.py`, so the shipped firmware
  already talks to the real chip.
- `RealOtos::begin()` detects by product ID and sets `initialized_`/`connected_`;
  `present()` surfaces as telemetry flags bit 0 and as `otos=` in the cleartext
  `STATUS` line.
- The telemetry frame already carries OTOS: `TLMFrame` exposes `otos`,
  `otos_reading`, `otos_present`, `otos_connected`, `otos_health`, and
  `tlm_log.py` already writes `otos_x/otos_y/otos_heading/otos_v_x/otos_v_y/
  otos_omega/otos_age` columns.
- `data/robots/tovez.json` already holds the full configuration:
  `geometry.odometry_offset_mm` = **{x: -47.7, y: 3.5, yaw_rad: 0.0}** (the
  lever arm from robot centre to the chip), `geometry.odometry_chip_upside_down`
  = false, `calibration.otos_linear_scale` = **1.067**,
  `calibration.otos_angular_scale` = **0.987**.
- `gen_boot_config.py`'s `otos_boot_config_values()` already bakes those five
  numbers into `OtosBootConfig` at boot.

**So this is a bring-up and calibration job, not a driver-writing job.**

## What is NOT known, and must be established

1. **Is the reported pose in the robot-centre frame or the chip frame, and is
   the lever-arm sign right?** At rest after a power cycle the frame reported
   `otos = (48, -4, 0)` — suspiciously close to the *negated* configured offset
   (-47.7, 3.5). That is consistent with the offset transform being applied to a
   zeroed sensor, but a single at-rest sample cannot distinguish that from
   coincidence. Establish it by moving a known distance and watching the delta.
2. **Units.** The tuple's scale is unconfirmed (mm vs cm). Settle it from a
   measured move, not from reading the code.
3. **Does the pose track motion at all**, and does it survive a full tour?
4. **Are `otos_linear_scale` / `otos_angular_scale` still correct?** They are
   committed values of unknown provenance. Re-measure against the camera.

## Blocking hazard found 2026-07-29 — read before touching hardware

The external I2C bus wedged. Symptom: the robot emitted its `DEVICE:` banner,
then **no `READY`**, and answered nothing on any port. `STATUS` reported
`otos=0`. A pyocd backtrace showed the firmware spinning in:

```
RobotLoop::boot -> Preamble::probeSlot -> NezhaMotor::hardReset
  -> readEncoderAtomicRaw -> MicroBitI2CBus::write -> NRF52I2C::waitForStop
```

i.e. hung during boot probing the LEFT MOTOR, with IRQs masked (which is why
serial went dead too). **A full power cycle cleared it**, after which
`STATUS` reported `otos=1` and telemetry flowed. A `pyocd reset` alone did NOT
clear it.

Lesson: `otos=0` may mean "the whole external bus is wedged", not "the chip is
missing" — the OTOS shares that bus with the Nezha motor controller. Check
`connL`/`connR` and whether `READY` was emitted before blaming the sensor. See
`.clasi/knowledge/silent-robot-dead-external-i2c-bus.md`.

## Tether constraint

The robot is on the playfield **with a serial tether**, so the firmware can be
flashed, but rotation must not wind the cable:

- Turn from facing east to facing west **through north**, never through south.
- **Alternate direction on successive turns** — east→west through north, then
  west→east through north — so net wrap stays at zero.
- Prefer straight-line moves for distance work; they cannot wrap the cable.

## Acceptance

Demonstrated on the playfield, against camera truth (tag 100 is registered as
the robot's centre of rotation), with the results printed and charted:

1. **Presence.** `STATUS` reports `otos=1` and telemetry flags bit 0 is set, on
   a robot that has reached `READY` with `connL=1`/`connR=1`.
2. **Liveness.** The OTOS pose changes when the robot moves and holds steady
   when it does not. Units and frame are stated explicitly, with the measurement
   that establishes them.
3. **Lever arm / frame.** It is shown whether the reported pose is the robot
   centre or the chip, and the configured `odometry_offset_mm` is confirmed
   correct in magnitude AND sign — e.g. a pure in-place rotation moves the chip
   on a circle of radius |offset| while the robot centre stays put; whichever
   frame the telemetry reports will be obvious from that.
4. **Distance calibration.** Over a set of straight runs of differing lengths in
   both directions, OTOS-reported distance is compared against camera distance;
   the linear scale is reported, and `otos_linear_scale` corrected if the
   measurement disagrees with the committed 1.067.
5. **Heading / turn calibration.** Over a set of turns of differing magnitudes
   in both directions (respecting the tether rule), OTOS-reported heading change
   is compared against camera heading change; the angular scale is reported and
   `otos_angular_scale` corrected if it disagrees with the committed 0.987.
6. **Residuals stated, not hidden.** Mean and spread of the OTOS-vs-camera error
   are given for distance and for heading, so a later fusion decision has real
   numbers to weigh.
7. Sim suite still passes; nothing in the estimator's fusion weights changed.

## Notes

- Rotation calibration and OTOS scale are **boot-baked, not runtime-settable**:
  `App::Configurator` accepts only OTOS/ESTIMATOR/MOTOR patches, and
  `otos_linear_scale`/`otos_angular_scale` are not in the host's config key map.
  Changing them requires editing `data/robots/tovez.json` and reflashing — which
  is why this work wants the tether.
- Check the playfield lights (Shelly relay `192.168.1.122`) before every camera
  run; they switch off on their own and a dark field looks exactly like a broken
  camera. See `.claude/rules/playfield-testing.md`.
- Any halt path must use `estop()`, never `stop()` — see the same rule file.
