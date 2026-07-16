---
status: pending
---

# Sim should honor the OTOS calibration scalars (simulate a calibrated chip)

## Problem

The host simulator (`tests/_infra/sim/`) simulates the OTOS chip via
`TestSim::OtosPlant` + `SimPlant`'s I2C responder, but it models an IDEAL OTOS:
it reports the plant ground truth (plus the sprint-108 drift/noise fault knobs),
with NO linear/angular scale error and NO honoring of the OTOS calibration
scalars. So even once a runtime OTOS-calibration message exists
([[otos-calibration-config-message]]), setting `OL`/`OA` would have no visible
effect in the sim.

Stakeholder point (2026-07-16): "if you are simulating the OTOS / the I2C bus,
you should be simulating calibrations." A faithful sim of a real OTOS chip has a
raw measurement error that the calibration scalars correct — so mis-setting the
scalars should show up as an OTOS pose error, and correctly-set scalars should
read true. That makes the sim a real testbed for OTOS calibration.

## Direction

- Give the simulated OTOS chip a modeled RAW scale error (a per-axis
  linear/angular factor, default configurable) that the firmware's OTOS scalar
  registers (`setLinearScalar`/`setAngularScalar`, applied at the CHIP level)
  correct — i.e. `SimPlant`'s OTOS burst-read response = truth * rawError, and
  the firmware's scalar register (written via the new config message) scales it
  back. Net = truth when calibrated, error when mis-calibrated.
- Expose the raw-error factor via the ctypes ABI (a sim fault-condition knob,
  like the existing OTOS drift knob) so a test / the Sim Errors panel can dial
  in a chip that needs calibrating.
- Requires [[otos-calibration-config-message]] to exist first (the wire path to
  the scalar registers).

## Notes
- The sim already has the register-level plumbing (`SimPlant` answers the OTOS
  burst read; `OtosPlant` computes pose). This is about adding a scale-error
  stage + honoring the scalar registers the firmware writes.
- Filed out-of-process 2026-07-16.
