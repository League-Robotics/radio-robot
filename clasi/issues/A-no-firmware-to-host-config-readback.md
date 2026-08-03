---
status: pending
priority: high
---

# There is no way to ask the robot what constants it is running

## Description

The sprint-130 post-mortem's **recommendation #1**, and it ranks it "the single
highest-leverage missing capability." The post-130 review reaches the same
conclusion independently (F6).

`config.proto` has **no `ConfigSnapshot` arm**. Nothing on the wire reports the
values the firmware is actually using.

## What it already cost

Twice in one sprint, a routine measurement produced a confident wrong answer:

- A duty sweep computed its x-axis from `tovez.json`'s `duty_per_speed` while
  the firmware had switched to a baked constant (`Drive::kDutyPerSpeed`,
  `drive.h:138`) — a ~1.6× error, undetectable from the host. It reported a
  28% L/R gain mismatch and a 0.24 breakaway; the truth was 1.9% and ~0.10, and
  it led to a recommendation to inspect a wheel mechanically.
- Both the team-lead's own first sweep and ticket 001's hit this independently.

## Three layers of tuning truth, none observable

1. The robot JSON, baked at build time through `gen_boot_config`.
2. Firmware constants that deliberately ignore the JSON —
   `Drive::kDutyPerSpeed` is "MEASURED, NOT CONFIGURED" (`drive.h:138`), and
   `boot_calibration.cpp:83-93` generates the JSON's `duty_per_speed_*` and
   then ignores it.
3. **Flash-persisted `pid.*`, which silently overrides the JSON at boot**
   (`configurator.cpp:89-103`, `boot_wiring.cpp:108-118`).

Layer 3 is the sharpest: a robot can boot tuning nobody in the room knows is
there, and there is no way to look. See
[[B-persisted-tuning-schema-version-not-bumped]] for the related hazard of
those persisted values being reinterpreted in a new unit domain.

## What to do

Add a `ConfigSnapshot` arm to `config.proto` and a cleartext or binary verb to
request it. It should report the **effective** value of every constant that
influences motion, with its provenance (JSON / baked / persisted), not just the
subset that happens to be configurable.

At minimum: `dutyPerSpeed` per wheel, `vMin`, `biasMax`, all `pid.*` gains,
`deadband`, `kCycle`, the planner limits, and which of the three layers each
came from.

Pair with the wire's other blind spots (post-130 review F6): `msg::Telemetry`
carries **no commanded wheel velocity and no applied duty**, so the setpoint the
controller is chasing never reaches the host either. A capture that shows
measured velocity without the commanded value cannot diagnose a controller.

## Verification

- A host command returns every effective motion constant, and its provenance.
- Deliberately push a `pid.kp` to flash, power-cycle, read back: the snapshot
  reports the persisted value and says it came from flash.
- `duty_sweep.py` and the other characterization tools take their axis from the
  read-back rather than from the JSON — delete
  `duty_sweep.py`'s hand-mirrored `KNOWN_DUTY_PER_SPEED`.
- Telemetry carries per-wheel commanded velocity and applied duty.

## Related

- `docs/post-mortem/2026-08-02-sprint-130-wheels-solid.md` recommendation 1,
  root cause 3.
- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` F6, F7.
- [[B-one-owner-per-constant-speed-floor-and-duty-per-speed]] — read-back is
  what makes the host side of that honest.
- `docs/knowledge/2026-08-02-sprint-130-residuals-and-what-went-wrong.md` —
  "anchor on a measurement that needs no config constant" is the workaround
  this issue removes the need for.
