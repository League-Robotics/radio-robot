---
id: '005'
title: 'Publish wiring: NezhaMotor + Otos rings (incl. velocity)'
status: open
use-cases:
- SUC-115-001
depends-on:
- '004'
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Publish wiring: NezhaMotor + Otos rings (incl. velocity)

## Description

Depends on ticket 004 (`Devices::Measurements` + record types must exist
first). Wires the two devices/-layer leaf producers: `NezhaMotor`
(encoder samples → `encoderLeft`/`encoderRight`) and `Otos` (pose+velocity
bursts → `otos`). Per sprint.md Architecture Decision 1, each producer
receives a reference to ONLY its own ring — never the whole
`Devices::Measurements` container — to avoid giving a leaf visibility
into rings it never touches.

`Otos::tick()` already burst-reads position AND velocity
(`readPositionVelocity()`) but `applyOtosSample()` (`odometry.cpp`)
currently drops the velocity before it ever reaches TLM. This ticket does
**not** change that — TLM's `otos=` field and `Otos::pose()`'s public
surface stay exactly as they are. The velocity is captured only at the
NEW ring-publish call, which is additive.

## Implementation Plan

- **Approach**:
  - `Devices::NezhaMotor`: add a `MeasurementRing<EncoderRecord>&`
    constructor dependency (the caller passes either the `encoderLeft` or
    `encoderRight` member, matching which wheel this instance is). Inside
    `tick()`, in the SAME freshness-gated branch that already computes
    velocity/glitch detection (see `nezha_motor.h`'s own tick() comment:
    "Velocity/glitch computation is gated on a FRESHNESS check"), publish
    an `EncoderRecord{stamp=nowUs, velocity=velocity(), position=position()}`
    — only on an accepted (non-glitch) fresh sample; a rejected sample
    publishes nothing.
  - `Devices::Otos`: add a `MeasurementRing<PoseRecord>&` constructor
    dependency (the `otos` member). Inside `tick()`, immediately after a
    successful `readPositionVelocity()` burst (same place `lastReadUs_`
    is already updated), publish a `PoseRecord{stamp=lastReadUs_, v_x,
    v_y, omega, x, y, heading}` using the SAME burst's position AND
    velocity (the velocity this leaf already reads and currently
    discards downstream) — apply the same mounting-yaw/lever-arm
    transform `pose()` already applies to position, or document if the
    ring intentionally carries the raw sensor-frame reading instead
    (implementer's call — state the choice in completion notes; prefer
    consistency with `pose()`'s existing centre-frame convention unless
    there is a concrete reason not to). A rate-limited-skip tick or a
    burst failure publishes nothing (mirrors `poseFresh()`'s existing
    semantics).
  - Wire the new constructor dependencies through `main.cpp` and the sim
    composition root (both already touched by ticket 004; this ticket
    passes the specific ring references, not just constructs the
    container).
- **Files to modify**: `src/firm/devices/nezha_motor.h`/`.cpp`,
  `src/firm/devices/otos.h`/`.cpp`, `src/firm/main.cpp`, sim composition
  root.
- **Testing plan**: sim test spinning both wheels and letting `Otos` tick
  (sim OTOS plant), asserting `encoderLeft`/`encoderRight`/`otos` rings
  hold plausible, monotonically-increasing `stamp` values and nonzero
  `velocity`/`v_x` respectively after a few cycles. Assert I2C
  transaction count per cycle is unchanged from a pre-ticket baseline run
  (publish is a pure memory write, zero added bus traffic).
- **Documentation updates**: none beyond inline doc comments on the new
  constructor parameters, following each file's existing comment density.

## Acceptance Criteria

- [ ] `NezhaMotor::tick()` publishes exactly one `EncoderRecord` per
      accepted (non-glitch) fresh sample; a rejected sample publishes
      nothing.
- [ ] `Otos::tick()` publishes a `PoseRecord` carrying `v_x`/`v_y`/`omega`
      (not zeroed/dropped) on every successful burst; a rate-limited-skip
      or burst failure publishes nothing.
- [ ] `Otos`'s existing `pose()`/`connected()`/`present()` public surface
      and TLM's `otos=` field are byte-for-byte unchanged — only the ring
      gains the velocity data.
- [ ] I2C transaction count per cycle is unchanged from the pre-ticket
      baseline (verified via sim transaction counter or equivalent).
- [ ] Sim test: spin wheels + let OTOS tick, confirm
      `encoderLeft`/`encoderRight`/`otos` rings hold plausible,
      monotonically-timestamped, nonzero-velocity records.

## Testing

- **Existing tests to run**: any existing `NezhaMotor`/`Otos` sim-unit
  tests (encoder glitch-rejection, OTOS burst-read tests); full
  `uv run python -m pytest` sim suite; `just build-clean`.
- **New tests to write**: see Implementation Plan's Testing Plan bullet.
- **Verification command**: `uv run pytest`
