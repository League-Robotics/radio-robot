---
status: pending
split_from: firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md
sprint: '125'
---

# Firmware base hardening: characterization, gate, and freeze

## Scope (split from the original directive, 2026-07-24)

This is the characterization/gate/freeze half of the original
`firmware-base-hardening-bounded-wheel-moves-and-wheel-observer.md`
directive, split off so the base-contract half (duty primitive, observer,
`NezhaMotor` shrink — sprint 124) and this half (sprint 125) can each be
planned and executed as their own right-sized sprint. The sibling issue
(kept under the original filename) now scopes to the base-contract
change alone; read it first for the observer/duty-boundary design this
gate measures.

## Characterization (constants with derivations — never swept)

- Sim: the plant is first-order by construction — the observer must track
  it EXACTLY (validates machinery; any residual is a bug, not tuning).
- Bench: per-wheel step-response battery (both directions, several
  speeds, loaded on the stand): dead time, effective tau, deadband floor,
  reversal-dwell effect; recorded per-robot in the JSON with measurement
  provenance. Sensitivity note vs battery voltage stated (observer's
  correction step absorbs slow drift; characterize, don't chase).
- Every constant names its measurement; the no-sweep rule applies in
  full.

## The base gate (numbers, then freeze)

- **Observer fidelity:** sim — exact (≤0.1 mm / ≤0.1 mm/s vs plant
  truth); bench — stated band between observer estimate and encoder
  truth at the next sample (e.g. ≤5 mm/s during cruise, ≤ one dead-time's
  worth during transients), measured across the step battery.
- **Completion of the duty write path:** dwell/deadband shaping visible
  in telemetry, applied duty reported (`appliedDuty` in `WheelSample`),
  zero-on-silence verified (a cycle with no `drive()` call writes zero).
- **Telemetry truthfulness:** same-generation L/R pairing test, rate test
  (~25 Hz sustained), gap-free sequence numbers.
- Gate green ⇒ the base is FROZEN: subsequent base changes require a
  stakeholder-signed issue; the motion library treats the boundary + gate
  numbers as a stable platform contract from this point on.

## Depends on

Sprint 124 (the duty-boundary migration, observer, and `NezhaMotor`
shrink) must land first — this issue measures and gates THAT change, it
does not itself change the base contract.

## Sequencing

Third and final sprint of the firmware-base-hardening restructuring
(123 COBS+CRC → 124 duty boundary/observer → 125 this issue). Bench
halves of the gate ride the existing bench-session cadence.
