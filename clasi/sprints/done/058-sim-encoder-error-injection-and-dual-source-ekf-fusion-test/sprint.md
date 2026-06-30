---
id: 058
title: Sim encoder-error injection and dual-source EKF fusion test
status: done
branch: sprint/058-sim-encoder-error-injection-and-dual-source-ekf-fusion-test
use-cases:
- SUC-001
- SUC-002
issues:
- sim-encoder-error-and-dual-source-fusion-test.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 058: Sim encoder-error injection and dual-source EKF fusion test

## Goals

Close the gap in the drive-simulation error model: add deterministic per-wheel
encoder error knobs to `SimMotor`/`PhysicsWorld` (mirroring the OTOS error knobs
added in 057-005), expose them via a C-ABI shim, and add a dual-noisy-source EKF
fusion test that injects error into BOTH the encoder and optical paths and asserts
the fused estimate beats each raw source individually.

## Problem

Sprint 057's `test_ekf_fusion_beats_noise` injects error only into the OTOS
path; the encoder reads ground truth perfectly. This proves "EKF discards a bad
OTOS and trusts a clean encoder" — NOT genuine sensor fusion. The stakeholder
explicitly asked for error versions of both the encoder and optical flow to
exercise the EKF as it operates on the real robot.

## Solution

1. Add scale-error and slip knobs to the `PhysicsWorld` reported-encoder
   accumulation path, with `SimMotor` setters and a C-ABI shim
   `drive2_api_enable_encoder_sim_model`.
2. Add `tests/simulation/unit/test_ekf_dual_source.py` with a dual-source
   fusion test (both sensors noisy) and a regression of the 057-005
   encoder-good/OTOS-bad scenario.

## Success Criteria

- `python build.py --clean` produces zero errors.
- `uv run python -m pytest` passes at baseline "2377 passed, 2 failed" PLUS all
  new tests passing.
- The dual-source test genuinely asserts `fused_err < encoder_only_err` AND
  `fused_err < optical_only_err` with non-trivial error magnitudes in both sensors.

## Scope

### In Scope

- New encoder error fields/setters in `PhysicsWorld` (scale error, slip per wheel).
- New setters `setScaleError` / `setSlip` on `SimMotor`.
- New C-ABI shim `drive2_api_enable_encoder_sim_model` in `drive2_api.cpp`.
- New test file `tests/simulation/unit/test_ekf_dual_source.py`.

### Out of Scope

- Changes to live firmware paths (`loopTickOnce`, `Drive::periodic`).
- Mecanum / holonomic drivetrain simulation.
- Protobuf or message-bus architecture changes.
- `SimOdometer` changes (already complete in 057-005).

## Test Strategy

Unit simulation tests only. The new test exercises `Drive2` on `SimHardware`
via the existing C-ABI shim layer. No hardware required.

Test command: `uv run python -m pytest`

## Architecture Notes

Encoder error is applied at the `PhysicsWorld::update()` reported-encoder delta
level, consistent with the existing `setEncoderNoise` pattern. Default zero means
no behavioral change. All new parameters are sim-only and do not affect the
device (firmware) build.

## GitHub Issues

None.

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [x] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [x] Architecture review passed
- [x] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Encoder error injection + dual-source EKF fusion test | — |

Tickets execute serially in the order listed.
