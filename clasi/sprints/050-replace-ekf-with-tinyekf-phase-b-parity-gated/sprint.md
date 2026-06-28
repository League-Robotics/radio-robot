---
id: '050'
title: Replace EKF with TinyEKF (Phase B, parity-gated)
status: planning-docs
branch: sprint/050-replace-ekf-with-tinyekf-phase-b-parity-gated
use-cases: []
issues:
- consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md
---

# Sprint 050: Replace EKF with TinyEKF (Phase B, parity-gated)

## Goals

Replace the hand-unrolled matrix internals of the 5-state `EKF` with
`TinyEKF` (MIT, header-only, static allocation, float-default) as the
linear-algebra backend, while retaining all hard-won robustness layers:
arc-segment motion model, per-channel Mahalanobis chi-squared gating, D3
gate-recovery, and wedge omega suppression. The switch is parity-gated —
the TinyEKF-backed implementation must pass `test_ekf.py` at parity before
the old internals are deleted.

## Issues addressed

- `consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md` —
  Phase B (EKF replacement; Phase A PID work is Sprint 049)

## Rationale / grouping

Phase B is the higher-risk half and is kept separate so that a parity failure
blocks only this sprint, not the PID work. Running after 049 lets both phases
share the same `libraries/` vendoring infra and dual-build CMake wiring
established in Phase A.

## Scope sketch

- Vendor `TinyEKF` into `libraries/tinyekf/` (`tinyekf.h` + LICENSE);
  configure `EKF_N=5`, `EKF_M=2`; wire into both build paths
- Rebuild `source/state/EKF.{h,cpp}` as a thin layer over `ekf_t`:
  keep motion model (fx, F), three update channels, gating, D3 recovery,
  wedge suppression; delete only the hand-unrolled matrix arithmetic
- Parity gate: new implementation must pass existing `test_ekf.py` in full
  before swapping `PhysicalStateEstimate.cpp` / `Odometry.cpp` references
  and deleting old EKF internals
- Sprint 048 eliminates mecanum `vy` from `PhysicalStateEstimate` / `Odometry`;
  this sprint's EKF layer must align with that cleaned-up state vector
- Detail-planning will produce tickets

## Dependencies

- Sprint 049 (PID/Phase A): provides `libraries/` vendoring infra and
  dual-build CMake wiring; 050 must run after 049
- Sprint 048 (eliminate-ifdef): removes mecanum `vy` from `Odometry` /
  `PhysicalStateEstimate` — the EKF layer touches those files, so 048 must
  land before 050 to avoid merge conflicts

## Success gate

`pytest tests/simulation/unit/test_ekf.py` passes at parity (zero numerical
regressions); `python build.py --clean` produces both firmware and host-sim
binaries; `test_vendor_confinement.py` green.

## Tickets

Tickets execute serially in dependency order.

| # | Title | Depends On |
|---|-------|------------|
| 001 | Vendor tinyekf.h into libraries/tinyekf with provenance preamble | — |
| 002 | Wire libraries/tinyekf include dirs into both CMakeLists.txt build paths | 050-001 |
| 003 | Implement EKFTiny thin layer over ekf_t keeping all robustness layers | 050-002 |
| 004 | Parity gate: verify EKFTiny passes test_ekf.py in full with no new suite failures | 050-003 |
| 005 | Swap Odometry to EKFTiny and delete old EKF.h/EKF.cpp | 050-004 |
| 006 | Final validation: full suite, firmware build, vendor confinement, and TinyEKF constraint check | 050-005 |
