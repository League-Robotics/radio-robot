---
id: '001'
title: Expose seven EKF noise fields via SET/GET (close 067 Open Question 5)
status: open
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: sim-error-model-runtime-settable-hardware-fit.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Expose seven EKF noise fields via SET/GET (close 067 Open Question 5)

## Description

Sprint 067 (ticket 067-003) built a noise-only `setNoise()` push path all the
way from `ConfigRegistry` through `Drive::configure()` to `EKFTiny`, for all
eight EKF process/measurement-noise `RobotConfig` fields — but only wired one
of them (`ekfRHead` → `ekfROtosTheta`) to a registry row. This is confirmed by
direct code read:

- `source/types/Config.h:88-96` already declares all seven still-unregistered
  fields: `ekfQxy`, `ekfQtheta`, `ekfROtosXy`, `ekfQv`, `ekfQomega`,
  `ekfROtosV`, `ekfREncV` (plus the already-registered `ekfROtosTheta`).
- `source/subsystems/drive/Drive.cpp:455-458` — `Drive::configure()`'s
  `_est.setNoise(...)` call already reads and pushes all eight fields live,
  every time a `"drive"`-annotated key is `SET`. This call is unconditional
  and already correct; it is NOT touched by this ticket.
- `source/robot/ConfigRegistry.cpp:105` — only `CFG_F_SS("ekfRHead",
  ekfROtosTheta, "drive")` exists today. The other seven fields have no
  registry row at all, so no wire client can `SET`/`GET` them, even though
  the consumer path is fully wired.

This ticket closes 067's own Open Question 5 (named in
`067-003-add-a-noise-only-ekf-setnoise-path-wire-ekfrhead-through-it.md`,
which explicitly designed `setNoise()`'s signature to accept these seven
fields "so that a future sprint (069 is the likely candidate) can expose them
via the registry with no further plumbing changes"). This is a same-shape,
zero-new-C++-logic, near-zero-risk registry addition — not new modeling.
EKF fusion noise is a real firmware parameter (applies identically on real
hardware and in sim), so it is a `SET` key, not a `SIMSET` key, per
`architecture-update.md` Step 5 item 4 and the Sprint Changes Summary.

## Acceptance Criteria

- [ ] `source/robot/ConfigRegistry.cpp`: seven new rows added to `kRegistry[]`
      immediately after the existing `CFG_F_SS("ekfRHead", ekfROtosTheta,
      "drive")` row (line 105), same macro shape:
      `CFG_F_SS("ekfQxy", ekfQxy, "drive")`,
      `CFG_F_SS("ekfQtheta", ekfQtheta, "drive")`,
      `CFG_F_SS("ekfQv", ekfQv, "drive")`,
      `CFG_F_SS("ekfQomega", ekfQomega, "drive")`,
      `CFG_F_SS("ekfROtosXy", ekfROtosXy, "drive")`,
      `CFG_F_SS("ekfROtosV", ekfROtosV, "drive")`,
      `CFG_F_SS("ekfREncV", ekfREncV, "drive")`.
- [ ] No change to `EKFTiny`, `Odometry`, `PhysicalStateEstimate`, or
      `Drive::configure()` — verify by inspection that `Drive.cpp:455-458`'s
      `setNoise(...)` call is untouched and already reads the live
      `_robCfg` fields these new keys write.
- [ ] Each of the seven keys is `SET`/`GET`-able:
      `SET ekfQxy=<v>` then `GET ekfQxy` round-trips the value (repeat per
      key, or one parametrized test).
- [ ] `SET`-ting any one of the seven keys changes EKF fusion behavior
      observably — new sim test: drive to a non-trivial pose, inject a
      deliberate OTOS position or velocity disagreement, vary one of the new
      keys (e.g. `ekfROtosXy`) between two values, and confirm the
      correction magnitude differs. Follow the existing pattern in
      `067-003`'s regression tests (same shape, applied to a newly-registered
      key instead of `ekfRHead`).
- [ ] No existing EKF-state/covariance disturbance: immediately after `SET`
      of any of the seven keys mid-mission, the fused pose/velocity read back
      identically to their pre-SET values (proves `setNoise()`'s
      non-resetting contract holds for these keys too — mirrors 067-003's
      own "no reset-to-origin regression" acceptance criterion).
- [ ] `docs/protocol-v2.md` §7 Named Key Table gains the seven new rows.
      Do NOT attempt to backfill the table's pre-existing drift (it is
      already missing several long-landed keys, e.g. `vel.kP`, `ekfRHead`
      itself — out of scope, same precedent as ticket 068-001's Open
      Question 1).
- [ ] Full default suite green: `uv run python -m pytest`.

## Testing

- **Existing tests to run**: any existing EKF/OTOS-fusion sim tests
  (e.g. the 067-003 `ekfRHead` regression tests); full default suite.
- **New tests to write**:
  - A parametrized (or per-key) `SET`/`GET` round-trip test for all seven
    new keys.
  - A sim test injecting a deliberate OTOS disagreement and varying one new
    noise key between two values, confirming the correction weighting
    changes (mirrors 067-003's `ekfRHead` fusion-behavior test).
  - A sim test that drives to a non-origin pose, `SET`s one of the new keys,
    and asserts the fused pose/velocity are unchanged immediately after
    (mirrors 067-003's no-reset test).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: This is a pure registry-table addition — the entire consumer
path (`EKFTiny::setNoise()` → `Odometry::setNoise()` →
`PhysicalStateEstimate::setNoise()` → `Drive::configure()`) was already built
and wired by ticket 067-003 and already reads all eight `RobotConfig` fields
live from `_robCfg`. No new C++ behavior is introduced; only wire
reachability is added.

**Files to modify**:
- `source/robot/ConfigRegistry.cpp` — seven new `CFG_F_SS(...)` rows in
  `kRegistry[]`, same shape as the existing `ekfRHead` row at line 105.
- `docs/protocol-v2.md` — §7 Named Key Table, seven new rows only.

**Testing plan**:
- Add/extend a sim test module for `SET`/`GET` round-trip coverage of the
  seven new keys.
- Add/extend a fusion-behavior test (deliberate OTOS disagreement + vary one
  noise key) and a no-reset-on-SET test, following 067-003's existing test
  shapes.
- Run the full default suite (`uv run python -m pytest`) and confirm no
  regressions.

**Documentation updates**: `docs/protocol-v2.md` §7 Named Key Table — seven
new rows, no other changes to that section.
