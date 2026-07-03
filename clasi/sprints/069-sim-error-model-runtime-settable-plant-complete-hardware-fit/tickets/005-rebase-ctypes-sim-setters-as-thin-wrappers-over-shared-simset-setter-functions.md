---
id: '005'
title: Rebase ctypes sim setters as thin wrappers over shared SIMSET setter functions
status: open
use-cases:
- SUC-002
- SUC-003
- SUC-004
- SUC-005
depends-on:
- '003'
- '004'
github-issue: ''
issue: sim-error-model-runtime-settable-hardware-fit.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Rebase ctypes sim setters as thin wrappers over shared SIMSET setter functions

## Description

After tickets 002-004, every sim plant/error knob has TWO potential entry
points: the pre-existing `extern "C"` ctypes surface
(`tests/_infra/sim/sim_api.cpp`) and the new `SIMSET`/`SIMGET` wire surface
(`source/commands/SimCommands.cpp`'s `kSimRegistry[]`). The sprint's own
stated requirement (Sprint Changes Summary item 1, Design Rationale
Decision 3) is a SINGLE source of truth per knob — the ctypes functions
must become thin wrappers over the SAME setter functions the registry
dispatches to, not a second, independently-maintained call site.

Several existing ctypes functions ALREADY are a one-line forward directly to
a `PhysicsWorld`/`SimOdometer` method (e.g. `sim_set_encoder_noise`,
`sim_api.cpp:638-640`, already just calls
`static_cast<SimHandle*>(h)->hal.plant().setEncoderNoise(side, sigma_mm)`).
For these, "rebasing" means confirming (and, where `SimCommands`'s registry
row for the same key calls a small per-side adapter function rather than the
raw method directly — e.g. an `encScaleErrL` row needs an adapter that
hardcodes `side=0`) that the ctypes function and the registry row call
EXACTLY the same underlying code path, extracting any per-row adapter logic
introduced in tickets 003/004 into a small, shared, non-static free-function
layer both call. `sim_set_motor_slip` is explicitly OUT of this ticket's
scope — it is the pre-existing, untouched `_rotationalSlip`/`setSlip()`
test-infra channel (Design Rationale Decision 4), not one of this sprint's
new knobs.

`tests/_infra/sim/drive_api.cpp` needs NO change — confirmed by
`architecture-update.md`'s Impact table: the Drive-level test harness never
touches the new scrub/error fields directly (it exercises `Drive`, not raw
`PhysicsWorld`). `host/robot_radio/io/sim_conn.py` needs NO interface
change — its existing per-field ctypes wrapper methods keep calling the
same-named, same-signature ctypes functions; only `transport.py`'s
`apply_error_profile()` (ticket 007) stops calling them in favor of a single
`SIMSET` string. This ticket's job is confined to the C++ side; verify (not
modify) the two Python files.

## Acceptance Criteria

- [ ] Any per-row adapter function introduced by tickets 003/004 inside
      `SimCommands.cpp` (e.g. a function that adapts
      `PhysicsWorld::setEncoderScaleError(int side, float)` to the
      per-key, per-side `encScaleErrL`/`encScaleErrR` registry rows) is
      extracted to a small, shared, non-static free-function layer — e.g.
      `namespace simsetters { void encoderScaleErrorL(SimHardware&, float);
      void encoderScaleErrorR(SimHardware&, float); … }` — reachable from
      BOTH `SimCommands.cpp` and `tests/_infra/sim/sim_api.cpp` (a small new
      header, or an existing shared sim header, whichever keeps the
      dependency direction clean — `SimCommands` and `sim_api.cpp` both
      already depend on `SimHardware`/`PhysicsWorld`, so this introduces no
      new dependency direction).
- [ ] `tests/_infra/sim/sim_api.cpp`'s existing `sim_set_encoder_noise`,
      `sim_set_otos_linear_noise`, `sim_set_otos_yaw_noise`, and
      `sim_set_motor_offset` bodies are rewritten as one-line forwards to
      the shared `simsetters::` functions (or confirmed, where already a
      direct one-line forward to a `PhysicsWorld`/`SimOdometer` method with
      no per-row adaptation needed, to already satisfy "single source of
      truth" — no change required in that case; document which is which).
- [ ] The two ticket-002 ctypes forwards (`sim_set_body_rot_scrub`,
      `sim_set_body_lin_scrub`) are rewritten to call the same
      `simsetters::` function(s) `SimCommands`'s `bodyRotScrub`/
      `bodyLinScrub` registry rows call — no duplicated
      `.hal.plant().setBodyRotationalScrub(...)` call site.
  - [ ] `sim_set_motor_slip` is explicitly left UNTOUCHED — verify by
      inspection that no change is made to it or its call site.
- [ ] `tests/_infra/sim/drive_api.cpp`: confirmed unaffected (no scrub/error
      field access) — no code change; note this explicitly in the PR/ticket
      close-out rather than silently skipping it.
- [ ] `host/robot_radio/io/sim_conn.py`: confirmed unaffected in interface —
      every existing per-field ctypes wrapper method (`set_slip`,
      `set_encoder_noise`, `set_otos_noise`, `set_motor_offset`, etc.) keeps
      its exact Python signature and still calls the same-named ctypes
      function; no code change required (the C-side rebase is
      behavior-preserving by construction). Run the existing pytest
      fixtures that call these methods directly to confirm.
- [ ] Every existing pytest test that calls one of the rebased ctypes
      functions directly (not through `SIMSET`) passes unchanged —
      behavior-preserving refactor, not a behavior change.
- [ ] Full default suite green: `uv run python -m pytest`.

## Testing

- **Existing tests to run**: every test that calls
  `sim_set_encoder_noise`/`sim_set_otos_linear_noise`/
  `sim_set_otos_yaw_noise`/`sim_set_motor_offset`/`sim_set_body_rot_scrub`/
  `sim_set_body_lin_scrub` directly (grep for each name across `tests/` to
  enumerate); `test_sim_otos_lever_arm.py` (066-001, uses
  `sim_set_motor_slip` — confirm untouched); full default suite.
- **New tests to write**: none required — this is a behavior-preserving
  refactor; existing tests are the regression net. If the shared
  `simsetters::` layer's extraction reveals a case where the ctypes function
  and the registry row previously did NOT call identical logic (a bug this
  ticket would then be fixing, not just refactoring), add a targeted
  regression test for that specific case.
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: This is a de-duplication and single-source-of-truth pass, not
new functionality. Identify every ctypes function whose body is not already
an exact match for the corresponding `SimCommands` registry row's call path,
extract the shared logic into a small `namespace simsetters` free-function
layer, and point both call sites at it. Where a ctypes function is already a
bare one-line forward directly to a `PhysicsWorld`/`SimOdometer` method (the
common case), no change is needed — verify and note rather than churn
working code.

**Files to modify**:
- `tests/_infra/sim/sim_api.cpp` — rebase the enumerated ctypes function
  bodies onto the shared `simsetters::` layer.
- `source/commands/SimCommands.cpp` — extract any per-row adapter logic to
  the same shared layer (new header, e.g. `source/commands/SimSetters.h`, or
  co-located with `SimCommands.h` if that keeps the include graph simplest).

**Files to verify, not modify**:
- `tests/_infra/sim/drive_api.cpp` — confirm no scrub/error field access.
- `host/robot_radio/io/sim_conn.py` — confirm interface unchanged.

**Testing plan**:
- Grep `tests/` for every ctypes function name touched by this ticket;
  re-run each matching test file and confirm unchanged pass/fail.
- Full `uv run python -m pytest`.

**Documentation updates**: none — no wire-protocol or Python-interface
change; this is an internal C++ implementation-sharing refactor.
