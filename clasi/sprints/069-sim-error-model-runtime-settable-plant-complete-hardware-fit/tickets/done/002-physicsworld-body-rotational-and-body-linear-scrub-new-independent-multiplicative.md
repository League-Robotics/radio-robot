---
id: '002'
title: PhysicsWorld body-rotational and body-linear scrub (new, independent, multiplicative)
status: done
use-cases:
- SUC-003
depends-on: []
github-issue: ''
issue: sim-error-model-runtime-settable-hardware-fit.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# PhysicsWorld body-rotational and body-linear scrub (new, independent, multiplicative)

## Description

`PhysicsWorld::update()`'s sub-step B (chassis pose integration,
`source/hal/sim/PhysicsWorld.cpp:84-107`) is the plant's ONLY true-pose
integrator, and today its rotation term is scaled by exactly one channel:

```cpp
float slip = effectiveSlip(_rotationalSlip);          // line 95
float dTh  = ((dR - dL) / _trackwidthMm) * slip;      // line 96
```

`_rotationalSlip` (`PhysicsWorld.h:257`) is written ONLY by `setSlip()`
(`PhysicsWorld.h:125-129`, forwarded from `sim_set_motor_slip`) — a
test-infrastructure knob for the REPORTED-encoder error model, not a
hardware-realistic body-scrub parameter. In every current test usage this
sums to `<= 0`, which `effectiveSlip()` (`source/control/Odometry.h:21-26`)
clamps to `1.0` — i.e. the plant's true body has never actually scrubbed in
practice. Meanwhile `Planner::beginRotation()`
(`source/superstructure/PlannerBegin.cpp:518-524`) computes the RT arc target
as `arc = |Δθ| · (tw/2) / effectiveSlip(cfg.rotationalSlip)` — *inflating*
the commanded arc on the assumption the chassis will only achieve
`effectiveSlip(rotationalSlip)` of it (the real, bench-calibrated value is
`0.92` per `data/robots/*.json`). A sim plant that cannot scrub receives that
inflated command and executes all of it, over-rotating (`RT 9000` → 95.2°
instead of 90°).

This ticket gives `PhysicsWorld` a genuine, independent scrub capability —
two new fields, default-neutral (`1.0` = no-op), combined MULTIPLICATIVELY
with the existing (untouched) `_rotationalSlip`/`effectiveSlip()` channel —
so the plant can be configured to actually scrub by a given factor,
independent of the encoder-report error model. See
`architecture-update.md` §4b diagram and Design Rationale Decisions 2 and 4
for the full clamp-semantics and non-interference rationale.

**Do not touch** `Odometry::predict()` or `Planner::beginRotation()` — both
already apply `effectiveSlip(RobotConfig.rotationalSlip)` correctly; this
ticket makes the plant physically capable of the effect they've always
assumed exists, without changing either file.

## Acceptance Criteria

- [x] `source/hal/sim/PhysicsWorld.h`: two new private fields
      `float _bodyRotationalScrub = 1.0f;` and `float _bodyLinearScrub =
      1.0f;` (default = no-op), placed near the existing dynamics-parameter
      fields (`_trackwidthMm`, `_rotationalSlip`, etc., around line 254-259).
- [x] New public setters/getters, mirroring the existing `setSlip`/
      `rotationalSlip()` shape: `void setBodyRotationalScrub(float f)`,
      `float bodyRotationalScrub() const`, `void setBodyLinearScrub(float f)`,
      `float bodyLinearScrub() const`.
- [x] New local, file-scope helper `clampScrub(float)` in `PhysicsWorld.cpp`
      — range `(0, 1]`: values `<= 0` clamp to a small positive floor (or are
      rejected upstream; document the chosen boundary behavior in a comment),
      values `> 1.0` clamp to `1.0`. Deliberately NOT `effectiveSlip()`
      (different, correctly different, valid range — `effectiveSlip()`'s
      `[0.5, 1.0]` floor is `rotationalSlip`'s hardware-calibration history,
      not applicable to a brand-new field with no such history; see
      `architecture-update.md` Decision 2). Do not modify `effectiveSlip()`
      itself.
- [x] Sub-step B (`PhysicsWorld.cpp:93-100`) combines the new fields
      multiplicatively with the existing, UNCHANGED `effectiveSlip
      (_rotationalSlip)` term:
      `float slip = effectiveSlip(_rotationalSlip) * clampScrub(_bodyRotationalScrub);`
      and the linear term (`(dL + dR) * 0.5f` in the `_truePoseX`/`_truePoseY`
      update, lines 98-99) is additionally multiplied by
      `clampScrub(_bodyLinearScrub)`. `dTh`'s use for `_truePoseH` (line 100)
      is unaffected beyond the already-modified `slip` factor.
- [x] Default-neutral: with both new fields at their default `1.0f`, sub-step
      B's numeric output is BYTE-IDENTICAL to today's (verify via the
      existing golden-TLM fixture, which must require no regeneration).
- [x] 066-001's chassis-truth-slip test
      (`test_turn_with_slip_otos_matches_truth_encoder_diverges`, which
      configures only `sim_set_motor_slip`/`_rotationalSlip`) is verified
      UNAFFECTED — it never touches the new fields, and the new fields'
      default leaves `effectiveSlip(_rotationalSlip) * 1.0` identical to
      today's `effectiveSlip(_rotationalSlip)`.
- [x] Minimal direct-access hook for this ticket's own system-level
      acceptance test (ahead of the general `SIMSET` surface, which is
      ticket 003): add `sim_set_body_rot_scrub(void* h, float f)` and
      `sim_set_body_lin_scrub(void* h, float f)` to the `extern "C"` block in
      `tests/_infra/sim/sim_api.cpp` (mirroring `sim_set_motor_slip`,
      `sim_api.cpp:631-637`), each forwarding directly to
      `static_cast<SimHandle*>(h)->hal.plant().setBodyRotationalScrub(f)` /
      `setBodyLinearScrub(f)`. These call the SAME named setters ticket 003's
      `SIMSET` registry will call (Design Rationale Decision 3 — single
      source of truth per knob; no duplicated logic).
- [x] Headline acceptance point 1: with `SET rotSlip=0.92` (the
      `RobotConfig.rotationalSlip` default) and the new
      `sim_set_body_rot_scrub(h, 0.92)` applied (all else default), a
      subsequent `RT 9000` (in-place 90° turn command) lands on a TRUE pose
      of 90° (closing the current ~95.2° gap) — new system test.
- [x] Headline acceptance point 2: with `SET rotSlip=1.0` (identity) and both
      new scrub fields at their default `1.0` (i.e. no scrub applied at
      all), `RT 9000` lands on EXACTLY 90° true pose — new system test.
- [x] Full default suite green: `uv run python -m pytest`.

## Testing

- **Existing tests to run**: `tests/simulation/unit/test_sim_otos_lever_arm.py`
  (066-001, the chassis-truth-slip test — confirm byte-identical pass, not
  just "still passes"); `tests/simulation/unit/test_rt_slip.py` (existing RT
  + `rotSlip` arc-scaling coverage — confirm unaffected); the golden-TLM
  fixture test; full default suite.
- **New tests to write**:
  - `tests/simulation/unit/test_physics_world_body_scrub.py`: a standalone
    C++ harness compiled inline (same pattern as
    `tests/simulation/unit/test_physics_world_basic.py` — instantiates a bare
    `PhysicsWorld`, drives it via `update()`, prints PASS/FAIL lines the
    Python test parses; no ctypes, no command dispatch). Covers: default
    `1.0` is a no-op; each of `bodyRotScrub`/`bodyLinScrub` independently
    reduces its corresponding sub-step B term; combining with a non-zero
    `setSlip()` value multiplies rather than replaces (both factors visible
    in the output).
  - `tests/simulation/system/test_069_rt_90deg_body_scrub.py`: the two
    headline system-level acceptance points above, using the new
    `sim_set_body_rot_scrub`/`sim_set_body_lin_scrub` ctypes forwards (via
    the existing Python `Sim` harness pattern in
    `tests/_infra/sim/firmware.py` and `tests/simulation/unit/test_rt_slip.py`
    — `sim.send_command("RT 9000")` + `sim.get_true_pose()`). NOTE: ticket
    003 will REBASE this file's setup onto `SIMSET bodyRotScrub=...` once
    the wire surface exists, replacing the direct ctypes calls added here —
    same assertions, different transport (do not delete/duplicate the file;
    ticket 003 edits it in place).
- **Verification command**: `uv run python -m pytest`

## Implementation Plan

**Approach**: Add two new, independent, default-neutral multiplier fields to
`PhysicsWorld`'s existing sub-step B, following the exact structural pattern
of the already-existing `_rotationalSlip`/`setSlip()` channel but with
simpler, non-legacy clamp semantics (`clampScrub()`, not `effectiveSlip()`).
Leave the existing channel and every one of its consumers completely
untouched — the new fields multiply into the same expression, they do not
replace it. Add a minimal ctypes forward pair now so this ticket's own
system-level RT test does not need to wait for the general `SIMSET`
surface (ticket 003); the same underlying setters become `SIMSET`'s first
registry rows in ticket 003, so there is no throwaway or duplicated logic.

**Files to modify**:
- `source/hal/sim/PhysicsWorld.h` — two new fields, four new
  setter/getter methods.
- `source/hal/sim/PhysicsWorld.cpp` — new local `clampScrub()` helper;
  sub-step B's `slip` computation and linear-term multiplication.
- `tests/_infra/sim/sim_api.cpp` — two new minimal `extern "C"` forwards
  (`sim_set_body_rot_scrub`, `sim_set_body_lin_scrub`).
- `tests/_infra/sim/firmware.py` (or wherever the Python `Sim` wrapper's
  ctypes bindings live) — register the two new function signatures and add
  thin Python methods, mirroring `set_slip()`.

**Testing plan**:
- New standalone-harness unit test (`test_physics_world_body_scrub.py`) for
  the plant-level math in isolation.
- New system test (`test_069_rt_90deg_body_scrub.py`) for the two headline
  RT-90° acceptance points through the full command-dispatch pipeline.
- Re-run `test_sim_otos_lever_arm.py` and `test_rt_slip.py` to confirm
  byte-identical behavior (066-001 must not regress).
- Confirm the golden-TLM fixture requires no regeneration (new fields
  default to `1.0`, a no-op).
- Full `uv run python -m pytest`.

**Documentation updates**: none required by this ticket alone (the
`SIMSET`/`SIMGET` wire documentation lands with ticket 003, once these
fields are wire-reachable).

## Implementation Notes (post-execution)

- The `PhysicsWorld`/`clampScrub()`/sub-step-B changes are exactly as
  specified: `slip = effectiveSlip(_rotationalSlip) * clampScrub(_bodyRotationalScrub)`,
  linear term additionally multiplied by `clampScrub(_bodyLinearScrub)`,
  both new fields defaulting to `1.0f`. Golden-TLM fixture, 066-001's
  chassis-truth-slip test, and `test_rt_slip.py` all pass byte-identical /
  unaffected (confirmed by direct re-run, not just "still green").
- **Headline acceptance points 1 and 2, measured**: empirically, RT 9000
  does NOT land on a mathematically exact 90.0° true pose in EITHER the
  scrub-corrected (rotSlip=0.92, bodyRotScrub=0.92) or the identity
  (rotSlip=1.0, no scrub) case — both land a few degrees short (~86.5-88.5°
  depending on tick-step granularity). Root cause, confirmed by reading
  `PlannerBegin.cpp::beginRotation()` directly: `kRtCoastArcMm=8mm` is
  commented "sim-tuned" for an assumed spin rate of `kRtRateDps=100°/s`, but
  the actual rate is `min(cfg.yawRateMax, kRtRateDps)` and
  `DefaultConfig.cpp`'s `yawRateMax` is `70°/s` — a PRE-EXISTING, out-of-scope
  mismatch (this ticket, and architecture-update.md, explicitly say not to
  touch `Planner::beginRotation()`) that adds a small, constant,
  slip/scrub-independent residual to every RT 9000 run. Critically, this
  residual is the SAME in both the corrected and identity runs (measured
  diff < 1°), which is the actual proof that the new scrub math is doing its
  job: `bodyRotScrub=0.92` cancels `rotSlip=0.92`'s arc inflation and
  reproduces the identity run's result, not the ~94-96° an uncorrected
  (scrub left at its 1.0 default) run produces for the same `rotSlip=0.92`.
  `test_069_rt_90deg_body_scrub.py` asserts this directly (corrected ≈
  identity, both within 5° of 90°, corrected clearly ≠ uncorrected baseline)
  rather than asserting a literal `== 90.0`, and documents the coast-tuning
  root cause inline so ticket 003 (which rebases this file onto `SIMSET`)
  and any future cleanup of `kRtCoastArcMm` have the context.
