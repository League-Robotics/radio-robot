---
id: '001'
title: Create Superstructure skeleton with Goal enum and requestGoal routing to existing
  MotionController.beginX()
status: open
use-cases:
- SUC-001
- SUC-003
depends-on: []
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 042-001: Create Superstructure skeleton with Goal enum and requestGoal routing to existing MotionController.beginX()

## Description

Create `source/superstructure/Superstructure.{h,cpp}` containing:
- `enum class Goal { IDLE, STREAM, TIMED, DISTANCE, GOTO, TURN, ROTATE, VELOCITY, ARC, ESTOP }`
- `struct GoalRequest` — flat struct carrying all parameters needed by any `beginX()` variant
  (see Implementation Plan for the required fields)
- `class Superstructure` with constructor `(MotionController& mc, HaltController& hc, const RobotConfig& cfg)`
- `requestGoal(const GoalRequest& gr)` — calls `goalAllowed(gr)` (stub: returns `true`), then
  dispatches via `switch(gr.goal)` to the appropriate `_mc.beginX(gr....)` call
- `bool goalAllowed(const GoalRequest& gr)` — stub, returns `true` unconditionally

Repoint `MotionCommandHandlers`: add `Superstructure* superstructure` to `MotionCtx` and
replace `ctx->mc->beginX(...)` calls in `handleVW` (the queue-dequeue path) with
`ctx->superstructure->requestGoal(GoalRequest{...})`. The direct-call fallback paths
(when `ctx->queue == nullptr`) may retain `ctx->mc->beginX(...)` for now (see OQ-2 in
architecture-update.md).

Wire `Superstructure` into `Robot`: add `Superstructure superstructure` value member
(declared AFTER `motionController` and `haltController` — declaration order is
construction order). Wire `superstructure` in `Robot::buildCommandTable` so the
`MotionCtx` gets the `superstructure` pointer.

Add `source/superstructure/` to `tests/_infra/sim/CMakeLists.txt` source glob so the
sim build picks up `Superstructure.cpp`.

After this ticket the **effect is identical** to the pre-ticket state: `requestGoal`
dispatches to exactly the same `beginX()` call that the verb handler previously made
directly. The golden-TLM canary must be byte-exact.

## Acceptance Criteria

- [ ] `source/superstructure/Superstructure.h` exists with `Goal` enum, `GoalRequest` struct,
      and `Superstructure` class declaration.
- [ ] `source/superstructure/Superstructure.cpp` exists with `requestGoal` dispatch switch
      and `goalAllowed()` stub (`return true`).
- [ ] `GoalRequest` carries ALL parameters for every `beginX()` variant: `goal`, `now_ms`,
      `replyFn`, `replyCtx`, `corrId`, `leftMms`, `rightMms`, `durationMs`, `targetMm`,
      `tx`, `ty`, `speedMms`, `headingCdeg`, `epsCdeg`, `relCdeg`, `v_mms`, `omega_rads`,
      `radiusMm`. Every field confirmed against the corresponding `beginX()` signature.
- [ ] `MotionCtx` gains `Superstructure* superstructure`; `handleVW` queue-path branches
      call `ctx->superstructure->requestGoal(GoalRequest{...})` for all nine goal types.
- [ ] `Robot.h` adds `Superstructure superstructure` value member, declared after
      `motionController` and `haltController`.
- [ ] `tests/_infra/sim/CMakeLists.txt` updated: `source/superstructure/` in source glob.
- [ ] Simulation tier green: `uv run --with pytest python -m pytest -q` ≥ 2001 passed,
      0 errors.
- [ ] Golden-TLM canary byte-exact.
- [ ] ARM firmware build: `python3 build.py --fw-only` → 0 errors; then
      `git checkout -- source/robot/DefaultConfig.cpp`.
- [ ] Vendor-confinement grep gate passes (`source/superstructure/` in INSPECT_DIRS):
      zero hits for `MicroBit`, `I2CBus`, vendor register addresses in `source/superstructure/`.
- [ ] No state-graph or transition-table introduced — `requestGoal` is a plain switch.

## Implementation Plan

### Approach

1. Create `source/superstructure/` directory.
2. Write `Superstructure.h`:
   ```
   enum class Goal { IDLE, STREAM, TIMED, DISTANCE, GOTO, TURN, ROTATE, VELOCITY, ARC, ESTOP };
   struct GoalRequest {
       Goal        goal;
       uint32_t    now_ms;
       ReplyFn     replyFn;
       void*       replyCtx;
       char        corrId[16];
       // Wheel-speed goals (S, T)
       float       leftMms;
       float       rightMms;
       uint32_t    durationMs;    // T
       int32_t     targetMm;      // D
       // GoTo (G)
       float       tx;
       float       ty;
       float       speedMms;      // G, R
       // Heading goals (TURN)
       float       headingCdeg;
       float       epsCdeg;
       // Relative rotation (RT)
       float       relCdeg;
       // Body-twist (VW)
       float       v_mms;
       float       omega_rads;
       // Arc (R)
       float       radiusMm;
   };
   ```
3. Write `Superstructure.cpp` with `requestGoal` switch: each case calls the
   corresponding `_mc.beginX()` using the relevant `GoalRequest` fields.
4. Add `Superstructure superstructure` to `Robot.h` AFTER `motionController` and
   `haltController`; include `"Superstructure.h"`.
5. Update `Robot.cpp` constructor: pass `motionController`, `haltController`, `config`
   to `Superstructure` initializer.
6. Add `Superstructure* superstructure` to `MotionCtx` in `MotionCommandHandlers.h`.
7. In `MotionCommandHandlers.cpp` `handleVW`: replace each `ctx->mc->beginX(...)` block
   (on the queue-dequeue path) with `ctx->superstructure->requestGoal(GoalRequest{...})`,
   populating all relevant fields. Retain `ctx->mc->beginX(...)` in the
   `ctx->queue == nullptr` fallback paths (lower risk for T1).
8. In `Robot::buildCommandTable` (or wherever `MotionCtx` is initialized), set
   `ctx.superstructure = &superstructure`.
9. Update `tests/_infra/sim/CMakeLists.txt` to glob `source/superstructure/*.cpp`.

### Files to Create

- `source/superstructure/Superstructure.h`
- `source/superstructure/Superstructure.cpp`

### Files to Modify

- `source/robot/Robot.h` — add `Superstructure superstructure` member + include
- `source/robot/Robot.cpp` — wire `superstructure` in constructor + `buildCommandTable`
- `source/app/MotionCommandHandlers.h` — add `Superstructure* superstructure` to `MotionCtx`
- `source/app/MotionCommandHandlers.cpp` — repoint `handleVW` queue-path branches
- `tests/_infra/sim/CMakeLists.txt` — add `source/superstructure/` glob

### Testing Plan

- Run full simulation tier after each sub-step: `uv run --with pytest python -m pytest -q`.
- Run golden-TLM canary (`test_golden_tlm.py`) explicitly after wiring `requestGoal` into
  the queue path — this is the byte-exactness gate.
- Run `python3 build.py --fw-only` (ARM gate); then `git checkout -- source/robot/DefaultConfig.cpp`.
- Run `test_watchdog_exemption.py`, `test_goto_bounds.py`, `test_incident_scenarios.py`
  as the safety behavior fence.

### Documentation Updates

None beyond the artifact files for this sprint.
