---
id: 053-005
title: "Eliminate R stringify round-trip; collapse VELOCITY goal"
status: open
use-cases:
- SUC-002
- SUC-004
depends-on:
- 053-001
- 053-002
issue: stop-conditions-as-a-first-class-system-primitive.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 053-005: Eliminate R stringify round-trip; collapse VELOCITY goal

## Description

`handleR` currently packs `speed=<mm/s>` and `radius=<mm>` as KV strings into
a VW ArgList, calls `pushVW`, and `handleVW` reads them back via
`argsHasKey(args, "radius")` + `argsScanKV` to route to `beginArc`. This is
the last stringify/re-parse round-trip for the open-loop family.

After this ticket:
1. `handleR` calls `requestGoal` directly with `Goal::VELOCITY` (arc is an
   open-loop twist command, not a separate goal variant), computing
   `omega = speed / radius` inline (same math as `beginArc`).
2. `Goal::ARC` is removed from the enum.
3. `handleVW`'s `argsHasKey(args, "radius")` block is removed.
4. `packKVArg`, `argsHasKey`, `argsScanKV` for `radius`/`speed` keys are removed.
5. If `packKVArg` has no remaining callers, it is deleted.
6. `beginArc` in `MotionControllerBegin.cpp` may be retained as a named entry
   point (it computes omega from speed/radius) or inlined into `beginVelocity`.
   Preferred: retain `beginArc` and have the VELOCITY case in Superstructure
   call it when the GoalRequest carries a non-zero `radiusMm` field, OR inline
   the omega computation in `handleR` and use `beginVelocity` directly.

The simpler path: compute omega in `handleR`, pass `(speed, omega)` as
`v_mms/omega_rads` in GoalRequest, route to `beginVelocity`. `beginArc` becomes
dead code and can be removed or deprecated.

After tickets 003, 004, 005: `handleVW` no longer has any KV demux for
t=/dist=/stream=/radius=. It retains `h=` (TURN), `rot=` (RT), and `x=/y=`
(G) demux — these are closed-loop goals and their KV push is intentional.

## Acceptance Criteria

- [ ] `handleR` in `MotionCommands.cpp`:
  - Computes `omega_rads = (radius != 0) ? (float)speed / (float)radius : 0.0f`.
  - Builds `GoalRequest gr{}` with:
    - `goal = Goal::VELOCITY`.
    - `v_mms = (float)speed`, `omega_rads = omega_rads`.
    - `doneLabel = "EVT done R"`.
    - `streamSeed = false`.
    - `nStops = 0` (open-ended; any stop= from `args[2..]` packed into
      `gr.stops[]`).
  - Calls `ctx->superstructure->requestGoal(gr)`.
  - Does NOT call `pushVW` or `packKVArg`.
  - Calls `replyOK` before `requestGoal`.
  - Queue-null fallback: calls `_mc.beginArc(...)` directly (or
    `beginVelocity(speed, omega, ...)`) and applies stop= via
    `mc_applyStopClauses`.
- [ ] `handleVW` in `MotionCommands.cpp`:
  - The `if (argsHasKey(args, "radius"))` block is removed.
  - `argsScanKV(args, "radius", ...)` and `argsScanKV(args, "speed", v)` calls
    (inside the former radius block) are removed.
- [ ] `Goal::ARC` is removed from the `Goal` enum in `Superstructure.h`.
- [ ] The `case Goal::ARC:` in `Superstructure::requestGoal` is removed.
- [ ] `packKVArg` static helper in `MotionCommands.cpp` is removed (no remaining
  callers after this ticket removes the last usage from handleR). Verify by
  searching for `packKVArg` before deleting.
- [ ] `argsHasKey` and `argsScanKV` for `speed`/`radius` keys removed from
  `handleVW`. `argsHasKey`/`argsScanKV` themselves are retained if still needed
  by h=/rot=/x=/y= checks in handleVW (verify).
- [ ] `beginArc` in `MotionControllerBegin.cpp` is either removed (if no
  remaining callers) or marked deprecated. If removed, update
  `MotionController.h` declaration.
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` passes with
  exactly 2 known failures. Existing R tests pass.
- [ ] `python build.py --clean` exits 0.

## Implementation Plan

### Approach

Rewrite `handleR` to call `requestGoal` directly. Remove the radius=/speed=
demux block from `handleVW`. Remove `Goal::ARC`. Clean up `packKVArg` and
`beginArc` if no callers remain.

### Files to Modify

- `source/commands/MotionCommands.cpp`
  - `handleR`: rewrite queue path. Compute `omega_rads`. Build GoalRequest.
    Pack any stop= from `args[2..]` into `gr.stops[]`. Call `replyOK` then
    `requestGoal`.
  - `handleVW`: remove the `if (argsHasKey(args, "radius"))` block.
  - Remove `packKVArg` static function (search codebase for remaining uses
    first with grep).
  - Audit whether `argsHasKey` and `argsScanKV` are still needed for remaining
    TURN/G paths. If only `h=`/`eps=`/`x=`/`y=`/`rot=` remain, retain the
    helpers. If they are now only used for one key, consider inlining.

- `source/superstructure/Superstructure.h`
  - Remove `ARC` from the `Goal` enum.

- `source/superstructure/Superstructure.cpp`
  - Remove `case Goal::ARC:` from `requestGoal`.

- `source/superstructure/MotionController.h`
  - Remove or deprecate `beginArc` declaration (if removing; add `[[deprecated]]`
    if keeping for any sim fallback path).

- `source/control/MotionControllerBegin.cpp`
  - Remove or deprecate `beginArc` definition (if removing).

### GoalRequest for R

```cpp
// handleR:
float omega_rads = (radius != 0) ? ((float)speed / (float)radius) : 0.0f;
GoalRequest gr{};
gr.goal       = Goal::VELOCITY;
gr.robot      = ctx->robot;
gr.now_ms     = now;
gr.replyFn    = replyFn;
gr.replyCtx   = replyCtx;
gr.corrId     = corrId;
gr.v_mms      = (float)speed;
gr.omega_rads = omega_rads;
gr.doneLabel  = "EVT done R";
gr.streamSeed = false;
// Pack stop= from args[2..]:
for (int i = 2; i < args.count && gr.nStops < MotionCommand::kMaxStopConds; ++i) {
    // ... same pattern as tickets 003/004 ...
}
CommandProcessor::replyOK(rbuf, sizeof(rbuf), "arc", body, corrId, replyFn, replyCtx);
ctx->superstructure->requestGoal(gr);
```

### Testing Plan

- Existing R arc tests must pass unchanged.
- Add/extend test: `R 300 500 stop=d:600` fires after ~600mm with `reason=dist`.
- Add test: `R 300 500` (no stop=) remains open-ended.
- Add test: `EVT done R` label in completion event.
- `uv run --with pytest python -m pytest tests/simulation -q` — 2 known failures.
- `python build.py --clean` exits 0.
- Run grep for `packKVArg` and `beginArc` to confirm no remaining callers
  before removing.

### Documentation Updates

- Remove the KV-encoding description from `handleR` and `handleVW` block comments.
- If `beginArc` is removed, update `MotionControllerBegin.cpp` file header
  comment (lists all functions defined there).
