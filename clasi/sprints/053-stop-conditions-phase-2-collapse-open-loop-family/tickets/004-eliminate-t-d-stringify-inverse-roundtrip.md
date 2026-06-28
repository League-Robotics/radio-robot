---
id: 053-004
title: "Eliminate T/D stringify+inverse round-trip"
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

# 053-004: Eliminate T/D stringify+inverse round-trip

## Description

`handleT` and `handleD` currently:
1. Compute `(v, ω)` from `(l, r)` via `BodyKinematics::forward`.
2. Pack `v_int, omega_int` plus `t=<ms>` or `dist=<mm>` as KV strings into a
   VW ArgList.
3. Call `pushVW` to enqueue a fake VW ParsedCommand.
4. `handleVW` dequeues it, reads `t=` or `dist=`, then re-computes `(vL, vR)`
   via `inverse()` to pass to `beginTimed(vL, vR, ...)` / `distanceDrive(vL, vR, ...)`.

This round-trip:
- Introduces integer truncation (mrad/s encoding loses fractional omega).
- Requires `packKVArg`, `argsHasKey`, `argsScanKV` for the `t=`/`dist=` keys.
- Delays the begin call by one queue-drain hop.

After ticket 001, `GoalRequest` has `stops[]`, `nStops`, `doneLabel`. After
ticket 002, `Origin` has `FIXED`. This ticket rewrites `handleT` and `handleD`
to call `requestGoal` directly.

**Note**: `handleD` uses `Goal::DISTANCE` (not VELOCITY) to preserve the atomic
encoder reset via `robot->distanceDrive`. This is correct per the architecture
decision. The GoalRequest DISTANCE case in Superstructure applies `doneLabel`
and `stops[]` after the call (added in ticket 001).

After this ticket, `packKVArg`, `argsHasKey(args, "t")`, and
`argsHasKey(args, "dist")` in `handleVW` are removed. `argsScanKV` for `t`
and `dist` are removed from `handleVW`.

## Acceptance Criteria

- [ ] `handleT` in `MotionCommands.cpp`:
  - Computes `(v, ω)` via `BodyKinematics::forward(l, r, trackwidthMm, ...)`.
  - Builds `GoalRequest gr{}` with:
    - `goal = Goal::VELOCITY`.
    - `v_mms = v_mms, omega_rads = omega_rads`.
    - `stops[0] = makeTimeStop((float)ms)`, `nStops = 1`.
    - Any additional `stop=` from `args[3..]` packed into `gr.stops[1..]`.
    - `doneLabel = "EVT done T"`.
    - `streamSeed = false`.
  - Calls `ctx->superstructure->requestGoal(gr)`.
  - Does NOT call `pushVW` or `packKVArg`.
  - Calls `replyOK` before `requestGoal` (preserving D11: converter already
    replied; handleVW is no longer called for T).
  - Queue-null fallback: calls `_mc.beginTimed(...)` directly and applies
    stop= via `mc_applyStopClauses` (sim path; existing behavior preserved).
- [ ] `handleD` in `MotionCommands.cpp`:
  - Computes `(v, ω)` via `BodyKinematics::forward(l, r, trackwidthMm, ...)`.
  - Builds `GoalRequest gr{}` with:
    - `goal = Goal::DISTANCE`.
    - `leftMms = (int32_t)l`, `rightMms = (int32_t)r`, `targetMm = mm`.
      (Preserved: distanceDrive takes wheel-speed ints, not twist.)
    - `stops[0] = makeDistanceStop((float)mm)`, `nStops = 1`.
    - Any additional `stop=` from `args[3..]` packed into `gr.stops[1..]`.
    - `doneLabel = "EVT done D"`.
  - Calls `ctx->superstructure->requestGoal(gr)`.
  - Does NOT call `pushVW` or `packKVArg`.
  - Calls `replyOK` before `requestGoal`.
  - Queue-null fallback: calls `ctx->robot->distanceDrive(...)` directly and
    applies stop= via `mc_applyStopClauses`.
- [ ] `handleVW` in `MotionCommands.cpp`:
  - The `if (argsHasKey(args, "t"))` block is removed.
  - The `if (argsHasKey(args, "dist"))` block is removed.
  - `argsScanKV(args, "t", ...)` and `argsScanKV(args, "dist", ...)` calls
    removed.
- [ ] `packKVArg` static helper is removed (if no other callers remain after
  this ticket and ticket 005). Check for remaining uses before removing.
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` passes with
  exactly 2 known failures. Existing T/D tests continue to pass.
- [ ] `python build.py --clean` exits 0.
- [ ] Encoder reset for D: `distanceDrive` still performs the atomic encoder
  reset. The programmer must verify that `MotionCommand::start()` captures a
  zero enc0 baseline after the reset (same as pre-sprint behavior).

## Implementation Plan

### Approach

Rewrite `handleT` and `handleD` to call `requestGoal` directly. Remove the
KV demux blocks for `t=` and `dist=` in `handleVW`. Note the D11 reply order:
for T and D, `replyOK` is called BEFORE `requestGoal` (the converter has
already replied; the queue-drain hop is eliminated).

### Files to Modify

- `source/commands/MotionCommands.cpp`
  - `handleT`: rewrite queue path to call `requestGoal` directly; remove
    `packKVArg(vwArgs, 2, "t", ms)` and `pushVW`. Call `replyOK` before
    `requestGoal`. Populate `gr.stops[]` from the time stop plus any extra
    stop= from `args[3..]` using the helper pattern established in ticket 003.
  - `handleD`: rewrite queue path to call `requestGoal` directly; remove
    `packKVArg(vwArgs, 2, "dist", mm)` and `pushVW`. Call `replyOK` before
    `requestGoal`.
  - `handleVW`: remove the `argsHasKey(args, "t")` and `argsHasKey(args, "dist")`
    check blocks entirely.
  - If `packKVArg` has no remaining callers (check R handler — ticket 005
    removes it from R), remove the `packKVArg` static helper.
  - If `argsScanKV` has no remaining callers for `t`/`dist` keys (other
    remaining users are `h`, `eps`, `speed`, `radius`, `rot` — all in the
    TURN/RT/G/R path), keep `argsScanKV` for now; remove per-key calls.

### GoalRequest stops[] population for T

```cpp
// handleT after computing v_mms, omega_rads:
GoalRequest gr{};
gr.goal       = Goal::VELOCITY;
gr.robot      = ctx->robot;
gr.now_ms     = now;
gr.replyFn    = replyFn;
gr.replyCtx   = replyCtx;
gr.corrId     = corrId;
gr.v_mms      = v_mms;
gr.omega_rads = omega_rads;
gr.doneLabel  = "EVT done T";
gr.streamSeed = false;
gr.stops[gr.nStops++] = makeTimeStop((float)ms);
// Additional stop= from args[3..]:
for (int i = 3; i < args.count && gr.nStops < MotionCommand::kMaxStopConds; ++i) {
    if (args.args[i].type != ArgType::STR) continue;
    const char* s = args.args[i].sval;
    StopCondition cond;
    if (strncmp(s, "stop=", 5) == 0 && mc_parseStopTokenInto(s + 5, cond))
        gr.stops[gr.nStops++] = cond;
    else if (strncmp(s, "sensor=", 7) == 0 && mc_parseSensorTokenInto(s + 7, cond))
        gr.stops[gr.nStops++] = cond;
}
// Reply before requestGoal (D11: converter already replied).
CommandProcessor::replyOK(rbuf, sizeof(rbuf), "drive", body, corrId, replyFn, replyCtx);
ctx->superstructure->requestGoal(gr);
```

### Testing Plan

- Existing T/D tests in `tests/simulation/unit/` and
  `tests/simulation/system/` must pass unchanged.
- Add or extend test to assert:
  - `T 300 300 1000` emits `EVT done T reason=time`.
  - `D 300 300 400` emits `EVT done D reason=dist`.
  - `T 300 300 5000 stop=sensor:line0:ge:512` fires on line before timeout.
  - D encoder baseline is 0 at start (distanceDrive reset preserved).
- `uv run --with pytest python -m pytest tests/simulation -q` — 2 known failures only.
- `python build.py --clean` exits 0.

### Documentation Updates

- Remove doc comment referencing the `t=`/`dist=` KV packing from `handleVW`
  block comment.
- Update `handleT`/`handleD` function comments to describe the direct
  `requestGoal` path.
