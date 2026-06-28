---
id: 053-003
title: "Migrate S onto MotionCommand (streamSeed path)"
status: open
use-cases:
- SUC-001
- SUC-004
depends-on:
- 053-001
- 053-002
issue: stop-conditions-as-a-first-class-system-primitive.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 053-003: Migrate S onto MotionCommand (streamSeed path)

## Description

The S command (DriveMode::STREAMING) currently calls `beginStream` which seeds
the BVC and sets `_mode = DriveMode::STREAMING` without creating a MotionCommand.
This means stop= clauses attached to S (Phase 1) parse correctly but can never
fire — `mc_applyStopClauses` calls `mc.activeCmd().addStop(...)` but no active
command exists.

This ticket migrates S onto the MotionCommand velocity path by:
1. Reworking `beginVelocity` to accept a `seedImmediate` flag.
2. Updating `handleS` to call `requestGoal` with `Goal::VELOCITY`,
   `streamSeed=true`, `doneLabel="EVT done S"`, and any stop= clauses packed
   into `gr.stops[]`.
3. Removing the `stream=1` KV packing in `handleS` and the `argsHasKey(args,
   "stream")` branch in `handleVW`.

After this ticket, S stop= clauses fire and report reason=. `pushVW` is no
longer called by handleS.

**CRITICAL**: Audit `MotionController::driveAdvance` (in
`source/superstructure/MotionController.cpp`) for any `mode == DriveMode::STREAMING`
branches that formerly guarded S-specific behavior. After this change, S sets
`_mode = DriveMode::VELOCITY` (not STREAMING); any guard on STREAMING for S
must be removed or restructured. Document findings in the PR. `DriveMode::STREAMING`
is still set by `beginRawVelocity` (_VW command) — do not affect that path.

## Acceptance Criteria

- [ ] `handleS` in `MotionCommands.cpp`:
  - Computes (v, ω) via `BodyKinematics::forward` (already does this).
  - Packs any `stop=` / `sensor=` clauses from `args[2..]` into
    `gr.stops[0..nStops-1]` (iterating and calling `mc_parseStopToken` inline
    or via a helper; not using `mc_applyStopClauses` directly since we have no
    active command yet).
  - Builds `GoalRequest gr{}` with `goal=Goal::VELOCITY`, `streamSeed=true`,
    `doneLabel="EVT done S"`, the computed twist, and the packed stops.
  - Calls `ctx->superstructure->requestGoal(gr)`.
  - Does NOT call `pushVW` or `packKVArg`.
  - Replies OK in the same place as before (no D11 change for S; S replies
    after requestGoal, not before).
- [ ] `beginVelocity` in `MotionControllerBegin.cpp` has a `bool seedImmediate`
  parameter (default `false`). When `true`:
  - Calls `_bvc.seedCurrent(v_mms, omega_rads)` before `_bvc.setTarget(...)`.
  - Sets `_mode = DriveMode::VELOCITY` (not STREAMING).
- [ ] `Superstructure::requestGoal` VELOCITY case: when `gr.streamSeed == true`,
  calls `_mc.beginVelocity(..., seedImmediate=true)`.
- [ ] `handleVW` in `MotionCommands.cpp`: the `argsHasKey(args, "stream")`
  branch is removed.
- [ ] `parseS` and `mc_packStopKVs` helper: `parseS` continues to pack stop=
  KVs into trailing STR args for the queue path (the stop-packing is needed for
  the direct GoalRequest stop[] population in handleS).
- [ ] `driveAdvance` audit: any `mode == DriveMode::STREAMING` branch that
  applied S-specific keepalive or watchdog logic is updated. `DriveMode::STREAMING`
  is only the mode of `_VW` / `beginRawVelocity` after this change.
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` passes with
  exactly 2 known failures; at least one new test covers:
  - `S 300 300 stop=d:400` fires `reason=dist`.
  - `S 300 300` with no stop= remains open-ended.
  - `EVT done S` label emitted on completion.
- [ ] `python build.py --clean` exits 0.

## Implementation Plan

### Approach

Two-part change: extend `beginVelocity` signature (MotionControllerBegin.cpp),
then rewrite `handleS` to call `requestGoal` directly. Remove the stream=1
KV branch from handleVW. Audit driveAdvance.

### Files to Modify

- `source/control/MotionControllerBegin.cpp`
  - Add `bool seedImmediate = false` to `beginVelocity` signature.
  - When `seedImmediate`: insert `_bvc.seedCurrent(v_mms, omega_rads);` before
    `_bvc.setTarget(v_mms, omega_rads);`.
  - Set `_mode = DriveMode::VELOCITY` in both paths (no STREAMING).

- `source/superstructure/MotionController.h`
  - Update `beginVelocity` declaration to include `bool seedImmediate = false`.

- `source/superstructure/Superstructure.cpp`
  - VELOCITY case: add `bool seed = gr.streamSeed;` and pass to
    `_mc.beginVelocity(..., seed)`.
  - STREAM case: can be removed (or left as a thin redirect to VELOCITY with
    seedImmediate=true for safety, then deprecated). Preferred: remove the STREAM
    case; any residual call sites that still use `Goal::STREAM` will fail to
    compile and must be updated.
  - `Goal::STREAM` can be removed from the `Goal` enum now that no handler uses it.

- `source/superstructure/Superstructure.h`
  - Remove `Goal::STREAM` from the enum (or leave as deprecated alias — remove
    preferred to avoid confusion).

- `source/commands/MotionCommands.cpp`
  - `handleS`: replace the `pushVW` path with a direct `requestGoal`. Build
    `gr.stops[]` by iterating `args[2..]` and calling `mc_parseStopToken` to
    populate `gr.stops[gr.nStops++]`. Cap at `kMaxStopConds`. Keep the
    queue-null fallback path calling `beginVelocity` directly (for sim paths
    that do not wire the queue). Calls `replyOK` after `requestGoal` (same as
    before — S replies in the handler, not deferred).
  - `handleVW`: remove the `if (argsHasKey(args, "stream"))` block (the entire
    branch including the `beginStream` call via requestGoal STREAM).

- `source/superstructure/MotionController.cpp` (driveAdvance)
  - Audit all `mode == DriveMode::STREAMING` and `_mode == DriveMode::STREAMING`
    checks. Any check that was meant to guard S-command behavior (e.g., a
    no-keepalive path for streaming) must be updated: after this ticket, S is
    in VELOCITY mode and has a MotionCommand. Any STREAMING check that remains
    should guard only _VW (beginRawVelocity). Document what was found in the
    commit message.

### Helper to pack stops[] from args

In `handleS`, to populate `gr.stops[]` without reusing `mc_applyStopClauses`
(which needs a MotionCommand), extract a small helper or inline:

```cpp
for (int i = startIdx; i < args.count && gr.nStops < MotionCommand::kMaxStopConds; ++i) {
    if (args.args[i].type != ArgType::STR) continue;
    const char* s = args.args[i].sval;
    StopCondition cond;
    bool ok = false;
    if (strncmp(s, "stop=", 5) == 0)
        ok = mc_parseStopTokenInto(s + 5, cond);
    else if (strncmp(s, "sensor=", 7) == 0)
        ok = mc_parseSensorTokenInto(s + 7, cond);
    if (ok) gr.stops[gr.nStops++] = cond;
}
```

This requires `mc_parseStopToken` and `mc_parseSensorToken` to have a variant
that returns a `StopCondition` by value instead of calling `mc.addStop()`.
The programmer may refactor `mc_parseStopToken` to take `StopCondition& out`
and return bool, then `mc_applyStopClauses` calls this variant. Both handlers
and GoalRequest population use the same underlying parser.

### Testing Plan

- Add `tests/simulation/unit/test_053_s_stop_condition.py` (or extend
  `test_052_stop_parser.py`) with tests:
  - `S 300 300 stop=d:400` → fires after ~400mm with `reason=dist`.
  - `S 300 300 stop=t:500` → fires after ~500ms with `reason=time`.
  - `S 300 300` → no stop fires (open-ended, watchdog only).
  - `EVT done S` label in emitted event.
- Run `uv run --with pytest python -m pytest tests/simulation -q`.
- `python build.py --clean` exits 0.

### Documentation Updates

- Update the comment in `handleVW` near the former stream= branch to note
  removal.
- Add a doc comment to the `seedImmediate` parameter in `beginVelocity`.
