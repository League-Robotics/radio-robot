---
id: '003'
title: Centralize keepalive/SAFE/ESTOP safety into Superstructure.evaluateSafety()
  and pre-cut goalAllowed() hook
status: in-progress
use-cases:
- SUC-002
- SUC-003
- SUC-005
depends-on:
- 042-001
- 042-002
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# 042-003: Centralize keepalive/SAFE/ESTOP safety into Superstructure.evaluateSafety() and pre-cut goalAllowed() hook

## Description

Move the three safety inline blocks from `loopTickOnce` into
`Superstructure::evaluateSafety(CommandProcessor& cmd, CommandQueue& queue, LoopTickState& ts, uint32_t now)`.
The bodies are moved **verbatim** — no logic changes, no reordering, no new conditions.

The three blocks (in order, from `LoopTickOnce.cpp`):
1. **Watchdog block** (lines 147–173): `needsWatchdog` logic, signed-delta math,
   `EVT safety_stop` emit, `cmd.setQueue(nullptr); cmd.process("X", ...); cmd.setQueue(&queue)` injection.
2. **Halt-controller block** (lines 175–192): `robot.haltController.evaluate(...)` call,
   HARD → X injection, SOFT → X soft injection (same queue-bypass pattern).

Drive advance (`robot.motionController.driveAdvance(...)`) is NOT moved — it stays in
`loopTickOnce` immediately after the `evaluateSafety` call.

Replace the three inline blocks in `loopTickOnce` with a single call:
```cpp
robot.superstructure.evaluateSafety(cmd, queue, ts, now);
```

The `goalAllowed()` stub is already present from T1 (returns `true`). No change needed
to `goalAllowed()` in this ticket.

After this ticket all sprint DoD criteria are met. The golden-TLM canary must remain
byte-exact, and all safety fences must pass green.

## Acceptance Criteria

- [x] `Superstructure::evaluateSafety(...)` exists in `Superstructure.{h,cpp}`.
      NOTE (annotation): the AC text shows the signature without `inputs`, but the
      Implementation Plan (lines 102-117) AND architecture-update.md require an
      added `const HardwareState& inputs` parameter — the halt block calls
      `haltController.evaluate(robot.state.inputs, ...)`. Followed the plan/arch:
      `evaluateSafety(CommandProcessor& cmd, CommandQueue& queue, LoopTickState& ts,
      const HardwareState& inputs, uint32_t now)`; call site passes `robot.state.inputs`.
- [x] The watchdog block body in `evaluateSafety` is textually identical to the
      pre-ticket inline block in `loopTickOnce` (verified by diff — only the
      documented re-source `robot.motionController` → `_mc`; logic byte-identical).
- [x] The halt-controller block body in `evaluateSafety` is textually identical to the
      pre-ticket inline block in `loopTickOnce` (verified by diff — only the
      documented re-source `robot.haltController` → `_hc` and
      `robot.state.inputs` → `inputs`; logic byte-identical).
- [x] The two blocks appear in `evaluateSafety` in the SAME ORDER as they appeared in
      `loopTickOnce`: watchdog first, then halt-controller.
- [x] `loopTickOnce` no longer contains the watchdog or halt-controller inline blocks;
      they are replaced by the single `robot.superstructure.evaluateSafety(cmd, queue, ts, robot.state.inputs, now)` call.
- [x] `robot.motionController.driveAdvance(...)` call remains in `loopTickOnce` immediately
      after the `evaluateSafety` call — NOT inside `evaluateSafety`.
- [x] `test_watchdog_exemption.py` passes (behavior-preservation fence). (5/5 PASSED)
- [x] `test_incident_scenarios.py` passes (behavior-preservation fence). (4/4 PASSED)
- [x] `test_goto_bounds.py` passes (behavior-preservation fence). (4/4 PASSED)
- [x] `test_033_005_wedge_hardening.py` passes (behavior-preservation fence). (3/3 PASSED)
- [x] Simulation tier green: `uv run --with pytest python -m pytest -q` → 2001 passed,
      0 errors.
- [x] Golden-TLM canary byte-exact. (`test_golden_tlm_unchanged` PASSED)
- [x] ARM firmware build: `python3 build.py --fw-only` → 0 errors (`MICROBIT.hex`); then
      `git checkout -- source/robot/DefaultConfig.cpp` (done).
- [x] Field-pin canary (`defaultRobotConfig()` diff) empty (only a cosmetic
      auto-gen source-path comment regen, discarded by the `git checkout` above).
- [x] Vendor-confinement grep gate passes. (`test_vendor_confinement_no_new_leaks` PASSED)
- [x] No new heap allocation or fibers introduced. (`evaluateSafety` uses only a
      stack `char wdBuf[64]`, identical to the pre-ticket inline block.)
- [x] No state-graph or transition-table introduced.
- [x] `source/superstructure/Superstructure.{h,cpp}` exists with `Goal` enum + `requestGoal` + `goalAllowed()` stub + `evaluateSafety()`.
- [x] All verb handlers route through `requestGoal` (queue-dispatch path). (T1; 16 `requestGoal` call sites)
- [x] `source/superstructure/MotionController.{h,cpp}` exists (from T2).

## Implementation Plan

### Approach

1. Add `evaluateSafety` declaration to `Superstructure.h`:
   ```cpp
   void evaluateSafety(CommandProcessor& cmd, CommandQueue& queue,
                       LoopTickState& ts, uint32_t now);
   ```
   Add any necessary forward declarations or includes (`CommandProcessor.h`,
   `CommandQueue.h`, `LoopTickState.h` / `LoopTickOnce.h`). `LoopTickState` is
   currently defined in `LoopTickOnce.h` — include it or forward-declare.
   Note: `Superstructure.h` is in `source/superstructure/`; include paths must
   reach `source/control/` headers.

2. In `Superstructure.cpp`, implement `evaluateSafety` by COPYING the two blocks
   verbatim from `loopTickOnce`. The blocks reference `robot.motionController` and
   `robot.config` — these must be accessed via the `_mc` reference and `_cfg`
   reference that `Superstructure` already holds. Specifically:
   - `mc.mode()` → `_mc.mode()`
   - `mc.hasActiveCommand()` → `_mc.hasActiveCommand()`
   - `mc.activeCmd().hasTimeStop()` → `_mc.activeCmd().hasTimeStop()`
   - `cfg.safetyEnabled` → `_cfg.safetyEnabled`
   - `cfg.sTimeoutMs` → `_cfg.sTimeoutMs`
   - `robot.haltController.evaluate(robot.state.inputs, ...)` →
     `_hc.evaluate(...)` — but `evaluateSafety` needs `HardwareState& inputs`.
     Add `HardwareState& inputs` as a parameter, OR store a `RobotStateContainer*`
     reference in `Superstructure`. Simpler: add `const HardwareState& inputs`
     as an additional parameter to `evaluateSafety`. Update the call site in
     `loopTickOnce` to pass `robot.state.inputs`.

   Revised signature (if inputs added as param):
   ```cpp
   void evaluateSafety(CommandProcessor& cmd, CommandQueue& queue,
                       LoopTickState& ts, const HardwareState& inputs,
                       uint32_t now);
   ```
   And the call in `loopTickOnce`:
   ```cpp
   robot.superstructure.evaluateSafety(cmd, queue, ts, robot.state.inputs, now);
   ```

3. In `LoopTickOnce.cpp`: delete the watchdog block and the halt-controller block.
   Replace with the single `robot.superstructure.evaluateSafety(...)` call. The
   `robot.motionController.driveAdvance(...)` call immediately below is unchanged.

4. Compile (`uv run --with pytest python -m pytest -q`) at this point to confirm
   green before running the canary.

5. Run golden-TLM canary immediately.

6. Run the four behavior-preservation fences:
   `test_watchdog_exemption.py`, `test_incident_scenarios.py`,
   `test_goto_bounds.py`, `test_033_005_wedge_hardening.py`.

7. ARM build gate: `python3 build.py --fw-only`; then
   `git checkout -- source/robot/DefaultConfig.cpp`.

### Careful: SAFE one-shot re-arm is NOT in the moved blocks

The SAFE one-shot re-arm (`_checkSafeOneShot()`) lives inside
`MotionController::beginX()` and is triggered when the next goal request calls
`requestGoal` → `beginX`. It is NOT in the watchdog or halt blocks. Do not add
a `_checkSafeOneShot` call to `evaluateSafety` — it would double-trigger.

### Files to Modify

- `source/superstructure/Superstructure.h` — add `evaluateSafety` declaration;
  add includes for `CommandProcessor.h`, `CommandQueue.h`, `LoopTickOnce.h` (or
  the header that defines `LoopTickState`), `RobotState.h` (for `HardwareState`)
- `source/superstructure/Superstructure.cpp` — implement `evaluateSafety` with
  verbatim block bodies; reference `_mc`, `_hc`, `_cfg` for robot members
- `source/control/LoopTickOnce.cpp` — delete watchdog + halt blocks; add
  `robot.superstructure.evaluateSafety(cmd, queue, ts, robot.state.inputs, now)`

### Testing Plan

Run after every sub-step:
- `uv run --with pytest python -m pytest -q` — full simulation tier.
- `test_golden_tlm.py` — byte-exact gate.
- `test_watchdog_exemption.py`, `test_incident_scenarios.py`, `test_goto_bounds.py`,
  `test_033_005_wedge_hardening.py` — safety behavior fences.
- `python3 build.py --fw-only` — ARM gate.
- Field-pin canary.
- `test_vendor_confinement.py` — grep gate.

### Documentation Updates

None beyond sprint artifacts.
