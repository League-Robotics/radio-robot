---
status: approved
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 042 Use Cases

## SUC-001: Verb handler issues a goal via the single guarded entry point

- **Actor**: Host application issuing a motion verb (S, T, D, G, R, TURN, RT, VW, X, ESTOP)
- **Preconditions**: Robot is running the cooperative loop; MotionController is operational.
- **Main Flow**:
  1. Host sends a motion verb over the wire.
  2. `MotionCommandHandlers` routes the parsed command to `Superstructure::requestGoal(GoalRequest)`.
  3. `requestGoal` validates the goal via `goalAllowed()` (stub — returns true).
  4. `requestGoal` dispatches to the appropriate `MotionController::beginX()` method.
  5. Motion executes identically to the pre-Sprint-042 path.
- **Postconditions**: Motion begins; behavior is byte-identical to the pre-refactor path.
- **Acceptance Criteria**:
  - [ ] All motion verb handlers call `requestGoal` rather than `motionController.beginX()` directly.
  - [ ] `goalAllowed()` stub returns `true` unconditionally.
  - [ ] Golden-TLM byte-exact canary passes.
  - [ ] All simulation tier tests pass (≥ 2001).

---

## SUC-002: Safety logic (keepalive, SAFE one-shot, ESTOP/X) fires in correct order

- **Actor**: Cooperative loop tick; host keepalive watchdog; SAFE off command; X/ESTOP command.
- **Preconditions**: `loopTickOnce` calls `Superstructure::periodic()` (or equivalent centralized entry) in the same order as the pre-refactor inline blocks.
- **Main Flow**:
  1. `loopTickOnce` calls into `Superstructure` once per tick for the safety-evaluation block.
  2. Superstructure evaluates the keepalive watchdog (same signed-delta math, same ordering).
  3. Superstructure evaluates `HaltController::evaluate()` (same call site).
  4. SAFE one-shot re-arm happens inside `MotionController::beginX()` (no change to trigger point).
  5. ESTOP/X path (injected `X` command) fires exactly as before.
- **Postconditions**: Safety behavior is byte-identical; no reordering of watchdog → halt → drive.
- **Acceptance Criteria**:
  - [ ] `test_watchdog_exemption.py`, `test_incident_scenarios.py`, `test_goto_bounds.py`, `test_033_005_wedge_hardening.py` all pass unmodified.
  - [ ] `loopTickOnce` calls the centralized safety in the same order as before.
  - [ ] Simulation tier green (≥ 2001).

---

## SUC-003: World-bounds hook pre-cut for future off-table fence

- **Actor**: Future sprint implementing off-table bounds checking.
- **Preconditions**: `Superstructure::requestGoal()` exists.
- **Main Flow**:
  1. Future sprint provides an implementation of `goalAllowed(GoalRequest)`.
  2. `requestGoal` calls `goalAllowed` before dispatching any goal.
  3. Denied goals are rejected at the single entry point — no other call sites exist.
- **Postconditions**: Off-table fence can be added in one place without touching verb handlers.
- **Acceptance Criteria**:
  - [ ] `goalAllowed()` stub exists in `Superstructure` and returns `true` for all goals.
  - [ ] No off-table logic is added in this sprint.

---

## SUC-004: MotionController accessible under source/superstructure/ with identical behavior

- **Actor**: Build system; programmer referencing MotionController.
- **Preconditions**: Phase C (Sprint 041) is complete; `source/superstructure/` directory does not yet exist.
- **Main Flow**:
  1. `source/control/MotionController.{h,cpp}` is moved to `source/superstructure/MotionController.{h,cpp}` using `git mv`.
  2. Include paths in all callers and CMakeLists are updated.
  3. Alias shim at `source/control/MotionController.h` provides backward compatibility until cleanup.
  4. ARM firmware build (`python3 build.py --fw-only`) succeeds with zero errors.
- **Postconditions**: `MotionController` code is identical to pre-move; no functional change.
- **Acceptance Criteria**:
  - [ ] `source/superstructure/MotionController.{h,cpp}` exists; `source/control/MotionController.{h,cpp}` is a shim.
  - [ ] ARM build passes (`python3 build.py --fw-only` → 0 errors).
  - [ ] Simulation tier green (≥ 2001).
  - [ ] Vendor-confinement grep gate passes with `source/superstructure/` in INSPECT_DIRS.

---

## SUC-005: loopTickOnce simplified — one centralized call replaces scattered safety blocks

- **Actor**: Firmware developer reading or modifying `loopTickOnce`.
- **Preconditions**: Safety logic (watchdog, halt, ESTOP) is currently scattered inline in `loopTickOnce`.
- **Main Flow**:
  1. The watchdog block, halt-condition block, and ESTOP injection move (verbatim bodies) into `Superstructure`.
  2. `loopTickOnce` calls `superstructure.evaluateSafety(cmd, queue, ts, now)` (or equivalent) in the same position.
  3. Internal ordering within the call is identical to the previous inline ordering.
- **Postconditions**: `loopTickOnce` is shorter; safety logic has one canonical home; behavior is byte-identical.
- **Acceptance Criteria**:
  - [ ] Watchdog, halt-controller, and ESTOP/X injection are no longer inline in `loopTickOnce`.
  - [ ] The call to the centralized method occupies the same position in `loopTickOnce` as the three former blocks.
  - [ ] All behavior-preservation fences green.
