---
sprint: "053"
status: ready
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Use Cases — Sprint 053: Stop conditions Phase 2

## SUC-001: S command with stop= clause fires and reports reason

- **Actor**: Robot host program
- **Preconditions**: Robot is idle; Phase 1 (052) is merged; S is wired to a
  MotionCommand velocity goal with streamSeed=true.
- **Main Flow**:
  1. Host sends `S 300 300 stop=d:400`.
  2. Firmware converts (l, r) to body twist, builds a MotionCommand with
     streamSeed=true and a DISTANCE stop for 400 mm; seeds BVC immediately (no ramp).
  3. After ~400 mm the DISTANCE stop fires; `_firedKind = DISTANCE`.
  4. `emitEvt` sends `EVT done S reason=dist #<corrId>`.
  5. If host goes silent, watchdog fires `EVT safety_stop reason=watchdog`.
- **Postconditions**: Host receives a terminal EVT with reason token.
- **Acceptance Criteria**:
  - [ ] S with stop=d:N fires at ~N mm and emits `EVT done S reason=dist`.
  - [ ] S with stop=line:ge:T emits `EVT done S reason=line`.
  - [ ] S with no stop= clauses remains open-ended (watchdog only).

---

## SUC-002: T/D retain wire-label and stop semantics after round-trip removal

- **Actor**: Robot host program
- **Preconditions**: T and D handlers build GoalRequest directly; no packKVArg/argsHasKey
  for t=/dist= keys; no inverse() round-trip in handleVW.
- **Main Flow**:
  1. Host sends `T 300 300 1000`.
  2. handleT computes (v, ω) via forward kinematics, builds GoalRequest with
     Goal::VELOCITY, durationMs=1000 ms stop, doneLabel="EVT done T".
  3. Superstructure routes to beginVelocity-with-stops (no inverse() in handleVW).
  4. After 1000 ms TIME stop fires; host receives `EVT done T reason=time #<corrId>`.
- **Postconditions**: Host sees correct terminal event; D resets encoders before baseline.
- **Acceptance Criteria**:
  - [ ] `T l r ms` emits `EVT done T reason=time`.
  - [ ] `D l r mm` emits `EVT done D reason=dist`.
  - [ ] Additional stop= clauses on T or D (Phase 1) still fire correctly.
  - [ ] D encoder reset: baseline enc0 is 0 after distanceDrive call.

---

## SUC-003: VW keepalive guard preserved after Origin shrink

- **Actor**: Robot host keepalive loop
- **Preconditions**: Origin enum shrunk to RETARGETABLE/FIXED.
- **Main Flow** (keepalive):
  1. Host issues `VW 300 0`; MotionCommand starts with Origin::RETARGETABLE.
  2. Host issues `VW 300 0` again 90 ms later.
  3. handleVW sees RETARGETABLE origin; calls `setTarget(300, 0)`; re-arms watchdog.
- **Main Flow** (busy):
  1. Host issues `T 300 300 1000`; MotionCommand starts with Origin::FIXED.
  2. Host issues `VW 300 0` during timed drive.
  3. handleVW sees FIXED origin; replies `OK vw busy=FIXED`.
  4. T continues uninterrupted.
- **Postconditions**: Keepalive behavior unchanged; busy reply for non-retargetable.
- **Acceptance Criteria**:
  - [ ] VW keepalive updates a RETARGETABLE command without busy=.
  - [ ] VW during a FIXED command replies `OK vw busy=FIXED` (or equivalent).
  - [ ] After T/D/R/S completes, VW starts a new RETARGETABLE command.

---

## SUC-004: Wire labels (EVT done T/D/R/G/TURN/RT/S) preserved after Goal collapse

- **Actor**: Robot host awaiting completion events
- **Preconditions**: doneLabel field in GoalRequest carries the per-verb string;
  collapsed to one VELOCITY begin path.
- **Main Flow**:
  1. Host issues `R 300 500`; handleR builds GoalRequest with doneLabel="EVT done R".
  2. beginVelocity-with-stops calls `setDoneEvt("EVT done R")`.
  3. When a stop fires, `emitEvt` sends `EVT done R reason=<x> #<corrId>`.
- **Postconditions**: Host code matching on verb label continues to work.
- **Acceptance Criteria**:
  - [ ] S emits `EVT done S`.
  - [ ] T emits `EVT done T`.
  - [ ] D emits `EVT done D`.
  - [ ] R emits `EVT done R`.
  - [ ] G, TURN, RT labels unaffected (closed-loop path unchanged).

---

## SUC-005: Firmware clean build and canary re-baseline pass after structural refactor

- **Actor**: CI / firmware flash process
- **Preconditions**: Phase 2 refactor is complete on the sprint branch.
- **Main Flow**:
  1. Developer runs `python build.py --clean` in the firmware root.
  2. ARM cross-compiler produces `MICROBIT.hex` with no errors.
  3. Golden-TLM canary diff is reviewed; new baseline committed and documented.
  4. `uv run --with pytest python -m pytest tests/simulation -q` passes with
     exactly 2 pre-existing failures.
- **Postconditions**: Sprint branch is clean-build and test-clean; canary re-baselined.
- **Acceptance Criteria**:
  - [ ] `python build.py --clean` exits 0.
  - [ ] Sim tests pass with exactly 2 known failures.
  - [ ] Golden-TLM canary diff documented as expected (timing/ordering, not behavioral).
  - [ ] The stop-conditions issue is marked resolved.
