---
id: '002'
title: Move Nezha split-phase state machine into Motor impl; add positionMm/velocityMmps
  accessors
status: done
use-cases:
- SUC-039-003
depends-on:
- 039-001
github-issue: ''
issue: ''
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# T2 — Move Nezha split-phase state machine into Motor impl; add positionMm/velocityMmps accessors

## Description

Move the Nezha split-phase encoder request/collect state machine (currently in
`Robot::controlCollectSplitPhase`) into the `Motor` impl, driven by
`IVelocityMotor::tick(now_ms)` (which was scaffolded as a no-op in T1). Add
`positionMm()` and `velocityMmps()` cheap accessors to `IVelocityMotor` — they return
the last-collected values without performing I2C. Move the outlier filter and wedge-
detector logic into `Motor::tick`. Remove `Robot::controlCollectSplitPhase`.

The `MockMotor` gains a trivial `tick(now_ms)` that promotes the existing `_encoderMm`
(set by `MockHAL::tick(now_ms, cmds)`) into `_lastPositionMm` so `positionMm()` returns
it. No double-integration (see Open Question 2 in the architecture update).

`MotorController::controlTick` is updated to read `motor.positionMm()` and
`motor.velocityMmps()` instead of calling `motor.readEncoderMmFSettle(cfg)`.

**This is the highest-risk ticket.** The golden-TLM canary is the behavioral oracle:
it must pass byte-exact after this change. Move ALL bodies VERBATIM — no numeric
changes, no algorithm improvements, no restructuring beyond the mechanical move.

**Host-verifiable:** Yes — via the simulation tier and the golden-TLM canary.
**ARM files touched:** `Motor.cpp` (split-phase I2C calls), `NezhaHAL.cpp` (tick wiring),
`LoopScheduler.cpp` (call site), `WedgeTest.cpp` (call site). All must be verified
textually if the ARM toolchain is absent.

## Approach

### Step 1 — Add state to Motor

In `source/hal/Motor.h`, add private state:
```cpp
// Split-phase state machine
enum class SplitPhase { IDLE, REQUESTED };
SplitPhase _splitPhase;
uint32_t   _lastTickMs;
float      _lastPositionMm;   // set by collect(); read by positionMm()
float      _lastVelocityMmps; // differentiated from _lastPositionMm

// Outlier filter state (moved from Robot::controlCollectSplitPhase)
float   _filterPrevMm;
bool    _filterPrevValid;

// Wedge detector state (moved from MotorController)
// NOTE: The outlier filter and wedge detector are currently in Robot and
// MotorController respectively. They move here so Motor::tick() is self-contained.
// MotorController::wheelWedgedL/R() will delegate to Motor::wedgeActive().
uint8_t _stuckCount;
bool    _wedgeEmitted;
bool    _hasMoved;
float   _wedgePrevMm;
bool    _wedgePrevValid;
uint8_t _filterRejectStreak;
```

### Step 2 — Implement Motor::tick(now_ms)

`Motor::tick(now_ms)` is called by `NezhaHAL::tick(now_ms)`. It replaces the
request/collect sequencing in `Robot::controlCollectSplitPhase`:

```
Motor::tick(now_ms):
  if _splitPhase == IDLE:
    requestEncoder()          // 0x46 write, returns immediately
    _splitPhase = REQUESTED
    return
  if _splitPhase == REQUESTED:
    raw = collectEncoder()    // 4-byte read
    apply outlier filter → _lastPositionMm
    differentiate → _lastVelocityMmps
    advance wedge detector
    _splitPhase = IDLE
    _lastTickMs = now_ms
```

**Critical invariant:** The cooperative loop calls `hal.tick(now_ms)` once per
iteration BEFORE `loopTickOnce`. The loop period (≥ 10 ms idle) satisfies the
vendor's post-write settle requirement between the REQUESTED tick and the next
IDLE→REQUESTED transition. This is the same guarantee the old
`controlCollectSplitPhase` relied on (it did both reads in the same call, each with
a 4 ms settle). Since the loop runs at ≥ 24 ms/iteration, two consecutive tick()
calls are always separated by the full loop period.

**Outlier filter:** Copy the `kMaxDeltaMm` formula and retry loop from
`Robot::controlCollectSplitPhase` VERBATIM. The filter needs `tgtLMms`/`tgtRMms`
to compute `kMaxDeltaMm`; pass the commanded speed to `tick()` or have `Motor`
store the last commanded speed (set via `setOutput`). The commanded pct is already
stored as `_lastWrittenPct`; derive the mm/s target from it and `cfg` (the Motor
has access to `_cfg` — see T4 note below). Or simpler: `Motor` stores the last
`tgtMms` via a new `setTargetMmps(float)` method called by `MotorController::setTarget`.

**Wedge detector:** The wedge detector state (`_stuckCount`, `_wedgeEmitted`,
`_hasMoved`, `_wedgePrevMm`) and the EVT emission logic move from `MotorController`
into `Motor`. `Motor::wedgeActive()` exposes the latch state. `MotorController::wheelWedgedL/R()` delegates to `motorL.wedgeActive()` / `motorR.wedgeActive()`.

The EVT emission in `MotorController` currently fires `_evtFn` with `_busDiag`
stats. After the move, `Motor::tick()` fires the EVT — so it needs the `_evtFn` /
`_evtCtx` pointers (passed in via `Motor::setEvtSink()` analogous to the current
`MotorController::setEvtSink()`).

**Alternative simpler approach (recommended for safety):** Keep the wedge DETECTOR
in `MotorController::controlTick` but feed it from `motor.positionMm()`. Only move
the request/collect + outlier filter into `Motor::tick`. This minimizes the scope of
the move and the risk of golden-TLM divergence. The wedge detector stays where it is;
`MotorController::controlTick` reads `positionMm()` instead of
`readEncoderMmFSettle(cfg)`.

**Programmer decision:** Choose whichever approach leaves the golden-TLM canary
green. The wedge detector is NOT in the TLM frame; the outlier filter affects
encoder values which ARE in the TLM frame.

### Step 3 — Update IVelocityMotor (already scaffolded in T1)

`tick(now_ms)` is already on `IVelocityMotor` as a virtual no-op.
Add `positionMm()` and `velocityMmps()` as pure-virtuals (they were default-impl'd
in T1 as returning 0.0; make them pure-virtual now):
```cpp
virtual float positionMm() const = 0;
virtual float velocityMmps() const = 0;
```

### Step 4 — Update MockMotor

`MockMotor::tick(now_ms)`: promote `_encoderMm` into `_lastPositionMm`.
`MockMotor::positionMm()`: return `_lastPositionMm`.
`MockMotor::velocityMmps()`: return the last computed velocity (already stored in the
mock as the velocity fed back to `controlTick` via `inputs.velLMms`/`velRMms`).

The existing `MockMotor` integration path via `MockHAL::tick(now_ms, cmds)` remains
unchanged. `MockHAL::tick(now_ms)` (no cmds) additionally calls
`_motorL.tick(now_ms)` and `_motorR.tick(now_ms)`.

### Step 5 — Update MotorController::controlTick

Replace:
```cpp
float newR = motorR.readEncoderMmFSettle(config);
```
with:
```cpp
float newR = motorR.positionMm();
```
and similarly for left. The velocity differentiation that `controlTick` was doing
(computing `velLMms`/`velRMms` from position deltas) is now done in `Motor::tick` —
so `controlTick` reads `motor.velocityMmps()` directly instead of computing it.

However, `controlTick` owns the PID dt calculation and the per-wheel ZOH. Keep
those intact. The ZOH pattern becomes: if `motor.positionMm()` changed since the
last `controlTick`, it is the "refreshed" wheel. Detect by comparing to the
previously stored `_prevEncL/R`.

### Step 6 — Remove Robot::controlCollectSplitPhase

- Remove declaration from `Robot.h`.
- Remove implementation from `Robot.cpp` (the entire function body, lines 91–224).
- Remove the private state members `_filterRejectStreakL`, `_filterRejectStreakR`,
  `_prevDriving`, `_lastControlMs`, `kFilterRejectStreakThreshold` that were
  `controlCollectSplitPhase`-only.

### Step 7 — Update call sites

**`LoopScheduler::run_blocks()` (source/control/LoopScheduler.cpp, line 225):**
Replace:
```cpp
_robot.controlCollectSplitPhase(now, 0);
```
with nothing additional — `hal.tick(now_ms)` is already called implicitly via
`loopTickOnce`'s `hal.tick(now, commands)` call... but wait: check whether
`LoopScheduler` calls `hal.tick` before `controlCollectSplitPhase`. Looking at the
architecture doc §4.1: step 6 of `loopTickOnce` is `hal.tick(now, commands)` (the
actuator tick). The encoder-read tick is NOT inside `loopTickOnce`; it is called by
`LoopScheduler` BEFORE `loopTickOnce`. So:

```cpp
// In LoopScheduler::run_blocks(), replace:
if (enControl) {
    _robot.controlCollectSplitPhase(now, 0);
}
// with:
if (enControl) {
    _robot.hal.tick(now);  // drives Motor::tick() for encoder read
}
```

Note: `Robot::hal` is a public member (`Hardware& hal`), so this is accessible.
Alternatively, add a `Robot::tickHardware(uint32_t now)` forwarding method.

**`sim_api.cpp` (two sites, lines 188-189 and 508-509):**
Replace `s->robot.controlCollectSplitPhase(now_ms, 0)` with
`s->hal.tick(now_ms)` at both sites. The preceding `s->hal.tick(now_ms, s->robot.state.commands)` (actuator tick) stays.

Note: After the change `sim_api.cpp` has two `hal.tick` calls before `loopTickOnce`:
1. `s->hal.tick(now_ms, cmds)` — actuator tick (integrates MockMotor position).
2. `s->hal.tick(now_ms)` — sensor tick (MockMotor::tick promotes _encoderMm → positionMm).
The second call must not re-integrate; confirm `MockMotor::tick` is a no-op copy only.

**`WedgeTest.cpp` (source/app/WedgeTest.cpp, line 227) — ARM only:**
Replace `robot->controlCollectSplitPhase(now, 0)` with the equivalent hardware tick
call. Since `WedgeTest` has a `Robot*`, use `robot->hal.tick(now)`. Verify textually.

## Files to Modify

- `source/io/capability/IVelocityMotor.h` — make `positionMm()` and `velocityMmps()` pure-virtual
- `source/hal/Motor.h` — add split-phase state, outlier-filter state, `tick()`, `positionMm()`, `velocityMmps()`, `wedgeActive()`
- `source/hal/Motor.cpp` — implement `tick()` (request/collect, outlier filter); implement accessors
- `source/hal/mock/MockMotor.h` — add `tick()`, `positionMm()`, `velocityMmps()`
- `source/hal/mock/MockMotor.cpp` — implement trivially
- `source/hal/mock/MockHAL.h/.cpp` — `tick(uint32_t)` (no cmds) calls motor ticks
- `source/hal/NezhaHAL.h/.cpp` — `tick(uint32_t)` calls `_motorL.tick()` + `_motorR.tick()`
- `source/control/MotorController.h/.cpp` — `controlTick` reads `positionMm()`/`velocityMmps()`; `wheelWedgedL/R()` may delegate to motor or stay in MotorController
- `source/robot/Robot.h` — remove `controlCollectSplitPhase` declaration; remove streak-filter private state
- `source/robot/Robot.cpp` — remove `controlCollectSplitPhase` implementation
- `source/control/LoopScheduler.cpp` — replace `controlCollectSplitPhase` call with `robot.hal.tick(now)`
- `tests/_infra/sim/sim_api.cpp` — replace two `controlCollectSplitPhase` calls with `hal.tick(now_ms)` (no cmds)
- `source/app/WedgeTest.cpp` (ARM only) — replace `controlCollectSplitPhase` call

## Acceptance Criteria

- [x] `Robot::controlCollectSplitPhase` does not exist in `Robot.h` or `Robot.cpp`.
- [x] `Motor::tick(uint32_t now_ms)` implements the request/collect cycle. (Per OQ-2
  resolution (b), the speed-scaled outlier filter is kept in the control layer —
  relocated verbatim into `loopTickOnce()`'s CONTROL COLLECT block, fed from
  `positionMm()` — NOT moved into `Motor::tick`. This is the lower-risk path that
  keeps the golden-TLM byte-exact AND the I2C-wire bytes unchanged.)
- [x] `Motor::positionMm()` and `Motor::velocityMmps()` return last-collected values.
- [x] `MockMotor::tick(now_ms)` promotes `_encoderMm` → `_lastPositionMm` (no re-integration).
  Existing integration renamed to `MockMotor::integrate(dt_ms)`, still the sole
  integration site (driven by `MockHAL::tick(now,cmds)`).
- [x] `LoopScheduler::run_blocks()` calls `robot.hal.tick(now)` (not `controlCollectSplitPhase`).
- [x] `sim_api.cpp` has no reference to `controlCollectSplitPhase` (both sites → `hal.tick(now)`).
- [x] `WedgeTest.cpp` has no reference to `controlCollectSplitPhase` (ARM verify — `hal.tick(now)`; firmware build clean).
- [x] Golden-TLM canary passes byte-exact (`pytest tests/simulation/unit/test_golden_tlm.py` — 1 passed).
- [x] Simulation tier green: `uv run --with pytest python -m pytest -q` — 1957 passed, 0 errors.
- [x] `defaultRobotConfig()` field-pin unchanged (134 golden/field-pin/default-config tests pass; DefaultConfig.cpp reverted post-build).
- [x] No new heap allocation or fiber introduced (all new state is value members; zero-heap, single-threaded preserved).

## Testing Plan

- Run `uv run --with pytest python -m pytest tests/simulation/unit/test_golden_tlm.py -v` first — this is the behavioral oracle.
- If it fails: the bodies were NOT moved verbatim; diff `Robot::controlCollectSplitPhase` line-by-line against `Motor::tick`. Do not proceed with other tests until golden-TLM is green.
- Run the full simulation tier: `uv run --with pytest python -m pytest -q`.
- Run `uv run --with pytest python -m pytest tests/simulation/unit/test_vendor_confinement.py -v`.
- If ARM toolchain present: `python3 build.py --clean`.
- **ARM-only files changed:** `Motor.cpp` (I2C call timing), `NezhaHAL.cpp` (tick wiring), `LoopScheduler.cpp`, `WedgeTest.cpp` — verify each textually.
