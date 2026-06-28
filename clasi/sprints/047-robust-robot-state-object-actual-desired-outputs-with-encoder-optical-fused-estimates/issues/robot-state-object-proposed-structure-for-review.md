---
status: in-progress
sprint: '047'
tickets:
- 047-001
- 047-002
- 047-003
- 047-004
- 047-005
---

# Robot State Object — Proposed Structure (for review)

## Context

Today the robot's "state" is one flat blob, `HardwareState` in
[source/types/Inputs.h](source/types/Inputs.h), plus a separate `MotorCommands`
and `TargetState`, all bundled into `RobotStateContainer`. The problem:

- `HardwareState.poseX/poseY/poseHrad` and `fusedV/fusedOmega` are **misleadingly
  named** — they are the *EKF-fused* outputs, not raw readings.
- There is **no separately-retained encoder-only pose**. `Odometry::predict()`
  ([source/control/Odometry.cpp](source/control/Odometry.cpp)) arc-integrates the
  wheel deltas but feeds them straight into the EKF and overwrites the shared pose
  with the fused result. The pre-fusion dead-reckoned pose is discarded.
- OTOS readings live as loose `otosX/Y/H` scalars; OTOS *velocity* is read but not
  persisted as its own estimate.
- The "desired" side is scattered across `MotorCommands` (target wheel speeds, PWM,
  port outputs), `TargetState` (mode, world target, deadline), and
  `BodyVelocityController`'s private profiler members.

**Goal:** a state object representing the *totality* of robot state, cleanly split
into **actual** ("where we are") and **desired** ("where we want to get to"), where
the actual side exposes the **encoder**, **optical-flow (OTOS)**, and **fused**
beliefs *side by side* — so we can dump all three and validate whether fusion is
working. The encoder belief is integrated over time from wheel deltas; camera/absolute
fixes reset it.

This proposal defines the structure only. It is intended for your review before any
code is written. It is compatible with the in-flight cmon-pid / TinyEKF consolidation
([clasi/issues/consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md](clasi/issues/consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md)).

Embedded constraints respected throughout: POD aggregates, `= {}` init, no heap, no
STL in the structs, float-only, units stay mm / mm·s⁻¹ / rad / rad·s⁻¹, and the
`#ifdef ROBOT_DRIVETRAIN_MECANUM` fork is confined to wheel-count array sizing.

---

## 1. Building block — `PoseEstimate`

One source's belief about where the robot is and how fast it's moving. Reused
identically for the encoder-only, OTOS-only, and fused estimates.

```cpp
// source/state/PoseEstimate.h  (new)
struct PoseEstimate {
    Pose2D     pose  = {0,0,0};   // x mm, y mm, h rad   (world/field frame)
    BodyTwist3 twist = {0,0,0};   // vx, vy mm/s, omega rad/s (body frame)
    ValueSet   stamp = {};        // per-source freshness: lagMs / lastUpdMs / valid
};
```

- Reuses existing `Pose2D` / `BodyTwist3` from
  [source/io/capability/Pose2D.h](source/io/capability/Pose2D.h).
- **Uses `BodyTwist3` uniformly** (vy always present; differential always writes
  vy = 0). Costs 4 bytes/estimate but keeps every consumer — telemetry, the dump,
  `sim_api`, the fusion-validation code — `#ifdef`-free. *(See open question Q1.)*
- **Per-estimate `stamp`** (not one global) is what lets the dump say "OTOS is stale
  but encoder is fresh" — the core fusion-validation signal.

---

## 2. Actual state — "where we are"

```cpp
// source/state/ActualState.h  (new)
struct ActualState {
    // --- the three field-pose beliefs, side by side (headline feature) ---
    PoseEstimate encoder;   // integrated dead-reckoned pose from wheel deltas ONLY
    PoseEstimate optical;   // OTOS pose + twist as reported by the sensor
    PoseEstimate fused;     // EKF/fusion output — the authoritative belief

    // --- raw wheel odometry (substrate the encoder estimate integrates) ---
    float    encMm [Kinematics::kWheelCount] = {};  // cumulative distance, mm
    float    velMms[Kinematics::kWheelCount] = {};  // per-wheel velocity, mm/s
    ValueSet enc = {};

    // --- raw sensors not part of pose estimation ---
    float    otosAccelX = 0, otosAccelY = 0;             // mm/s^2 passthrough
    uint16_t line[4] = {};                  ValueSet lineVS  = {};
    uint16_t colorR=0, colorG=0, colorB=0, colorC=0; ValueSet colorVS = {};
    bool     digitalIn[4] = {}; int16_t analogIn[4] = {}; ValueSet portsVS = {};
};
```

- `encoder.pose` is the **new retained pre-fusion pose** (§4).
- `optical` promotes the loose `otosX/Y/H` into a first-class estimate, with twist
  filled from the OTOS velocity the code already reads.
- `fused` replaces `poseX/poseY/poseHrad` + `fusedV/fusedOmega/fusedVy`.
- `encMm/velMms` become `kWheelCount`-sized arrays (2 differential, 4 mecanum). The
  L/R scalar names survive as accessor shims during migration (§6), respecting the
  documented `[0]=FR,[1]=FL,[2]=BR,[3]=BL` index convention.

---

## 3. Desired state — "where we want to get to"

```cpp
// source/state/DesiredState.h  (new)
struct DesiredState {
    // --- desired chassis/body motion (profiled setpoint) ---
    BodyTwist3 bodyTwist    = {0,0,0};  // vx,vy,omega the profiler is ramping TO
    BodyTwist3 bodyTwistRaw = {0,0,0};  // commanded target before clamp/profile

    // --- desired wheel motion (output of inverse kinematics) ---
    float wheelMms[Kinematics::kWheelCount] = {};  // target wheel speeds, mm/s

    // --- goal / mode (the "where we want to be" world target) ---
    DriveMode mode = DriveMode::IDLE;
    float targetXWorld = 0, targetYWorld = 0;   // mm
    float targetSpeedMms = 0, distanceTargetMm = 0;
    uint32_t deadlineMs = 0;

    // --- desired port outputs (commanded, not yet flushed) ---
    bool    digitalOut[4] = {};
    int16_t analogOut[4]  = {};

    // --- async command bookkeeping ---
    ReplyFn replyFn = nullptr; void* replyCtx = nullptr;
    char corrId[16] = {}; MotionEventSink sink = {};
};
```

What moves where:

| Today | Lives in | Goes to |
|---|---|---|
| BVC `_v/_omega/_vy` (live profiled) | BVC private | mirror to `desired.bodyTwist` each `advance()` |
| BVC `_vTgt/_omegaTgt/_vyTgt` (commanded) | BVC private | mirror to `desired.bodyTwistRaw` |
| `MotorCommands.tgtLMms/tgtMms[]` | `commands` | `desired.wheelMms[]` |
| `TargetState.*` (mode, target, reply) | `target` | absorbed into `DesiredState` *(Q3)* |
| `MotorCommands.digitalOut/analogOut` | `commands` | `desired.*Out` |
| `MotorCommands.pwm*` + `*Dirty[]` | `commands` | `OutputState` (§4) — **not** desired |

`BodyVelocityController` stays the owner of profiler dynamics (its S-curve/jerk
integrator state is genuinely private compute); it additionally *publishes* its result
into `desired.bodyTwist*` so the dump and the actual-vs-desired comparison can read it
without poking into BVC internals. `currentV()/targetV()` accessors keep working.

---

## 4. Top-level container + the "outputs" split

```cpp
struct OutputState {                 // the *determined command* being driven now
    int16_t pwm[Kinematics::kWheelCount] = {};
    bool digitalDirty[4] = {}; bool analogDirty[4] = {};  // flush bookkeeping
};

struct RobotStateContainer {
    ActualState  actual;
    DesiredState desired;
    OutputState  outputs;
};
```

PWM and dirty-flags go in `OutputState`, **not** `desired`: your mental model is
"look at actual vs desired, then *determine the command*." PWM is that determined
command — a third thing. `actual / desired / outputs` mirrors measure → plan →
actuate. *(See open question Q2 — this adds a third top-level group you didn't name.)*

---

## 5. Dump / diagnostic surface

A no-allocation method that emits all three estimates side by side:

```cpp
struct EstimateDump { const char* source; Pose2D pose; BodyTwist3 twist;
                      uint32_t ageMs; bool valid; };
void dumpEstimates(const ActualState& a, uint32_t now_ms, EstimateDump out[3]);
```

Telemetry form (build-agnostic — `vy` always present, 0 on differential):

```
EST enc   x=.. y=.. h=.. vx=.. vy=.. w=.. age=.. v=1
EST otos  x=.. y=.. h=.. vx=.. vy=.. w=.. age=.. v=1
EST fuse  x=.. y=.. h=.. vx=.. vy=.. w=.. age=.. v=1
```

The `age`/`v` columns surface staleness, so a static FUSED while ENC drifts (or vice
versa) immediately shows whether fusion is catching.

---

## 6. The behavioral change — retaining the three estimates

This is the substantive change. Today the encoder pose is discarded into the EKF.
Instead, `Odometry` keeps its own dead-reckoning accumulator that fusion never
overwrites:

- Add private `float _encPoseX/_Y/_H` and `_encVx/_Vy/_Omega` — the integrated
  encoder pose/twist, never touched by fusion.
- `predict()`: arc-integrate encoder deltas into `_encPose*` (today it goes into the
  shared pose), run EKF predict as before, then write **two** destinations —
  `actual.encoder.{pose,twist}` ← `_encPose*` (NEW) and `actual.fused.{pose,twist}`
  ← EKF state (as today).
- `correctEKF()`: persist the raw OTOS observation into `actual.optical.{pose,twist}`
  *before* fusing, then write the EKF result into `actual.fused.*`.
- `setPose()` / camera re-anchor: reset **both** `_encPose*` and the EKF state — an
  absolute fix invalidates the dead-reckoned integral.
- `zero()`: zero `_encPose*` alongside the existing snapshot reset.

[source/state/PhysicalStateEstimate.h](source/state/PhysicalStateEstimate.h) gains thin
forwarders: `encoderEstimate()`, `opticalEstimate()`, `fusedEstimate()`. `predict()` /
`correctEKF()` take `ActualState&` (it owns all three estimates + raw wheel data)
instead of `HardwareState&`.

This is interface-compatible with the TinyEKF swap: the filter internals change but the
wiring (predict→fused, correct→fused, raw obs→encoder/optical) does not.

---

## 7. Migration — incremental layering (recommended)

A clean rebuild would force ~140 references to `poseX/fusedV/otos*` across 10+ files
plus the `sim_api` C-ABI and the Python suite to change atomically — high risk, and it
collides with the cmon-pid/TinyEKF work. Instead, layer it in:

1. **Phase A — types, no behavior change.** Add the new structs. Provide inline
   accessor shims so old names resolve to new fields (e.g.
   `inline float& poseX(RobotStateContainer& s){ return s.actual.fused.pose.x; }`,
   `encLMm` → `actual.encMm[1]`). *(Reference-member aliasing is rejected — it breaks
   `= {}` aggregate init, as `Inputs.h` already documents.)*
2. **Phase B — wire the three estimates** (§6) while keeping the legacy mirror writes
   alive. The dump now works; nothing else moved.
3. **Phase C — migrate consumers** off legacy names file-by-file (RobotTelemetry,
   MotionCommand, StopCondition, WorldView, DebugCommandable, Robot).
4. **Phase D — migrate `sim_api.cpp` bodies in lockstep**, then drop the mirror + shims.

**Keeping tests green:** the Python tests call the C-ABI functions in
[tests/_infra/sim/sim_api.cpp](tests/_infra/sim/sim_api.cpp), not struct fields. Update
those function bodies (`sim_get_pose_x` → `actual.fused.pose.x`, `sim_get_enc_l` →
`actual.encMm[1]`, `sim_get_pwm_l` → `outputs.pwm[1]`, etc.) — signatures unchanged, so
**no Python test edits are required**. New `sim_get_enc_pose_*` / `sim_get_otos_pose_*`
ABI functions can be *added* for tests that validate the three-way split.

**Mecanum:** the `#ifdef` shrinks to just `kWheelCount` array sizing; the structs
become build-agnostic and the `fusedVy` fork disappears (vy is now unconditional).

---

## Resolved decisions (stakeholder, 2026-06-27)

- **Q1 — RESOLVED: `vy` always present.** Use `BodyTwist3` uniformly. It will be 0 in
  most differential-drive cases and that is acceptable.
- **Q2 — RESOLVED: keep the three-way `actual / desired / outputs` split.** PWM +
  dirty-flags live in `OutputState`, not `desired`. (Stakeholder flagged wanting to
  discuss further but deferred; default to the clean three-way split, collapsible later
  if they object on review.)
- **Q3 — RESOLVED: flatten `TargetState` into `DesiredState`.** They are essentially
  the same thing — the goal/desired-motion state. Do not keep a nested `target`
  sub-struct; absorb the fields directly.
- **Q4 — RESOLVED: reuse `v_otos/omega_otos`** (the values already passed into
  `correctEKF`) for `optical.twist`. Do not differentiate OTOS pose.
- **Q5 — RESOLVED: published read-model copy.** BVC remains the single source of truth
  for profiler dynamics and publishes a copy into `desired.bodyTwist*` each `advance()`.

## Verification (when implemented)

- Build both variants: `python build.py --clean` (differential) and the mecanum config,
  confirm zero shared-code breakage.
- Run the sim unit suite (`tests/simulation/unit/`) — must stay green with no test edits
  (ABI-preserved).
- Add a focused test asserting that after a straight drive with injected OTOS offset,
  `actual.encoder.pose` and `actual.fused.pose` diverge as expected and the `EST` dump
  emits all three rows.
- On-bench: drive a square, capture the `EST` telemetry, confirm encoder vs optical vs
  fused tracks plausibly (fusion-validation use case).
