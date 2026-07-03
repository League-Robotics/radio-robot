---
status: done
sprint: '070'
tickets:
- 070-003
---

# PhysicalStateEstimate: stop threading HardwareState through every call; make config and inputs explicit

## Description

`PhysicalStateEstimate` (source/state/PhysicalStateEstimate.h) takes a
`HardwareState& s` parameter on essentially every method —
`addOdometryObservation`, `addOtosObservation`, `resetPose`, `zero`,
`getPose`, `getVelocity`, and even the three-estimate forwarders
(`encoderEstimate(s)` just returns `s.encoder`). A dependency that appears
in every function signature is not a parameter; it should be injected at
construction time or set once — or better, decomposed so the class only
receives the specific things it actually needs.

Three related smells:

1. **HardwareState threading.** Every observation/read call re-passes the
   state object. (`HardwareState` is itself only a back-compat alias for
   `ActualState`, per ActualState.h.) What the class actually touches is
   narrow: it *reads* encoder distances (`s.encMm`) as input and *writes*
   the three pose estimates (`s.encoder`, `s.optical`, `s.fused` — pose,
   twist, stamp). The rest of HardwareState is never used.

2. **Per-call configuration.** `addOdometryObservation(s, trackwidthMm,
   rotationalSlip, now_ms)` takes trackwidth and rotational slip on every
   call, and the caller (Drive.cpp:127-131) re-derives them from config
   every tick. These are configuration, not observations — they should be
   set once on the estimate (constructor or a `configure()`/setter, with
   the setter kept so runtime `SET` changes propagate — cf. the
   set-config-not-propagated-to-planner issue).

3. **Inconsistent `setCtx`.** `setCtx(IOdometer*, const HardwareState*)`
   already binds a HardwareState pointer (Robot.cpp:130 passes
   `&state.actual`), but the observation methods ignore it and take the
   state per-call anyway. Half-injected, half-threaded is the worst of
   both.

## Options

1. **Construct on the hardware state.** Give PhysicalStateEstimate a
   `HardwareState&` (or `ActualState&`) at construction and drop the
   parameter from all methods. Simple, removes the threading, but keeps
   the class coupled to the whole state blob.

2. **(Preferred, per stakeholder)** **Extract the explicit pieces.**
   Identify exactly what the estimate consumes and produces, and make
   those explicit:
   - Inputs become real observation parameters: e.g.
     `addOdometryObservation(encLeftMm, encRightMm, nowMs)` — pass the
     encoder readings themselves, not a state object to fish them out of.
   - Outputs (the three `PoseEstimate`s — encoder/optical/fused) are
     either owned by PhysicalStateEstimate and mirrored/read from there,
     or bound once at construction as an explicit "estimates out"
     destination.
   - Config (trackwidth, rotational slip, EKF noise) is set at
     construction/configure time with runtime setters.

Option 2 also cleans up the leftovers: the static `getPose`/`getVelocity`
that just read fields off the passed state, and the three-estimate
forwarders whose returned-reference-lifetime caveat exists only because
the state is passed per-call.

## Scope / call sites

- source/state/PhysicalStateEstimate.h / .cpp (the API itself)
- source/control/Odometry.{h,cpp} (the wrapped implementation reads
  `s.encMm`, writes `s.encoder/optical/fused`)
- Callers: source/subsystems/drive/Drive.cpp (tick-time observation calls,
  pose reset), source/robot/Robot.cpp (setCtx wiring, OTOS observation),
  plus TLM/readers that consume the estimates.

## Acceptance criteria

- No PhysicalStateEstimate method takes a `HardwareState&`/`ActualState&`
  parameter; the class's inputs and outputs are explicit (per option 2) or
  bound once at construction (fallback option 1).
- Trackwidth and rotational slip are configured on the estimate, not
  passed per call; runtime SET updates still reach it.
- `setCtx` is either removed or becomes the single injection point — no
  mixed injected/threaded state.
- Behavior unchanged: sim + host tests green; TLM three-pose output
  identical before/after.
