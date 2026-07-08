---
status: in-progress
sprint: 090
tickets:
- 090-002
---

# Odometer owns reset translation and fusability (remove reset plumbing from the main loop)

Extracted from `sprint-089-omnibus.md` entry 2 (stakeholder design discussion 2026-07-07).

## Problem

`source/runtime/main_loop.cpp:158-203` tangles three odometer-domain concerns
into `MainLoop::tick`:

1. Drains the reset commands (`otosCommandIn`, `otosSetPoseIn`) and applies them
   to the odometer.
2. Translates `SetPose → Pose2D → OdometerCommand` inline (lines 178-188) — pure
   odometer-domain plumbing sitting in the loop.
3. Computes a loop-local `odometerResetThisPass` bool that gates OTOS fusion at
   line 202: `(bb.otosValid && !odometerResetThisPass) ? &bb.otos : nullptr`.

Concern 3 is the real cohesion smell: "is OTOS fusable this pass" is a
measurement/odometer property, computed in the loop as a local bool instead of
exposed by the odometer. (Draining the reset command is *not* inference — the
loop applied the reset, so it legitimately knows one happened; that part is
fine.)

## Direction (behavior-preserving refactor — minimal scope)

Move the *logic* into the odometer; the loop keeps orchestration only:

- **`odometer->applySetPose(pose)`** — the odometer owns the
  `SetPose → Pose2D → OdometerCommand` translation. Loop stops building
  `OdometerCommand` by hand.
- **Replace the loop-local `odometerResetThisPass` with an odometer-exposed
  query** (e.g. `odometer->fusableThisPass()` / `wasResetThisPass()`). The loop
  still wires the fusion gate into `poseEstimator_.tick()`, but the *decision*
  becomes an odometer property, not loop arithmetic.

## MUST preserve — this block guards a live-debugged EKF fix

`odometerResetThisPass` is not incidental. On the exact pass a reset lands,
`bb.otos` still holds the **stale, pre-reset** reading (it is an x[k] cell,
refreshed only at COMMIT, line 272), so feeding it to the pose estimator
fabricates a large false innovation against the freshly `setPose`'d EKF —
reproduced live via SI (`fusedPose` dragged back toward the pre-reset reading;
`encoderPose` landed correctly). The refactor MUST preserve: **skip OTOS fusion
for exactly the one pass a reset is applied; resume next pass with zero
innovation.** The SI/OZ/OR/OV regression tests must be green before and after.

## Known tension (bounds how clean this gets)

The one-pass skip is inherently a **this-pass transient**: a reset applied at the
*start* of a pass must suppress a read of a *last-pass-committed* cell. It does
not map onto a committed blackboard cell (you cannot retroactively edit this
pass's already-committed `otosValid`). So an explicit transient "reset just
happened" signal survives no matter how it is abstracted — this issue moves
*where that signal lives* (into the odometer), it does not eliminate it.

## Out of scope (possible follow-on)

Making the odometer a first-class ordered-tick subsystem with its own blackboard
slice that drains its own reset commands is a larger restructure — it reverses
the current design's "odometer is a composition-root exception, no tick()-driven
queue" note (`main_loop.cpp:154-157`). Noted as a possible future sprint, NOT
folded into this issue.

## Scope

- `source/runtime/main_loop.cpp` (~lines 158-203)
- `source/hal/.../odometer.{h,cpp}` (new `applySetPose()` + fusability query)
