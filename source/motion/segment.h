// segment.h -- Motion::Segment: the pose-free, all-relative motion command
// Motion::SegmentExecutor (segment_executor.h) executes as a chain of up to
// three single-channel Ruckig phases (PRE_PIVOT -> TRANSLATE ->
// TERMINAL_PIVOT).
//
// Sprint 094 (architecture-update.md Section 3, "the segment message shape").
// Deviation from the originating issue's suggested `msg::Segment`: a segment
// is NOT a wire or serialized type (the wire carries `MOVE <args>`; this
// struct is the internal executor input) and `source/messages/*.h` is
// auto-generated from `protos/*.proto` -- a hand-authored type there would be
// fragile. This is a plain host-safe POD in `Motion::`, includable by both
// the executor and (for the blackboard mailbox payload, ticket 094-005) by
// blackboard.h, with zero CODAL dependency -- exactly like Motion::
// MotionBaseline and Motion::JerkTrajectory::State.
//
// Naming per .claude/rules/naming-and-style.md -- quantities, not units; no
// `duration` field (it is an OUTPUT of the Ruckig solve, never an input).
#pragma once

namespace Motion {

struct Segment {
  // --- geometry (all RELATIVE to the segment's start pose; pose-free) ---
  float distance = 0.0f;      // [mm] signed straight-line translation
  float direction = 0.0f;     // [rad] pre-pivot heading change, CCW+ (0 = straight ahead)
  float finalHeading = 0.0f;  // [rad] final heading relative to start, CCW+

  // --- per-segment motion limits (0 => fall back to the executor's
  //     configured default, i.e. msg::PlannerConfig; the 0-sentinel matches
  //     JerkTrajectory's own jerk-off sentinel, jerk_trajectory.h) ---
  float speedMax = 0.0f;      // [mm/s]     translation speed ceiling
  float accelMax = 0.0f;      // [mm/s^2]   translation accel
  float jerkMax = 0.0f;       // [mm/s^3]   translation jerk (0 => trapezoid)
  float yawRateMax = 0.0f;    // [rad/s]    pivot angular-speed ceiling
  float yawAccelMax = 0.0f;   // [rad/s^2]  pivot angular accel
  float yawJerkMax = 0.0f;    // [rad/s^3]  pivot angular jerk
};

}  // namespace Motion
