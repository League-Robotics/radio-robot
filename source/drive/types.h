// types.h -- Drive:: plain value types: Pose, Twist, WheelState, BodyState,
// WheelVelocities, Limits. Every type here is a bare struct with default-
// initialized members -- no methods beyond the implicit ones, no msg::, no
// Hal::, no CODAL, no heap. This is the foundation every other file under
// source/drive/ builds on (architecture-update.md (100) M2); the directory's
// isolation boundary (SUC-008, enforced by the grep test alongside this
// file) starts here: nothing in this header may name anything outside
// source/drive/, libc/libm, or libraries/ruckig.
//
// Naming: no units in identifiers (.claude/rules/naming-and-style.md) --
// every unit lives in a leading `// [unit]` comment tag on its field. A body
// twist is never a bare directionless `v`; it always carries its v_x/v_y/
// omega components (a future holonomic drivetrain may use v_y; a
// differential one leaves it 0.0f, same convention as
// kinematics/body_kinematics.h's msg::BodyTwist3 array-form overloads).
#pragma once

namespace Drive {

// Pose -- a world-frame (or, where documented by the caller, a single
// segment's own anchor-relative) rigid-body pose.
struct Pose {
  float x = 0.0f;  // [mm]
  float y = 0.0f;  // [mm]
  float h = 0.0f;  // [rad] heading, CCW+, wrapped to (-pi, pi] by convention
                    // (see arc_math.h's wrapAngle) -- not enforced by this
                    // struct itself, since a Pose is a plain value type.
};

// Twist -- a body-frame velocity. Never a bare directionless `v` -- always
// the full (v_x, v_y, omega) triple (naming-and-style.md Rule 2), even
// though today's differential drivetrain always has v_y == 0.0f.
struct Twist {
  float v_x = 0.0f;    // [mm/s] body-forward
  float v_y = 0.0f;    // [mm/s] body-lateral; 0 for a differential drivetrain
  float omega = 0.0f;  // [rad/s] yaw rate, CCW+
};

// WheelState -- one wheel's measured position/velocity, with the
// observation validity that msg::MotorState's has-fields carry today (this
// type is the drive/-local, msg::-free equivalent -- StepInput's `left`/
// `right` fields, motion_plan.h's own doc comment). A caller must never
// fabricate a value when the observation is absent this tick; it sets the
// corresponding *Valid flag false and leaves the float at its last value or
// 0.0f, the consumer's choice.
struct WheelState {
  float position = 0.0f;    // [mm] measured wheel travel
  float velocity = 0.0f;    // [mm/s] measured wheel speed
  bool positionValid = false;
  bool velocityValid = false;
};

// BodyState -- the caller-maintained pose estimate plus the body-frame
// twist derived from it (e.g. via BodyKinematics::forward at the boundary
// adapter, outside this directory) -- StepInput.measured, replan()'s
// `measured`, planVelocity()'s `current` (the two core header sketches in
// the driving issue). Pose ownership is OUTSIDE source/drive/ (the
// subsystem is stateless); this struct is how the caller hands its own
// pose estimate IN for one call, never stored.
struct BodyState {
  Pose pose;    // [mm][mm][rad] world pose estimate
  Twist twist;  // [mm/s][mm/s][rad/s] body-frame velocity estimate
};

// WheelVelocities -- the subsystem's one output shape: wheel velocity
// setpoints staged to the LEVEL-2 motor velocity PIDs (unchanged,
// bench-tuned, outside this directory -- see the issue's "Two levels of
// control"). This is StepOutput.command.
struct WheelVelocities {
  float left = 0.0f;   // [mm/s] left wheel velocity setpoint
  float right = 0.0f;  // [mm/s] right wheel velocity setpoint
};

// ProfileLimits -- one master_profile.h channel's kinematic bounds: an
// outer velocity ceiling plus accelerating/decelerating/jerk bounds. Used
// as-is for both the linear channel (path length, mm) and the rotational
// channel (heading, rad) -- the rotational channel is symmetric by
// construction (accel == decel), matching jerk_trajectory.h's
// isRotational-collapses-to-symmetric behavior, but WITHOUT a separate
// boolean flag: the caller simply sets accel == decel for a symmetric
// channel instead of MasterProfile branching on a flag internally.
struct ProfileLimits {
  float velocity = 0.0f;  // [mm/s] or [rad/s] outer ceiling (this channel's
                           // own configure()-time bound; every solve's own
                           // per-call maxVelocity argument is clamped
                           // underneath it, never the other way around)
  float accel = 0.0f;     // [mm/s^2] or [rad/s^2] accelerating-direction bound
  float decel = 0.0f;     // [mm/s^2] or [rad/s^2] decelerating-direction
                           // bound (magnitude); equal to accel for a
                           // symmetric (rotational) channel
  float jerk = 0.0f;      // [mm/s^3] or [rad/s^3] jerk bound; 0.0f is the
                           // sentinel that maps to Ruckig's own +infinity
                           // default (master_profile.h's own doc comment)
};

// Limits -- Drive::Drivetrain's immutable configuration (the driving
// issue's `Drivetrain(const Limits& limits, float trackwidth)` ctor). This
// ticket (100-002) only needs the two ProfileLimits channels master_profile
// consumes; later tickets (003-005: admission, tracker, policy) extend this
// struct with the remaining PlannerConfig-sourced quantities (wheel/steer
// headroom, trim gains, replan envelopes -- architecture-update.md M1's
// PlannerConfig fields 15-31) as those modules land. Adding fields here is
// source-compatible with every existing caller (default member
// initializers), so this struct is deliberately grown incrementally rather
// than speculatively populated now.
struct Limits {
  ProfileLimits linear;      // path-length master DOF (arcs)
  ProfileLimits rotational;  // heading master DOF (pivots; also arc omega)

  // -- ticket 100-003 additions: the PlannerConfig-sourced scalars
  // Drivetrain::admit()/plan()/planVelocity() consume directly, beyond the
  // two ProfileLimits channels above. Tracker/policy gains (track_k_s/
  // track_k_theta/track_k_cross, replan envelopes, handoff/arrive
  // tolerances -- architecture-update.md M1's remaining PlannerConfig
  // fields 15-31) are deliberately NOT added here; they land with tickets
  // 004/005, the modules that actually consume them (this struct's own
  // "grown incrementally, not speculatively" rule, stated above).
  float vWheelMax = 0.0f;     // [mm/s] wheel velocity ceiling
                               // (PlannerConfig.v_wheel_max) -- plan()'s
                               // v_eff/omega_eff wheel-budget fold
  float trimVMax = 0.0f;      // [mm/s] along-track trim clamp
                               // (PlannerConfig.trim_v_max) -- plan()'s
                               // headroom fold uses this scalar directly;
                               // the TRACKER's (ticket 004/005) own
                               // per-tick clamp is a separate, later
                               // consumer of the same wire field
  float trimOmegaMax = 0.0f;  // [rad/s] heading trim clamp
                               // (PlannerConfig.trim_omega_max) -- same
                               // headroom-fold role as trimVMax above. The
                               // issue's control-law table lists a second,
                               // pivot-specific 2.0 rad/s trim cap; ticket
                               // 100-004's tracker.cpp resolves this: the
                               // AC's own transcribed pivot formula
                               // (omegaCmd = omegaRef + trackKTheta *
                               // eTheta) carries NO clamp at all (matching
                               // sprint 098's proven, unclamped heading
                               // loop -- confirmed by reading
                               // motion/segment_executor.cpp's own PD
                               // cascade, which never clamps omega
                               // either), so this single scalar is
                               // consumed ONLY by the arc-mode clamp
                               // (plan()'s headroom fold, above, and
                               // track()'s own arc-mode omegaTrim clamp
                               // below) -- the pivot-specific 2.0 rad/s
                               // table value is not wired to any clamp by
                               // this ticket, as transcribed
  float wheelStepMax = 0.0f;  // [mm/s] admit()'s joint wheel-speed-step
                               // cap (PlannerConfig.wheel_step_max)

  // -- ticket 100-004 additions: the tracker's own P-only Kanayama gains
  // and pivot-mode threshold (PlannerConfig fields track_k_s/
  // track_k_theta/track_k_cross/min_speed) -- the remaining scalars
  // tracker.{h,cpp}'s track() cascade consumes directly, alongside
  // trimVMax/trimOmegaMax/vWheelMax above. No k_d/derivative gain field
  // exists here, deliberately -- the P-only outer-loop rule
  // (architecture-update.md, the issue's "encoder omega-hat is stale
  // staggered noise" rationale); adding one would be a structural
  // regression, not an oversight.
  float trackKS = 0.0f;       // [1/s] along-track trim proportional gain
                               // (PlannerConfig.track_k_s) -- tracker.cpp's
                               // vTrim = clamp(trackKS * (reference -
                               // measured along error), +/-trimVMax); see
                               // tracker.h's "Reconciled sign convention"
                               // class comment for why the sign is
                               // reference-measured, not arc_math's own
                               // measured-reference ArcError convention
  float trackKTheta = 0.0f;   // [1/s] heading trim proportional gain
                               // (PlannerConfig.track_k_theta) -- used in
                               // both arc mode (clamped, alongside the
                               // cross term) and pivot mode (UNCLAMPED,
                               // omegaTrim = trackKTheta * (reference -
                               // measured heading error) -- sprint 098's
                               // proven heading loop; see tracker.h's
                               // "Reconciled sign convention" class comment
  float trackKCross = 0.0f;   // [rad/mm^2] cross-track trim gain, coupled
                               // with the reference speed
                               // (PlannerConfig.track_k_cross) -- arc-mode
                               // omegaTrim's own v_ref*(reference-measured
                               // cross error) term (same reconciled sign as
                               // trackKTheta above); not consumed in pivot
                               // mode (v_ref == 0 there by construction)
  float minSpeed = 0.0f;      // [mm/s] pivot-mode threshold
                               // (PlannerConfig.min_speed) -- track()
                               // selects pivot mode (literal-zero v_cmd,
                               // unclamped heading-only omega trim) iff
                               // fabsf(ref.v) < minSpeed
};

}  // namespace Drive
