// stop_condition.h -- Motion::StopCondition: reports whether one bounded
// MOVE's stop condition (time/distance/angle) or its required timeout
// backstop has been met.
//
// Boundary (sprint 116, protocol-set-point issue "Firmware design notes"):
// inside -- kind (Time/Distance/Angle) + threshold + the activation-time
// baselines (clock time, App::Odometry::pathLength(), App::Odometry::
// theta()) + the per-cycle comparison against hand-fed CURRENT readings;
// outside -- what happens once tick() reports the MOVE has ended
// (App::MoveQueue's job, ticket 005) and where the readings compared
// against come from (App::Odometry/Devices::Clock -- the owning
// MoveQueue's job to read and pass in; NEVER read from a reference held
// here). No dependency on App::MoveQueue, App::Drive, or any msg::* wire
// type -- constructible and testable with hand-fed numbers alone, the
// same pure-computation shape BodyKinematics (kinematics/) already
// established in this tree.
//
// Lifecycle: one StopCondition instance per ACTIVATED Move. The
// constructor captures every baseline (activation time, path length,
// heading) at construction -- there is no separate arm()/activate() call;
// a Move's activation IS this object's construction (sprint.md Decision 1:
// MoveQueue owns and drives its own StopCondition, constructing a fresh
// one each time a Move activates). When a Move ends (stop condition met
// or timed out), the owning MoveQueue simply discards this instance; the
// next activated Move gets a fresh one.
//
// theta() is already verified UNWRAPPED (theta_ += headingDelta, no
// modulo anywhere in odometry.cpp) -- the Angle kind diffs the caller's
// theta reading against its own activation baseline directly; no wrap
// handling here.
//
// Design/rationale: DESIGN.md (this directory).
#pragma once

#include <cstdint>

namespace Motion {

class StopCondition {
 public:
  // Which reading `threshold` is measured against -- mirrors the wire's
  // own Move.stop oneof (msg::Move::StopKind) in spirit, but is NOT that
  // type: this module has zero dependency on messages/envelope.h or any
  // other wire type (see file header).
  enum class Kind : uint8_t { Time, Distance, Angle };

  // Distinguishable per-tick outcomes -- NOT a collapsed bool. The caller
  // needs to tell "the kind-specific stop condition fired" apart from
  // "the timeout backstop fired" to set kFlagFaultMoveTimeout correctly.
  //   Continue         -- neither has fired yet; the Move keeps running.
  //   StopConditionMet -- the kind-specific (Time/Distance/Angle)
  //                       comparison fired.
  //   TimedOut         -- the timeout backstop fired, and the
  //                       kind-specific comparison did NOT also fire this
  //                       same cycle (see tick()'s own doc comment for
  //                       the tie-break).
  enum class Outcome : uint8_t { Continue, StopConditionMet, TimedOut };

  // kind      -- which reading `threshold` is measured against.
  // threshold -- the kind-specific stop value, Move.stop's own wire
  //   units: [ms] elapsed time (Kind::Time), [mm] |path length| traveled
  //   (Kind::Distance), or [rad] |heading change| (Kind::Angle).
  //   threshold <= 0 (including NaN) clamps to 0 -- see "Zero/negative
  //   threshold" below.
  // timeout   -- [ms] REQUIRED safety backstop, independent of kind.
  //   timeout <= 0 (including NaN) clamps to 0, mirroring the same
  //   malformed-input-safety posture the deleted App::Deadman::arm() used
  //   for its own `duration` parameter. This is defense in depth only --
  //   the wire-level ERR_BADARG rejection of a non-positive Move.timeout
  //   is ticket 006's handleMove()'s job; a well-formed Move never
  //   reaches this constructor with a non-positive timeout at all.
  // now       -- [us], Devices::Clock::nowMicros()'s own unit -- the
  //   activation-time baseline.
  // pathLength/theta -- [mm]/[rad], App::Odometry::pathLength()/theta()'s
  //   own readings AT ACTIVATION -- the Distance/Angle baselines. Passed
  //   in, never read from an owned Odometry reference (this module owns
  //   no collaborator at all).
  //
  // Zero/negative threshold (sprint.md Architecture Open Question 1,
  // PINNED HERE): threshold and timeout both clamp non-positive (<=0,
  // including NaN) input to 0 -- the SAME ">0, else 0" rule, applied
  // uniformly to every kind and to timeout, with no per-kind special
  // case. Effect: a magnitude-based kind (Distance/Angle) with a clamped
  // 0 threshold reports StopConditionMet on the very FIRST tick() call
  // (|delta| >= 0 is trivially true from the first reading onward); a
  // Kind::Time with a clamped 0 threshold reports StopConditionMet on the
  // very first tick() call too (elapsed >= 0 is always true at or after
  // activation) -- the "deliberate one-cycle no-op" idiom sprint.md's
  // Open Question 1 names is achieved uniformly across all three kinds by
  // this one clamp-to-zero rule, rather than a Time-specific carve-out.
  // A clamped-to-0 timeout behaves identically (TimedOut on the very
  // first tick() call) unless the kind-specific condition ALSO fires
  // that same call, in which case the tie-break below still applies.
  StopCondition(Kind kind, float threshold, float timeout, uint64_t now,
                float pathLength, float theta);

  // tick -- one per-cycle comparison. now/pathLength/theta are the
  // caller's CURRENT readings (same units as the constructor's own
  // baseline arguments) -- never read from a held reference. Pure: no
  // state mutated, safe to call speculatively.
  //
  // Tie-break (acceptance criterion, sprint.md Step 3's own summary):
  // when BOTH the kind-specific condition and the timeout are met the
  // same cycle, StopConditionMet is reported, never TimedOut -- the
  // kind-specific result always takes precedence. Consequence: a
  // well-formed Move (kind threshold reachable before timeout) always
  // ends via StopConditionMet; TimedOut is only ever reported on a cycle
  // where the kind-specific condition has NOT also fired.
  Outcome tick(uint64_t now, float pathLength, float theta) const;

 private:
  Kind kind_;

  // [mm] or [rad] -- Distance/Angle kinds only, clamped >0:0. Unused
  // (left 0) for Kind::Time, which compares deadlines instead (below).
  float threshold_;

  // [us] deadlines, both precomputed once at construction as
  // activation-time + clamped-threshold-converted-to-us, mirroring
  // Deadman::arm()'s own "convert once, compare with a plain >=" shape.
  // timeDeadlineUs_ is meaningful only for Kind::Time; timeoutDeadlineUs_
  // always applies, independent of kind.
  uint64_t timeDeadlineUs_;
  uint64_t timeoutDeadlineUs_;

  float activationPathLength_;  // [mm]
  float activationTheta_;       // [rad]
};

}  // namespace Motion
