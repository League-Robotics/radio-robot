// state_estimator.h -- App::StateEstimator: predict-to-now wheel/body PEER
// state estimates, zero-order-hold (ZOH) v1 extrapolation, and a v1
// complementary-blend fusion against OTOS heading/omega.
//
// Boundary: inside -- holding the latest per-wheel and body basis readings,
// answering "predict to now"/"predict to an arbitrary instant" queries via
// ZOH extrapolation, and blending a fresh OTOS heading/omega reading onto
// the body peer's basis via a live-tunable complementary weight; outside --
// deciding WHEN to call update() (RobotLoop's own kPace-block placement,
// ticket 004), where the fusion weights actually come from at boot/live-tune
// time (Config::defaultEstimatorConfig()/a live EstimatorConfigPatch, both
// ticket 003's job -- this class only ever receives a plain FusionWeights
// value, never a msg::* patch type), and consuming whereAmI()/wheelAt() to
// drive motion (a later, out-of-this-sprint trajectory controller).
//
// No #include of any messages/ or config/ header: this module is pure
// app/-internal computation and never spells msg::* anywhere in its own
// source -- mirrors App::Odometry's own established posture (odometry.h
// includes app/telemetry.h and reads/writes Telemetry::Frame's msg-typed
// fields via plain member access without ever naming msg:: itself). Wire-
// plane conversion for a live EstimatorConfigPatch stops at
// RobotLoop::handleConfig(), exactly like msg::MotorConfigPatch/
// msg::OtosConfigPatch already do -- setWeights() below takes a plain,
// Devices-local-style FusionWeights struct, never a msg::* type.
//
// No I2C bus access, no sleeping, no owned Devices::Clock& collaborator:
// every time-taking method (update()/wheelAt()/bodyAt()/whereAmI()) takes
// an EXPLICIT now/t argument instead, mirroring Motion::StopCondition's own
// "hand-fed readings, no owned collaborator" shape (stop_condition.h's own
// file header) -- keeps this class constructible/testable with plain
// numbers, no fake clock needed. Basis times are uint32_t [ms], matching
// EncoderReading/OtosReading::time's existing wire-frame units (NOT
// uint64_t [us]) -- the granularity Frame's own fields actually carry, not
// a claim of more precision than the frame provides.
//
// Design/rationale: DESIGN.md (this directory) -- see its "117" sections.
#pragma once

#include <cstdint>

#include "app/telemetry.h"

namespace App {

// Which wheel a wheelAt()/wheelNow() query targets -- a plain, module-local
// enum (mirrors Motion::StopCondition::Kind's own un-prefixed scoped-enum
// style), never msg::BoundMotorSide (a wire type this module must not
// depend on).
enum class Wheel : uint8_t { Left, Right };

// WheelEstimate -- one wheel's PEER basis reading (independently
// valid/stale from the other wheel and from the body peer). `distance` is
// the wheel's own traveled distance (EncoderReading::position, NOT a world
// pose) at `basisTime`; `velocity` is held constant across a ZOH
// extrapolation from that basis.
struct WheelEstimate {
  float distance = 0.0f;   // [mm]
  float velocity = 0.0f;   // [mm/s] signed
  uint32_t basisTime = 0;  // [ms]
  bool valid = false;      // false until the peer's first update() call
};

// BodyEstimate -- the body peer's PEER basis reading: a world pose
// (x, y, heading) plus a body-frame twist (v_x, v_y, omega), all held
// constant across a ZOH extrapolation from `basisTime`. x/y/v_x/v_y always
// come straight from Odometry's own dead-reckoned frame.pose/frame.twist
// (encoder-only, never OTOS-blended this sprint -- the wire schema's
// EstimatorConfigPatch has no weight_x_otos/weight_y_otos field, ticket
// 003); heading/omega are the v1 complementary blend against a fresh
// frame.otos reading when present (see update()'s own doc comment).
struct BodyEstimate {
  float x = 0.0f;          // [mm]
  float y = 0.0f;          // [mm]
  float heading = 0.0f;    // [rad]
  float v_x = 0.0f;        // [mm/s] body-frame, signed
  float v_y = 0.0f;        // [mm/s] body-frame, signed
  float omega = 0.0f;      // [rad/s] signed
  uint32_t basisTime = 0;  // [ms]
  bool valid = false;      // false until the peer's first update() call
};

// FusionWeights -- the v1 complementary-blend weights, plus the staleness
// window that gates whether a given cycle's OTOS reading is even eligible
// to blend. Constructor-injected plain values this ticket (002); the
// fail-closed boot-time default (baked from data/robots/*.json) and the
// live CONFIG-patch wire arm that actually FEED setWeights() at runtime
// are ticket 003's job -- see this file's own header. Dimensionless
// weights carry no unit tag (coding-standards.md); `staleness` does.
struct FusionWeights {
  float headingOtos = 0.0f;         // [0..1] blend weight: body heading vs OTOS heading
  float omegaOtos = 0.0f;           // [0..1] blend weight: body omega vs OTOS omega
  uint32_t staleness = 200;         // [ms] max OTOS reading age still eligible to blend
};

// Innovations -- the most recent OTOS-vs-predicted heading/omega residual,
// computed by update() whenever a fresh OTOS reading is blended -- even at
// weight 0.0 (diagnostic/validation only at that weight; never fed back
// into the estimate itself at v1). Holds its last value on a cycle with no
// fresh OTOS reading, mirroring Telemetry's own "last staged snapshot,
// never a diff" posture.
struct Innovations {
  float heading = 0.0f;  // [rad] OTOS heading minus predicted heading, at blend time
  float omega = 0.0f;    // [rad/s] OTOS omega minus predicted omega, at blend time
  bool valid = false;    // false until a fresh OTOS reading has been blended at least once
};

class StateEstimator {
 public:
  // weights -- constructor-injected plain value (see FusionWeights' own
  // doc comment); defaults to a conservative encoder-only/no-blend
  // FusionWeights{} for a caller (e.g. a ZOH-extrapolation-only unit test)
  // that never cares about fusion.
  explicit StateEstimator(FusionWeights weights = FusionWeights{});

  // update -- refreshes both wheel peers straight from `frame.encLeft`/
  // `frame.encRight` (position, velocity, their own collect `time`) and the
  // body peer from `frame.pose`/`frame.twist` (Odometry's own dead-reckoned
  // fusion, already computed earlier the same cycle) blended with
  // `frame.otos`/`frame.otosPresent` (when fresh -- `frame.otosPresent`
  // AND the reading's own age against `weights().staleness`) via the v1
  // complementary weight. Call once per cycle, after `frame.pose` is
  // staged (RobotLoop's trailing kPace block, ticket 004). Pure
  // computation over already-staged data: no I2C access, no sleep, bounded
  // work -- mirrors Odometry::integrate()/applyOtosSample()'s own posture
  // in that same block.
  void update(const Telemetry::Frame& frame, uint32_t now);  // [ms]

  // wheelAt/bodyAt -- pure ZOH extrapolation from the CURRENT basis to an
  // explicit query time `t`. Precondition: `t` is at or after the queried
  // peer's own basisTime (a query before the peer's first update() simply
  // returns the peer's own valid=false zero-state, extrapolated or not).
  // wheelAt: distance = basis.distance + basis.velocity * (t - basisTime),
  // velocity held constant. bodyAt: x/y extrapolate along the basis-time
  // world-frame velocity (basis heading rotates the held-constant
  // body-frame v_x/v_y into world frame -- a first-order approximation
  // valid for the small ages this sprint's every-cycle basis refresh
  // produces); heading = basis.heading + basis.omega * (t - basisTime) --
  // HeadingSource::headingLead()'s equation, generalized to the full pose.
  // The returned estimate's own `basisTime` field is left as the ORIGINAL
  // basis reading's timestamp (what informed the extrapolation), not `t`.
  WheelEstimate wheelAt(Wheel wheel, uint32_t t) const;  // [ms]
  BodyEstimate bodyAt(uint32_t t) const;                 // [ms]

  // whereAmI -- exactly bodyAt(now); a named convenience for the common
  // "predict to right now" query.
  BodyEstimate whereAmI(uint32_t now) const;  // [ms]

  // wheelNow -- the wheel's raw basis reading, zero extrapolation (NOT
  // wheelAt(wheel, basis.basisTime) re-derived -- returns the stored peer
  // verbatim, though the two are numerically identical when t == basisTime).
  WheelEstimate wheelNow(Wheel wheel) const;

  // reset -- re-anchors ONLY the body peer's world pose (x, y, heading),
  // mirroring Odometry::reset()'s own teleport semantics (no `now`
  // argument, same as Odometry -- the next update() call naturally
  // re-baselines basisTime within one cycle). Wheel-peer state (distance/
  // velocity/basisTime/valid) is UNTOUCHED -- wheel peers track per-wheel
  // distance, not world pose, the same reasoning Odometry::pathLength() is
  // untouched by Odometry::reset(). Does NOT change `valid` -- a peer that
  // was never updated stays valid=false after a reset() alone (valid only
  // ever flips true via update(), per this class's own contract).
  void reset(float x, float y, float heading);  // [mm] [mm] [rad]

  // innovations -- see Innovations' own doc comment.
  Innovations innovations() const { return innovations_; }

  // setWeights -- RobotLoop::handleConfig()'s own entry point for a live
  // EstimatorConfigPatch (ticket 003): replaces the whole live weight
  // state at once (ticket 003's own handler merges the wire patch's
  // PRESENT fields onto a snapshot of weights() BEFORE calling this, the
  // same present-field-merge-then-apply shape applyMotorConfigPatch()/
  // applyOtosPatch() already use -- this method itself does no partial
  // merging). A plain in-memory update: no I2C access, no bus transaction.
  void setWeights(FusionWeights weights) { weights_ = weights; }
  FusionWeights weights() const { return weights_; }

 private:
  WheelEstimate wheelLeft_;
  WheelEstimate wheelRight_;
  BodyEstimate body_;
  FusionWeights weights_;
  Innovations innovations_;
};

}  // namespace App
