// heading_source.h -- App::HeadingSource: decides which sensor is truth for
// heading RIGHT NOW, and makes that choice visible. Sprint 109 ticket 005's
// own new seam (sprint.md's Architecture Step 3, responsibility group 4).
//
// Boundary: inside -- the OTOS-first/encoder-fallback POLICY (present() &&
// connected() && poseFresh() -> OTOS; N consecutive stale cycles -> encoder;
// OTOS fresh again -> immediate re-promotion) and the active-source/
// transition-edge bookkeeping; outside -- issuing any bus traffic of its own
// (this class reads Devices::Otos's/Devices::NezhaMotor's OWN cached,
// already-sampled state -- otos_.pose()/poseFresh()/connected()/present()
// and left_.position()/right_.position() -- never otos_.tick() or a motor
// requestSample()/tick() call), and deciding WHAT to do with the chosen
// heading (App::Pilot's job -- the heading PD cascade, dwell completion).
//
// Encoder-fallback formula: (right_.position() - left_.position()) /
// trackWidth_ -- both NezhaMotor::position() values are already fwdSign-
// corrected (nezha_motor.h) and cumulative since this leaf's own encoder
// baseline (re-anchored every boot, same as App::Odometry's own dead-
// reckoning -- see odometry.h's own "Rebaselining note"). This is
// mathematically the same quantity App::Odometry's theta_ accumulates
// internally (BodyKinematics::forward()'s headingDelta summed every
// integrate() call), computed independently here by direct formula rather
// than by depending on Odometry -- this class is a leaf that reads
// Devices::Otos + Devices::NezhaMotor ONLY (src/firm/DESIGN.md's dependency
// diagram: app/ modules may read devices/ directly; there is no requirement
// to route through another app/ module), per sprint.md's own module
// boundary ("Passive reader over OTOS pose + encoder differential ALREADY
// SAMPLED BY THE LOOP").
//
// Sample cadence: sample() is called once per App::Pilot::tick() cycle (the
// same cadence RobotLoop already samples both motors and applyOtosSample()
// -- see pilot.cpp). No caching/rate-limiting of its own is needed here:
// this class is a pure, cheap function of the two leaves' OWN already-
// rate-limited state.
//
// Staleness counting: `kFallbackStaleCycles` consecutive sample() calls
// where OTOS is NOT usable (present() && connected() && poseFresh() all
// true) demote to encoder; ANY single sample() call where OTOS IS usable
// immediately re-promotes (no analogous hysteresis on the recovery side --
// sprint.md's own "re-promote when OTOS recovers" wording, no "N cycles of
// health" qualifier the way the demotion side has one). kFallbackStaleCycles
// is a v1, NOT-bench-tuned constant (mirrors this sprint's own kDeadTime/
// kTerminalDecelWindow precedent in motion/executor.cpp) -- there is no
// hardware bench session behind this specific number; it is a conservative
// "a few tenths of a second at the 40ms cycle" guess, flagged for
// bench revision like every other v1 constant this sprint adds.
#pragma once

#include <cstdint>

#include "devices/nezha_motor.h"
#include "devices/otos.h"
#include "messages/planner.h"

namespace App {

// kFallbackStaleCycles -- see file header. 5 cycles @ ~40ms/cycle = ~200ms
// before an OTOS staleness episode demotes to encoder.
constexpr uint8_t kFallbackStaleCycles = 5;

class HeadingSource {
 public:
  HeadingSource(Devices::Otos& otos, Devices::NezhaMotor& left, Devices::NezhaMotor& right,
                float trackWidth);

  // configure -- applies PlannerConfig.heading_source's per-robot override
  // (planner.proto's HeadingSourceMode): AUTO runs the normal policy;
  // FORCE_OTOS/FORCE_ENCODER pin the active source permanently, skipping
  // the fallback state machine entirely (for a robot with a known-bad OTOS
  // mount, or a bench rig with no OTOS wired at all -- file header/
  // planner.proto's own doc comment). Must be called before the first
  // sample() to take effect from boot; safe to call again later (e.g. a
  // future live-tuning path) -- it only rewrites this leaf's own mode_
  // field, no bus traffic.
  void configure(const msg::PlannerConfig& config);

  // sample -- see file header for cadence/boundary. Re-evaluates the
  // active source for THIS cycle and updates the transition-edge flags
  // (fellBackThisSample()/recoveredThisSample()) -- both are true for AT
  // MOST the one sample() call the transition actually happens on, false
  // every other call (mirrors Telemetry's own event_bits level-set, not
  // sticky-latch, convention -- see telemetry.proto's own doc comment).
  void sample();

  // heading -- the ACTIVE source's current heading estimate: OTOS's own
  // pose().heading when usingOtos(), else the encoder-differential formula
  // above. Cheap accessor, no bus traffic -- just reads whichever leaf's
  // already-cached state sample() last chose.
  float heading() const;  // [rad]

  // usingOtos -- true iff OTOS is the currently-active source (mirrors
  // telemetry.proto's HeadingSourceStatus: usingOtos() == true <->
  // HEADING_SOURCE_STATUS_OTOS).
  bool usingOtos() const { return usingOtos_; }

  // fellBackThisSample / recoveredThisSample -- see sample()'s own comment.
  bool fellBackThisSample() const { return fellBackEdge_; }
  bool recoveredThisSample() const { return recoveredEdge_; }

 private:
  // otosUsable -- present() && connected() && poseFresh(), the AUTO policy's
  // own "is OTOS trustworthy RIGHT NOW" test. Independent of mode_ --
  // FORCE_OTOS/FORCE_ENCODER don't consult this at all (see sample()).
  bool otosUsable() const;

  // encoderHeading -- see file header's "Encoder-fallback formula" note.
  float encoderHeading() const;  // [rad]

  Devices::Otos& otos_;
  Devices::NezhaMotor& left_;
  Devices::NezhaMotor& right_;
  float trackWidth_;  // [mm]

  msg::HeadingSourceMode mode_ = msg::HeadingSourceMode::HEADING_SOURCE_AUTO;

  bool usingOtos_ = true;  // AUTO policy starts optimistic -- see the demotion loop below
  uint8_t staleCount_ = 0;  // consecutive sample() calls with !otosUsable(), AUTO mode only

  bool fellBackEdge_ = false;
  bool recoveredEdge_ = false;
};

}  // namespace App
