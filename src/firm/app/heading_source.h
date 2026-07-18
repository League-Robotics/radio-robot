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
//
// -- 109-010: measurement-age projection (locus 1 of the ticket's own three
// lead-compensation loci) --
// The dominant staleness mechanism is NOT (only) `Devices::Otos::
// kReadPeriod`'s own internal 20ms read-rate limit -- it is App::RobotLoop's
// own CYCLE ORDERING (src/firm/DESIGN.md Sec 3's timing schedule): the OTOS
// burst read (`applyOtosSample()`) happens in the cycle's own LAST
// (`kPace`) block, but App::Pilot::tick() (which reads this class's
// heading()) runs EARLIER, in the motorR settle block -- so on every single
// cycle, Pilot is reading the pose the OTOS chip reported at the END OF THE
// PREVIOUS CYCLE, not "just now". At ~40ms/cycle this is a roughly CONSTANT
// ~one-cycle staleness on every cycle (not merely an occasional skipped
// read) -- during a cruise pivot at ~250-300deg/s, that is ~10-12deg of REAL
// rotation the control loop has not yet been told about (109-009's own
// Impossibility Argument, deferred to this ticket).
//
// `headingLead()` below inverts this: `theta_est = theta_meas + omega_meas *
// age`, where `age` is the REAL elapsed time (Devices::Otos::lastReadUs()
// vs. `nowUs`, the SAME timestamp App::RobotLoop's own clock_.nowMicros()
// already reads every cycle) since the chip's own cached pose was actually
// sampled -- NOT "cycles since sample() last ran" and NOT gated on
// poseFresh() (an earlier draft of this projection tried resetting a
// cycle-counted age to 0 whenever poseFresh() was true and measured NO
// effect at all: applyOtosSample() runs every cycle and is always "due"
// against the 20ms kReadPeriod at a ~40ms cycle rate, so poseFresh() is
// ALWAYS true and that tracker never once saw staleness -- it was measuring
// the wrong mechanism, Otos's own internal read-skip, not the cycle-
// ordering lag that actually dominates). `sample()` therefore takes the
// loop's own `nowUs` directly (a parameter, like App::Pilot::tick()'s
// existing `now` -- no Devices::Clock dependency of this class's own).
// `omega_meas` is OTOS's own angular rate (`otos_.pose().omega`) from the
// SAME burst read `heading()` already consumes -- no new bus traffic.
// `headingLeadBias` (msg::PlannerConfig's own `heading_lead_bias`, [s]) is
// an ADDITIONAL, separately-fitted bias on top of the real measured age --
// see src/firm/motion/DESIGN.md's own "Turn-error characterization" entry
// for the fitted regression this was derived from. The projection is
// deliberately gated on `usingOtos_` -- while on the encoder fallback,
// App::Odometry samples the encoder differential every cycle with no
// analogous cross-cycle read-then-consume ordering gap, so `headingLead()`
// collapses to `heading()` unchanged in that case.
//
// This projected value feeds ONLY the heading-PD's own error term (App::
// Pilot's arithmetic, via Motion::Executor::Twist::thetaMeasLead) -- it is
// a SEPARATE quantity from the raw `heading()` this class already exposed,
// which continues to feed Motion::Executor's own dwell/divergence
// bookkeeping unchanged (this ticket's own "divergence checking stays
// un-led" rule, and the dwell gate's own SEPARATE `terminal_lead`
// locus -- see executor.h/.cpp).
#pragma once

#include <cstdint>

#include "devices/motor.h"
#include "devices/otos.h"
#include "messages/planner.h"

namespace App {

// kFallbackStaleCycles -- see file header. 5 cycles @ ~40ms/cycle = ~200ms
// before an OTOS staleness episode demotes to encoder.
constexpr uint8_t kFallbackStaleCycles = 5;

class HeadingSource {
 public:
  HeadingSource(Devices::Otos& otos, Devices::Motor& left, Devices::Motor& right,
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
  //
  // nowUs -- 109-010: the loop's own current time ([us], the SAME
  // Devices::Clock::nowMicros() reading App::RobotLoop already takes every
  // cycle) -- used ONLY to compute `age = nowUs - otos_.lastReadUs()` for
  // the measurement-age projection (file header's own comment); no bus
  // traffic, no Devices::Clock dependency of this class's own (a parameter,
  // like App::Pilot::tick()'s existing `now`). Defaulted to 0 so no
  // existing test caller (pre-109-010) needs to change -- age() then reads
  // as a (harmless, since headingLead() gates on usingOtos_/is otherwise
  // unused by any pre-110-010 test) large or negative-wrapping value that
  // no pre-existing test ever reads.
  void sample(uint64_t nowUs = 0);  // [us]

  // heading -- the ACTIVE source's current heading estimate: OTOS's own
  // pose().heading when usingOtos(), else the encoder-differential formula
  // above. Cheap accessor, no bus traffic -- just reads whichever leaf's
  // already-cached state sample() last chose.
  float heading() const;  // [rad]

  // headingLead -- 109-010, locus 1: `heading()` projected forward by the
  // measurement-age term (file header's own comment). Equals `heading()`
  // exactly while on the encoder fallback (no rate-limited bus read to be
  // stale against) or immediately after a fresh OTOS read (age == 0).
  float headingLead() const;  // [rad]

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
  Devices::Motor& left_;
  Devices::Motor& right_;
  float trackWidth_;  // [mm]

  msg::HeadingSourceMode mode_ = msg::HeadingSourceMode::HEADING_SOURCE_AUTO;

  bool usingOtos_ = true;  // AUTO policy starts optimistic -- see the demotion loop below
  uint8_t staleCount_ = 0;  // consecutive sample() calls with !otosUsable(), AUTO mode only

  bool fellBackEdge_ = false;
  bool recoveredEdge_ = false;

  // -- 109-010: measurement-age projection (locus 1), see file header --
  float headingLeadBias_ = 0.0f;  // [s] msg::PlannerConfig.heading_lead_bias
  float ageS_ = 0.0f;              // [s] nowUs - otos_.lastReadUs(), clamped >= 0
};

}  // namespace App
