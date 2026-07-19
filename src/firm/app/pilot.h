// pilot.h -- App::Pilot: bridges Motion::Executor into RobotLoop's own
// cycle. Sprint 109 ticket 003's own "loop glue" module (sprint.md's
// Architecture section, Step 3 responsibility group 3): decides WHEN
// Executor solves/samples happen, never HOW (that is entirely Motion::
// Executor/JerkTrajectory's job) or WHAT the result means on the wire
// (that is RobotLoop::handleMove()'s job). 109-005 additionally makes
// Pilot the one place the heading PD cascade's own ARITHMETIC runs (see
// tick()'s own comment) -- App::HeadingSource decides WHICH sensor is
// truth; Motion::Executor decides the feedforward/reference; Pilot
// combines them into the final commanded omega, per sprint.md's own SUC-002
// flow ("Each cycle, Pilot::tick() computes omega_cmd = omega_ff +
// heading_kp*(...)").
//
// Boundary: inside -- calling Executor::plan()/tick()/enqueue()/flush()/
// popEvent() at the right cycle points, sampling HeadingSource/Odometry and
// forwarding their readings into Executor::tick(), the heading PD
// arithmetic itself, and staging the combined twist onto Drive; outside --
// decoding a wire Move into a Motion::Cmd (RobotLoop/Comms's job, via
// Motion::fromMove()), turning a CompletionEvent into a wire ack
// (RobotLoop's job, via Telemetry's ack ring), and the HeadingSource
// fallback POLICY itself (App::HeadingSource's own job -- Pilot only reads
// heading()/usingOtos()/the transition edges, never decides them).
//
// Cycle placement (src/firm/DESIGN.md Sec 3/sprint.md's own table):
// `plan()` is called from the `kPace` budget block (all Ruckig solves
// happen there, <=1/cycle -- Executor::plan()'s own contract);
// `tick(now)` is called from the motorR settle block, AFTER
// processMessage()/the deadman check and BEFORE `drive_.tick()`, so a
// same-cycle enqueue/flush is reflected in this cycle's own staged twist.
// `tick()` only stages a twist (via Drive::setTwist()) while
// `state() != Motion::State::kIdle` -- while IDLE it does nothing at all,
// deliberately, so a same-cycle raw TWIST (handleTwist()'s own
// Drive::setTwist() call, which always also calls flush() first -- see
// this file's own flush() doc comment) is never immediately clobbered by
// a stale Executor sample. 111-003 extends this contract for the ONE case
// still uncovered by it: a natural running->idle transition happening
// INSIDE this tick() call (the command completes on this exact call, not
// a same-cycle flush -- distinguished by sampling `state()` both before
// and after the internal `executor_.tick()` call). On that transition
// `tick()` stages `Drive::setTwist(0, 0)` exactly once, so Drive stops
// commanding the PREVIOUS cycle's stale twist instead of creeping until
// the deadman lease (robot_loop.cpp's `kPilotDeadmanLease`, ~300ms) force-
// stops it. A same-cycle flush is unaffected: RobotLoop::handleTwist()/
// handleStop() call Pilot::flush() BEFORE Pilot::tick() runs this same
// cycle, so the "before" state sampled at the top of tick() is ALREADY
// kIdle by the time this tick() call sees it -- the "just completed"
// branch is naturally never taken, and the raw command's own twist
// (already staged by handleTwist()/handleStop()) survives untouched.
// HeadingSource::sample() is called every tick(), IDLE included --
// App::HeadingSource is a passive reader with no per-tick cost (no bus
// traffic of its own), and keeping its active-source/fallback state
// current even while idle means a fallback that happens between commands
// is still visible in telemetry.
//
// No bus traffic, no sleeps, no Clock/Sleeper dependency of its own --
// `tick(now)` takes the loop's own `now` (matching Telemetry::emit(now)'s
// existing pattern) and derives its own internal dt from consecutive
// calls, rather than reaching for a Devices::Clock the way Deadman does;
// Pilot has no need to read "now" for any purpose beyond that one delta.
#pragma once

#include <cstdint>

#include "app/drive.h"
#include "app/heading_source.h"
#include "app/odometry.h"
#include "messages/config.h"
#include "messages/planner.h"
#include "motion/cmd.h"
#include "motion/executor.h"

namespace App {

// kDistanceTrimCeiling -- 112-003: the bounded linear position-feedback
// trim's own clamp ceiling. A fixed, Pilot-local C++ constant, deliberately
// NOT a wire field (only the GAIN, distance_kp, is per-robot tunable --
// see planner.proto's own field comment and this ticket's completion
// notes) -- declared at namespace scope (mirroring Motion::kDeadTime/
// kQueueDepth's own "other code/tests can reference the SAME constant by
// name" shape) so pilot.cpp's tick() and a test harness verifying the
// 087-009 guardrail read one shared value, never a duplicated literal.
//
// 50.0mm/s: comfortably ABOVE the shipped distance_kp*distance_tol product
// (15.0 * 3.0mm = 45.0mm/s, see planner.proto's own distance_kp field
// comment for the full deadband-inequality derivation) -- an in-tolerance
// residual error is NOT yet clipped, matching the heading PD's own
// unclamped-near-target shape -- while staying far below any velocity that
// could look like a solve-side reversal (typical cruise speeds run
// 100-300mm/s; v_body_max defaults to 1000mm/s): a genuinely large
// divergence (e.g. the 40mm gross-divergence reanchor threshold,
// Motion::kDivergenceReanchorLinearMm) would demand distance_kp*40mm =
// 600mm/s if unclamped -- this ceiling caps that to a 50mm/s nudge, never
// a re-plan-scale correction (sprint 112 Architecture "Guardrails/SUC-007"
// Main Flow step 3: "the new linear feedback trim's authority is bounded
// (clamped) such that near the target its magnitude... stays below what a
// full re-solve-style reversal would need -- a residual-error nudge,
// never a re-plan").
constexpr float kDistanceTrimCeiling = 50.0f;  // [mm/s]

class Pilot {
 public:
  Pilot(Motion::Executor& executor, Drive& drive, HeadingSource& headingSource, Odometry& odom)
      : executor_(executor), drive_(drive), headingSource_(headingSource), odom_(odom) {}

  // configureHeading -- 109-005: the heading PD cascade's own gains
  // (msg::PlannerConfig.heading_kp/heading_kd), read here rather than
  // inside Motion::Executor -- see this file's own header comment for why
  // the PD arithmetic itself lives in Pilot. Must be called before the
  // first tick() that carries heading content to take effect (matches
  // Executor::configure()'s/HeadingSource::configure()'s own "call before
  // first use" contract).
  //
  // 109-008: ALSO stores the whole `config` as `plannerConfig_`, the live
  // baseline `applyPlannerPatch()` merges future wire patches onto. main.cpp
  // calls this once at boot with `Config::defaultPlannerConfig()` -- that
  // boot call is what seeds `plannerConfig_` for the lifetime of the robot.
  void configureHeading(const msg::PlannerConfig& config) {
    headingKp_ = config.heading_kp;
    headingKd_ = config.heading_kd;
    minSpeed_ = config.min_speed;   // [mm/s] heading-PD minimum-command floor (see tick())
    distanceKp_ = config.distance_kp;  // [1/s] 112-003: linear position-feedback trim's own gain (see tick())
    plannerConfig_ = config;
  }

  // applyPlannerPatch -- 109-008: un-stubs `RobotLoop::handleConfig`'s
  // PLANNER arm. Merges `patch` onto `plannerConfig_` (the last config
  // applied via `configureHeading()`, boot default or a previous live
  // patch) -- only `Opt<T>` fields PRESENT in `patch` overwrite the
  // baseline (the SAME merge-then-write shape `RobotLoop::handleConfig`
  // already uses for `MotorConfigPatch` gains and `OtosConfigPatch`'s
  // offset triple) -- then re-applies the merged config to every live
  // consumer (`Executor::configure()`, `HeadingSource::configure()`, this
  // class's own `configureHeading()`) so the change takes effect
  // immediately, with no reflash. `Executor::configure()`/`HeadingSource::
  // configure()` are both pure config-limit setters (no queue/state
  // mutation -- see their own definitions), so calling them mid-motion is
  // safe.
  //
  // Note: `PlannerConfigPatch` (config.proto) curates 5 of `msg::
  // PlannerConfig`'s fields: `min_speed`/`heading_kp`/`heading_kd`/
  // `arrive_dwell` plus (112-003) `distance_kp` -- added to this curated
  // set (not left boot-only) so the linear trim's own gain can be bench-
  // iterated live, the same reason `heading_kp` is curated; a STATED
  // decision, not an oversight (this ticket's own completion notes). It
  // does NOT declare `a_max`/`a_decel`/`v_body_max`/`yaw_rate_max`/
  // `yaw_acc_max`/`j_max`/`yaw_jerk_max`/`heading_source`/
  // `heading_dwell_tol`/`heading_dwell_rate`/`heading_lead_bias`/
  // `plan_lead`/`terminal_lead`/`actuation_lag`, nor (112-003)
  // `distance_tol` -- deliberately left boot-config-only for this ticket,
  // no live consumer to tune yet (config.proto's own PlannerConfigPatch
  // header comment) -- so those fields are never touched by this method
  // and stay at whatever `plannerConfig_` already holds (the boot
  // default, absent a future wire-schema addition). `PlannerConfigPatch`
  // used to also curate 16 Drive::Limits/tracker/policy fields
  // (`v_wheel_max`..`arrive_vel_tol`, ticket 006) that were never wired to
  // any live consumer -- removed as dead in 111-004 (step 7 of the
  // terminal-blips-close-the-loop fix plan); see config.proto's own
  // PlannerConfigPatch header comment for the full accounting.
  void applyPlannerPatch(const msg::PlannerConfigPatch& patch) {
    msg::PlannerConfig merged = plannerConfig_;
    if (patch.min_speed.has) merged.min_speed = patch.min_speed.val;
    if (patch.heading_kp.has) merged.heading_kp = patch.heading_kp.val;
    if (patch.heading_kd.has) merged.heading_kd = patch.heading_kd.val;
    if (patch.arrive_dwell.has) merged.arrive_dwell = patch.arrive_dwell.val;
    if (patch.distance_kp.has) merged.distance_kp = patch.distance_kp.val;  // 112-003

    executor_.configure(merged);
    headingSource_.configure(merged);
    configureHeading(merged);  // stores merged as plannerConfig_ too
  }

  // enqueue -- forwards to Executor::enqueue(); see executor.h for the
  // classification/outcome contract. RobotLoop::handleMove() is the only
  // caller, and is the one that turns the outcome into a wire ack.
  Motion::EnqueueOutcome enqueue(const Motion::Cmd& cmd) { return executor_.enqueue(cmd); }

  // flush -- TWIST/STOP preemption. RobotLoop::handleTwist()/handleStop()
  // both call this BEFORE (or alongside) their own existing
  // Drive::setTwist()/stop() call, so this cycle's tick() (which runs
  // after processMessage() in the schedule) sees state()==kIdle and does
  // not restage a twist over the raw command's own.
  void flush() { executor_.flush(); }

  // plan -- see this file's own cycle-placement doc comment. At most one
  // JerkTrajectory solve (Executor::plan()'s own contract).
  void plan() { executor_.plan(); }

  // tick -- samples HeadingSource/Odometry, forwards their readings into
  // Executor::tick(dt, measuredDistance, measuredHeading), computes the
  // heading PD cascade on top of the executor's own feedforward when
  // Twist::headingActive is true, and (while running) stages the combined
  // twist onto Drive via setTwist(). dt is derived from consecutive `now`
  // values ([ms], the loop's own markTime()); the very first call after
  // construction has no prior `now` to diff against and contributes dt=0
  // (a single zero-length sample -- harmless, JerkTrajectory::sample(0)
  // just returns the seed state). On a natural running->idle transition
  // happening INSIDE this call, stages a zero twist exactly once instead
  // of leaving Drive holding the previous cycle's stale twist -- see this
  // file's own header comment for the full contract (111-003) and why a
  // same-cycle flush is unaffected.
  //
  // The PD cascade itself: `omega = twist.omega [omega_ff] +
  // heading_kp*(twist.thetaRef - twist.thetaMeas) + heading_kd*(twist.
  // omegaDes - omegaMeasEst)`, where omegaMeasEst is THIS class's own
  // finite-difference estimate of the measured heading's rate (thetaMeas
  // deltas across consecutive tick() calls) -- Executor cannot supply this
  // itself because it spans two tick() calls and Executor's own per-
  // command dwell-rate estimate is a SEPARATE, internal computation (see
  // executor.h's own comment) serving a different purpose (completion, not
  // the PD term). Both estimates use the same finite-difference METHOD on
  // the same thetaMeas sequence, just kept in two independent state
  // variables (Executor's own prevThetaMeasRel_, this class's own
  // prevThetaMeas_) -- not a duplicated bug, a deliberate non-coupling
  // between "when is this command done" and "what should the PD command
  // right now" so neither concern's own bookkeeping leaks into the other's
  // class.
  // nowUs -- 109-010: the SAME instant as `now` ([ms]), expressed in [us]
  // (App::RobotLoop's own clock_.nowMicros(), the same clock markTime()
  // itself derives `now` from) -- forwarded to headingSource_.sample(nowUs)
  // for its own measurement-age projection (App::HeadingSource's own doc
  // comment). This does NOT give Pilot a Devices::Clock dependency of its
  // own -- both `now` and `nowUs` are plain parameters the caller (already
  // holding a Clock) supplies, the same shape as `now` already was.
  void tick(uint32_t now, uint64_t nowUs);  // [ms] [us]

  // popEvent -- drains one pending completion event. RobotLoop drains all
  // pending events each cycle (bounded: the ring holds at most
  // Motion::kEventRingDepth) into Telemetry's ack ring.
  bool popEvent(Motion::CompletionEvent* out) { return executor_.popEvent(out); }

  // plannerConfig -- read-only access to the live PlannerConfig baseline
  // (109-008): the last config `configureHeading()`/`applyPlannerPatch()`
  // applied (boot default, then merged with every live PlannerConfigPatch
  // since). Used by `RobotLoop::handleConfig` callers/tests that need to
  // observe a live-patched value directly (Executor/HeadingSource keep no
  // single queryable copy of their own -- see `applyPlannerPatch()`'s doc
  // comment).
  const msg::PlannerConfig& plannerConfig() const { return plannerConfig_; }

  uint8_t queueDepth() const { return executor_.queueDepth(); }
  uint32_t activeId() const { return executor_.activeId(); }
  Motion::State state() const { return executor_.state(); }

  // HeadingSource visibility -- RobotLoop::updateTlm() reads these to
  // populate Telemetry::Frame::headingSourceIsOtos/headingSourceFellBack/
  // headingSourceRecovered (109-005, SUC-004).
  bool headingSourceIsOtos() const { return headingSource_.usingOtos(); }
  bool headingSourceFellBack() const { return headingSource_.fellBackThisSample(); }
  bool headingSourceRecovered() const { return headingSource_.recoveredThisSample(); }

  // refLeft/refRight -- 112-002: the PLANNED per-wheel reference, i.e.
  // Motion::Executor's own jerk-limited trajectory (`twist.v`/`twist.omega`
  // BEFORE this method's heading-PD correction and BEFORE App::Drive's own
  // actuation-lag feedforward are added on top), mapped through the SAME
  // `BodyKinematics::inverse()` map `tick()` below and `Drive::tick()` both
  // use. Recomputed every `tick()` call (0/0 while idle, since `twist` is
  // `Twist{}` then). Exists purely as a diagnostic/verification seam (see
  // this method's own `tick()` doc comment) -- no production consumer reads
  // it; a genuine wire telemetry field would need `RobotLoop::updateTlm()`
  // to populate it, and this sprint's own guardrail (SUC-007) keeps
  // `robot_loop.cpp` untouched, so this stays a live accessor instead,
  // mirroring `TestSim::SimHarness::driveTargetVelLeft/Right()`'s own
  // established "test-only, non-wire" shape for the analogous COMMANDED
  // signal.
  float refLeft() const { return refLeft_; }    // [mm/s] signed
  float refRight() const { return refRight_; }  // [mm/s] signed

 private:
  Motion::Executor& executor_;
  Drive& drive_;
  HeadingSource& headingSource_;
  Odometry& odom_;

  bool hasLastTick_ = false;
  uint32_t lastTick_ = 0;  // [ms]

  float headingKp_ = 0.0f;  // [1/s] msg::PlannerConfig.heading_kp
  float headingKd_ = 0.0f;  // dimensionless msg::PlannerConfig.heading_kd
  float minSpeed_ = 0.0f;   // [mm/s] msg::PlannerConfig.min_speed -- the heading
  // PD's minimum-command floor (tick()): the smallest per-wheel speed that
  // actually moves the plant (the write shaping's output deadband eats duty
  // below ~0.03, and real motors have stiction besides). 0 disables the
  // floor. Adopted for this purpose 2026-07-18 -- the field's previous
  // consumer (the old Drive tracker's pivot-mode threshold) was deleted
  // with source/drive/, leaving it consumer-less.

  float distanceKp_ = 0.0f;  // [1/s] msg::PlannerConfig.distance_kp -- 112-003
  // linear position-feedback trim's own gain (tick()): `distanceKp_ *
  // (twist.sRef - twist.sMeas)`, clamped to kDistanceTrimCeiling, mirrors
  // headingKp_ above exactly. 0 makes the trim a genuine no-op (matches
  // this field's own zero-value default -- the sim harness/any caller
  // that never configures this leaves the new mechanism completely
  // inert, unchanged behavior).

  // 109-008: the live PlannerConfig baseline applyPlannerPatch() merges
  // future wire patches onto -- see configureHeading()'s own doc comment.
  msg::PlannerConfig plannerConfig_ = {};

  // Finite-difference measured-heading-rate bookkeeping for the PD term's
  // own omegaMeas -- see tick()'s own doc comment for why this is separate
  // from Executor's own internal dwell-rate estimate.
  bool hasPrevThetaMeas_ = false;
  float prevThetaMeas_ = 0.0f;  // [rad]

  // refLeft_/refRight_ -- 112-002: this tick()'s own PLANNED per-wheel
  // reference. See refLeft()/refRight()'s own doc comment above.
  float refLeft_ = 0.0f;   // [mm/s] signed
  float refRight_ = 0.0f;  // [mm/s] signed
};

}  // namespace App
