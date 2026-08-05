// drive.h -- App::Drive: the wheel-drive subsystem, the owner of the
// WHEELS command's whole lifecycle, AND (130-004, wheel-speed-controller-
// moves-into-drive.md -- DECIDED, stakeholder 2026-08-01) the ONE unified
// wheel-speed controller every cmdVelocity writer shares. Responsibilities:
//
//   1. The bounded wheel command (command-ingestion-ring-buffered-comms-
//      subsystem-routing-two-stops.md §4). `WHEELS` is the dumb teleop
//      primitive -- a per-wheel velocity pair held for a fixed duration --
//      and Drive owns its targets, its deadline, and its completion event.
//      Before this change those four pieces of state lived as loose members
//      on App::RobotLoop and were overwritten in place by each arriving
//      command, so a superseded command never completed and never acked.
//   2. Actuation: commanded wheel SPEED -> motor duty, through the
//      three-timescale controller below, then the crawl shaper and the
//      leaf writes. EVERY writer (WHEELS teleop and a planner Move alike)
//      goes through the SAME tick() -- no privileged or degraded path.
//   3. The controller itself, at three timescales (parameter table and
//      full derivation: wheel-speed-controller-moves-into-drive.md Phase
//      2). `correctedCommand()`/tick() carry the per-tick detail; this is
//      the shape:
//        Stage A (offline)   -- the calibrated conversion map (the
//                                existing per-wheel/per-direction affine
//                                correction, corrGain_/corrIntercept_),
//                                corrected by Stage C's bias:
//                                v_corrected = map(v_cmd) + bias;
//                                dutyFF = dutyPerSpeed * v_corrected. The
//                                map's SLOPE is never adapted online.
//        Stage B (fast)      -- a small-authority controller
//                                (p + i + kaff*a_cmd, clamped ±pidMax)
//                                whose P and feedforward terms read
//                                wheel-speed error and whose I term reads
//                                POSITION (133-002 -- see the "THE I TERM
//                                IS A POSITION TERM" note below, and
//                                fastPid()/positionError() in drive.cpp).
//                                Because Stage A+C already carry all DC
//                                content, the I term idles near zero
//                                -- the condition under which a loop near
//                                stiction does not stick-slip (the
//                                documented 2-3 Hz duty-domain limit
//                                cycle this design avoids by construction,
//                                not by tuning). Its position reference is
//                                re-anchored on estop(). Skipped OUTRIGHT
//                                (output forced 0) at commanded-zero.
//
// THE I TERM IS A POSITION TERM (133-002; measured on vevov 2026-08-03,
// clasi/issues/B-wheel-controller-position-loop-and-tuning.md §3). Stage B
// used to accumulate `integral += ki * err * dt` -- and `err` is a
// VELOCITY, so `err * dt` is MILLIMETRES. The accumulator was already
// (commanded position - measured position); it just built its measured
// half by summing a DERIVED, QUANTIZED velocity (sd 8.0 mm/s, one whole
// duty count) instead of reading the encoder's own position register,
// with a freshness gate that FROZE the sum on exactly the samples where
// real distance was being travelled unobserved. `positionError()` reads
// the register instead. Same control law, same units (ki stays [1/s]),
// same output -- the loop still commands velocity, which is all the motor
// takes. What it DELETES is the accumulator: no windup, no `steady` gate
// to freeze, no reset to lose. A position error that appears while the
// loop is not looking is simply still there when it looks again. Measured
// effect: L/R distance imbalance 1.05-1.07 -> 1.000-1.006, and note the
// direction -- MORE position gain gave BETTER balance, the opposite of
// how every velocity-side knob behaved.
//
// posErrMax AND iMax ARE BOTH LOAD-BEARING, IN DIFFERENT DOMAINS. Do not
// collapse them; confusing the two IS the original defect:
//   AdaptationBounds::posErrMax  [mm]    bounds the INPUT  (position
//                                        error, BEFORE ki)
//   ControlGains::iMax           [mm/s]  bounds the OUTPUT (ki*posError,
//                                        AFTER it)
// Before 133-002 only iMax existed, and because the accumulated quantity
// was already millimetres it acted as a DISGUISED position limit of
// iMax/ki mm -- at ki=6/iMax=20 that is 3.3 mm, smaller than every
// residual being chased, so RAISING ki SHRANK the loop's position memory
// (which is why the ki sweep plateaued between 2 and 6). Clamping the
// input in millimetres is what lets iMax finally be the velocity-
// authority bound it was always named for. Delete iMax and nothing bounds
// how much speed the I term may demand.
//        Stage C (slow)      -- bounded adaptation of ONE parameter, the
//                                map's additive `bias` -- NEVER the
//                                slope. Runs only when steady
//                                (|a_cmd| < aSteady), at/above the speed
//                                floor (vMin), and on a fresh, connected,
//                                non-frozen measurement. tauAdapt-slow
//                                (design tens of seconds), clamped to
//                                ±biasMax (the population-measured
//                                spread, ticket 001), reset on estop() --
//                                see setAdaptationBounds()'s own doc
//                                comment for the full fail-closed
//                                contract (issue 04's safety intent,
//                                folded in and superseded here; sprint
//                                130 Architecture Design Rationale
//                                Decision 3).
//        Stage D (unchanged) -- the crawl shaper / quiet-at-zero
//                                baseline below; the breakaway dead zone
//                                stays feedforward-owned, never
//                                rediscovered by the adaptive bias.
//      Wheels-solid contract enforcement: a sub-vMin nonzero wheel PAIR is
//      scaled UP so its dominant (larger-magnitude) wheel reaches vMin
//      (applySpeedFloor(), Open Question 2 -- matches the project's own
//      established boost-to-breakaway fix rather than silently stalling
//      the wheel). 131-003, REVISED post-shipment (issue
//      A-speed-floor-snaps-the-planner-differential.md; sprint 131
//      sprint.md's "Revision -- Ticket 003's speed-floor semantics" has the
//      full history): the floor is a RATIO-PRESERVING SCALE applied to the
//      raw wheel pair directly, not a common-mode/differential
//      decomposition -- `dominantMag = max(|cmdVelocityLeft|,
//      |cmdVelocityRight|)`; if nonzero and below vMin, BOTH wheels are
//      scaled by `vMin / dominantMag`, preserving their exact commanded
//      ratio, so the dominant wheel lands at exactly vMin; otherwise the
//      raw pair passes through unchanged. The originally-shipped
//      common-mode-only floor (Design Rationale Decision 4, now
//      superseded) never engaged during a pure pivot -- `Planner::
//      planWheels()`'s ratio-locked output for an Angle Move has an
//      EXACTLY zero common mode by construction, so the entire turn's
//      commanded magnitude passed through unfloored and measurably
//      regressed turn accuracy. The ratio-preserving scale fixes that
//      (a symmetric pivot scales both wheels to vMin identically, matching
//      the pre-131-003 per-wheel-independent floor) while still leaving an
//      already-above-floor differential trim (Planner::applyHeadingHold(),
//      the one reachable real-world case) untouched, because its dominant
//      wheel already clears vMin. See tick()'s own doc comment (drive.cpp)
//      for the full computation. 134-002: the floor is a TELEOP
//      affordance and runs ONLY while Drive owns motion (owns(), below) --
//      a planner-owned pair reaches actuation exactly as it was profiled,
//      because a decelerating tail and a terminal alignment nudge are
//      small ON PURPOSE and boosting them to vMin destroys the terminal
//      authority they exist to deliver. That resolves sprint 131 Design
//      Rationale Decision 4's deferral without the planner gaining any
//      vMin awareness of its own; applySpeedFloor()'s own doc comment
//      (drive.cpp) carries the bench measurement behind it and the
//      deadman-expiry decision. A sustained large error
//      while BOTH bias and the fast PID sit pinned at their configured
//      authority raises a deficit-flag policy (deficitLeft()/Right()) --
//      the robot runs slow, loudly, never silently. `Motion::Planner`
//      sheds ALL wheel-actuation code (Motion::WheelTrim, its own former
//      home) -- see wheel_trim.h's own header, parked pending ticket
//      005's deletion; Motion::Planner publishes cmdVelocity/cmdAccel
//      only, from here on.
//
// LOAD-BEARING (the 2026-07-31 runaway): estop() re-asserts its commanded
// zero for kStopEnforceTicks cycles, and unconditionally for as long as
// either wheel still measures above kRestVelocity, rather than trusting a
// single write. This is Drive's own mirror of NezhaMotor's stopNotTaken
// write-on-change exemption (nezha_motor.h/.cpp) -- a stop is asserted
// until it is OBSERVED (the encoder actually reads at rest), not until it
// is merely sent once. Do not fold this back into a single unconditional
// write.
//
// LOAD-BEARING (the 2026-08-03 runaway, 133-001): that window is now armed
// by tick() as well, on the nonzero->zero transition of the commanded duty
// pair -- not only by estop(). Both of the pre-133 defences (this window's
// `wheelsMoving` half, and NezhaMotor::writeRawDuty()'s own re-issue) gate
// on the ENCODER, so both are disarmed by the same condition: a wheel that
// reports at rest. Measured on vevov, 16/16: a stop issued once by a host
// that then went quiet produced 936mm of continued travel with no decay,
// and estop() failed 5 of 6 attempts. Arming on what was COMMANDED, with
// no measured velocity in the condition, is what makes the re-assertion
// independent of an encoder that may be wedged, stale, or manufacturing a
// zero. Do not re-gate this arm on velocity.
//
// takeover() vs estop() (131-001, sprint 131 Design Rationale Decisions 1/2
// -- closes the sprint-130 midpoint review's finding #1 / post-130 review
// F5): before this ticket, RobotLoop::handleMove() called estop() to make
// the planner take over motion, and estop() (130-004) ALSO zeroed Stage
// B's I-term state, Stage C's bias, and the deficit latch -- so every
// accepted MOVE cold-started adaptation, and Stage C (tauAdapt tens of
// seconds) could never converge on the path it exists to serve. takeover()
// is the ownership-handover verb: zero targets, disarm WHEELS -- KEEP
// everything Stage B/C learned, because the plant did not change, only the
// writer did. estop() stays the full-reset safety verb, reserved for the
// ESTOP wire verb and genuine panic paths, and is now implemented ON TOP
// OF takeover() (the shared "zero targets, disarm" part), adding only the
// learned-state reset and the stop-reassertion window on top. Two
// distinctly-named methods, not one estop(bool resetLearnedState) --
// Motion::Planner's own tick()/update() two-method contract already
// established this project's preference for named verbs over parameterized
// ones at this exact architectural layer (Decision 1).
//
// This split makes a converged bias long-lived across MOVE boundaries for
// the first time, which in turn makes correctedCommand()'s bias
// application direction-sensitive for the first time: a bias learned under
// sustained forward motion must still HELP (not hurt) a subsequent reverse
// command. correctedCommand() applies `bias` as a magnitude-domain
// correction that follows the CURRENT commanded direction (`desired`'s
// sign), not a fixed additive term -- see its own doc comment (drive.cpp)
// and Decision 2. This preserves 130-004's one-bias-per-wheel model (no
// per-direction split: the physical droop this trim corrects is a
// property of the wheel's instantaneous load, not which side of a ramp it
// arrived from) while making the correction help regardless of direction.
//
// Two-method contract, adopted from Motion::Planner (planner.h) so the two
// motion deciders read the same way at their call sites:
//   tick(const Types::RobotState&)   -- DO THE WORK: shape and write duty,
//                                       reading whichever subsystem's
//                                       cmdVelocity/cmdAccel the blackboard
//                                       currently carries (130-003: the
//                                       signature widened from two loose
//                                       speed floats to the whole state so
//                                       the forthcoming unified controller
//                                       -- ticket 004 -- can also see
//                                       cmdAccel and each wheel's measured
//                                       sampleTime/freshness, without a
//                                       second signature change).
//   update(Types::RobotState&, now)  -- SAVE results into the blackboard:
//                                       expire the armed command and
//                                       publish this subsystem's targets,
//                                       but only while Drive OWNS motion.
//
// Exactly one subsystem owns motion at a time; that exclusivity is enforced
// at routing (App::RobotLoop::routeCommand -- a WHEELS command clears the
// planner, a MOVE clears Drive's armed command), and `owns()` below is how
// the loop reads which one it currently is.
//
// THE cmdVelocity OWNERSHIP INVARIANT (133-001 -- stated in full, with its
// history, at App::RobotLoop::publishWheels(), robot_loop.cpp):
//
//   cmdVelocity has exactly one DECIDER per cycle -- Motion::Planner::
//   update() or App::Drive::update() -- and exactly one SAFETY ARBITER,
//   App::RobotLoop, whose writes are restricted to zero, which runs after
//   every decider and before actuation, and which supersedes all deciders.
//   No other writer exists.
//
// What this means for Drive specifically: update() publishes ONLY while
// Drive owns motion, and stops publishing entirely the cycle after its
// command expires (`if (!owned) return;`). That silence is correct and
// stays correct -- it is not Drive's job to keep asserting zero forever.
// Covering the gap it leaves is the arbiter's job
// (App::RobotLoop::zeroUnownedMotion(), which derives idleness from
// owns() below rather than being handed ownership). Do NOT "fix" this by
// making update() publish unconditionally: that would make Drive a
// permanent decider and re-open the two-decider conflict routing exists to
// prevent.
#pragma once

#include <cstdint>

#include "config/robot.h"
#include "devices/motor.h"
#include "firm/types/robot_state.h"

namespace App {

class Drive {
 public:
  Drive(Devices::Motor& left, Devices::Motor& right, float trackWidth);

  // The measured plant inverse: duty = speed * kDutyPerSpeed.
  //
  // HISTORICAL / REFERENCE ONLY as of 132-009 -- see below for the
  // reversal. Measured by src/tests/bench/duty_sweep.py on the stand at
  // firmware v0.20260731.13: 853.6 mm/s per unit duty on the left wheel,
  // 837.8 on the right -- 1.9% apart -- corroborated by a saturation
  // reading (696-795 mm/s at full duty) that depends on no constant at
  // all. 1/845.7 = 0.001182.
  //
  // ONE constant for both wheels on purpose. At 1.9% the two wheels are the
  // same wheel, and a per-wheel pair invites fitting one against the other --
  // which is exactly how duty_per_speed and wheel_gain became circularly
  // calibrated, each measured against the other's error (see
  // `_wheel_correction_note` in data/robots/tovez.json).
  //
  // BAKED IN, NOT CONFIGURED (stakeholder, 2026-07-31) -- REVERSED (132-009,
  // the-configuration-object.md / .claude/rules/configuration-discipline.md,
  // stakeholder 2026-08-03: "every value the robot uses comes from the
  // file"). From 2026-07-31 until 132-009, `Configurator::install()` read
  // this constant directly and ignored config_.drive.duty_per_speed_left/
  // right entirely -- ONE hardcoded literal applied identically to EVERY
  // robot (tovez/togov/tovez_nocal) regardless of which JSON was baked,
  // which is itself the exact anti-pattern the "NO DEFAULTS" policy on
  // dutyPerSpeedLeft_/Right_ below exists to prevent ("baking a value here
  // is what made one robot's gearboxes every robot's"). 132-009 retargets
  // `Configurator::install()` to read config_.drive.duty_per_speed_left/
  // right instead -- the ONE-population-scale-value-plus-gain/intercept-
  // absorbs-all-deviation design the-configuration-object.md settled on
  // structurally prevents the SAME circular-fit risk the 07-31 decision was
  // guarding against (duty_per_speed's two fields are measured together and
  // kept equal by convention, never re-fit against wheel_gain residuals),
  // without requiring a value the robot boots with to live outside the
  // file. data/robots/tovez.json/tovez_nocal.json's own duty_per_speed_left/
  // right were corrected from a stale 0.00187325 (the ~1.6x error
  // the-configuration-object.md's Cause section cites) to 0.001182 as part
  // of the same ticket, so this is a behavior-preserving source change, not
  // a new calibration decision -- see Configurator::install()'s own doc
  // comment (configurator.cpp) for the full writeup. This constant is KEPT,
  // unread by production boot, as the documented measurement the file's
  // value traces to and as duty_sweep.py's own constant-free cross-check
  // anchor.
  //
  // The robot JSONs still carry duty_per_speed_left/right and the generator
  // still bakes them into Config::DriveBootConfig/Config::Robot; nothing
  // ignores those fields as of 132-009. clasi/issues/04-continuous-duty-
  // per-speed-calibration.md's own further step -- making this value a
  // boot-time starting ESTIMATE for runtime adaptation, rather than a fixed
  // constant for the whole session -- remains open, unaddressed by this
  // reversal.
  static constexpr float kDutyPerSpeed = 0.001182f;  // [duty/(mm/s)]

  void setDutyPerSpeed(float left, float right) {  // [duty/(mm/s)] x2
    dutyPerSpeedLeft_ = left;
    dutyPerSpeedRight_ = right;
    calibrated_ = left != 0.0f && right != 0.0f;
  }

  float dutyPerSpeedLeft() const { return dutyPerSpeedLeft_; }  // [duty/(mm/s)]
  float dutyPerSpeedRight() const { return dutyPerSpeedRight_; }  // [duty/(mm/s)]

  void setWheelCorrection(float gainLeftAccel, float interceptLeftAccel,
                          float gainLeftDecel, float interceptLeftDecel,
                          float gainRightAccel, float interceptRightAccel,
                          float gainRightDecel, float interceptRightDecel);

  void setCrawlPulse(float crawlPulse) { crawlPulse_ = crawlPulse; }

  // Stage B's wire-tunable gains (wheel-speed-controller-moves-into-
  // drive.md's parameter table, rows 8-12) -- fail-closed default (every
  // field 0) produces exactly zero PID contribution, the SAME "all-zero
  // gains, exactly zero correction" contract Motion::WheelTrim's own
  // header documents (wheel_trim.h) and this ticket's own testing
  // requirement carries forward. Shipped at all-zero on every robot as
  // of 130-004 (Open Question 4): the profiler's own every-tick
  // re-plan-from-measured-position may already supply equivalent
  // correction at the planning layer, and this stage's live bench
  // tuning is ticket 006's job, on hardware -- not a judgment call this
  // ticket can make with tovez hard-silent.
  //
  // 131-002: a commanded-zero wheel skips fastPid() OUTRIGHT (no
  // P/feedforward/clamp math runs) -- "stop is stop" through Stage B
  // too. 133-002 removed the OTHER gate this comment used to describe
  // (the fresh/connected/non-frozen conjunct that froze the
  // accumulator): there is no accumulator left to freeze, and the
  // hazard it guarded -- winding against a MANUFACTURED ZERO VELOCITY
  // on a failed encoder collect -- cannot reach the I term any more,
  // because the I term no longer reads velocity. The equivalent guards
  // for the position domain live in positionError() itself (drive.cpp)
  // and are about a manufactured zero POSITION instead. See tick()'s own
  // doc comment for the full rationale.
  struct ControlGains {
    float kp = 0.0f;      // [1] dimensionless: mm/s of PID output per mm/s of error
    float ki = 0.0f;      // [1/s]
    // iMax bounds the I term's OUTPUT, in mm/s -- how much wheel speed
    // the position error is allowed to demand. Its INPUT-side sibling is
    // AdaptationBounds::posErrMax [mm]; the two clamp different domains
    // and both are load-bearing (this file's own header). 0 = the I term
    // is OFF ENTIRELY, unchanged from the accumulator era's "0 disables
    // integration" -- deliberately NOT re-read as "unclamped", which
    // would silently hand every already-shipped robot JSON (all of which
    // carry iMax = 0) unbounded I authority the first time a nonzero ki
    // is pushed.
    float iMax = 0.0f;    // [mm/s] I-term output clamp; 0 disables the I term
    float kaff = 0.0f;    // [s] accel feedforward ~= the plant time constant
    // Total fast-loop authority; 0 disables the clamp. Also gates the
    // deficit-flag policy below (updateDeficit()'s own doc comment,
    // drive.cpp): with pidMax == 0, the fast PID can never register as
    // "saturated," so the deficit flag never raises regardless of
    // deficitThreshold/deficitWindow -- both stages activate together.
    float pidMax = 0.0f;  // [mm/s]
  };
  void setControlGains(const ControlGains& gains) { gains_ = gains; }
  const ControlGains& controlGains() const { return gains_; }

  struct AdaptationBounds {
    float vMin = 0.0f;              // [mm/s] speed floor (Open Question 2)
    float biasMax = 0.0f;           // [mm/s] Stage C trim authority clamp
    float tauAdapt = 0.0f;          // [s] Stage C adaptation time constant; <=0 disables
    float aSteady = 0.0f;           // [mm/s^2] |a_cmd| below this counts as steady
    // posErrMax bounds Stage B's I term INPUT -- how far behind its own
    // reference a wheel may be counted as being, so a blocked or
    // slipping wheel cannot bank unbounded catch-up debt and then sprint
    // to repay it the moment it comes free. Its OUTPUT-side sibling is
    // ControlGains::iMax [mm/s]; both are load-bearing and neither
    // replaces the other (this file's own header). 0 = unclamped, the
    // same "0 = off" convention every sibling here uses -- the I term's
    // authority is still bounded by iMax and pidMax in that case.
    float posErrMax = 0.0f;         // [mm] Stage B position-error clamp; 0 = unclamped
    float deficitThreshold = 0.0f;  // [mm/s] sustained error magnitude that flags a deficit
    float deficitWindow = 0.0f;  // [ms] how long the deficit condition must sustain
  };
  void setAdaptationBounds(const AdaptationBounds& bounds) { bounds_ = bounds; }
  const AdaptationBounds& adaptationBounds() const { return bounds_; }

  // configure -- the-configuration-object.md's "subsystems take the
  // whole object and pull out what they want" entry point (132-007):
  // nothing outside Drive needs to know what Drive reads out of
  // Config::Robot. A thin pull-and-forward onto the setters above --
  // setWheelCorrection() (Stage A, config.drive.wheel_*),
  // setControlGains()/setAdaptationBounds() (Stage B/C,
  // config.wheelControl.*), setCrawlPulse() (config.drive.crawl_pulse).
  // Deliberately does NOT touch trackWidth_ (no post-construction setter
  // exists -- constructor injection stays, boot_wiring.cpp's own
  // bakeBootValues()) or dutyPerSpeed. dutyPerSpeed stays BOOT-ONLY by
  // design even after 132-009's reversal of kDutyPerSpeed's own C++-literal
  // sourcing (see that constant's doc comment, above, for the full
  // writeup) -- the wire's DRIVE group never carried it and still does
  // not; Configurator::install() (configurator.cpp) calls
  // setDutyPerSpeed(config.drive.duty_per_speed_left/right) itself,
  // separately from this method, at boot only.
  void configure(const Config::Robot& config);

  // --- Stage B/C observability (wired into the wire Telemetry frame /
  // TestGUI by 130-005, per issue 04's own folded-in observability
  // mandate -- App::Telemetry::update() reads these directly) ---
  float biasLeft() const { return biasLeft_; }      // [mm/s] Stage C's adapted parameter
  float biasRight() const { return biasRight_; }    // [mm/s]
  float pidLeft() const { return lastPidLeft_; }    // [mm/s] last-computed Stage B output
  float pidRight() const { return lastPidRight_; }  // [mm/s]
  bool deficitLeft() const { return deficitLeft_; }
  bool deficitRight() const { return deficitRight_; }

  bool calibrated() const { return calibrated_; }


  void command(float vLeft, float vRight, float duration, uint32_t moveId,
               uint32_t now);  // [mm/s] [mm/s] [ms] -- now [ms]

  // Ownership handover: another subsystem is about to command the wheels
  // (RobotLoop::handleMove(), the moment a MOVE is accepted -- 131-001).
  // Zero the targets and disarm the WHEELS command, exactly like estop()'s
  // own "zero targets, disarm" half -- but deliberately leave Stage B's
  // position references (posRefLeft_/Right_), Stage C's bias (biasLeft_/
  // Right_), the deficit latch (deficitSinceLeft_/Right_, deficitLeft_/
  // Right_), and the stop re-assertion countdown (stopEnforceCountdown_)
  // completely untouched: the plant did not change, only the writer did,
  // and there is no "verify the wheels actually reached rest" safety
  // concern the way there is for a genuine estop -- the new owner is about
  // to command motion again next cycle. See this file's own header
  // (Decisions 1/2) for the full rationale, and estop()'s own doc comment
  // for the contrasting full-reset verb.
  void takeover();

  // Halt now: zero the targets, disarm, and emit NO completion ack for the
  // discarded command (the ESTOP path -- the host asked for a stop, not for
  // a report that the thing it cancelled finished) -- PLUS a full reset of
  // every learned/latched control-loop value (Stage B's position
  // references, Stage C's bias, the deficit latch) and the stop
  // re-assertion window (this
  // file's own LOAD-BEARING comment above). Reserved for the ESTOP wire
  // verb (RobotLoop::handleEstop()) and genuine panic paths -- NOT the
  // ownership-handover path (a MOVE taking over from WHEELS, or vice
  // versa), which uses takeover() instead so a converged Stage C bias
  // survives across legs of a chained tour (131-001, this file's own
  // header).
  void estop();

  // True while an armed command is running -- i.e. while Drive, not the
  // planner, owns motion. Read by App::RobotLoop::zeroUnownedMotion() (this
  // file's own header) and, since 134-002, by applySpeedFloor() itself --
  // the speed floor is a teleop affordance and this is its gate.
  bool owns() const { return commandActive_; }

  bool takeCompletion(uint32_t* moveId);


  void tick(const Types::RobotState& state);

  // Live override for Stage B's position-error clamp -- the DEVELOPMENT
  // tuning path (133-003's `DBG:pos <mm>` verb is the one caller;
  // .claude/rules/configuration-discipline.md binds PRODUCTION BOOT, not
  // bench tuning, precisely because a value pushed ad hoc can be read
  // back). RAM-only: a reboot restores the robot JSON's own
  // wheel_control.pos_err_max through the generated boot config, which is
  // also what the live WHEEL_CONTROL wire push writes -- one file, both
  // paths. Clamped non-negative; 0 = unclamped.
  void setPositionErrorMax(float posErrMax) {  // [mm]
    bounds_.posErrMax = (posErrMax > 0.0f) ? posErrMax : 0.0f;
  }

  // Live override for the speed floor -- same DEVELOPMENT-tuning contract
  // as setPositionErrorMax() above (133-003's `DBG:vmin <mm/s>` verb is the
  // one caller; RAM-only, boot restores wheel_control.v_min from the robot
  // JSON). Clamped non-negative; 0 disables the floor entirely, the same
  // "0 = off" convention every sibling in AdaptationBounds uses.
  void setSpeedFloor(float vMin) {  // [mm/s]
    bounds_.vMin = (vMin > 0.0f) ? vMin : 0.0f;
  }

  // Live override for the steady gate -- same contract again (133-003's
  // `DBG:asteady <mm/s^2>` verb; RAM-only, boot restores
  // wheel_control.a_steady). This gates Stage C's bias adaptation
  // (`|a_cmd| < aSteady`); as of 133-002 it no longer gates Stage B, whose
  // I term reads position and therefore has no accumulator left to freeze.
  // Clamped non-negative; <= 0 still disables Stage C's adaptation
  // outright, unchanged from the boot path.
  void setASteady(float aSteady) {  // [mm/s^2]
    bounds_.aSteady = (aSteady > 0.0f) ? aSteady : 0.0f;
  }

  // Expire an armed command whose deadline has passed (latching the
  // completion event), then publish this subsystem's targets into
  // state.wheelLeft/Right.cmdVelocity -- but ONLY while Drive owns motion.
  // When the planner owns it, this is a no-op on the blackboard, so the
  // planner's own update() is left as the single writer. Must therefore run
  // AFTER Motion::Planner::update() in the cycle.
  void update(Types::RobotState& state, uint32_t now);  // [ms]

  float targetLeft() const { return targetLeft_; }  // [mm/s] signed
  float targetRight() const { return targetRight_; }  // [mm/s] signed

  float trackWidth() const { return trackWidth_; }  // [mm]

 private:
  float correctedCommand(float desired, float previous, bool leftWheel,
                         float bias) const;

  float corrGain_[2][2] = {{1.0f, 1.0f}, {1.0f, 1.0f}};
  float corrIntercept_[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
  float lastSpeedLeft_ = 0.0f;  // [mm/s]
  float lastSpeedRight_ = 0.0f;  // [mm/s]

  // Commanded-accel estimate for the WHEELS path (update(), drive.cpp) --
  // 133-002, UNVALIDATED, see update()'s own comment at the write site.
  // Motion::Planner publishes its own cmdAccel; Drive never did, which
  // left Stage B's feedforward multiplying an identically-zero field and
  // `steady` pinned true for every WHEELS ramp -- so both the kff and the
  // aSteady sweeps measured nothing, twice. Smoothed with a first-order
  // filter because the host re-arms WHEELS on a slower tick than kCycle,
  // making the raw command a STAIRCASE whose bare finite difference
  // alternates a double-size spike with a zero.
  static constexpr float kAccelSmoothing = 0.35f;  // [1] first-order weight, per cycle
  float previousTargetLeft_ = 0.0f;   // [mm/s] last cycle's published target
  float previousTargetRight_ = 0.0f;  // [mm/s]
  float cmdAccelLeft_ = 0.0f;         // [mm/s^2] smoothed
  float cmdAccelRight_ = 0.0f;        // [mm/s^2]

  // Crawl shaper (drive.cpp).
  float crawlDuty(float duty, float& carry) const;


  // Stage B's per-wheel position reference (133-002). `reference` is the
  // integral of the COMMANDED speed since `origin` was anchored -- our
  // own signal, noise-free -- and the error the I term consumes is
  // `reference - (Wheel::position - origin)`. `armed` stays false until a
  // live, same-epoch reading has anchored it, so the very first cycle
  // after an anchor reports exactly zero error rather than a step.
  //
  // A struct rather than four loose members per wheel because all four
  // fields are re-anchored together, atomically, or not at all --
  // `PositionRef{}` is the whole reset (estop(), drive.cpp).
  struct PositionRef {
    float reference = 0.0f;  // [mm] integral of commanded speed since the anchor
    float origin = 0.0f;     // [mm] Wheel::position when anchored
    uint8_t epoch = 0;       // Wheel::positionEpoch when anchored
    bool armed = false;
  };

  // Stage B's per-wheel controller output. `posError` is millimetres of
  // position error (positionError(), below); `err` is mm/s of velocity
  // error, which only the P term reads; `aCmd` is mm/s^2 of commanded
  // accel for the feedforward. No `dt` and no `steady` parameter: the I
  // term is a direct function of the current position error, not an
  // accumulation, so there is nothing per-cycle to scale and nothing to
  // gate off (133-002 -- this file's own header). tick()'s own
  // commanded-zero guard skips calling this function entirely rather than
  // passing a flag through it -- see tick()'s own doc comment.
  float fastPid(float posError, float err, float aCmd) const;  // [mm] [mm/s] [mm/s^2]

  // Advance `ref` by this cycle's commanded travel and return the
  // resulting position error, clamped to ±bounds_.posErrMax. Re-anchors
  // (returning EXACTLY zero, correcting nothing) whenever the reference
  // cannot be trusted -- see drive.cpp's own doc comment for the three
  // conditions and why each one earns its place.
  float positionError(float speed, const Types::RobotState::Wheel& wheel,
                      PositionRef& ref, float dt) const;  // [mm/s] [s] -> [mm]

  void adaptBias(float& bias, float err, float aCmd, float vCmdMagnitude,
                bool fresh, float dt) const;

  // Wheels-solid speed floor (Open Question 2, resolved; REVISED 131-003 --
  // see this file's own header). A ratio-preserving scale on the raw wheel
  // pair, written into speedLeft/speedRight: if the dominant (larger-
  // magnitude) of rawLeft/rawRight is nonzero and below vMin, both wheels
  // are scaled by the same factor so the dominant wheel lands at exactly
  // vMin; otherwise the pair passes through unchanged. A wheel raw-commanded
  // exactly 0.0f is exactly 0.0f afterward regardless of scaling
  // (`0 * scale == 0`), so "stop is stop" holds without a special case.
  // 134-002: a complete no-op unless owns() -- the floor is a teleop
  // affordance, never applied to a planner-shaped command.
  void applySpeedFloor(float rawLeft, float rawRight, float& speedLeft,
                       float& speedRight) const;

  void updateDeficit(bool conditionNow, uint32_t now, uint32_t& since,
                     bool& latched) const;

  uint32_t sampleAge(uint32_t now, uint32_t sampleTime) const;

  ControlGains gains_;
  AdaptationBounds bounds_;

  // Stage B's I-term state (133-002): a position REFERENCE, not an
  // accumulated correction. Replaces the old pidIntegralLeft_/Right_
  // accumulators outright -- see this file's own header for why the
  // accumulator was always a position term computed the worst possible
  // way. `mutable` because tick() is const-correct with respect to the
  // blackboard it reads, not with respect to this reference, which it
  // must advance once per cycle.
  mutable PositionRef posRefLeft_;
  mutable PositionRef posRefRight_;
  float lastPidLeft_ = 0.0f;       // [mm/s] observability: last-computed Stage B output
  float lastPidRight_ = 0.0f;      // [mm/s]

  float biasLeft_ = 0.0f;  // [mm/s] Stage C's ONE adapted parameter, per wheel
  float biasRight_ = 0.0f;  // [mm/s]

  uint32_t deficitSinceLeft_ = 0;  // [ms]
  uint32_t deficitSinceRight_ = 0;  // [ms]
  bool deficitLeft_ = false;
  bool deficitRight_ = false;

  static constexpr uint32_t kMaxSampleAge = 200;  // [ms]

  Devices::Motor& left_;
  Devices::Motor& right_;
  float trackWidth_;  // [mm]

  float targetLeft_ = 0.0f;  // [mm/s]
  float targetRight_ = 0.0f;  // [mm/s]

  bool commandActive_ = false;
  uint32_t commandDeadline_ = 0;  // [ms]
  uint32_t commandMoveId_ = 0;
  bool completionPending_ = false;
  uint32_t completedMoveId_ = 0;

  float dutyPerSpeedLeft_ = 0.0f;  // [duty/(mm/s)]
  float dutyPerSpeedRight_ = 0.0f;  // [duty/(mm/s)]
  bool calibrated_ = false;

  float crawlPulse_ = 0.0f;  // [-1, 1] pulse amplitude; 0 = off
  float crawlCarryLeft_ = 0.0f;
  float crawlCarryRight_ = 0.0f;

  float writtenLeft_ = 0.0f;  // [-1, 1]
  float writtenRight_ = 0.0f;  // [-1, 1]

  // Stop re-assertion (see this file's own header) -- counts down once per
  // tick() call from kStopEnforceTicks; tick() bypasses the quiet-at-zero
  // shortcut while this is nonzero OR either wheel still measures above
  // kRestVelocity, so a commanded stop keeps being handed to the leaves
  // instead of being trusted after one write.
  //
  // TWO arming sites (133-001 added the second): estop(), and tick() itself
  // on the nonzero->zero transition of the commanded duty pair. The second
  // is the one that survives a wheel reporting at rest -- see tick()'s own
  // comment at that site for why an encoder-gated arm is not enough.
  uint8_t stopEnforceCountdown_ = 0;

  static constexpr uint8_t kStopEnforceTicks = 30;

  static constexpr float kRestVelocity = 8.0f;  // [mm/s]
};

}
