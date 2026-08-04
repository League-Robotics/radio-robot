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
//        Stage B (fast)      -- a small-authority PID (p + i + kaff*a_cmd,
//                                clamped ±pidMax) on wheel-speed error.
//                                Because Stage A+C already carry all DC
//                                content, its integrator idles near zero
//                                -- the condition under which a loop near
//                                stiction does not stick-slip (the
//                                documented 2-3 Hz duty-domain limit
//                                cycle this design avoids by construction,
//                                not by tuning). Reset on estop(). Skipped
//                                OUTRIGHT (output forced 0, integrator
//                                untouched) at commanded-zero, and its
//                                integrator only accumulates on a fresh/
//                                connected/non-frozen measurement -- same
//                                shape as the steady gate below (131-002,
//                                see tick()'s own doc comment, drive.cpp).
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
//      for the full computation and sprint 131 Design Rationale Decision 4
//      / its Revision for why the planner does not gain its own vMin
//      awareness this ticket. A sustained large error
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
// takeover() vs estop() (131-001, sprint 131 Design Rationale Decisions 1/2
// -- closes the sprint-130 midpoint review's finding #1 / post-130 review
// F5): before this ticket, RobotLoop::handleMove() called estop() to make
// the planner take over motion, and estop() (130-004) ALSO zeroed Stage
// B's integrators, Stage C's bias, and the deficit latch -- so every
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
#pragma once

#include <cstdint>

#include "config/robot.h"
#include "devices/motor.h"
#include "firm/types/robot_state.h"

namespace App {

class Drive {
 public:
  // left/right -- the two drive-wheel NezhaMotor leaves, in BodyKinematics'
  // own L/R convention. trackWidth -- [mm], BodyKinematics::inverse()/
  // forward()'s own `b` parameter: Drive does no kinematics of its own, but
  // holds and exposes this value because RobotLoop::publishPose() needs the
  // SAME number to fuse the two leaves' measured velocities into the
  // telemetry twist, and Drive is where it has always been constructed.
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

  // Install this robot's own wheel calibration
  // (command-ingestion-ring-buffered-comms-subsystem-routing-two-stops.md
  // §6). Drive carries NO calibration defaults, so this MUST be called --
  // by the composition root -- before any motion is commanded; until it is,
  // tick() writes nothing (see calibrated() below).
  void setDutyPerSpeed(float left, float right) {  // [duty/(mm/s)] x2
    dutyPerSpeedLeft_ = left;
    dutyPerSpeedRight_ = right;
    calibrated_ = left != 0.0f && right != 0.0f;
  }

  // Live observability of the installed conversion scale (130-005, issue
  // 04's folded-in observability mandate -- "ships with the feature, not
  // after it") -- wired into the wire Telemetry frame / TestGUI so the
  // baked calibration this robot is actually running is visible without a
  // firmware rebuild, same reasoning as biasLeft()/Right()/pidLeft()/
  // Right() below.
  float dutyPerSpeedLeft() const { return dutyPerSpeedLeft_; }    // [duty/(mm/s)]
  float dutyPerSpeedRight() const { return dutyPerSpeedRight_; }  // [duty/(mm/s)]

  // Commanded->actual correction, per wheel per direction of approach
  // (docs/design/wheel-speed-command-mapping.md). Drive inverts the measured
  // line to seed the feedforward. gain 1 / intercept 0 = no correction.
  void setWheelCorrection(float gainLeftAccel, float interceptLeftAccel,
                          float gainLeftDecel, float interceptLeftDecel,
                          float gainRightAccel, float interceptRightAccel,
                          float gainRightDecel, float interceptRightDecel);

  // Crawl-mode pulse amplitude; 0 disables (per-robot breakaway property,
  // and the shipped default -- see Config::DriveBootConfig's own doc
  // comment for why 0.20 was wrong).
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
  // 131-002: two further gates apply to this stage regardless of the
  // gain values configured here -- a commanded-zero wheel skips
  // fastPid() OUTRIGHT (no P/feedforward/clamp math runs, integrator
  // untouched), and the integrator only ACCUMULATES on a fresh/
  // connected/non-frozen measurement (the same conjunct Stage C's
  // adaptBias() already required) -- a stale/disconnected/frozen
  // reading freezes it exactly like the existing steady gate, rather
  // than winding against a manufactured zero-velocity reading. See
  // tick()'s own doc comment (drive.cpp) for the full rationale.
  struct ControlGains {
    float kp = 0.0f;      // [1] dimensionless: mm/s of PID output per mm/s of error
    float ki = 0.0f;      // [1/s]
    float iMax = 0.0f;    // [mm/s] integrator clamp; 0 disables integration
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

  // Stage C / deficit-flag policy's generated-constant bounds (parameter
  // table rows 4/6/7/13) -- population-measured (ticket 001) speed floor
  // and adaptation authority, plus the design-chosen adaptation time
  // constant/steady-gate/deficit thresholds. `tauAdapt <= 0` OR
  // `aSteady <= 0` disables Stage C's adaptation outright (bias stays
  // wherever estop()/the boot default left it, the same "0 = off"
  // convention crawlPulse_/ControlGains::iMax/pidMax already use);
  // `deficitThreshold <= 0` OR `deficitWindow <= 0` disables the
  // deficit-flag policy outright.
  struct AdaptationBounds {
    float vMin = 0.0f;              // [mm/s] speed floor (Open Question 2)
    float biasMax = 0.0f;           // [mm/s] Stage C trim authority clamp
    float tauAdapt = 0.0f;          // [s] Stage C adaptation time constant; <=0 disables
    float aSteady = 0.0f;           // [mm/s^2] |a_cmd| below this counts as steady
    float deficitThreshold = 0.0f;  // [mm/s] sustained error magnitude that flags a deficit
    float deficitWindow = 0.0f;     // [ms] how long the deficit condition must sustain
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

  // False until setDutyPerSpeed() has landed a real (nonzero) pair. An
  // uncalibrated Drive REFUSES to drive: tick() writes no duty at all,
  // rather than quietly running this robot's wheels on some other robot's
  // numbers. Same fail-closed posture RobotLoop's own `configured_` gate
  // already takes for motion commands.
  bool calibrated() const { return calibrated_; }

  // --- The WHEELS command lifecycle ---

  // Arm a bounded wheel command: velocity targets + an expiry deadline +
  // the id acked on completion (takeCompletion()). `duration` is REQUIRED
  // and positive -- a wheel command is always time-bounded, so a dead host
  // can never mean a runaway. Supersedes any command already armed; the
  // superseded one does NOT emit a completion event (the caller that
  // replaced it already knows, and RobotLoop acked its arrival).
  void command(float vLeft, float vRight, float duration, uint32_t moveId,
               uint32_t now);  // [mm/s] [mm/s] [ms] -- now [ms]

  // Ownership handover: another subsystem is about to command the wheels
  // (RobotLoop::handleMove(), the moment a MOVE is accepted -- 131-001).
  // Zero the targets and disarm the WHEELS command, exactly like estop()'s
  // own "zero targets, disarm" half -- but deliberately leave Stage B's
  // integrators (pidIntegralLeft_/Right_), Stage C's bias (biasLeft_/
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
  // every learned/latched control-loop value (Stage B's integrators, Stage
  // C's bias, the deficit latch) and the stop re-assertion window (this
  // file's own LOAD-BEARING comment above). Reserved for the ESTOP wire
  // verb (RobotLoop::handleEstop()) and genuine panic paths -- NOT the
  // ownership-handover path (a MOVE taking over from WHEELS, or vice
  // versa), which uses takeover() instead so a converged Stage C bias
  // survives across legs of a chained tour (131-001, this file's own
  // header).
  void estop();

  // True while an armed command is running -- i.e. while Drive, not the
  // planner, owns motion.
  bool owns() const { return commandActive_; }

  // One-shot completion event for a command that reached its deadline; the
  // loop acks it. False (and *moveId untouched) when there is none pending.
  bool takeCompletion(uint32_t* moveId);

  // --- The two-method contract ---

  // Convert the commanded wheel speeds to duty -- Stage A (calibration +
  // bias) -> Stage B (fast PID) -> Stage C (bias adaptation, computed
  // this tick for NEXT tick's Stage A) -> Stage D (crawl shaping ->
  // quiet-at-zero) -- and write the leaves. Reads state.wheelLeft/
  // Right.cmdVelocity/cmdAccel/velocity/sampleTime/connected rather than
  // its own targets: the loop hands it whichever subsystem's targets the
  // blackboard currently carries, so there is one actuation path
  // regardless of who decided the motion. See this file's own header
  // for the full per-stage algorithm.
  void tick(const Types::RobotState& state);

  // Expire an armed command whose deadline has passed (latching the
  // completion event), then publish this subsystem's targets into
  // state.wheelLeft/Right.cmdVelocity -- but ONLY while Drive owns motion.
  // When the planner owns it, this is a no-op on the blackboard, so the
  // planner's own update() is left as the single writer. Must therefore run
  // AFTER Motion::Planner::update() in the cycle.
  void update(Types::RobotState& state, uint32_t now);  // [ms]

  // Last-staged velocity targets (test observability; the blackboard's own
  // copy is written by update()).
  float targetLeft() const { return targetLeft_; }    // [mm/s] signed
  float targetRight() const { return targetRight_; }  // [mm/s] signed

  // trackWidth -- read-only accessor; fixed at construction, matching
  // Drive's own "no live-reconfigure" contract. See the constructor's own
  // doc comment for who reads it and why it lives here.
  float trackWidth() const { return trackWidth_; }  // [mm]

 private:
  // Invert the measured line for one wheel: the command whose ACTUAL
  // result is `desired`, PLUS `bias` (Stage C's adapted parameter) --
  // see drive.cpp's own doc comment for why `bias` must be folded in
  // here rather than added at the call site (the "stop is stop" guard
  // below must cover the bias term too, or a nonzero adapted bias would
  // creep a commanded-zero wheel). `previous` picks the accel/decel
  // branch.
  float correctedCommand(float desired, float previous, bool leftWheel,
                         float bias) const;

  // Correction table [wheel][direction]: 0 = left/right, 0 = accel/decel.
  float corrGain_[2][2] = {{1.0f, 1.0f}, {1.0f, 1.0f}};
  float corrIntercept_[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
  // Speed last converted per wheel -- the accel/decel discriminator.
  float lastSpeedLeft_ = 0.0f;   // [mm/s]
  float lastSpeedRight_ = 0.0f;  // [mm/s]

  // Crawl shaper (drive.cpp).
  float crawlDuty(float duty, float& carry) const;

  // --- Stage B/C: the wheel-speed controller (130-004; see this file's
  // own header for the full algorithm) ---

  // Stage B's per-wheel PID computation. `integral` is the caller's own
  // persistent state (pidIntegralLeft_/Right_ below), mutated in place.
  // `steady` gates the integrator exactly like Motion::WheelTrim's own
  // Hold-phase gate (wheel_trim.cpp) -- see drive.cpp's own doc comment.
  // 131-002: the caller folds this wheel's freshness into `steady` too
  // (steady && fresh), so a stale/disconnected/frozen reading freezes the
  // integrator through this SAME parameter -- fastPid() itself knows
  // nothing about freshness, only "may I integrate right now." tick()'s
  // own commanded-zero guard skips calling this function entirely rather
  // than passing a flag through it -- see tick()'s own doc comment.
  float fastPid(float& integral, float err, float aCmd, float dt,
               bool steady) const;

  // Stage C's bounded bias adaptation -- mutates `bias` in place.
  // `vCmdMagnitude` is the (already speed-floored) |commanded speed|,
  // for the vMin gate. See drive.cpp's own doc comment for the full
  // steady/floor/freshness gate.
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
  void applySpeedFloor(float rawLeft, float rawRight, float& speedLeft,
                       float& speedRight) const;

  // Sustained-condition latch for the deficit-flag policy -- mutates
  // `since`/`latched` in place. See drive.cpp's own doc comment.
  void updateDeficit(bool conditionNow, uint32_t now, uint32_t& since,
                     bool& latched) const;

  // Guards the same now<sampleTime clock-domain edge Telemetry::ageOf()
  // guards (telemetry.cpp) -- returns 0 rather than an underflowed huge
  // value.
  uint32_t sampleAge(uint32_t now, uint32_t sampleTime) const;

  ControlGains gains_;
  AdaptationBounds bounds_;

  float pidIntegralLeft_ = 0.0f;   // [mm/s] Stage B integrator state
  float pidIntegralRight_ = 0.0f;  // [mm/s]
  float lastPidLeft_ = 0.0f;       // [mm/s] observability: last-computed Stage B output
  float lastPidRight_ = 0.0f;      // [mm/s]

  float biasLeft_ = 0.0f;   // [mm/s] Stage C's ONE adapted parameter, per wheel
  float biasRight_ = 0.0f;  // [mm/s]

  // Deficit-flag sustained-condition tracking (0 == not currently
  // accumulating; see updateDeficit()'s own doc comment, drive.cpp).
  uint32_t deficitSinceLeft_ = 0;   // [ms]
  uint32_t deficitSinceRight_ = 0;  // [ms]
  bool deficitLeft_ = false;
  bool deficitRight_ = false;

  // Wheel-measurement freshness gate for Stage C (see adaptBias()'s own
  // doc comment, drive.cpp) -- generous relative to one control period
  // (~47-50ms) plus the brick's own per-wheel skew, so normal per-cycle
  // jitter never trips it; a genuinely stale/disconnected reading does.
  static constexpr uint32_t kMaxSampleAge = 200;  // [ms]

  Devices::Motor& left_;
  Devices::Motor& right_;
  float trackWidth_;  // [mm]

  float targetLeft_ = 0.0f;   // [mm/s]
  float targetRight_ = 0.0f;  // [mm/s]

  // Armed bounded command (command() -> update() expiry).
  bool commandActive_ = false;
  uint32_t commandDeadline_ = 0;  // [ms]
  uint32_t commandMoveId_ = 0;
  bool completionPending_ = false;
  uint32_t completedMoveId_ = 0;

  // Open-loop duty per commanded speed, per wheel. NO DEFAULTS (§6): zero
  // until the composition root installs this robot's own measured pair via
  // setDutyPerSpeed(), and zero means uncalibrated, which means tick()
  // refuses to write. Baking a value here is what made one robot's
  // gearboxes every robot's -- see Config::DriveBootConfig's own doc
  // comment (config/boot_config.h) for the full history.
  float dutyPerSpeedLeft_ = 0.0f;   // [duty/(mm/s)]
  float dutyPerSpeedRight_ = 0.0f;  // [duty/(mm/s)]
  bool calibrated_ = false;

  float crawlPulse_ = 0.0f;  // [-1, 1] pulse amplitude; 0 = off
  float crawlCarryLeft_ = 0.0f;   // Bresenham accumulators
  float crawlCarryRight_ = 0.0f;

  // Last duty pair actually written (quiet-at-zero baseline).
  float writtenLeft_ = 0.0f;   // [-1, 1]
  float writtenRight_ = 0.0f;  // [-1, 1]

  // Stop re-assertion (see this file's own header) -- counts down once per
  // tick() call from kStopEnforceTicks after estop() arms it; tick()
  // bypasses the quiet-at-zero shortcut while this is nonzero OR either
  // wheel still measures above kRestVelocity, so a commanded stop keeps
  // being handed to the leaves instead of being trusted after one write.
  uint8_t stopEnforceCountdown_ = 0;

  // 30 cycles at RobotLoop::kCycle(50ms, 130-007: was 40ms) == 1.5s --
  // comfortably past the <=0.15s measured stop-observed bound, without
  // holding the re-assertion open indefinitely (App cannot reference
  // App::RobotLoop::kCycle directly here without a layer cycle, so this
  // is a plain literal, same as NezhaMotor's own kMinWriteIntervalUs
  // comment coupling).
  static constexpr uint8_t kStopEnforceTicks = 30;

  // Wheel-at-rest threshold for the unconditional half of the re-assertion
  // window. NOT shared with MotorArmor's own kRestVelocity (motor_armor.h)
  // or NezhaMotor's kStopConfirmVelocity (nezha_motor.h) -- each is a
  // leaf/subsystem-local constant for its own guard, per this project's
  // established pattern (see nezha_motor.h's kReconfigureRestVelocity
  // comment).
  static constexpr float kRestVelocity = 8.0f;  // [mm/s]
};

}  // namespace App
