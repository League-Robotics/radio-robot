// sim_harness.h -- TestSim::SimHarness: the composition root wiring the REAL
// App::RobotLoop firmware graph to a SimPlant (ticket 108-002,
// tests/_infra/sim/sim_plant.{h,cpp}).
//
// Sprint 108 ticket 003 (clasi/issues/plan-pure-i2cbus-clock-interfaces-a-
// real-simplant-simulator.md, Stage 2 part b). Supersedes
// tests/sim/support/sim_api.{h,cpp} (TestSim::SimApi + its DutyPredictor),
// deleted by this same ticket -- SimApi's own predictor GUESSED what
// firmware would write to a scripted-FIFO bus from a duty-write count, and
// under an arbitrary twist stream the guess and the firmware's real write
// sequence could drift apart (the divergence bug this whole sprint exists to
// fix). SimHarness instead composes the real App::RobotLoop against a REAL
// I2CBus implementation (SimPlant) that just parses whatever bytes firmware
// actually put on the wire -- there is no prediction left to desync.
//
// This class is a THIN composition root -- it contains no simulation logic
// (that lives entirely in SimPlant/WheelPlant/OtosPlant) and no firmware
// dispatch logic (that lives entirely in App::RobotLoop, unmodified).
// Modeled directly on how source/main.cpp constructs the same graph (and,
// before it, on sim_api.cpp's own construction order), substituting only the
// I2CBus& slot: main.cpp passes a Devices::MicroBitI2CBus, this class passes
// a TestSim::SimPlant.
//
// --- The one invariant that matters: tick the plant BEFORE the loop reads
// it, every cycle ---
// step(n) calls plant_.tick(dt) FIRST, then robotLoop_.cycle() -- never the
// other order. A cycle's own I2C reads (motor encoder collect, OTOS
// position/velocity burst) must observe THIS cycle's physics, not the
// physics left over from the previous cycle -- ticking after cycle() would
// make every read one cycle stale, silently shifting the whole simulated
// actuation-lag curve by one cycle period and defeating the entire point of
// SimPlant's live-response design. See SimPlant::tick()'s own doc comment
// (sim_plant.h) for the physics-side half of this contract.
//
// --- Two entry points, matching App::RobotLoop's own boot()/cycle() split
// (robot_loop.h) ---
//   boot()   -- drives App::Preamble to done() via preamble_.step() calls
//               issued directly (NOT via a single robotLoop_.boot() call,
//               whose own while(!preamble_.done()) loop is one synchronous
//               C++ call with no opportunity to advance the fake Clock in
//               between -- color_/line_'s own retry pacing needs REAL
//               elapsed virtual time between attempts, see
//               driveBootToDone()'s own comment below), then calls
//               robotLoop_.boot() itself once preamble_.done() is already
//               true (a real call, still exercising its own
//               setEvent(kEventBootReady, true) tail).
//   step(n)  -- n cycles of (plant_.tick(dt); clock_.advanceMicros(dt);
//               robotLoop_.cycle()). Call boot() first.
// Unlike the deleted SimApi::step(), boot and cycling are two SEPARATE
// entry points here, not one overloaded step() -- SimPlant's own live
// responses mean there is no scripting step to hide inside a first step()
// call, so there is no reason to conflate the two.
//
// --- Command injection / telemetry drain ---
// Reuses tests/sim/support/fake_transport.h (TestSupport::FakeTransport) and
// tests/sim/support/wire_test_codec.{h,cpp} (TestSupport::armorTwistCommand/
// armorStopCommand/decodeOutboundLine) UNMODIFIED -- this class does not
// reinvent wire injection or telemetry decoding, exactly as this ticket's
// implementation plan requires.
//
// --- True pose ---
// trueX()/trueY()/trueHeading() read SimPlant's owned OtosPlant ground
// truth (x()/y()/heading()) directly -- NOT the wire-visible
// reportedX()/reportedY()/reportedHeading() (which apply the OTOS
// drift/bias fault knob) and NOT App::Odometry's own independently-
// integrated pose (which a test would otherwise have no way to read without
// adding accessors to Odometry itself). These are ground truth, bypassing
// any sensor noise, for test assertions.
#pragma once

#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

#include "app/comms.h"
#include "app/deadman.h"
#include "app/drive.h"
#include "app/heading_source.h"
#include "app/odometry.h"
#include "app/pilot.h"
#include "app/preamble.h"
#include "app/robot_loop.h"
#include "app/telemetry.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/line_sensor.h"
#include "devices/motor_armor.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"
#include "fake_transport.h"
#include "motion/executor.h"
#include "sim_clock.h"
#include "sim_plant.h"
#include "wire_test_codec.h"

namespace TestSim {

class SimHarness {
 public:
  // trackWidth: [mm] -- passed to BOTH the SimPlant's own OtosPlant and the
  // real App::Drive/App::Odometry instances constructed here, so the
  // simulated OTOS chip and the firmware's own independently-integrated
  // odometry describe the same physical wheelbase (see otos_plant.h's own
  // "MUST match" comment on this exact requirement). Defaults to
  // TestSim::kDefaultTrackWidth (sim_plant.h) -- SimPlant's own default,
  // NOT the unrelated 130mm sim_api.cpp used, which never had to agree with
  // anything else since the old scripted bus carried no plant of its own.
  explicit SimHarness(float trackWidth = kDefaultTrackWidth)
      : plant_(trackWidth),
        motorL_(plant_, Devices::MotorConfig{}),
        motorR_(plant_, Devices::MotorConfig{}),
        // PARITY (stakeholder 2026-07-18: "I want them to be the same in
        // both places"): the sim composes the motor stack EXACTLY as
        // src/firm/main.cpp does -- bare NezhaMotor wrapped in the
        // MotorArmor decorator, the ARMOR handed to the app graph. The only
        // sim/production difference is what answers on the I2C bus.
        armorL_(motorL_),
        armorR_(motorR_),
        otos_(plant_, Devices::OtosConfig{}),
        color_(plant_, Devices::ColorConfig{}),
        line_(plant_, Devices::LineConfig{}),
        comms_(serialLink_, radioLink_, "DEVICE:NEZHA2:sim:sim_harness:1"),
        tlm_(comms_, serialLink_, radioLink_),
        deadman_(clock_),
        drive_(armorL_, armorR_, trackWidth),
        odom_(armorL_, armorR_, trackWidth),
        preamble_(armorL_, armorR_, otos_, color_, line_, clock_),
        headingSource_(otos_, armorL_, armorR_, trackWidth),
        pilot_(executor_, drive_, headingSource_, odom_),
        robotLoop_(plant_, armorL_, armorR_, otos_, comms_, tlm_, drive_, odom_,
                   deadman_, preamble_, pilot_, clock_, sleeper_) {
    // 114-001: no self-configuration here anymore -- executor_/
    // headingSource_/drive_/pilot_ are left at their own default-constructed
    // state (msg::PlannerConfig{}'s all-zero fields) and motorL_/motorR_ at
    // Devices::MotorConfig{}'s all-zero fields, exactly matching a real,
    // not-yet-booted composition root. robotLoop_.isConfigured() is false
    // here (App::RobotLoop's own configured_ default). A caller MUST call
    // configurePlanner() and both configureMotor() calls (or, for a test,
    // TestSupport::configureSimForBenchTest()) before commanding a TWIST/
    // MOVE -- see this file's own header and configurePlanner()'s/
    // configureMotor()'s own comments for the load-bearing-for-the-first-
    // time contract this ticket adds.
    //
    // "Pre-boot state": everything above is constructed and wired, but
    // App::Preamble::step() has not yet been called even once -- boot() is
    // the caller's job (the first call after construction), not the
    // constructor's, mirroring main.cpp's own construct-then-boot split.
  }

  // Drives App::Preamble to done(), then calls the real robotLoop_.boot().
  // See this file's own header for why Preamble is driven directly instead
  // of through a single robotLoop_.boot() call. Idempotent guard: a second
  // call is a no-op (booted_ already true) -- callers are not expected to
  // call this more than once, but nothing breaks if they do.
  void boot() {
    if (booted_) return;
    driveBootToDone();
    robotLoop_.boot();
    booted_ = true;
  }

  // Advances the sim `cycles` times: plant_.tick(dt) THEN
  // clock_.advanceMicros(kCycleDtUs) THEN robotLoop_.cycle() -- see this
  // file's own header for why this order (plant ticks BEFORE the loop reads
  // it) is the one invariant this class exists to enforce. Call boot()
  // first -- matches App::RobotLoop::cycle()'s own "assumes every device is
  // already resolved" precondition.
  void step(int cycles = 1) {
    for (int i = 0; i < cycles; ++i) {
      plant_.tick(static_cast<float>(kCycleDtUs) / 1e6f);  // [s]
      clock_.advanceMicros(kCycleDtUs);
      robotLoop_.cycle();
      ++cycleCount_;
    }
  }

  // Pushes one complete armored ("*B...") line onto the inbound serial
  // FakeTransport -- App::Comms::pump() consumes at most one per cycle()
  // call, so a line enqueued immediately before a step(n) call is consumed
  // on that call's FIRST cycle.
  void injectCommand(const char* armoredLine) { serialLink_.enqueueInbound(armoredLine); }

  // Convenience wrappers over injectCommand() + TestSupport::armor*Command()
  // -- there is no encode(CommandEnvelope) in the generated codec (only a
  // host builds commands), so these are the only way a caller injects a
  // Twist/Stop.
  void injectTwist(float v_x, float omega, float duration, uint32_t corrId = 0) {
    injectCommand(TestSupport::armorTwistCommand(v_x, omega, duration, corrId).c_str());
  }
  void injectStop(uint32_t corrId = 0) {
    injectCommand(TestSupport::armorStopCommand(corrId).c_str());
  }
  // injectMove -- 109-003. See wire_test_codec.h's armorMoveCommand() for
  // the field order (mirrors msg::Move field-for-field).
  void injectMove(float distance, float deltaHeading, float vMax, float omega, float timeMs,
                   bool replace, uint32_t id, uint32_t corrId = 0) {
    injectCommand(TestSupport::armorMoveCommand(distance, deltaHeading, vMax, omega, timeMs,
                                                 replace, id, corrId)
                       .c_str());
  }

  // Motion::Executor visibility -- test-only accessors mirroring the new
  // TLM fields (queueDepth/activeId/state), for tests that want to assert
  // executor state directly rather than only via decoded telemetry.
  uint8_t pilotQueueDepth() const { return pilot_.queueDepth(); }
  uint32_t pilotActiveId() const { return pilot_.activeId(); }
  Motion::State pilotState() const { return pilot_.state(); }

  // App::HeadingSource visibility (109-005) -- test-only accessors mirroring
  // the new TLM heading_source field/event bit, for tests that want to
  // assert the active source directly rather than only via decoded
  // telemetry.
  bool headingSourceIsOtos() const { return pilot_.headingSourceIsOtos(); }

  // plannerConfig -- 111-001 test-only accessor exposing the live
  // msg::PlannerConfig baseline this harness was configured with
  // (Pilot::plannerConfig(), itself derived from whatever configurePlanner()
  // was last called with -- 114-001: TestSupport::configureSimForBenchTest(),
  // for every test that wants the sim's own test-only stand-in values). Lets
  // a test read REAL configured limits
  // (a_max/a_decel/v_body_max/j_max/yaw_acc_max/yaw_rate_max/yaw_jerk_max)
  // instead of hand-duplicating numeric bounds that could silently drift
  // from that configured baseline -- see behavior_lock_harness.cpp.
  //
  // 113-002: this is ALSO the test-only readback for configurePlanner()
  // below -- pilot_.configureHeading(cfg) (the last of configurePlanner()'s
  // four fan-out calls) does `plannerConfig_ = config;`, a full-struct copy,
  // so this accessor reflects EVERY field of whatever msg::PlannerConfig was
  // last pushed through configurePlanner(), not just the fields Pilot's own
  // arithmetic reads. Reused here rather than adding a parallel readback
  // (this ticket's own "choose whichever is less code" acceptance
  // criterion) -- see configurePlanner()'s own comment.
  const msg::PlannerConfig& plannerConfig() const { return pilot_.plannerConfig(); }

  // configurePlanner -- 113-002: ADDITIVE public config-load surface,
  // generalizing the identical 4-call fan-out the constructor above and the
  // three sim-only test hooks below (setYawRateMax()/setLeadCompensation()/
  // setDistanceKp()) already use piecemeal. Lets a caller (the host, via
  // ticket 005's ctypes surface) push a COMPLETE, JSON-derived
  // msg::PlannerConfig in one call instead of only ever getting a
  // hardcoded stand-in baseline (114-001: SimHarness itself carries none
  // anymore -- see this file's own header).
  //
  // Also becomes the new "last known config" baseline the three hooks below
  // build their own override on top of (lastPlannerConfig_/
  // hasConfiguredPlanner_) -- so a caller who calls configurePlanner() with
  // real robot values and THEN calls e.g. setYawRateMax() does not silently
  // lose every other field configurePlanner() set (the bug this ticket's own
  // acceptance criteria call out).
  //
  // Purely ADDITIVE (SUC-005): every pre-existing caller that already calls
  // configurePlanner() (or, since 114-001, TestSupport::
  // configureSimForBenchTest()) keeps rebuilding setYawRateMax()/
  // setLeadCompensation()/setDistanceKp()'s own base config from
  // lastPlannerConfig_ exactly as before -- byte-for-byte unchanged
  // behavior for every pre-existing caller.
  //
  // 114-001: ALSO now load-bearing for App::RobotLoop's configuration-
  // completeness gate -- see maybeMarkConfigured()'s own comment below.
  void configurePlanner(const msg::PlannerConfig& cfg) {
    lastPlannerConfig_ = cfg;
    hasConfiguredPlanner_ = true;
    executor_.configure(cfg);
    headingSource_.configure(cfg);
    drive_.configure(cfg);
    pilot_.configureHeading(cfg);
    maybeMarkConfigured();
  }

  // motorConfig -- 113-002 test-only readback exposing the Devices::
  // MotorConfig last passed to configureMotor() below for the given port
  // (1=left, 2=right -- same convention as configureMotor() itself). This is
  // SimHarness's OWN record of the request, not a live re-read off
  // Devices::MotorArmor/NezhaMotor -- neither stores/exposes a full
  // MotorConfig copy of its own (MotorArmor::configure() only caches one
  // derived field, motionThreshold_, from config.outputDeadband; see
  // configureMotor()'s own comment) -- so this is the only way a caller can
  // read back what configureMotor() was actually called with. Defaults to a
  // default-constructed Devices::MotorConfig{} if configureMotor() was never
  // called for that port.
  const Devices::MotorConfig& motorConfig(uint32_t port) const {
    return (port == 2) ? lastMotorConfigR_ : lastMotorConfigL_;
  }

  // configureMotor -- 113-002: ADDITIVE public config-load surface for one
  // motor channel (114-001: the constructor no longer self-configures at
  // all -- this is now the ONLY way a motor's config_ is ever set past its
  // Devices::MotorConfig{} construction default, see this file's own
  // header). port: 1 = left, 2 = right, matching every other port-keyed
  // convention in this file (see SimPlant::setEncScaleErr()/
  // setEncTickQuantization()'s own "1=left, 2=right" precedent).
  //
  // REVISION 1 (114-001, Decision 6, sprint.md): Devices::MotorArmor::
  // reconfigure() (motor_armor.h) now forwards the WHOLE config to the
  // wrapped NezhaMotor -- port/fwdSign/velGains/velFiltAlpha/slewRate/
  // wheelTravelCalib all take live effect through THIS call, not just
  // outputDeadband's derived motionThreshold_ cache.
  //
  // armorR_.reconfigure()/armorL_.reconfigure() are [[nodiscard]] and
  // guarded (refuse while genuinely in motion) -- for THIS gate scenario (a
  // freshly constructed, never-yet-commanded SimHarness) they must always
  // return true; a false here is a real bug, not the expected
  // operator-driven refusal that can only happen via the independent
  // mid-session sim_configure_motor()/TestGUI robot-select path (which
  // calls this same method after the sim may already be driving) -- so
  // assert rather than silently drop it.
  //
  // 114-001: ALSO now load-bearing for App::RobotLoop's configuration-
  // completeness gate -- see maybeMarkConfigured()'s own comment below.
  void configureMotor(uint32_t port, const Devices::MotorConfig& cfg) {
    if (port == 2) {
      lastMotorConfigR_ = cfg;
      bool applied = armorR_.reconfigure(cfg);
      assert(applied && "armorR_.reconfigure() refused on a fresh SimHarness -- real bug, not expected");
      (void)applied;
      hasConfiguredMotorR_ = true;
    } else {
      lastMotorConfigL_ = cfg;
      bool applied = armorL_.reconfigure(cfg);
      assert(applied && "armorL_.reconfigure() refused on a fresh SimHarness -- real bug, not expected");
      (void)applied;
      hasConfiguredMotorL_ = true;
    }
    // 114-007 (Decision 7): teach the plant this port's own mount
    // orientation so its OtosPlant-feeding boundary can correct for a
    // mirror-mounted motor -- see SimPlant::setFwdSign()'s own comment.
    // Routes through this SAME call site sim_ctypes.cpp's
    // sim_configure_motor() already uses (harness->configureMotor()), so
    // both the C++ direct-harness path and the ctypes/TestGUI robot-select
    // path pick this up with this one change.
    plant_.setFwdSign(static_cast<int>(port), cfg.fwdSign);
    maybeMarkConfigured();
  }

  // driveTargetVelLeft/driveTargetVelRight -- 111-003 test-only accessors
  // exposing the STAGED PID-target velocity (Devices::Motor::
  // velocityTarget(), the value last written by App::Drive::tick()'s own
  // setVelocity() call), NOT the measured/decoded telemetry velocity. Used
  // by behavior_lock_harness.cpp's measureShelfCycles() to measure the
  // post-completion "shelf" directly: the ideal sim's terminal decel
  // already drives the MEASURED wheel velocity near zero by the time a
  // command completes, so a stale nonzero COMMAND held for the ~300ms
  // deadman-lease window is invisible in the measured trace (see ticket
  // 003's own completion notes); the commanded target only reads EXACTLY
  // 0.0f once something explicitly stages a zero twist, which is exactly
  // the behavior ticket 003's fix changes the TIMING of.
  float driveTargetVelLeft() const { return armorL_.velocityTarget(); }    // [mm/s] signed
  float driveTargetVelRight() const { return armorR_.velocityTarget(); }  // [mm/s] signed

  // plannedRefLeft/plannedRefRight -- 112-002 test-only accessors exposing
  // App::Pilot's own PLANNED per-wheel reference (Pilot::refLeft/refRight():
  // Motion::Executor's jerk-limited trajectory mapped through
  // BodyKinematics::inverse(), BEFORE the heading-PD correction and BEFORE
  // App::Drive's actuation-lag feedforward) -- NOT driveTargetVelLeft/Right()
  // above, which is the FINAL, FF-augmented command Devices::Motor actually
  // chases. Used by behavior_lock_harness.cpp's ramp/terminal-bounds and
  // single-lobe/lobes-opposite-sign checks (112-002 re-grade): those check
  // the PLANNED trajectory's own jerk-boundedness, which the accel
  // feedforward (112-002) deliberately perturbs on the commanded signal --
  // see that file's own header comment for the full rationale.
  float plannedRefLeft() const { return pilot_.refLeft(); }    // [mm/s] signed
  float plannedRefRight() const { return pilot_.refRight(); }  // [mm/s] signed

  // debugHeadingLead -- 109-010 diagnostic-only accessor (temporary
  // instrumentation, mirrors this sprint's own precedent of ad hoc trace
  // instrumentation during characterization -- see ticket 009's own
  // Iteration Log): exposes heading()/headingLead() and usingOtos() so the
  // characterization work can directly confirm the projection is actually
  // engaged before trusting a sweep's own numbers.
  void debugHeadingLead(bool* usingOtos, float* heading, float* headingLead) const {
    *usingOtos = headingSource_.usingOtos();
    *heading = headingSource_.heading();
    *headingLead = headingSource_.headingLead();
  }

  // setLeadCompensation -- 109-010: test-only hook for the rate-sweep
  // characterization harness to try different lead-compensation Δt's
  // WITHOUT a reflash/rebuild -- these three fields have no wire
  // PlannerConfigPatch arm (they are boot-baked-default-only per this
  // ticket's own scope, see planner.proto's own field comments), so this
  // sim-only C++/ctypes path (mirrored by sim_ctypes.cpp's
  // sim_set_lead_compensation() and SimLoop.set_lead_compensation() on the
  // Python side) is the ONLY way a test can vary them against the compiled
  // sim. Re-applies the full current configured baseline plus the three
  // overrides to every consumer (Executor::configure()/
  // HeadingSource::configure()/Pilot::configureHeading()), the same
  // re-apply-to-every-consumer shape Pilot::applyPlannerPatch() already
  // uses for a live wire patch.
  //
  // 113-002: rebuilds its base config from lastPlannerConfig_ (whatever
  // configurePlanner() last received) instead of ALWAYS restarting from a
  // hardcoded baseline -- falls back to msg::PlannerConfig{}'s own all-zero
  // default (114-001: SimHarness itself carries no hardcoded stand-in
  // anymore -- see this file's own header) only when configurePlanner() was
  // never called, so this hook's behavior is byte-for-byte unchanged for
  // every existing caller (SUC-005: none of them call configurePlanner()).
  // Delegates the actual fan-out to configurePlanner() itself (the same 4
  // calls, same order, this method used to make inline) so the two surfaces
  // share one bookkeeping path.
  void setLeadCompensation(float headingLeadBias, float planLead, float terminalLead) {
    lastHeadingLeadBias_ = headingLeadBias;
    lastPlanLead_ = planLead;
    lastTerminalLead_ = terminalLead;
    msg::PlannerConfig cfg = hasConfiguredPlanner_ ? lastPlannerConfig_ : msg::PlannerConfig{};
    cfg.heading_lead_bias = headingLeadBias;
    cfg.plan_lead = planLead;
    cfg.terminal_lead = terminalLead;
    cfg.yaw_rate_max = lastYawRateMax_;
    cfg.distance_kp = lastDistanceKp_;  // 112-003: compose with setDistanceKp() regardless of call order
    configurePlanner(cfg);
  }

  // setYawRateMax -- 109-010 rate-sweep characterization harness hook: vary
  // the pivot cruise-rate ceiling (Motion::JerkTrajectory's own rotational
  // channel ceiling, `PlannerConfig.yaw_rate_max`) across a test's own sweep
  // of commanded rates WITHOUT a reflash/rebuild, the same sim-only-hook
  // shape as setLeadCompensation() above (and re-applying whatever lead
  // compensation was last set, so the two hooks compose regardless of call
  // order).
  //
  // 113-002: same lastPlannerConfig_-or-msg::PlannerConfig{} base + delegate-
  // to-configurePlanner() refactor as setLeadCompensation() above -- see
  // that method's own comment.
  void setYawRateMax(float yawRateMax) {
    lastYawRateMax_ = yawRateMax;
    msg::PlannerConfig cfg = hasConfiguredPlanner_ ? lastPlannerConfig_ : msg::PlannerConfig{};
    cfg.yaw_rate_max = yawRateMax;
    cfg.heading_lead_bias = lastHeadingLeadBias_;
    cfg.plan_lead = lastPlanLead_;
    cfg.terminal_lead = lastTerminalLead_;
    cfg.distance_kp = lastDistanceKp_;  // 112-003: compose with setDistanceKp() regardless of call order
    configurePlanner(cfg);
  }

  // setDistanceKp -- 112-003 test-only hook, same shape as
  // setYawRateMax()/setLeadCompensation() above: lets a test override
  // PlannerConfig.distance_kp away from bench_test_config.cpp's own
  // benchTestPlannerConfig() shipped default (8.0 as of 112-004; see that
  // function's own comment) -- e.g. to 0.0 for a test that wants
  // App::Pilot's bounded linear position-feedback trim completely inert, or
  // to a specific value for a targeted gain/clamp check. Used by
  // pilot_distance_trim_harness.cpp's own 087-009 clamp-authority guardrail
  // check.
  //
  // 113-002: same lastPlannerConfig_-or-msg::PlannerConfig{} base + delegate-
  // to-configurePlanner() refactor as setLeadCompensation()/setYawRateMax()
  // above -- see setLeadCompensation()'s own comment.
  void setDistanceKp(float distanceKp) {
    lastDistanceKp_ = distanceKp;
    msg::PlannerConfig cfg = hasConfiguredPlanner_ ? lastPlannerConfig_ : msg::PlannerConfig{};
    cfg.distance_kp = distanceKp;
    cfg.heading_lead_bias = lastHeadingLeadBias_;
    cfg.plan_lead = lastPlanLead_;
    cfg.terminal_lead = lastTerminalLead_;
    cfg.yaw_rate_max = lastYawRateMax_;
    configurePlanner(cfg);
  }

  // Decodes and returns every outbound line captured on the serial
  // FakeTransport since the last call (both FakeTransport instances receive
  // an IDENTICAL broadcast -- App::Comms::sendReply()/App::Telemetry's own
  // secondary-frame send both fan out to serial AND radio -- so draining
  // just one is sufficient and avoids duplicate decoded frames).
  std::vector<TestSupport::DecodedLine> drainTelemetry() {
    std::vector<TestSupport::DecodedLine> result;
    const auto& sent = serialLink_.sent();
    for (; telemetryDrainIndex_ < sent.size(); ++telemetryDrainIndex_) {
      result.push_back(TestSupport::decodeOutboundLine(sent[telemetryDrainIndex_]));
    }
    return result;
  }

  // Raw (still-armored "*B...") outbound lines captured on the serial
  // FakeTransport since the last call to THIS method -- a separate drain
  // index from drainTelemetry()'s own telemetryDrainIndex_, so a caller
  // using one drain method is unaffected by (and does not starve) the
  // other. Ticket 108-005's sim_ctypes.cpp C ABI wants raw wire text (its
  // Python caller dearmors/decodes with the same pb2 codec a real robot's
  // replies use, per sim_ctypes.cpp's own header) rather than the
  // C++-side TestSupport::DecodedLine drainTelemetry() returns.
  std::vector<std::string> drainRawTelemetry() {
    std::vector<std::string> result;
    const auto& sent = serialLink_.sent();
    for (; rawTelemetryDrainIndex_ < sent.size(); ++rawTelemetryDrainIndex_) {
      result.push_back(sent[rawTelemetryDrainIndex_]);
    }
    return result;
  }

  bool booted() const { return booted_; }
  int cycleCount() const { return cycleCount_; }  // total robotLoop_.cycle() calls made so far

  // isConfigured -- 114-001 thin passthrough to App::RobotLoop's own
  // configuration-completeness gate (robot_loop.h). false immediately after
  // construction (SimHarness no longer self-configures); true only once
  // configurePlanner() AND both configureMotor() calls have landed (see
  // maybeMarkConfigured()'s own comment below).
  bool isConfigured() const { return robotLoop_.isConfigured(); }

  // The composed SimPlant -- exposes fault knobs (setDisconnected()/
  // freezePosition()/setDropoutRate()/setOtosDrift()) and the read/write
  // hook registration (setReadHook()/setWriteHook()) directly, per this
  // ticket's own "callers can set fault knobs/hooks" requirement.
  SimPlant& plant() { return plant_; }
  const SimPlant& plant() const { return plant_; }

  // True pose -- SimPlant's owned OtosPlant ground truth, bypassing any
  // sensor noise/drift. See this file's own header for why these three
  // accessors, specifically, are "the" true pose.
  float trueX() const { return plant_.otosPlant().x(); }              // [mm]
  float trueY() const { return plant_.otosPlant().y(); }              // [mm]
  float trueHeading() const { return plant_.otosPlant().heading(); }  // [rad]

  // Pose reset ("Set Robot @ 0,0" / SI). Teleports the plant TRUTH (OtosPlant)
  // AND resets the firmware's own encoder-derived state to match, so the
  // telemetry pose/otos the UI shows actually snap to (x,y,heading) -- not
  // just the avatar. Without the firmware half, the wire's SI/OZ/ZERO verbs
  // (no binary arm yet) leave the firmware believing its old pose.
  void setTruePose(float x, float y, float heading) {  // [mm] [mm] [rad]
    plant_.setTruePose(x, y, heading);
    // Zero each motor's software encoder offset against its CURRENT (kept-
    // continuous) raw so position() reads 0 with no discontinuity, then snap
    // odometry to (x,y,heading) with its delta baseline re-anchored to those
    // now-zero positions. begin() == hardReset() (nezha_motor.cpp) and is
    // public; it drives the bus (SimPlant answers it).
    motorL_.begin();
    motorR_.begin();
    odom_.reset(x, y, heading);
  }

  Devices::NezhaMotor& motorLeft() { return motorL_; }
  Devices::NezhaMotor& motorRight() { return motorR_; }

  // Concrete TestSim::SimClock&, not Devices::Clock& -- callers (this
  // class's own driveBootToDone(), sim_api_harness.cpp) need the
  // setMicros()/advanceMicros() stepping surface, which only the concrete
  // fake exposes now that Devices::Clock is a pure interface (ticket 010).
  TestSim::SimClock& clock() { return clock_; }

  // Ticket 108-004's own migrated sim_api_harness.cpp timing-diagnostic
  // scenario needs this to reproduce the deleted SimApi::measureOneCycle()'s
  // sleepCount()/lastSleepMillis()/yieldCount() deltas directly -- exposed
  // here rather than re-adding a bespoke CycleTimingReport wrapper, matching
  // this class's existing "expose the owned device, let the caller read it"
  // pattern (motorLeft()/motorRight()/clock() above). Concrete
  // TestSim::SimSleeper&, for the same reason as clock() above.
  TestSim::SimSleeper& sleeper() { return sleeper_; }

  // [us] the fixed per-cycle virtual-time advance step() applies before
  // every robotLoop_.cycle() call -- matches sim_api.h's own kCycleDtUs
  // derivation (>=40ms so a fresh duty write is never write-rate-throttled;
  // comfortably >= Devices::Otos::kReadPeriod so OTOS is due every cycle).
  static constexpr uint32_t kCycleDtUs = 50000;  // [us]

 private:
  // 114-001: the two private static methods that used to live here (the
  // hardcoded planner/motor stand-in values) are DELETED -- SimHarness
  // itself no longer carries a behavioral default. The SAME values,
  // byte-for-byte, now live at src/tests/sim/support/bench_test_config.h
  // (TestSupport::benchTestPlannerConfig()/benchTestMotorConfig()), an
  // explicitly test-tree-only header the existing sim harnesses opt into
  // via TestSupport::configureSimForBenchTest() (Decision 3, sprint.md).

  // Drives App::Preamble to done() via preamble_.step() calls issued
  // OURSELVES, advancing the fake Clock between each one -- see this file's
  // own header comment for why (color_/line_'s own retry pacing needs real
  // elapsed virtual time between attempts; a single robotLoop_.boot() call
  // offers no such opportunity). Left/Right/Otos each resolve on their own
  // very first real transaction against the live SimPlant (SimPlant answers
  // correctly regardless of how many times it has been asked before -- there
  // is no scripted-count budget to exhaust the way the old scripted-FIFO bus
  // had), so only Color/Line's own retry-until-exhausted budgets govern how
  // long this loop actually needs to run.
  void driveBootToDone() {
    clock_.setMicros(0);
    preamble_.step();  // arms Preamble's own startUs_ at 0 -- power-settle no-op

    clock_.setMicros(50000);  // >= Preamble::kPowerSettle -- probing starts on the NEXT step()

    // 200 passes at 50ms apart is a generous bound over color_/line_'s own
    // natural worst case (~21 * 50ms and 20 * 50ms respectively) -- see
    // sim_api.cpp's own driveBootToDone() for the identical derivation this
    // duplicates. If this is ever exceeded, done() staying false is a real
    // bug, not a slow-but-fine boot -- left able to actually hang a test
    // run rather than silently forcing a false "success."
    for (int i = 0; i < 200 && !preamble_.done(); ++i) {
      preamble_.step();
      clock_.advanceMicros(50000);
    }
  }

  SimPlant plant_;
  TestSim::SimClock clock_;
  TestSim::SimSleeper sleeper_;

  Devices::NezhaMotor motorL_;
  Devices::NezhaMotor motorR_;
  // PARITY: the armor wraps each bare motor exactly as main.cpp does; the
  // app graph below takes the ARMOR, never the bare leaf. Declared after
  // the motors (init order) and before every consumer.
  Devices::MotorArmor armorL_;
  Devices::MotorArmor armorR_;
  Devices::Otos otos_;
  Devices::ColorSensorLeaf color_;
  Devices::LineSensorLeaf line_;

  TestSupport::FakeTransport serialLink_;
  TestSupport::FakeTransport radioLink_;

  App::Comms comms_;
  App::Telemetry tlm_;
  App::Deadman deadman_;
  App::Drive drive_;
  App::Odometry odom_;
  App::Preamble preamble_;

  Motion::Executor executor_;
  App::HeadingSource headingSource_;
  App::Pilot pilot_;

  App::RobotLoop robotLoop_;

  bool booted_ = false;
  int cycleCount_ = 0;

  size_t telemetryDrainIndex_ = 0;  // index into serialLink_.sent() already returned by drainTelemetry()
  size_t rawTelemetryDrainIndex_ = 0;  // index into serialLink_.sent() already returned by drainRawTelemetry()

  // 109-010: setLeadCompensation()/setYawRateMax()/setDistanceKp() each
  // rebuild a fresh msg::PlannerConfig from their own base (lastPlannerConfig_
  // if configurePlanner() was ever called, else an all-zero
  // msg::PlannerConfig{} -- 114-001, since SimHarness no longer carries a
  // hardcoded fallback baseline) -- these remember whichever of the three
  // was last set by ANY of the three hooks so the others can re-apply it
  // instead of silently clobbering it back to a default.
  float lastYawRateMax_ = 4.0f;  // [rad/s] matches TestSupport::benchTestPlannerConfig()'s own default
  float lastHeadingLeadBias_ = -0.05f;  // [s] matches TestSupport::benchTestPlannerConfig()'s own default
  float lastPlanLead_ = 0.20f;         // [s] matches TestSupport::benchTestPlannerConfig()'s own default
  float lastTerminalLead_ = 0.0f;      // [s]
  float lastDistanceKp_ = 2.5f;        // [1/s] 112-003/112-004, matches TestSupport::benchTestPlannerConfig()'s own default

  // 113-002: configurePlanner()'s own "last known config" bookkeeping --
  // see that method's own comment. 114-001: ALSO the planner half of the
  // configuration-completeness gate's completion tracking (the motor half
  // is hasConfiguredMotorL_/hasConfiguredMotorR_ below) -- see
  // maybeMarkConfigured()'s own comment.
  bool hasConfiguredPlanner_ = false;
  msg::PlannerConfig lastPlannerConfig_ = {};

  // 113-002: configureMotor()'s own test-only readback state -- see
  // motorConfig()'s own comment for why SimHarness keeps this copy itself
  // rather than reading it back off Devices::MotorArmor/NezhaMotor (neither
  // stores one). Defaults to Devices::MotorConfig{} (all-zero) until
  // configureMotor() is called for that port.
  Devices::MotorConfig lastMotorConfigL_ = {};
  Devices::MotorConfig lastMotorConfigR_ = {};

  // 114-001: the motor half of the configuration-completeness gate's
  // completion tracking -- see maybeMarkConfigured()'s own comment.
  bool hasConfiguredMotorL_ = false;
  bool hasConfiguredMotorR_ = false;

  // maybeMarkConfigured -- Decision 1, sprint.md: the whole graph is
  // considered configured once ALL THREE of the atomic fan-out calls that
  // together constitute "the sim's own boot bake" have landed
  // (configurePlanner() once, configureMotor() for BOTH ports) --
  // mirroring how main.cpp's real boot-configure sequence is one atomic
  // whole before markConfigured() fires. Called from the tail of
  // configurePlanner()/configureMotor(); markConfigured() itself is
  // idempotent (a plain configured_ = true;), so calling this once too
  // often (e.g. a caller that configures the same port twice) is harmless.
  void maybeMarkConfigured() {
    if (hasConfiguredPlanner_ && hasConfiguredMotorL_ && hasConfiguredMotorR_) {
      robotLoop_.markConfigured();
    }
  }
};

}  // namespace TestSim
