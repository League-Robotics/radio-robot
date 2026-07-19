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
        motorL_(plant_, makeMotorConfig(1)),
        motorR_(plant_, makeMotorConfig(2)),
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
    armorL_.configure(makeMotorConfig(1));
    armorR_.configure(makeMotorConfig(2));
    // Motion::Executor + App::HeadingSource + App::Pilot (109-003/109-005)
    // -- configured from the same default msg::PlannerConfig{} zero-value
    // struct main.cpp's real Config::defaultPlannerConfig() would otherwise
    // supply; the sim harness has no boot_config.cpp to read from, so
    // tests that need real gains set them explicitly via a future accessor
    // (none needed by this ticket's own tests, which only exercise
    // TIMED-mode ramps and 109-005's own DISTANCE/heading-PD scenarios
    // against the vBodyMax/yawRateMax/aDecel/jMax/headingKp/headingDwell*
    // values makeExecutorConfig() below supplies).
    msg::PlannerConfig cfg = makeExecutorConfig();
    executor_.configure(cfg);
    headingSource_.configure(cfg);
    pilot_.configureHeading(cfg);

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
  // (Pilot::plannerConfig(), itself derived from this class's own
  // makeExecutorConfig()). Lets a test read REAL configured limits
  // (a_max/a_decel/v_body_max/j_max/yaw_acc_max/yaw_rate_max/yaw_jerk_max)
  // instead of hand-duplicating numeric bounds that could silently drift
  // from makeExecutorConfig()'s own values -- see behavior_lock_harness.cpp.
  const msg::PlannerConfig& plannerConfig() const { return pilot_.plannerConfig(); }

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
  // sim. Re-applies the full current makeExecutorConfig() baseline plus the
  // three overrides to every consumer (Executor::configure()/
  // HeadingSource::configure()/Pilot::configureHeading()), the same
  // re-apply-to-every-consumer shape Pilot::applyPlannerPatch() already
  // uses for a live wire patch.
  void setLeadCompensation(float headingLeadBias, float planLead, float terminalLead) {
    lastHeadingLeadBias_ = headingLeadBias;
    lastPlanLead_ = planLead;
    lastTerminalLead_ = terminalLead;
    msg::PlannerConfig cfg = makeExecutorConfig();
    cfg.heading_lead_bias = headingLeadBias;
    cfg.plan_lead = planLead;
    cfg.terminal_lead = terminalLead;
    cfg.yaw_rate_max = lastYawRateMax_;
    executor_.configure(cfg);
    headingSource_.configure(cfg);
    pilot_.configureHeading(cfg);
  }

  // setYawRateMax -- 109-010 rate-sweep characterization harness hook: vary
  // the pivot cruise-rate ceiling (Motion::JerkTrajectory's own rotational
  // channel ceiling, `PlannerConfig.yaw_rate_max`) across a test's own sweep
  // of commanded rates WITHOUT a reflash/rebuild, the same sim-only-hook
  // shape as setLeadCompensation() above (and re-applying whatever lead
  // compensation was last set, so the two hooks compose regardless of call
  // order).
  void setYawRateMax(float yawRateMax) {
    lastYawRateMax_ = yawRateMax;
    msg::PlannerConfig cfg = makeExecutorConfig();
    cfg.yaw_rate_max = yawRateMax;
    cfg.heading_lead_bias = lastHeadingLeadBias_;
    cfg.plan_lead = lastPlanLead_;
    cfg.terminal_lead = lastTerminalLead_;
    executor_.configure(cfg);
    headingSource_.configure(cfg);
    pilot_.configureHeading(cfg);
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
  // See sim_api.cpp's own makeMotorConfig() for the byte-for-byte
  // derivation of every field set here -- unchanged tuning, just relocated.
  // A large proportional gain (kp) plus a wide slew rate lets an injected
  // twist saturate the PID quickly and reach full duty in one write; this
  // harness's own SimPlant then integrates whatever duty actually lands on
  // the wire, live, so there is no predictor to keep in sync with this
  // tuning the way SimApi's DutyPredictor had to be.
  static Devices::MotorConfig makeMotorConfig(uint32_t port) {
    Devices::MotorConfig cfg;
    cfg.port = port;
    cfg.fwdSign = 1;
    // mm per encoder count (== mm/motor-degree, 360/rev). Must be the RECIPROCAL
    // of sim_plant.cpp's kEncoderCountsPerMm so counts*travelCalib round-trips
    // to true mm (1.4187 * 0.704871 == 1.0). The GUI overrides this at connect
    // with the geometry-derived ml/mr push (~0.70486), which agrees.
    cfg.wheelTravelCalib = 0.704871f;
    cfg.velFiltAlpha = 1.0f;
    cfg.slewRate = 100.0f;
    // Velocity feedforward so the sim tracks the COMMANDED velocity (like the
    // real robot's calibrated gains do), instead of under-tracking ~17% on
    // pure-P and undershooting every drive/turn. kff = 1/kDefaultDutyVelMax:
    // duty = target/500 -> plant velocity = 500*duty = target (open-loop
    // exact), with kp trimming transients/disturbance.
    cfg.velGains.kff = 1.0f / TestSim::kDefaultDutyVelMax;  // 0.002 duty per mm/s
    cfg.velGains.kp = 0.01f;   // feedback trim -- needed for turn accuracy
                               // (kp=0 lands 90deg turns ~30deg off + faults)
    // PARITY (stakeholder 2026-07-18): reversalDwell/outputDeadband are
    // deliberately left UNSET -- exactly what the production boot config
    // bakes (gen_boot_config.py leaves both .has == false on purpose), so
    // NezhaMotor's ctor substitutes the SAME ship defaults (100ms / 0.03)
    // in the sim as on the robot. The sim gets no special write-shaping
    // configuration of its own -- the whole motor stack behaves
    // identically in both places; only the far side of the I2C bus
    // differs.
    return cfg;
  }

  // makeExecutorConfig -- a non-zero msg::PlannerConfig for Motion::
  // Executor's own configure() call (109-003). This harness has no
  // boot_config.cpp to read a real per-robot value from (main.cpp's own
  // Config::defaultPlannerConfig()); these are reasonable stand-in values
  // (matching data/robots/tovez.json's own order of magnitude) sufficient
  // for a TIMED-mode ramp/hold/ramp-down to exercise real jerk-limited
  // motion in a sim test -- NOT bench-tuned, and not meant to be (no
  // bench/sim test in this ticket asserts a SPECIFIC numeric gain, only
  // jerk-boundedness/no-instant-step/queue-mechanics).
  static msg::PlannerConfig makeExecutorConfig() {
    msg::PlannerConfig cfg;
    cfg.a_max = 800.0f;         // [mm/s^2]
    cfg.a_decel = 1000.0f;      // [mm/s^2]
    cfg.v_body_max = 600.0f;    // [mm/s]
    cfg.yaw_rate_max = 4.0f;    // [rad/s]
    cfg.yaw_acc_max = 20.0f;    // [rad/s^2]
    cfg.j_max = 8000.0f;        // [mm/s^3]
    cfg.yaw_jerk_max = 80.0f;   // [rad/s^3]
    // 109-005: heading PD cascade gain + dwell-completion gate. kp=6.0
    // matches data/robots/tovez.json's own bench-proven sprint-098 value
    // (see .clasi/knowledge/heading-loop-solves-turn-accuracy.md); the
    // dwell tolerance/rate/hold match this same file's own
    // planner.proto/gen_boot_config.py default derivation (0.5deg/1deg-per-
    // s/150ms).
    cfg.heading_kp = 6.0f;                     // [1/s]
    cfg.heading_kd = 0.0f;                     // dimensionless
    // Dwell tolerance 1.5deg (was 0.5deg) + min_speed 20mm/s (2026-07-18,
    // terminal stiction/deadband floor): with the write shaping honestly ON
    // (parity), the smallest wheel command that moves the plant is
    // ~outputDeadband/kff ~= 15mm/s -- the PD stalls below that, so
    // App::Pilot floors its terminal output at min_speed (pilot.cpp) and
    // the dwell tolerance must sit ABOVE where a floored approach can stop
    // (floor rate x plant decay ~= 1.3deg). Matches gen_boot_config.py's
    // own updated defaults -- same values both places.
    cfg.heading_dwell_tol = 3.0f * 3.14159265f / 180.0f;   // [rad]
    cfg.heading_dwell_rate = 1.0f * 3.14159265f / 180.0f;  // [rad/s]
    cfg.arrive_dwell = 0.15f;                  // [s]
    cfg.min_speed = 16.0f;                     // [mm/s] Pilot heading-PD floor: just above the ~15mm/s deadband cut; coast quantum = floor x (tau 0.13s + a write cycle) ~= 2.6deg < the 3deg dwell tol
    // 109-010: lead-compensation defaults. heading_lead_bias defaults to
    // -0.05 -- NOT 0.0 -- matching gen_boot_config.py's own shipped
    // HEADING_LEAD_BIAS_DEFAULT (see that constant's own doc comment for
    // the full characterization writeup): a genuinely UNCOMPENSATED raw
    // age lead (bias=0.0) was found, DURING this ticket's own work, to
    // actively FAULT pre-existing sim system tests that construct a
    // SimHarness/SimLoop and never call setLeadCompensation() at all
    // (test_sim_transport_tour1.py, heading_source_harness.cpp) -- this
    // class's own OTOS burst-read omega used to always read 0 (TestSim::
    // OtosPlant's own pre-109-010 stub), so the projection was silently
    // inert everywhere until this ticket's own OtosPlant::omega() fix
    // made it real; -0.05 (this harness's own 50ms kCycleDtUs, exactly
    // canceled) restores the pre-109-010 NO-OP behavior as the harness's
    // own default, matching the shipped firmware default's own posture.
    // plan_lead/terminal_lead default to 0.0 (genuine no-ops, unaffected
    // by the omega fix). setLeadCompensation() below overrides all three
    // for a test that wants to sweep them.
    cfg.heading_lead_bias = -0.05f;  // [s]
    cfg.plan_lead = 0.20f;           // [s] ~2 staging cycles + plant tau -- eliminates the terminal PD reversal (2026-07-18 sweep; matches gen_boot_config.py PLAN_LEAD_DEFAULT)
    cfg.terminal_lead = 0.0f;        // [s]
    return cfg;
  }

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

  // 109-010: setLeadCompensation()/setYawRateMax() each re-derive a fresh
  // msg::PlannerConfig from makeExecutorConfig() (there is no single
  // persisted PlannerConfig instance this harness keeps outside that
  // factory function) -- these remember whichever of the two was last set
  // by EITHER hook so the other can re-apply it instead of silently
  // clobbering it back to 0.
  float lastYawRateMax_ = 4.0f;  // [rad/s] matches makeExecutorConfig()'s own default
  float lastHeadingLeadBias_ = -0.05f;  // [s] matches makeExecutorConfig()'s own default
  float lastPlanLead_ = 0.20f;         // [s] matches makeExecutorConfig()s own default
  float lastTerminalLead_ = 0.0f;      // [s]
};

}  // namespace TestSim
