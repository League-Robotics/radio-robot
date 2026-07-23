// sim_harness.h -- TestSim::SimHarness: the composition root wiring the REAL
// App::RobotLoop firmware graph to a SimPlant (tests/_infra/sim/sim_plant.{h,cpp}).
// Supersedes tests/sim/support/sim_api.{h,cpp} (TestSim::SimApi + its
// DutyPredictor) -- full history: src/sim/DESIGN.md. THIN composition root
// -- no simulation logic (SimPlant/WheelPlant/OtosPlant), no firmware
// dispatch logic (App::RobotLoop, unmodified). Modeled on how
// src/firm/main.cpp constructs the same graph, substituting only the
// I2CBus& slot: main.cpp passes a Devices::MicroBitI2CBus, this class
// passes a TestSim::SimPlant.
//
// The one invariant that matters: tick the plant BEFORE the loop reads it,
// every cycle -- step(n) calls plant_.tick(dt) FIRST, then robotLoop_.
// cycle(), never the other order (src/sim/DESIGN.md's "Invariants worth
// keeping" #1; SimPlant::tick()'s own doc comment has the physics-side
// half). Two entry points, matching RobotLoop's own boot()/cycle() split --
// boot() drives App::Preamble to done() via direct preamble_.step() calls
// (see driveBootToDone()'s own comment for why not a single
// robotLoop_.boot() call), then calls the real robotLoop_.boot(); step(n)
// runs n cycles of (plant_.tick(dt); clock_.advanceMicros(dt);
// robotLoop_.cycle()). Call boot() first.
//
// trueX()/trueY()/trueHeading() read SimPlant's owned OtosPlant ground
// truth directly -- NOT the wire-visible reportedX/Y/Heading() (applies the
// OTOS drift/bias fault knob) and NOT App::Odometry's own independently
// integrated pose -- for test assertions that must bypass sensor noise.
#pragma once

#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

#include "app/comms.h"
#include "app/drive.h"
#include "app/move_queue.h"
#include "app/odometry.h"
#include "app/preamble.h"
#include "app/robot_loop.h"
#include "app/state_estimator.h"
#include "app/telemetry.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/line_sensor.h"
#include "devices/motor_armor.h"
#include "devices/nezha_motor.h"
#include "devices/otos.h"
#include "fake_transport.h"
#include "sim_clock.h"
#include "sim_plant.h"
#include "wire_test_codec.h"

namespace TestSim {

class SimHarness {
 public:
  // trackWidth: [mm] -- passed to BOTH the SimPlant's own OtosPlant and the
  // real App::Drive/App::Odometry instances constructed here, so the
  // simulated OTOS chip and firmware's own odometry describe the same
  // wheelbase. Defaults to TestSim::kDefaultTrackWidth (SimPlant's own).
  explicit SimHarness(float trackWidth = kDefaultTrackWidth)
      : plant_(trackWidth),
        motorL_(plant_, Devices::MotorConfig{}),
        motorR_(plant_, Devices::MotorConfig{}),
        // PARITY: the sim composes the motor stack exactly as
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
        drive_(armorL_, armorR_, trackWidth),
        odom_(armorL_, armorR_, trackWidth),
        // Default-constructed, not sourced from Config::
        // defaultEstimatorConfig() -- that generated config lives outside
        // the sim CMake target (bakes in the active robot JSON at ARM
        // build time; src/sim/CMakeLists.txt's own "Absent (deliberately)"
        // note). Behaviorally equivalent (FusionWeights{}'s defaults match
        // every robot JSON's committed estimator weights). Kept solely for
        // robotLoop_'s own stateEstimator_.update() call -- App::MoveQueue
        // no longer holds a StateEstimator& (move_queue.h).
        stateEstimator_(),
        // shaperLimits similarly left at its default (App::ShaperLimits{},
        // shaping OFF) for the same "not part of the sim graph" boundary --
        // a test needing shaping calls moveQueue().setShaperLimits() directly.
        moveQueue_(drive_, odom_, clock_),
        preamble_(armorL_, armorR_, otos_, color_, line_, clock_),
        robotLoop_(plant_, armorL_, armorR_, otos_, color_, line_, comms_, tlm_,
                   drive_, odom_, moveQueue_, preamble_, stateEstimator_, clock_,
                   sleeper_) {
    // No self-configuration -- motorL_/motorR_ stay at their default
    // Devices::MotorConfig{} (all-zero), matching a real, not-yet-booted
    // composition root. A caller MUST call configureMotor() for BOTH ports
    // (or TestSupport::configureSimForBenchTest()) before commanding a
    // MOVE -- see maybeMarkConfigured()'s own comment below. "Pre-boot
    // state": everything above is constructed and wired, but
    // App::Preamble::step() has not yet been called -- boot() is the
    // caller's job, mirroring main.cpp's own construct-then-boot split.
  }

  // Drives App::Preamble to done(), then calls the real robotLoop_.boot()
  // (see this file's own header for why). Idempotent -- a second call is a
  // no-op (booted_ already true).
  void boot() {
    if (booted_) return;
    driveBootToDone();
    robotLoop_.boot();
    booted_ = true;
  }

  // Advances the sim `cycles` times: plant_.tick(dt) THEN clock_.
  // advanceMicros(kCycleDtUs) THEN robotLoop_.cycle() -- see this file's
  // own header for the ordering invariant. Call boot() first.
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
  // call.
  void injectCommand(const char* armoredLine) { serialLink_.enqueueInbound(armoredLine); }

  // Convenience wrappers over injectCommand() + TestSupport::armorMoveCommand()
  // -- the only way a caller injects a Move/Stop (there is no
  // encode(CommandEnvelope) in the generated codec; only a host builds
  // commands). Two injectMove() overloads mirror armorMoveCommand()'s own
  // two velocity-variant overloads (twist vs wheels), disambiguated by
  // `stopKind` (TestSupport::MoveStopKind) sitting at a different,
  // type-incompatible parameter position in each signature.
  void injectMove(float v_x, float v_y, float omega, TestSupport::MoveStopKind stopKind,
                   float stopValue, float timeout, bool replace, uint32_t id,
                   uint32_t corrId = 0) {
    injectCommand(TestSupport::armorMoveCommand(v_x, v_y, omega, stopKind, stopValue, timeout,
                                                 replace, id, corrId)
                      .c_str());
  }
  void injectMove(float v_left, float v_right, TestSupport::MoveStopKind stopKind,
                   float stopValue, float timeout, bool replace, uint32_t id,
                   uint32_t corrId = 0) {
    injectCommand(TestSupport::armorMoveCommand(v_left, v_right, stopKind, stopValue, timeout,
                                                 replace, id, corrId)
                      .c_str());
  }
  void injectStop(uint32_t corrId = 0) {
    injectCommand(TestSupport::armorStopCommand(corrId).c_str());
  }

  // motorConfig -- test-only readback of the Devices::MotorConfig last
  // passed to configureMotor() below for the given port (1=left, 2=right).
  // SimHarness's OWN record of the request, not a live re-read off
  // Devices::MotorArmor/NezhaMotor (neither stores a full copy). Defaults
  // to Devices::MotorConfig{} if configureMotor() was never called for
  // that port.
  const Devices::MotorConfig& motorConfig(uint32_t port) const {
    return (port == 2) ? lastMotorConfigR_ : lastMotorConfigL_;
  }

  // configureMotor -- the ONLY way a motor's config_ is ever set past its
  // construction default. port: 1 = left, 2 = right. reconfigure()
  // forwards the WHOLE config to the wrapped NezhaMotor and is
  // [[nodiscard]]/guarded (refuses while genuinely in motion); on a fresh
  // SimHarness it must always return true, so a false is asserted as a
  // real bug. Also load-bearing for RobotLoop's configuration-completeness
  // gate -- see maybeMarkConfigured()'s own comment below.
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
    // Teach the plant this port's mount orientation (mirror-mounted motor
    // correction) -- see SimPlant::setFwdSign()'s own comment.
    plant_.setFwdSign(static_cast<int>(port), cfg.fwdSign);
    maybeMarkConfigured();
  }

  // Test-only accessors exposing the STAGED PID-target velocity, NOT the
  // measured/decoded telemetry velocity -- used to measure the
  // post-completion "shelf" a stale nonzero COMMAND can leave.
  float driveTargetVelLeft() const { return armorL_.velocityTarget(); }    // [mm/s] signed
  float driveTargetVelRight() const { return armorR_.velocityTarget(); }  // [mm/s] signed

  // Decodes and returns every outbound line captured on the serial
  // FakeTransport since the last call (serial and radio receive an
  // IDENTICAL broadcast, so draining one is sufficient).
  std::vector<TestSupport::DecodedLine> drainTelemetry() {
    std::vector<TestSupport::DecodedLine> result;
    const auto& sent = serialLink_.sent();
    for (; telemetryDrainIndex_ < sent.size(); ++telemetryDrainIndex_) {
      result.push_back(TestSupport::decodeOutboundLine(sent[telemetryDrainIndex_]));
    }
    return result;
  }

  // Raw (still-armored "*B...") lines, own drain index (doesn't starve
  // drainTelemetry()) -- sim_ctypes.cpp's C ABI wants raw wire text.
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

  // Thin passthrough to App::RobotLoop's own configuration-completeness
  // gate. false immediately after construction; true only once both
  // configureMotor() calls have landed.
  bool isConfigured() const { return robotLoop_.isConfigured(); }

  // The composed SimPlant -- exposes fault knobs (setDisconnected()/
  // freezePosition()/setDropoutRate()/setOtosDrift()) and the read/write
  // hook registration (setReadHook()/setWriteHook()) directly.
  SimPlant& plant() { return plant_; }
  const SimPlant& plant() const { return plant_; }

  // True pose -- SimPlant's owned OtosPlant ground truth (see this file's
  // own header for why these three, specifically, are "the" true pose).
  float trueX() const { return plant_.otosPlant().x(); }              // [mm]
  float trueY() const { return plant_.otosPlant().y(); }              // [mm]
  float trueHeading() const { return plant_.otosPlant().heading(); }  // [rad]

  // Pose reset ("Set Robot @ 0,0"). Teleports the plant TRUTH AND resets
  // firmware's own encoder-derived state, so telemetry pose/otos actually
  // snap to (x,y,heading), not just the avatar.
  void setTruePose(float x, float y, float heading) {  // [mm] [mm] [rad]
    plant_.setTruePose(x, y, heading);
    motorL_.begin();
    motorR_.begin();
    odom_.reset(x, y, heading);
  }

  Devices::NezhaMotor& motorLeft() { return motorL_; }
  Devices::NezhaMotor& motorRight() { return motorR_; }

  // Exposes the owned App::StateEstimator; a test needing non-default
  // fusion weights calls stateEstimator().setWeights(...) directly.
  App::StateEstimator& stateEstimator() { return stateEstimator_; }

  // Concrete TestSim::SimClock&, not Devices::Clock& -- callers need the
  // setMicros()/advanceMicros() stepping surface only the concrete fake
  // exposes.
  TestSim::SimClock& clock() { return clock_; }

  // Concrete TestSim::SimSleeper&, for the same reason as clock() above --
  // exposes sleepCount()/lastSleepMillis()/yieldCount() for timing-
  // diagnostic scenarios.
  TestSim::SimSleeper& sleeper() { return sleeper_; }

  // [us] fixed per-cycle virtual-time advance step() applies before every
  // robotLoop_.cycle() call -- DERIVED from App::RobotLoop::kCycle, never
  // hardcoded (sim's step period must equal firmware's real control period
  // exactly, or every sim-tuned finding is measured on a materially
  // different control period than what ships). Pre-118 history (a
  // hand-picked 50ms that never was a deliberate fidelity choice):
  // src/sim/DESIGN.md.
  static constexpr uint32_t kCycleDtUs = App::RobotLoop::kCycle * 1000;  // [us]
  static_assert(kCycleDtUs == App::RobotLoop::kCycle * 1000,
                "SimHarness::kCycleDtUs must equal firmware's own App::RobotLoop::kCycle "
                "(converted ms->us) -- derive it, never hardcode a second matching literal "
                "that can drift apart silently (118 ticket 003)");

 private:
  // Drives App::Preamble to done() via preamble_.step() calls issued
  // OURSELVES, advancing the fake Clock between each one -- a single
  // robotLoop_.boot() call offers no opportunity to advance virtual time
  // between attempts, and color_/line_'s own retry pacing needs real
  // elapsed virtual time between them.
  void driveBootToDone() {
    clock_.setMicros(0);
    preamble_.step();  // arms Preamble's own startUs_ at 0 -- power-settle no-op

    clock_.setMicros(50000);  // >= Preamble::kPowerSettle -- probing starts on the NEXT step()

    // 200 passes at 50ms apart is a generous bound over color_/line_'s own
    // natural worst case; if ever exceeded, done() staying false is a real
    // bug, not a slow-but-fine boot.
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
  // app graph below takes the ARMOR, never the bare leaf.
  Devices::MotorArmor armorL_;
  Devices::MotorArmor armorR_;
  Devices::Otos otos_;
  Devices::ColorSensorLeaf color_;
  Devices::LineSensorLeaf line_;

  TestSupport::FakeTransport serialLink_;
  TestSupport::FakeTransport radioLink_;

  App::Comms comms_;
  App::Telemetry tlm_;
  App::Drive drive_;
  App::Odometry odom_;
  App::StateEstimator stateEstimator_;  // default-constructed, see ctor initializer list's own comment above
  // Declared AFTER drive_/odom_ (MoveQueue's constructor holds references to both).
  App::MoveQueue moveQueue_;
  App::Preamble preamble_;
  App::RobotLoop robotLoop_;

  bool booted_ = false;
  int cycleCount_ = 0;

  size_t telemetryDrainIndex_ = 0;  // index into serialLink_.sent() already returned by drainTelemetry()
  size_t rawTelemetryDrainIndex_ = 0;  // index into serialLink_.sent() already returned by drainRawTelemetry()

  // configureMotor()'s own test-only readback state -- see motorConfig().
  Devices::MotorConfig lastMotorConfigL_ = {};
  Devices::MotorConfig lastMotorConfigR_ = {};

  // The motor half of the configuration-completeness gate's tracking.
  bool hasConfiguredMotorL_ = false;
  bool hasConfiguredMotorR_ = false;

  // The whole graph is configured once BOTH configureMotor() calls have
  // landed, mirroring main.cpp's real boot-configure sequence. Idempotent.
  void maybeMarkConfigured() {
    if (hasConfiguredMotorL_ && hasConfiguredMotorR_) {
      robotLoop_.markConfigured();
    }
  }
};

}  // namespace TestSim
