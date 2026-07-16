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
#include "app/odometry.h"
#include "app/preamble.h"
#include "app/robot_loop.h"
#include "app/telemetry.h"
#include "devices/color_sensor.h"
#include "devices/device_config.h"
#include "devices/line_sensor.h"
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
        otos_(plant_, Devices::OtosConfig{}),
        color_(plant_, Devices::ColorConfig{}),
        line_(plant_, Devices::LineConfig{}),
        comms_(serialLink_, radioLink_, "DEVICE:NEZHA2:sim:sim_harness:1"),
        tlm_(comms_, serialLink_, radioLink_),
        deadman_(clock_),
        drive_(motorL_, motorR_, trackWidth),
        odom_(motorL_, motorR_, trackWidth),
        preamble_(motorL_, motorR_, otos_, color_, line_, clock_),
        robotLoop_(plant_, motorL_, motorR_, otos_, comms_, tlm_, drive_, odom_,
                   deadman_, preamble_, clock_, sleeper_) {
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

  // Plant teleport -- thin call-through to SimPlant::setTruePose(). See
  // that method's own comment for why the OtosPlant re-baseline and the
  // WheelPlant position resets it performs must happen together.
  void setTruePose(float x, float y, float heading) {  // [mm] [mm] [rad]
    plant_.setTruePose(x, y, heading);
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
    cfg.wheelTravelCalib = 1.0f;
    cfg.velFiltAlpha = 1.0f;
    cfg.slewRate = 100.0f;
    cfg.velGains.kp = 0.01f;
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

  App::RobotLoop robotLoop_;

  bool booted_ = false;
  int cycleCount_ = 0;

  size_t telemetryDrainIndex_ = 0;  // index into serialLink_.sent() already returned by drainTelemetry()
  size_t rawTelemetryDrainIndex_ = 0;  // index into serialLink_.sent() already returned by drainRawTelemetry()
};

}  // namespace TestSim
