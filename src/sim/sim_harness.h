// sim_harness.h -- TestSim::SimHarness: the composition root wiring the REAL
// App::RobotLoop firmware graph to a SimPlant (tests/_infra/sim/sim_plant.{h,cpp}).
// Supersedes tests/sim/support/sim_api.{h,cpp} (TestSim::SimApi + its
// DutyPredictor) -- full history: src/sim/DESIGN.md. THIN composition root
// -- no simulation logic (SimPlant/WheelPlant/OtosPlant), no firmware
// dispatch logic (App::RobotLoop, unmodified). 130-002 (unify-sim-and-
// robot-composition-roots.md): the whole App::/Motion:: graph is now built
// by the SAME App::composeRobot() (src/firm/app/boot_wiring.h) src/firm/
// main.cpp calls -- this class constructs the ONE leaf that differs
// (TestSim::SimPlant, in the Devices::I2CBus& slot main.cpp fills with a
// real Devices::MicroBitI2CBus) and hands everything else to that shared
// function. Before this ticket, this file hand-wired its own independent
// copy of the graph (simPlannerLimits()'s own hand-picked literals), which
// had already drifted from main.cpp's boot values once -- the wheel-trim
// gains booted at their fail-closed all-zero default in every sim session
// while live on every hardware session. That class of drift is now
// structurally impossible: this class calls composeRobot(), the graph is
// identical by construction, and the only two deliberately-different
// values (trackWidth, controlPeriod/actuationDelay) are explicit,
// commented BootOverrides passed at this file's own composeRobot() call
// site below -- never silent.
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
#include <cstring>
#include <string>
#include <vector>

#include "app/boot_wiring.h"
#include "app/robot_loop.h"
#include "devices/device_config.h"
#include "fake_transport.h"
#include "motion/odometry.h"
#include "motion/planner/planner.h"
#include "sim_clock.h"
#include "sim_plant.h"
#include "wire_test_codec.h"

namespace TestSim {

class SimHarness {
 public:
  // trackWidth: [mm] -- passed to BOTH the SimPlant's own OtosPlant and the
  // real App::Drive/App::Odometry instances App::composeRobot() constructs
  // (via a BootOverrides -- see this constructor's own comment below), so
  // the simulated OTOS chip and firmware's own odometry describe the same
  // wheelbase. Defaults to TestSim::kDefaultTrackWidth (SimPlant's own).
  explicit SimHarness(float trackWidth = kDefaultTrackWidth)
      : plant_(trackWidth),
        // PARITY (130-002): composeRobot() is the SAME function
        // src/firm/main.cpp calls -- the only leaf substituted here is the
        // bus (plant_, a TestSim::SimPlant, in main.cpp's
        // Devices::MicroBitI2CBus slot) and the transports (FakeTransport
        // in main.cpp's real SerialTransport/RadioTransport slots).
        // tuningStore is null: persistence is disabled in the sim, exactly
        // as the pre-composeRobot() RobotLoop's own null tuningStore was.
        //
        // THREE deliberate, explicit overrides (BootOverrides -- see its
        // own doc comment, app/boot_wiring.h, for the full rationale each):
        //   - trackWidth: this constructor's own fixture parameter, a
        //     property of the calling test's scenario, not a hardware
        //     calibration fact.
        //   - controlPeriod/actuationDelay (kSimControlPeriod, below): the
        //     sim's own step() advances virtual time by EXACTLY
        //     App::RobotLoop::kCycle every call, with none of a real
        //     board's vendor-bus-clearance overrun the robot JSON's own
        //     baked ~47ms accounts for (PlannerBootConfig::controlPeriod's
        //     own doc comment). Deriving both the override value AND
        //     kCycleDtUs (below) from the SAME App::RobotLoop::kCycle
        //     constant is what closes 130-002's own acceptance criterion
        //     ("the sim's step dt and the planner's controlPeriod/
        //     actuationDelay derive from the SAME constant").
        //   - otosConfig (kIdentityOtosConfig, below): TestSim::SimPlant's
        //     OtosPlant is already a perfect, zero-mounting-error sensor by
        //     construction -- the robot JSON's OtosConfig corrects a REAL
        //     chip's measured lever-arm/scale error, which has no
        //     counterpart to correct here (found empirically: applying it
        //     unconditionally put a 2s/150mm/s straight run's otos reading
        //     55.8mm off true ground truth in a test that explicitly zeroes
        //     every OTHER simulated OTOS fault knob).
        graph_(App::composeRobot(plant_, clock_, sleeper_, serialLink_, radioLink_,
                                 /*tuningStore=*/nullptr, "DEVICE:NEZHA2:sim:sim_harness:1",
                                 "ID:unknown",
                                 App::BootOverrides{&trackWidth, &kSimControlPeriod,
                                                    &kSimControlPeriod, &kIdentityOtosConfig})) {
    // SIM OVERRIDE: composeRobot() already installed App::Drive's
    // calibration via installDriveCalibration() (boot through the SAME
    // path main.cpp uses), baking Drive::kDutyPerSpeed -- the MEASURED
    // REAL-HARDWARE gearbox constant. That is wrong for THIS plant:
    // TestSim::WheelPlant is a fixed synthetic plant (velocity
    // kDefaultDutyVelMax at |duty| == 1), so its exact inverse is a fact
    // about the sim, not a per-robot measurement -- there is no robot JSON
    // to validate against. Override it here, explicitly, rather than
    // silently keeping the hardware value. Callers that DO have a robot
    // config still override this AGAIN: SimLoop.configure_from_robot()
    // pushes the JSON's own control.duty_per_speed_left/right through
    // sim_configure_drive(), the same values main.cpp installs on
    // hardware.
    //
    // setWheelCorrection() is deliberately left at composeRobot()'s own
    // installed value (identity: gain 1, intercept 0, from the baked
    // DriveBootConfig defaults) rather than overridden again here: it
    // linearizes a real gearbox (measured = gain*commanded + intercept)
    // and this plant is already linear, so identity is the correct value
    // for the sim too -- see sim_boot_config.py's drive_boot_config_for()
    // docstring.
    graph_.drive().setDutyPerSpeed(1.0f / kDefaultDutyVelMax, 1.0f / kDefaultDutyVelMax);

    // No further self-configuration -- motorL_/motorR_ stay at their default
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
    graph_.robotLoop().boot();
    booted_ = true;
  }

  // Advances the sim `cycles` times: plant_.tick(dt) THEN clock_.
  // advanceMicros(kCycleDtUs) THEN robotLoop_.cycle() -- see this file's
  // own header for the ordering invariant. Call boot() first.
  void step(int cycles = 1) {
    for (int i = 0; i < cycles; ++i) {
      plant_.tick(static_cast<float>(kCycleDtUs) / 1e6f);  // [s]
      clock_.advanceMicros(kCycleDtUs);
      graph_.robotLoop().cycle();
      ++cycleCount_;
    }
  }

  // Pushes one complete `<COMMAND>':'<COBS+CRC bytes>` wire LINE (124-005 --
  // was a bare COBS+CRC frame body pre-124-005, itself was armored "*B..."
  // text pre-123) onto the inbound serial FakeTransport. App::Comms::pump()
  // now DRAINS both transports (command-ingestion-ring-buffered-comms-
  // subsystem-routing-two-stops.md §1) and RobotLoop routes the whole ring
  // once per cycle, so N lines injected before one step() are all consumed
  // by that step -- NOT one per cycle() as this comment used to say. `frame`/`len` is always a
  // TestSupport::armorMoveCommand()/armorStopCommand()-built line -- an
  // EXPLICIT length, never recovered via strlen(): COBS is now keyed on
  // 0x0A (wire_runtime.h item 8), not 0x00, so the line may legitimately
  // contain an embedded 0x00 byte that strlen() would truncate at.
  void injectCommand(const char* frame, size_t len) {
    serialLink_.enqueueInboundBinary(reinterpret_cast<const uint8_t*>(frame), len);
  }
  void injectCommand(const std::string& frame) { injectCommand(frame.data(), frame.size()); }

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
                                                 replace, id, corrId));
  }
  void injectMove(float v_left, float v_right, TestSupport::MoveStopKind stopKind,
                   float stopValue, float timeout, bool replace, uint32_t id,
                   uint32_t corrId = 0) {
    injectCommand(TestSupport::armorMoveCommand(v_left, v_right, stopKind, stopValue, timeout,
                                                 replace, id, corrId));
  }
  // injectStop() -- the PLANNED stop (command-ingestion-ring-buffered-
  // comms-subsystem-routing-two-stops.md §2): a planner queue entry that
  // executes in sequence, NOT a panic stop. `id` is its completion-ack key.
  void injectStop(uint32_t corrId = 0, uint32_t id = 0) {
    injectCommand(TestSupport::armorStopCommand(corrId, id));
  }

  // injectEstop() -- "halt now, everywhere" (§2/§3). THIS is what
  // injectStop() meant before the command-ingestion rework; a caller that
  // wanted the drivetrain zeroed immediately belongs here.
  void injectEstop(uint32_t corrId = 0) {
    injectCommand(TestSupport::armorEstopCommand(corrId));
  }

  // injectWheels() -- the dumb teleop primitive (§2): straight to
  // App::Drive, superseding the planner, held for `duration` ms.
  void injectWheels(float vLeft, float vRight, float duration,  // [mm/s] [mm/s] [ms]
                    uint32_t id = 0, uint32_t corrId = 0) {
    injectCommand(TestSupport::armorWheelsCommand(vLeft, vRight, duration, id, corrId));
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
      bool applied = graph_.armorRight().reconfigure(cfg);
      assert(applied && "armorR_.reconfigure() refused on a fresh SimHarness -- real bug, not expected");
      (void)applied;
      hasConfiguredMotorR_ = true;
    } else {
      lastMotorConfigL_ = cfg;
      bool applied = graph_.armorLeft().reconfigure(cfg);
      assert(applied && "armorL_.reconfigure() refused on a fresh SimHarness -- real bug, not expected");
      (void)applied;
      hasConfiguredMotorL_ = true;
    }
    // Teach the plant this port's mount orientation (mirror-mounted motor
    // correction) -- see SimPlant::setFwdSign()'s own comment.
    plant_.setFwdSign(static_cast<int>(port), cfg.fwdSign);
    maybeMarkConfigured();
  }

  // Test-only accessors exposing the STAGED target velocity Drive last
  // received via setDuty() (125-003: NOT a live Devices::Motor::
  // velocityTarget() read -- that accessor is gone, since NezhaMotor no
  // longer tracks a velocity target at all; Drive's own vLeft_/vRight_ is
  // now the one place "what was commanded" lives), NOT the measured/decoded
  // telemetry velocity -- used to measure the post-completion "shelf" a
  // stale nonzero COMMAND can leave.
  // Planner integration (2026-07-26): the staged wheel-velocity target
  // lives on the planner now (Drive carries duty in raw mode).
  App::RobotLoop& robotLoop() { return graph_.robotLoop(); }

  float driveTargetVelLeft() const { return graph_.robotLoop().state().wheelLeft.cmdVelocity; }    // [mm/s] signed
  float driveTargetVelRight() const { return graph_.robotLoop().state().wheelRight.cmdVelocity; }  // [mm/s] signed

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

  // 125-006: cleartext replies (HELLO's DEVICE: banner, PING/PONG, ID,
  // VER, READY, STATUS, HELP) ride App::Transport::sendReliable() --
  // App::Comms's OWN send-path split, comms.h's file header -- which lands
  // in FakeTransport's separate sentReliable_ capture, never sent_ (that
  // one is send()'s own capture: Telemetry's armored binary frames only).
  // drainTelemetry()/drainRawTelemetry() above therefore NEVER see a
  // STATUS/HELP/READY line -- a caller that needs one (e.g. a TLM:ON/OFF
  // mode-change's own STATUS reply) drains this instead. Own drain index,
  // symmetric with the other two drains.
  std::vector<std::string> drainReliable() {
    std::vector<std::string> result;
    const auto& sent = serialLink_.sentReliable();
    for (; reliableDrainIndex_ < sent.size(); ++reliableDrainIndex_) {
      result.push_back(sent[reliableDrainIndex_]);
    }
    return result;
  }

  bool booted() const { return booted_; }
  int cycleCount() const { return cycleCount_; }  // total robotLoop_.cycle() calls made so far

  // Thin passthrough to App::RobotLoop's own configuration-completeness
  // gate. false immediately after construction; true only once both
  // configureMotor() calls have landed.
  bool isConfigured() const { return graph_.robotLoop().isConfigured(); }

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
    graph_.motorLeft().begin();
    graph_.motorRight().begin();
    // 122-002: Motion::Odometry::reset() takes the CURRENT leaf positions
    // explicitly (both leaves already reset to 0 by begin() above, same
    // values the pre-122-002 reset() read internally at this exact point).
    graph_.odometry().reset(x, y, heading, graph_.motorLeft().position(),
                            graph_.motorRight().position());
  }

  Devices::NezhaMotor& motorLeft() { return graph_.motorLeft(); }
  Devices::NezhaMotor& motorRight() { return graph_.motorRight(); }

  // drive -- 125-003: App::Drive now holds the interim closed-loop gains
  // (drive.h's own header) a test needs to push directly
  // (drive().applyGainsLeft()/applyGainsRight()) or read back
  // (drive().gainsLeft()/gainsRight()) -- the CONFIG-patch routing split
  // this sprint's own RobotLoop::applyMotorConfigPatch() implements.
  App::Drive& drive() { return graph_.drive(); }
  Motion::Planner& planner() { return graph_.planner(); }

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
  // src/sim/DESIGN.md. 130-002: kSimControlPeriod (the PlannerLimits
  // controlPeriod/actuationDelay override passed to composeRobot(), this
  // file's own constructor) derives from the SAME App::RobotLoop::kCycle
  // constant as this -- one source, not two hand-kept literals that can
  // drift apart silently.
  static constexpr uint32_t kCycleDtUs = App::RobotLoop::kCycle * 1000;  // [us]
  static_assert(kCycleDtUs == App::RobotLoop::kCycle * 1000,
                "SimHarness::kCycleDtUs must equal firmware's own App::RobotLoop::kCycle "
                "(converted ms->us) -- derive it, never hardcode a second matching literal "
                "that can drift apart silently (118 ticket 003)");

 private:
  // See this constructor's own composeRobot() call comment above -- the
  // sim's genuinely-justified controlPeriod/actuationDelay override.
  static constexpr float kSimControlPeriod = static_cast<float>(App::RobotLoop::kCycle);  // [ms]

  // See this constructor's own composeRobot() call comment above -- the
  // sim's genuinely-justified otosConfig override. Devices::OtosConfig's
  // own default member initializers (device_config.h) ARE identity (zero
  // offset, 1.0 scale), so a plain default-constructed instance is exactly
  // the value this override needs.
  static constexpr Devices::OtosConfig kIdentityOtosConfig{};

  // Drives App::Preamble to done() via preamble_.step() calls issued
  // OURSELVES, advancing the fake Clock between each one -- a single
  // robotLoop_.boot() call offers no opportunity to advance virtual time
  // between attempts, and color_/line_'s own retry pacing needs real
  // elapsed virtual time between them.
  void driveBootToDone() {
    clock_.setMicros(0);
    graph_.preamble().step();  // arms Preamble's own startUs_ at 0 -- power-settle no-op

    clock_.setMicros(50000);  // >= Preamble::kPowerSettle -- probing starts on the NEXT step()

    // 200 passes at 50ms apart is a generous bound over color_/line_'s own
    // natural worst case; if ever exceeded, done() staying false is a real
    // bug, not a slow-but-fine boot.
    for (int i = 0; i < 200 && !graph_.preamble().done(); ++i) {
      graph_.preamble().step();
      clock_.advanceMicros(50000);
    }
  }

  SimPlant plant_;
  TestSim::SimClock clock_;
  TestSim::SimSleeper sleeper_;
  TestSupport::FakeTransport serialLink_;
  TestSupport::FakeTransport radioLink_;

  // The WHOLE App::/Motion:: graph -- see this file's own header and this
  // constructor's own comment for why this is the SAME composeRobot() call
  // src/firm/main.cpp makes, substituting only plant_/serialLink_/
  // radioLink_ for main.cpp's real hardware leaves.
  App::RobotGraph graph_;

  bool booted_ = false;
  int cycleCount_ = 0;

  size_t telemetryDrainIndex_ = 0;  // index into serialLink_.sent() already returned by drainTelemetry()
  size_t rawTelemetryDrainIndex_ = 0;  // index into serialLink_.sent() already returned by drainRawTelemetry()
  size_t reliableDrainIndex_ = 0;  // index into serialLink_.sentReliable() already returned by drainReliable()

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
      graph_.robotLoop().markConfigured();
    }
  }
};

}  // namespace TestSim
