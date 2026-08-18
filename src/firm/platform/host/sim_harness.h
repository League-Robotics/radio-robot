// sim_harness.h -- TestSim::SimHarness: the composition root wiring the REAL
// Core::RobotLoop firmware graph to a SimPlant (tests/_infra/sim/sim_plant.{h,cpp}).
// Supersedes tests/sim/support/sim_api.{h,cpp} (TestSim::SimApi + its
// DutyPredictor) -- full history: src/firm/platform/host/DESIGN.md. THIN composition root
// -- no simulation logic (SimPlant/WheelPlant/OtosPlant), no firmware
// dispatch logic (Core::RobotLoop, unmodified). 130-002 (unify-sim-and-
// robot-composition-roots.md): the whole Core::/Motion:: graph is now built
// by the SAME Core::composeRobot() (src/firm/app/boot_wiring.h) src/firm/
// main.cpp calls -- this class constructs the ONE leaf that differs
// (TestSim::SimPlant, in the Hal::I2CBus& slot main.cpp fills with a
// real Platform::MicroBitI2CBus) and hands everything else to that shared
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
// cycle(), never the other order (src/firm/platform/host/DESIGN.md's "Invariants worth
// keeping" #1; SimPlant::tick()'s own doc comment has the physics-side
// half). Two entry points, matching RobotLoop's own boot()/cycle() split --
// boot() drives Core::Preamble to done() via direct preamble_.step() calls
// (see driveBootToDone()'s own comment for why not a single
// robotLoop_.boot() call), then calls the real robotLoop_.boot(); step(n)
// runs n cycles of (plant_.tick(dt); clock_.advanceMicros(dt);
// robotLoop_.cycle()). Call boot() first.
//
// trueX()/trueY()/trueHeading() read SimPlant's owned OtosPlant ground
// truth directly -- NOT the wire-visible reportedX/Y/Heading() (applies the
// OTOS drift/bias fault knob) and NOT Core::Odometry's own independently
// integrated pose -- for test assertions that must bypass sensor noise.
#pragma once

#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "core/boot_wiring.h"
#include "core/robot_loop.h"
#include "hal/device_config.h"
#include "fake_transport.h"
#include "host_fiber.h"
#include "sim_clock.h"
#include "sim_plant.h"
#include "wire_test_codec.h"

namespace TestSim {

class SimHarness {
 public:
  // trackWidth: [mm] -- passed to BOTH the SimPlant's own OtosPlant and the
  // real Control::DifferentialDrive/Core::Odometry instances Core::composeRobot() constructs
  // (via a BootOverrides -- see this constructor's own comment below), so
  // the simulated OTOS chip and firmware's own odometry describe the same
  // wheelbase. Defaults to TestSim::kDefaultTrackWidth (SimPlant's own).
  //
  // tuningStore: nullptr (the default) keeps persistence disabled, exactly
  // as the pre-composeRobot() RobotLoop's own null tuningStore was --
  // every pre-132-015 caller of this constructor is unaffected. 132-015
  // (trap 1, the-configuration-object.md) added this parameter so a test
  // can inject a real (or seeded-mock) Config::TuningStore and exercise
  // loadPersistedTuning() through the SAME composition root main.cpp uses
  // (see loadPersistedTuning() below for the call this enables).
  explicit SimHarness(float trackWidth = kDefaultTrackWidth,
                      Config::TuningStore* tuningStore = nullptr)
      : plant_(trackWidth),
        // PARITY: composeRobot() is the SAME function src/firm/main.cpp
        // calls -- the only leaf substituted here is the bus (plant_, a
        // TestSim::SimPlant, in main.cpp's Platform::MicroBitI2CBus slot)
        // and the transports (FakeTransport in main.cpp's real
        // Platform::MicroBitSerialPort/MicroBitRadioLink slots).
        //
        // EXPLORATORY-KERNEL REWRITE (2026-08-15): trackWidth/
        // controlPeriod/actuationDelay/navigatorYawSign are GONE from
        // BootOverrides -- Control::DifferentialDrive's kernel takes no
        // trackWidth parameter at all, and controlPeriod/actuationDelay/
        // navigatorYawSign fed Motion::PlannerLimits/Motion::
        // NavigatorLimits, both deleted with src/firm/motion/. `trackWidth`
        // (this constructor's own fixture parameter) now feeds ONLY
        // plant_'s own OtosPlant construction above -- it is not part of
        // the firmware graph any more.
        //
        // TWO deliberate, explicit overrides remain (BootOverrides -- see
        // its own doc comment, core/boot_wiring.h, for the full rationale
        // each):
        //   - otosConfig (kIdentityOtosConfig, below): TestSim::SimPlant's
        //     OtosPlant is already a perfect, zero-mounting-error sensor.
        //   - wheelCorrection (kIdentityWheelCorrection, below):
        //     TestSim::WheelPlant is linear, so identity is the only
        //     correct Stage A correction here.
        graph_(Core::composeRobot(plant_, clock_, sleeper_, fiberLauncher_,
                                 serialLink_, radioLink_,
                                 tuningStore, "DEVICE:NEZHA2:sim:sim_harness:1",
                                 "ID:unknown",
                                 Core::BootOverrides{&kIdentityOtosConfig,
                                                    &kIdentityWheelCorrection})) {
    // THE PLANT OBEYS THE CONFIG -- the one invariant that keeps this sim
    // honest, learned twice over (see syncPlantToConfig() below). Applied
    // here for the BAKED config; re-applied every step() so live wire
    // pushes keep it true.
    syncPlantToConfig();

    // No further self-configuration -- motorL_/motorR_ stay at their default
    // Hal::MotorConfig{} (all-zero), matching a real, not-yet-booted
    // composition root. A caller MUST call configureMotor() for BOTH ports
    // (or TestSupport::configureSimForBenchTest()) before commanding a
    // MOVE -- see maybeMarkConfigured()'s own comment below. "Pre-boot
    // state": everything above is constructed and wired, but
    // Core::Preamble::step() has not yet been called -- boot() is the
    // caller's job, mirroring main.cpp's own construct-then-boot split.
  }

  // Drives Core::Preamble to done(), then calls the real robotLoop_.boot(),
  // then drive_.begin() -- priming both encoders + arming the boot
  // zero-write, mirroring main.cpp's own post-boot() sequencing exactly
  // (core/boot_wiring.h's "Lifecycle, one level up" note). Deliberately
  // does NOT call drive_.start() -- a host harness has no real
  // Hal::FiberLauncher and drives the kernel's step() itself instead
  // (see step() below). Idempotent -- a second call is a no-op (booted_
  // already true).
  void boot() {
    if (booted_) return;
    driveBootToDone();
    graph_.robotLoop().boot();
    graph_.drive().begin();
    booted_ = true;
  }

  // Advances the sim `cycles` times: plant_.tick(dt) THEN clock_.
  // advanceMicros(kCycleDtUs) THEN drive_.step() THEN robotLoop_.
  // cycle() -- see this file's own header for the plant-before-loop
  // ordering invariant. Call boot() first.
  //
  // drive_.step() runs ONCE per outer step here, not on its own
  // kKernelCyclePeriod cadence (the kernel has no fiber in a host
  // harness -- there is nothing else to pace it). This is a known
  // simplification: the kernel's own internal dt terms read ~kCycleDtUs
  // per step instead of the real fiber's ~24ms, which is fine for the
  // sim's deterministic dt-parametric math (every stage takes dt
  // explicitly) but does NOT reproduce the kernel's real multi-rate
  // relationship to RobotLoop -- a host-build harness for the kernel's
  // OWN control math (stepped against the plant directly, matching this
  // file's own header note on a `src/tests/sim/plant/`-precedent harness)
  // is the right tool for characterizing that, not this composition-level
  // harness.
  void step(int cycles = 1) {
    for (int i = 0; i < cycles; ++i) {
      syncPlantToConfig();
      plant_.tick(static_cast<float>(kCycleDtUs) / 1e6f);  // [s]
      clock_.advanceMicros(kCycleDtUs);
      graph_.drive().step();
      graph_.robotLoop().cycle();
      ++cycleCount_;
    }
  }

  // Pushes one complete `<COMMAND>':'<COBS+CRC bytes>` wire LINE (124-005 --
  // was a bare COBS+CRC frame body pre-124-005, itself was armored "*B..."
  // text pre-123) onto the inbound serial FakeTransport. Core::Comms::pump()
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
  // Control::DifferentialDrive, superseding the planner, held for `duration` ms.
  void injectWheels(float vLeft, float vRight, float duration,  // [mm/s] [mm/s] [ms]
                    uint32_t id = 0, uint32_t corrId = 0) {
    injectCommand(TestSupport::armorWheelsCommand(vLeft, vRight, duration, id, corrId));
  }

  // injectBodyTwist() -- a body twist (v_x, omega) expressed as the WHEELS
  // command, which is the only motion arm left: MOVE and GO_TO are both
  // DEREGISTERED in this tree (handleMoveOrGoto() acks ERR_UNIMPLEMENTED),
  // so every harness that used to say injectMove(v_x, 0, omega, ...) says
  // this instead.
  //
  // The differential inverse-kinematics done here is deliberately NOT in
  // the kernel: Control::DifferentialDrive is counts-native and knows no
  // chassis geometry at all. Resolving a body twist into per-wheel speeds
  // is the APPLICATION's job now, and a test harness is an application.
  // Reads the SAME baked trackwidth the firmware does, so a harness and the
  // robot cannot disagree about what "omega" meant.
  //
  //   vLeft  = v_x - omega * b/2
  //   vRight = v_x + omega * b/2      (REP-103: +omega is CCW)
  void injectBodyTwist(float v_x, float omega, float duration,  // [mm/s] [rad/s] [ms]
                       uint32_t id = 0, uint32_t corrId = 0) {
    const float trackwidth = graph_.configurator().config().geometry.trackwidth;  // [mm]
    const float halfTrack = 0.5f * trackwidth;
    injectWheels(v_x - omega * halfTrack, v_x + omega * halfTrack, duration, id, corrId);
  }

  // injectGoto() -- GO_TO is DEREGISTERED in this tree (ERR_UNIMPLEMENTED);
  // kept only so the protocol-level "a deregistered arm still acks, and acks
  // an ERROR" coverage can still send one. Not a motion primitive any more.
  // `frame`: 0 = WORLD (OTOS/SEED frame), 1 = ROBOT.
  void injectGoto(float x, float y, uint32_t frame, float speed, float arrive,  // [mm] [mm] [] [mm/s] [mm]
                  float timeout, uint32_t id = 0, uint32_t corrId = 0) {  // [ms]
    injectCommand(TestSupport::armorGotoCommand(x, y, frame, speed, arrive, timeout, id, corrId));
  }

  // motorConfig -- test-only readback of the Hal::MotorConfig last
  // passed to configureMotor() below for the given port (1=left, 2=right).
  // SimHarness's OWN record of the request, not a live re-read off
  // Hardware::MotorArmor/NezhaMotor (neither stores a full copy). Defaults
  // to Hal::MotorConfig{} if configureMotor() was never called for
  // that port.
  const Hal::MotorConfig& motorConfig(uint32_t port) const {
    return (port == 2) ? lastMotorConfigR_ : lastMotorConfigL_;
  }

  // configureMotor -- the ONLY way a motor's config_ is ever set past its
  // construction default. port: 1 = left, 2 = right. reconfigure()
  // forwards the WHOLE config to the wrapped NezhaMotor and is
  // [[nodiscard]]/guarded (refuses while genuinely in motion); on a fresh
  // SimHarness it must always return true, so a false is asserted as a
  // real bug. Also load-bearing for RobotLoop's configuration-completeness
  // gate -- see maybeMarkConfigured()'s own comment below.
  void configureMotor(uint32_t port, const Hal::MotorConfig& cfg) {
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

  Core::RobotLoop& robotLoop() { return graph_.robotLoop(); }

  // driveTargetVelLeft()/driveTargetVelRight() -- DELETED (exploratory-
  // kernel rewrite): read Types::RobotState::Wheel::cmdVelocity, which no
  // longer exists (the kernel's own Command mailbox is the one place
  // "what was commanded" lives now, and it is private). A caller that
  // needs the CURRENTLY-COMMANDED (velocity, twist) pair has no
  // replacement accessor on this class -- only the MEASURED
  // drive().output().velocityLeft/Right survive on the public interface.

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
  // VER, READY, STATUS, HELP) ride Core::Transport::sendReliable() --
  // Core::Comms's OWN send-path split, comms.h's file header -- which lands
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

  // Thin passthrough to Core::RobotLoop's own configuration-completeness
  // gate. false immediately after construction; true only once both
  // configureMotor() calls have landed.
  bool isConfigured() const { return graph_.robotLoop().isConfigured(); }

  // Thin passthrough to Core::RobotGraph::loadPersistedTuning() (132-015,
  // trap 1) -- NOT called automatically by boot() above, deliberately: a
  // caller drives this explicitly, in whatever order relative to boot() it
  // wants to exercise (main.cpp's own fixed order is boot() THEN this
  // call -- see main.cpp's own comment for why). A no-op if this
  // SimHarness was constructed with the default null tuningStore.
  void loadPersistedTuning() { graph_.loadPersistedTuning(); }

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

  // Pose reset ("Set Robot @ 0,0"). Teleports the plant TRUTH and resets
  // both motor leaves' own encoder position via begin() (the leaf's own
  // hardReset() re-prime), so the wire-visible position starts at 0
  // matching the teleport. The encoder-odometry half this used to also
  // reset (Motion::Odometry::reset()) is GONE with motion/ -- pose is
  // OTOS-only now, and plant_.setTruePose() above already snapped the
  // OtosPlant ground truth, which is what state_.otos/state_.pose read.
  void setTruePose(float x, float y, float heading) {  // [mm] [mm] [rad]
    plant_.setTruePose(x, y, heading);
    graph_.motorLeft().begin();
    graph_.motorRight().begin();
  }

  Hardware::NezhaMotor& motorLeft() { return graph_.motorLeft(); }
  Hardware::NezhaMotor& motorRight() { return graph_.motorRight(); }

  // drive -- Control::DifferentialDrive is the whole wheel kernel now
  // (differential_drive.h's own header): config()/setConfig() (whole-block
  // Stage A/B/C tuning) and output() (measured snapshot) a test needs to
  // push directly or read back. planner()/navigator() accessors are GONE
  // -- Motion::Planner/Motion::Navigator no longer exist.
  Control::DifferentialDrive& drive() { return graph_.drive(); }

  // configurator -- Core::Configurator owns the one
  // Config::Robot instance (config()/loadBaked()/install(), configurator.h)
  // -- exposed the same way drive()/planner() already are, so a test can
  // read back what composeRobot() baked without reaching into graph_
  // itself (private).
  Core::Configurator& configurator() { return graph_.configurator(); }

  // Concrete TestSim::SimClock&, not Hal::Clock& -- callers need the
  // setMicros()/advanceMicros() stepping surface only the concrete fake
  // exposes.
  TestSim::SimClock& clock() { return clock_; }

  // Concrete TestSim::SimSleeper&, for the same reason as clock() above --
  // exposes sleepCount()/lastSleepMillis()/yieldCount() for timing-
  // diagnostic scenarios.
  TestSim::SimSleeper& sleeper() { return sleeper_; }

  // [us] fixed per-cycle virtual-time advance step() applies before every
  // robotLoop_.cycle() call. NOTE (exploratory-kernel rewrite): this no
  // longer has a Motion::PlannerLimits controlPeriod/actuationDelay
  // override to stay derived-together with -- both are gone with
  // motion/. It remains derived from Core::RobotLoop::kCycle because
  // that is still the right cadence for stepping RobotLoop itself; the
  // kernel's own cadence (kKernelCyclePeriod, boot_calibration.h) is a
  // separate, un-synced constant now -- see step()'s own doc comment for
  // the resulting simplification.
  static constexpr uint32_t kCycleDtUs = Core::RobotLoop::kCycle * 1000;  // [us]

 private:
  // See this constructor's own composeRobot() call comment above -- the
  // sim's genuinely-justified otosConfig override. Hal::OtosConfig's
  // own default member initializers (device_config.h) ARE identity (zero
  // offset, 1.0 scale), so a plain default-constructed instance is exactly
  // the value this override needs.
  static constexpr Hal::OtosConfig kIdentityOtosConfig{};

  // See this constructor's own composeRobot() call comment above -- the
  // sim's genuinely-justified wheelCorrection override.
  // Config::WheelCorrection's own default member initializers
  // (config/boot_config.h) ARE identity (gain 1, intercept 0), so a plain
  // default-constructed instance is exactly the value this override needs.
  static constexpr Config::WheelCorrection kIdentityWheelCorrection{};

  // Drives Core::Preamble to done() via preamble_.step() calls issued
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

  // syncPlantToConfig() -- make the synthetic plant BE the robot the LIVE
  // config describes, per wheel:
  //
  //     plant gain     = 1 / duty_per_speed   [mm/s at full duty]
  //     encoder scale  = 1 / travel_calib     [counts emitted per mm]
  //
  // With those two identities the open-loop velocity map and the
  // perception chain are EXACT for any robot JSON, by construction:
  // commanded v -> counts (x10/tc) -> duty (/fdv, where baked fdv is
  // 10/(tc*dps)) -> duty = v*dps -> plant velocity = duty/dps = v. The
  // config file stops being a description that can disagree with the
  // plant and becomes the plant's own parameters.
  //
  // WHY LIVE, EVERY CYCLE, and not once at construction: both halves of
  // this invariant have been broken by construction-time-only derivation.
  // First the encoder scale was a hardcoded literal that drifted from the
  // baked travel_calib (11.2% perception error, a 704 mm square-tour
  // closure miss). Then the constructor-time fullDutyVelocity override
  // was silently clobbered by configure_from_robot()'s wire pushes
  // (open-loop map 0.66x, integrator railed at iMax, wheels 17% slow with
  // every subsystem testing clean in isolation). A per-cycle re-sync from
  // the configurator's live config is a few float writes and makes the
  // whole drift class structurally impossible.
  //
  // Port mapping follows the config's own left_port/right_port (tovez is
  // wired port 1 = RIGHT), defaulting 1/2 when unset -- the same rule
  // gen_boot_config.py bakes with.
  //
  // Deliberately NOT emulated: Stage A wheel_gain/intercept nonlinearity
  // (this harness pins identity via kIdentityWheelCorrection); a config
  // whose gains are non-identity describes a nonlinear plant this linear
  // WheelPlant cannot be. All shipped sim profiles carry identity gains.
  void syncPlantToConfig() {
    const Config::Robot& cfg = graph_.configurator().config();
    const uint32_t leftPort = cfg.motors.left_port != 0 ? cfg.motors.left_port : 1u;
    const uint32_t rightPort = cfg.motors.right_port != 0 ? cfg.motors.right_port : 2u;
    if (cfg.motors.travel_calib_left > 0.0f) {
      plant_.setEncoderCountsPerMm(static_cast<int>(leftPort),
                                   1.0f / cfg.motors.travel_calib_left);
    }
    if (cfg.motors.travel_calib_right > 0.0f) {
      plant_.setEncoderCountsPerMm(static_cast<int>(rightPort),
                                   1.0f / cfg.motors.travel_calib_right);
    }
    if (cfg.drive.duty_per_speed_left > 0.0f) {
      plant_.setDutyVelMax(static_cast<int>(leftPort),
                           1.0f / cfg.drive.duty_per_speed_left);
    }
    if (cfg.drive.duty_per_speed_right > 0.0f) {
      plant_.setDutyVelMax(static_cast<int>(rightPort),
                           1.0f / cfg.drive.duty_per_speed_right);
    }
  }

  SimPlant plant_;
  TestSim::FailingFiberLauncher fiberLauncher_;
  TestSim::SimClock clock_;
  TestSim::SimSleeper sleeper_;
  TestSupport::FakeTransport serialLink_;
  TestSupport::FakeTransport radioLink_;

  // The WHOLE Core:: graph -- see this file's own header and this
  // constructor's own comment for why this is the SAME composeRobot() call
  // src/firm/main.cpp makes, substituting only plant_/serialLink_/
  // radioLink_ for main.cpp's real hardware leaves.
  Core::RobotGraph graph_;

  bool booted_ = false;
  int cycleCount_ = 0;

  size_t telemetryDrainIndex_ = 0;  // index into serialLink_.sent() already returned by drainTelemetry()
  size_t rawTelemetryDrainIndex_ = 0;  // index into serialLink_.sent() already returned by drainRawTelemetry()
  size_t reliableDrainIndex_ = 0;  // index into serialLink_.sentReliable() already returned by drainReliable()

  // configureMotor()'s own test-only readback state -- see motorConfig().
  Hal::MotorConfig lastMotorConfigL_ = {};
  Hal::MotorConfig lastMotorConfigR_ = {};

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
