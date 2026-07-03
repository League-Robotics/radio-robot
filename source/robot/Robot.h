#pragma once
#include "Config.h"
#include "Hardware.h"
#include "hal/capability/IVelocityMotor.h"
#include "hal/capability/IOdometer.h"
#include "hal/capability/ILineSensor.h"
#include "hal/capability/IColorSensor.h"
#include "hal/capability/IPositionMotor.h"
#include "hal/capability/IPortIO.h"
#include "MotorController.h"
#include "PhysicalStateEstimate.h"
#include "OtosCommands.h"
#include "PortController.h"
#include "ServoController.h"
#include "Inputs.h"
#include "Protocol.h"
#include "../types/CommandTypes.h"
#include "../robot/ConfigRegistry.h"
#include "../control/HaltController.h"
#include "../superstructure/Superstructure.h"
#include "MotionCommands.h"
// Phase E (043-001): thin sensor subsystems owning the timed LINE/COLOUR/PORTS
// reads.  Each is a value member declared after the device-interface ref it binds.
#include "../subsystems/sensors/LineSensor.h"
#include "../subsystems/sensors/ColorSensor.h"
#include "../subsystems/sensors/Ports.h"
// Phase E (043-003): Gripper subsystem — structural seam for the optional servo
// actuator (+ GripperIONull null-object).  periodic()/updateInputs() are no-ops;
// NOT wired into loopTickOnce this sprint (gripper is command-driven via
// ServoController).  Value member binds the existing `gripper` IServo ref.
#include "../subsystems/gripper/Gripper.h"
// Phase 3 (059-004): new message-contract subsystems — Drive, Sensors, and
// Planner.  ADDITIVE: constructed alongside the existing subsystems;
// configure() called in the Robot constructor.  Drive requires its own
// BodyVelocityController member (Robot gains one named bvc to avoid
// shadowing Planner's internal _bvc).
#include "../control/BodyVelocityController.h"
#include "../subsystems/drive/Drive.h"
#include "../subsystems/sensors/Sensors.h"
#include "../superstructure/Planner.h"

// Forward declarations — keeps the header-graph shallow.
class DebugCommands;
class LoopScheduler;
// SimCommands (069-003) — sim-build-only Commandable; forward-declared only,
// exactly like DebugCommands above.  buildCommandTable() takes it as an
// optional Commandable* the ARM build never constructs and never includes
// SimCommands.h for (see SystemCommands.cpp's HOST_BUILD-guarded include).
class SimCommands;
struct Robot;

// ---------------------------------------------------------------------------
// RobotSysCtx — context bundle for system command handlers (HELLO, PING, …).
// handlerCtx for system CommandDescriptors is a RobotSysCtx*.
// ---------------------------------------------------------------------------
struct RobotSysCtx {
    Robot*         robot;
    LoopScheduler* sched;
};

/**
 * Robot — open struct that owns and wires all robot firmware subsystems.
 *
 * Replaces the Robot facade class (sprint 016).  All subsystem members are
 * public: there are no invariants to protect at this level — each subsystem
 * class protects its own.  Caller code reaches subsystems directly instead
 * of going through forwarding methods.
 *
 * Hardware is provided through a Hardware& (NezhaHAL on device, MockHAL in
 * host tests). Robot binds interface refs from hal.motorL() etc.
 * The control/state layer (MotorController, PhysicalStateEstimate, Planner)
 * and state (RobotConfig, RobotStateContainer) are VALUE MEMBERS owned by Robot.
 *
 * Member declaration order is load-bearing (C++ initialises members in
 * declaration order):
 *   1. hal ref                 — must be first (initialised before interface refs)
 *   2. config, state           — owned values; state needs config
 *   3. motorL, motorR refs     — bound before motorController constructs
 *   4. otos, line, color, gripper, portio refs
 *   5. motorController         — needs motorL, motorR, config refs
 *   6. estimate                — default ctor (PhysicalStateEstimate)
 *   7. portController          — needs portio ref
 *   8. planner depends on motorController, estimate.odometry(), drive, config
 */
struct Robot {
    // ---- HAL reference (must be declared first) ----
    Hardware&           hal;

    // ---- Owned value members (initialized after hal) ----
    RobotConfig         config;   // owned copy; SET commands mutate this
    RobotStateContainer state;    // = defaultInputs(config)

    // ---- Device interface references (bound from hal in constructor) ----
    // Declared before motorController so the refs are live when motorController
    // constructs and binds motorL/motorR.
    IMotor&             motorL;
    IMotor&             motorR;
    IOdometer&          otos;
    ILineSensor&        line;
    IColorSensor&       colorSensor;  // named colorSensor to avoid macro collisions
    IServo&             gripper;
    IPortIO&            portio;

    // ---- Owned control-layer members (depend on refs above) ----
    MotorController     motorController;   // (motorL, motorR, config)
    PhysicalStateEstimate estimate;        // default ctor; wraps Odometry+EKF (041-003)
    // motionController removed in 061-004: absorbed into Planner as direct members.
    PortController      portController;    // (portio)
    ServoController     servoController;   // (gripper)
    // Phase E (043-001) sensor subsystems — own the timed LINE/COLOUR/PORTS reads
    // that were inline blocks in loopTickOnce.  Each binds the device-interface ref
    // (line / colorSensor / portio), state.inputs, and config — all declared above,
    // so they are live when these members construct (C++ inits in declaration order).
    // Types are namespaced under `subsystems` because LineSensor/ColorSensor are
    // also io/real device-driver class names (firmware build collision).
    subsystems::LineSensor  lineSensor;    // (line, state.inputs, config)
    subsystems::ColorSensor colorSensor_;  // (colorSensor, state.inputs, config)
    subsystems::Ports       ports;         // (portio, state.inputs, config)
    // Phase E (043-003) Gripper subsystem — structural seam for the OPTIONAL
    // servo actuator.  Binds the existing `gripper` IServo& (== IPositionMotor&)
    // device ref declared above, so it is live when this member constructs.
    // periodic()/updateInputs() are no-ops and gripper_sub is NOT called from
    // loopTickOnce — pure additive seam, zero behavior change (golden-TLM stays
    // byte-exact).  Named gripper_sub to NOT shadow the `IServo& gripper` device
    // ref above (OQ-4).  The existing ServoController servoController member that
    // dispatches the GRIP command is unchanged and still owns actuation.
    subsystems::Gripper     gripper_sub;   // (gripper)
    HaltController      haltController;    // user-facing named stop-condition registry
    // Superstructure (Seam 3, 042-001) — thin Goal coordinator.  MUST be declared
    // AFTER haltController: it holds a reference to it, and C++ initialises
    // members in declaration order.  Also holds Planner& and const RobotConfig&.
    // Note: planner is declared AFTER superstructure; the Planner& is only stored
    // (not used) during Superstructure construction, so this is safe (061-002).
    Superstructure      superstructure;    // (planner, haltController, config)
    // Phase 3 (059-004): new message-contract subsystems.  configure() is called
    // in the Robot constructor body after all legacy members are fully constructed.
    //
    // Declaration order is load-bearing (must follow all legacy members they ref):
    //   bvc      depends on motorController (declared above)
    //   drive    depends on motorL/motorR, motorController, bvc, estimate
    //   sensors  depends on lineSensor, colorSensor_ (declared above)
    //   planner  depends on motorController, estimate.odometry(), drive, config (061-004)
    //
    // bvc: Drive's own BodyVelocityController.  Separate from the _bvc inside
    // Planner to avoid sharing state between the Drive and Planner BVC instances.
    BodyVelocityController  bvc;           // Drive's BVC — (motorController, config)
    subsystems::Drive       drive;         // new-arch Drive subsystem (059-004)
    subsystems::Sensors     sensors;       // new-arch Sensors facade (059-004)
    Planner                 planner;       // new-arch Planner wrapper (059-004)

    // OtosCommands — app-layer Commandable for the seven OTOS-tuning verbs
    // (OI/OZ/OR/OV/OL/OA/OP), moved out of Odometry in 041-002.  No construction
    // dependency on other members; wired post-construction via setCtx() in the
    // Robot constructor.  buildCommandTable aggregates its getCommands().
    OtosCommands        _otosCommands;     // default ctor; setCtx()'d in constructor

    // ---- Constructor ----
    explicit Robot(Hardware& hal, const RobotConfig& cfg);

    // ---- Kept orchestration methods ----
    // These methods span multiple subsystems and are kept as Robot members
    // rather than inlined at every call site.

    // controlCollectSplitPhase REMOVED (039-002): the encoder read moved into
    // Hardware::tick(now) → Motor::tick(); the outlier filter + controlTick() +
    // wedge push moved (verbatim) into loopTickOnce()'s CONTROL COLLECT block.
    // The streak/wedge state members below remain — the relocated block uses them.

    // otosCorrect — read OTOS device, write state.inputs.otos*, call estimate.addOtosObservation().
    // Uses otos.readTransformed(pose, heading) — no inlined LSB math.  039-004:
    // RobotConfig is sealed out of the read signature (held as an OtosSensor impl member).
    void otosCorrect(uint32_t now_ms);

    // Sensor read task entry points REMOVED (043-001, Phase E): lineRead/colorRead/
    // portsRead bodies moved verbatim into the LineSensor/ColorSensor/Ports
    // subsystems' updateInputs(now).  loopTickOnce calls their periodic() instead.

    // resetEncoders — atomically resets ALL encoder state so that both the outlier
    // filter baseline and Odometry's previous-encoder snapshot see a fresh zero.
    //
    // Atomically:
    //   1. Calls motorController.resetEncoderAccumulators() — resets hardware
    //      accumulators AND MotorController velocity baselines (_prevEncL/R,
    //      _hasTimestamp*, _prevTimeMsL/R).
    //   2. Zeroes state.inputs.encLMm / encRMm — aligns the outlier filter
    //      baseline with the fresh accumulators.
    //   3. Calls estimate.rebaselinePrev(0, 0) — prevents Odometry::predict()
    //      from computing a large negative delta (dL = 0 - _prevEncL) on the
    //      very next tick, which previously teleported the pose backward by the
    //      prior segment's travel.
    //
    // Does NOT touch pose (x, y, theta) or the EKF state.
    // Called by distanceDrive() (D command) and handleZero() (ZERO enc path).
    // (N1 fix, sprint 030-001.)
    void resetEncoders();

    // distanceDrive — calls motionController.beginDistance + calls resetEncoders()
    // to atomically reset hardware accumulators, velocity baselines, the outlier
    // filter baseline, and Odometry's encoder snapshot.
    void distanceDrive(int32_t l, int32_t r, int32_t targetMm,
                       ReplyFn fn, void* ctx, const char* corr_id = nullptr);

    // buildTlmFrame — assemble unified TLM frame; shared by STREAM and SNAP.
    int  buildTlmFrame(char* buf, int len);

    // telemetryEmit — gate and emit the periodic TLM frame.
    void telemetryEmit(uint32_t now_ms, ReplyFn fn, void* ctx);

    // systemTime — robot system time in ms since boot.
    uint32_t systemTime() const;

    // setMotionQueue — bind the CommandQueue for VW converter push_front.
    // Called by LoopScheduler (or test harness) after the queue is created.
    // Null (default) causes converter handlers to fall back to direct begin*() calls.
    // Replaces motionController.setQueue() which was removed in sprint 026-002.
    void setMotionQueue(CommandQueue* q) { _motionCtx.queue = q; }

    // ---- Command-table building ----
    // Aggregate all command descriptors into a vector:
    //   Commandable members (motionController, _otosCommands, portController,
    //   servoController), optional DebugCommands, optional SimCommands, then
    //   system commands (HELLO, PING, ECHO, ID, VER, HELP, SNAP, ZERO, STREAM,
    //   RF, GET VEL, GET, SET).
    // sched may be nullptr; RF will reply ERR noradio if it is.
    // sim (069-003) is sim-build-only: the ARM target's call site never
    // passes it (defaults to nullptr), and SIMSET/SIMGET are absent from the
    // ARM command table entirely -- see SystemCommands.cpp.
    std::vector<CommandDescriptor> buildCommandTable(
        DebugCommands* dbg   = nullptr,
        LoopScheduler*    sched = nullptr,
        SimCommands*      sim   = nullptr) const;

    // ---- Gating state that pairs with the kept methods ----
    uint32_t _lastTlmMs     = 0;
    uint32_t _lastActiveMs  = 0;

    // ---- D10 telemetry: sequence counter + channel binding (028-005) ----
    // _tlmSeq: monotonically incrementing uint16 emitted as seq=<n> in every
    //   TLM frame (both STREAM and SNAP share the same counter).  Wraps at 65535.
    // _tlmBoundFn / _tlmBoundCtx: the reply channel bound by the last STREAM
    //   command.  Set in handleStream; nullptr means no STREAM has been issued
    //   (TLM is suppressed, same behaviour as tlmPeriod=0 on init).
    uint16_t _tlmSeq        = 0;
    ReplyFn  _tlmBoundFn    = nullptr;
    void*    _tlmBoundCtx   = nullptr;
    // _tlmBoundIsRadio: true when the bound TLM channel is the radio (relay).
    //   Set in runCommsIn alongside _tlmBoundFn (the only site that resolves the
    //   channel type from _tlmBoundCtx).  telemetryEmit uses it to cap the TLM
    //   rate on the radio: the link sustains only ~5 Hz, so emitting at the full
    //   serial rate drops ~85-100% of frames during motion (bench-measured).
    bool     _tlmBoundIsRadio = false;

    // ---- OTOS validity tracking (D9 — 027-005) ----
    // _otosInvalidStartMs: system time when OTOS first became invalid in the
    //   current invalidity window (0 = OTOS is currently valid / no window open).
    // _otosLostEmitted: true once "EVT otos lost" has been emitted for the
    //   current invalidity window; reset to false when OTOS becomes valid again.
    uint32_t _otosInvalidStartMs = 0;
    bool     _otosLostEmitted    = false;

    // ---- OTOS WARNING-bit persistence gate (CR-06 — 065-006) ----
    // Restores the transient-vs-persistent distinction the 2026-06-17
    // `healthy = poseOk` change lost: a READABLE-but-WARNING reading (bench,
    // lifted, freshly-placed robot) is fused through <= kOtosWarnPersistK
    // consecutive warn ticks (transient), then blocked until
    // kOtosCleanReadmitN consecutive clean ticks re-admit it.  See
    // architecture-update.md Step 4-5 item 6 / Design Rationale Decision 5.
    uint8_t _otosWarnStreak    = 0;
    uint8_t _otosCleanStreak   = 0;
    bool    _otosFusionBlocked = false;
    static constexpr uint8_t kOtosWarnPersistK  = 3;
    static constexpr uint8_t kOtosCleanReadmitN = 5;

private:
    // Stable storage for command contexts; pointers into these are placed in
    // CommandDescriptors, which must outlive the CommandProcessor.
    mutable CfgCtx      _cfgCtx    = {};  // GET / SET
    mutable RobotSysCtx _sysCtx    = {};  // HELLO, PING, ECHO, ID, VER, …, RF
    mutable MotionCtx   _motionCtx = {};  // S/T/D/G/R/TURN/RT/VW/X/STOP handlers (sprint 026-002)

    // CommandQueue forward declaration for setMotionQueue.
    // (Included via MotionCommands.h → CommandQueue.h.)
};
