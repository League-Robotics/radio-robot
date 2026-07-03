#include "Robot.h"
#include "subsystems/sensors/SensorsConfig.h"
#include "superstructure/PlannerConfig.h"
#ifndef HOST_BUILD
#include "MicroBit.h"        // IWYU pragma: keep — firmware build needs system_timer_current_time()
#include "MicroBitDevice.h"  // IWYU pragma: keep — firmware runtime init (clangd false-positive under HOST_BUILD)
#endif
#include "DebugCommands.h"
#include "ConfigRegistry.h"
#include <cassert>

// ---------------------------------------------------------------------------
// HOST_BUILD stubs — replace CODAL runtime calls with safe no-op equivalents.
// These are only compiled when building the shared library for host tests.
// ---------------------------------------------------------------------------
#ifdef HOST_BUILD
#include <cstdint>

// Sim-injected clock — updated by sim_tick() and sim_command() in sim_api.cpp
// so that Robot::systemTime() returns sim time rather than real wall-clock time.
// This ensures time-based stop conditions (T, HALT TIME) use the same epoch as
// driveAdvance(now_ms) and evaluate(now_ms), preventing immediate false-fire.
//
// thread_local (066-002 / CR-13): must match sim_api.cpp's definition exactly
// (a thread_local declaration and a non-thread_local declaration of the same
// name are not link-compatible). HOST_BUILD-only (this whole block is
// #ifdef HOST_BUILD) — the ARM firmware target never sees this declaration.
extern thread_local uint32_t g_sim_now_ms;

static uint32_t system_timer_current_time() { return g_sim_now_ms; }
#endif

// Note: microbit_friendly_name() and microbit_serial_number() stubs
// moved to SystemCommands.cpp (split 035 A3) — only needed by system
// command handlers there.

// ---------------------------------------------------------------------------
// Constructor — initializer list must match member declaration order.
//
// Declaration order (from Robot.h):
//   hal, config, state, motorL, motorR, otos, line, colorSensor, gripper, portio,
//   motorController, estimate, portController, servoController
//   (motionController removed 061-004 — absorbed into Planner)
//
// hal must be declared (and therefore initialized) before the interface refs so
// that hal.motorL() etc. are valid when the refs are bound.
//
// Two post-construction binds:
//   planner.setHardwareState(&state.inputs)  — Planner reads authoritative pose
//   motorController.setCommandsRef(&state.commands)   — MotorController writes tgt*/pwm*
// ---------------------------------------------------------------------------

Robot::Robot(Hardware& h, const RobotConfig& cfg)
    : hal(h),
      config(cfg),
      state(defaultInputs(cfg)),
      motorL(hal.motorL()), motorR(hal.motorR()),
      otos(hal.otos()), line(hal.lineSensor()),
      colorSensor(hal.colorSensor()), gripper(hal.gripper()), portio(hal.portIO()),
      motorController(motorL, motorR, config),
      estimate(),
      portController(portio),
      servoController(gripper),
      // Phase E (043-001) sensor subsystems — wired with their device ref,
      // state.actual (ActualState / HardwareState alias), and config.
      // Declaration order in Robot.h puts these after the refs they bind.
      // NOTE: the ColorSensor subsystem member is named colorSensor_ (trailing
      // underscore) because the existing IColorSensor& device ref is already
      // named colorSensor (kept to avoid macro collisions; used by
      // SystemCommands::caps).
      lineSensor(line, state.actual, config),
      colorSensor_(colorSensor, state.actual, config),
      ports(portio, state.actual, config),
      // Phase E (043-003) Gripper subsystem — binds the existing `gripper` IServo&
      // (== IPositionMotor&) device ref bound above.  Declaration order in Robot.h
      // puts gripper_sub after `gripper`, so the ref is live here.  No-op subsystem
      // (periodic/updateInputs are no-ops); not wired into loopTickOnce.  Actuation
      // still flows through servoController (unchanged) — zero behavior change.
      gripper_sub(gripper),
      // Superstructure (042-001) — wired with references to planner and
      // haltController plus config.  Declaration order in Robot.h guarantees
      // haltController is constructed first; planner is constructed after
      // superstructure (safe: Superstructure only stores the reference, never
      // uses it during construction).
      superstructure(planner, haltController, config),
      // Phase 3 (059-004): new message-contract subsystems.  ADDITIVE — NOT yet
      // wired into loopTickOnce; configure() called in the constructor body below.
      //
      // bvc: Drive's own BodyVelocityController.  Separate from
      // Planner's internal _bvc so the two paths don't share PID state.
      bvc(motorController, config),
      // drive: new-arch Drive, built with the same device refs as the legacy
      // drive subsystem.  Own BVC (bvc), own EKF state (via est + odo).
      drive(motorL, motorR, motorController, bvc, estimate, estimate.odometry(),
            hal, config),
      // sensors: facade over the existing lineSensor / colorSensor_ subsystems;
      // shares the same HardwareState they write into.
      sensors(lineSensor, colorSensor_, state.actual),
      // planner: owns motion control directly (061-004: motionController removed).
      planner(motorController, estimate.odometry(), drive, config)
{
    // -----------------------------------------------------------------------
    // Phase 3 (059-004): bottom-up configure() calls.
    // Project the initial RobotConfig into each new-arch subsystem so their
    // internal config slices are live-equivalent from construction time.
    // These calls are idempotent and cheap; order matches dependency direction.
    // -----------------------------------------------------------------------
    drive.configure(toDriveConfig(config));
    sensors.configure(subsystems::toLineSensorConfig(config),
                      subsystems::toColorSensorConfig(config));
    planner.configure(toPlannerConfig(config));
    planner.setHardwareState(&state.actual);
    // 060-002: Drive's constructor already called _mc.setCommandsRef(&_outputs),
    // binding MotorController to drive._outputs.  Do NOT override that binding
    // here, or drive.outputs() will be stale.
    //
    // 061-004: Planner's constructor already called _bvc.setStateRef(&_desired)
    // so that planner.tick() reads the BVC body-twist output from planner._desired.
    // Do NOT override that binding here or planner.tick() will read stale zeros.
    // setRobotCtx replaces setCtx (sprint 026-002): MotionCtx now lives in Robot.
    planner.setRobotCtx(this);
    // Initialise _motionCtx (sprint 026-002): mc and robot pointers; queue wired
    // later by setMotionQueue() from LoopScheduler or test harness.
    // 042-001: superstructure pointer wired so handleVW queue-path branches route
    // begin* through requestGoal (Seam 3).
    _motionCtx.mc             = &planner;
    _motionCtx.superstructure = &superstructure;
    _motionCtx.robot          = this;
    _motionCtx.queue          = nullptr;
    // 070-003: estimate.setCtx() deleted — it was already a documented no-op
    // (PhysicalStateEstimate::setCtx forwarded to Odometry::setCtx, which
    // ignored both parameters). No replacement injection point is needed;
    // every PhysicalStateEstimate method is now explicit per-call.
    // 041-002: the OTOS command handlers (OI/OZ/OR/OV/OL/OA/OP) moved out of
    // Odometry into the app-layer OtosCommands.  Bind the Hardware so the
    // handlers resolve the ACTIVE odometer live via hal.otos() on every
    // dispatch — a construction-bound &otos froze them onto the real chip and
    // OZ/OV could never re-anchor the bench sensor (bench-OTOS issue, 2026-07-03).
    _otosCommands.setCtx(&hal, &state.actual);
    // Bind the LIVE config (this Robot-owned copy is what SET mutates) into
    // the HAL so the bench-OTOS kinematics track runtime SET tw=… updates
    // instead of the boot-time copy the HAL was constructed with.
    hal.bindLiveConfig(&config);
    estimate.initEKF(config.ekfQxy, config.ekfQtheta,
                     config.ekfQv, config.ekfQomega,
                     config.ekfROtosXy, config.ekfROtosV, config.ekfREncV,
                     config.ekfROtosTheta);
}

// ---------------------------------------------------------------------------
// systemTime — robot system time in milliseconds since boot.
// ---------------------------------------------------------------------------

uint32_t Robot::systemTime() const
{
    return (uint32_t)system_timer_current_time();
}

// ---------------------------------------------------------------------------
// controlCollectSplitPhase REMOVED (039-002); CONTROL COLLECT relocated (043-002).
//
// Its body moved into loopTickOnce()'s CONTROL COLLECT block (verbatim, 039-002)
// and the per-loop encoder read moved into Hardware::tick(now) → Motor::tick().
// Phase E (043-002): the CONTROL COLLECT block then moved VERBATIM into
// subsystems::Drive::periodic(now, fn, ctx) and is now deleted together with the
// legacy loop branch in 060-005.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// otosCorrect REMOVED (bench-otos cleanup, 2026-07-03).
//
// Dead code since the 060-005 ordered-tick cutover: nothing called it.  The
// live OTOS read/gate/fuse path is Drive::tickUpdate() STEP 5
// (source/subsystems/drive/Drive.cpp) — lag-gated read through the LIVE
// hal.otos() (074-002), two-tier readable/healthy gate with warn persistence
// and re-admission (065-006/074-003), fusion via _est.addOtosObservation().
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// lineRead / colorRead / portsRead REMOVED (043-001, Phase E).
//
// The 4-channel line read, the non-blocking RGBC poll, and the digital/analogue
// GPIO read moved VERBATIM into the new sensor subsystems'
// updateInputs(uint32_t now) methods:
//   source/subsystems/sensors/LineSensor.cpp
//   source/subsystems/sensors/ColorSensor.cpp
//   source/subsystems/sensors/Ports.cpp
// systemTime() became the `now` parameter (same value loopTickOnce threads).
// loopTickOnce now calls robot.lineSensor / robot.colorSensor_ / robot.ports
// .periodic(ts, now) in the SAME order/position the inline blocks ran.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// resetEncoders — single canonical atomic encoder reset (N1, sprint 030-001).
//
// Atomically resets hardware accumulators, MotorController velocity baselines,
// the outlier-filter baseline (state.inputs.encPos[]), and Odometry's internal
// encoder snapshot — without touching pose.
//
// Previously distanceDrive() reset hardware+MC but left Odometry::_prevEncL/R
// stale, so the very next predict() computed dL = 0 - _prevEncL (large negative)
// and teleported the pose backward by the prior segment's travel.  ZERO enc
// was worse: hardware+MC reset but state.inputs.encPos[] stayed stale, causing
// the outlier filter to freeze encoder reads until the fresh accumulator climbed
// back, then a pose jump.
// ---------------------------------------------------------------------------

void Robot::resetEncoders()
{
    // 1. Reset hardware accumulators AND MotorController velocity baselines
    //    (_prevEncL/R, _hasTimestamp*, _prevTimeMsL/R).
    motorController.resetEncoderAccumulators();

    // 2. Align the outlier-filter baseline with the now-zeroed accumulators.
    // Array convention: [0]=FR=R, [1]=FL=L (sized by kWheelCount; #ifdef-free).
    for (int i = 0; i < kWheelCount; ++i) state.actual.encPos[i] = 0.0f;

    // 3. Re-baseline Odometry's encoder snapshot so predict() sees delta=0
    //    on the very next tick rather than (0 - _prevEncL) = large negative.
    estimate.rebaselinePrev(0.0f, 0.0f);

    // 4. 060-004: Drive owns an independent encoder baseline in _hw.encPos[].
    //    Reset it so tickUpdate() sees 0 delta after the hardware reset, and
    //    LoopTickOnce.cpp's sync block copies 0 back into state.actual.encPos[]
    //    (not the stale pre-reset accumulator).
    drive.resetEncoders();
}

// ---------------------------------------------------------------------------
// distanceDrive — begin a distance drive and atomically reset encoder state.
// ---------------------------------------------------------------------------

void Robot::distanceDrive(int32_t l, int32_t r, int32_t targetDistance,
                                ReplyFn fn, void* ctx, const char* corr_id)
{
    planner.beginDistance((float)l, (float)r, targetDistance,
                          systemTime(), state.desired, fn, ctx, corr_id);
    // Atomic encoder reset: aligns hardware accumulators, MC velocity baselines,
    // outlier-filter baseline, and Odometry encoder snapshot in one call.
    // (Replaces the split reset that was here + inside beginDistance().)
    resetEncoders();
}

// buildTlmFrame, telemetryEmit → moved to RobotTelemetry.cpp (split 035 A3)
// buildCommandTable + all system command handlers → moved to SystemCommands.cpp (split 035 A3)
