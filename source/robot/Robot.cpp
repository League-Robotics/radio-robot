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
            hal.otos(), config),
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
    // Odometry into the app-layer OtosCommands.  Bind the same IOdometer device
    // and cached HardwareState pointers the handlers previously reached through
    // Odometry::setCtx, so the verbs dispatch and behave identically.
    _otosCommands.setCtx(&otos, &state.actual);
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
// otosCorrect — EKF Kalman update from OTOS position and velocity (sprint 023).
//
// Reads OTOS position, velocity, and acceleration.  Passes position + velocity
// to correctEKF() for EKF fusion.  Stores acceleration in HardwareState for
// host telemetry via the RobotStateContainer.
//
// Encoder-derived velocity is NOT fused here: as of 033-003 it is fused
// unconditionally in Odometry::predict() every tick, so fusedV/fusedOmega stay
// live even when this OTOS-gated path is skipped (lifted stand, dropout).
// ---------------------------------------------------------------------------

void Robot::otosCorrect(uint32_t now_ms)
{
    // Indirection through hal.otos() — reads the LIVE active pointer, not the
    // cached `otos` ref (which was bound at construction to the real OtosSensor
    // and cannot be re-seated).  When NezhaHAL::setOtosBench(true) is called,
    // hal.otos() returns the BenchOtosSensor; the cached `otos` ref keeps
    // pointing to the real chip.  This is the ONLY place otosCorrect() diverges
    // from the `otos` ref; all other Robot read sites keep the cached ref.
    // (sprint 031-002 reference-reseating fix)
    IOdometer& activeOtos = hal.otos();

    if (!activeOtos.is_initialized()) return;

    // -----------------------------------------------------------------------
    // D9 (027-005): OTOS STATUS register validity gate.
    //
    // Read REG_STATUS (0x1F) and check the most-recent I2C read flag before
    // fusing into the EKF.  A lifted or just-placed robot reports a non-zero
    // STATUS byte (tracking invalid).  Passing zero velocity to the EKF while
    // the sensor is invalid drags fused velocity to zero and fights the
    // controller — this was the root cause of the "spin on placement" symptom.
    //
    // EVT path (Open Question 3 resolution): we call
    //   planner.emitToActiveChannel("EVT otos lost", state.desired)
    // which wraps the existing static emitEvt(base, TargetState&) helper.
    // Robot owns state.target and can pass it directly; no new reply-sink
    // plumbing is required.  emitEvt routes via target.sink.emitFn — the
    // reply channel captured when the active command (G/T/D/TURN) started.
    // -----------------------------------------------------------------------
    uint8_t otosStatus = 0;
    bool statusOk = activeOtos.readStatus(otosStatus);

    // Two-tier gate (D9 + telemetry decoupling):
    //
    // 1. READABLE — is there a usable reading at all?  Only an I2C failure or a
    //    HARD error (errPaa bit6 / errLsm bit7) means "no reading".  WARNING bits
    //    (warnTiltAngle bit0 / warnOpticalTracking bit1) do NOT block the read:
    //    the OTOS still returns a pose + IMU heading, just degraded.  On a
    //    bench/stand warnOpticalTracking is ALWAYS set (no surface in range) —
    //    we still want the raw reading visible in telemetry (otos=) and the IMU
    //    heading usable.
    //
    // 2. HEALTHY — is it good enough to FUSE into the EKF?  Only when fully clean
    //    (otosStatus == 0).  warnOpticalTracking ⇒ the optical position is
    //    unreliable; fusing it drags fused velocity/pose (the D9 "spin on
    //    placement" symptom).  Degraded readings are shown but not fused; pose
    //    tracking falls back to encoder odometry.
    //
    // NOTE: do NOT gate on lastReadOk() before the read — it reflects the PREVIOUS
    // tick's readXYH and starts false, which deadlocks the real sensor forever
    // (valid never set → readTransformed never runs → _lastReadOk never set).
    // The read is validated by readTransformed's own return value (poseOk) below.
    static constexpr uint8_t kOtosHardErr = 0xC0;   // errLsm(7) | errPaa(6)
    bool readable = statusOk && ((otosStatus & kOtosHardErr) == 0);

    // Pass current fused heading for the lever-arm offset rotation.
    float headingRad = state.actual.fused.pose.h;

    OtosPose p{0.0f, 0.0f, 0.0f};
    bool poseOk = readable && activeOtos.readTransformed(p, headingRad);

    // Telemetry: expose the raw OTOS pose whenever a fresh reading exists (even
    // degraded).  otos.valid drives the TLM otos= freshness gate; it means "a
    // recent raw reading exists", NOT "was fused".  On a same-tick read failure
    // do not write otosX/Y/H with garbage zeros.
    if (poseOk) {
        // Write canonical optical.pose (047-002).
        state.actual.optical.pose.x = p.x;
        state.actual.optical.pose.y = p.y;
        state.actual.optical.pose.h = p.h;
        state.actual.otos.lastUpdMs = now_ms;
        state.actual.otos.valid = true;
    } else {
        state.actual.otos.valid = false;
    }

    // Fusion / "OTOS lost" health: a successful read with no HARD errors
    // (readable/poseOk already exclude kOtosHardErr above).  Do NOT additionally
    // gate on otosStatus==0 — benign WARNING bits (warnTiltAngle from the IMU,
    // transient warnOpticalTracking) would otherwise drop the OTOS from fusion
    // ENTIRELY, leaving the fused pose to ride the encoder.  The OTOS tracks the
    // camera well even with a warn bit set; fuse it (2026-06-17).
    bool healthy = poseOk;
    if (!healthy) {
        // Emit "EVT otos lost" once per unhealthy window, only during an active
        // motion command (no point signalling on a parked robot).  Trigger is
        // unchanged from D9; the raw telemetry above is independent of this.
        if (planner.hasActiveCommand()) {
            if (_otosInvalidStartMs == 0) {
                _otosInvalidStartMs = now_ms;
            }
            if (!_otosLostEmitted &&
                ((now_ms - _otosInvalidStartMs) >= 500u)) {
                planner.emitToActiveChannel("EVT otos lost",
                                            state.desired);
                _otosLostEmitted = true;
            }
        }
        return;  // shown in telemetry (if poseOk), but not fused
    }

    // Healthy: reset the invalidity tracking window and fuse.
    _otosInvalidStartMs = 0;
    _otosLostEmitted    = false;

    // -----------------------------------------------------------------------
    // CR-06 (065-006): WARNING-bit persistence gate.
    //
    // `readable` above already excludes HARD errors (kOtosHardErr); a
    // WARNING bit (warnTiltAngle bit0 / warnOpticalTracking bit1) leaves the
    // reading READABLE but degraded.  A short warn streak (<= K ticks) is
    // transient (I2C/IMU noise) and safe to fuse through; a PERSISTENT warn
    // streak (lifted robot, robot on the stand, freshly placed robot) must
    // NOT be fused — otherwise EKFTiny's own gate-recovery force-snaps the
    // fused pose to the frozen observation after 10 consecutive rejections,
    // reopening the "spin on placement" failure (D9 / 027-005) the original
    // two-tier gate existed to prevent.
    // -----------------------------------------------------------------------
    if (otosStatus != 0) {
        // Warning tick: accumulate the persistent-warn streak and drop any
        // partial progress toward re-admission (the clean streak must be
        // consecutive).
        if (_otosWarnStreak < 0xFF) ++_otosWarnStreak;
        _otosCleanStreak = 0;
        if (_otosWarnStreak > kOtosWarnPersistK) {
            _otosFusionBlocked = true;
        }
    } else {
        // Clean tick: reset the warn streak; count toward re-admission if
        // currently blocked.
        _otosWarnStreak = 0;
        if (_otosFusionBlocked) {
            if (++_otosCleanStreak >= kOtosCleanReadmitN) {
                _otosFusionBlocked = false;
                _otosCleanStreak   = 0;
            }
        }
    }

    if (_otosFusionBlocked) {
        // Raw telemetry (state.actual.optical.pose / otos.valid) was already
        // written above — only the EKF fusion call below is skipped.
        return;
    }

    // Read OTOS velocity and acceleration; store acceleration for telemetry.
    OtosVelocity vel;
    bool velOk = activeOtos.readVelocityTransformed(vel, headingRad);
    OtosAccel    acc = activeOtos.readAccelTransformed();
    state.actual.otosAccelX = acc.ax_mmps2;
    state.actual.otosAccelY = acc.ay_mmps2;

    // If the velocity read also failed this tick, use zero velocity rather
    // than fusing garbage — the EKF's encoder-based velocity estimate is a
    // better fallback.  We still fuse pose (poseOk was true).
    if (!velOk) {
        vel.v_mmps     = 0.0f;
        vel.omega_rads = 0.0f;
    }

    // Encoder-derived velocity is fused unconditionally in Odometry::predict()
    // every tick (033-003), so correctEKF() fuses only the OTOS observations.
    // 047-002: now_ms added so correctEKF can stamp actual.optical.stamp.lastUpdMs.
    // Differential build: vy is always 0.0f (no lateral encoder/OTOS observation).
    estimate.addOtosObservation(p.x, p.y, p.h,
                        vel.v_mmps, vel.omega_rads, 0.0f, now_ms,
                        state.actual.optical, state.actual.fused);
}

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
