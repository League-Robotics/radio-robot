#include "Robot.h"
#include "MotionController.h"
#ifndef HOST_BUILD
#include "MicroBit.h"
#include "MicroBitDevice.h"
#endif
#include "Odometry.h"
#include "DebugCommandable.h"
#include "CommandProcessor.h"
#include "ConfigRegistry.h"
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>
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
extern uint32_t g_sim_now_ms;

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
//   motorController, estimate, motionController, portController, servoController
//
// hal must be declared (and therefore initialized) before the interface refs so
// that hal.motorL() etc. are valid when the refs are bound.
//
// Two post-construction binds:
//   motionController.setHardwareState(&state.inputs)  — MotionController reads pose
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
      motionController(motorController, estimate.odometry(), config),
      portController(portio),
      servoController(gripper),
      // Phase E (043-002) Drive subsystem — wired with the IMotor& device refs
      // (motorL, motorR), motorController, estimate, state.inputs, state.commands,
      // and config.  Declaration order in Robot.h puts `drive` after all of these,
      // so the refs are live here.  The five filter-streak members it owns are
      // value-initialised inside Drive (same initial values as the former Robot
      // fields).  See architecture-update.md OQ-1/OQ-2.
      drive(motorL, motorR, motorController, estimate,
            state.inputs, state.commands, config),
      // Phase E (043-001) sensor subsystems — wired with their device ref,
      // state.inputs (HardwareState), and config.  Declaration order in Robot.h
      // puts these after the refs they bind, so the refs are live here.
      // NOTE: the ColorSensor subsystem member is named colorSensor_ (trailing
      // underscore) because the existing IColorSensor& device ref is already
      // named colorSensor (kept to avoid macro collisions; used by
      // SystemCommands::caps).  Architecture-update.md names it colorSensor; the
      // device-ref collision forces the underscore.  Internal naming only — no
      // behavior/TLM change.  See report annotation.
      lineSensor(line, state.inputs, config),
      colorSensor_(colorSensor, state.inputs, config),
      ports(portio, state.inputs, config),
      // Phase E (043-003) Gripper subsystem — binds the existing `gripper` IServo&
      // (== IPositionMotor&) device ref bound above.  Declaration order in Robot.h
      // puts gripper_sub after `gripper`, so the ref is live here.  No-op subsystem
      // (periodic/updateInputs are no-ops); not wired into loopTickOnce.  Actuation
      // still flows through servoController (unchanged) — zero behavior change.
      gripper_sub(gripper),
      // Superstructure (042-001) — wired with references to motionController and
      // haltController (both declared before it) plus config.  Declaration order
      // in Robot.h guarantees those are constructed first.
      superstructure(motionController, haltController, config)
{
    motionController.setHardwareState(&state.inputs);
    motorController.setCommandsRef(&state.commands);
    // setRobotCtx replaces setCtx (sprint 026-002): MotionCtx now lives in Robot.
    motionController.setRobotCtx(this);
    // Initialise _motionCtx (sprint 026-002): mc and robot pointers; queue wired
    // later by setMotionQueue() from LoopScheduler or test harness.
    // 042-001: superstructure pointer wired so handleVW queue-path branches route
    // begin* through requestGoal (Seam 3).
    _motionCtx.mc             = &motionController;
    _motionCtx.superstructure = &superstructure;
    _motionCtx.robot          = this;
    _motionCtx.queue          = nullptr;
    estimate.setCtx(&otos, &state.inputs);
    // 041-002: the OTOS command handlers (OI/OZ/OR/OV/OL/OA/OP) moved out of
    // Odometry into the app-layer OtosCommands.  Bind the same IOdometer device
    // and cached HardwareState pointers the handlers previously reached through
    // Odometry::setCtx, so the verbs dispatch and behave identically.
    _otosCommands.setCtx(&otos, &state.inputs);
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
// subsystems::Drive::periodic(now, fn, ctx), and the per-wheel streak/wedge state
// members it used (_filterRejectStreakL/R, _prevDriving, _lastControlMs,
// _prevAnyWedged, kFilterRejectStreakThreshold) moved off Robot onto Drive as
// value members.  loopTickOnce now calls robot.drive.periodic(...) in the same
// position the inline block ran (before cmd.dequeueOne).
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
    //   motionController.emitToActiveChannel("EVT otos lost", state.target)
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

    // Pass poseHrad for the lever-arm offset rotation (no-op when offsets are zero).
    float headingRad = state.inputs.poseHrad;

    OtosPose p{0.0f, 0.0f, 0.0f};
    bool poseOk = readable && activeOtos.readTransformed(p, headingRad);

    // Telemetry: expose the raw OTOS pose whenever a fresh reading exists (even
    // degraded).  otos.valid drives the TLM otos= freshness gate; it means "a
    // recent raw reading exists", NOT "was fused".  On a same-tick read failure
    // do not write otosX/Y/H with garbage zeros.
    if (poseOk) {
        state.inputs.otosX = p.x;
        state.inputs.otosY = p.y;
        state.inputs.otosH = p.h;
        state.inputs.otos.lastUpdMs = now_ms;
        state.inputs.otos.valid = true;
    } else {
        state.inputs.otos.valid = false;
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
        if (motionController.hasActiveCommand()) {
            if (_otosInvalidStartMs == 0) {
                _otosInvalidStartMs = now_ms;
            }
            if (!_otosLostEmitted &&
                ((now_ms - _otosInvalidStartMs) >= 500u)) {
                motionController.emitToActiveChannel("EVT otos lost",
                                                     state.target);
                _otosLostEmitted = true;
            }
        }
        return;  // shown in telemetry (if poseOk), but not fused
    }

    // Healthy: reset the invalidity tracking window and fuse.
    _otosInvalidStartMs = 0;
    _otosLostEmitted    = false;

    // Read OTOS velocity and acceleration; store acceleration for telemetry.
    OtosVelocity vel;
    bool velOk = activeOtos.readVelocityTransformed(vel, headingRad);
    OtosAccel    acc = activeOtos.readAccelTransformed();
    state.inputs.otosAccelX = acc.ax_mmps2;
    state.inputs.otosAccelY = acc.ay_mmps2;

    // If the velocity read also failed this tick, use zero velocity rather
    // than fusing garbage — the EKF's encoder-based velocity estimate is a
    // better fallback.  We still fuse pose (poseOk was true).
    if (!velOk) {
        vel.v_mmps     = 0.0f;
        vel.omega_rads = 0.0f;
    }

    // Encoder-derived velocity is fused unconditionally in Odometry::predict()
    // every tick (033-003), so correctEKF() fuses only the OTOS observations.
    estimate.addOtosObservation(state.inputs, p.x, p.y,
                        p.h,
                        vel.v_mmps, vel.omega_rads);
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
// the outlier-filter baseline (state.inputs.encLMm/R), and Odometry's internal
// encoder snapshot — without touching pose.
//
// Previously distanceDrive() reset hardware+MC but left Odometry::_prevEncL/R
// stale, so the very next predict() computed dL = 0 - _prevEncL (large negative)
// and teleported the pose backward by the prior segment's travel.  ZERO enc
// was worse: hardware+MC reset but state.inputs.encLMm/R stayed stale, causing
// the outlier filter to freeze encoder reads until the fresh accumulator climbed
// back, then a pose jump.
// ---------------------------------------------------------------------------

void Robot::resetEncoders()
{
    // 1. Reset hardware accumulators AND MotorController velocity baselines
    //    (_prevEncL/R, _hasTimestamp*, _prevTimeMsL/R).
    motorController.resetEncoderAccumulators();

    // 2. Align the outlier-filter baseline with the now-zeroed accumulators.
    state.inputs.encLMm = 0.0f;
    state.inputs.encRMm = 0.0f;

    // 3. Re-baseline Odometry's encoder snapshot so predict() sees delta=0
    //    on the very next tick rather than (0 - _prevEncL) = large negative.
    estimate.rebaselinePrev(0.0f, 0.0f);
}

// ---------------------------------------------------------------------------
// distanceDrive — begin a distance drive and atomically reset encoder state.
// ---------------------------------------------------------------------------

void Robot::distanceDrive(int32_t l, int32_t r, int32_t targetMm,
                                ReplyFn fn, void* ctx, const char* corr_id)
{
    motionController.beginDistance((float)l, (float)r, targetMm,
                                   systemTime(), state.target, fn, ctx, corr_id);
    // Atomic encoder reset: aligns hardware accumulators, MC velocity baselines,
    // outlier-filter baseline, and Odometry encoder snapshot in one call.
    // (Replaces the split reset that was here + inside beginDistance().)
    resetEncoders();
}

// buildTlmFrame, telemetryEmit → moved to RobotTelemetry.cpp (split 035 A3)
// buildCommandTable + all system command handlers → moved to SystemCommands.cpp (split 035 A3)
