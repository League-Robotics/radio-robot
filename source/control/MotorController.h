#pragma once
#include "hal/capability/IVelocityMotor.h"
#include "hal/capability/IBusDiagnostics.h"
#include "Config.h"
#include "VelocityController.h"
#include "Inputs.h"
#include "Protocol.h"
#include <stdint.h>

/**
 * MotorController — per-wheel velocity PID wheel speed control.
 *
 * Inner loop is VelocityController (PI+FF) — one instance per wheel.
 *
 * Sprint 010 replaces the cumulative-distance ratio PID inner loop with
 * two independent VelocityController instances (_vcL, _vcR) that track
 * per-wheel mm/s setpoints. See docs/kinematics-model.md §2.1.
 *
 * Thread safety: single-threaded tick loop only.
 */
class MotorController {
public:
    MotorController(IMotor& left, IMotor& right, const RobotConfig& cal);

    // Gains — public so CommandProcessor can update via K-commands.
    // Defaults: kFF=0.15, kP=0.05, kI=0.20, iClamp=60, kRatio=0.01
    struct Gains {
        float kFF;      // feed-forward coefficient
        float kP;       // proportional gain
        float kI;       // integral gain
        float iClamp;   // integral windup clamp (PWM units, ±)
        float kRatio;   // ratio cross-coupling gain (sprint 2 stub, small)
    } gains;

    // Set speed targets in mm/s. Zero both to coast (not brake).
    void setTarget(float left, float right);   // [mm/s]

    /**
     * startDriveClean — used by T, D, and G commands.
     * Full clean start: snapshot encoders, compute ratio, reset PID.
     * Always call this when starting a new bounded command.
     */
    void startDriveClean(float left, float right);   // [mm/s]

    /**
     * startDrive — used by the S (streaming) command only.
     * Re-seeds cmdEncStart to preserve accumulated ratio history across keepalive re-sends.
     * Does NOT reset PID unless the faster/slower assignment changes.
     */
    void startDrive(float left, float right);   // [mm/s]

    // Stop: zero targets, reset PID, and write zero PWM.
    void stop();

    // Reset integrators only (called by CommandProcessor on mode change).
    void resetIntegrators();

    // Push per-wheel velocity gains (vel.kP/kI/kFF/iMax/kAw/minWheel) from config
    // into the live VelocityControllers after a SET. (filt/sync are read per-tick.)
    void updateVelGains(const RobotConfig& cal);

    /**
     * controlTick — cooperative-loop control step (014-003 / 014-007).
     *
     * Per-wheel zero-order-hold velocity (014-007 ZOH fix):
     *   refreshedWheel: 0 = none (first iteration / sync fallback),
     *                   1 = left wheel was just collected,
     *                   2 = right wheel was just collected.
     *
     * For the refreshed wheel only: computes velocity as
     *   (inputs.encWMm - _prevEncW) / elapsed_s
     * using per-wheel timestamps for the correct elapsed time, then
     * writes inputs.velWMms and updates _prevEncW / _prevTimeMsW.
     *
     * For the other wheel: leaves inputs.velWMms unchanged (ZOH).
     *
     * Then runs PID for BOTH wheels using the held inputs.velLMms/velRMms,
     * writes cmds.pwmL/R, and calls Motor::setSpeed().
     *
     * now: current system time in milliseconds (for per-wheel dt).
     */
    void controlTick(HardwareState& inputs, MotorCommands& cmds,
                     uint32_t now,          // [ms]
                     int refreshedWheel);

    /**
     * setCommandsRef — bind the authoritative MotorCommands struct so that
     * setTarget / startDrive / stop can write tgtLMms/R directly (014-007).
     * Must be called before the first setTarget (Robot constructor does this).
     */
    void setCommandsRef(MotorCommands* cmds) { _cmds = cmds; }

    /**
     * setBusDiagnostics — bind the bus-diagnostics capability so controlTick()
     * can read per-bus error/reentry/lastErr stats when emitting EVT enc_wedged
     * (015-003; capability-typed in 039-001).
     * Optional — if unset, EVT stats show zeros.
     */
    void setBusDiagnostics(IBusDiagnostics* diag) { _busDiag = diag; }

    /**
     * setEvtSink — bind the reply sink for EVT enc_wedged emission (015-003).
     * Points at the LoopScheduler's activeFn / activeCtx live fields so the
     * event goes to whichever channel sent the most recent command.
     * Optional — if fn is nullptr, the event is silently dropped.
     */
    void setEvtSink(ReplyFn* fn, void** ctx) { _evtFn = fn; _evtCtx = ctx; }

    /**
     * stuckCountL / stuckCountR — current consecutive-identical-reading
     * counter for left and right wheels respectively (015-003; unconditional
     * since 064-004 — no longer gated on commanded target or arming grace).
     * Read by CommandProcessor::DBG I2C to include in the dump line.
     */
    uint8_t stuckCountL() const { return _stuckCountL; }
    uint8_t stuckCountR() const { return _stuckCountR; }

    /** resetStuckCounters — zero both stuck counters and re-arm both latch flags. */
    void resetStuckCounters();

    /**
     * wheelWedgedL / wheelWedgedR — expose the per-wheel latch state (033-005e).
     *
     * Returns true from the tick the EVT enc_wedged latch fires until the encoder
     * moves again (latch re-arms).  Used by Robot::controlCollectSplitPhase() to
     * push the wedge state into Odometry so predict() can suppress phantom dTheta.
     */
    bool wheelWedgedL() const { return _wedgeEmittedL; }
    bool wheelWedgedR() const { return _wedgeEmittedR; }

    /**
     * getVelocitySourceFlags — always returns false for both wheels.
     *
     * The chip readSpeed (0x47) path was disabled in sprint 013 to fix
     * motor throb. The encoder-delta fallback is the sole velocity source.
     * Method retained for CommandProcessor wire-format compatibility until
     * ticket 007 migrates callers to Robot::state.
     */
    void getVelocitySourceFlags(bool& leftChip, bool& rightChip) const;

    // Read cumulative encoder positions in mm (sum since last resetEncoderAccumulators()).
    void getEncoderPositions(int32_t& left,    // [mm]
                             int32_t& right) const;   // [mm]

    // Zero encoder accumulators — delegates to Motor::resetEncoder() for each wheel.
    void resetEncoderAccumulators();

    /**
     * isAtRest — true when the drivetrain is genuinely at rest (064-003
     * decision: commanded targets both zero AND measured |velocity| below
     * kAtRestVelEpsilon). Exposes the SAME epsilon/decision computation
     * resetEncoderAccumulators() uses internally to choose hardware-atomic
     * vs. software-only rebaseline — reused (not duplicated) by Drive's
     * auto-re-prime gate (064-004) so it only attempts a re-prime when a
     * hardware re-prime is actually the path resetEncoderAccumulators()
     * would take.
     */
    bool isAtRest() const;

private:
    IMotor&            _motorL;
    IMotor&            _motorR;
    const RobotConfig& _cal;

    // Per-wheel velocity controllers (PI + feed-forward). Sprint 010 inner loop.
    VelocityController _vcL;
    VelocityController _vcR;

    // Encoder/ratio bookkeeping for startDrive seeding (used by streaming command).
    float _cmdEncStartL;     // encoder mm snapshot at command start (left)
    float _cmdEncStartR;     // encoder mm snapshot at command start (right)
    float _cmdRatio;         // |fasterSpeed| / |slowerSpeed|, always >= 1.0
    bool  _fasterIsRight;    // true if right wheel is the commanded-faster wheel

    // Pointer to the authoritative MotorCommands (set by Robot via setCommandsRef).
    // setTarget / startDrive / stop write tgtLMms/R here directly (014-007).
    MotorCommands* _cmds;

    // Previous encoder snapshots — intermediate compute state for velocity differentiation.
    // These are NOT robot state (they are algorithm state private to MotorController).
    float    _prevEncL;   // [mm] at start of last controlTick for left wheel
    float    _prevEncR;   // [mm] at start of last controlTick for right wheel

    // Per-wheel timing for ZOH velocity computation (014-007).
    // lastUpdMs is 0 until the first valid reading (first-sample guard).
    uint32_t _prevTimeL;  // [ms] system time of last left-wheel collect
    uint32_t _prevTimeR;  // [ms] system time of last right-wheel collect
    bool     _hasTimestampL;
    bool     _hasTimestampR;

    // PID integrator dt: actual elapsed control-tick time (ms). The loop runs at
    // ~24 ms (10 ms nominal + 2x4 ms encoder settle + bus), NOT the nominal
    // controlPeriod, so using the nominal value made kI act at ~0.4x strength
    // and never close the steady-state error. Use the measured delta, clamped to
    // a sane window so a stalled tick can't spike the integrator.
    uint32_t _lastPid;       // [ms] system time of last controlTick PID update
    bool     _hasPidTick;    // false until the first PID tick

    // Measured per-wheel velocity snapshot (064-003), refreshed each
    // controlTick() call from inputs.velMms[] AFTER that tick's per-wheel ZOH
    // velocity update runs. Read by resetEncoderAccumulators() as the
    // measured component of the at-rest decision (hardware atomic re-prime
    // vs. software-only rebaseline) — see architecture-update.md ticket 3.
    float    _lastVelL;  // [mm/s]
    float    _lastVelR;  // [mm/s]

    // -------------------------------------------------------------------------
    // Encoder-wedge detector (015-003; blind spots removed 064-004)
    //
    // Per-wheel consecutive-identical-reading counter, unconditional: any
    // identical consecutive raw reading increments it, any changed reading
    // resets it to 0 — regardless of commanded target (the old tgtW==0.0f
    // reset is gone) and regardless of whether the wheel has ever moved this
    // episode (the old 033-005d arming grace is gone). A genuinely idle,
    // uncommanded wheel is therefore "stuck" too — correctly so; TLM's mode=
    // field lets a host distinguish idle quiescence from an in-motion fault.
    // When either counter reaches kWedgeThreshold and the latch flag is clear,
    // an EVT enc_wedged line is emitted via _evtFn/_evtCtx and the latch is set.
    // The latch re-arms (clears) when the encoder value changes.
    // -------------------------------------------------------------------------
    static constexpr uint8_t kWedgeThreshold = 10;  // consecutive identical reads

    // Previous encoder snapshots used for identity comparison (distinct from the
    // velocity-compute snapshots _prevEncL/R above, which are updated only on
    // the refreshed wheel; these are updated every tick regardless of wheel).
    float    _wedgePrevEncL;
    float    _wedgePrevEncR;
    bool     _wedgePrevValidL;    // false until first reading
    bool     _wedgePrevValidR;

    uint8_t  _stuckCountL;        // consecutive identical readings (L)
    uint8_t  _stuckCountR;        // consecutive identical readings (R)
    bool     _wedgeEmittedL;      // latch: EVT already sent for this episode (L); exposed via wheelWedgedL() (033-005e)
    bool     _wedgeEmittedR;      // latch: EVT already sent for this episode (R); exposed via wheelWedgedR() (033-005e)

    // (064-004) The 033-005d arming grace (_hasMovedL/R — wedge detector did
    // not arm until the wheel had moved at least once since the command
    // started) is REMOVED. It was a structural blind spot: a wheel that
    // enters a new command already frozen never "moves," so counting never
    // started (Episode A: RT turn frozen for 14 TLM frames, zero EVT). The
    // per-wheel comparison is now unconditional — see controlTick().

    // Optional bus-diagnostics capability for stats in EVT body (039-001).
    IBusDiagnostics* _busDiag;

    // Live pointers into LoopScheduler::activeFn / activeCtx so the EVT goes
    // to the channel that most recently sent a command.
    ReplyFn* _evtFn;
    void**   _evtCtx;

    // (064-003/064-004) At-rest decision shared by resetEncoderAccumulators()
    // (chooses hardware-atomic vs. software-only rebaseline) and isAtRest()
    // (Drive's auto-re-prime gate) — one computation, two callers.
    // kAtRestVelEpsilon is a design-time estimate (BENCH-CONFIRM — see
    // architecture-update.md "Open Questions"), not yet HITL-validated —
    // same convention as Motor::setSpeed()'s kMaxDeltaPwmPerWrite.
    static constexpr float kAtRestVelEpsilon = 5.0f;  // [mm/s]
    bool computeAtRest() const;

    // clamp helper
    static float clamp(float v, float lo, float hi);
};
