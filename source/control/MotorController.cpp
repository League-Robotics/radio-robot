#include "MotorController.h"
#include <math.h>
#include <cstdio>

// N13 (030-010): PID_BYPASS macro removed. It was a sprint-014 debug flag
// (open-loop feedforward bypass of the velocity PID) that was always disabled
// (set to 0) and no longer needed now that the encoder-wedge root cause is fixed.

MotorController::MotorController(IMotor& left, IMotor& right, const RobotConfig& cal)
    : _motorL(left), _motorR(right), _cal(cal),
      _vcL(cal.velKff, cal.velKp, cal.velKi, cal.velIMax, cal.minWheelSpeed, cal.velKaw),
      _vcR(cal.velKff, cal.velKp, cal.velKi, cal.velIMax, cal.minWheelSpeed, cal.velKaw),
      _cmdEncStartL(0.0f), _cmdEncStartR(0.0f),
      _cmdRatio(1.0f), _fasterIsRight(false),
      _cmds(nullptr),
      _prevEncL(0.0f), _prevEncR(0.0f),
      _prevTimeL(0), _prevTimeR(0),
      _hasTimestampL(false), _hasTimestampR(false),
      _lastPid(0), _hasPidTick(false),
      _lastVelL(0.0f), _lastVelR(0.0f),
      _wedgePrevEncL(0.0f), _wedgePrevEncR(0.0f),
      _wedgePrevValidL(false), _wedgePrevValidR(false),
      _stuckCountL(0), _stuckCountR(0),
      _wedgeEmittedL(false), _wedgeEmittedR(false),
      _busDiag(nullptr),
      _evtFn(nullptr), _evtCtx(nullptr)
{
    gains.kFF     = 0.15f;
    gains.kP      = 0.05f;
    gains.kI      = 0.20f;
    gains.iClamp  = 60.0f;
    gains.kRatio  = 0.01f;
}

void MotorController::resetStuckCounters()
{
    _stuckCountL     = 0;
    _stuckCountR     = 0;
    _wedgeEmittedL   = false;
    _wedgeEmittedR   = false;
    _wedgePrevValidL = false;
    _wedgePrevValidR = false;
}

void MotorController::setTarget(float left, float right)
{
    if (_cmds) {
        // Write canonical tgtSpeed[] arrays ([0]=FR=R, [1]=FL=L).
        _cmds->tgtSpeed[1] = left;    // FL = index 1
        _cmds->tgtSpeed[0] = right;   // FR = index 0
    }
}

void MotorController::startDriveClean(float left, float right)
{
    if (_cmds) {
        // Write canonical tgtSpeed[] arrays ([0]=FR=R, [1]=FL=L).
        _cmds->tgtSpeed[1] = left;    // FL = index 1
        _cmds->tgtSpeed[0] = right;   // FR = index 0
    }
    _fasterIsRight = (fabsf(right) >= fabsf(left));
    float fasterAbs = _fasterIsRight ? fabsf(right) : fabsf(left);
    float slowerAbs = _fasterIsRight ? fabsf(left)  : fabsf(right);
    _cmdRatio = (slowerAbs > 0.0f) ? (fasterAbs / slowerAbs) : 1.0f;
    // Use the control loop's cached encoder values (not a fresh atomic read,
    // which wedges the Nezha encoder — see encoder-wedge note).
    _cmdEncStartL = _prevEncL;
    _cmdEncStartR = _prevEncR;
    _vcL.reset();
    _vcR.reset();
}

void MotorController::startDrive(float left, float right)
{
    if (_cmds) {
        // Write canonical tgtSpeed[] arrays ([0]=FR=R, [1]=FL=L) for all builds.
        _cmds->tgtSpeed[1] = left;    // FL = index 1
        _cmds->tgtSpeed[0] = right;   // FR = index 0
    }

    bool newFasterIsRight = (fabsf(right) >= fabsf(left));
    float newFasterAbs = newFasterIsRight ? fabsf(right) : fabsf(left);
    float newSlowerAbs = newFasterIsRight ? fabsf(left)  : fabsf(right);
    float newRatio = (newSlowerAbs > 0.0f) ? (newFasterAbs / newSlowerAbs) : 1.0f;

    // Use the control loop's cached encoder values (updated every tick) — NOT a
    // fresh atomic read. Firing an atomic 0x46 read from this comms-path call,
    // butted against the control task's own reads / the 0x5F stop, wedges the
    // Nezha encoder. See docs/knowledge encoder-wedge note.
    float curL = _prevEncL;
    float curR = _prevEncR;
    float curFaster   = newFasterIsRight ? curR : curL;
    float curSlower   = newFasterIsRight ? curL : curR;
    float startFaster = newFasterIsRight ? _cmdEncStartR : _cmdEncStartL;
    float prevDeltaFaster = fabsf(curFaster - startFaster);

    float seedFaster = fmaxf(prevDeltaFaster, newFasterAbs);
    float seedSlower = (newRatio > 0.0f) ? (seedFaster / newRatio) : 0.0f;

    float signFaster = ((newFasterIsRight ? right : left) >= 0.0f) ? 1.0f : -1.0f;
    float signSlower = ((newFasterIsRight ? left  : right) >= 0.0f) ? 1.0f : -1.0f;

    if (newFasterIsRight) {
        _cmdEncStartR = curFaster - signFaster * seedFaster;
        _cmdEncStartL = curSlower - signSlower * seedSlower;
    } else {
        _cmdEncStartL = curFaster - signFaster * seedFaster;
        _cmdEncStartR = curSlower - signSlower * seedSlower;
    }

    _fasterIsRight = newFasterIsRight;
    _cmdRatio = newRatio;
}

void MotorController::stop()
{
    if (_cmds) {
        // Zero canonical tgtSpeed[] arrays.
        for (int i = 0; i < kWheelCount; ++i) _cmds->tgtSpeed[i] = 0.0f;
    }
    _vcL.reset();
    _vcR.reset();
    // Use the control loop's cached encoder values (not a fresh atomic read,
    // which — butted against the 0x5F stop below — wedges the Nezha encoder).
    _cmdEncStartL = _prevEncL;
    _cmdEncStartR = _prevEncR;
    _motorL.setSpeed(0);
    _motorR.setSpeed(0);
}

void MotorController::resetIntegrators()
{
    _vcL.reset();
    _vcR.reset();
}

void MotorController::updateVelGains(const RobotConfig& cal)
{
    // Push the (possibly SET-modified) per-wheel velocity gains into both live
    // VelocityControllers. Without this, SET vel.kP/kI/kFF/iMax/kAw only changed
    // the config struct, not the running controllers (which hold copies made at
    // construction). velFiltAlpha and syncGain are read from _cal each tick so
    // they do not need pushing here.
    _vcL.kFF = cal.velKff;  _vcR.kFF = cal.velKff;
    _vcL.kP  = cal.velKp;   _vcR.kP  = cal.velKp;
    _vcL.kI  = cal.velKi;   _vcR.kI  = cal.velKi;
    _vcL.iMax = cal.velIMax; _vcR.iMax = cal.velIMax;
    _vcL.kAw  = cal.velKaw;  _vcR.kAw  = cal.velKaw;
    _vcL.minWheelSpeed = cal.minWheelSpeed; _vcR.minWheelSpeed = cal.minWheelSpeed;
    // Reconfigure the cmon-pid instances with the updated gains (049-003).
    // This re-applies ParallelPid/Backcalculation coefficients immediately so
    // the new gains take effect on the very next controlTick, not one tick later.
    _vcL.reconfigurePid();
    _vcR.reconfigurePid();
}

void MotorController::controlTick(HardwareState& inputs, MotorCommands& cmds,
                                    uint32_t now, int refreshedWheel)
{
    // Per-wheel zero-order-hold velocity update (014-007 ZOH fix).
    //
    // Only the refreshed wheel's velocity is recomputed this tick, using the
    // true elapsed time since the last collect for that wheel.  The other
    // wheel's velocity is held from the previous tick (ZOH — not zeroed).
    //
    // refreshedWheel: 0 = none (first iteration or sync fallback — skip all
    //                            velocity updates so both wheels start at 0),
    //                 1 = left wheel was just collected,
    //                 2 = right wheel was just collected.

    // Max physically-plausible wheel speed. The robot tops out near ~400 mm/s;
    // an occasional corrupt encoder read produces a huge bogus delta (seen: tens
    // of thousands of mm/s). Reject any sample beyond this bound — keep the prev
    // position/time so the NEXT good read computes a correct delta — then EMA the
    // accepted samples.
    static constexpr float kMaxPlausibleSpeed = 1000.0f;

    // Array convention: [0]=R (FR), [1]=L (FL) — see ActualState.h.
    // Reads use encPos[] canonical arrays; writes use vel[] canonical arrays.
    if (refreshedWheel == 1) {
        // Left wheel was just collected.
        float encL = inputs.encPos[1];   // FL = index 1
        if (!_hasTimestampL) {
            _prevEncL = encL; _prevTimeL = now; _hasTimestampL = true;
        } else {
            float elapsed_s = static_cast<float>(now - _prevTimeL) / 1000.0f;
            if (elapsed_s > 0.0f) {
                float rawV = (encL - _prevEncL) / elapsed_s;
                if (fabsf(rawV) <= kMaxPlausibleSpeed) {        // accept plausible
                    float a = _cal.velFiltAlpha;               // EMA smoothing
                    inputs.vel[1] = a * rawV + (1.0f - a) * inputs.vel[1];
                    _prevEncL    = encL;
                    _prevTimeL = now;
                }
                // else: garbage read — reject, hold velocity and prev refs.
            }
        }
        // Right wheel: ZOH — leave vel[0] unchanged.
    } else if (refreshedWheel == 2) {
        // Right wheel was just collected.
        float encR = inputs.encPos[0];   // FR = index 0
        if (!_hasTimestampR) {
            _prevEncR = encR; _prevTimeR = now; _hasTimestampR = true;
        } else {
            float elapsed_s = static_cast<float>(now - _prevTimeR) / 1000.0f;
            if (elapsed_s > 0.0f) {
                float rawV = (encR - _prevEncR) / elapsed_s;
                if (fabsf(rawV) <= kMaxPlausibleSpeed) {        // accept plausible
                    float a = _cal.velFiltAlpha;               // EMA smoothing
                    inputs.vel[0] = a * rawV + (1.0f - a) * inputs.vel[0];
                    _prevEncR    = encR;
                    _prevTimeR = now;
                }
                // else: garbage read — reject, hold velocity and prev refs.
            }
        }
        // Left wheel: ZOH — leave vel[1] unchanged.
    } else if (refreshedWheel == 3) {
        // Both wheels updated this tick (WedgeTest pattern — sprint 015).
        float encL = inputs.encPos[1];   // FL = index 1
        if (!_hasTimestampL) {
            _prevEncL = encL; _prevTimeL = now; _hasTimestampL = true;
        } else {
            float elapsed_s = static_cast<float>(now - _prevTimeL) / 1000.0f;
            if (elapsed_s > 0.0f) {
                float rawV = (encL - _prevEncL) / elapsed_s;
                if (fabsf(rawV) <= kMaxPlausibleSpeed) {
                    float a = _cal.velFiltAlpha;
                    inputs.vel[1] = a * rawV + (1.0f - a) * inputs.vel[1];
                    _prevEncL    = encL;
                    _prevTimeL = now;
                }
            }
        }
        float encR = inputs.encPos[0];   // FR = index 0
        if (!_hasTimestampR) {
            _prevEncR = encR; _prevTimeR = now; _hasTimestampR = true;
        } else {
            float elapsed_s = static_cast<float>(now - _prevTimeR) / 1000.0f;
            if (elapsed_s > 0.0f) {
                float rawV = (encR - _prevEncR) / elapsed_s;
                if (fabsf(rawV) <= kMaxPlausibleSpeed) {
                    float a = _cal.velFiltAlpha;
                    inputs.vel[0] = a * rawV + (1.0f - a) * inputs.vel[0];
                    _prevEncR    = encR;
                    _prevTimeR = now;
                }
            }
        }
    }
    // refreshedWheel == 0: first iteration or no collect — both velocities held at 0.

    // (064-003) Refresh the measured-velocity snapshot consumed by
    // resetEncoderAccumulators()'s at-rest decision, AFTER the per-wheel ZOH
    // velocity update above (whichever wheel(s) were refreshed this tick;
    // unrefreshed wheels retain their held/ZOH value here too, which is the
    // correct "last known" reading for that wheel).
    _lastVelL = inputs.vel[1];   // FL = index 1
    _lastVelR = inputs.vel[0];   // FR = index 0

    // PID runs for BOTH wheels using the held (ZOH) velocities.

    // -------------------------------------------------------------------------
    // Encoder-wedge detector (015-003; blind spots removed 064-004).
    //
    // Per-wheel: an identical consecutive raw reading increments the stuck
    // counter; a changed reading resets it. Unconditional — NOT gated by
    // commanded target (the old tgtW==0.0f reset wiped any streak
    // accumulated during the tail of a command, exactly where the latch
    // mechanism onsets) and NOT gated by an arming grace (the old
    // "has moved since command start" gate meant a wheel that entered a
    // new command already frozen never armed — Episode A: RT turn frozen
    // for 14 TLM frames, zero EVT). When the counter reaches
    // kWedgeThreshold and the latch is clear, emit EVT enc_wedged once
    // (latched) and set the latch. Re-arm when the encoder moves.
    //
    // Only checked when a wheel's encoder was just refreshed (refreshedWheel
    // matches the wheel index) so we compare two real hardware reads, not
    // stale ZOH-held values.
    //
    // See: .clasi/issues/residual-motor-encoder-wedge-after-stop.md
    // -------------------------------------------------------------------------
    {
        // Left-wheel check — when left (or both) was just collected.
        if (refreshedWheel == 1 || refreshedWheel == 3) {
            float encL = inputs.encPos[1];    // FL = index 1 ([0]=R, [1]=L)
            if (_wedgePrevValidL) {
                if (encL != _wedgePrevEncL) {
                    // Encoder moved (or a genuine encoder-reset event shifted
                    // the baseline): re-arm.
                    _stuckCountL   = 0;
                    _wedgeEmittedL = false;
                } else if (_stuckCountL < 255) {
                    ++_stuckCountL;
                }
            }
            _wedgePrevEncL   = encL;
            _wedgePrevValidL = true;

            if (_stuckCountL >= kWedgeThreshold && !_wedgeEmittedL) {
                _wedgeEmittedL = true;
                if (_evtFn && *_evtFn && _evtCtx && *_evtCtx) {
                    // (039-001) Diagnostics now come through the IBusDiagnostics
                    // capability instead of a raw bus pointer.  Null (host /
                    // unbound) yields zeros — byte-identical to the prior
                    // HOST_BUILD path.
                    uint32_t busErr    = _busDiag ? _busDiag->errorCount() : 0;
                    uint32_t reentryN  = _busDiag ? _busDiag->reentryViolations() : 0;
                    int      lastErrV  = _busDiag ? (int)_busDiag->lastError() : 0;
                    // (033-005c) Include a fresh raw read alongside the filtered
                    // value.  raw frozen + enc frozen → real chip/I2C wedge or
                    // stall; raw moving + enc frozen → outlier-filter hold.
                    // readEncoderSettle is used (not Atomic) to avoid the
                    // extra 4 ms pre-write idle during the EVT emit path.
                    int rawL = (int)_motorL.readEncoderSettle(_cal);
                    char evtBuf[112];
                    snprintf(evtBuf, sizeof(evtBuf),
                             "EVT enc_wedged wheel=L enc=%d raw=%d n=%u err=%lu reentry=%lu lastErr=%d",
                             (int)encL,
                             rawL,
                             (unsigned)_stuckCountL,
                             (unsigned long)busErr,
                             (unsigned long)reentryN,
                             lastErrV);
                    (*_evtFn)(evtBuf, *_evtCtx);
                }
            }
        }

        // Right-wheel check — when right (or both) was just collected.
        if (refreshedWheel == 2 || refreshedWheel == 3) {
            float encR = inputs.encPos[0];    // FR = index 0 ([0]=R, [1]=L)
            if (_wedgePrevValidR) {
                if (encR != _wedgePrevEncR) {
                    // Encoder moved (or a genuine encoder-reset event shifted
                    // the baseline): re-arm.
                    _stuckCountR   = 0;
                    _wedgeEmittedR = false;
                } else if (_stuckCountR < 255) {
                    ++_stuckCountR;
                }
            }
            _wedgePrevEncR   = encR;
            _wedgePrevValidR = true;

            if (_stuckCountR >= kWedgeThreshold && !_wedgeEmittedR) {
                _wedgeEmittedR = true;
                if (_evtFn && *_evtFn && _evtCtx && *_evtCtx) {
                    // (039-001) Diagnostics now come through the IBusDiagnostics
                    // capability instead of a raw bus pointer.  Null (host /
                    // unbound) yields zeros — byte-identical to the prior
                    // HOST_BUILD path.
                    uint32_t busErr    = _busDiag ? _busDiag->errorCount() : 0;
                    uint32_t reentryN  = _busDiag ? _busDiag->reentryViolations() : 0;
                    int      lastErrV  = _busDiag ? (int)_busDiag->lastError() : 0;
                    // (033-005c) Include a fresh raw read alongside the filtered
                    // value.  raw frozen + enc frozen → real chip/I2C wedge or
                    // stall; raw moving + enc frozen → outlier-filter hold.
                    int rawR = (int)_motorR.readEncoderSettle(_cal);
                    char evtBuf[112];
                    snprintf(evtBuf, sizeof(evtBuf),
                             "EVT enc_wedged wheel=R enc=%d raw=%d n=%u err=%lu reentry=%lu lastErr=%d",
                             (int)encR,
                             rawR,
                             (unsigned)_stuckCountR,
                             (unsigned long)busErr,
                             (unsigned long)reentryN,
                             lastErrV);
                    (*_evtFn)(evtBuf, *_evtCtx);
                }
            }
        }
    }

    // If no drive command active, ensure motors are stopped.
    // Array convention: [0]=R (FR), [1]=L (FL) — see ActualState.h / OutputState.h.
    if (cmds.tgtSpeed[1] == 0.0f && cmds.tgtSpeed[0] == 0.0f) {
        cmds.pwm[1] = 0;   // canonical array FL
        cmds.pwm[0] = 0;   // canonical array FR
        _motorL.setSpeed(0);
        _motorR.setSpeed(0);
        // Clear stale EMA velocity: MockMotor stops instantly, so the
        // measurement should reflect 0 immediately rather than freezing
        // at the last filtered value until the next drive command.
        inputs.vel[1] = 0.0f;   // canonical array FL
        inputs.vel[0] = 0.0f;   // canonical array FR
        return;
    }

    // PID integrator dt: the ACTUAL elapsed control-tick time, not the nominal
    // controlPeriod. The real loop runs at ~24 ms (10 ms nominal + 2x4 ms
    // encoder settle + bus time), so using the 10 ms nominal made kI accumulate
    // at ~0.4x strength and never close the steady-state error (wheels held ~190
    // of a 200 mm/s command). Clamp the measured delta to [5, 50] ms so a stalled
    // or first tick can't spike the integrator (preserves windup bounding).
    float dt_s;
    if (_hasPidTick) {
        int32_t dms = static_cast<int32_t>(now - _lastPid);
        if (dms < 5)  dms = 5;
        if (dms > 50) dms = 50;
        dt_s = static_cast<float>(dms) / 1000.0f;
    } else {
        dt_s = static_cast<float>(_cal.controlPeriod) / 1000.0f;
        _hasPidTick = true;
    }
    _lastPid = now;
    if (dt_s <= 0.0f) return;

    // Cross-wheel coupling — "slowest wheel governs" (015). Computed BEFORE the
    // per-wheel PID by adjusting the effective setpoints (not the PWM), so the
    // per-wheel PID does the work and there is no fighting. The wheel that is
    // ACHIEVING more of its target is slaved toward the slower wheel's pace;
    // disturbing one wheel pulls the other onto the ratio line. A deadband lets
    // the per-wheel PID absorb LIGHT touches; only a real, sustained discrepancy
    // couples. SET sync=0 -> independent.
    //
    // Blend fraction = syncGain * (discrepancy - deadband), clamped [0,1].
    // At syncGain=1 the coupled target reaches the fully-matched value only when
    // the discrepancy is 100 % (one wheel fully stopped).  Proportional blending
    // prevents the bang-bang setpoint switch that caused oscillation.
    // Array convention: [0]=R (FR), [1]=L (FL) — see ActualState.h / OutputState.h.
    float effTgtL = cmds.tgtSpeed[1];   // FL target
    float effTgtR = cmds.tgtSpeed[0];   // FR target
    // Only couple when BOTH wheels drive the SAME direction (straight / curve).
    // For a spin-in-place the targets are opposite-sign (tgtL=-X, tgtR=+X); the
    // "slowest-wheel-governs" math (coupled = velOther/ratio, ratio<0) then
    // collapses the faster wheel toward the lagging one and the spin degenerates
    // to a single wheel. Same-sign-only (product > 0) keeps spins independent.
    if (_cal.syncGain > 0.0f && cmds.tgtSpeed[1] * cmds.tgtSpeed[0] > 0.0f) {
        float ratio = cmds.tgtSpeed[0] / cmds.tgtSpeed[1];        // commanded vR/vL
        float achL  = inputs.vel[1] / cmds.tgtSpeed[1];      // fraction-of-target each wheel does
        float achR  = inputs.vel[0] / cmds.tgtSpeed[0];
        const float deadband = 0.08f;
        float disc = achL - achR;                              // positive = left ahead
        if (disc > deadband) {
            float blend = _cal.syncGain * (disc - deadband);
            if (blend > 1.0f) blend = 1.0f;
            float coupled = inputs.vel[0] / ratio;          // fully-matched target
            effTgtL = cmds.tgtSpeed[1] * (1.0f - blend) + coupled * blend;
        } else if (-disc > deadband) {
            float blend = _cal.syncGain * (-disc - deadband);
            if (blend > 1.0f) blend = 1.0f;
            float coupled = inputs.vel[1] * ratio;
            effTgtR = cmds.tgtSpeed[0] * (1.0f - blend) + coupled * blend;
        }
    }

    // Per-wheel velocity PID (Sprint 010 inner loop) on the (possibly coupled) targets.
    float uL = _vcL.update(effTgtL, inputs.vel[1], dt_s);
    float uR = _vcR.update(effTgtR, inputs.vel[0], dt_s);

    // Write canonical pwm[] arrays.
    cmds.pwm[1] = static_cast<int16_t>(roundf(uL));   // FL = index 1
    cmds.pwm[0] = static_cast<int16_t>(roundf(uR));   // FR = index 0
    _motorL.setSpeed(static_cast<int8_t>(roundf(uL)));
    _motorR.setSpeed(static_cast<int8_t>(roundf(uR)));
}

void MotorController::getVelocitySourceFlags(bool& leftChip, bool& rightChip) const
{
    // The chip readSpeed (0x47) path was disabled in sprint 013 (motor throb fix).
    // Encoder-delta is the sole velocity source. Always report false.
    leftChip  = false;
    rightChip = false;
}

void MotorController::getEncoderPositions(int32_t& left, int32_t& right) const
{
    // Use atomic reads (request → 4 ms wait → collect) to ensure valid readings
    // outside the split-phase control tick.
    left  = static_cast<int32_t>(_motorL.readEncoderAtomic(_cal));
    right = static_cast<int32_t>(_motorR.readEncoderAtomic(_cal));
}

bool MotorController::computeAtRest() const
{
    // (064-003) At-rest decision: commanded targets both zero AND measured
    // |velocity| below a small epsilon.
    //
    // Commanded component: read directly off the authoritative MotorCommands
    // (the same _cmds pointer setTarget()/startDrive()/stop() write through).
    // Measured component: _lastVelL/R, refreshed every controlTick() call
    // from the EMA-filtered inputs.vel[] (see controlTick() above).
    bool cmdAtRest = (!_cmds) ||
                     (_cmds->tgtSpeed[0] == 0.0f && _cmds->tgtSpeed[1] == 0.0f);
    bool velAtRest = (fabsf(_lastVelL) < kAtRestVelEpsilon) &&
                     (fabsf(_lastVelR) < kAtRestVelEpsilon);
    return cmdAtRest && velAtRest;
}

bool MotorController::isAtRest() const
{
    return computeAtRest();
}

void MotorController::resetEncoderAccumulators()
{
    // (064-003) At-rest decision: firing the hardware atomic-read burst
    // (Motor::resetEncoder(), 3x 0x46 reads + readback-verify, ~24-32 ms of
    // busy-wait I2C) while the wheels are actually rotating latches the
    // Nezha encoder readback — stand-proven ~1.4 transient latches/cycle,
    // escalating to persistent (see clasi/sprints/064-.../issues/
    // encoder-reset-while-moving-latches-readback.md). It is safe only when
    // the drivetrain is genuinely at rest (computeAtRest()). When not at
    // rest, rebaseline in software only (no I2C transaction) instead —
    // Motor::rebaselineSoft() / SimMotor::rebaselineSoft().
    if (computeAtRest()) {
        // At rest: unchanged hardware atomic re-prime. This is ALSO the
        // transient-wedge self-heal mechanism relied on elsewhere (an
        // at-rest reset, e.g. the next D from idle or ZERO enc, re-primes and
        // heals a transient latch) — must stay reachable exactly as today.
        _motorL.resetEncoder();
        _motorR.resetEncoder();
    } else {
        // Not at rest: software-only rebaseline — no I2C transaction.
        _motorL.rebaselineSoft();
        _motorR.rebaselineSoft();
    }
    _prevEncL = 0.0f;
    _prevEncR = 0.0f;
    _hasTimestampL = false;
    _hasTimestampR = false;
    _prevTimeL   = 0;
    _prevTimeR   = 0;
}

float MotorController::clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
