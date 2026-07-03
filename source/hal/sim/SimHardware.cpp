#include "SimHardware.h"
#include "types/Config.h"             // RobotConfig, defaultRobotConfig()
#include "types/Inputs.h"       // MotorCommands
#include <cmath>

// ---------------------------------------------------------------------------
// Constructor — own the plant, construct each observation model against it, and
// wire robot geometry so the OTOS sim model integrates correctly.
// ---------------------------------------------------------------------------
SimHardware::SimHardware(const RobotConfig& cfg)
    : _plant()
    , _motorL(_plant, _plant, SimMotor::Side::LEFT)
    , _motorR(_plant, _plant, SimMotor::Side::RIGHT)
    , _odom(_plant, cfg)
    , _line(_plant)
    , _color(_plant)
    , _portIO(_plant)
    , _servo()
    , _benchOtos()
    , _otosActive(&_odom)   // default: real (ground-truth) odometer (074-001)
{
    _trackwidth = cfg.trackwidth;
    _plant.setTrackwidth(cfg.trackwidth);
    _benchOtos.begin();
}

// tick(now) — sensor tick.  Promotes each sim motor's plant reported-encoder
// position into its position() accessor (mirrors MockHAL::tick).  RIGHT before
// LEFT to match the retired MockHAL ordering (immaterial to values — no I2C —
// but kept consistent).  Does NOT integrate the plant; integration happens in
// tick(now,cmds) via advance() (the single integration site).
void SimHardware::tick(uint32_t now_ms) {
    _motorR.tick(now_ms);
    _motorL.tick(now_ms);
}

// tick(now, cmds) — the firmware loop's actuator-state tick.  Drives the ONE
// plant integration step and the OTOS/line/color advances (mirrors MockHAL::tick
// → advance with the bench-OTOS branch removed — there is no bench OTOS in SIM).
void SimHardware::tick(uint32_t now_ms, const MotorCommands& cmds) {
    advance(now_ms, cmds);
}

void SimHardware::advance(uint32_t now_ms, const MotorCommands& cmds) {
    int32_t dt = static_cast<int32_t>(now_ms - _lastTickMs);
    if (dt > 0) {
        uint32_t udt = static_cast<uint32_t>(dt);

        // Compute turn rate from the current PWM commands and feed it to the
        // plant before update() so the reported-encoder slip model sees the
        // correct turn intensity.
        // Array convention: [0]=R (FR), [1]=L (FL) — see OutputState.h.
        float aL = fabsf(static_cast<float>(cmds.pwm[1]));
        float aR = fabsf(static_cast<float>(cmds.pwm[0]));
        float turnRate = (aL + aR > 0.5f)
            ? fabsf(static_cast<float>(cmds.pwm[0] - cmds.pwm[1])) / (aL + aR)
            : 0.0f;
        _plant.setTurnRate(turnRate);

        // ONE ordered integration step.  setActuators uses the SAME rounded PWM
        // the control law produced this tick (cmds.pwm[1]=FL=L, cmds.pwm[0]=FR=R).
        _plant.setActuators(static_cast<int8_t>(cmds.pwm[1]),
                            static_cast<int8_t>(cmds.pwm[0]));
        _plant.update(udt);

        // OTOS sim model: sample the plant's true (post-slip) centre pose into
        // the odometer accumulator (ticket 066-001 — ground-truth sampling,
        // replacing the former true-velocity re-integration; see SimOdometer.h).
        _odom.tick(udt);

        // Auxiliary sensor schedules advance on the actuator tick only.
        _line.tick(udt);
        _color.tick(udt);
    }
    _lastTickMs = now_ms;

    // Bench-OTOS dt baseline (074-001): maintained EVERY call, even when bench
    // mode is off — exactly the discipline NezhaHAL::tick(now,cmds) uses (see
    // that function's header comment). If the stamp were only updated while
    // bench mode was active, the FIRST tick after `DBG OTOS BENCH 1` would
    // compute a large stale dt and integrate a spike on the bench plant.
    // Signed-delta avoids uint32 underflow (project memory:
    // watchdog-uint32-underflow).
    int32_t  benchDtSigned = static_cast<int32_t>(now_ms - _lastBenchTick);
    uint32_t benchDt = (benchDtSigned > 0) ? static_cast<uint32_t>(benchDtSigned) : 0u;
    _lastBenchTick = now_ms;

    if (isBenchMode()) {
        // Feed MEASURED wheel travel (SimMotor::position(), the reported
        // encoder value cached by this tick's sensor read), not commanded
        // tgtSpeed — parity with NezhaHAL::tick's encoder feed so bench mode
        // behaves identically in sim and on hardware (see
        // BenchOtosSensor::tickEncoder for the rationale).
        _benchOtos.tickEncoder(_motorL.position(), _motorR.position(),
                               _trackwidth, benchDt);
    }
}
