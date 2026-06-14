#include "NezhaHAL.h"
#include "RobotState.h"   // MotorCommands full definition (034-001)

NezhaHAL::NezhaHAL(MicroBitI2C& i2c, MicroBitIO& io, const RobotConfig& cfg)
    : _bus(i2c),
      // Physical wiring is mirrored vs the original M2=left / M1=right labels:
      // the robot turned CW for +omega (should be CCW under the ENU+CCW camera).
      // M1 is the physical LEFT wheel, M2 the physical RIGHT. Each motor keeps
      // its own chip channel, fwd polarity, and mm/deg calibration (calibration
      // follows motorId inside Motor: id 1 -> mmPerDegR, id 2 -> mmPerDegL); only
      // the L/R role is swapped so +omega -> physical CCW. Forward is unaffected.
      _motorL(_bus, 1, cfg.fwdSignR),   // chip M1 = physical LEFT
      _motorR(_bus, 2, cfg.fwdSignL),   // chip M2 = physical RIGHT
      _otos(_bus, cfg),
#ifdef BENCH_OTOS_ENABLED
      _benchOtos(),
#endif
      _line(_bus),
      _color(_bus),
      _portio(io),
      _gripper(io.P1)
#ifdef BENCH_OTOS_ENABLED
      ,
      _otosActive(&_otos),              // default: real sensor
      _trackwidthMm(cfg.trackwidthMm),  // cache for bench tick (034-001)
      _lastBenchTickMs(0u)
#endif
{
}

void NezhaHAL::begin()
{
    _otos.begin();
#ifdef BENCH_OTOS_ENABLED
    _benchOtos.begin();   // no-op: sets _initialized = true
#endif
    _line.begin();
    _color.begin();
}

// ---------------------------------------------------------------------------
// tick(now_ms, cmds) — actuator-state tick for bench sensor integration.
//
// When bench mode is active, integrates the commanded wheel velocities into
// BenchOtosSensor so that the plant advances position/heading consistently
// with the control loop's outputs.
//
// The dt baseline (_lastBenchTickMs) is maintained EVERY tick — even when bench
// mode is off — exactly as the original Robot::benchOtosTick did.  loopTickOnce
// calls this every loop iteration, so dt tracks the loop period.  If the stamp
// were only updated while bench mode was active, the FIRST tick after
// `DBG OTOS BENCH 1` would compute dt = now_ms (a large spike) and integrate a
// huge step on the plant.  Signed-delta avoids uint32 underflow (project memory:
// watchdog-uint32-underflow).
//
// When bench mode is off this is a near-no-op (one subtraction + store).
//
// Ported from Robot::benchOtosTick; Robot will call this instead of the
// downcast pattern once ticket 002 is implemented (034-001).
// ---------------------------------------------------------------------------

void NezhaHAL::tick(uint32_t now_ms, const MotorCommands& cmds)
{
#ifdef BENCH_OTOS_ENABLED
    // Maintain the dt baseline every tick (see header comment) before the
    // bench-mode gate.
    int32_t  dt_signed = (int32_t)(now_ms - _lastBenchTickMs);
    uint32_t dt_ms     = (dt_signed > 0) ? (uint32_t)dt_signed : 0u;
    _lastBenchTickMs   = now_ms;

    // Early-return when bench mode is off (production path — nearly free).
    if (!isBenchMode()) return;

    benchOtosPtr()->tick(cmds.tgtLMms, cmds.tgtRMms, _trackwidthMm, dt_ms);
#else
    // Production: no bench sensor; this override is a no-op.  (034-006)
    (void)now_ms;
    (void)cmds;
#endif
}
