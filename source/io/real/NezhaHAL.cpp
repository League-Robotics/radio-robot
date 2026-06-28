#include "NezhaHAL.h"
#include "Inputs.h"   // MotorCommands full definition (034-001)

NezhaHAL::NezhaHAL(MicroBitI2C& i2c, MicroBitIO& io, const RobotConfig& cfg)
    : _bus(i2c),
      // Canonical wiring (verified at the controller): LEFT wheel = M2 (chip id 2),
      // RIGHT = M1 (chip id 1). Calibration follows motorId inside Motor (id 2 ->
      // mmPerDegL, id 1 -> mmPerDegR), so this honest mapping keeps the per-wheel
      // calibration correct AND makes the encoder-difference heading (encR-encL)/tw
      // CCW+ — matching the (un-negated) OTOS heading and the ENU camera. The motor
      // forward SENSE is inverted, handled separately by fwdSignL/R (flipped in
      // gen_default_config) which does NOT affect the L/R encoder ordering here.
      // (A prior L/R swap here was a mirror-era hack — the overhead camera was
      // vertically flipped at the time — and it inverted the encoder heading.)
      _motorL(_bus, 2, cfg.fwdSignL, cfg),   // chip M2 = physical LEFT
      _motorR(_bus, 1, cfg.fwdSignR, cfg),   // chip M1 = physical RIGHT
      _otos(_bus, cfg),
#ifdef BENCH_OTOS_ENABLED
      _benchOtos(),
#endif
      _line(_bus),
      _color(_bus),
      _portio(io),
      _gripper(io.P1),
      _busDiag(_bus),                   // 039-001: bus-diagnostics adapter (0x10)
      _rawBusAccess(_bus)               // 044-003: raw-bus-access adapter (I2CW/I2CR)
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
    // Prime both motor encoders: the Nezha 0x46 register sits frozen at 0 until
    // the first atomic read (Motor::begin() calls resetEncoder() to trigger it).
    // The I2C bus is live at this point (the sensor begins above already use it).
    _motorL.begin();
    _motorR.begin();
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

// ---------------------------------------------------------------------------
// tick(now_ms) — sensor tick: per-loop split-phase encoder read (039-002).
//
// Replaces the encoder read that Robot::controlCollectSplitPhase performed.
// RIGHT (M1) is read BEFORE LEFT (M2), matching the proven WedgeTest ordering
// the old controlCollectSplitPhase relied on.  Each Motor::tick() issues the
// identical 0x46-write + 4-byte-read transaction (via readEncoderMmFSettle),
// so the bytes on the I2C wire are unchanged.
// ---------------------------------------------------------------------------
void NezhaHAL::tick(uint32_t now_ms)
{
    _motorR.tick(now_ms);   // Right (M1) first — proven ordering (sprint 015)
    _motorL.tick(now_ms);   // Left (M2) second
}

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

    // Array convention: [0]=R (FR), [1]=L (FL) — see OutputState.h.
    benchOtosPtr()->tick(cmds.tgtMms[1], cmds.tgtMms[0], _trackwidthMm, dt_ms);
#else
    // Production: no bench sensor; this override is a no-op.  (034-006)
    (void)now_ms;
    (void)cmds;
#endif
}
