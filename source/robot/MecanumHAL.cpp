#include "MecanumHAL.h"
#include "Inputs.h"   // MotorCommands full definition (034-001)

MecanumHAL::MecanumHAL(MicroBitI2C& i2c, MicroBitIO& io, const RobotConfig& cfg)
    : _bus(i2c),
      // Port assignments (Nezha V2 motor controller):
      //   FR = M1 (chip id 1), FL = M2 (chip id 2),
      //   BR = M3 (chip id 3), BL = M4 (chip id 4).
      // fwdSign from per-wheel config baked by gen_default_config.py.
      _motorFR(_bus, 1, cfg.fwdSignFR, cfg),   // port 1 — Front-Right
      _motorFL(_bus, 2, cfg.fwdSignFL, cfg),   // port 2 — Front-Left
      _motorBR(_bus, 3, cfg.fwdSignBR, cfg),   // port 3 — Back-Right
      _motorBL(_bus, 4, cfg.fwdSignBL, cfg),   // port 4 — Back-Left
      _otos(_bus, cfg),
#ifdef BENCH_OTOS_ENABLED
      _benchOtos(),
#endif
      _line(_bus),
      _color(_bus),
      _portio(io),
      _gripper(io.P1),
      _busDiag(_bus),
      _rawBusAccess(_bus)
#ifdef BENCH_OTOS_ENABLED
      ,
      _otosActive(&_otos),
      _halfTrackMm(cfg.halfTrack),
      _halfWheelbaseMm(cfg.halfWheelbase),
      _lastBenchTickMs(0u),
      _fwdSignFR(cfg.fwdSignFR),
      _fwdSignFL(cfg.fwdSignFL),
      _fwdSignBR(cfg.fwdSignBR),
      _fwdSignBL(cfg.fwdSignBL)
#endif
{
}

void MecanumHAL::begin()
{
    _otos.begin();
#ifdef BENCH_OTOS_ENABLED
    _benchOtos.begin();   // no-op: sets _initialized = true
#endif
    _line.begin();   // probe may fail gracefully if no line sensor is present
    _color.begin();
    // Prime all four motor encoders: the Nezha 0x46 register sits frozen at 0
    // until the first atomic read (Motor::begin() calls resetEncoder() to trigger
    // it).  The I2C bus is live at this point.
    _motorFR.begin();
    _motorFL.begin();
    _motorBR.begin();
    _motorBL.begin();
}

// ---------------------------------------------------------------------------
// tick(now_ms) — sensor tick: per-loop split-phase encoder read (039-002).
//
// Four motors read in RIGHT-before-LEFT order, extended to the rear pair:
//   FR(port 1), BR(port 3), FL(port 2), BL(port 4).
// This preserves the NezhaHAL convention for the front pair and adds the
// rear pair with the same right-first discipline.
// ---------------------------------------------------------------------------
void MecanumHAL::tick(uint32_t now_ms)
{
    _motorFR.tick(now_ms);   // Front-Right first (port 1)
    _motorBR.tick(now_ms);   // Back-Right  second (port 3)
    _motorFL.tick(now_ms);   // Front-Left  third (port 2)
    _motorBL.tick(now_ms);   // Back-Left   fourth (port 4)
}

// ---------------------------------------------------------------------------
// tick(now_ms, cmds) — actuator-state tick for bench sensor integration.
//
// When bench mode is active, integrates the commanded wheel velocities into
// BenchOtosSensor.
//
// TODO(T5): replace the front-pair-only approximation below with a proper
// MecanumKinematics::forward call once MotorCommands gains the 4-element
// tgtMms[] array for mecanum.  For now, use the
// same differential approximation as NezhaHAL (front-pair vx, zero vy) so
// the bench OTOS plant gives a reasonable forward-motion estimate.
//
// dt baseline is maintained every tick (even when bench mode is off) to
// avoid a large spike on the first tick after bench mode is enabled —
// identical to NezhaHAL's rationale (see NezhaHAL.cpp comment).
// ---------------------------------------------------------------------------
void MecanumHAL::tick(uint32_t now_ms, const MotorCommands& cmds)
{
#ifdef BENCH_OTOS_ENABLED
    int32_t  dt_signed = (int32_t)(now_ms - _lastBenchTickMs);
    uint32_t dt_ms     = (dt_signed > 0) ? (uint32_t)dt_signed : 0u;
    _lastBenchTickMs   = now_ms;

    if (!isBenchMode()) return;

    // Front-pair approximation: treat as differential using FL(L) and FR(R).
    // trackwidthMm used here is 2 * halfTrackMm (full track width).
    // Array convention: [0]=R (FR), [1]=L (FL) — see OutputState.h.
    float trackwidthMm = 2.0f * _halfTrackMm;
    benchOtosPtr()->tick(cmds.tgtMms[1], cmds.tgtMms[0], trackwidthMm, dt_ms);
#else
    (void)now_ms;
    (void)cmds;
#endif
}
