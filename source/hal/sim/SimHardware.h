#pragma once
#include <stdint.h>
#include "../Hardware.h"
#include "PhysicsWorld.h"
#include "SimMotor.h"
#include "SimOdometer.h"
#include "SimLineSensor.h"
#include "SimColorSensor.h"
#include "SimPortIO.h"
#include "SimServo.h"
#include "hal/real/BenchOtosSensor.h"

struct RobotConfig;

/**
 * SimHardware — SIM-mode Hardware implementation (Sprint 040 Phase B, 040-002).
 *
 * Replaces MockHAL.  Owns the ONE PhysicsWorld plant (the single source of ground
 * truth) and constructs each observation model against it.  Its tick(now,cmds) does
 * the ONE ordered plant.update(dt); the observation models then read the plant.
 *
 * Tick ordering (identical to MockHAL semantic):
 *   tick(now, cmds):  plant integration + OTOS/line/color advance (actuator tick)
 *     plant.setTurnRate(turn) ; plant.setActuators(cmds.pwmL, cmds.pwmR)
 *     plant.update(dt)                       // ONE integration step
 *     odom.tick(trueVelL, trueVelR, tw, dt)  // OTOS sim model
 *     line.tick(dt) ; color.tick(dt)
 *   tick(now):        sensor-read tick — promotes plant reported encoders into
 *     simMotorR/L position() accessors (RIGHT before LEFT, matching MockHAL).
 *
 * Control law stays ABOVE the device line (Case B): the per-wheel PI+FF runs in
 * MotorController; SimMotor only stores PWM.  No second controller here.
 *
 * Value-member ownership (zero heap).  No CODAL dependency; compiles host-side.
 *
 * Active-OTOS pointer (074-001 — bench-swap parity with NezhaHAL/MecanumHAL):
 *   otos() returns *_otosActive, which starts pointed at _odom (the ground-truth
 *   SimOdometer).  setOtosBench(true) redirects _otosActive to _benchOtos (an
 *   owned BenchOtosSensor, reused as-is from source/hal/real/); setOtosBench(false)
 *   restores _odom.  advance() drives _benchOtos.tick(...) every actuator tick
 *   using its own dt baseline (_lastBenchTick), gated on isBenchMode() — mirroring
 *   NezhaHAL::tick(now,cmds)'s anti-spike-on-enable discipline exactly.
 */
class SimHardware : public Hardware {
public:
    explicit SimHardware(const RobotConfig& cfg);

    // Hardware interface -----------------------------------------------------
    IVelocityMotor& motorL()    override { return _motorL; }
    IVelocityMotor& motorR()    override { return _motorR; }
    ILineSensor&  lineSensor()  override { return _line; }
    IColorSensor& colorSensor() override { return _color; }
    // otos() returns the ACTIVE odometer — real (SimOdometer ground truth) or
    // bench (BenchOtosSensor), depending on _otosActive (074-001).
    IOdometer&    otos()        override { return *_otosActive; }
    IPortIO&      portIO()      override { return _portIO; }
    IPositionMotor& gripper()   override { return _servo; }

    void begin() override {
        _motorL.begin();
        _motorR.begin();
    }
    void tick(uint32_t now_ms) override;
    void tick(uint32_t now_ms, const MotorCommands& cmds) override;

    // Bench-OTOS swap (074-001): redirect the active OTOS pointer to the owned
    // BenchOtosSensor (on=true) or restore the real SimOdometer (on=false).
    // Mirrors NezhaHAL::setOtosBench exactly, giving host-sim the same real
    // swap firmware already has (previously flag-only: _benchMode recorded the
    // toggle but nothing read it, and otos() always returned _odom).
    void setOtosBench(bool on) override {
        _otosActive = on
            ? static_cast<IOdometer*>(&_benchOtos)
            : static_cast<IOdometer*>(&_odom);
    }

    // Returns true when the bench sensor is currently active.  Overrides
    // Hardware::isBenchMode (074-001).
    bool isBenchMode() const override {
        return _otosActive == static_cast<const IOdometer*>(&_benchOtos);
    }

    // Direct accessor to the owned BenchOtosSensor.  Overrides
    // Hardware::benchOtosPtr (074-001) — always non-null for SimHardware,
    // unlike production firmware without BENCH_OTOS_ENABLED.
    BenchOtosSensor* benchOtosPtr() override { return &_benchOtos; }

    // Bind the LIVE RobotConfig (Robot's SET-mutated copy) so the bench
    // branch reads the same rotationalSlip/trackwidth the firmware's encpose
    // heading law uses.  Mirrors NezhaHAL.  Overrides Hardware::bindLiveConfig.
    void bindLiveConfig(const RobotConfig* cfg) override { _liveCfg = cfg; }

    // Test accessors ---------------------------------------------------------
    PhysicsWorld&   plant()          { return _plant; }
    SimMotor&       simMotorL()      { return _motorL; }
    SimMotor&       simMotorR()      { return _motorR; }
    SimOdometer&    simOdometer()    { return _odom; }
    SimLineSensor&  simLineSensor()  { return _line; }
    SimColorSensor& simColorSensor() { return _color; }
    SimPortIO&      simPortIO()      { return _portIO; }
    SimServo&       servoSim()       { return _servo; }

    // Robot trackwidth (mm) so the OTOS sim model integrates correctly.
    void setTrackwidth(float mm) { _trackwidth = mm; _plant.setTrackwidth(mm); }

    // Trackwidth getter (069-003) — forwards to the plant (the single source
    // of truth); SimHardware's own _trackwidth cache always mirrors it.
    float trackwidth() const { return _plant.trackwidth(); }

    // Ground-truth pass-throughs (ticket 057-005): expose the plant's authoritative
    // integrated pose so Python test shims can compare fused output vs. ground truth.
    float groundTruthX() const { return _plant.groundTruthX(); }
    float groundTruthY() const { return _plant.groundTruthY(); }
    float groundTruthH() const { return _plant.groundTruthH(); }

private:
    // Shared dt-guarded plant integration for tick(now,cmds).
    void advance(uint32_t now_ms, const MotorCommands& cmds);

    PhysicsWorld   _plant;
    SimMotor       _motorL;
    SimMotor       _motorR;
    SimOdometer    _odom;
    SimLineSensor  _line;
    SimColorSensor _color;
    SimPortIO      _portIO;
    SimServo       _servo;
    BenchOtosSensor _benchOtos;   // owned bench-OTOS sensor (074-001)

    uint32_t       _lastTickMs   = 0;
    float          _trackwidth = 0.0f;

    // LIVE RobotConfig bound via bindLiveConfig(); nullptr until Robot's
    // constructor runs (fallback: _trackwidth cache, slip=1).
    const RobotConfig* _liveCfg = nullptr;

    // Bench-tick dt baseline (074-001): mirrors NezhaHAL::_lastBenchTick.
    // Maintained every advance() call, even when bench mode is off, so the
    // first tick after `DBG OTOS BENCH 1` does not see a large stale dt and
    // integrate a spike on the plant.
    uint32_t       _lastBenchTick = 0u;   // [ms]

    // Active OTOS pointer — initialized to &_odom in the constructor.
    // Must be declared AFTER both _odom and _benchOtos so those members are
    // fully constructed before _otosActive is assigned (074-001, mirrors
    // NezhaHAL::_otosActive).
    IOdometer*     _otosActive;
};
