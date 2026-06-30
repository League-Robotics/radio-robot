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
 *     simMotorR/L positionMm() accessors (RIGHT before LEFT, matching MockHAL).
 *
 * Control law stays ABOVE the device line (Case B): the per-wheel PI+FF runs in
 * MotorController; SimMotor only stores PWM.  No second controller here.
 *
 * Value-member ownership (zero heap).  No CODAL dependency; compiles host-side.
 */
class SimHardware : public Hardware {
public:
    explicit SimHardware(const RobotConfig& cfg);

    // Hardware interface -----------------------------------------------------
    IVelocityMotor& motorL()    override { return _motorL; }
    IVelocityMotor& motorR()    override { return _motorR; }
    ILineSensor&  lineSensor()  override { return _line; }
    IColorSensor& colorSensor() override { return _color; }
    IOdometer&    otos()        override { return _odom; }
    IPortIO&      portIO()      override { return _portIO; }
    IPositionMotor& gripper()   override { return _servo; }

    void begin() override {
        _motorL.begin();
        _motorR.begin();
    }
    void tick(uint32_t now_ms) override;
    void tick(uint32_t now_ms, const MotorCommands& cmds) override;

    // No bench-OTOS SENSOR SWAP in SIM mode — there is no bench OTOS device here
    // (sim_bench_otos_* drive the standalone SimHandle::benchOtos).  But the flag
    // is RECORDED so the DBG OTOS BENCH round-trip command can observe the toggle
    // through isBenchMode() (behaviour preserved from MockHAL, which likewise only
    // recorded the flag host-side).  otos() always returns the SimOdometer.
    void setOtosBench(bool on) override { _benchMode = on; }
    bool isBenchMode() const override { return _benchMode; }

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
    void setTrackwidth(float mm) { _trackwidthMm = mm; _plant.setTrackwidth(mm); }

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

    uint32_t       _lastTickMs   = 0;
    float          _trackwidthMm = 0.0f;
    bool           _benchMode    = false;   // DBG OTOS BENCH round-trip flag only
};
