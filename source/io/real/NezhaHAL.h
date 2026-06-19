#pragma once
#include "MicroBit.h"
#include "Hardware.h"
#include "I2CBus.h"
#include "Motor.h"
#include "OtosSensor.h"
// 034-006: BenchOtosSensor is bench-build only.
#ifdef BENCH_OTOS_ENABLED
#include "BenchOtosSensor.h"
#endif
#include "LineSensor.h"
#include "ColorSensor.h"
#include "PortIO.h"
#include "Servo.h"
#include "MotorBusDiagnostics.h"
#include "Config.h"

/**
 * NezhaHAL — concrete HAL implementation for the PlanetX Nezha V2 robot.
 *
 * Owns all seven device objects as value members (no heap allocation).
 * Constructed in main() as a static and passed to Robot as Hardware&.
 *
 * begin() initialises the sensors in the order: OTOS, line, color.
 * tick() is a no-op (devices are self-contained / interrupt-driven).
 *
 * bus() exposes the shared I2CBus so main.cpp can wire it into
 * MotorController::setI2CBus() for the enc_wedge diagnostic.
 *
 * NOTE: This header includes CODAL/MicroBit headers and must NOT be
 * included from host-build translation units. The host build (ticket 003)
 * will use MockHAL instead.
 *
 * Active-OTOS pointer (sprint 031):
 *   _otosActive initially points to _otos (real sensor).
 *   When BENCH_OTOS_ENABLED: setOtosBench(true) redirects _otosActive to
 *   _benchOtos so that otos() returns the bench sensor transparently.
 *   setOtosBench(false) restores the real sensor.
 *   Without BENCH_OTOS_ENABLED (production): _benchOtos / _otosActive are
 *   absent; otos() returns _otos directly; setOtosBench is the base no-op.
 *   (034-006)
 */
class NezhaHAL : public Hardware {
public:
    NezhaHAL(MicroBitI2C& i2c, MicroBitIO& io, const RobotConfig& cfg);

    // Hardware interface overrides ----------------------------------------
    IVelocityMotor& motorL()    override { return _motorL; }
    IVelocityMotor& motorR()    override { return _motorR; }
    ILineSensor&  lineSensor()  override { return _line; }
    IColorSensor& colorSensor() override { return _color; }
#ifdef BENCH_OTOS_ENABLED
    // otos() returns the ACTIVE sensor — real or bench depending on _otosActive.
    IOdometer&    otos()        override { return *_otosActive; }
#else
    // Production: no bench sensor; otos() always returns the real sensor.
    IOdometer&    otos()        override { return _otos; }
#endif
    IPortIO&      portIO()      override { return _portio; }
    IPositionMotor& gripper()   override { return _gripper; }

    // Call otos.begin(), line.begin(), color.begin().
    // With BENCH_OTOS_ENABLED: also calls _benchOtos.begin().
    void begin() override;

    // Sensor tick (039-002): drives the per-loop split-phase encoder read for
    // both wheels.  RIGHT (M1) is ticked BEFORE LEFT (M2) to preserve the exact
    // I2C transaction ordering controlCollectSplitPhase used ("right motor first,
    // then left" — Robot.cpp / WedgeTest sprint 015).  Called once per
    // cooperative-loop iteration before loopTickOnce.
    void tick(uint32_t now_ms) override;

    // Actuator-state tick (034-001): integrates commanded velocities into the
    // bench OTOS plant when bench mode is active; no-op when bench mode is off.
    // Ports the signed-delta dt logic from Robot::benchOtosTick so Robot no
    // longer needs a downcast.
    // In production (no BENCH_OTOS_ENABLED) this degenerates to a no-op.
    void tick(uint32_t now_ms, const MotorCommands& cmds) override;

    // Expose the shared I2CBus for DebugCommandable (DBG I2C / I2CW / I2CR).
    I2CBus& bus() { return _bus; }

    // Expose bus diagnostics as a capability for MotorController::setBusDiagnostics().
    // Routes the motor-controller (0x10) error/reentry/lastErr counters upward
    // without leaking I2CBus above the IO boundary (039-001).
    IBusDiagnostics& busDiagnostics() { return _busDiag; }

#ifdef BENCH_OTOS_ENABLED
    // --- Bench OTOS swap (sprint 031) --- [034-006: bench-build only]

    // Redirect the active OTOS pointer to the bench sensor (on=true) or
    // restore the real sensor (on=false).  Overrides Hardware::setOtosBench
    // (034-003).
    void setOtosBench(bool on) override {
        _otosActive = on
            ? static_cast<IOdometer*>(&_benchOtos)
            : static_cast<IOdometer*>(&_otos);
    }

    // Direct accessor to the BenchOtosSensor for tick() calls and noise tuning.
    BenchOtosSensor* benchOtosPtr() { return &_benchOtos; }

    // Returns true when the bench sensor is currently active.  Overrides
    // Hardware::isBenchMode (034-003).
    bool isBenchMode() const override {
        return _otosActive == static_cast<const IOdometer*>(&_benchOtos);
    }
#endif // BENCH_OTOS_ENABLED

private:
    I2CBus           _bus;
    Motor            _motorL;
    Motor            _motorR;
    OtosSensor       _otos;
#ifdef BENCH_OTOS_ENABLED
    BenchOtosSensor  _benchOtos;
#endif
    LineSensor       _line;
    ColorSensor      _color;
    PortIO           _portio;
    Servo            _gripper;

    // Bus-diagnostics adapter (039-001) — constructed from _bus (declared above,
    // so _bus is fully constructed first). Exposed via busDiagnostics().
    MotorBusDiagnostics _busDiag;

#ifdef BENCH_OTOS_ENABLED
    // Active OTOS pointer — initialized to &_otos in the constructor.
    // Must be declared AFTER both _otos and _benchOtos so those members
    // are fully constructed before _otosActive is assigned.
    IOdometer*       _otosActive;

    // Bench-tick state (034-001): trackwidth cached from RobotConfig at
    // construction; last-tick timestamp for signed-delta dt computation
    // (mirrors the logic formerly in Robot::benchOtosTick).
    float            _trackwidthMm    = 0.0f;
    uint32_t         _lastBenchTickMs = 0u;
#endif // BENCH_OTOS_ENABLED
};
