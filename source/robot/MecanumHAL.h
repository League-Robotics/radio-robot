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
#include "I2CBusRawAccess.h"
#include "Config.h"

/**
 * MecanumHAL — concrete HAL implementation for a 4-wheel mecanum drivetrain
 * using the PlanetX Nezha V2 motor controller (046-003).
 *
 * Sibling of NezhaHAL: same base class (Hardware), same non-motor devices
 * (OTOS, color sensor, portIO, gripper, bus diagnostics, raw bus access).
 *
 * Four Motor members on Nezha ports 1–4:
 *   _motorFR  (port 1) — Front-Right  (motorR() / motorBR accessor FR alias)
 *   _motorFL  (port 2) — Front-Left   (motorL() / motorBL accessor FL alias)
 *   _motorBR  (port 3) — Back-Right
 *   _motorBL  (port 4) — Back-Left
 *
 * motorL() → FL (front-left, semantic "left").
 * motorR() → FR (front-right, semantic "right").
 * motorBL() → _motorBL; motorBR() → _motorBR.
 * motorCount() → 4.
 *
 * tick(now_ms): split-phase encoder read in RIGHT-BEFORE-LEFT order, extended
 *   to all four motors: FR(1), BR(3), FL(2), BL(4).
 *
 * LineSensor: constructed exactly like NezhaHAL; begin() probe fails gracefully
 * if no physical line sensor is present (no crash, no bus wedge).
 *
 * NOTE: This header includes CODAL/MicroBit headers and must NOT be included
 * from host-build translation units — same restriction as NezhaHAL.h.
 */
class MecanumHAL : public Hardware {
public:
    MecanumHAL(MicroBitI2C& i2c, MicroBitIO& io, const RobotConfig& cfg);

    // Hardware interface overrides ----------------------------------------
    IVelocityMotor& motorL()    override { return _motorFL; }  // front-left
    IVelocityMotor& motorR()    override { return _motorFR; }  // front-right
    IVelocityMotor& motorBR()   override { return _motorBR; }
    IVelocityMotor& motorBL()   override { return _motorBL; }
    int             motorCount() const override { return 4; }

    ILineSensor&  lineSensor()  override { return _line; }
    IColorSensor& colorSensor() override { return _color; }
#ifdef BENCH_OTOS_ENABLED
    IOdometer&    otos()        override { return *_otosActive; }
#else
    IOdometer&    otos()        override { return _otos; }
#endif
    IPortIO&      portIO()      override { return _portio; }
    IPositionMotor& gripper()   override { return _gripper; }

    // Initialise all devices: OTOS, line, color, prime all four motor encoders.
    void begin() override;

    // Sensor tick (039-002 pattern): per-loop split-phase encoder read for all
    // four motors.  Order: FR(1), BR(3), FL(2), BL(4) — RIGHT-before-LEFT
    // convention extended to the rear pair.
    void tick(uint32_t now_ms) override;

    // Actuator-state tick (034-001): integrates commanded velocities into the
    // bench OTOS plant when bench mode is active; no-op otherwise.
    void tick(uint32_t now_ms, const MotorCommands& cmds) override;

    // Expose the shared I2CBus for DebugCommands (DBG I2C / I2CW / I2CR).
    I2CBus& bus() { return _bus; }

    // Expose bus diagnostics for MotorController and DebugCommands.
    IBusDiagnostics& busDiagnostics() { return _busDiag; }

    // Expose raw I2C read/write for DebugCommands's I2CW / I2CR handlers.
    IRawBusAccess& rawBusAccess() { return _rawBusAccess; }

#ifdef BENCH_OTOS_ENABLED
    // --- Bench OTOS swap (sprint 031) --- [034-006: bench-build only]

    void setOtosBench(bool on) override {
        _otosActive = on
            ? static_cast<IOdometer*>(&_benchOtos)
            : static_cast<IOdometer*>(&_otos);
    }

    // Overrides Hardware::benchOtosPtr (074-001).
    BenchOtosSensor* benchOtosPtr() override { return &_benchOtos; }

    bool isBenchMode() const override {
        return _otosActive == static_cast<const IOdometer*>(&_benchOtos);
    }

    // Bind the LIVE RobotConfig (Robot's SET-mutated copy) so the bench tick
    // reads the same trackwidth Drive's EKF uses.  Mirrors NezhaHAL.
    void bindLiveConfig(const RobotConfig* cfg) override { _liveCfg = cfg; }
#endif // BENCH_OTOS_ENABLED

private:
    I2CBus           _bus;
    Motor            _motorFR;   // port 1 — Front-Right
    Motor            _motorFL;   // port 2 — Front-Left
    Motor            _motorBR;   // port 3 — Back-Right
    Motor            _motorBL;   // port 4 — Back-Left
    OtosSensor       _otos;
#ifdef BENCH_OTOS_ENABLED
    BenchOtosSensor  _benchOtos;
#endif
    LineSensor       _line;
    ColorSensor      _color;
    PortIO           _portio;
    Servo            _gripper;

    MotorBusDiagnostics _busDiag;
    I2CBusRawAccess     _rawBusAccess;

#ifdef BENCH_OTOS_ENABLED
    IOdometer*       _otosActive;

    // Bench-tick state (034-001): geometry cached from RobotConfig.
    float            _halfTrack       = 0.0f;   // [mm]
    float            _halfWheelbase   = 0.0f;   // [mm]
    uint32_t         _lastBenchTickMs = 0u;

    // LIVE RobotConfig bound via bindLiveConfig(); nullptr until Robot's
    // constructor runs (fallback: the construction-time _halfTrack cache).
    const RobotConfig* _liveCfg       = nullptr;

    // Per-wheel forward signs, cached for bench integration.
    int8_t           _fwdSignFR = -1;
    int8_t           _fwdSignFL =  1;
    int8_t           _fwdSignBR = -1;
    int8_t           _fwdSignBL =  1;
#endif // BENCH_OTOS_ENABLED
};
