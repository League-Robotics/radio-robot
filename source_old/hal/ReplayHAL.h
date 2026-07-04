#pragma once
#include <stdint.h>
#include "Hardware.h"
#include "hal/NoopDevices.h"
#include "hal/capability/IVelocityMotor.h"
#include "hal/capability/IPositionMotor.h"
#include "hal/capability/IOdometer.h"
#include "hal/capability/ILineSensor.h"
#include "hal/capability/IColorSensor.h"
#include "hal/capability/IPortIO.h"

/**
 * ReplayHAL — stub Hardware implementation for ROBOT_RUN_MODE=REPLAY (039-005).
 *
 * Phase F will implement deterministic TLM-frame replay (feeding recorded
 * sensor data back through the capability interfaces so a captured field run
 * can be re-executed off-robot). For now this is an empty, no-op HAL that
 * compiles, satisfies the Hardware pure-virtual contract, and is selected only
 * when ROBOT_RUN_MODE=REPLAY in CMake. It is NOT wired for real use yet.
 *
 * Every device accessor returns a reference to a trivial no-op capability
 * implementation owned as a value member (zero-heap, value-member ownership —
 * the same ownership model NezhaHAL / MockHAL use). The no-op feed impls below
 * return zeros / report success so a replay-mode build links and runs without
 * touching any hardware.
 */
class ReplayHAL : public Hardware {
public:
    ReplayHAL() = default;
    ~ReplayHAL() override = default;

    IVelocityMotor& motorL()      override { return _motorL; }
    IVelocityMotor& motorR()      override { return _motorR; }
    ILineSensor&    lineSensor()  override { return _line; }
    IColorSensor&   colorSensor() override { return _color; }
    IOdometer&      otos()        override { return _otos; }
    IPortIO&        portIO()      override { return _portIO; }
    IPositionMotor& gripper()     override { return _gripper; }

    void begin() override {}
    void tick(uint32_t now_ms) override { (void)now_ms; }

private:
    // NoopVelocityMotor is now in io/NoopDevices.h (046-003); it is included
    // above so existing callers that forwarded through ReplayHAL.h still see it.

    // ---- No-op position motor (gripper / servo) ----
    class NoopPositionMotor : public IPositionMotor {
    public:
        void     commandAngle(uint16_t angle, uint8_t mode) override { (void)angle; (void)mode; }
        uint16_t currentAngle() const override { return 0; }
    };

    // ---- No-op odometer ----
    class NoopOdometer : public IOdometer {
    public:
        bool begin() override { _initialized = false; return false; }
        bool readTransformed(Pose2D& poseOut, float headingRad = 0.0f) const override {
            (void)headingRad;
            poseOut = Pose2D{0.0f, 0.0f, 0.0f};
            return false;
        }
        bool readVelocityTransformed(BodyTwist& velOut, float headingRad = 0.0f) const override {
            (void)headingRad;
            velOut = BodyTwist{0.0f, 0.0f};
            return false;
        }
        bool      readStatus(uint8_t& out) const override { out = 0; return false; }
        bool      lastReadOk() const override { return false; }
        BodyAccel readAccelTransformed() const override { return BodyAccel{0.0f, 0.0f}; }
        void      init() override {}
        void      calibrateImu(uint8_t samples) override { (void)samples; }
        void      resetTracking() override {}
        void      getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const override { x = 0; y = 0; h = 0; }
        void      setPositionRaw(int16_t x, int16_t y, int16_t h) override { (void)x; (void)y; (void)h; }
        int8_t    getLinearScalar() const override { return 0; }
        void      setLinearScalar(int8_t val) override { (void)val; }
        int8_t    getAngularScalar() const override { return 0; }
        void      setAngularScalar(int8_t val) override { (void)val; }
    };

    // ---- No-op line sensor ----
    class NoopLineSensor : public ILineSensor {
    public:
        bool begin() override { _initialized = false; return false; }
        bool readValues(uint16_t out[4]) const override {
            out[0] = out[1] = out[2] = out[3] = 0;
            return false;
        }
        bool readNormalized(uint16_t out[4]) override {
            out[0] = out[1] = out[2] = out[3] = 0;
            return false;
        }
    };

    // ---- No-op color sensor ----
    class NoopColorSensor : public IColorSensor {
    public:
        bool begin() override { _initialized = false; return false; }
        bool readRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) override {
            r = g = b = c = 0;
            return false;
        }
        bool pollRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) override {
            r = g = b = c = 0;
            return false;
        }
    };

    // ---- No-op port IO ----
    class NoopPortIO : public IPortIO {
    public:
        void setDigital(uint8_t port, bool high) override { (void)port; (void)high; }
        int  readDigital(uint8_t port) const override { (void)port; return -1; }
        void setAnalog(uint8_t port, uint16_t val) override { (void)port; (void)val; }
        int  readAnalog(uint8_t port) const override { (void)port; return -1; }
    };

    NoopVelocityMotor _motorL;
    NoopVelocityMotor _motorR;
    NoopPositionMotor _gripper;
    NoopOdometer      _otos;
    NoopLineSensor    _line;
    NoopColorSensor   _color;
    NoopPortIO        _portIO;
};
