#include "Motor.h"
#include <math.h>

// ---------------------------------------------------------------------------
// I2C wire protocol constants (verified against PlanetX pxt-nezha2/main.ts)
// ---------------------------------------------------------------------------
//
// Every command is an 8-byte write to address 0x10.
// The frame always starts with 0xFF 0xF9 followed by motor-id, then
// a command-specific payload in bytes [3..7].
//
// Motor start (__start):
//   [0xFF, 0xF9, motorId, direction, 0x60, speed, 0xF5, 0x00]
//   direction: 1 = CW (positive speed), 2 = CCW (negative speed)
//   speed: absolute value 0-100
//
// Motor stop (stop):
//   [0xFF, 0xF9, motorId, 0x00, 0x5F, 0x00, 0xF5, 0x00]
//
// Encoder read (readAngle):
//   Write: [0xFF, 0xF9, motorId, 0x00, 0x46, 0x00, 0xF5, 0x00]
//   Read:  4 bytes, signed int32 little-endian, units = tenths of degrees
// ---------------------------------------------------------------------------

Motor::Motor(MicroBitI2C& i2c, uint8_t motorId, int8_t fwdSign)
    : _i2c(i2c), _motorId(motorId), _fwdSign(fwdSign), _lastDir(0), _encOffset(0)
{
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

void Motor::setSpeed(int8_t pct)
{
    // Clamp to [-100, 100].
    if (pct >  100) pct =  100;
    if (pct < -100) pct = -100;

    // Apply fwdSign: positive pct = logical forward; fwdSign maps that to
    // the chip's CW/CCW convention.  For the right wheel, fwdSign = -1 so
    // that a positive command results in CCW chip rotation (physical forward).
    int16_t effective = (int16_t)_fwdSign * (int16_t)pct;

    if (effective == 0) {
        // Zero speed: send the explicit stop command.
        uint8_t stopBuf[8] = {0xFF, 0xF9, _motorId, 0x00, 0x5F, 0x00, 0xF5, 0x00};
        _i2c.write((ADDR << 1), (uint8_t*)stopBuf, 8, false);
        _lastDir = 0;
    } else {
        uint8_t dir   = (effective > 0) ? DIR_CW : DIR_CCW;
        uint8_t speed = (effective > 0) ? (uint8_t)effective : (uint8_t)(-effective);
        writeMotorCmd(dir, speed);
        // Track logical direction (sign of the original pct, not the chip direction)
        // so readSpeed() can apply the correct sign to the unsigned chip reading.
        _lastDir = (pct > 0) ? (int8_t)1 : (int8_t)-1;
    }
}

int32_t Motor::readEncoder(const RobotConfig& cfg) const
{
    // motorId 2 = M2 = left wheel; use mmPerDegL.
    // motorId 1 = M1 = right wheel; use mmPerDegR.
    float mmPerDeg = (_motorId == 2) ? cfg.mmPerDegL : cfg.mmPerDegR;

    int32_t raw = readEncoderRaw();   // tenths of degrees
    // Mirror TypeScript: (raw / 10.0) * mmPerDeg * fwdSign
    float degF  = raw / 10.0f;
    float mmF   = degF * mmPerDeg * (float)_fwdSign;
    return (int32_t)mmF;
}

void Motor::resetEncoder()
{
    // Mirror TypeScript resetRelAngleValue(): snapshot the current raw
    // angle into the software offset so that subsequent reads return zero.
    _encOffset += readEncoderRaw();
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void Motor::writeMotorCmd(uint8_t direction, uint8_t speed)
{
    uint8_t buf[8] = {
        0xFF,
        0xF9,
        _motorId,
        direction,
        0x60,
        speed,
        0xF5,
        0x00
    };
    _i2c.write((ADDR << 1), (uint8_t*)buf, 8, false);
}

int32_t Motor::readEncoderRaw() const
{
    // Vendor pxt-nezha2 readAngle() requires a 4ms delay before and after
    // the write command before reading; omitting these causes corrupt reads.
    fiber_sleep(4);
    uint8_t cmd[8] = {
        0xFF, 0xF9,
        _motorId,
        0x00, 0x46,
        0x00, 0xF5,
        0x00
    };
    _i2c.write((ADDR << 1), (uint8_t*)cmd, 8, false);
    fiber_sleep(4);

    // Read 4 bytes (signed int32, little-endian).
    uint8_t resp[4] = {0, 0, 0, 0};
    _i2c.read((ADDR << 1), (uint8_t*)resp, 4, false);

    int32_t raw = (int32_t)(
        ((uint32_t)resp[3] << 24) |
        ((uint32_t)resp[2] << 16) |
        ((uint32_t)resp[1] <<  8) |
        ((uint32_t)resp[0])
    );

    // Subtract the software offset captured at last resetEncoder() call.
    return raw - _encOffset;
}

int32_t Motor::readSpeedRaw() const
{
    // Vendor pxt-nezha2 readSpeed() — register 0x47.
    // Same 4 ms pre/post delay as readEncoderRaw (required by vendor protocol).
    // Frame: [0xFF, 0xF9, motorId, 0x00, 0x47, 0x00, 0xF5, 0x00]
    // Response: 2 bytes, unsigned uint16 little-endian.
    //
    // The chip returns unsigned speed magnitude; direction must be inferred
    // from the commanded PWM sign (_lastDir), not from this register.
    fiber_sleep(4);
    uint8_t cmd[8] = {
        0xFF, 0xF9,
        _motorId,
        0x00, 0x47,
        0x00, 0xF5,
        0x00
    };
    int writeResult = _i2c.write((ADDR << 1), (uint8_t*)cmd, 8, false);
    fiber_sleep(4);

    if (writeResult != MICROBIT_OK) {
        return -1;  // I2C error sentinel
    }

    // Read 2 bytes (unsigned uint16 LE).
    uint8_t resp[2] = {0, 0};
    int readResult = _i2c.read((ADDR << 1), (uint8_t*)resp, 2, false);

    if (readResult != MICROBIT_OK) {
        return -1;  // I2C error sentinel
    }

    uint16_t raw = (uint16_t)(((uint16_t)resp[1] << 8) | (uint16_t)resp[0]);
    return (int32_t)raw;
}

bool Motor::readSpeed(float& mmPerSec, const RobotConfig& cfg) const
{
    int32_t raw = readSpeedRaw();
    if (raw < 0) {
        // I2C error — caller should fall back to encoder-delta velocity.
        mmPerSec = 0.0f;
        return false;
    }

    // Convert raw uint16 to laps/s using the vendor formula:
    //   laps_per_sec = floor(raw / 3.6) * 0.01
    //
    // cfg.lapsToMmScale converts laps/s to mm/s. This constant is
    // empirically pinned from bench measurements (see SUC-003 bench log).
    // The default in defaultRobotConfig() is provisional pending calibration.
    float lapsPerSec = floorf((float)raw / 3.6f) * 0.01f;
    float magnitude  = lapsPerSec * cfg.lapsToMmScale;

    // Apply direction sign: the chip returns unsigned speed only.
    // _lastDir is +1 (forward), -1 (reverse), or 0 (stopped).
    mmPerSec = magnitude * (float)_lastDir;
    return true;
}
