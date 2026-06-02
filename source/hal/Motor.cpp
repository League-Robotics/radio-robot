#include "Motor.h"

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
    : _i2c(i2c), _motorId(motorId), _fwdSign(fwdSign), _encOffset(0)
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
    } else {
        uint8_t dir   = (effective > 0) ? DIR_CW : DIR_CCW;
        uint8_t speed = (effective > 0) ? (uint8_t)effective : (uint8_t)(-effective);
        writeMotorCmd(dir, speed);
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
