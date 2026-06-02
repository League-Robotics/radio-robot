#pragma once
#include "MicroBit.h"
#include "Config.h"

/**
 * Motor — I2C driver for one channel of the PlanetX Nezha V2 motor controller.
 *
 * I2C address: 0x10 (7-bit).
 *
 * Protocol verified against PlanetX pxt-nezha2/main.ts:
 *   Motor start (8-byte write):
 *     [0xFF, 0xF9, motorId, direction, 0x60, speed, 0xF5, 0x00]
 *     direction: 1=CW (forward from chip perspective), 2=CCW (reverse)
 *     speed: 0-100 (absolute)
 *
 *   Encoder read (8-byte write + 4-byte read):
 *     Write: [0xFF, 0xF9, motorId, 0x00, 0x46, 0x00, 0xF5, 0x00]
 *     Read:  4 bytes, signed int32 little-endian, units = tenths of degrees
 *
 *   Encoder zero is maintained in software (offset scalar), matching the
 *   TypeScript resetRelAngleValue() behaviour.
 *
 * Constructor args:
 *   motorId  — 1 = M1 (right wheel), 2 = M2 (left wheel)
 *   fwdSign  — +1 or -1; maps the logical "forward" command to chip direction.
 *              Right wheel requires -1 because the motor is mounted mirrored.
 *
 * Vendor register coverage:
 *   Register | Method             | Sprint
 *   ---------|--------------------|-------
 *   0x60     | setSpeed()         | 008
 *   0x5F     | setSpeed(0) (stop) | 008
 *   0x46     | readEncoderRaw()   | 008
 *   (offset) | resetEncoder()     | 008
 *   0x47     | readSpeedRaw()     | 008
 */
class Motor {
public:
    Motor(MicroBitI2C& i2c, uint8_t motorId, int8_t fwdSign);

    // Set speed as signed percentage (-100..100). Positive = logical forward.
    // fwdSign is applied internally to map logical direction to chip direction.
    // Stores the commanded direction in _lastDir for readSpeed() sign inference.
    void    setSpeed(int8_t pct);

    // Read cumulative encoder in mm using calibration from cfg.
    // Uses mmPerDegL if motorId==LEFT_MOTOR, mmPerDegR otherwise.
    int32_t readEncoder(const RobotConfig& cfg) const;

    // Zero this motor's encoder accumulator (software offset reset,
    // matches chip TypeScript resetRelAngleValue() behaviour).
    void    resetEncoder();

    /**
     * readSpeed — read chip-native wheel velocity.
     *
     * Issues a readSpeed command (register 0x47) and converts the raw uint16
     * reading to mm/s using:
     *   laps_per_sec = floor(raw / 3.6) * 0.01
     *   mm_per_sec   = laps_per_sec * cfg.lapsToMmScale * _lastDir
     *
     * Sign convention: the chip reports unsigned magnitude only. Direction is
     * inferred from _lastDir (set by the most recent setSpeed() call). When
     * the motor is stopped (_lastDir == 0), velocity is reported as 0.
     *
     * IMPORTANT: cfg.lapsToMmScale is an empirically-pinned constant. The
     * default value in defaultRobotConfig() is provisional and must be
     * confirmed against bench measurements (drive at multiple PWM values,
     * compare chip mm/s to encoder-derived mm/s, adjust until they agree).
     * See SUC-003 bench log acceptance criterion.
     *
     * Returns true on success; false if the I2C transaction fails (caller
     * should fall back to encoder-delta velocity).
     */
    bool readSpeed(float& mmPerSec, const RobotConfig& cfg) const;

private:
    MicroBitI2C& _i2c;
    uint8_t      _motorId;  // 1=M1/right, 2=M2/left
    int8_t       _fwdSign;  // +1 or -1

    // Commanded direction: +1 = logical forward, -1 = logical reverse, 0 = stopped.
    // Set by setSpeed(); read by readSpeed() to apply sign to the unsigned chip reading.
    int8_t _lastDir;

    static constexpr uint8_t ADDR    = 0x10;
    static constexpr uint8_t DIR_CW  = 1;   // positive speed from chip perspective
    static constexpr uint8_t DIR_CCW = 2;   // negative speed from chip perspective

    // Software encoder offset (tenths of degrees), zeroed by resetEncoder().
    mutable int32_t _encOffset;

    // Write an 8-byte motor command to the chip.
    void    writeMotorCmd(uint8_t direction, uint8_t speed);

    // Read raw cumulative encoder from chip for this motor (tenths of degrees,
    // minus the software offset).
    int32_t readEncoderRaw() const;

    // Read raw speed from chip register 0x47 (uint16 LE, unsigned magnitude).
    // Returns -1 on I2C error.
    int32_t readSpeedRaw() const;
};
