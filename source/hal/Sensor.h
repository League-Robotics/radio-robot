#pragma once

/**
 * Sensor — small base class for the I2C sensors (OTOS, color, line).
 *
 * Each concrete sensor implements begin() to detect + initialize its device
 * over I2C and set _initialized to the result.  All public read/write methods
 * gate on is_initialized() and become no-ops when the device is absent, so a
 * sensor can be disabled simply by not calling its begin().
 */
class Sensor {
public:
    virtual ~Sensor() {}
    // Detect + initialize the device over I2C; set _initialized; return it.
    virtual bool begin() = 0;
    bool is_initialized() const { return _initialized; }
protected:
    bool _initialized = false;
};
