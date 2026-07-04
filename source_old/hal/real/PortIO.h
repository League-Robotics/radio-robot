#pragma once
#include "MicroBit.h"
#include "hal/capability/IPortIO.h"
#include <stdint.h>

/**
 * PortIO — CODAL IO abstraction for the four sensor/actuator ports on the
 * ELECFREAKS PlanetX / Nezha V2 expansion board.
 *
 * Real board structure (authoritative — ELECFREAKS pxt-PlanetX basic.ts): each
 * RJ11 port carries TWO micro:bit GPIO lines plus 3V and GND. There is NO
 * separate "digital connector" vs "analog connector" — every port has both
 * lines; they're just wired to different micro:bit pins:
 *
 *     Port   line S1     line S2
 *     J1     P1          P8
 *     J2     P2          P12
 *     J3     P13         P14
 *     J4     P15         P16
 *
 * Only J1/J2's S1 pins (P1/P2) are ADC-capable; P13/P15 are NOT real analog
 * inputs (PlanetX's AnalogRJPin enum only defines J1/J2 for that reason).
 *
 * This class picks one line per port for each access mode:
 *   readDigital/setDigital(port) -> the S2 line: J1=P8, J2=P12, J3=P14, J4=P16
 *   readAnalog/setAnalog(port)   -> the S1 line: J1=P1, J2=P2, J3=P13, J4=P15
 *   (analog on J3/J4 is nominal only — P13/P15 lack an ADC.)
 *
 * Port numbers are 1-based (1..4); out-of-range values return -1 or are ignored.
 */
class PortIO : public IPortIO {
public:
    explicit PortIO(MicroBitIO& io);

    // Set the digital output on port 1..4. Ignores out-of-range port.
    void setDigital(uint8_t port, bool high) override;

    // Read the digital input on port 1..4. Returns 0 or 1, or -1 for invalid port.
    int  readDigital(uint8_t port) const override;

    // Set the analog (PWM) output on port 1..4, value 0..1023.
    void setAnalog(uint8_t port, uint16_t val) override;

    // Read the analog input on port 1..4, value 0..1023. Returns -1 for invalid port.
    int  readAnalog(uint8_t port) const override;

private:
    MicroBitIO& _io;

    // Returns the digital pin for port 1..4, or nullptr for out-of-range.
    MicroBitPin* digitalPin(uint8_t port) const;

    // Returns the analog pin for port 1..4, or nullptr for out-of-range.
    MicroBitPin* analogPin(uint8_t port) const;
};
