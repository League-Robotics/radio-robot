#include "MockServo.h"

void MockServo::setAngleDeg(uint16_t deg, uint8_t mode) {
    // 039-003: host behaviour preserved — store the commanded angle verbatim.
    // A mock hobby servo has no Nezha motion mode, so `mode` is ignored.
    (void)mode;
    _angle = deg;
}

uint16_t MockServo::currentAngleDeg() const {
    return _angle;
}
