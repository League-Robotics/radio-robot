#include "SimServo.h"

void SimServo::setAngleDeg(uint16_t deg, uint8_t mode) {
    // Host behaviour preserved from MockServo — store the commanded angle
    // verbatim. A sim hobby servo has no Nezha motion mode, so `mode` is ignored.
    (void)mode;
    _angle = deg;
}

uint16_t SimServo::currentAngleDeg() const {
    return _angle;
}
