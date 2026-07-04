#include "SimServo.h"

void SimServo::commandAngle(uint16_t angle, uint8_t mode) {
    // Host behaviour preserved from MockServo — store the commanded angle
    // verbatim. A sim hobby servo has no Nezha motion mode, so `mode` is ignored.
    (void)mode;
    _angle = angle;
}

uint16_t SimServo::currentAngle() const {
    return _angle;
}
