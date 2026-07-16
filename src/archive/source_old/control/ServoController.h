#pragma once
#include "CommandTypes.h"
#include "hal/capability/IPositionMotor.h"

/**
 * ServoController — Commandable wrapper around IPositionMotor that owns the GRIP
 * command descriptor.
 *
 * GRIP <deg>  — set servo angle (0–180); replies "OK grip deg=<deg>"
 * GRIP        — query current angle; replies "OK grip deg=<current>"
 *
 * 039-003: canonicalized from IServo (now an alias) to IPositionMotor.  The
 * handler calls commandAngle(angle, 0) / currentAngle() — same semantics as the
 * former setAngle/currentAngle; mode 0 is the hobby-servo default (OQ-3).
 *
 * The old switch case in CommandProcessor.cpp remains live until T010.
 */
class ServoController : public Commandable {
public:
    explicit ServoController(IPositionMotor& srv);
    virtual std::vector<CommandDescriptor> getCommands() const override;

    IPositionMotor& servo() { return _srv; }

private:
    IPositionMotor& _srv;
};
