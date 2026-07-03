#pragma once
#include <stdint.h>
#include "hal/capability/IPositionMotor.h"

// ---------------------------------------------------------------------------
// Gripper — thin subsystem wrapper for the OPTIONAL servo actuator (Phase E seam).
//
// Phase E (043-003): names the structural seam for the gripper concern.  The
// gripper is COMMAND-DRIVEN — actuation happens via the GRIP command handler
// through ServoController, NOT polled each tick.  Therefore:
//   - periodic()     is a no-op this sprint (the servo is not polled per cycle).
//   - updateInputs() is a no-op this sprint (no gripper state in HardwareState).
//
// This ticket is purely STRUCTURAL and ADDITIVE: it establishes the seam and the
// GripperIONull null-object so Phase F has a clean hook.  loopTickOnce does NOT
// call gripper.periodic() — zero behavior change, golden-TLM stays byte-exact.
// The existing ServoController servoController member (which dispatches GRIP) is
// unchanged; this subsystem does not touch command behavior.
//
// No virtual dispatch, no SubsystemBase: a standalone value-type class held by
// Robot.  Holds a reference to the position-motor (servo) capability interface,
// live for the lifetime of the owning Robot.  The no-op methods are non-virtual
// so GripperIONull simply inherits them with zero dispatch cost on nRF52.
//
// No printf / telemetryEmit inside any method (Phase F logging-contract pre-cut).
//
// No vendor / device-runtime types — depends only on the capability interface
// (io/capability/IPositionMotor.h), satisfying the vendor-confinement fence.
//
// Namespaced under `subsystems` for consistency with the Drive and sensor
// subsystems and to keep the global namespace clear.  Robot exposes it as the
// member `gripper_sub` (a name chosen to NOT shadow the existing `IServo& gripper`
// device ref on Robot — OQ-4), so call sites read robot.gripper_sub.periodic().
// ---------------------------------------------------------------------------
namespace subsystems {

class Gripper {
public:
    explicit Gripper(IPositionMotor& servo) : _servo(servo) {}

    // updateInputs — no-op: no gripper state lives in HardwareState yet (Phase F
    // may add it).  Present as the conceptual seam documented for Phase F.
    void updateInputs() {}

    // periodic — no-op: the gripper is command-driven (GRIP via ServoController),
    // not polled each tick.  NOT called from loopTickOnce this sprint.
    void periodic() {}

    // servo — accessor for the bound position-motor interface.  Not used this
    // sprint (actuation still flows through ServoController); exposed so Phase F
    // can reach the device through the subsystem seam if needed.
    IPositionMotor& servo() { return _servo; }

private:
    IPositionMotor& _servo;
};

// ---------------------------------------------------------------------------
// GripperIONull — null-object for has_gripper = false.
//
// A concrete Gripper variant whose periodic() / updateInputs() are no-ops (it
// inherits the no-op Gripper methods unchanged).  It binds a private static
// NullPositionMotor so it satisfies Gripper's IPositionMotor& dependency without
// a live device.  Used in place of `if (has_gripper)` guards at future call
// sites: the null-object pattern lets the caller always invoke the same methods.
//
// Not instantiated by Robot this sprint (Robot binds a real Gripper to the
// existing `gripper` ref); it exists so Phase F has the optional-gripper seam.
// ---------------------------------------------------------------------------
class GripperIONull : public Gripper {
public:
    GripperIONull() : Gripper(nullServo()) {}

private:
    // NullPositionMotor — minimal no-op IPositionMotor for the null-object.
    // commandAngle discards the command; currentAngle always reports 0.
    struct NullPositionMotor : public IPositionMotor {
        void     commandAngle(uint16_t /*angle*/, uint8_t /*mode*/) override {}
        uint16_t currentAngle() const override { return 0; }
    };

    // A single shared null servo backing every GripperIONull instance.  No state
    // is mutated through it (commandAngle is a no-op), so sharing is safe.
    static IPositionMotor& nullServo() {
        static NullPositionMotor instance;
        return instance;
    }
};

}  // namespace subsystems
