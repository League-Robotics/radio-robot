#pragma once
#include <stdint.h>
#include "io/NoopDevices.h"
#include "io/capability/IVelocityMotor.h"
#include "io/capability/ILineSensor.h"
#include "io/capability/IColorSensor.h"
#include "io/capability/IOdometer.h"
#include "io/capability/IPortIO.h"
#include "io/capability/IPositionMotor.h"

// Full definition needed for the tick() overload that takes a const reference.
// MotorCommands is now a using-alias for OutputState (sprint 047-001).
#include "types/Inputs.h"

/**
 * Hardware — abstract HAL registry / factory base class.
 *
 * Concrete subclasses (NezhaHAL for the physical robot, MockHAL for host
 * unit tests) own all device objects and expose them through this interface.
 * Robot takes a Hardware& in its constructor and binds interface refs from
 * the accessors below.
 */
class Hardware {
public:
    virtual ~Hardware() = default;

    virtual IVelocityMotor& motorL()      = 0;
    virtual IVelocityMotor& motorR()      = 0;
    virtual ILineSensor&    lineSensor()  = 0;
    virtual IColorSensor&   colorSensor() = 0;
    virtual IOdometer&      otos()        = 0;
    virtual IPortIO&        portIO()      = 0;
    virtual IPositionMotor& gripper()     = 0;

    // Initialize all owned devices (calls begin() on sensors).
    virtual void begin() = 0;

    // Periodic tick, called once per cooperative loop iteration.
    virtual void tick(uint32_t now_ms) = 0;

    // Actuator-state tick — delivers commanded motor velocities to the HAL
    // so that bench-mode sensor plants (BenchOtosSensor) can integrate them
    // without requiring a downcast in Robot (034-001).
    //
    // Default no-op: subclasses that do not use bench mode (MockHAL until
    // ticket 005, any future minimal HAL) inherit this and require no change.
    // NezhaHAL overrides this to drive BenchOtosSensor when bench mode is on.
    virtual void tick(uint32_t now_ms, const MotorCommands& cmds) {
        (void)now_ms;
        (void)cmds;
    }

    // Bench-OTOS swap (034-003): redirect otos() to the bench sensor (on=true)
    // or restore the real sensor (on=false).  Default no-op for HALs that do
    // not support bench mode (MockHAL uses the overrides below; NezhaHAL
    // overrides with the real swap).
    virtual void setOtosBench(bool on) { (void)on; }

    // Returns true when the bench OTOS sensor is currently active (034-003).
    // Default false for HALs that do not support bench mode.
    virtual bool isBenchMode() const { return false; }

    // -----------------------------------------------------------------------
    // Default Noop accessors for rear motors (mecanum build overrides these
    // in MecanumHAL; all existing HAL subclasses — NezhaHAL, MockHAL,
    // ReplayHAL, SimHardware — inherit these defaults and require no change).
    // Added in ticket 046-003.
    // -----------------------------------------------------------------------
    virtual IVelocityMotor& motorBR()         { return _noopMotor; }
    virtual IVelocityMotor& motorBL()         { return _noopMotor; }
    virtual int             motorCount() const { return 2; }

private:
    // Shared no-op motor instance backing the default motorBR()/motorBL()
    // accessors. Declared last so subclass constructors (which run before
    // the base destructor) never depend on this field's ordering.
    NoopVelocityMotor _noopMotor;
};
