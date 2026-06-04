#include "MicroBit.h"
#include "Robot.h"
#include "CommandProcessor.h"
#include "LoopScheduler.h"
#include "SerialPort.h"
#include "Icons.h"

// ---------------------------------------------------------------------------
// MicroBit uBit singleton — must be file-scope so CODAL peripherals are
// fully initialised before Robot is constructed in main().
// ---------------------------------------------------------------------------
static MicroBit uBit;

// ---------------------------------------------------------------------------
// serialReply — thin adapter used for the boot HELLO banner.
// ---------------------------------------------------------------------------
static void serialReply(const char* msg, void* ctx)
{
    static_cast<SerialPort*>(ctx)->send(msg);
}

// ---------------------------------------------------------------------------
// main — constructs the robot and runs the single cooperative main loop.
//
// Single cooperative main loop architecture (014-006/007):
//
//   LoopScheduler::run() (never returns):
//     1. HARD TASK: split-phase encoder COLLECT → velocity (ZOH) → PID → PWM.
//     2. LOW-PRIORITY SWEEP: comms-in, drive-advance, odometry-predict,
//        otos-correct, line-read, color-read, ports-read, telemetry-emit.
//        Round-robin, persistent cursor, budget-gated against controlDeadline.
//     3. ENCODER REQUEST: fire next wheel request (last I2C before idle).
//     4. IDLE SLEEP: sleep until controlDeadline.
//
// No CODAL fibers. All I/O inline. All task entry points on Robot.
// ---------------------------------------------------------------------------

int main() {
    uBit.init();

    // Show a heart on the 5x5 LED matrix as a "powered and ready" indicator.
    uBit.display.printAsync(icons::boot()); // delay=0 → show forever, non-blocking

    static Robot            robot(uBit.i2c, uBit.serial, uBit.radio,
                                  uBit.io, uBit.messageBus, uBit);
    static CommandProcessor cmd(robot);

    // Emit DEVICE: identification banner once at boot over serial.
    cmd.process("HELLO", serialReply, &robot.serialPort());

    // Run the cooperative main loop — never returns.
    static LoopScheduler sched(robot, cmd, uBit);
    sched.run();

    return 0;
}
