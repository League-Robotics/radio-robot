// ---------------------------------------------------------------------------
// main.cpp -- dev-loop stub (077-001, HAL wired in 077-003).
//
// This is the first hex out of the new source/ tree: MicroBit init, comms
// (serial + radio) wired through the copied Communicator, and the copied
// CommandProcessor dispatching only the liveness command family
// (system_commands.cpp: PING/VER/HELP/ECHO/ID). There is no DEV command
// family yet -- that lands in ticket 5. The full dev loop (comms poll ->
// hal.tick -> drivetrain.tick -> motor.tick -> watchdog.check, per
// architecture-update.md's Component Diagram) is ticket 5's job; this loop
// is comms poll -> dispatch -> hal.tick.
//
// 077-003: instantiates NezhaHal on all four ports and ticks it every loop
// iteration -- this is the "minimal smoke call...to prove the translation
// units build" ticket 3's acceptance criteria calls for (dead-code
// elimination must not hide a link error against nezha_motor.cpp/
// nezha_hal.cpp). Nothing addresses individual motors yet (no DEV commands,
// no apply() calls) -- that wiring is ticket 5's job, which supersedes the
// bare defaultMotorConfigs()/hal below with real per-robot configuration.
//
// 077-004: a single static Subsystems::Drivetrain, configured once, is the
// equivalent smoke reference proving drivetrain.cpp/body_kinematics.cpp
// link into the firmware -- it is not yet bound to a motor pair or ticked
// from the loop (no DEV DT PORTS binding exists yet); ticket 5 wires that.
// ---------------------------------------------------------------------------

#include "MicroBit.h"
#include "communicator.h"
#include "radio_channel.h"
#include "command_processor.h"
#include "system_commands.h"
#include "com/i2c_bus.h"
#include "hal/nezha/nezha_hal.h"
#include "messages/motor.h"
#include "messages/drivetrain.h"
#include "subsystems/drivetrain.h"

static MicroBit uBit;

// ---------------------------------------------------------------------------
// serialReply / radioReply -- reply adapters. CommandProcessor calls one of
// these per response line, routing the reply back out on whichever channel
// (serial or radio) the command arrived on.
// ---------------------------------------------------------------------------
static void serialReply(const char* msg, void* ctx) {
    static_cast<SerialPort*>(ctx)->send(msg);
}

static void radioReply(const char* msg, void* ctx) {
    static_cast<Radio*>(ctx)->send(msg);
}

// 077-003 smoke wiring: four bare MotorConfigs, one per port, with no
// calibration applied -- ticket 5 replaces this with real per-robot
// configuration (loaded the way ticket 5 decides, since source/robot/ does
// not exist this sprint).
static msg::MotorConfig defaultMotorConfigs[Hal::NezhaHal::kPortCount];

static void initDefaultMotorConfigs() {
    for (uint32_t i = 0; i < Hal::NezhaHal::kPortCount; ++i) {
        defaultMotorConfigs[i] = msg::MotorConfig();
        defaultMotorConfigs[i].setPort(i + 1);
    }
}

int main() {
    uBit.init();
    uBit.i2c.setFrequency(100000);

    // Comms: serial + radio, both enabled. radio_channel.cpp (persisted boot
    // channel selection, button-edit UI) is not copied this ticket -- the
    // radio simply comes up on radiochan::kDefault every boot.
    static Communicator comm(uBit.serial, uBit.radio, uBit.messageBus);
    comm.begin(radiochan::kDefault);

    // Command table: liveness only (PING/VER/HELP/ECHO/ID) this ticket.
    static CommandProcessor cmd(systemCommands());
    cmd.setSerialReply(serialReply, &comm.serial());

    // HAL: one NezhaMotor per port (1-4) over the shared I2CBus -- see the
    // file header for why this is a smoke wire, not real DEV-command
    // wiring (that's ticket 5).
    initDefaultMotorConfigs();
    static I2CBus i2cBus(uBit.i2c);
    static Hal::NezhaHal hal(i2cBus, defaultMotorConfigs);
    hal.begin();

    // 077-004 smoke wiring: configure a Drivetrain so drivetrain.cpp (and,
    // through it, kinematics/body_kinematics.cpp) link into this firmware --
    // not bound to a motor pair or ticked from the loop yet (ticket 5's job,
    // once DEV DT PORTS exists to decide which two NezhaHal ports are
    // "left"/"right" for a session).
    static Subsystems::Drivetrain drivetrain;
    drivetrain.configure(msg::DrivetrainConfig());

    char serialLine[256];
    char radioLine[256];

    while (true) {
        if (comm.serial().readLine(serialLine, sizeof(serialLine))) {
            cmd.process(serialLine, serialReply, &comm.serial());
        }
        if (comm.radio().poll(radioLine, sizeof(radioLine))) {
            cmd.process(radioLine, radioReply, &comm.radio());
        }
        hal.tick(uBit.systemTime());
    }

    return 0;
}
