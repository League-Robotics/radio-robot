// ---------------------------------------------------------------------------
// main.cpp -- dev-loop stub (077-001).
//
// This is the first hex out of the new source/ tree: MicroBit init, comms
// (serial + radio) wired through the copied Communicator, and the copied
// CommandProcessor dispatching only the liveness command family
// (system_commands.cpp: PING/VER/HELP/ECHO/ID). There is no HAL, no
// Drivetrain, and no DEV command family yet -- those land in tickets 3-5.
// The full dev loop (comms poll -> hal.tick -> drivetrain.tick -> motor.tick
// -> watchdog.check, per architecture-update.md's Component Diagram) is
// ticket 5's job; this loop is just comms poll -> dispatch.
// ---------------------------------------------------------------------------

#include "MicroBit.h"
#include "communicator.h"
#include "radio_channel.h"
#include "command_processor.h"
#include "system_commands.h"

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

int main() {
    uBit.init();

    // Comms: serial + radio, both enabled. radio_channel.cpp (persisted boot
    // channel selection, button-edit UI) is not copied this ticket -- the
    // radio simply comes up on radiochan::kDefault every boot.
    static Communicator comm(uBit.serial, uBit.radio, uBit.messageBus);
    comm.begin(radiochan::kDefault);

    // Command table: liveness only (PING/VER/HELP/ECHO/ID) this ticket.
    static CommandProcessor cmd(systemCommands());
    cmd.setSerialReply(serialReply, &comm.serial());

    char serialLine[256];
    char radioLine[256];

    while (true) {
        if (comm.serial().readLine(serialLine, sizeof(serialLine))) {
            cmd.process(serialLine, serialReply, &comm.serial());
        }
        if (comm.radio().poll(radioLine, sizeof(radioLine))) {
            cmd.process(radioLine, radioReply, &comm.radio());
        }
    }

    return 0;
}
