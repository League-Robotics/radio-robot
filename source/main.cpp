// ---------------------------------------------------------------------------
// main.cpp -- sprint 093 DEBUG shape: the COMMUNICATION PLANE ONLY, in main().
//
// There is no control loop and no hardware here. main() constructs just the
// Communicator (serial + radio), one Rt::Blackboard (the queues commands post
// onto), and one Rt::CommandRouter (parse + dispatch), then runs a bare,
// explicit loop: tick the Communicator, and the instant it holds one wire
// line, hand it straight to the CommandRouter, which parses it
// (CommandProcessor) and posts the result onto the Blackboard queues + replies
// on the arriving channel. Everything -- channels, replies, blackboard -- is
// wired right here; it moves to a function once verified.
//
// This lets us verify the command plane in isolation: PING/VER/ECHO reply, and
// a queue-posting verb (S) can be seen landing on its Blackboard queue via the
// `QLEN` debug command (nothing drains the queues, so a posted command
// accumulates -- QLEN's drive count going 0->1 after `S` is the routing proof).
//
// The DEVICE: identity banner is emitted by Communicator::begin() itself now
// (moved out of main()) -- the announcement is the Communicator's own job.
//
// uBit.sleep(1) yields to CODAL each pass so a received radio datagram
// (Radio::onData, a MessageBus listener) is delivered; serial RX is IRQ-driven
// and needs no yield.
// ---------------------------------------------------------------------------

#include "MicroBit.h"
#include "subsystems/communicator.h"
#include "runtime/blackboard.h"
#include "runtime/command_router.h"

static MicroBit uBit;

// ---------------------------------------------------------------------------
// serialReply / radioReply -- reply adapters. The CommandProcessor calls one
// of these per response line, routing the reply back out on whichever channel
// the command arrived on. Both take the Communicator as ctx and build on its
// primitive sends.
// ---------------------------------------------------------------------------
static void serialReply(const char* msg, void* ctx) {
    static_cast<Subsystems::Communicator*>(ctx)->sendSerial(msg);
}

static void radioReply(const char* msg, void* ctx) {
    static_cast<Subsystems::Communicator*>(ctx)->sendRadio(msg);
}

int main() {
    uBit.init();

    // Comms: the Communicator subsystem (serial + radio, both enabled).
    // begin() brings up both transports AND emits the DEVICE: identity banner.
    static Subsystems::Communicator comm(uBit.serial, uBit.radio, uBit.messageBus);
    comm.configure(msg::CommunicatorConfig());
    comm.begin();

    // The two-plane transport commands post onto, and the pointerless command
    // router that parses + dispatches inbound wire lines against it.
    static Rt::Blackboard bb;
    static Rt::CommandRouter router;
    router.setReplyChannels(serialReply, &comm, radioReply, &comm);

    // The whole loop: Communicator -> CommandRouter -> Blackboard + reply.
    for (;;) {
        uint32_t now = uBit.systemTime();
        comm.tick(now);
        if (comm.hasCommand()) {
            Subsystems::CommunicatorToCommandProcessorCommand command = comm.takeCommand();
            router.route(command, bb);
        }
        uBit.sleep(1);   // yield: radio RX delivery + other fibers
    }

    return 0;
}
