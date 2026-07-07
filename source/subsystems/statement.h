// statement.h -- Subsystems::Channel / Subsystems::
// CommunicatorToCommandProcessorStatement: the host-safe, CODAL-free
// statement POD that Communicator produces and (from sprint 087)
// Rt::Blackboard's statementsIn queue stores by value.
//
// Extracted out of communicator.h (sprint 087 ticket 002, Decision 10 of
// clasi/sprints/087-.../architecture-update-r1.md): Rt::WorkQueue<T, N>
// (source/runtime/queue.h) stores T buf_[N] as a fixed array member, so T
// must be a complete, host-compilable type wherever Rt::Blackboard is
// defined -- there is no forward-declare escape. communicator.h itself pulls
// in MicroBit.h/com/radio.h/com/serial_port.h with no HOST_BUILD guard, so
// the statement type could not stay defined there once a host-instantiable
// Blackboard needed to name it. This header has zero CODAL includes
// (<cstdint> only) so blackboard.h -- and any host test harness that
// includes it -- never transitively drags in MicroBit.h.
//
// line[] is an OWNED fixed-size copy, not an aliasing pointer into
// Communicator's internal buffer (the pre-r1 shape). A value stored in a
// 16-deep Rt::WorkQueue must not alias mutable state a later
// Communicator::tick() can overwrite out from under an unread queued entry
// -- see Decision 10's rationale. Communicator::takeStatement() copies the
// held line into this buffer.
#pragma once

#include <cstdint>

namespace Subsystems {

// Which comms channel a statement line arrived on -- and therefore where its
// reply must be sent.
enum class Channel : uint8_t { NONE, SERIAL, RADIO };

// Command-out edge type, named by its endpoints
// (<Producer>To<Consumer><Payload> per .claude/rules/naming-and-style.md,
// payload=Statement): one parsable statement line plus its return path.
struct CommunicatorToCommandProcessorStatement {
  char line[256];      // owned copy, not an alias
  Channel returnPath;   // where the reply to this line must be sent
};

}  // namespace Subsystems
