#pragma once
#include "CommandTypes.h"
#include "io/capability/IBusDiagnostics.h"
#include "io/capability/IRawBusAccess.h"

// Forward declarations — keeps the header-graph shallow.
class LoopScheduler;
struct Robot;

// ---------------------------------------------------------------------------
// DbgCtx — context bundle passed to every DBG / I2CW / I2CR handler.
//
// 044-003 (Phase F): the vendor bus pointer is replaced by two narrow capability
// pointers, sealing the final vendor leak above source/io/.  busDiag serves the
// DBG I2C / I2CLOG / IRQGUARD diagnostics; busAccess serves the I2CW / I2CR raw
// transfers.  Both are implemented by adapters in source/io/real/ and are null
// in the host build (DebugCommandable's I2C handlers are #ifndef HOST_BUILD).
// ---------------------------------------------------------------------------
struct DbgCtx {
    LoopScheduler*   sched;
    IBusDiagnostics* busDiag;    // was: the vendor bus* (DBG I2C / I2CLOG / IRQGUARD)
    IRawBusAccess*   busAccess;  // new: I2CW / I2CR raw byte transfers
    Robot*           robot;
};

// ---------------------------------------------------------------------------
// DebugCommandable — Commandable that owns all diagnostic commands:
//   DBG LOOP RESET, DBG LOOP, DBG I2CLOG, DBG I2C, DBG IRQGUARD,
//   DBG WEDGE, I2CW, I2CR.
//
// All descriptors use ForceReply::SERIAL so debug output always goes to
// the serial port regardless of which channel the command arrived on.
//
// Handler logic mirrors the existing switch cases in CommandProcessor.cpp
// exactly.  Those switch cases remain live until T011 cutover.
// ---------------------------------------------------------------------------
class DebugCommandable : public Commandable {
public:
    explicit DebugCommandable(DbgCtx ctx);

    virtual std::vector<CommandDescriptor> getCommands() const override;

    // Accessor used by handler functions in DebugCommandable.cpp.
    DbgCtx ctx() const { return _ctx; }

private:
    DbgCtx _ctx;
};
