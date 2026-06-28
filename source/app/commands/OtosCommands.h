#pragma once
#include <vector>
#include "CommandTypes.h"
#include "IOdometer.h"
#include "Inputs.h"

// ---------------------------------------------------------------------------
// OtosCommands — app-layer handler set for the seven OTOS-tuning verbs
// (OI / OZ / OR / OV / OL / OA / OP).  Phase C, Sprint 041 (041-002).
//
// These handlers were formerly the Commandable implementation embedded in
// Odometry.  They were moved here VERBATIM (parse functions, handler functions,
// getCommands body) so the estimator (source/state/) can be dependency-clean —
// no CommandTypes.h / Commandable / Protocol.h.  Behaviour is byte-identical:
// same verbs, same args, same parse guards, same reply strings, same effects on
// the IOdometer device.
//
// Dependency direction (correct): source/app/ -> IOdometer.h (capability) +
// CommandTypes.h (types) + Inputs.h (cached OTOS pose for OP).  The
// estimator does NOT depend on these.
// ---------------------------------------------------------------------------

/**
 * OtosCtx — context bundle for OtosCommands handlers.
 *
 * Both pointers are populated by OtosCommands::setCtx() before any command
 * can arrive.  OTOS command handlers (OI, OZ, OR, OV, OL, OA) reach the
 * IOdometer through this struct.  handleOP reads hwState directly (cached
 * state from the main loop) instead of calling otos->getPositionRaw().
 *
 * Verbatim from Odometry's OdomCtx, minus the unused `odo` self-pointer
 * (none of the seven handlers reference it — that field served the old
 * correct() path, which is not a command handler).
 */
struct OtosCtx {
    IOdometer*           otos;
    const HardwareState* hwState;  // cached OTOS pose for OP read (no device call)
};

/**
 * OtosCommands — Commandable aggregating the seven OTOS-tuning verbs.
 *
 * Registered into the command table by Robot::buildCommandTable in place of
 * the old odometry.getCommands().  Owns its OtosCtx as a value member so the
 * context pointer placed into each CommandDescriptor stays valid for the
 * lifetime of this object.
 */
class OtosCommands : public Commandable {
public:
    OtosCommands();

    virtual std::vector<CommandDescriptor> getCommands() const override;

    // Bind the IOdometer device and cached HardwareState pointer.
    // Call from the Robot constructor after otos and state.inputs are live.
    // hwState may be nullptr in unit tests that do not exercise OP; handleOP
    // checks for null before dereferencing.
    void setCtx(IOdometer* otos, const HardwareState* hwState = nullptr) {
        _ctx.otos    = otos;
        _ctx.hwState = hwState;
    }

private:
    mutable OtosCtx _ctx;  // context bundle for Commandable handlers
};
