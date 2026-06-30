#pragma once
#include <stddef.h>
#include "Protocol.h"
#include "../types/CommandTypes.h"
#include "../types/Config.h"

// Forward declarations — avoids pulling full header graphs into every TU
// that includes ConfigRegistry.h.
class MotorController;
class MotionController2;
namespace subsystems { class Drive2; }
namespace subsystems { class Sensors; }

// ---------------------------------------------------------------------------
// ConfigFieldType — discriminator for a RobotConfig field's wire encoding.
// ---------------------------------------------------------------------------
enum ConfigFieldType {
    CFG_FLOAT,        // float field, wire format: %.3f
    CFG_INT,          // int32_t field, wire format: %d
    CFG_FLOAT_AS_INT  // float field stored as integer magnitude, wire format: %d
};

// ---------------------------------------------------------------------------
// ConfigEntry — one row in kRegistry[]: maps a friendly key name to a
// RobotConfig field via byte offset and wire type.
//
// subsystem — annotation for live SET routing (059-004).  When non-null,
//   handleSet builds a typed config delta and calls the owning subsystem's
//   configure() method AFTER the normal direct-write commit:
//     "drive"   → builds msg::DrivetrainConfig delta → drive2.configure()
//     "planner" → builds msg::PlannerConfig delta    → planner.configure()
//     "sensors" → builds sensor config delta         → sensors.configure()
//   nullptr means the existing kRegistry[] direct-write path only (no
//   configure() call for this field).  Both mechanisms coexist by design:
//   the direct write keeps RobotConfig consistent; configure() pushes the
//   delta into the live subsystem.
// ---------------------------------------------------------------------------
struct ConfigEntry {
    const char*     key;
    ConfigFieldType type;
    size_t          offset;     // offsetof(RobotConfig, field)
    const char*     subsystem;  // "drive" | "planner" | "sensors" | nullptr
};

// ---------------------------------------------------------------------------
// CfgCtx — subsystem context for handleGet / handleSet.
// Cast from the handlerCtx pointer in HandlerFn-compatible calls.
//
// Subsystem pointers (059-004): optional — nullptr when the new-architecture
// subsystems are not yet wired into the live loop (ticket 005 cutover).
// handleSet checks for non-null before routing; unannotated fields and null
// subsystem pointers fall through to the existing direct-write path.
// ---------------------------------------------------------------------------
struct CfgCtx {
    RobotConfig*            cfg;
    MotorController*        mc;
    subsystems::Drive2*     drive2    = nullptr;  // SET "drive" routing
    MotionController2*      planner   = nullptr;  // SET "planner" routing
    subsystems::Sensors*    sensors   = nullptr;  // SET "sensors" routing
};

// ---------------------------------------------------------------------------
// kRegistry[] — complete config key-to-field mapping (~50 entries).
// Defined in ConfigRegistry.cpp.
// ---------------------------------------------------------------------------
extern const ConfigEntry kRegistry[];
extern const int         kRegistryCount;

// ---------------------------------------------------------------------------
// handleGet — HandlerFn-compatible GET handler.
//
//   args.args[0..args.count-1].sval = requested key names (empty list = dump all)
//   corrId    = correlation id string (may be "" but never nullptr)
//   replyFn   = reply callback
//   replyCtx  = opaque context forwarded to replyFn
//   handlerCtx = CfgCtx* (cast inside function)
//
// Emits one CFG response line, plus one ERR badkey per unknown key.
// ---------------------------------------------------------------------------
void handleGet(const ArgList& args, const char* corrId,
               ReplyFn replyFn, void* replyCtx, void* handlerCtx);

// ---------------------------------------------------------------------------
// handleSet — HandlerFn-compatible SET handler.
//
//   args.args[0..args.count-1].sval = "key=value" strings (one per pair)
//   corrId    = correlation id string
//   replyFn   = reply callback
//   replyCtx  = opaque context forwarded to replyFn
//   handlerCtx = CfgCtx* (cast inside function)
//
// Emits OK set <applied> once, plus ERR badkey per unknown key.
// Calls MotorController::updatePidGains / updateVelGains when relevant
// params change.
// ---------------------------------------------------------------------------
void handleSet(const ArgList& args, const char* corrId,
               ReplyFn replyFn, void* replyCtx, void* handlerCtx);
