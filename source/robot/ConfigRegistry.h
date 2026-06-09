#pragma once
#include <stddef.h>
#include "Protocol.h"
#include "../types/CommandTypes.h"
#include "../types/Config.h"

// Forward declaration — avoids pulling in MotorController.h's full header
// graph into every TU that includes ConfigRegistry.h.
class MotorController;

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
// ---------------------------------------------------------------------------
struct ConfigEntry {
    const char*     key;
    ConfigFieldType type;
    size_t          offset;  // offsetof(RobotConfig, field)
};

// ---------------------------------------------------------------------------
// CfgCtx — subsystem context for handleGet / handleSet.
// Cast from the handlerCtx pointer in HandlerFn-compatible calls.
// ---------------------------------------------------------------------------
struct CfgCtx {
    RobotConfig*    cfg;
    MotorController* mc;
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
