// binary_channel.h -- BinaryChannel::handle(): the M5 "BinaryChannel"
// module (095-007, architecture-update.md). Translates one armored
// `*B<base64>` binary wire line into a Blackboard post (drive/segment/
// replace/stop) or an inline system-verb reply (ping/echo/id), or a typed
// `Error{ERR_UNIMPLEMENTED}` for the declared-only arms (config/pose/otos/
// get/stream). A malformed line or an out-of-bound field value (caught by
// ticket 005's generated decode() validation) yields a typed
// `Error{code, field}` reply -- never a crash, never a silent drop.
//
// Reaches the Blackboard ONLY through the same opaque
// handlerCtx-cast-to-Rt::CommandRouter* idiom every text command family
// already uses (see architecture-update.md Decision 1) -- never a stored
// pointer of its own, never touches hardware directly. Built on
// source/messages/wire_runtime.{h,cpp} (M3, base64 armor) and
// source/messages/wire.{h,cpp} (M4, decode/encode) -- both already proven
// correct against google.protobuf by ticket 006's differential/fuzz suite
// before this module was written.
#pragma once

#include "types/protocol.h"

namespace BinaryChannel {

// handle() -- dispatched from CommandProcessor::process() when line[0] ==
// '*' (BEFORE parseTokens() runs -- base64 must never be tokenized/
// uppercased; see command_processor.cpp). `line` is the raw, NUL-terminated
// wire line, still carrying its `*B` armor prefix. `replyFn`/`replyCtx` are
// the already-resolved reply channel (same as every text HandlerFn
// receives). `routerCtx` is CommandProcessor's own `_binaryCtx`, set once
// at construction time by Rt::CommandRouter's constructor
// (`processor_.setBinaryContext(this)`) -- cast back to Rt::CommandRouter*
// here, exactly the way every `commands/*.cpp` handler already casts its
// own `handlerCtx`.
void handle(const char* line, ReplyFn replyFn, void* replyCtx, void* routerCtx);

}  // namespace BinaryChannel
