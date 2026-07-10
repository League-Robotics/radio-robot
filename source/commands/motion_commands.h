#pragma once

// ---------------------------------------------------------------------------
// motion_commands.h -- the STOP/TLM wire family (084-002..005, rewritten
// pointerless 087-006).
//
// 097-006 (architecture-update-r2.md Decision 9, Eric's 2026-07-10 redirect
// to a pure-binary firmware): DELETES the S/T/D/R/TURN/RT/G/MOVE/MOVER
// parser/handler pairs and QLEN outright -- unconditionally, not merely
// unregistered the way 093-001 left T/D/R/TURN/RT/G "source-unchanged but
// unwired" (see this file's git history for that prior state; 094-006
// un-registered D/T/RT into the segment path and added MOVE/MOVER on top of
// that same table). Every deleted verb's binary-plane parity
// (`drive`/`segment`/`replace` arms, source/commands/binary_channel.cpp)
// already carries its runtime behavior forward -- hardware-bench-smoke-
// tested (095, drive/stop) and sim-exhaustive (096, segment/replace) -- so
// no behavior is lost, only the SECOND (text) implementation of it. R/TURN/
// G had no binary arm at all and are deleted with no replacement --
// Decision 9's "no consumer-gating, no preservation" ethos explicitly
// overrides r1's earlier "keep until something proven replaces it"
// reasoning for exactly these three (see architecture-update-r2.md's own
// "forced consequence" note). The shared stop-clause text grammar
// (`parseStopClauseValue`/`collectStopClauses`/`packStopKVs`/
// `kMaxStopConds`/`replyStopBadarg`) and `copyCorrId()` had no callers left
// once those six/three handlers were gone and were deleted alongside them.
// `bb.motionIn`/`Rt::MotionCommand` (runtime/commands.h/blackboard.h) are
// now fully unreferenced plumbing as a result -- flagged as a future
// cleanup, explicitly OUT of this ticket's file scope (architecture-
// update-r2.md Open Question 1).
//
// STOP survives (093-001's fix, physical behavior updated by 094-004/006):
// `handleStop` builds a msg::DrivetrainCommand{NEUTRAL} inline (deliberately
// WITHOUT the standby side-channel -- see handleStop's own doc comment in
// motion_commands.cpp for why dev_commands.h's buildDrivetrainStop(), which
// sets standby=true, silently dropped the neutral instead of stopping the
// wheels) and posts it to the same bb.driveIn mailbox the binary `stop` arm
// also targets. STOP is one of this sprint's confirmed liveness-adjacent
// rump verbs (architecture-update-r2.md's 3-verb default: STOP/PING/HELLO)
// -- see system_commands.h for PING/HELLO's own rationale.
//
// TLM (094-006's one-shot pull-based telemetry read) was UNTOUCHED by
// ticket 006 -- its deletion was ticket 008's own scope (the text telemetry
// family), even though its source lived in this file. 097-008 has since
// deleted handleTlm/TLM outright (Decision 9's "no consumer-gating, no
// preservation" ethos, same as every other verb this file names above --
// see that ticket's own Description for why and for the file-edit
// coordination note with this ticket).
//
// StreamingDriveWatchdog -- DELETED (097-006): already-dead code, fed by
// nothing (confirmed in the 097 architecture research before this ticket
// ran; it fed the S-only streaming-drive-silence timeout, and S no longer
// exists). The text SET/GET config family (source/commands/
// config_commands.{h,cpp}) used to `#include "commands/motion_commands.h"`
// for a doc-comment-only reference to this type -- never an actual type use
// (it touched only `bb.streamWatchdogWindow`/`bb.streamWatchdogWindowIn`,
// both plain `uint32_t`/`Mailbox<uint32_t>`, not this class). That file was
// itself deleted outright by ticket 007, so the point is now moot.
// ---------------------------------------------------------------------------


#include <vector>

#include "command_types.h"
#include "runtime/command_router.h"

// Returns the STOP command table, bound to `router`. TLM (094-006's
// one-shot pull-based telemetry read) was deleted by 097-008 (see this
// file's header comment) -- STOP is this file's own sole live verb.
std::vector<CommandDescriptor> motionCommands(Rt::CommandRouter& router);
