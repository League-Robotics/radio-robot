// telemetry_tick.h -- tickTelemetry(): the loop-owned periodic-emission
// step (096-002; relocated verbatim from commands/binary_channel.{h,cpp} by
// a stakeholder-directed cleanup after 097-011 -- see binary_channel.h's
// git history for the file's shape before this move). The periodic-
// telemetry emission functions do not belong in the command dispatcher;
// they only landed there when 097-011 deleted telemetry_commands.{h,cpp}
// and folded its contents into binary_channel.{h,cpp} as an expedient.
// This dedicated pair separates the loop-driven emission step from
// tlm_frame.{h,cpp}'s pure frame-assembly (Telemetry::tick()/
// buildTelemetryMessage()), which it calls into and which stays untouched
// by this move.
#pragma once

#include "runtime/command_router.h"

// tickTelemetry -- the loop-owned periodic-emission step (096-002,
// relocated verbatim from telemetry_commands.h by 097-011, then from
// binary_channel.h by this move): checks bb.telemetryPeriod > 0 and elapsed
// time (bb.telemetryLastEmitMs/bb.telemetryHasLastEmit, the SAME fields the
// deleted text handleStream() used to maintain for its own
// immediate-first-frame emission); if a frame is due, resolves
// bb.telemetryChannel to a live ReplyFn/void* pair via router.replySink()
// (command_router.h) and emits one binary frame via telemetryEmitBinary()
// (.cpp-local). Unconditional since 097-008 -- there is no more text
// sibling to branch against, so bb.telemetryBinary (blackboard.h, still
// written by binary_channel.cpp's `stream` arm) is no longer read here. A
// no-op when no frame is due (bb.telemetryPeriod == 0, or not enough time
// has elapsed since the last emission). Global namespace scope (not nested
// under any namespace) -- matches its pre-move declaration shape exactly,
// so main.cpp's/sim_api.cpp's existing unqualified `tickTelemetry(bb,
// router, now)` call sites needed no change beyond the #include swap.
void tickTelemetry(Rt::Blackboard& bb, Rt::CommandRouter& router, uint32_t now);
