// motion_commands.cpp -- STOP handler. 097-006 (architecture-update-r2.md
// Decision 9) deleted the S/T/D/R/TURN/RT/G/MOVE/MOVER parser/handler pairs,
// QLEN, the shared stop-clause text grammar (parseStopClauseValue/
// collectStopClauses/packStopKVs/kMaxStopConds/replyStopBadarg), copyCorrId
// (orphaned once R/TURN/G were gone), and StreamingDriveWatchdog (already
// dead) -- see motion_commands.h for the file-level rationale and each
// deleted verb's binary-plane parity pointer. 097-008 additionally deleted
// handleTlm/TLM (the one-shot text telemetry verb) -- see motion_commands.h
// for why its deletion lives in this file but was ticket 008's own scope,
// not 006's. STOP's own handler and registration are byte-for-byte
// unchanged by either ticket.
#include "commands/motion_commands.h"


#include "commands/command_processor.h"
#include "messages/drivetrain.h"

namespace {

Rt::Blackboard& bb(void* handlerCtx) { return static_cast<Rt::CommandRouter*>(handlerCtx)->blackboard(); }

// ---------------------------------------------------------------------------
// handleStop -- 093-001 (fixed), physical behavior updated by 094-004/006:
// STOP posts a NEUTRAL msg::DrivetrainCommand straight to bb.driveIn, built
// inline WITHOUT the standby side-channel -- deliberately NOT
// dev_commands.h's buildDrivetrainStop() helper, which sets {NEUTRAL,
// standby=true}. That shape was found to be a correctness bug (093-001):
// the pre-094 routeOutputs() step posted the computed NEUTRAL wheel command
// to bb.motorIn[] only when drivetrain_.active() was true, and
// Subsystems::Drivetrain::apply() processed standby=true AFTER the NEUTRAL
// arm, immediately flipping active_ back to false in the same apply() call
// -- so the neutral command was silently dropped and the wheels kept
// spinning at their last commanded speed. Leaving standby unset keeps the
// drivetrain active, so the neutral reaches the motors. That
// routeOutputs()/bb.motorIn[] plumbing is itself gone now (094-005 --
// Drivetrain stages its own wheel writes directly through hardware_'s
// motor refs), but the underlying reason to leave standby unset (a
// subsequent `S` re-activates via setWheelTargets() regardless, and there
// is still no authority-steal producer in this trimmed table for the
// standby gate to protect against) is unchanged, so this handler's own
// NEUTRAL construction is unchanged.
//
// PHYSICAL EFFECT changed with 094-004's Drivetrain rewrite, though: this
// same NEUTRAL command no longer means "instant brake" in every case.
// Subsystems::Drivetrain::dispatchEscapeHatch() (drivetrain.cpp) inspects
// whether a Motion::Segment is actively executing (SEGMENT mode AND
// executor_.active()) when a NEUTRAL arrives -- if so, it arms the owned
// Motion::SegmentExecutor's OWN presolved graceful decel-to-zero
// (executor_.stop(now)) instead of zeroing the wheels instantly, and this
// Drivetrain keeps riding that decel down to a literal 0.0f twist over
// subsequent ticks (architecture-update.md Section 6, "STOP triggers the
// graceful decel-to-zero" -- the communicator issue's own fix request).
// Only when there is nothing in-flight to decelerate (a plain `S` then
// `STOP`, no segment ever queued, or the executor was already idle) does
// STOP fall straight through to the pre-094 instant-neutral behavior --
// see test_bare_loop_commands.py's own DIRECT-mode STOP test, still green
// unchanged, and test_bare_loop_move_and_tlm.py's SEGMENT-mode STOP test
// (094-006) for the graceful path, both exercised over this SAME handler.
// This handler itself needed no code change for that behavior switch --
// entirely a Drivetrain-level (094-004) decision on the SAME NEUTRAL
// command shape this handler already built. No EVT. Reply stays `OK stop`
// unchanged, even though the physical effect it describes changed.
// ---------------------------------------------------------------------------
void handleStop(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
                void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);

  msg::DrivetrainCommand cmd;
  cmd.setNeutral(msg::Neutral::BRAKE);
  b.driveIn.post(cmd);

  char rbuf[32];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "stop", nullptr, corrId, replyFn, replyCtx);
}

}  // namespace

// 097-006/097-008: gutted to STOP only (see this file's header comment).
// S/D/T/R/TURN/RT/G/MOVE/MOVER/QLEN (097-006) and TLM (097-008) are
// DELETED, not merely unregistered -- their parser/handler functions no
// longer exist anywhere in this file.
std::vector<CommandDescriptor> motionCommands(Rt::CommandRouter& router) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(makeCmd("STOP", nullptr, handleStop, &router, "badarg", ForceReply::NONE,
                         CMD_ACCESS_HARDWARE));
  return cmds;
}
