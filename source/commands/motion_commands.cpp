// motion_commands.cpp -- STOP/TLM handlers. 097-006 (architecture-update-r2.md
// Decision 9) deleted the S/T/D/R/TURN/RT/G/MOVE/MOVER parser/handler pairs,
// QLEN, the shared stop-clause text grammar (parseStopClauseValue/
// collectStopClauses/packStopKVs/kMaxStopConds/replyStopBadarg), copyCorrId
// (orphaned once R/TURN/G were gone), and StreamingDriveWatchdog (already
// dead) -- see motion_commands.h for the file-level rationale and each
// deleted verb's binary-plane parity pointer. STOP's own handler and
// registration are byte-for-byte unchanged by this ticket; TLM's are
// untouched (ticket 008's scope).
#include "commands/motion_commands.h"


#include <cstdio>
#include <math.h>

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

// ---------------------------------------------------------------------------
// handleTlm -- 094-006: one-shot, SNAP-style synchronous read of
// bb.drivetrain (itself sourced from Subsystems::Drivetrain::state(),
// populated every pass by Rt::MainLoop::commit()/main.cpp's own commit
// line -- MEASURED per-wheel encoder position/velocity, not a commanded
// target, since 094-004's rewrite of Drivetrain::state()). Replies through
// the command's own ReplyFn/ctx, exactly like PING/STOP above -- no
// blackboard post (the simplest handler in this file), no EVT, no periodic
// timer, no loop-output queue: architecture-update.md Section 7 Decision 2
// explicitly rules out reviving the pre-093 STREAM/SNAP drain seam for a
// single one-shot producer. Reply shape is this ticket's own choice --
// `OK tlm ...`, wrapped like every other verb in this trimmed table,
// deliberately NOT the pre-093 SNAP verb's unwrapped raw `TLM t=...` line
// (docs/protocol-v2.md section 8) -- see this ticket's completion notes for
// the docs/protocol-v2.md reconciliation this implies (that update itself
// is deferred, per architecture-update.md Step 7 Open Question 1).
// `active=` reports msg::DrivetrainState.active, which 094-006 also widened
// (drivetrain.cpp's state()) to OR in the owned Motion::SegmentExecutor's
// own active/idle status alongside the pre-079 authority flag -- see that
// method's own doc comment for why the authority flag alone would report
// `active=0` throughout a MOVE-only session.
// ---------------------------------------------------------------------------
void handleTlm(const ArgList& /*args*/, const char* corrId, ReplyFn replyFn, void* replyCtx,
              void* handlerCtx) {
  Rt::Blackboard& b = bb(handlerCtx);
  const msg::DrivetrainState& dt = b.drivetrain;

  float encL = dt.enc_count_val() >= 1 ? dt.enc()[0] : 0.0f;
  float encR = dt.enc_count_val() >= 2 ? dt.enc()[1] : 0.0f;
  float velL = dt.vel_count_val() >= 1 ? dt.vel()[0] : 0.0f;
  float velR = dt.vel_count_val() >= 2 ? dt.vel()[1] : 0.0f;
  float cmdL = dt.cmd_count_val() >= 1 ? dt.cmd()[0] : 0.0f;
  float cmdR = dt.cmd_count_val() >= 2 ? dt.cmd()[1] : 0.0f;
  float accL = dt.acc_count_val() >= 1 ? dt.acc()[0] : 0.0f;
  float accR = dt.acc_count_val() >= 2 ? dt.acc()[1] : 0.0f;

  // cmd= is the post-governor commanded wheel velocity (the setpoint the
  // velocity PID chases) vs measured vel=; acc= is the firmware-EMA measured
  // acceleration (raw host-side d(vel)/dt is quantization noise). conn=
  // surfaces per-drive-motor I2C health (NezhaMotor::connected(), via
  // bb.motors[] which main.cpp commits each pass) -- conn=0,0 with
  // everything else ACKing = the Nezha brick is off the bus. glitch= is the
  // cumulative count of encoder samples rejected by the leaf's source-side
  // plausibility gate (corrupted reads; a rising count = bus noise). Drive
  // pair = bound indices 0/1 -> motors[0]/[1].
  unsigned glitchL = b.motors[0].enc_glitch_count.has ? b.motors[0].enc_glitch_count.val : 0;
  unsigned glitchR = b.motors[1].enc_glitch_count.has ? b.motors[1].enc_glitch_count.val : 0;
  // ts= -- each wheel's OWN sample instant (firmware loop clock): the
  // flip-flop samples the two motors on different ~40-80ms slots, so a host
  // plotting both at poll-receive time renders an aliasing staircase; these
  // stamps let it place every reading at its true time (2026-07-09
  // smooth-telemetry fix). enc=/vel= gain 0.1 resolution for the same
  // reason -- integer truncation was adding artificial texture.
  unsigned tsL = b.motors[0].sampled_at.has ? b.motors[0].sampled_at.val : 0;
  unsigned tsR = b.motors[1].sampled_at.has ? b.motors[1].sampled_at.val : 0;
  // Tenths rendered with integer math: the firmware's newlib-nano snprintf
  // has no float support linked (%f silently emits NOTHING -- verified on
  // the bench: `enc=, vel=,`), and pulling in _printf_float costs flash.
  auto formatTenths = [](char* out, size_t n, float v) {
    long t = lroundf(v * 10.0f);
    const char* sign = (t < 0) ? "-" : "";
    if (t < 0) t = -t;
    snprintf(out, n, "%s%ld.%ld", sign, t / 10, t % 10);
  };
  char encLs[16], encRs[16], velLs[16], velRs[16];
  formatTenths(encLs, sizeof(encLs), encL);
  formatTenths(encRs, sizeof(encRs), encR);
  formatTenths(velLs, sizeof(velLs), velL);
  formatTenths(velRs, sizeof(velRs), velR);
  char body[240];
  snprintf(body, sizeof(body),
           "enc=%s,%s vel=%s,%s cmd=%d,%d acc=%d,%d active=%d conn=%d,%d glitch=%u,%u ts=%u,%u now=%u",
           encLs, encRs, velLs, velRs,
           static_cast<int>(cmdL), static_cast<int>(cmdR),
           static_cast<int>(accL), static_cast<int>(accR),
           // active= reports BUSY (motion in progress), not the authority
           // flag -- setNeutral() sets the authority flag TRUE (holding
           // neutral IS governing), so dt.active latches 1 after the first
           // STOP and can never mean "idle". See DrivetrainState.busy.
           dt.busy ? 1 : 0,
           b.motors[0].connected ? 1 : 0, b.motors[1].connected ? 1 : 0,
           glitchL, glitchR, tsL, tsR, b.loopNow);
  char rbuf[272];
  CommandProcessor::replyOK(rbuf, sizeof(rbuf), "tlm", body, corrId, replyFn, replyCtx);
}

}  // namespace

// 097-006: gutted to STOP + TLM (see this file's header comment). S/D/T/R/
// TURN/RT/G/MOVE/MOVER/QLEN are DELETED, not merely unregistered -- their
// parser/handler functions no longer exist anywhere in this file.
std::vector<CommandDescriptor> motionCommands(Rt::CommandRouter& router) {
  std::vector<CommandDescriptor> cmds;
  cmds.push_back(makeCmd("STOP", nullptr, handleStop, &router, "badarg", ForceReply::NONE,
                         CMD_ACCESS_HARDWARE));
  // 094-006: TLM -- one-shot synchronous read of bb.drivetrain; no
  // CMD_ACCESS_HARDWARE flag (it reads the ALREADY-committed blackboard
  // cell, not hardware directly, at dispatch time). Untouched by 097-006 --
  // ticket 008's own scope, coordinated to avoid a duplicate/conflicting
  // edit to this same file (see motion_commands.h's header comment).
  cmds.push_back(makeCmd("TLM", nullptr, handleTlm, &router, "badarg"));
  return cmds;
}
