// runtime_blackboard_harness.cpp — off-hardware acceptance harness for
// ticket 087-002 (SUC-001/SUC-006): default-constructs Rt::Blackboard and
// exercises a representative post/take round-trip on every command-plane
// queue/mailbox (driveIn, configIn, poseResetIn, motorIn[0], commandsIn,
// otosSetPoseIn), confirms every state cell defaults to zero/default, and
// confirms each queue's exact vehicle (Mailbox latest-wins vs. WorkQueue
// FIFO-with-capacity) and capacity per architecture-update-r1.md's
// Reference code.
//
// Folds in the acceptance test for source/subsystems/wire_command.h (Decision
// 10's extracted, CODAL-free Channel/CommunicatorToCommandProcessorCommand
// POD): blackboard.h's commandsIn round-trip below IS that proof --
// wire_command.h compiles here with zero CODAL includes (this harness never
// touches MicroBit.h/com/radio.h/com/serial_port.h/subsystems/communicator.h)
// and the value round-trips through Rt::WorkQueue<..., 16> with no aliasing
// (line[] is copied by value, not pointed-to).
//
// Mirrors runtime_queue_harness.cpp's shape exactly (see that file's header
// for the pattern): #includes only source/runtime/blackboard.h (which itself
// includes only messages/*.h, runtime/queue.h, subsystems/hardware.h, and
// subsystems/wire_command.h — no MicroBit.h, no I2CBus, no ARM toolchain).
// Hand-rolled assertions, prints PASS/FAIL, exits nonzero on any failure.
// Run by test_runtime_blackboard.py, which compiles and runs this binary via
// subprocess.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#include "runtime/blackboard.h"

namespace {

// --- Hand-rolled assertion plumbing (same tiny shape as
// runtime_queue_harness.cpp/drivetrain_harness.cpp -- a handful of scenarios
// do not warrant a test framework dependency for a dependency-free host
// harness). ---

int g_failureCount = 0;
std::string g_scenarioName;

void beginScenario(const std::string& name) {
  g_scenarioName = name;
  std::printf("--- %s\n", name.c_str());
}

void fail(const std::string& what) {
  ++g_failureCount;
  std::printf("  FAIL [%s]: %s\n", g_scenarioName.c_str(), what.c_str());
}

void checkTrue(bool condition, const std::string& what) {
  if (!condition) fail(what + " -- expected true, got false");
}

void checkFalse(bool condition, const std::string& what) {
  if (condition) fail(what + " -- expected false, got true");
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

void checkFloatEq(float actual, float expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

// 1. A freshly constructed Blackboard's state-plane cells default to
// zero/default msg:: values, with no subsystem wiring of any kind.
void scenarioBlackboardStateCellsDefaultZero() {
  beginScenario("Blackboard: state-plane cells default to zero/default");
  Rt::Blackboard bb;

  checkUintEq(Rt::kPortCount, 4, "Rt::kPortCount mirrors Subsystems::Hardware::kPortCount (4)");

  for (uint32_t i = 0; i < Rt::kPortCount; ++i) {
    checkFalse(bb.motor[i].connected, "motor[i].connected defaults false");
    checkFalse(bb.motor[i].position.has, "motor[i].position defaults unset (Opt.has == false)");
  }
  checkFalse(bb.drivetrain.connected, "drivetrain.connected defaults false");
  checkFloatEq(bb.encoderPose.pose.x, 0.0f, "encoderPose.pose.x defaults 0");
  checkFloatEq(bb.fusedPose.pose.x, 0.0f, "fusedPose.pose.x defaults 0");
  checkFalse(bb.planner.active, "planner.active defaults false");
  checkFalse(bb.otosValid, "otosValid defaults false");
  checkFloatEq(bb.otos.pose.x, 0.0f, "otos.pose.x defaults 0");

  checkFloatEq(bb.drivetrainConfig.trackwidth, 0.0f, "drivetrainConfig.trackwidth defaults 0");
  for (uint32_t i = 0; i < Rt::kPortCount; ++i) {
    checkFloatEq(bb.motorConfig[i].travel_calib, 0.0f, "motorConfig[i].travel_calib defaults 0");
  }
  checkFloatEq(bb.plannerConfig.a_max, 0.0f, "plannerConfig.a_max defaults 0");
  checkFloatEq(bb.odometerConfig.linear_scalar, 0.0f, "odometerConfig.linear_scalar defaults 0");
}

// 2. A freshly constructed Blackboard's command-plane queues/mailboxes all
// start empty, and motorResetIn[] (a plain flag array, not a queue) starts
// all-false.
void scenarioBlackboardCommandPlaneStartsEmpty() {
  beginScenario("Blackboard: command-plane queues/mailboxes start empty");
  Rt::Blackboard bb;

  checkTrue(bb.commandsIn.empty(), "commandsIn starts empty");
  checkTrue(bb.driveIn.empty(), "driveIn starts empty");
  for (uint32_t i = 0; i < Rt::kPortCount; ++i) {
    checkTrue(bb.motorIn[i].empty(), "motorIn[i] starts empty");
    checkFalse(bb.motorResetIn[i], "motorResetIn[i] starts false");
  }
  checkTrue(bb.configIn.empty(), "configIn starts empty");
  checkTrue(bb.poseResetIn.empty(), "poseResetIn starts empty");
  checkTrue(bb.otosSetPoseIn.empty(), "otosSetPoseIn starts empty");
}

// 3. driveIn (Mailbox<msg::DrivetrainCommand>): capacity-1, latest-wins
// round-trip -- the vehicle Decision 1's authority-gated arbitration is
// built on.
void scenarioDriveInMailboxRoundTrip() {
  beginScenario("Blackboard.driveIn: Mailbox<msg::DrivetrainCommand> post/take round-trip");
  Rt::Blackboard bb;

  msg::DrivetrainCommand cmd;
  cmd.setStandby(true);
  bb.driveIn.post(cmd);
  checkFalse(bb.driveIn.empty(), "post() marks driveIn non-empty");

  // Latest-wins: a second post() before any take() replaces the first.
  msg::DrivetrainCommand cmd2;
  cmd2.setStandby(false);
  bb.driveIn.post(cmd2);

  msg::DrivetrainCommand taken = bb.driveIn.take();
  checkTrue(taken.standby.has, "taken command retains the standby field");
  checkFalse(taken.standby.val, "take() returns only the latest posted value (standby=false)");
  checkTrue(bb.driveIn.empty(), "driveIn empty after take()");
}

// 4. motorIn[0] (Mailbox<msg::MotorCommand>): independent per-port mailbox
// -- posting to port 0 leaves every other port's mailbox untouched
// (Decision 2's per-port independence).
void scenarioMotorInMailboxRoundTrip() {
  beginScenario("Blackboard.motorIn[0]: Mailbox<msg::MotorCommand> post/take round-trip, per-port independence");
  Rt::Blackboard bb;

  msg::MotorCommand cmd;
  cmd.setVelocity(123.0f);
  bb.motorIn[0].post(cmd);
  checkFalse(bb.motorIn[0].empty(), "post() marks motorIn[0] non-empty");
  for (uint32_t i = 1; i < Rt::kPortCount; ++i) {
    checkTrue(bb.motorIn[i].empty(), "motorIn[i] (i != 0) is untouched by motorIn[0]'s post()");
  }

  msg::MotorCommand taken = bb.motorIn[0].take();
  checkTrue(taken.control_kind == msg::MotorCommand::ControlKind::VELOCITY,
            "taken motorIn[0] command retains its VELOCITY control_kind");
  checkFloatEq(taken.control.velocity, 123.0f, "taken motorIn[0] command retains its velocity value");
  checkTrue(bb.motorIn[0].empty(), "motorIn[0] empty after take()");
}

// 5. configIn (WorkQueue<ConfigDelta, 16>): FIFO order preserved, and
// capacity is exactly 16 (the 17th post() is rejected).
void scenarioConfigInWorkQueueCapacity() {
  beginScenario("Blackboard.configIn: WorkQueue<ConfigDelta,16> FIFO + capacity 16");
  Rt::Blackboard bb;

  Rt::ConfigDelta first;
  first.target = Rt::ConfigDelta::kDrivetrain;
  first.port = 0;
  checkTrue(bb.configIn.post(first), "post() #1 succeeds");

  Rt::ConfigDelta second;
  second.target = Rt::ConfigDelta::kMotor;
  second.port = 2;
  checkTrue(bb.configIn.post(second), "post() #2 succeeds");

  Rt::ConfigDelta taken = bb.configIn.take();
  checkTrue(taken.target == Rt::ConfigDelta::kDrivetrain, "take() #1 returns the first-posted delta (FIFO order)");
  checkUintEq(taken.port, 0, "take() #1 retains its port field");

  // Fill to capacity 16 (one already queued: `second`), confirm the 16th
  // succeeds and the 17th is rejected -- proves the exact capacity named in
  // architecture-update-r1.md's Reference code (WorkQueue<ConfigDelta,16>).
  for (int i = 0; i < 15; ++i) {
    Rt::ConfigDelta d;
    d.target = Rt::ConfigDelta::kPlanner;
    d.port = static_cast<uint32_t>(i);
    checkTrue(bb.configIn.post(d), "post() up to capacity 16 succeeds");
  }
  checkUintEq(bb.configIn.size(), 16, "configIn holds exactly 16 elements at capacity");
  Rt::ConfigDelta overflow;
  checkFalse(bb.configIn.post(overflow), "post() #17 is rejected -- configIn capacity is exactly 16");
}

// 6. poseResetIn (WorkQueue<PoseResetCommand, 4>): FIFO order preserved,
// and capacity is exactly 4 (the 5th post() is rejected).
void scenarioPoseResetInWorkQueueCapacity() {
  beginScenario("Blackboard.poseResetIn: WorkQueue<PoseResetCommand,4> FIFO + capacity 4");
  Rt::Blackboard bb;

  Rt::PoseResetCommand setPoseCmd;
  setPoseCmd.kind = Rt::PoseResetCommand::kSetPose;
  setPoseCmd.pose.x = 10.0f;
  setPoseCmd.pose.y = 20.0f;
  setPoseCmd.pose.h = 1.5f;
  checkTrue(bb.poseResetIn.post(setPoseCmd), "post() #1 (kSetPose) succeeds");

  Rt::PoseResetCommand resetBaselineCmd;
  resetBaselineCmd.kind = Rt::PoseResetCommand::kResetBaseline;
  checkTrue(bb.poseResetIn.post(resetBaselineCmd), "post() #2 (kResetBaseline) succeeds");
  checkUintEq(bb.poseResetIn.size(), 2, "poseResetIn holds 2 queued elements");

  Rt::PoseResetCommand taken = bb.poseResetIn.take();
  checkTrue(taken.kind == Rt::PoseResetCommand::kSetPose, "take() #1 returns the first-posted (kSetPose) command, FIFO order");
  checkFloatEq(taken.pose.x, 10.0f, "take() #1 retains pose.x");
  checkFloatEq(taken.pose.y, 20.0f, "take() #1 retains pose.y");
  checkFloatEq(taken.pose.h, 1.5f, "take() #1 retains pose.h");

  taken = bb.poseResetIn.take();
  checkTrue(taken.kind == Rt::PoseResetCommand::kResetBaseline, "take() #2 returns the second-posted (kResetBaseline) command");
  checkTrue(bb.poseResetIn.empty(), "poseResetIn empty after draining both posted commands");

  // Fill to capacity 4, confirm the 5th is rejected -- the exact capacity
  // named in architecture-update-r1.md's Reference code
  // (WorkQueue<PoseResetCommand,4>).
  for (int i = 0; i < 4; ++i) {
    Rt::PoseResetCommand d;
    d.kind = Rt::PoseResetCommand::kResetBaseline;
    checkTrue(bb.poseResetIn.post(d), "post() up to capacity 4 succeeds");
  }
  checkUintEq(bb.poseResetIn.size(), 4, "poseResetIn holds exactly 4 elements at capacity");
  Rt::PoseResetCommand overflow;
  checkFalse(bb.poseResetIn.post(overflow), "post() #5 is rejected -- poseResetIn capacity is exactly 4");
}

// 7. otosSetPoseIn (Mailbox<msg::SetPose>): capacity-1, latest-wins
// round-trip.
void scenarioOtosSetPoseInMailboxRoundTrip() {
  beginScenario("Blackboard.otosSetPoseIn: Mailbox<msg::SetPose> post/take round-trip");
  Rt::Blackboard bb;

  msg::SetPose pose;
  pose.x = 5.0f;
  pose.y = -3.0f;
  pose.h = 0.25f;
  bb.otosSetPoseIn.post(pose);
  checkFalse(bb.otosSetPoseIn.empty(), "post() marks otosSetPoseIn non-empty");

  msg::SetPose taken = bb.otosSetPoseIn.take();
  checkFloatEq(taken.x, 5.0f, "taken otosSetPoseIn retains x");
  checkFloatEq(taken.y, -3.0f, "taken otosSetPoseIn retains y");
  checkFloatEq(taken.h, 0.25f, "taken otosSetPoseIn retains h");
  checkTrue(bb.otosSetPoseIn.empty(), "otosSetPoseIn empty after take()");
}

// 8. commandsIn (WorkQueue<Subsystems::CommunicatorToCommandProcessorCommand, 16>):
// post/take a command with a known line + returnPath, confirm it
// round-trips BY VALUE with no aliasing -- mutating the source struct (and
// the local buffer it was built from) after posting must not affect the
// value already stored in the queue, and the taken copy must be independent
// of the original (proving the owned char line[256], not an aliasing
// pointer -- Decision 10). Also confirms commandsIn's exact capacity (16).
void scenarioCommandsInWorkQueueValueRoundTrip() {
  beginScenario("Blackboard.commandsIn: WorkQueue<CommunicatorToCommandProcessorCommand,16> "
                "value round-trip, no aliasing, capacity 16");
  Rt::Blackboard bb;

  Subsystems::CommunicatorToCommandProcessorCommand cmd;
  std::strncpy(cmd.line, "S+150+150", sizeof(cmd.line));
  cmd.line[sizeof(cmd.line) - 1] = '\0';
  cmd.returnPath = Subsystems::Channel::RADIO;

  checkTrue(bb.commandsIn.post(cmd), "post() succeeds");

  // Mutate the SOURCE struct after posting -- if commandsIn aliased it
  // (rather than copying by value), this mutation would corrupt the queued
  // entry. It must not.
  std::strncpy(cmd.line, "CLOBBERED", sizeof(cmd.line));
  cmd.returnPath = Subsystems::Channel::SERIAL;

  const Subsystems::CommunicatorToCommandProcessorCommand* peeked = bb.commandsIn.peek(0);
  checkTrue(peeked != nullptr, "peek(0) is non-null on a non-empty commandsIn");
  if (peeked != nullptr) {
    checkTrue(std::strcmp(peeked->line, "S+150+150") == 0,
              "queued entry's line is unaffected by mutating the source struct after post() (no aliasing)");
    checkTrue(peeked->returnPath == Subsystems::Channel::RADIO,
              "queued entry's returnPath is unaffected by mutating the source struct after post() (no aliasing)");
  }

  Subsystems::CommunicatorToCommandProcessorCommand taken = bb.commandsIn.take();
  checkTrue(std::strcmp(taken.line, "S+150+150") == 0, "take() returns the originally posted line, byte-for-byte");
  checkTrue(taken.returnPath == Subsystems::Channel::RADIO, "take() returns the originally posted returnPath");
  checkTrue(bb.commandsIn.empty(), "commandsIn empty after take()");

  // Mutating the TAKEN copy must not affect a subsequent independent post/take
  // -- proves take() itself returns an independent value, not a reference
  // into internal storage.
  std::strncpy(taken.line, "MUTATED-AFTER-TAKE", sizeof(taken.line));
  Subsystems::CommunicatorToCommandProcessorCommand cmd2;
  std::strncpy(cmd2.line, "T+90", sizeof(cmd2.line));
  cmd2.line[sizeof(cmd2.line) - 1] = '\0';
  cmd2.returnPath = Subsystems::Channel::SERIAL;
  bb.commandsIn.post(cmd2);
  Subsystems::CommunicatorToCommandProcessorCommand taken2 = bb.commandsIn.take();
  checkTrue(std::strcmp(taken2.line, "T+90") == 0,
            "a later independent post/take is unaffected by mutating an earlier taken() copy");

  // Exact capacity: 16 (post up to capacity, confirm the 17th is rejected).
  for (int i = 0; i < 16; ++i) {
    Subsystems::CommunicatorToCommandProcessorCommand d;
    std::snprintf(d.line, sizeof(d.line), "PING%d", i);
    d.returnPath = Subsystems::Channel::SERIAL;
    checkTrue(bb.commandsIn.post(d), "post() up to capacity 16 succeeds");
  }
  checkUintEq(bb.commandsIn.size(), 16, "commandsIn holds exactly 16 elements at capacity");
  Subsystems::CommunicatorToCommandProcessorCommand overflow;
  std::strncpy(overflow.line, "OVERFLOW", sizeof(overflow.line));
  overflow.returnPath = Subsystems::Channel::NONE;
  checkFalse(bb.commandsIn.post(overflow), "post() #17 is rejected -- commandsIn capacity is exactly 16");
}

}  // namespace

int main() {
  scenarioBlackboardStateCellsDefaultZero();
  scenarioBlackboardCommandPlaneStartsEmpty();
  scenarioDriveInMailboxRoundTrip();
  scenarioMotorInMailboxRoundTrip();
  scenarioConfigInWorkQueueCapacity();
  scenarioPoseResetInWorkQueueCapacity();
  scenarioOtosSetPoseInMailboxRoundTrip();
  scenarioCommandsInWorkQueueValueRoundTrip();

  if (g_failureCount == 0) {
    std::printf("OK: all Rt::Blackboard scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Rt::Blackboard scenarios\n", g_failureCount);
  return 1;
}
