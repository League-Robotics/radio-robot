// dev_command_outbox_harness.cpp -- off-hardware acceptance harness,
// originally for ticket 079-005 (SUC-004/SUC-005/SUC-006/SUC-007): exercises
// the DEV command family's pure-transformer reshape -- raw wire commands in,
// posted commands (+ replies) out, no handler ever calling
// Hal::Motor::apply()/Subsystems::Drivetrain::apply() directly.
//
// Rewritten pointerless for sprint 087 ticket 006: drives the REAL parse
// path (full ASCII command lines through Rt::CommandRouter::route(),
// built from the REAL devCommands() table via command_router.cpp's unified
// six-family table) and asserts on Rt::Blackboard's queues (bb.motorIn[]/
// bb.driveIn/bb.hardwareBroadcastIn/bb.configIn) and the captured reply text
// afterward -- exactly the "wire commands in, posted commands + replies out"
// contract, now against the Blackboard's command-plane instead of a deleted
// DevLoopState's outbox fields.
//
// None of these scenarios need to script the I2CBus at all: capability
// pre-validation (now against bb.motorCaps[], a boot-time snapshot -- see
// blackboard.h's file header), port binding (bb.drivetrainConfig.left_port/
// right_port), and queue posting never touch the bus -- a freshly
// constructed NezhaHardware is enough, used only to seed bb.motorCaps[]'s
// snapshot.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "com/i2c_bus.h"
#include "commands/dev_commands.h"   // buildBroadcastNeutral()/buildDrivetrainStop()
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "runtime/blackboard.h"
#include "runtime/command_router.h"
#include "subsystems/nezha_hardware.h"

namespace {

// --- Hand-rolled assertion plumbing (see motor_policy_harness.cpp /
// nezha_flipflop_harness.cpp / drivetrain_harness.cpp) ---

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
  if (std::fabs(static_cast<double>(actual - expected)) > 1e-4) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

void checkMotorKindEq(msg::MotorCommand::ControlKind actual,
                       msg::MotorCommand::ControlKind expected, const std::string& what) {
  if (actual != expected) fail(what + " -- msg::MotorCommand::ControlKind mismatch");
}

void checkDtKindEq(msg::DrivetrainCommand::ControlKind actual,
                    msg::DrivetrainCommand::ControlKind expected, const std::string& what) {
  if (actual != expected) fail(what + " -- msg::DrivetrainCommand::ControlKind mismatch");
}

// --- Fixture: a fresh, boot-equivalent DEV command stack per scenario ---

msg::MotorConfig g_configsTemplate[Subsystems::NezhaHardware::kPortCount];

void initConfigsTemplate() {
  for (uint32_t i = 0; i < Subsystems::NezhaHardware::kPortCount; ++i) {
    g_configsTemplate[i] = msg::MotorConfig{};
    g_configsTemplate[i].setPort(i + 1).setFwdSign(1).setTravelCalib(1.0f);
  }
}

// Captures every reply line Rt::CommandRouter::route() emits for one
// command (usually one line, occasionally more -- e.g. a CFG badkey ERR
// followed by the ack).
struct ReplyCapture {
  std::vector<std::string> lines;
};

void captureReply(const char* msg, void* ctx) {
  static_cast<ReplyCapture*>(ctx)->lines.push_back(msg);
}

struct Fixture {
  I2CBus bus;
  Subsystems::NezhaHardware hal;
  Rt::Blackboard bb;
  Rt::CommandRouter router;

  Fixture() : hal(bus, g_configsTemplate) {
    // Default binding: left=1, right=2 (Rt::Blackboard's own zero default --
    // matches Subsystems::Drivetrain::ports()'s pre-087 default of the same
    // shape, since no boot config generator runs in this harness).
    bb.drivetrainConfig.left_port = 1;
    bb.drivetrainConfig.right_port = 2;

    // Boot-time capability snapshot (blackboard.h's file header) -- DEV M's
    // capability pre-validation gate (dev_commands.cpp) reads this instead
    // of a live Hal::Motor::capabilities() call.
    for (uint32_t port = 1; port <= Rt::kPortCount; ++port) {
      bb.motorCaps[port - 1] = hal.motor(port).capabilities();
    }

    router.setReplyChannels(captureReply, nullptr, captureReply, nullptr);
  }

  // dispatch -- builds a command from `line`, routes it, and captures its
  // reply into `cap`.
  void dispatch(const char* line, ReplyCapture& cap) {
    router.setReplyChannels(captureReply, &cap, captureReply, &cap);
    Subsystems::CommunicatorToCommandProcessorCommand cmd;
    cmd.returnPath = Subsystems::Channel::SERIAL;
    std::snprintf(cmd.line, sizeof(cmd.line), "%s", line);
    router.route(cmd, bb);
  }
};

// --- Scenarios ------------------------------------------------------------

// 1. Capability pre-validation (Hal::motorCommandAllowed() against
// bb.motorCaps[]) rejects an unsupported mode BEFORE posting -- VOLT on
// Nezha (capabilities().voltage == false). No queue is touched; the wire
// ERR is unchanged.
void scenarioCapabilityRejectionBlocksStaging() {
  beginScenario("capability pre-validation rejects VOLT before staging (no voltage mode on Nezha)");
  Fixture f;
  ReplyCapture cap;
  f.dispatch("DEV M 1 VOLT 5", cap);

  checkTrue(f.bb.motorIn[0].empty(), "a rejected command must not post to bb.motorIn[]");
  checkTrue(f.bb.driveIn.empty(), "a rejected command must not touch bb.driveIn");
  checkTrue(!cap.lines.empty() && cap.lines.back() == "ERR unsupported volt",
            "wire reply is unchanged: ERR unsupported volt");
}

// 2. A bound-port motion verb (DEV M <n>, n in the Drivetrain's bound pair)
// posts BOTH an addressed msg::MotorCommand to bb.motorIn[n-1] AND a
// standby-only ({NONE, standby}) Drivetrain authority-steal command to
// bb.driveIn.
void scenarioBoundPortStagesHalAndDrivetrainSteal() {
  beginScenario("DEV M <bound port> VEL posts to bb.motorIn[] + a standby-only bb.driveIn steal");
  Fixture f;   // default binding: left=1, right=2
  ReplyCapture cap;
  f.dispatch("DEV M 1 VEL 120", cap);

  checkFalse(f.bb.motorIn[0].empty(), "accepted VEL posts to bb.motorIn[0]");
  msg::MotorCommand motorCmd = f.bb.motorIn[0].take();
  checkMotorKindEq(motorCmd.control_kind, msg::MotorCommand::ControlKind::VELOCITY,
                   "posted command is VELOCITY");
  checkFloatEq(motorCmd.control.velocity, 120.0f, "posted velocity target");

  checkFalse(f.bb.driveIn.empty(), "a bound-port motion verb also steals drivetrain authority");
  msg::DrivetrainCommand dtCmd = f.bb.driveIn.take();
  checkDtKindEq(dtCmd.control_kind, msg::DrivetrainCommand::ControlKind::NONE,
               "authority steal leaves mode_ untouched (control_kind stays NONE)");
  checkTrue(dtCmd.standby.has && dtCmd.standby.val, "authority steal sets standby=true");
}

// 3. An unbound-port motion verb posts to that port's bb.motorIn[] but
// leaves bb.driveIn completely untouched.
void scenarioUnboundPortLeavesDrivetrainUntouched() {
  beginScenario("DEV M <unbound port> posts to bb.motorIn[] but never touches bb.driveIn");
  Fixture f;   // bound pair defaults to 1,2
  ReplyCapture cap;
  f.dispatch("DEV M 3 DUTY 40", cap);

  checkFalse(f.bb.motorIn[2].empty(), "accepted DUTY on an unbound port still posts to bb.motorIn[2]");
  checkTrue(f.bb.driveIn.empty(), "an unbound port's motion verb must not touch bb.driveIn");
}

// 4. DEV STOP posts a broadcast neutral to bb.hardwareBroadcastIn (NOT
// bb.motorIn[] -- a broadcast must not mark any port in-use) plus a
// Drivetrain {NEUTRAL, standby=true} to bb.driveIn -- the global stop shape.
void scenarioDevStopBroadcastShape() {
  beginScenario("DEV STOP posts a broadcast neutral to bb.hardwareBroadcastIn + a Drivetrain NEUTRAL/standby");
  Fixture f;
  ReplyCapture cap;
  f.dispatch("DEV STOP", cap);

  checkFalse(f.bb.hardwareBroadcastIn.empty(), "DEV STOP posts to bb.hardwareBroadcastIn");
  msg::MotorCommand broadcastCmd = f.bb.hardwareBroadcastIn.take();
  checkMotorKindEq(broadcastCmd.control_kind, msg::MotorCommand::ControlKind::NEUTRAL,
                   "broadcast command is NEUTRAL");
  checkTrue(broadcastCmd.control.neutral == msg::Neutral::BRAKE, "broadcast neutral mode is BRAKE");
  checkTrue(f.bb.motorIn[0].empty() && f.bb.motorIn[1].empty() && f.bb.motorIn[2].empty() &&
                f.bb.motorIn[3].empty(),
            "DEV STOP's broadcast never posts to any bb.motorIn[] slot");

  checkFalse(f.bb.driveIn.empty(), "DEV STOP posts a Drivetrain command");
  msg::DrivetrainCommand dtCmd = f.bb.driveIn.take();
  checkDtKindEq(dtCmd.control_kind, msg::DrivetrainCommand::ControlKind::NEUTRAL,
               "Drivetrain command sets mode NEUTRAL");
  checkTrue(dtCmd.standby.has && dtCmd.standby.val, "Drivetrain command also drops authority (standby=true)");

  checkTrue(!cap.lines.empty() && cap.lines.back() == "OK DEV STOP", "wire reply text is unchanged");
}

// 5. DEV DT STOP posts an ADDRESSED neutral to exactly the bound pair's OWN
// bb.motorIn[] slots -- NEVER bb.hardwareBroadcastIn -- so an independent,
// unbound motor's slot is never touched.
void scenarioDevDtStopAddressedPairShape() {
  beginScenario("DEV DT STOP posts an addressed neutral to the bound pair's bb.motorIn[] slots only");
  Fixture f;   // bound pair 1,2 by default
  ReplyCapture cap;
  f.dispatch("DEV DT STOP", cap);

  checkFalse(f.bb.motorIn[0].empty(), "DEV DT STOP posts to bb.motorIn[0] (bound left)");
  checkFalse(f.bb.motorIn[1].empty(), "DEV DT STOP posts to bb.motorIn[1] (bound right)");
  checkTrue(f.bb.motorIn[2].empty(),
            "an independent, unbound motor (port 3) is never posted to");
  checkTrue(f.bb.hardwareBroadcastIn.empty(), "DEV DT STOP is addressed, never a broadcast");

  msg::MotorCommand leftCmd = f.bb.motorIn[0].take();
  checkMotorKindEq(leftCmd.control_kind, msg::MotorCommand::ControlKind::NEUTRAL,
                   "addressed neutral command is NEUTRAL");

  checkFalse(f.bb.driveIn.empty(), "DEV DT STOP also posts a Drivetrain command");
  msg::DrivetrainCommand dtCmd = f.bb.driveIn.take();
  checkDtKindEq(dtCmd.control_kind, msg::DrivetrainCommand::ControlKind::NEUTRAL,
               "Drivetrain command sets mode NEUTRAL");
  checkTrue(dtCmd.standby.has && dtCmd.standby.val,
            "Drivetrain command drops authority (standby=true), scoped to the bound pair only");
}

// 6. Queries (STATE/CAPS) produce a reply and NEVER touch any queue -- the
// "principled asymmetry": observations are pull-only reads against bb's
// committed state cells, answered at parse time, never posted.
void scenarioQueryProducesReplyNoOutboxTraffic() {
  beginScenario("STATE/CAPS queries produce a reply and never touch any queue");
  Fixture f;
  ReplyCapture cap;
  f.dispatch("DEV M 1 STATE", cap);
  f.dispatch("DEV M 1 CAPS", cap);
  f.dispatch("DEV DT STATE", cap);
  f.dispatch("DEV STATE", cap);

  for (uint32_t port = 1; port <= Rt::kPortCount; ++port) {
    checkTrue(f.bb.motorIn[port - 1].empty(), "queries never post to bb.motorIn[]");
  }
  checkTrue(f.bb.driveIn.empty(), "queries never post to bb.driveIn");
  checkTrue(f.bb.hardwareBroadcastIn.empty(), "queries never post to bb.hardwareBroadcastIn");
  checkTrue(f.bb.configIn.empty(), "queries never post to bb.configIn");
  checkTrue(cap.lines.size() >= 4, "every query produced at least one reply line");
  for (const std::string& line : cap.lines) {
    checkTrue(line.compare(0, 3, "OK ") == 0, "every query reply is an OK line: " + line);
  }
}

// 7. Config-plane commands (CFG, PORTS) post ONE Rt::ConfigDelta to
// bb.configIn each (087-006: replaces the pre-087 "takes effect directly"
// shadow-write -- the Configurator, not this harness, folds+applies it) and
// never touch bb.motorIn[]/bb.driveIn/bb.hardwareBroadcastIn.
void scenarioConfigCommandPostsOneDeltaNoOutbox() {
  beginScenario("CFG/PORTS commands post ONE Rt::ConfigDelta each (config-plane) and never touch any other queue");
  Fixture f;
  ReplyCapture cap;

  f.dispatch("DEV M 2 CFG kp=0.5", cap);
  checkUintEq(f.bb.configIn.size(), 1, "DEV M CFG posts exactly one Rt::ConfigDelta");
  Rt::ConfigDelta motorDelta = f.bb.configIn.take();
  checkTrue(motorDelta.target == Rt::ConfigDelta::kMotor, "DEV M CFG's delta targets kMotor");
  checkUintEq(motorDelta.port, 2, "DEV M CFG's delta addresses port 2");
  checkTrue((motorDelta.mask & Rt::bitOf(Rt::MotorConfigField::kVelGainsKp)) != 0,
            "DEV M CFG's delta mask carries the kp bit");
  checkFloatEq(motorDelta.motor.vel_gains.kp, 0.5f, "DEV M CFG's delta carries the parsed kp value");
  checkTrue(f.bb.motorIn[1].empty(), "CFG never posts a HAL command");

  f.dispatch("DEV DT PORTS 3 4", cap);
  checkUintEq(f.bb.configIn.size(), 1, "DEV DT PORTS posts exactly one Rt::ConfigDelta");
  Rt::ConfigDelta dtDelta = f.bb.configIn.take();
  checkTrue(dtDelta.target == Rt::ConfigDelta::kDrivetrain, "DEV DT PORTS's delta targets kDrivetrain");
  checkTrue((dtDelta.mask & Rt::bitOf(Rt::DrivetrainConfigField::kLeftPort)) != 0 &&
                (dtDelta.mask & Rt::bitOf(Rt::DrivetrainConfigField::kRightPort)) != 0,
            "DEV DT PORTS's delta mask carries both port bits");
  checkUintEq(dtDelta.drivetrain.left_port, 3, "DEV DT PORTS's delta carries the parsed left port");
  checkUintEq(dtDelta.drivetrain.right_port, 4, "DEV DT PORTS's delta carries the parsed right port");
  checkTrue(f.bb.driveIn.empty(), "DEV DT PORTS never posts a Drivetrain command");
  checkTrue(f.bb.hardwareBroadcastIn.empty(), "DEV DT PORTS never posts a HAL command either");

  checkTrue(!cap.lines.empty() && cap.lines.back() == "OK DEV DT ports=3,4",
            "DEV DT PORTS wire reply is unchanged (echoes the requested values)");
}

// 8. Latest-wins: two undrained DEV M commands on the same port leave only
// the SECOND command's value in bb.motorIn[] (Mailbox semantics).
void scenarioLatestWinsOverwrite() {
  beginScenario("two undrained DEV M commands: latest-wins, only the second is staged");
  Fixture f;
  ReplyCapture cap;
  f.dispatch("DEV M 1 DUTY 20", cap);
  f.dispatch("DEV M 1 DUTY 60", cap);

  checkFalse(f.bb.motorIn[0].empty(), "still exactly one held command after two undrained posts");
  msg::MotorCommand motorCmd = f.bb.motorIn[0].take();
  checkFloatEq(motorCmd.control.duty_cycle, 0.60f,
               "latest-wins: bb.motorIn[0] holds only the SECOND command's value");
}

// 9. NEUTRAL/RESET (never capability-gated) still run through the same
// pre-validate-then-post path as every other motion verb, not a hand-rolled
// special case.
void scenarioNeutralAndResetAlsoStageThroughTheOutbox() {
  beginScenario("NEUTRAL/RESET post through bb.motorIn[] like every other motion verb");
  Fixture f;
  ReplyCapture cap;

  f.dispatch("DEV M 1 NEUTRAL B", cap);
  checkFalse(f.bb.motorIn[0].empty(), "NEUTRAL posts to bb.motorIn[0]");
  msg::MotorCommand neutralCmd = f.bb.motorIn[0].take();
  checkMotorKindEq(neutralCmd.control_kind, msg::MotorCommand::ControlKind::NEUTRAL,
                   "posted command is NEUTRAL");

  f.dispatch("DEV M 1 RESET", cap);
  checkFalse(f.bb.motorIn[0].empty(), "RESET posts to bb.motorIn[0]");
  msg::MotorCommand resetCmd = f.bb.motorIn[0].take();
  checkTrue(resetCmd.reset_position.has && resetCmd.reset_position.val,
            "posted command carries reset_position=true");
}

// 10. buildBroadcastNeutral()/buildDrivetrainStop() -- the exact shapes the
// loop's watchdog-fire path applies IMMEDIATELY (not via any bb queue).
// DEV STOP's own shape (scenario 4) already proves these via the posted
// path; this scenario proves the free functions themselves, which is what
// the watchdog-fire path calls directly.
void scenarioBuildBroadcastNeutralAndDrivetrainStopShapes() {
  beginScenario("buildBroadcastNeutral()/buildDrivetrainStop() -- the shapes the watchdog-fire path applies immediately");
  Hal::CommandProcessorToHardwareCommand hardwareCmd = buildBroadcastNeutral(msg::Neutral::BRAKE);
  checkTrue(hardwareCmd.allPorts, "buildBroadcastNeutral() is a broadcast");
  checkMotorKindEq(hardwareCmd.addressed[0].command.control_kind,
                   msg::MotorCommand::ControlKind::NEUTRAL, "buildBroadcastNeutral()'s command is NEUTRAL");
  checkTrue(hardwareCmd.addressed[0].command.control.neutral == msg::Neutral::BRAKE,
            "buildBroadcastNeutral() honors the requested mode");

  msg::DrivetrainCommand dtCmd = buildDrivetrainStop(msg::Neutral::BRAKE);
  checkDtKindEq(dtCmd.control_kind, msg::DrivetrainCommand::ControlKind::NEUTRAL,
               "buildDrivetrainStop() sets mode NEUTRAL");
  checkTrue(dtCmd.standby.has && dtCmd.standby.val,
            "buildDrivetrainStop() also drops authority (standby=true)");
}

}  // namespace

int main() {
  initConfigsTemplate();

  scenarioCapabilityRejectionBlocksStaging();
  scenarioBoundPortStagesHalAndDrivetrainSteal();
  scenarioUnboundPortLeavesDrivetrainUntouched();
  scenarioDevStopBroadcastShape();
  scenarioDevDtStopAddressedPairShape();
  scenarioQueryProducesReplyNoOutboxTraffic();
  scenarioConfigCommandPostsOneDeltaNoOutbox();
  scenarioLatestWinsOverwrite();
  scenarioNeutralAndResetAlsoStageThroughTheOutbox();
  scenarioBuildBroadcastNeutralAndDrivetrainStopShapes();

  if (g_failureCount == 0) {
    std::printf("OK: all DEV command outbox scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the DEV command outbox scenarios\n", g_failureCount);
  return 1;
}
