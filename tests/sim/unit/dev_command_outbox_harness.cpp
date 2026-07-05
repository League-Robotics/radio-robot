// dev_command_outbox_harness.cpp -- off-hardware acceptance harness for
// ticket 079-005 (SUC-004/SUC-005/SUC-006/SUC-007): exercises the REAL
// CommandProcessor + DevLoopState + Subsystems::Drivetrain + Hal::NezhaHal
// pure-transformer reshape -- statements in, staged commands (+ replies)
// out, no handler ever calling Hal::Motor::apply()/Subsystems::Drivetrain::
// apply() directly.
//
// Per the design sketch's "subsystem is the unit of test" principle, this
// drives the REAL parse path: full ASCII statement lines through
// CommandProcessor::process() (built from the REAL devCommands() table),
// asserting on DevLoopState's outbox fields (hasHalCommand/halCommand,
// hasDrivetrainCommand/drivetrainCommand) and the captured reply text
// afterward -- exactly the "statements in, commands + replies out" contract
// architecture-update.md describes. Mirrors nezha_flipflop_harness.cpp's
// shape (hand-rolled assertions, PASS/FAIL per scenario) but additionally
// links dev_commands.cpp/command_processor.cpp/arg_parse.cpp/drivetrain.cpp/
// body_kinematics.cpp against the SAME source/hal/nezha/*.cpp + ticket 001's
// HOST_BUILD scripted I2CBus fake nezha_flipflop_harness.cpp already proved
// out -- no new hardware-fake surface, just a fresh consumer of it.
//
// None of these scenarios need to script the I2CBus at all: capability
// pre-validation, port binding, and outbox construction never touch the
// bus (that only happens inside NezhaHal::tick()'s flip-flop, which no
// scenario here calls) -- a freshly constructed NezhaHal is enough. This
// mirrors the ticket's own testing-plan note: "Drivetrain/processor need no
// fakes at all" -- the REAL NezhaHal, unticked, behaves exactly like a
// minimal test stand-in for everything this ticket's acceptance criteria
// care about.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "com/i2c_bus.h"
#include "commands/command_processor.h"
#include "commands/dev_commands.h"
#include "hal/nezha/nezha_hal.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "subsystems/drivetrain.h"

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

msg::MotorConfig g_configsTemplate[Hal::NezhaHal::kPortCount];

void initConfigsTemplate() {
  for (uint32_t i = 0; i < Hal::NezhaHal::kPortCount; ++i) {
    g_configsTemplate[i] = msg::MotorConfig{};
    g_configsTemplate[i].setPort(i + 1).setFwdSign(1).setTravelCalib(1.0f);
  }
}

msg::DrivetrainConfig defaultDrivetrainConfig() {
  msg::DrivetrainConfig cfg;
  cfg.setLeftPort(1);
  cfg.setRightPort(2);
  cfg.setTrackwidth(128.0f);
  return cfg;
}

// Captures every reply line CommandProcessor::process() emits for one
// statement (usually one line, occasionally more -- e.g. a CFG badkey ERR
// followed by the ack).
struct ReplyCapture {
  std::vector<std::string> lines;
};

void captureReply(const char* msg, void* ctx) {
  static_cast<ReplyCapture*>(ctx)->lines.push_back(msg);
}

struct Fixture {
  I2CBus bus;
  Hal::NezhaHal hal;
  Subsystems::Drivetrain drivetrain;
  SerialSilenceWatchdog watchdog;
  DevLoopState state;
  CommandProcessor cmd;

  Fixture() : hal(bus, g_configsTemplate) {
    drivetrain.configure(defaultDrivetrainConfig());

    state.hal = &hal;
    state.drivetrain = &drivetrain;
    state.watchdog = &watchdog;
    for (uint32_t i = 0; i < Hal::NezhaHal::kPortCount; ++i) {
      state.motorConfigShadow[i] = g_configsTemplate[i];
    }
    state.drivetrainConfigShadow = defaultDrivetrainConfig();
    drivetrain.setMotorCapabilities(hal.motor(1).capabilities(), hal.motor(2).capabilities());

    cmd = CommandProcessor(devCommands(state));
  }
};

// --- Scenarios ------------------------------------------------------------

// 1. Capability pre-validation (Hal::motorCommandAllowed()) rejects an
// unsupported mode BEFORE staging -- VOLT on Nezha (capabilities().voltage
// == false). Neither outbox is touched; the wire ERR is unchanged.
void scenarioCapabilityRejectionBlocksStaging() {
  beginScenario("capability pre-validation rejects VOLT before staging (no voltage mode on Nezha)");
  Fixture f;
  ReplyCapture cap;
  f.cmd.process("DEV M 1 VOLT 5", captureReply, &cap);

  checkFalse(f.state.hasHalCommand, "a rejected command must not stage a HAL command");
  checkFalse(f.state.hasDrivetrainCommand, "a rejected command must not touch the Drivetrain outbox");
  checkTrue(!cap.lines.empty() && cap.lines.back() == "ERR unsupported volt",
            "wire reply is unchanged: ERR unsupported volt");
}

// 2. A bound-port motion verb (DEV M <n>, n in the Drivetrain's ports())
// stages BOTH an addressed HAL command AND a standby-only ({NONE, standby})
// Drivetrain authority-steal command.
void scenarioBoundPortStagesHalAndDrivetrainSteal() {
  beginScenario("DEV M <bound port> VEL stages an addressed HAL command + a standby-only Drivetrain steal");
  Fixture f;   // default binding: left=1, right=2
  ReplyCapture cap;
  f.cmd.process("DEV M 1 VEL 120", captureReply, &cap);

  checkTrue(f.state.hasHalCommand, "accepted VEL stages a HAL command");
  checkFalse(f.state.halCommand.allPorts, "single-motor stage is addressed, not a broadcast");
  checkUintEq(f.state.halCommand.count, 1, "single-motor stage addresses exactly one port");
  checkUintEq(f.state.halCommand.addressed[0].port, 1, "addressed port matches the commanded port");
  checkMotorKindEq(f.state.halCommand.addressed[0].command.control_kind,
                   msg::MotorCommand::ControlKind::VELOCITY, "staged command is VELOCITY");
  checkFloatEq(f.state.halCommand.addressed[0].command.control.velocity, 120.0f,
               "staged velocity target");

  checkTrue(f.state.hasDrivetrainCommand, "a bound-port motion verb also steals drivetrain authority");
  checkDtKindEq(f.state.drivetrainCommand.control_kind,
               msg::DrivetrainCommand::ControlKind::NONE,
               "authority steal leaves mode_ untouched (control_kind stays NONE)");
  checkTrue(f.state.drivetrainCommand.standby.has && f.state.drivetrainCommand.standby.val,
            "authority steal sets standby=true");
}

// 3. An unbound-port motion verb (077-007's fix, restaged for outboxes)
// stages the HAL command but leaves the Drivetrain outbox completely
// untouched.
void scenarioUnboundPortLeavesDrivetrainUntouched() {
  beginScenario("DEV M <unbound port> stages a HAL command but never touches the Drivetrain outbox");
  Fixture f;   // bound pair defaults to 1,2
  ReplyCapture cap;
  f.cmd.process("DEV M 3 DUTY 40", captureReply, &cap);

  checkTrue(f.state.hasHalCommand, "accepted DUTY on an unbound port still stages a HAL command");
  checkUintEq(f.state.halCommand.addressed[0].port, 3, "addressed to the commanded (unbound) port");
  checkFalse(f.state.hasDrivetrainCommand,
             "an unbound port's motion verb must not stage a Drivetrain command");
}

// 4. DEV STOP stages a BROADCAST HAL neutral (allPorts=true) plus a
// Drivetrain {NEUTRAL, standby=true} -- the global stop shape.
void scenarioDevStopBroadcastShape() {
  beginScenario("DEV STOP stages a broadcast HAL neutral + a Drivetrain NEUTRAL/standby");
  Fixture f;
  ReplyCapture cap;
  f.cmd.process("DEV STOP", captureReply, &cap);

  checkTrue(f.state.hasHalCommand, "DEV STOP stages a HAL command");
  checkTrue(f.state.halCommand.allPorts, "DEV STOP's HAL command is a broadcast");
  checkMotorKindEq(f.state.halCommand.addressed[0].command.control_kind,
                   msg::MotorCommand::ControlKind::NEUTRAL, "broadcast command is NEUTRAL");
  checkTrue(f.state.halCommand.addressed[0].command.control.neutral == msg::Neutral::BRAKE,
            "broadcast neutral mode is BRAKE");

  checkTrue(f.state.hasDrivetrainCommand, "DEV STOP stages a Drivetrain command");
  checkDtKindEq(f.state.drivetrainCommand.control_kind,
               msg::DrivetrainCommand::ControlKind::NEUTRAL, "Drivetrain command sets mode NEUTRAL");
  checkTrue(f.state.drivetrainCommand.standby.has && f.state.drivetrainCommand.standby.val,
            "Drivetrain command also drops authority (standby=true)");

  checkTrue(!cap.lines.empty() && cap.lines.back() == "OK DEV STOP", "wire reply text is unchanged");
}

// 5. DEV DT STOP stages an ADDRESSED (count=2, the bound pair) HAL command
// -- NEVER a broadcast -- so an independent, unbound motor is never placed
// in the addressed array at all (the acceptance criterion's "untouched"
// proof, at the layer dev_commands.cpp is responsible for; ticket 004's own
// harness already proves NezhaHal::apply() only ever reaches addressed
// ports).
void scenarioDevDtStopAddressedPairShape() {
  beginScenario("DEV DT STOP stages an addressed (count=2) bound-pair HAL command, not a broadcast");
  Fixture f;   // bound pair 1,2 by default
  ReplyCapture cap;
  f.cmd.process("DEV DT STOP", captureReply, &cap);

  checkTrue(f.state.hasHalCommand, "DEV DT STOP stages a HAL command");
  checkFalse(f.state.halCommand.allPorts, "DEV DT STOP is addressed, never a broadcast");
  checkUintEq(f.state.halCommand.count, 2, "DEV DT STOP addresses exactly the bound pair");
  checkUintEq(f.state.halCommand.addressed[0].port, 1, "addressed[0] is the bound left port");
  checkUintEq(f.state.halCommand.addressed[1].port, 2, "addressed[1] is the bound right port");
  checkTrue(f.state.halCommand.addressed[0].port != 3 && f.state.halCommand.addressed[1].port != 3,
            "an independent, unbound motor (port 3) is never placed in the addressed array");

  checkTrue(f.state.hasDrivetrainCommand, "DEV DT STOP also stages a Drivetrain command");
  checkDtKindEq(f.state.drivetrainCommand.control_kind,
               msg::DrivetrainCommand::ControlKind::NEUTRAL, "Drivetrain command sets mode NEUTRAL");
  checkTrue(f.state.drivetrainCommand.standby.has && f.state.drivetrainCommand.standby.val,
            "Drivetrain command drops authority (standby=true), scoped to the bound pair only");
}

// 6. Queries (STATE/CAPS) produce a reply and NEVER touch either outbox --
// the "principled asymmetry" (Part 3): observations are pull-only reads,
// answered at parse time, never staged.
void scenarioQueryProducesReplyNoOutboxTraffic() {
  beginScenario("STATE/CAPS queries produce a reply and never touch either outbox");
  Fixture f;
  ReplyCapture cap;
  f.cmd.process("DEV M 1 STATE", captureReply, &cap);
  f.cmd.process("DEV M 1 CAPS", captureReply, &cap);
  f.cmd.process("DEV DT STATE", captureReply, &cap);
  f.cmd.process("DEV STATE", captureReply, &cap);

  checkFalse(f.state.hasHalCommand, "queries never stage a HAL command");
  checkFalse(f.state.hasDrivetrainCommand, "queries never stage a Drivetrain command");
  checkTrue(cap.lines.size() >= 4, "every query produced at least one reply line");
  for (const std::string& line : cap.lines) {
    checkTrue(line.compare(0, 3, "OK ") == 0, "every query reply is an OK line: " + line);
  }
}

// 7. Config-plane statements (CFG, PORTS) take effect directly at parse
// time and never touch either outbox.
void scenarioConfigStatementDirectEffectNoOutbox() {
  beginScenario("CFG/PORTS statements take effect directly (config-plane) and never touch the outbox");
  Fixture f;
  ReplyCapture cap;

  f.cmd.process("DEV M 2 CFG kp=0.5", captureReply, &cap);
  checkFloatEq(f.state.motorConfigShadow[1].vel_gains.kp, 0.5f,
               "DEV M CFG merges directly into the shadow config");
  checkFalse(f.state.hasHalCommand, "CFG never stages a HAL command");

  f.cmd.process("DEV DT PORTS 3 4", captureReply, &cap);
  Subsystems::DrivetrainPorts p = f.state.drivetrain->ports();
  checkUintEq(p.left, 3, "DEV DT PORTS updates the live binding (left)");
  checkUintEq(p.right, 4, "DEV DT PORTS updates the live binding (right)");
  checkFalse(f.state.hasDrivetrainCommand, "DEV DT PORTS never stages a Drivetrain command");
  checkFalse(f.state.hasHalCommand, "DEV DT PORTS never stages a HAL command either");

  checkTrue(!cap.lines.empty() && cap.lines.back() == "OK DEV DT ports=3,4",
            "DEV DT PORTS wire reply is unchanged");
}

// 8. Latest-wins: two undrained DEV M statements on the same port leave
// only the SECOND command's value in the outbox.
void scenarioLatestWinsOverwrite() {
  beginScenario("two undrained DEV M statements: latest-wins, only the second is staged");
  Fixture f;
  ReplyCapture cap;
  f.cmd.process("DEV M 1 DUTY 20", captureReply, &cap);
  f.cmd.process("DEV M 1 DUTY 60", captureReply, &cap);

  checkTrue(f.state.hasHalCommand, "still exactly one held command after two undrained stages");
  checkUintEq(f.state.halCommand.count, 1, "still a single addressed entry");
  checkFloatEq(f.state.halCommand.addressed[0].command.control.duty_cycle, 0.60f,
               "latest-wins: the outbox holds only the SECOND command's value");
}

// 9. NEUTRAL/RESET (never capability-gated) still run through the same
// pre-validate-then-stage path as every other motion verb, not a
// hand-rolled special case.
void scenarioNeutralAndResetAlsoStageThroughTheOutbox() {
  beginScenario("NEUTRAL/RESET stage through the outbox like every other motion verb");
  Fixture f;
  ReplyCapture cap;

  f.cmd.process("DEV M 1 NEUTRAL B", captureReply, &cap);
  checkTrue(f.state.hasHalCommand, "NEUTRAL stages a HAL command");
  checkMotorKindEq(f.state.halCommand.addressed[0].command.control_kind,
                   msg::MotorCommand::ControlKind::NEUTRAL, "staged command is NEUTRAL");

  f.state.hasHalCommand = false;   // simulate main.cpp draining between statements
  f.cmd.process("DEV M 1 RESET", captureReply, &cap);
  checkTrue(f.state.hasHalCommand, "RESET stages a HAL command");
  checkTrue(f.state.halCommand.addressed[0].command.reset_position.has &&
            f.state.halCommand.addressed[0].command.reset_position.val,
            "staged command carries reset_position=true");
}

// 10. buildBroadcastNeutral()/buildDrivetrainStop() -- the exact shapes
// main.cpp's watchdog-fire path applies IMMEDIATELY (not via the outbox).
// DEV STOP's own broadcast shape (scenario 4) already proves these via the
// staged path; this scenario proves the free functions themselves, which is
// what the watchdog-fire path calls directly.
void scenarioBuildBroadcastNeutralAndDrivetrainStopShapes() {
  beginScenario("buildBroadcastNeutral()/buildDrivetrainStop() -- the shapes the watchdog-fire path applies immediately");
  Hal::CommandProcessorToHalCommand halCmd = buildBroadcastNeutral(msg::Neutral::BRAKE);
  checkTrue(halCmd.allPorts, "buildBroadcastNeutral() is a broadcast");
  checkMotorKindEq(halCmd.addressed[0].command.control_kind,
                   msg::MotorCommand::ControlKind::NEUTRAL, "buildBroadcastNeutral()'s command is NEUTRAL");
  checkTrue(halCmd.addressed[0].command.control.neutral == msg::Neutral::BRAKE,
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
  scenarioConfigStatementDirectEffectNoOutbox();
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
