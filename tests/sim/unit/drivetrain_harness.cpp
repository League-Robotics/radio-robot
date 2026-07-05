// drivetrain_harness.cpp — off-hardware acceptance harness for ticket
// 079-003 (SUC-004/SUC-005/SUC-006): exercises Subsystems::Drivetrain's
// held-output reshape (tick() void + hasCommand()/takeCommand()) and its new
// port-binding/authority-arbitration surface (ports()/active()/standby(),
// the DrivetrainCommand.standby side-channel) directly against plain
// msg::* structs — no fakes needed, per the ticket's Testing plan
// ("Drivetrain has no hardware dependency").
//
// Mirrors motor_policy_harness.cpp's shape exactly (see that file's header
// for the pattern): #includes only subsystems/drivetrain.h and messages/*.h
// (both dependency-free — no MicroBit.h, no I2CBus), links against
// subsystems/drivetrain.cpp and kinematics/body_kinematics.cpp (Drivetrain's
// one real dependency, itself dependency-free), compiles with the plain
// system C++ compiler — no CMake, no ARM toolchain. Hand-rolled assertions,
// prints PASS/FAIL, exits nonzero on any failure. Run by
// test_drivetrain.py, which compiles and runs this binary via subprocess.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "hal/capability/hal_command.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "subsystems/drivetrain.h"

namespace {

// --- Hand-rolled assertion plumbing (same tiny shape as
// motor_policy_harness.cpp -- a handful of scenarios do not warrant a test
// framework dependency for a dependency-free host harness). ---

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

void checkFloatEq(float actual, float expected, const std::string& what) {
  if (std::fabs(actual - expected) > 1e-4f) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual));
    fail(buf);
  }
}

void checkUintEq(uint32_t actual, uint32_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %u, got %u", what.c_str(),
                  static_cast<unsigned>(expected), static_cast<unsigned>(actual));
    fail(buf);
  }
}

void checkKindEq(msg::MotorCommand::ControlKind actual,
                  msg::MotorCommand::ControlKind expected, const std::string& what) {
  if (actual != expected) {
    fail(what + " -- MotorCommand::ControlKind mismatch");
  }
}

// --- Fixture helpers ---

msg::DrivetrainConfig configWithPorts(uint32_t left, uint32_t right,
                                       float trackwidth = 100.0f, float syncGain = 0.0f) {
  msg::DrivetrainConfig cfg;
  cfg.setLeftPort(left);
  cfg.setRightPort(right);
  cfg.setTrackwidth(trackwidth);
  cfg.setSyncGain(syncGain);
  return cfg;
}

msg::MotorState obsVelocity(float velocity) {
  msg::MotorState s;
  s.velocity.has = true;
  s.velocity.val = velocity;
  return s;
}

msg::DrivetrainCommand wheelsCommand(float left, float right) {
  msg::WheelTargets wt;
  wt.w_count = 2;
  wt.w_[0].speed.has = true;
  wt.w_[0].speed.val = left;
  wt.w_[1].speed.has = true;
  wt.w_[1].speed.val = right;
  msg::DrivetrainCommand cmd;
  cmd.setWheels(wt);
  return cmd;
}

// --- Scenarios ---

// 1. ports() reflects whatever left_port/right_port configure() was given --
// the binding moved into DrivetrainConfig per sprint 079 decision 8.
void scenarioPortsReflectConfig() {
  beginScenario("ports() reflects the configured left_port/right_port binding");
  Subsystems::Drivetrain dt;
  dt.configure(configWithPorts(3, 4));

  Subsystems::DrivetrainPorts p = dt.ports();
  checkUintEq(p.left, 3, "ports().left");
  checkUintEq(p.right, 4, "ports().right");
}

// 2. Each of the three command-arm setters (via apply()'s oneof dispatch)
// (re)activates drivetrain authority, matching docs/protocol-v2.md's
// existing rule.
void scenarioCommandArmsActivateAuthority() {
  beginScenario("TWIST/WHEELS/NEUTRAL each activate authority");
  Subsystems::Drivetrain dt;
  dt.configure(configWithPorts(1, 2));
  checkFalse(dt.active(), "freshly constructed Drivetrain starts inactive");

  msg::DrivetrainCommand twist;
  msg::BodyTwist3 t; t.v_x = 50.0f; t.v_y = 0.0f; t.omega = 0.0f;
  twist.setTwist(t);
  dt.apply(twist);
  checkTrue(dt.active(), "TWIST activates authority");

  dt.standby();
  checkFalse(dt.active(), "standby() drops authority (setup for next check)");

  dt.apply(wheelsCommand(10.0f, 20.0f));
  checkTrue(dt.active(), "WHEELS activates authority");

  dt.standby();
  msg::DrivetrainCommand neutral;
  neutral.setNeutral(msg::Neutral::BRAKE);
  dt.apply(neutral);
  checkTrue(dt.active(), "NEUTRAL activates authority");
}

// 3. Acceptance criterion: {control_kind=NEUTRAL(mode), standby=true} ->
// mode_==NEUTRAL && active_==false after apply() -- setNeutral() sets
// mode_=NEUTRAL, active_=true, then the standby side-channel immediately
// drops active_ back to false. mode_==NEUTRAL is observed indirectly via
// state().vel == {0, 0} (state()'s NEUTRAL-mode zero report).
void scenarioNeutralWithStandbyDropsAuthorityKeepsMode() {
  beginScenario("{NEUTRAL, standby=true}: mode_==NEUTRAL, active_==false");
  Subsystems::Drivetrain dt;
  dt.configure(configWithPorts(1, 2));

  // Start from an active WHEELS command so activation is a real transition,
  // not a no-op against the default-false state.
  dt.apply(wheelsCommand(80.0f, 80.0f));
  checkTrue(dt.active(), "precondition: WHEELS left the Drivetrain active");

  msg::DrivetrainCommand cmd;
  cmd.setNeutral(msg::Neutral::BRAKE);
  cmd.setStandby(true);
  dt.apply(cmd);

  checkFalse(dt.active(), "standby=true dropped authority");
  msg::DrivetrainState s = dt.state();
  checkFloatEq(s.vel_[0], 0.0f, "mode_==NEUTRAL: state().vel[0] reports 0");
  checkFloatEq(s.vel_[1], 0.0f, "mode_==NEUTRAL: state().vel[1] reports 0");
}

// 4. Acceptance criterion: {control_kind=NONE, standby=true} -> active_==false
// with mode_/targets UNCHANGED (the authority-steal case) -- the oneof
// switch's NONE/default case takes no action, so the LAST commanded WHEELS
// target must still be what state() reports after the steal, matching
// today's exact quirk (a stolen Drivetrain's STATE still reports its
// pre-steal target until the next real command).
void scenarioNoneWithStandbyStealsAuthorityOnly() {
  beginScenario("{NONE, standby=true}: active_==false, mode_/targets untouched");
  Subsystems::Drivetrain dt;
  dt.configure(configWithPorts(1, 2));

  dt.apply(wheelsCommand(100.0f, 50.0f));
  checkTrue(dt.active(), "precondition: WHEELS left the Drivetrain active");
  msg::DrivetrainState before = dt.state();
  checkFloatEq(before.vel_[0], 100.0f, "precondition: state().vel[0] == 100");
  checkFloatEq(before.vel_[1], 50.0f, "precondition: state().vel[1] == 50");

  msg::DrivetrainCommand steal;   // control_kind defaults to NONE
  steal.setStandby(true);
  dt.apply(steal);

  checkFalse(dt.active(), "standby=true (alone) dropped authority");
  msg::DrivetrainState after = dt.state();
  checkFloatEq(after.vel_[0], 100.0f,
               "authority steal did NOT reset the last commanded target (left)");
  checkFloatEq(after.vel_[1], 50.0f,
               "authority steal did NOT reset the last commanded target (right)");
}

// 5. hasCommand()/takeCommand(): false before any tick(), true after tick()
// runs (unconditionally, per tick()'s doc comment), false again immediately
// after takeCommand() drains it.
void scenarioHasCommandTakeCommandClears() {
  beginScenario("hasCommand()/takeCommand(): set by tick(), cleared by takeCommand()");
  Subsystems::Drivetrain dt;
  dt.configure(configWithPorts(1, 2));
  checkFalse(dt.hasCommand(), "no command held before the first tick()");

  dt.apply(wheelsCommand(10.0f, 10.0f));
  dt.tick(1000, obsVelocity(0.0f), obsVelocity(0.0f));
  checkTrue(dt.hasCommand(), "tick() unconditionally holds a command");

  Hal::DrivetrainToHalCommand held = dt.takeCommand();
  (void)held;
  checkFalse(dt.hasCommand(), "takeCommand() clears hasCommand()");
}

// 6. The held Hal::DrivetrainToHalCommand's wheel[].port matches the
// configured binding, and (with sync_gain==0, the governor a no-op) the
// commanded velocities pass through exactly.
void scenarioHeldCommandPortsMatchBindingAndCarryTargets() {
  beginScenario("held command: wheel[].port == configured binding, targets pass through");
  Subsystems::Drivetrain dt;
  dt.configure(configWithPorts(3, 4, /*trackwidth=*/100.0f, /*syncGain=*/0.0f));

  dt.apply(wheelsCommand(42.0f, -17.0f));
  dt.tick(2000, obsVelocity(0.0f), obsVelocity(0.0f));

  checkTrue(dt.hasCommand(), "tick() held a command");
  Hal::DrivetrainToHalCommand cmd = dt.takeCommand();

  checkUintEq(cmd.wheel[0].port, 3, "held command wheel[0].port == left_port");
  checkUintEq(cmd.wheel[1].port, 4, "held command wheel[1].port == right_port");
  checkKindEq(cmd.wheel[0].command.get_control_kind(),
              msg::MotorCommand::ControlKind::VELOCITY, "wheel[0] is a VELOCITY command");
  checkKindEq(cmd.wheel[1].command.get_control_kind(),
              msg::MotorCommand::ControlKind::VELOCITY, "wheel[1] is a VELOCITY command");
  checkFloatEq(cmd.wheel[0].command.control.velocity, 42.0f, "wheel[0] velocity target");
  checkFloatEq(cmd.wheel[1].command.control.velocity, -17.0f, "wheel[1] velocity target");
}

// 7. NEUTRAL mode's held command carries a NEUTRAL MotorCommand for both
// wheels (matching neutralMode_), not a stale VELOCITY target.
void scenarioHeldCommandReflectsNeutralMode() {
  beginScenario("held command reflects NEUTRAL mode as a NEUTRAL MotorCommand pair");
  Subsystems::Drivetrain dt;
  dt.configure(configWithPorts(1, 2));

  msg::DrivetrainCommand cmd;
  cmd.setNeutral(msg::Neutral::COAST);
  dt.apply(cmd);
  dt.tick(3000, obsVelocity(0.0f), obsVelocity(0.0f));

  Hal::DrivetrainToHalCommand held = dt.takeCommand();
  checkKindEq(held.wheel[0].command.get_control_kind(),
              msg::MotorCommand::ControlKind::NEUTRAL, "wheel[0] is a NEUTRAL command");
  checkKindEq(held.wheel[1].command.get_control_kind(),
              msg::MotorCommand::ControlKind::NEUTRAL, "wheel[1] is a NEUTRAL command");
  checkTrue(held.wheel[0].command.control.neutral == msg::Neutral::COAST,
            "wheel[0] neutral mode matches setNeutral(COAST)");
  checkTrue(held.wheel[1].command.control.neutral == msg::Neutral::COAST,
            "wheel[1] neutral mode matches setNeutral(COAST)");
}

// 8. Ratio-governor regression guard (TWIST arm): the governor's math is
// UNCHANGED by this ticket's reshape (only the output plumbing changed) --
// pin the exact pre-ticket numeric result for a representative bogged-down
// case. trackwidth=100 -> inverse(v_x=100, omega=0, b=100) -> targetLeft ==
// targetRight == 100. leftObs=80 (achievedLeft=0.8), rightObs=100
// (achievedRight=1.0) -> achievedMin=0.8. sync_gain=0.5 ->
// scale = 1 - 0.5*(1-0.8) = 0.9 -> both targets *= 0.9 -> 90.0.
void scenarioRatioGovernorTwistRegression() {
  beginScenario("ratio governor TWIST regression: unchanged numeric output");
  Subsystems::Drivetrain dt;
  dt.configure(configWithPorts(1, 2, /*trackwidth=*/100.0f, /*syncGain=*/0.5f));

  msg::DrivetrainCommand cmd;
  msg::BodyTwist3 t; t.v_x = 100.0f; t.v_y = 0.0f; t.omega = 0.0f;
  cmd.setTwist(t);
  dt.apply(cmd);
  dt.tick(4000, obsVelocity(80.0f), obsVelocity(100.0f));

  Hal::DrivetrainToHalCommand held = dt.takeCommand();
  checkFloatEq(held.wheel[0].command.control.velocity, 90.0f, "governed left target");
  checkFloatEq(held.wheel[1].command.control.velocity, 90.0f, "governed right target");
}

// 9. Ratio-governor regression guard (WHEELS arm): left=50 (obs 50,
// achieved=1.0), right=100 (obs 50, achieved=0.5) -> achievedMin=0.5.
// sync_gain=0.5 -> scale = 1 - 0.5*(1-0.5) = 0.75 -> left=37.5, right=75.0.
void scenarioRatioGovernorWheelsRegression() {
  beginScenario("ratio governor WHEELS regression: unchanged numeric output");
  Subsystems::Drivetrain dt;
  dt.configure(configWithPorts(1, 2, /*trackwidth=*/100.0f, /*syncGain=*/0.5f));

  dt.apply(wheelsCommand(50.0f, 100.0f));
  dt.tick(5000, obsVelocity(50.0f), obsVelocity(50.0f));

  Hal::DrivetrainToHalCommand held = dt.takeCommand();
  checkFloatEq(held.wheel[0].command.control.velocity, 37.5f, "governed left target");
  checkFloatEq(held.wheel[1].command.control.velocity, 75.0f, "governed right target");
}

}  // namespace

int main() {
  scenarioPortsReflectConfig();
  scenarioCommandArmsActivateAuthority();
  scenarioNeutralWithStandbyDropsAuthorityKeepsMode();
  scenarioNoneWithStandbyStealsAuthorityOnly();
  scenarioHasCommandTakeCommandClears();
  scenarioHeldCommandPortsMatchBindingAndCarryTargets();
  scenarioHeldCommandReflectsNeutralMode();
  scenarioRatioGovernorTwistRegression();
  scenarioRatioGovernorWheelsRegression();

  if (g_failureCount == 0) {
    std::printf("OK: all Drivetrain reshape scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the Drivetrain reshape scenarios\n",
              g_failureCount);
  return 1;
}
