// tlm_frame_harness.cpp — off-hardware acceptance harness for ticket 082-004
// (SUC-004), extended by ticket 087-008 (SUC-001/SUC-002/SUC-006): exercises
// Telemetry::tick() (bb -> TlmFrameInput), Telemetry's own frame-assembly
// step, against a bare, non-live Rt::Blackboard -- no CommandRouter/
// Communicator/Hardware/Drivetrain/PoseEstimator/Planner object of any
// kind, proving Telemetry's isolated testability (SUC-002) directly.
//
// 097-008 (architecture-update-r2.md Decision 9, pure-binary firmware)
// DELETED Telemetry::buildTlmFrame() (the text "TLM t=... mode=..." line
// formatter) along with its only callers (STREAM/SNAP's text handlers,
// commands/telemetry_commands.cpp) -- this harness's own scenarios (a)-(e)
// and the text-output half of (h), which exercised that function directly
// via hand-rolled exact-string / substring-presence checks, are DELETED
// alongside it (see git history for that prior code). What remains:
//   (f) Telemetry::tick() assembling a frame from a bare Rt::Blackboard --
//       now asserted directly against TlmFrameInput's own fields (values,
//       has* flags) instead of via a formatted text line.
//   (i)/(j)/(k) Telemetry::buildTelemetryMessage() (096-003) -- unaffected
//       by this deletion, still the sole remaining formatter.
//
// Per ekf_tiny_harness.cpp / velocity_pid_harness.cpp's precedent (082-001 /
// 081-001), this #includes only telemetry/tlm_frame.h (which itself pulls in
// runtime/blackboard.h -- host-safe, see that header's own file comment)
// plus its own translation unit (tlm_frame.cpp) and
// kinematics/body_kinematics.cpp (Telemetry::tick()'s one pure-math
// dependency, for twist=), so it compiles with the plain system C++
// compiler -- no CMake, no ARM toolchain.
//
// Scenario (f), 087-008's own Testing plan:
//   Telemetry::tick() reads every field directly off a bare, hand-populated
//   Rt::Blackboard (no live subsystem behind any cell) and the resulting
//   TlmFrameInput matches hand-computed expected values -- the
//   isolated-testability proof for Telemetry's own frame assembly.
//
// Folded into scenario (f) by 092-002 (frozen-fused-pose investigation,
// diagnostic telemetry -- clasi/issues/poseestimator-fused-pose-frozen-on-
// hardware.md):
//   (g) otosConnected (Hal::Odometer::connected() this pass) is copied by
//       tick() alongside otos= itself, independent of otos='s own pose
//       values.
//
// Added by ticket 096-003 (SUC-003, architecture-update.md (096) M3/
// Decision 6): TlmFrameInput gains bench-diagnostic fields (acc/active/
// conn/glitch/ts, transcribed from the since-deleted handleTlm()) and a
// second formatter, Telemetry::buildTelemetryMessage() (TlmFrameInput ->
// msg::Telemetry):
//   (i) buildTelemetryMessage() populates every msg::Telemetry field
//       (including the `has_*` presence flags and the five
//       bench-diagnostic field groups) correctly from a fully-populated
//       TlmFrameInput.
//   (j) buildTelemetryMessage() copies each `has_*` flag independently and
//       always resets `out` first (pure, stateless).
//   (k) Telemetry::tick() sources the bench-diagnostic fields from
//       bb.drivetrain.acc()/.busy and bb.motors[0]/bb.motors[1] directly,
//       against a bare, non-live Rt::Blackboard -- folded into scenario (f).
//
// Plain C++ program, hand-rolled assertions (mirrors the existing
// harnesses' shape) -- prints a PASS/FAIL line per scenario and exits
// nonzero if any assertion failed.
//
// Verification command:
//   c++ -std=c++20 -Wall -Wextra \
//       -I source \
//       -o /tmp/tlm_frame_harness \
//       tests/sim/unit/tlm_frame_harness.cpp source/telemetry/tlm_frame.cpp \
//       source/kinematics/body_kinematics.cpp
//   /tmp/tlm_frame_harness

#include <cmath>
#include <cstdio>
#include <string>

#include "telemetry/tlm_frame.h"

namespace {

// --- Hand-rolled assertion plumbing (mirrors ekf_tiny_harness.cpp /
// velocity_pid_harness.cpp) ---

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
  if (!condition) fail(what + " — expected true, got false");
}

// A fully-populated TlmFrameInput, used as the common starting point for
// the buildTelemetryMessage() scenarios below (each omission scenario
// clears exactly one `has*` flag off of this baseline).
Telemetry::TlmFrameInput baselineInput() {
  Telemetry::TlmFrameInput in;
  in.now = 12345;
  in.driveMode = msg::DriveMode::STREAMING;
  in.seq = 7;

  in.hasEnc = true;
  in.encLeft = 1024.0f;
  in.encRight = 1019.0f;

  in.hasVel = true;
  in.velLeft = 198.0f;
  in.velRight = 201.0f;

  // cmd= (commanded/setpoint) uses values distinct from vel= (measured) so a
  // formatter that swapped the two tokens would be caught by the exact match.
  in.hasCmdVel = true;
  in.cmdVelLeft = 205.0f;
  in.cmdVelRight = 195.0f;

  in.hasPose = true;
  in.pose.x = 350.0f;
  in.pose.y = -12.0f;
  in.pose.h = 0.3f;

  in.hasEncPose = true;
  in.encPose.x = 349.0f;
  in.encPose.y = -11.0f;
  in.encPose.h = 0.31f;

  in.hasOtos = true;
  in.otos.x = 351.0f;
  in.otos.y = -13.0f;
  in.otos.h = 0.32f;
  in.otosConnected = true;

  in.hasTwist = true;
  in.twist.v_x = 200.0f;
  in.twist.omega = 0.5f;

  // Bench-diagnostic fields (096-003) -- distinctive non-default,
  // non-symmetric values (left != right, several nonzero digits) so a
  // field swap would be caught.
  in.accLeft = 15.5f;
  in.accRight = -8.25f;
  in.active = true;
  in.connLeft = true;
  in.connRight = false;
  in.glitchLeft = 3;
  in.glitchRight = 9;
  in.tsLeft = 111222;
  in.tsRight = 333444;

  return in;
}

// --- Scenarios ----------------------------------------------------------

// (f)/(g)/(k) 087-008/092-002/096-003: Telemetry::tick() assembles a
// TlmFrameInput directly from a bare, hand-populated Rt::Blackboard -- no
// live subsystem (Hardware, Drivetrain, PoseEstimator, Planner,
// CommandRouter) behind any cell. Proves Telemetry's OWN frame-assembly
// internals read exclusively from bb (SUC-006) and are independently
// unit-testable with no wiring at all (SUC-002) -- the isolated-testability
// bar every other subsystem (Blackboard itself, runtime_blackboard_harness.cpp;
// Drivetrain, drivetrain_harness.cpp; etc.) already meets.
void scenarioTickAssemblesFromBareBlackboard() {
  beginScenario("Telemetry::tick() assembles a frame from a bare Rt::Blackboard");

  Rt::Blackboard bb;   // default-constructed -- no subsystem behind any cell

  // Drivetrain's bound pair: left=port 1 (bb.motors[0]), right=port 2
  // (bb.motors[1]) -- enc=/vel= must read THESE two cells directly.
  bb.drivetrainConfig.left_port = 1;
  bb.drivetrainConfig.right_port = 2;
  bb.drivetrainConfig.trackwidth = 100.0f;   // [mm]

  bb.motors[0].position.has = true;
  bb.motors[0].position.val = 500.0f;
  bb.motors[0].velocity.has = true;
  bb.motors[0].velocity.val = 180.0f;

  bb.motors[1].position.has = true;
  bb.motors[1].position.val = 495.0f;
  bb.motors[1].velocity.has = true;
  bb.motors[1].velocity.val = 220.0f;

  // Commanded per-wheel velocity (the PID setpoints) live on bb.drivetrain's
  // cmd_[] array (cmd_[0]=left, cmd_[1]=right -- drivetrain.cpp state()'s
  // post-governor commanded targets). bb.drivetrain ALSO carries a vel_[]
  // array, but that one is MEASURED (state()'s "vel_[] are sourced from
  // hardware_.motorState(i) -- MEASURED, not commanded" contract) -- the
  // 2026-07-11 tlm_frame.cpp fix moved cmd='s source from vel_[] (which
  // silently duplicated vel=) to cmd_[]. All three arrays get distinct
  // values here so any source mix-up (cmd= reading bb.motors, cmd= reading
  // bb.drivetrain's measured vel_[], or vel= reading bb.drivetrain) is
  // caught below.
  bb.drivetrain.vel_[0] = 185.0f;   // measured mirror -- must NOT surface as cmd=
  bb.drivetrain.vel_[1] = 215.0f;
  bb.drivetrain.vel_count = 2;
  bb.drivetrain.cmd_[0] = 190.0f;
  bb.drivetrain.cmd_[1] = 210.0f;
  bb.drivetrain.cmd_count = 2;

  // Three independent, distinct headings, with distinct x/y so a field
  // swap between pose=/encpose=/otos= would still be caught.
  bb.fusedPose.pose.x = 400.0f;
  bb.fusedPose.pose.y = -20.0f;
  bb.fusedPose.pose.h = 0.3f;

  bb.encoderPose.pose.x = 398.0f;
  bb.encoderPose.pose.y = -19.0f;
  bb.encoderPose.pose.h = 0.31f;

  bb.otosPresent = true;
  bb.otos.pose.x = 402.0f;
  bb.otos.pose.y = -21.0f;
  bb.otos.pose.h = 0.32f;
  bb.otosConnected = true;   // 092-002: proves tick() copies bb.otosConnected too

  bb.planner.mode = msg::DriveMode::DISTANCE;
  bb.telemetrySeq = 42;

  // Bench-diagnostic fields (096-003, (k)) -- bb.drivetrain.acc()/.busy for
  // acc=/active=; bb.motors[0]/bb.motors[1] DIRECTLY (the SAME hardcoded
  // bound-pair the since-deleted handleTlm() itself read) for
  // conn=/glitch=/ts=. Distinct from the enc=/vel= values above so a source
  // mix-up would be caught.
  bb.drivetrain.acc_[0] = 33.0f;
  bb.drivetrain.acc_[1] = -17.0f;
  bb.drivetrain.acc_count = 2;
  bb.drivetrain.busy = true;

  bb.motors[0].connected = true;
  bb.motors[0].enc_glitch_count.has = true;
  bb.motors[0].enc_glitch_count.val = 4;
  bb.motors[0].sampled_at.has = true;
  bb.motors[0].sampled_at.val = 88888;

  bb.motors[1].connected = false;
  bb.motors[1].enc_glitch_count.has = true;
  bb.motors[1].enc_glitch_count.val = 12;
  bb.motors[1].sampled_at.has = true;
  bb.motors[1].sampled_at.val = 77777;

  Telemetry::TlmFrameInput in = Telemetry::tick(99999, bb);

  // bb is untouched by tick() -- confirms the "read only" half of the
  // contract before checking the assembled frame itself.
  checkTrue(bb.telemetrySeq == 42, "tick() does not mutate bb.telemetrySeq");

  checkTrue(in.now == 99999, "now == the `now` argument");
  checkTrue(in.driveMode == msg::DriveMode::DISTANCE, "driveMode == bb.planner.mode, copied verbatim");
  checkTrue(in.seq == 42, "seq == bb.telemetrySeq (read only)");

  checkTrue(in.hasEnc, "hasEnc set");
  checkTrue(in.encLeft == 500.0f, "encLeft == bb.motors[0].position");
  checkTrue(in.encRight == 495.0f, "encRight == bb.motors[1].position");

  checkTrue(in.hasVel, "hasVel set");
  checkTrue(in.velLeft == 180.0f, "velLeft == bb.motors[0].velocity (measured, not commanded)");
  checkTrue(in.velRight == 220.0f, "velRight == bb.motors[1].velocity (measured, not commanded)");

  checkTrue(in.hasCmdVel, "hasCmdVel set (bb.drivetrain.cmd_count >= 2)");
  checkTrue(in.cmdVelLeft == 190.0f, "cmdVelLeft == bb.drivetrain.cmd()[0] (commanded, not measured)");
  checkTrue(in.cmdVelRight == 210.0f, "cmdVelRight == bb.drivetrain.cmd()[1] (commanded, not measured)");

  checkTrue(in.hasPose, "hasPose set");
  checkTrue(in.pose.x == 400.0f && in.pose.y == -20.0f && in.pose.h == 0.3f,
            "pose == bb.fusedPose.pose");

  checkTrue(in.hasEncPose, "hasEncPose set");
  checkTrue(in.encPose.x == 398.0f && in.encPose.y == -19.0f && in.encPose.h == 0.31f,
            "encPose == bb.encoderPose.pose");

  checkTrue(in.hasOtos, "hasOtos set (bb.otosPresent)");
  checkTrue(in.otos.x == 402.0f && in.otos.y == -21.0f && in.otos.h == 0.32f,
            "otos == bb.otos.pose");
  checkTrue(in.otosConnected, "otosConnected == bb.otosConnected (092-002)");

  checkTrue(in.hasTwist, "hasTwist set");
  // twist= is a REAL BodyKinematics::forward() computation over the
  // directly-read wheel velocities and bb.drivetrainConfig.trackwidth:
  // v=(180+220)/2=200 exactly (both operands and their sum are exactly
  // representable in float32), and omega=(220-180)/100=0.4 rad/s (0.4's
  // float32 rounding error is ~6e-6, well inside the tolerance below).
  checkTrue(in.twist.v_x == 200.0f, "twist.v_x == BodyKinematics::forward() of the measured velocities");
  checkTrue(std::fabs(in.twist.omega - 0.4f) < 1e-5f,
            "twist.omega == BodyKinematics::forward() of the measured velocities");

  // (k) Bench-diagnostic fields sourced exactly as the since-deleted
  // handleTlm() used to compute them (git history, motion_commands.cpp).
  checkTrue(in.accLeft == 33.0f, "accLeft == bb.drivetrain.acc()[0]");
  checkTrue(in.accRight == -17.0f, "accRight == bb.drivetrain.acc()[1]");
  checkTrue(in.active == true, "active == bb.drivetrain.busy");
  checkTrue(in.connLeft == true, "connLeft == bb.motors[0].connected");
  checkTrue(in.connRight == false, "connRight == bb.motors[1].connected");
  checkTrue(in.glitchLeft == 4, "glitchLeft == bb.motors[0].enc_glitch_count");
  checkTrue(in.glitchRight == 12, "glitchRight == bb.motors[1].enc_glitch_count");
  checkTrue(in.tsLeft == 88888, "tsLeft == bb.motors[0].sampled_at");
  checkTrue(in.tsRight == 77777, "tsRight == bb.motors[1].sampled_at");
}

// (i) 096-003: Telemetry::buildTelemetryMessage() populates every
// msg::Telemetry field -- including the `has_*` presence flags and the
// five bench-diagnostic field groups -- from a fully-populated
// TlmFrameInput, field-for-field against hand-computed expected values.
void scenarioBuildTelemetryMessageAllFieldsPresent() {
  beginScenario("buildTelemetryMessage() populates every msg::Telemetry field correctly");

  Telemetry::TlmFrameInput in = baselineInput();
  msg::Telemetry out;
  Telemetry::buildTelemetryMessage(out, in);

  checkTrue(out.now == 12345, "now");
  checkTrue(out.mode == msg::DriveMode::STREAMING, "mode carries the RAW enum (driveMode)");
  checkTrue(out.seq == 7, "seq");

  checkTrue(out.has_enc == true, "has_enc");
  checkTrue(out.enc_left == 1024.0f, "enc_left");
  checkTrue(out.enc_right == 1019.0f, "enc_right");

  checkTrue(out.has_vel == true, "has_vel");
  checkTrue(out.vel_left == 198.0f, "vel_left");
  checkTrue(out.vel_right == 201.0f, "vel_right");

  checkTrue(out.has_cmd_vel == true, "has_cmd_vel");
  checkTrue(out.cmd_vel_left == 205.0f, "cmd_vel_left");
  checkTrue(out.cmd_vel_right == 195.0f, "cmd_vel_right");

  checkTrue(out.has_pose == true, "has_pose");
  checkTrue(out.pose.x == 350.0f && out.pose.y == -12.0f && out.pose.h == 0.3f, "pose");

  checkTrue(out.has_otos == true, "has_otos");
  checkTrue(out.otos.x == 351.0f && out.otos.y == -13.0f && out.otos.h == 0.32f, "otos");
  checkTrue(out.otos_connected == true, "otos_connected");

  checkTrue(out.has_twist == true, "has_twist");
  checkTrue(out.twist.v_x == 200.0f && out.twist.omega == 0.5f, "twist (radians, NOT mrad -- binary carries real units)");

  // Bench-diagnostic fields -- unconditionally present, no has_* flag.
  checkTrue(out.acc_left == 15.5f, "acc_left");
  checkTrue(out.acc_right == -8.25f, "acc_right");
  checkTrue(out.active == true, "active");
  checkTrue(out.conn_left == true, "conn_left");
  checkTrue(out.conn_right == false, "conn_right");
  checkTrue(out.glitch_left == 3, "glitch_left");
  checkTrue(out.glitch_right == 9, "glitch_right");
  checkTrue(out.ts_left == 111222, "ts_left");
  checkTrue(out.ts_right == 333444, "ts_right");
}

// (j) 096-003: each `has_*` presence flag is copied independently -- an
// omitted TlmFrameInput field produces a `false` msg::Telemetry has_*
// flag while every other field is unaffected.
void scenarioBuildTelemetryMessagePresenceFlagsIndependent() {
  beginScenario("buildTelemetryMessage() copies has_* flags independently");

  Telemetry::TlmFrameInput in = baselineInput();
  in.hasEnc = false;
  in.hasOtos = false;

  msg::Telemetry out;
  Telemetry::buildTelemetryMessage(out, in);

  checkTrue(out.has_enc == false, "has_enc reflects hasEnc == false");
  checkTrue(out.has_otos == false, "has_otos reflects hasOtos == false");
  checkTrue(out.has_vel == true, "has_vel unaffected by the other two flags being false");
  checkTrue(out.has_cmd_vel == true, "has_cmd_vel unaffected");
  checkTrue(out.has_pose == true, "has_pose unaffected");
  checkTrue(out.has_twist == true, "has_twist unaffected");
  checkTrue(out.active == true, "bench-diagnostic fields unaffected by has_* flags (they have none of their own)");
}

// (j) buildTelemetryMessage() always resets `out` first -- pure, stateless:
// a caller-supplied struct carrying stale/garbage values from a PRIOR call
// must not leak into this call's output.
void scenarioBuildTelemetryMessageResetsStaleState() {
  beginScenario("buildTelemetryMessage() resets stale caller state (pure, stateless)");

  Telemetry::TlmFrameInput in;   // minimal input -- no optional fields present
  in.now = 500;
  in.driveMode = msg::DriveMode::IDLE;
  in.seq = 0;

  msg::Telemetry out;
  // Poison `out` with values a minimal `in` would never itself produce.
  out.has_enc = true;
  out.enc_left = 9999.0f;
  out.has_pose = true;
  out.pose.x = -9999.0f;
  out.acc_left = 9999.0f;

  Telemetry::buildTelemetryMessage(out, in);

  checkTrue(out.has_enc == false, "stale has_enc cleared");
  checkTrue(out.enc_left == 0.0f, "stale enc_left cleared");
  checkTrue(out.has_pose == false, "stale has_pose cleared");
  checkTrue(out.pose.x == 0.0f, "stale pose.x cleared");
  checkTrue(out.acc_left == 0.0f, "stale acc_left cleared (in.accLeft defaults to 0)");
}

// Determinism: the SAME input, called twice, must produce the SAME
// msg::Telemetry -- buildTelemetryMessage() is pure (no I/O, no hidden
// state).
void scenarioBuildTelemetryMessageDeterministic() {
  beginScenario("buildTelemetryMessage(): same input produces the same output (pure, stateless)");

  Telemetry::TlmFrameInput in = baselineInput();
  msg::Telemetry outA;
  msg::Telemetry outB;
  Telemetry::buildTelemetryMessage(outA, in);
  Telemetry::buildTelemetryMessage(outB, in);

  checkTrue(outA.now == outB.now && outA.mode == outB.mode && outA.seq == outB.seq &&
                outA.enc_left == outB.enc_left && outA.acc_left == outB.acc_left &&
                outA.ts_right == outB.ts_right,
            "two calls with an identical input match exactly");
}

}  // namespace

int main() {
  scenarioTickAssemblesFromBareBlackboard();
  scenarioBuildTelemetryMessageAllFieldsPresent();
  scenarioBuildTelemetryMessagePresenceFlagsIndependent();
  scenarioBuildTelemetryMessageResetsStaleState();
  scenarioBuildTelemetryMessageDeterministic();

  if (g_failureCount == 0) {
    std::printf("OK: all tlm_frame scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the tlm_frame scenarios\n", g_failureCount);
  return 1;
}
