// tlm_frame_harness.cpp — off-hardware acceptance harness for ticket 082-004
// (SUC-004), extended by ticket 087-008 (SUC-001/SUC-002/SUC-006): exercises
// Telemetry::buildTlmFrame() (source/telemetry/tlm_frame.{h,cpp}) -- the
// pure, stateless TLM frame-formatting function -- in isolation, with no
// DevLoop/Hardware/Drivetrain/PoseEstimator wiring (that wiring is
// commands/telemetry_commands.cpp's job, exercised end-to-end via the
// ctypes sim harness in ticket 005); and, since 087-008, Telemetry::tick()
// (bb -> TlmFrameInput), Telemetry's OWN frame-assembly step, against a
// bare, non-live Rt::Blackboard -- no CommandRouter/Communicator/Hardware/
// Drivetrain/PoseEstimator/Planner object of any kind, proving Telemetry's
// isolated testability (SUC-002) directly.
//
// Per ekf_tiny_harness.cpp / velocity_pid_harness.cpp's precedent (082-001 /
// 081-001), this #includes only telemetry/tlm_frame.h (which itself pulls in
// runtime/blackboard.h -- host-safe, see that header's own file comment)
// plus its own translation unit (tlm_frame.cpp) and
// kinematics/body_kinematics.cpp (Telemetry::tick()'s one pure-math
// dependency, for twist=), so it compiles with the plain system C++
// compiler -- no CMake, no ARM toolchain.
//
// Required scenarios (ticket 082-004's Testing plan):
//   (a) all fields present -- exact wire-line match, proving field order,
//       integer scaling (pose/encpose/otos centidegrees, twist mrad/s), and
//       token spelling all at once.
//   (b) each optional field independently omitted -- the field's own token
//       is absent from the line while every other present field's token is
//       unaffected.
//   (c) otos= specifically: omitted (hasOtos = false) must NOT appear as a
//       zero-filled "otos=0,0,0" -- omission and zero-fill are two
//       different things, and only omission is correct when the caller has
//       no odometer (Decision 7).
//   (d) no optional fields present at all -- just the mandatory t=/mode=/
//       seq= prefix.
//   (e) a buffer too small to hold the full line still NUL-terminates
//       safely (no overrun) -- buildTlmFrame() takes a caller-supplied
//       length like every other frame-builder in this codebase.
//
// Added by ticket 087-008's Testing plan:
//   (f) Telemetry::tick() reads every field directly off a bare,
//       hand-populated Rt::Blackboard (no live subsystem behind any cell)
//       and the resulting frame matches hand-computed expected values --
//       the isolated-testability proof for Telemetry's own frame assembly.
//
// Added by ticket 092-002 (frozen-fused-pose investigation, diagnostic
// telemetry -- clasi/issues/poseestimator-fused-pose-frozen-on-hardware.md):
//   (g) otosconn= (Hal::Odometer::connected() this pass) is a SEPARATE
//       token sharing otos='s own omission gate, independent of otos='s own
//       pose values -- proven both in isolation (buildTlmFrame()) and
//       through Telemetry::tick()'s bb.otosConnected wiring.
//
// Added by ticket 096-003 (SUC-003, architecture-update.md (096) M3/
// Decision 6): TlmFrameInput gains bench-diagnostic fields (acc/active/
// conn/glitch/ts, transcribed from handleTlm()) and a second formatter,
// Telemetry::buildTelemetryMessage() (TlmFrameInput -> msg::Telemetry):
//   (h) buildTlmFrame()'s text output stays byte-identical even when the
//       new bench-diagnostic fields are populated with distinctive
//       non-default values -- the hard regression gate this ticket's own
//       acceptance criteria require.
//   (i) buildTelemetryMessage() populates every msg::Telemetry field
//       (including the `has_*` presence flags and the five
//       bench-diagnostic field groups) correctly from a fully-populated
//       TlmFrameInput.
//   (j) buildTelemetryMessage() copies each `has_*` flag independently
//       (mirrors buildTlmFrame()'s own per-field omission proof) and
//       always resets `out` first (pure, stateless).
//   (k) Telemetry::tick() sources the bench-diagnostic fields from
//       bb.drivetrain.acc()/.busy and bb.motors[0]/bb.motors[1] directly,
//       against a bare, non-live Rt::Blackboard.
//
// Plain C++ program, hand-rolled assertions (mirrors the existing
// harnesses' shape) -- prints a PASS/FAIL line per scenario and exits
// nonzero if any assertion failed.
//
// Verification command (see ticket 082-004's Testing plan, extended by
// 087-008 for Telemetry::tick()'s body_kinematics.cpp dependency):
//   c++ -std=c++11 -Wall -Wextra \
//       -I source \
//       -o /tmp/tlm_frame_harness \
//       tests/sim/unit/tlm_frame_harness.cpp source/telemetry/tlm_frame.cpp \
//       source/kinematics/body_kinematics.cpp
//   /tmp/tlm_frame_harness

#include <cstdio>
#include <cstring>
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

void checkEq(const std::string& actual, const std::string& expected, const std::string& what) {
  if (actual != expected) {
    fail(what + " — expected \"" + expected + "\", got \"" + actual + "\"");
  }
}

// Substring presence/absence helpers -- used for the per-field omission
// scenarios, where the exact surrounding text (seq=, other fields' values)
// is irrelevant; only "is this token present or absent" matters.
bool contains(const std::string& haystack, const std::string& needle) {
  return haystack.find(needle) != std::string::npos;
}

// A fully-populated TlmFrameInput, used as the common starting point for
// every scenario below (each omission scenario clears exactly one `has*`
// flag off of this baseline).
Telemetry::TlmFrameInput baselineInput() {
  Telemetry::TlmFrameInput in;
  in.now = 12345;
  in.mode = 'S';
  in.driveMode = msg::DriveMode::STREAMING;   // same source value 'S' is mapped from
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

  // Heading values are chosen with a comfortable fractional margin (>0.1)
  // away from any centidegree integer boundary -- e.g. 0.3 rad * kAngleScale
  // ~= 1718.87 -- so float32 rounding noise can never flip the truncated
  // integer result, and each of the three headings is distinct (catches an
  // accidental field swap between pose/encpose/otos).
  in.hasPose = true;
  in.pose.x = 350.0f;
  in.pose.y = -12.0f;
  in.pose.h = 0.3f;    // -> 1718 centidegrees

  in.hasEncPose = true;
  in.encPose.x = 349.0f;
  in.encPose.y = -11.0f;
  in.encPose.h = 0.31f;   // -> 1776 centidegrees

  in.hasOtos = true;
  in.otos.x = 351.0f;
  in.otos.y = -13.0f;
  in.otos.h = 0.32f;   // -> 1833 centidegrees
  in.otosConnected = true;

  in.hasTwist = true;
  in.twist.v_x = 200.0f;
  in.twist.omega = 0.5f;   // -> 500 mrad/s

  // Bench-diagnostic fields (096-003) -- distinctive non-default,
  // non-symmetric values (left != right, several nonzero digits) so a
  // field swap or an accidental buildTlmFrame() leak would be caught.
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

// (a)+(h) All fields present -- exact wire-line match. Proves field order
// (t= mode= seq= enc= vel= pose= encpose= otos= twist=, per the ticket's
// own field-list ordering), integer truncation (not rounding -- matches
// source_old/robot/RobotTelemetry.cpp's (int) casts), and the
// radians-to-centidegrees / rad-per-s-to-mrad-per-s scale factors.
// Since 096-003, baselineInput() ALSO populates the bench-diagnostic
// fields (acc/active/conn/glitch/ts) with distinctive non-default values
// -- this scenario's expected string is UNCHANGED from before that
// extension, and the explicit absence checks below confirm none of the
// bench-diagnostic tokens leak into the text line: this IS this ticket's
// "buildTlmFrame()'s text output is byte-identical before and after"
// regression proof (SUC-003's own acceptance criterion).
void scenarioAllFieldsPresentExactMatch() {
  beginScenario("all fields present -- exact wire-line match (096-003: byte-identical despite bench-diagnostic fields)");

  Telemetry::TlmFrameInput in = baselineInput();
  char buf[300];
  int n = Telemetry::buildTlmFrame(buf, sizeof(buf), in);

  const std::string expected =
      "TLM t=12345 mode=S seq=7 enc=1024,1019 vel=198,201 cmd=205,195"
      " pose=350,-12,1718 encpose=349,-11,1776 otos=351,-13,1833 otosconn=1"
      " twist=200,500";
  checkEq(std::string(buf), expected, "exact formatted line, unchanged since before 096-003");
  checkTrue(n == static_cast<int>(expected.size()), "return value equals formatted length");
  checkTrue(std::strlen(buf) == expected.size(), "NUL terminator lands exactly at the formatted length");
  checkTrue(!contains(buf, "acc="), "acc= never appears in the text line (bench-diagnostic, binary-only)");
  checkTrue(!contains(buf, "active="), "active= never appears in the text line (bench-diagnostic, binary-only)");
  // " conn=" (leading space) -- NOT a bare "conn=" substring check, which
  // would false-positive on "otosconn=1" (the pre-existing, unrelated
  // 092-002 token whose own tail spells "...conn=").
  checkTrue(!contains(buf, " conn="), "conn= never appears in the text line (bench-diagnostic, binary-only)");
  checkTrue(!contains(buf, "glitch="), "glitch= never appears in the text line (bench-diagnostic, binary-only)");
  checkTrue(!contains(buf, " ts="), "ts= never appears in the text line (bench-diagnostic, binary-only)");
}

// (d) No optional fields present -- just the mandatory prefix.
void scenarioNoOptionalFields() {
  beginScenario("no optional fields present -- mandatory prefix only");

  Telemetry::TlmFrameInput in;
  in.now = 500;
  in.mode = 'I';
  in.seq = 0;

  char buf[128];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);

  checkEq(std::string(buf), "TLM t=500 mode=I seq=0", "prefix-only line");
  checkTrue(!contains(buf, "enc="), "enc= absent");
  checkTrue(!contains(buf, "vel="), "vel= absent");
  checkTrue(!contains(buf, "cmd="), "cmd= absent");
  checkTrue(!contains(buf, "pose="), "pose= absent (encpose= substring check below disambiguates)");
  checkTrue(!contains(buf, "encpose="), "encpose= absent");
  checkTrue(!contains(buf, "otos="), "otos= absent");
  checkTrue(!contains(buf, "otosconn="), "otosconn= absent");
  checkTrue(!contains(buf, "twist="), "twist= absent");
}

// (b) enc= independently omitted -- every other baseline field unaffected.
void scenarioEncOmittedIndependently() {
  beginScenario("enc= omitted independently");

  Telemetry::TlmFrameInput in = baselineInput();
  in.hasEnc = false;

  char buf[300];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);

  checkTrue(!contains(buf, "enc="), "enc= absent when hasEnc is false");
  checkTrue(contains(buf, "vel=198,201"), "vel= still present and correct");
  checkTrue(contains(buf, "pose=350,-12,1718"), "pose= still present and correct");
  checkTrue(contains(buf, "encpose=349,-11,1776"), "encpose= still present and correct");
  checkTrue(contains(buf, "otos=351,-13,1833"), "otos= still present and correct");
  checkTrue(contains(buf, "twist=200,500"), "twist= still present and correct");
}

// (b) vel= independently omitted.
void scenarioVelOmittedIndependently() {
  beginScenario("vel= omitted independently");

  Telemetry::TlmFrameInput in = baselineInput();
  in.hasVel = false;

  char buf[300];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);

  checkTrue(contains(buf, "enc=1024,1019"), "enc= still present and correct");
  checkTrue(!contains(buf, "vel="), "vel= absent when hasVel is false");
  checkTrue(contains(buf, "cmd=205,195"), "cmd= still present and correct (independent of vel=)");
  checkTrue(contains(buf, "pose=350,-12,1718"), "pose= still present and correct");
  checkTrue(contains(buf, "twist=200,500"), "twist= still present and correct");
}

// (b) cmd= independently omitted -- the commanded-velocity token is absent
// while measured vel= (and every other field) is unaffected.
void scenarioCmdVelOmittedIndependently() {
  beginScenario("cmd= omitted independently");

  Telemetry::TlmFrameInput in = baselineInput();
  in.hasCmdVel = false;

  char buf[300];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);

  checkTrue(!contains(buf, "cmd="), "cmd= absent when hasCmdVel is false");
  checkTrue(contains(buf, "vel=198,201"), "vel= (measured) still present and correct");
  checkTrue(contains(buf, "pose=350,-12,1718"), "pose= still present and correct");
}

// (b) pose= independently omitted -- must not also remove encpose= (which
// shares the "pose" substring as a suffix of its own token name).
void scenarioPoseOmittedIndependently() {
  beginScenario("pose= omitted independently (encpose= must remain)");

  Telemetry::TlmFrameInput in = baselineInput();
  in.hasPose = false;

  char buf[300];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);

  checkTrue(!contains(buf, " pose="), "pose= absent when hasPose is false");
  checkTrue(contains(buf, "encpose=349,-11,1776"), "encpose= still present and correct");
}

// (b) encpose= independently omitted.
void scenarioEncPoseOmittedIndependently() {
  beginScenario("encpose= omitted independently");

  Telemetry::TlmFrameInput in = baselineInput();
  in.hasEncPose = false;

  char buf[300];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);

  checkTrue(!contains(buf, "encpose="), "encpose= absent when hasEncPose is false");
  checkTrue(contains(buf, "pose=350,-12,1718"), "pose= still present and correct");
}

// (b)+(c) otos= independently omitted -- the acceptance-critical case
// (Decision 7): must be ABSENT, never a zero-filled "otos=0,0,0".
void scenarioOtosOmittedNotZeroFilled() {
  beginScenario("otos= omitted, not zero-filled, when no odometer");

  Telemetry::TlmFrameInput in = baselineInput();
  in.hasOtos = false;
  // Deliberately leave in.otos at its non-zero baseline values -- if
  // buildTlmFrame() ignored hasOtos and formatted the struct anyway, this
  // scenario would still catch it (the token would show baseline values,
  // not necessarily zero) as well as the explicit zero-fill check below.
  checkTrue(in.otos.x != 0.0f, "sanity: baseline otos.x is non-zero");

  char buf[300];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);

  checkTrue(!contains(buf, "otos="), "otos= entirely absent (not emitted at all)");
  checkTrue(!contains(buf, "otos=0,0,0"), "otos= is not a zero-filled placeholder");
  checkTrue(!contains(buf, "otosconn="), "otosconn= absent too -- shares otos='s own omission gate");
  checkTrue(contains(buf, "pose=350,-12,1718"), "pose= still present and correct");
  checkTrue(contains(buf, "twist=200,500"), "twist= still present and correct");
}

// (092-002) otosconn= reflects otosConnected independently of otos='s own
// pose values -- a present-but-disconnected reading (e.g. Hal::OtosOdometer
// never detected a chip) must show otosconn=0 while otos= itself still
// prints its (stale/zero) cached pose, never conflating the two signals.
void scenarioOtosConnFalseWhilePosePresent() {
  beginScenario("otosconn=0 while otos= itself is still present");

  Telemetry::TlmFrameInput in = baselineInput();
  in.otosConnected = false;

  char buf[300];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);

  checkTrue(contains(buf, "otos=351,-13,1833"), "otos= pose still present and correct");
  checkTrue(contains(buf, "otosconn=0"), "otosconn=0 when otosConnected is false");
  checkTrue(!contains(buf, "otosconn=1"), "otosconn=1 must not also appear");
}

// (b) twist= independently omitted.
void scenarioTwistOmittedIndependently() {
  beginScenario("twist= omitted independently");

  Telemetry::TlmFrameInput in = baselineInput();
  in.hasTwist = false;

  char buf[300];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);

  checkTrue(!contains(buf, "twist="), "twist= absent when hasTwist is false");
  checkTrue(contains(buf, "otos=351,-13,1833"), "otos= still present and correct");
}

// (e) A buffer too small for the full line must still NUL-terminate safely
// (no overrun) -- buildTlmFrame() takes a caller-supplied length like every
// other frame-builder in this codebase (see dev_commands.cpp's
// emitMotorState()).
void scenarioSmallBufferTruncatesSafely() {
  beginScenario("small buffer truncates safely (no overrun)");

  Telemetry::TlmFrameInput in = baselineInput();
  char smallBuf[16];
  // Poison the buffer so a missing/incorrect NUL terminator is detectable.
  std::memset(smallBuf, 'X', sizeof(smallBuf));

  Telemetry::buildTlmFrame(smallBuf, sizeof(smallBuf), in);

  bool nulFound = false;
  for (size_t i = 0; i < sizeof(smallBuf); ++i) {
    if (smallBuf[i] == '\0') { nulFound = true; break; }
  }
  checkTrue(nulFound, "a NUL terminator exists within the caller-supplied length");
  checkTrue(std::strlen(smallBuf) < sizeof(smallBuf), "the terminated string fits inside the small buffer");
  checkTrue(std::strncmp(smallBuf, "TLM t=12345", 11) == 0,
            "the mandatory prefix is written first, before truncation");
}

// Determinism: the SAME input, called twice, must produce the SAME string --
// buildTlmFrame() is pure (no I/O, no hidden state).
void scenarioDeterministic() {
  beginScenario("same input produces the same string (pure, stateless)");

  Telemetry::TlmFrameInput in = baselineInput();
  char bufA[300];
  char bufB[300];
  Telemetry::buildTlmFrame(bufA, sizeof(bufA), in);
  Telemetry::buildTlmFrame(bufB, sizeof(bufB), in);

  checkEq(std::string(bufA), std::string(bufB), "two calls with an identical input match exactly");
}

// (f) 087-008: Telemetry::tick() assembles a TlmFrameInput directly from a
// bare, hand-populated Rt::Blackboard -- no live subsystem (Hardware,
// Drivetrain, PoseEstimator, Planner, CommandRouter) behind any cell.
// Proves Telemetry's OWN frame-assembly internals read exclusively from bb
// (SUC-006) and are independently unit-testable with no wiring at all
// (SUC-002) -- the isolated-testability bar every other subsystem
// (Blackboard itself, runtime_blackboard_harness.cpp; Drivetrain,
// drivetrain_harness.cpp; etc.) already meets.
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

  // Commanded per-wheel velocity (the PID setpoints) live on bb.drivetrain,
  // vel_[0]=left, vel_[1]=right -- distinct from the measured bb.motors[]
  // velocities above so a source mix-up (cmd= reading bb.motors, or vel=
  // reading bb.drivetrain) would be caught by the exact match below.
  bb.drivetrain.vel_[0] = 190.0f;
  bb.drivetrain.vel_[1] = 210.0f;
  bb.drivetrain.vel_count = 2;

  // Three independent, distinct headings (reusing baselineInput()'s own
  // 0.3/0.31/0.32 margin-tested values, so their centidegree truncations
  // -- 1718/1776/1833 -- are already known-good) with distinct x/y so a
  // field swap between pose=/encpose=/otos= would still be caught.
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

  bb.planner.mode = msg::DriveMode::DISTANCE;   // -> mode=D
  bb.telemetrySeq = 42;

  // Bench-diagnostic fields (096-003, (k)) -- bb.drivetrain.acc()/.busy for
  // acc=/active=; bb.motors[0]/bb.motors[1] DIRECTLY (the SAME hardcoded
  // bound-pair handleTlm() itself reads) for conn=/glitch=/ts=. Distinct
  // from the enc=/vel= values above so a source mix-up would be caught.
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

  // (k) Bench-diagnostic fields sourced exactly as handleTlm() computes
  // them -- checked directly on TlmFrameInput (buildTlmFrame() never emits
  // these, per scenario (h) above).
  checkTrue(in.accLeft == 33.0f, "accLeft == bb.drivetrain.acc()[0]");
  checkTrue(in.accRight == -17.0f, "accRight == bb.drivetrain.acc()[1]");
  checkTrue(in.active == true, "active == bb.drivetrain.busy");
  checkTrue(in.connLeft == true, "connLeft == bb.motors[0].connected");
  checkTrue(in.connRight == false, "connRight == bb.motors[1].connected");
  checkTrue(in.glitchLeft == 4, "glitchLeft == bb.motors[0].enc_glitch_count");
  checkTrue(in.glitchRight == 12, "glitchRight == bb.motors[1].enc_glitch_count");
  checkTrue(in.tsLeft == 88888, "tsLeft == bb.motors[0].sampled_at");
  checkTrue(in.tsRight == 77777, "tsRight == bb.motors[1].sampled_at");
  checkTrue(in.driveMode == msg::DriveMode::DISTANCE, "driveMode == bb.planner.mode, unmapped (raw enum)");

  char buf[300];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);

  // enc=/vel= straight off bb.motors[0]/[1] (the bound pair); twist= is a
  // REAL BodyKinematics::forward() computation over those same velocities
  // and bb.drivetrainConfig.trackwidth: v=(180+220)/2=200 exactly (both
  // operands and their sum are exactly representable in float32), and
  // omega=(220-180)/100=0.4 rad/s -> 400 mrad/s (0.4's float32 rounding
  // error is ~6e-6, nowhere near the next integer boundary at 401).
  const std::string expected =
      "TLM t=99999 mode=D seq=42 enc=500,495 vel=180,220 cmd=190,210"
      " pose=400,-20,1718 encpose=398,-19,1776 otos=402,-21,1833 otosconn=1"
      " twist=200,400";
  checkEq(std::string(buf), expected, "frame assembled entirely from bare Rt::Blackboard cells");
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
  checkTrue(out.mode == msg::DriveMode::STREAMING, "mode carries the RAW enum (driveMode), not the text char");
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
// flag while every other field is unaffected. Mirrors buildTlmFrame()'s
// own per-field omission scenarios above, proving the two formatters
// treat presence identically even though one omits a token and the other
// clears a bool.
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

// (j) buildTelemetryMessage() always resets `out` first -- pure, stateless,
// like buildTlmFrame(): a caller-supplied struct carrying stale/garbage
// values from a PRIOR call must not leak into this call's output.
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
// state), mirroring buildTlmFrame()'s own determinism scenario.
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
  scenarioAllFieldsPresentExactMatch();
  scenarioNoOptionalFields();
  scenarioEncOmittedIndependently();
  scenarioVelOmittedIndependently();
  scenarioCmdVelOmittedIndependently();
  scenarioPoseOmittedIndependently();
  scenarioEncPoseOmittedIndependently();
  scenarioOtosOmittedNotZeroFilled();
  scenarioOtosConnFalseWhilePosePresent();
  scenarioTwistOmittedIndependently();
  scenarioSmallBufferTruncatesSafely();
  scenarioDeterministic();
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
