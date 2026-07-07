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
  in.seq = 7;

  in.hasEnc = true;
  in.encLeft = 1024.0f;
  in.encRight = 1019.0f;

  in.hasVel = true;
  in.velLeft = 198.0f;
  in.velRight = 201.0f;

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

  in.hasTwist = true;
  in.twist.v_x = 200.0f;
  in.twist.omega = 0.5f;   // -> 500 mrad/s

  return in;
}

// --- Scenarios ----------------------------------------------------------

// (a) All fields present -- exact wire-line match. Proves field order
// (t= mode= seq= enc= vel= pose= encpose= otos= twist=, per the ticket's
// own field-list ordering), integer truncation (not rounding -- matches
// source_old/robot/RobotTelemetry.cpp's (int) casts), and the
// radians-to-centidegrees / rad-per-s-to-mrad-per-s scale factors.
void scenarioAllFieldsPresentExactMatch() {
  beginScenario("all fields present -- exact wire-line match");

  Telemetry::TlmFrameInput in = baselineInput();
  char buf[300];
  int n = Telemetry::buildTlmFrame(buf, sizeof(buf), in);

  const std::string expected =
      "TLM t=12345 mode=S seq=7 enc=1024,1019 vel=198,201 pose=350,-12,1718"
      " encpose=349,-11,1776 otos=351,-13,1833 twist=200,500";
  checkEq(std::string(buf), expected, "exact formatted line");
  checkTrue(n == static_cast<int>(expected.size()), "return value equals formatted length");
  checkTrue(std::strlen(buf) == expected.size(), "NUL terminator lands exactly at the formatted length");
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
  checkTrue(!contains(buf, "pose="), "pose= absent (encpose= substring check below disambiguates)");
  checkTrue(!contains(buf, "encpose="), "encpose= absent");
  checkTrue(!contains(buf, "otos="), "otos= absent");
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
  checkTrue(contains(buf, "pose=350,-12,1718"), "pose= still present and correct");
  checkTrue(contains(buf, "twist=200,500"), "twist= still present and correct");
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
  checkTrue(contains(buf, "pose=350,-12,1718"), "pose= still present and correct");
  checkTrue(contains(buf, "twist=200,500"), "twist= still present and correct");
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

  // Drivetrain's bound pair: left=port 1 (bb.motor[0]), right=port 2
  // (bb.motor[1]) -- enc=/vel= must read THESE two cells directly.
  bb.drivetrainConfig.left_port = 1;
  bb.drivetrainConfig.right_port = 2;
  bb.drivetrainConfig.trackwidth = 100.0f;   // [mm]

  bb.motor[0].position.has = true;
  bb.motor[0].position.val = 500.0f;
  bb.motor[0].velocity.has = true;
  bb.motor[0].velocity.val = 180.0f;

  bb.motor[1].position.has = true;
  bb.motor[1].position.val = 495.0f;
  bb.motor[1].velocity.has = true;
  bb.motor[1].velocity.val = 220.0f;

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

  bb.planner.mode = msg::DriveMode::DISTANCE;   // -> mode=D
  bb.telemetrySeq = 42;

  Telemetry::TlmFrameInput in = Telemetry::tick(99999, bb);

  // bb is untouched by tick() -- confirms the "read only" half of the
  // contract before checking the assembled frame itself.
  checkTrue(bb.telemetrySeq == 42, "tick() does not mutate bb.telemetrySeq");

  char buf[300];
  Telemetry::buildTlmFrame(buf, sizeof(buf), in);

  // enc=/vel= straight off bb.motor[0]/[1] (the bound pair); twist= is a
  // REAL BodyKinematics::forward() computation over those same velocities
  // and bb.drivetrainConfig.trackwidth: v=(180+220)/2=200 exactly (both
  // operands and their sum are exactly representable in float32), and
  // omega=(220-180)/100=0.4 rad/s -> 400 mrad/s (0.4's float32 rounding
  // error is ~6e-6, nowhere near the next integer boundary at 401).
  const std::string expected =
      "TLM t=99999 mode=D seq=42 enc=500,495 vel=180,220 pose=400,-20,1718"
      " encpose=398,-19,1776 otos=402,-21,1833 twist=200,400";
  checkEq(std::string(buf), expected, "frame assembled entirely from bare Rt::Blackboard cells");
}

}  // namespace

int main() {
  scenarioAllFieldsPresentExactMatch();
  scenarioNoOptionalFields();
  scenarioEncOmittedIndependently();
  scenarioVelOmittedIndependently();
  scenarioPoseOmittedIndependently();
  scenarioEncPoseOmittedIndependently();
  scenarioOtosOmittedNotZeroFilled();
  scenarioTwistOmittedIndependently();
  scenarioSmallBufferTruncatesSafely();
  scenarioDeterministic();
  scenarioTickAssemblesFromBareBlackboard();

  if (g_failureCount == 0) {
    std::printf("OK: all tlm_frame scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the tlm_frame scenarios\n", g_failureCount);
  return 1;
}
