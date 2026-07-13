// measurement_ring_harness.cpp — off-hardware acceptance harness for ticket
// DB-002 (device-bus-tickets.md): proves Devices::MeasurementRing<T>'s
// gap-write protocol, publish-immutability guarantee, and bracket()
// contract (source/devices/measurement_ring.h), plus the linear and
// wrap-aware angular lerp helpers (source/devices/interpolation.h).
//
// Header-only sources (no companion .cpp for either header) — this harness
// just includes them directly and compiles with -DHOST_BUILD for
// consistency with every other tests/sim/unit harness, though neither
// header actually forks on HOST_BUILD (both are plain host-clean C++, zero
// bus/CODAL dependency, per the isolation invariant).
//
// Hand-rolled assertions — mirrors devices_clock_harness.cpp / devices_
// otos_harness.cpp's shape exactly: prints a PASS/FAIL line per scenario
// and exits nonzero if any assertion failed, run by the pytest wrapper in
// test_measurement_ring.py.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "devices/interpolation.h"
#include "devices/measurement_ring.h"

namespace {

// --- Hand-rolled assertion plumbing (see devices_clock_harness.cpp) -------

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

void checkU64Eq(uint64_t actual, uint64_t expected, const std::string& what) {
  if (actual != expected) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %llu, got %llu",
                  what.c_str(), static_cast<unsigned long long>(expected),
                  static_cast<unsigned long long>(actual));
    fail(buf);
  }
}

void checkNear(float actual, float expected, float tol, const std::string& what) {
  if (std::fabs(static_cast<double>(actual - expected)) > tol) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%s -- expected %g, got %g (tol %g)", what.c_str(),
                  static_cast<double>(expected), static_cast<double>(actual),
                  static_cast<double>(tol));
    fail(buf);
  }
}

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;

// checkNearAngleDeg — compares `actualRad` (radians) against `expectedDeg`
// (degrees) via the WRAPPED difference, not a bare subtraction — ±180° is a
// single physical angle that can legitimately come back as either +180° or
// -180° depending on floating-point rounding right at the atan2 branch cut,
// and a bare-subtraction comparison would be flaky exactly at that seam
// (the case this harness most needs to exercise). Wrapping the difference
// into (-180°, 180°] before comparing against `tolDeg` sidesteps that.
void checkNearAngleDeg(float actualRad, float expectedDeg, float tolDeg,
                        const std::string& what) {
  const float actualDeg = actualRad * kRadToDeg;
  float diff = actualDeg - expectedDeg;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff <= -180.0f) diff += 360.0f;
  if (std::fabs(diff) > tolDeg) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "%s -- expected ~%g deg, got %g deg (wrapped diff %g, tol %g)",
                  what.c_str(), static_cast<double>(expectedDeg),
                  static_cast<double>(actualDeg), static_cast<double>(diff),
                  static_cast<double>(tolDeg));
    fail(buf);
  }
}

// --- Scenarios --------------------------------------------------------

// 1. Fill past the 6 physical slots (8 publishes into a 6-slot/5-published
//    ring) and prove exactly 5 correctly-ordered published samples survive
//    — the oldest 3 of the 8 are evicted, the newest 5 remain, newest-first.
void scenarioFillAndWrapKeepsFivePublishedInOrder() {
  beginScenario("8 publishes into a 6-slot ring keep the newest 5, in order");
  Devices::MeasurementRing<int> ring;

  checkFalse(ring.latest().valid, "a fresh ring's latest() is not valid");

  for (int i = 1; i <= 8; ++i) {
    ring.publish(i * 10, static_cast<uint64_t>(i) * 100);  // value=10*i, stamp=100*i [us]
  }

  // Newest-first: age 0 = publish #8 (value 80, stamp 800) ... age 4 =
  // publish #4 (value 40, stamp 400). Publishes #1-3 (values 10/20/30) must
  // be gone -- evicted past the 5-deep published window.
  const int expectedValues[Devices::MeasurementRing<int>::kDepth] = {80, 70, 60, 50, 40};
  const uint64_t expectedStamps[Devices::MeasurementRing<int>::kDepth] = {800, 700, 600, 500, 400};

  for (uint8_t age = 0; age < Devices::MeasurementRing<int>::kDepth; ++age) {
    const Devices::Sample<int> s = ring.sample(age);
    checkTrue(s.valid, "age " + std::to_string(age) + " is published/valid");
    if (s.value != expectedValues[age]) {
      fail("age " + std::to_string(age) + " value: expected " +
           std::to_string(expectedValues[age]) + ", got " + std::to_string(s.value));
    }
    checkU64Eq(s.stamp, expectedStamps[age], "age " + std::to_string(age) + " stamp");
  }

  checkTrue(ring.latest().value == 80 && ring.latest().stamp == 800,
            "latest() matches sample(0) -- the most recent publish");
}

// 2. A reader copy taken BEFORE a publish() is unchanged AFTER it --
//    immutability. Takes copies at several ages, publishes enough MORE
//    samples to physically wrap the slots those copies came from, and
//    proves every copy's fields are untouched.
void scenarioReaderCopyImmuneToLaterPublish() {
  beginScenario("a reader copy taken before publish() is unchanged after it");
  Devices::MeasurementRing<int> ring;

  for (int i = 1; i <= 5; ++i) {
    ring.publish(i * 10, static_cast<uint64_t>(i) * 100);
  }

  const Devices::Sample<int> copyLatest = ring.latest();     // value 50, stamp 500
  const Devices::Sample<int> copyAge2 = ring.sample(2);      // value 30, stamp 300
  const Devices::Sample<int> copyAge4 = ring.sample(4);      // value 10, stamp 100 (oldest)

  // Publish enough MORE samples to physically cycle every slot at least
  // once (kSlots == 6) -- if the ring mutated a published slot in place
  // instead of only ever writing the gap, this would corrupt the copies
  // above.
  for (int i = 6; i <= 14; ++i) {
    ring.publish(i * 10, static_cast<uint64_t>(i) * 100);
  }

  checkTrue(copyLatest.value == 50 && copyLatest.stamp == 500 && copyLatest.valid,
            "copy of the old latest() is unchanged after 9 further publishes");
  checkTrue(copyAge2.value == 30 && copyAge2.stamp == 300 && copyAge2.valid,
            "copy of old age-2 sample is unchanged after 9 further publishes");
  checkTrue(copyAge4.value == 10 && copyAge4.stamp == 100 && copyAge4.valid,
            "copy of old age-4 (oldest) sample is unchanged after 9 further publishes");

  // The ring itself, meanwhile, has moved on -- proving the copies really
  // were snapshots, not references into live ring state.
  checkTrue(ring.latest().value == 140 && ring.latest().stamp == 1400,
            "the ring's own latest() DID advance to the newest publish");
}

// 3. bracket() returns the correct straddling pair inside the published
//    window (including exactly on a sample's own stamp), and false outside
//    it -- both "later than everything published" and "earlier than
//    everything published" -- including on a not-yet-fully-filled ring.
void scenarioBracketFindsStraddlingPairAndFalseOutsideWindow() {
  beginScenario("bracket() finds the straddling pair, false outside the window");
  Devices::MeasurementRing<int> ring;
  for (int i = 1; i <= 5; ++i) {
    ring.publish(i * 10, static_cast<uint64_t>(i) * 100);  // stamps 100,200,300,400,500
  }

  Devices::Sample<int> older;
  Devices::Sample<int> newer;

  checkTrue(ring.bracket(250, older, newer), "t=250 is inside the published window");
  checkTrue(older.stamp == 200 && older.value == 20, "t=250's older bracket is stamp 200");
  checkTrue(newer.stamp == 300 && newer.value == 30, "t=250's newer bracket is stamp 300");

  checkTrue(ring.bracket(500, older, newer), "t=500 exactly at the newest stamp is inside (closed interval)");
  checkTrue(older.stamp == 400 && newer.stamp == 500, "t=500 brackets [400,500]");

  checkTrue(ring.bracket(100, older, newer), "t=100 exactly at the oldest stamp is inside (closed interval)");
  checkTrue(older.stamp == 100 && newer.stamp == 200, "t=100 brackets [100,200]");

  checkFalse(ring.bracket(600, older, newer), "t=600, newer than every published sample -- false");
  checkFalse(ring.bracket(50, older, newer), "t=50, older than every published sample -- false");

  // A partially-filled ring (fewer than kDepth samples published yet) must
  // not read past its own valid history -- bracket() should behave exactly
  // as if the ring only ever had that many slots.
  Devices::MeasurementRing<int> sparse;
  sparse.publish(111, 1000);
  sparse.publish(222, 2000);
  sparse.publish(333, 3000);

  checkTrue(sparse.bracket(1500, older, newer), "partially-filled ring: t inside its 3 valid samples");
  checkTrue(older.stamp == 1000 && newer.stamp == 2000, "partially-filled ring brackets [1000,2000]");
  checkFalse(sparse.bracket(500, older, newer), "partially-filled ring: t before its oldest valid sample -- false");
  checkFalse(sparse.bracket(3500, older, newer), "partially-filled ring: t after its newest sample -- false");
}

// 4. Linear lerp (lerpFraction()/lerp() -- interpolation.h) at the midpoint
//    between two bracketed samples, plus its boundary/degenerate cases.
void scenarioLinearLerpMidpoint() {
  beginScenario("linear lerp: midpoint and boundary fractions");

  const float fracMid = Devices::lerpFraction(100, 300, 200);
  checkNear(fracMid, 0.5f, 1e-6f, "t halfway between stamps 100 and 300 -> frac 0.5");
  checkNear(Devices::lerp(10.0f, 30.0f, fracMid), 20.0f, 1e-6f, "lerp(10, 30, 0.5) == 20, the midpoint value");

  checkNear(Devices::lerpFraction(100, 300, 100), 0.0f, 1e-6f, "t at olderStamp -> frac 0.0");
  checkNear(Devices::lerpFraction(100, 300, 300), 1.0f, 1e-6f, "t at newerStamp -> frac 1.0");
  checkNear(Devices::lerpFraction(100, 300, 50), 0.0f, 1e-6f, "t before olderStamp clamps to frac 0.0");
  checkNear(Devices::lerpFraction(100, 300, 400), 1.0f, 1e-6f, "t after newerStamp clamps to frac 1.0");
  checkNear(Devices::lerpFraction(200, 200, 200), 0.0f, 1e-6f, "degenerate equal stamps -> frac 0.0 (no divide-by-zero)");

  checkNear(Devices::lerp(-5.0f, 5.0f, 0.25f), -2.5f, 1e-6f, "lerp at a non-midpoint fraction");
}

// 5. Wrap-aware angular lerp (lerpAngle() -- interpolation.h) across the
//    ±180° seam interpolates the SHORT way, not the naive-linear long way.
//    This is the issue's own flagged trap and its own worked example
//    (170deg -> -170deg midpoint ~180deg, not 0deg).
void scenarioAngularLerpTakesShortWayAcrossSeam() {
  beginScenario("angular lerp crosses the +-180deg seam the short way");

  // The issue's worked example: 170deg -> -170deg. Naive linear lerp would
  // average to 0deg (the LONG way, 340deg of travel); the short way is
  // +20deg of travel through 180deg, landing the midpoint at ~180deg.
  const float older1 = 170.0f * kDegToRad;
  const float newer1 = -170.0f * kDegToRad;
  const float mid1 = Devices::lerpAngle(older1, newer1, 0.5f);
  checkNearAngleDeg(mid1, 180.0f, 0.5f, "170deg -> -170deg midpoint lands at ~180deg (short way)");
  checkTrue(std::fabs(mid1 * kRadToDeg) > 170.0f,
            "170deg -> -170deg midpoint is near +-180deg, nowhere near 0deg");

  // Symmetric case, opposite starting direction: -170deg -> 170deg. Short
  // way is the same 20deg arc, traversed the other direction; midpoint is
  // still ~180deg (the same physical angle either sign represents).
  const float mid2 = Devices::lerpAngle(-170.0f * kDegToRad, 170.0f * kDegToRad, 0.5f);
  checkNearAngleDeg(mid2, 180.0f, 0.5f, "-170deg -> 170deg midpoint also lands at ~180deg (short way)");

  // A non-wrapping case is unaffected -- lerpAngle() must reduce to
  // ordinary linear interpolation when the seam is nowhere near involved.
  const float mid3 = Devices::lerpAngle(10.0f * kDegToRad, 50.0f * kDegToRad, 0.5f);
  checkNearAngleDeg(mid3, 30.0f, 0.5f, "10deg -> 50deg midpoint is the ordinary 30deg, no wrap involved");

  // frac=0 / frac=1 endpoints reproduce the inputs exactly, even across the
  // seam.
  const float end0 = Devices::lerpAngle(older1, newer1, 0.0f);
  const float end1 = Devices::lerpAngle(older1, newer1, 1.0f);
  checkNearAngleDeg(end0, 170.0f, 0.1f, "frac=0 reproduces the older angle exactly");
  checkNearAngleDeg(end1, -170.0f, 0.1f, "frac=1 reproduces the newer angle exactly");
}

}  // namespace

int main() {
  scenarioFillAndWrapKeepsFivePublishedInOrder();
  scenarioReaderCopyImmuneToLaterPublish();
  scenarioBracketFindsStraddlingPairAndFalseOutsideWindow();
  scenarioLinearLerpMidpoint();
  scenarioAngularLerpTakesShortWayAcrossSeam();

  if (g_failureCount == 0) {
    std::printf("OK: all MeasurementRing/interpolation scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the MeasurementRing/interpolation scenarios\n",
              g_failureCount);
  return 1;
}
