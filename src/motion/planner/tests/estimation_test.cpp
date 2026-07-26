// estimation_test.cpp -- unit tests for the sense-side modules
// (estimation.h): EMA fresh-sample gating, convergence under injected
// noise, ZOH predict-to-now, and arc-exact odometry (straight and
// pure-rotation legs).
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "estimation.h"
#include "tests/test_support.h"

using Motion::PoseTracker;
using Motion::WheelChannel;

namespace {

void testFreshSampleGating() {
  WheelChannel channel;
  channel.configure(0.3f);
  channel.ingest(0.0f, 100.0f, 1000);
  CHECK(channel.valid());
  CHECK_NEAR(channel.velocity(), 100.0f, 1e-6);  // first sample seeds

  // Re-seeing the SAME sampleTime must not advance the filter, no matter
  // what values ride along (the ~80 ms refresh vs 50 ms loop case).
  channel.ingest(999.0f, 999.0f, 1000);
  CHECK_NEAR(channel.velocity(), 100.0f, 1e-6);
  CHECK_NEAR(channel.basisPosition(), 0.0f, 1e-6);

  // A fresh sample advances it by the EMA weight.
  channel.ingest(8.0f, 200.0f, 1080);
  CHECK_NEAR(channel.velocity(), 100.0f + 0.3f * 100.0f, 1e-3);
  CHECK_NEAR(channel.basisPosition(), 8.0f, 1e-6);
}

void testEmaConvergenceUnderNoise() {
  WheelChannel channel;
  channel.configure(0.2f);
  // True velocity 150 mm/s with +-40 mm/s deterministic zig-zag noise.
  uint32_t t = 0;
  float position = 0.0f;
  for (int i = 0; i < 400; ++i) {
    const float noise = (i % 2 == 0) ? 40.0f : -40.0f;
    channel.ingest(position, 150.0f + noise, t);
    t += 80;
    position += 150.0f * 0.08f;
  }
  // Converged near truth, far tighter than the raw noise band.
  CHECK_NEAR(channel.velocity(), 150.0f, 10.0f);
}

void testZohPredict() {
  WheelChannel channel;
  channel.configure(1.0f);
  channel.ingest(100.0f, 200.0f, 5000);
  // 30 ms later: 100 + 200 * 0.030 = 106 mm.
  CHECK_NEAR(channel.positionAt(5030), 106.0f, 1e-3);
  // At the basis instant: no extrapolation.
  CHECK_NEAR(channel.positionAt(5000), 100.0f, 1e-6);
}

void testOdometryStraight() {
  PoseTracker pose;
  pose.configure(100.0f);
  pose.integrate(0.0f, 0.0f);  // seed
  for (int i = 1; i <= 100; ++i) {
    pose.integrate(5.0f * i, 5.0f * i);
  }
  CHECK_NEAR(pose.x(), 500.0f, 1e-3);
  CHECK_NEAR(pose.y(), 0.0f, 1e-6);
  CHECK_NEAR(pose.heading(), 0.0f, 1e-6);
  CHECK_NEAR(pose.pathLength(), 500.0f, 1e-3);
}

void testOdometryPureRotation() {
  PoseTracker pose;
  pose.configure(100.0f);
  pose.integrate(0.0f, 0.0f);
  // Opposite wheels: quarter turn = heading pi/2, track 100 -> each wheel
  // travels pi/2 * 50 mm; path length stays zero (ds == 0 per step).
  const float wheelTravel = static_cast<float>(M_PI) * 0.5f * 50.0f;
  for (int i = 1; i <= 50; ++i) {
    const float d = wheelTravel * static_cast<float>(i) / 50.0f;
    pose.integrate(-d, d);
  }
  CHECK_NEAR(pose.heading(), M_PI * 0.5, 1e-5);
  CHECK_NEAR(pose.x(), 0.0f, 1e-4);
  CHECK_NEAR(pose.y(), 0.0f, 1e-4);
  CHECK_NEAR(pose.pathLength(), 0.0f, 1e-5);
}

void testOtosHeadingBlend() {
  PoseTracker pose;
  pose.configure(100.0f);
  pose.integrate(0.0f, 0.0f);
  pose.blendHeading(0.1f, 0.5f);
  CHECK_NEAR(pose.heading(), 0.05f, 1e-6);
  // Wrap seam: OTOS says +3.1, we say -3.1 -- residual goes the short way
  // (through pi), never the 6.2 rad long way.
  PoseTracker wrapped;
  wrapped.configure(100.0f);
  wrapped.integrate(0.0f, 0.0f);
  wrapped.reset(0.0f, 0.0f, -3.1f);
  wrapped.blendHeading(3.1f, 0.5f);
  CHECK(wrapped.heading() < -3.1f);  // moved AWAY from zero, toward -pi
}

}  // namespace

int main() {
  testFreshSampleGating();
  testEmaConvergenceUnderNoise();
  testZohPredict();
  testOdometryStraight();
  testOdometryPureRotation();
  testOtosHeadingBlend();
  std::printf("estimation_test: all checks passed\n");
  return 0;
}
