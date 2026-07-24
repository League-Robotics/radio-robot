// app_fake_otos_harness.cpp -- off-hardware acceptance proof for
// App::FakeOtos (src/firm/app/fake_otos.{h,cpp}), the bench implementation
// of the Devices::Otos interface that reports the dead-reckoned Odometry
// pose + wheel-fused body twist as if it were a real OTOS chip
// (otos-fake-seam issue). Replaces the coverage the deleted
// Devices::Otos::feedSyntheticSample() scenario used to give in
// devices_otos_harness.cpp.
//
// FakeOtos needs an App::Odometry pose source and two Devices::Motor leaves
// (for velocity() -> body twist). Rather than drag in the NezhaMotor + PID +
// SimPlant + I2C-scripting stack (App::Odometry is exercised against that in
// app_odometry_harness.cpp), this harness drives a tiny StubMotor whose
// position()/velocity() the test sets directly -- isolating FakeOtos's own
// synthesis logic from motor/bus behavior.
#include <cmath>
#include <cstdio>

#include "app/fake_otos.h"
#include "app/odometry.h"
#include "devices/motor.h"
#include "kinematics/body_kinematics.h"

namespace {

int g_failureCount = 0;

void checkTrue(bool cond, const char* what) {
  if (!cond) {
    std::printf("  FAIL: %s\n", what);
    ++g_failureCount;
  }
}

void checkNear(float got, float want, float tol, const char* what) {
  if (std::fabs(got - want) > tol) {
    std::printf("  FAIL: %s (got %.6f, want %.6f)\n", what, got, want);
    ++g_failureCount;
  }
}

// StubMotor -- a Devices::Motor whose position()/velocity() the test drives
// directly. Every other method is an inert no-op: FakeOtos and Odometry read
// only position() and velocity() off a Motor.
class StubMotor : public Devices::Motor {
 public:
  void setPosition(float position) { position_ = position; }  // [mm]
  void setVelocity(float velocity) override { velocity_ = velocity; }  // [mm/s]

  float position() const override { return position_; }   // [mm]
  float velocity() const override { return velocity_; }   // [mm/s]

  // --- inert remainder of the Devices::Motor interface ---
  void begin() override {}
  void requestSample() override {}
  void setDuty(float) override {}
  void setNeutral(Devices::Neutral) override {}
  void setPidEnabled(bool) override {}
  void applyGains(const Devices::Gains&, Devices::Opt<float> = {}) override {}
  const Devices::Gains& gains() const override { return gains_; }
  bool reconfigure(const Devices::MotorConfig&) override { return true; }
  void tick(uint64_t) override {}
  float velocityTarget() const override { return velocity_; }  // [mm/s]
  float appliedDuty() const override { return 0.0f; }
  bool connected() const override { return true; }
  void resetPosition() override { position_ = 0.0f; }
  void rebaseline() override {}

 private:
  float position_ = 0.0f;   // [mm]
  float velocity_ = 0.0f;   // [mm/s]
  Devices::Gains gains_{};
};

constexpr float kTrackWidth = 128.0f;  // [mm]

// 1. Before the first tick(), a fake is already "present"/"connected" (it is
//    always there) but has no fresh sample; present()/connected() never gate
//    on a chip probe the way the real leaf does.
void scenarioAlwaysPresentFreshOnlyAfterTick() {
  std::printf("scenario: FakeOtos present/connected always true, poseFresh only after tick()\n");
  StubMotor left, right;
  App::Odometry odom(left, right, kTrackWidth);
  App::FakeOtos fake(odom, left, right, kTrackWidth);

  checkTrue(fake.present(), "present() true before any tick (a fake is always there)");
  checkTrue(fake.connected(), "connected() true before any tick");
  checkTrue(!fake.poseFresh(), "poseFresh() false before the first tick()");

  fake.tick(1000000);
  checkTrue(fake.poseFresh(), "poseFresh() true after the first tick()");
}

// 2. A straight drive: both wheels advance equally. FakeOtos.pose() must
//    mirror Odometry's integrated pose exactly, and its body twist must be
//    the SAME BodyKinematics::forward() the loop fuses for frame_.twist.
void scenarioStraightMirrorsOdometryAndTwist() {
  std::printf("scenario: straight drive -- pose mirrors Odometry, twist matches BodyKinematics::forward\n");
  StubMotor left, right;
  App::Odometry odom(left, right, kTrackWidth);
  App::FakeOtos fake(odom, left, right, kTrackWidth);

  // Both wheels advance 100 mm -> a straight 100 mm leg, heading unchanged.
  left.setPosition(100.0f);
  right.setPosition(100.0f);
  odom.integrate();

  left.setVelocity(200.0f);
  right.setVelocity(200.0f);

  float expV = 0.0f, expOmega = 0.0f;
  BodyKinematics::forward(200.0f, 200.0f, kTrackWidth, expV, expOmega);

  fake.tick(1000000);
  Devices::PoseReading r = fake.pose();
  checkNear(r.x, odom.x(), 1e-4f, "pose().x mirrors Odometry x");
  checkNear(r.y, odom.y(), 1e-4f, "pose().y mirrors Odometry y");
  checkNear(r.heading, odom.theta(), 1e-4f, "pose().heading mirrors Odometry theta");
  checkNear(r.v_x, expV, 1e-4f, "pose().v_x matches BodyKinematics::forward v");
  checkNear(r.v_y, 0.0f, 1e-4f, "pose().v_y is 0 for a differential drive");
  checkNear(r.omega, expOmega, 1e-4f, "pose().omega matches BodyKinematics::forward omega");
}

// 3. A spin: wheels counter-rotate. Heading advances, forward speed ~0, and
//    the fused omega is non-zero -- again mirroring Odometry + forward().
void scenarioSpinMirrorsOdometryAndTwist() {
  std::printf("scenario: spin -- heading mirrors Odometry, non-zero omega from forward()\n");
  StubMotor left, right;
  App::Odometry odom(left, right, kTrackWidth);
  App::FakeOtos fake(odom, left, right, kTrackWidth);

  // Left forward, right backward by equal amounts -> pure spin.
  left.setPosition(40.0f);
  right.setPosition(-40.0f);
  odom.integrate();

  left.setVelocity(100.0f);
  right.setVelocity(-100.0f);

  float expV = 0.0f, expOmega = 0.0f;
  BodyKinematics::forward(100.0f, -100.0f, kTrackWidth, expV, expOmega);

  fake.tick(1020000);
  Devices::PoseReading r = fake.pose();
  checkNear(r.heading, odom.theta(), 1e-4f, "pose().heading mirrors Odometry theta on a spin");
  checkNear(r.v_x, expV, 1e-4f, "pose().v_x matches forward() v (~0 on a pure spin)");
  checkNear(r.omega, expOmega, 1e-4f, "pose().omega matches forward() omega (non-zero on a spin)");
  checkTrue(std::fabs(r.omega) > 1e-3f, "spin produces a non-zero omega");
}

}  // namespace

int main() {
  scenarioAlwaysPresentFreshOnlyAfterTick();
  scenarioStraightMirrorsOdometryAndTwist();
  scenarioSpinMirrorsOdometryAndTwist();

  if (g_failureCount == 0) {
    std::printf("OK: all App::FakeOtos scenarios passed\n");
    return 0;
  }
  std::printf("FAILED: %d assertion(s) across the App::FakeOtos scenarios\n", g_failureCount);
  return 1;
}
