// otos.h — Devices::Otos: the internal leaf for the SparkFun Optical
// Tracking Odometry Sensor (OTOS), I2C address 0x17. The loop constructs
// and drives this leaf directly — there is no separate handle class.
//
// Notable behaviors this leaf preserves:
//   - Fire-and-forget IMU-calibration kickoff — init() WRITES
//     REG_IMU_CALIBRATION and returns; it does NOT block-poll for the chip's
//     background bias calibration to finish (the chip finishes on its own
//     background timer regardless of whether anything waits for it).
//   - The combined 12-byte POSITION_XL+VELOCITY_XL burst read
//     (readPositionVelocity()) — the two register blocks are contiguous, so
//     one 12-byte read replaces two separate 6-byte bursts, halving the
//     leaf's per-tick I2C transaction count.
//   - Per-transaction clearance (kBusClearance, [us]) on every bus_.write()/
//     bus_.read() call this leaf makes — the same lazy preClear/postClear
//     discipline nezha_motor.cpp's writeMotorRun()/requestEncoder() use, and
//     for the identical reason (the nRF52 TWIM NRF52I2C::waitForStop()
//     multi-second stall a zero-clearance burst reproduces).
//   - Same-instant-heading lever-arm compensation (sensorToCentre()/
//     centreToSensor()) — see those methods' own comments below for the
//     phantom-translation regression this contract prevents.
//   - Wrap-aware heading: the chip's HEADING register is a signed int16
//     whose full range maps to exactly (-pi, +pi] (kHdgRadPerLsb below) — the
//     hardware itself wraps at the same point radians do, so every heading
//     this leaf ever reports is already in wrap-safe range for angular-lerp
//     interpolation to consume later; no separate unwrap step is needed or
//     added here.
//
// present()/connected() distinction (see present()'s own doc comment
// below): present() is a permanent, boot-time-only flag set once by
// begin()'s product-ID detect and never reassigned; connected() is the
// live, per-tick bus-health result, retried every tick() regardless of a
// prior failure. A caller deciding whether to schedule this leaf a bus slot
// at all must use present(), never connected() — gating scheduling on
// connected() would let one transient I2C glitch permanently stop an
// otherwise-healthy chip from ever being read again.
//
// Staged setPose() re-anchor: setPose() below stages an (x, y, heading)
// request and touches no bus; tick() drains it at the top of its next call
// (see tick()'s own comment) — the loop's cycle is the only caller of
// tick(), so "drained at a safe slot" falls directly out of tick()'s own
// call order.
//
// Scope changes from a full odometer abstraction (isolation-invariant
// driven, mirrors nezha_motor.h's own precedent):
//   - No msg::Pose2D-typed parameters (setPose()/setOffset()/getOffset()) —
//     plain (x, y, heading) float triples instead. msg:: is unreachable
//     under the isolation invariant, and a Devices-local "Pose2D" struct
//     would add a type this leaf's own callers don't need (PoseReading
//     already covers the one place a pose+twist STRUCT is actually useful —
//     the published reading itself).
//   - Devices::OtosConfig (device_config.h), not Config::OtosBootConfig —
//     identical fields (offsetX/offsetY/offsetYaw/linearScale/
//     angularScale), Devices-local so this leaf never includes config/.
//   - No freshness/valid bit on PoseReading itself (device_types.h's own
//     file header: that scaffolding belongs one level up, in a ring's own
//     Sample<T> wrapper). This leaf still needs to say "the pose I'm
//     holding right now was NOT refreshed by this tick() call" — the
//     rate-limit-skip and burst-failure cases both need it — so it is
//     carried at the leaf level as poseFresh() below instead of a struct
//     field.
//   - readDue()/tick() take a [us] uint64_t nowUs parameter (Devices::Clock
//     is THE fiber-level time seam; see clock.h) rather than a [ms]
//     uint32_t plus an internal clock read. kReadPeriod is expressed in
//     [us] (20000). A [us] uint64_t clock does not wrap on any timescale
//     this firmware will ever run, so a plain unsigned subtraction is exact
//     and simpler than a rollover-safe signed-cast subtraction would be —
//     a deliberate, documented simplification, not an oversight.
#pragma once

#include <cstdint>

#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/i2c_bus.h"

namespace Devices {

// 7-bit I2C address of the SparkFun OTOS chip — a different device slot from
// kNezhaDeviceAddr (0x10, nezha_motor.h), so this leaf's own I2CBus
// clearance timers never interact with the motor leaves'.
constexpr uint8_t kOtosDeviceAddr = 0x17;

class Otos {
 public:
  Otos(I2CBus& bus, const OtosConfig& config);

  // Detect (PRODUCT_ID read) and, if found: enable signal processing +
  // reset Kalman tracking + kick off IMU bias calibration (init(), below —
  // fire-and-forget, see file header), apply the config's linear/angular
  // scale multipliers (converted to the chip's raw int8 register scalar),
  // and zero the OTOS position AND heading (the chip retains its tracked
  // pose across a micro:bit reset/reflash, so without this the very first
  // tick() would report a stale pose against the encoders' fresh (0,0,0)
  // origin). Sets present()/connected() accordingly; a failed product-ID
  // detect leaves this leaf permanently un-initialized — no further bus
  // traffic from any method below.
  void begin();

  // Returns the cached pose computed by the most recent successful tick()
  // read (or a staged setPose() drain — see setPose()'s own comment) — a
  // cheap accessor, never issues I2C traffic. Defaults to a zero PoseReading
  // before the first successful tick().
  PoseReading pose() const;

  // True iff pose() reflects a sample this leaf actually refreshed on the
  // MOST RECENT tick() call — false when that call was rate-limited
  // (readDue() was false), drained a staged setPose() instead of reading, or
  // its burst read failed. See file header's "Scope changes" note for why
  // PoseReading itself carries no such bit yet.
  bool poseFresh() const;

  // True once PRODUCT_ID was detected at begin() AND the most recent tick()
  // (or begin()'s own probe) completed its I2C burst without error. Live,
  // per-tick — see present()'s own comment for the distinction.
  bool connected() const;

  // True once (and permanently, never re-evaluated after) begin()'s
  // PRODUCT_ID detect succeeded — independent of connected()'s live,
  // per-tick health. Answers "was a chip ever detected at this address at
  // all", the question a caller deciding whether to give this leaf a bus
  // slot at all needs — gating that decision on connected() instead would
  // let one transient I2C read failure on an otherwise-healthy chip
  // permanently stop it from ever being scheduled again.
  bool present() const;

  // The last product-ID byte begin()'s probe read (bench triage of a
  // connected=False condition): 0x5F when the OTOS answered correctly, 0xFF
  // on a floating/NAK'd bus, some other value for a wrong device.
  uint8_t lastProbeId() const { return lastProbeId_; }

  // lastReadUs -- the timestamp (Devices::Clock's own [us] epoch) of the
  // most recent REAL bus read this leaf performed -- 0 before the first one
  // (matches hasRead_'s own default). A cheap accessor, no bus traffic.
  // "age" is `now - lastReadUs()`, NOT "cycles since sample() last ran" (a
  // caller that only gets scheduled every other cycle, or a rate-limited-
  // skip tick(), must still see the REAL elapsed time since the chip's own
  // reading was actually taken). App::HeadingSource -- 109-010's own
  // measurement-age projection consumer this accessor was added for -- is
  // DELETED (115-002, gut-to-minimal-firmware S1 motion-stack excision);
  // no live App:: consumer remains, but the accessor stays (exercised by
  // app_robot_loop_harness.cpp and useful for future bench diagnostics).
  uint64_t lastReadUs() const { return lastReadUs_; }

  // True if a real bus read is due: either no real read has ever happened
  // (hasRead_ false — before begin(), or on a chip begin() never detected,
  // in which case this stays true forever too — a caller must always check
  // present() before scheduling on readDue(), never readDue() alone), or at
  // least kReadPeriod [us] has elapsed since the last real read. A pure
  // function of this leaf's own hasRead_/lastReadUs_ bookkeeping — no bus
  // traffic, deliberately NOT itself gated on present()/initialized_ (that
  // is the caller's own, separate conjunct).
  bool readDue(uint64_t nowUs) const;  // [us]

  // The leaf's one bus-touching entry point, called once per cycle by the
  // loop. No-op (no bus traffic, no bookkeeping) if begin() never
  // detected the chip.
  //
  // Drain order: a staged setPose() re-anchor request (if any) is applied
  // FIRST and unconditionally takes this tick's bus slot — an anchor write
  // immediately followed by a read of the chip's own not-yet-settled
  // registers would be worse than just deferring the read one more cycle.
  // poseFresh() is false after a drain (no read happened) and pose() is
  // left unchanged until a later tick's read actually confirms the new
  // anchor.
  //
  // Otherwise: rate-limited to at most one real read every kReadPeriod
  // (readDue()) — a tick() call that arrives sooner performs zero bus
  // traffic and marks THIS sample stale (poseFresh() false) rather than
  // re-publishing the same reading. A due call burst-reads
  // POSITION_XL+VELOCITY_XL together (readPositionVelocity()), applies the
  // mounting-yaw rotation then the lever-arm compensation using the
  // SAME-INSTANT heading from THIS burst (see sensorToCentre()'s own
  // comment), and caches the result. A burst failure holds the previously
  // cached pose but marks it stale — always-retried next tick(), never
  // permanently latched.
  void tick(uint64_t nowUs);  // [us]

  // Stages an (x, y, heading) re-anchor request; touches no bus. Drained by
  // the next tick() call (see tick()'s own "Drain order" comment above).
  // Used to anchor the OTOS to an external fix (e.g. a camera observation)
  // so its absolute readings agree with the controller pose instead of
  // dragging against the boot frame. Safe to call before begin()/before
  // present() — the drain in tick() is itself a no-op on an uninitialized
  // chip, exactly like every other primitive below.
  void setPose(float x, float y, float heading);  // [mm] [mm] [rad]

  // --- Remaining primitive setters/getters — each a no-op (zero return
  // where applicable) if begin() never detected the chip, mirroring every
  // primitive above. Unlike setVelocity()-style staged setters, these
  // issue their write immediately (not staged) — this matches the OI/OR/
  // OL/OA wire-command shape (wiring any of these to a live command is a
  // separate concern). ---

  void resetTracking();  // OR — reset Kalman tracking

  // OL/OA operate on the chip's raw int8 register scalar directly (a -127..
  // 127, 0.1%-per-LSB value — docs/protocol-v2.md's OL/OA wire contract),
  // NOT the OtosConfig linear/angular SCALE multipliers begin() converts
  // once via scaleToRegister() before handing them to these same setters.
  void setLinearScalar(float scalar);   // OL
  void setAngularScalar(float scalar);  // OA

  // setOffset()/getOffset() — REG_OFFSET (0x10-0x15), the chip's own
  // mounting-offset compensation register. Shares the exact same
  // writeXYH()/kPosMmPerLsb/kHdgRadPerLsb-scaled int16 path kRegPositionXl
  // already uses. Deliberately NO lever-arm or mounting-yaw transform here
  // (unlike setPose()): this writes/reads the mounting-offset VALUE ITSELF
  // (config_.offsetX/offsetY/offsetYaw's own domain), not a world/chassis-
  // centre pose that must be converted THROUGH the lever arm the way
  // setPose() converts one.
  // Not `const` — getOffset()/signalProcessConfig()/
  // imuCalibrationSamplesRemaining() all issue a real I2C read as an
  // externally-visible side effect (bus traffic, txnCount()), so they are
  // read ACCESSORS in the "getter" sense but not in the const-method sense.
  void setOffset(float x, float y, float heading);       // [mm] [mm] [rad]
  void getOffset(float& x, float& y, float& heading);    // [mm] [mm] [rad]

  // REG_SIGNAL_PROCESS_CFG (0x0E) raw register value (LUT=0x01, Accel=0x02,
  // Rotation=0x04, Variance=0x08). init() already writes 0x0F (all four
  // enabled) via this same setter.
  void setSignalProcessConfig(uint8_t config);
  uint8_t signalProcessConfig();

  // REG_IMU_CALIBRATION (0x06) read-back — the samples-remaining counterpart
  // to init()'s fire-and-forget write (see file header). Returns the RAW
  // register value (0 once calibration is done) — a sample count, not a
  // physical unit, so no `// [unit]` tag applies.
  uint8_t imuCalibrationSamplesRemaining();

  // OI — enable all signal processing, reset Kalman tracking, and
  // fire-and-forget kick off IMU bias calibration. Public (not just
  // begin()'s private helper) so a later wire command can re-trigger it
  // without a full begin(), mirroring the pre-port file's own OI primitive.
  void init();

 private:
  I2CBus& bus_;
  OtosConfig config_;

  // True once PRODUCT_ID matched at begin() — gates ALL further bus traffic;
  // never re-probed after begin() (present()'s backing field).
  bool initialized_ = false;

  // Live per-tick bus-health flag (connected()'s backing field) — see
  // connected()'s own comment for why this is retried every tick() rather
  // than latching permanently false.
  bool connected_ = false;
  uint8_t lastProbeId_ = 0;   // [101-001] last product-ID byte read by begin()

  PoseReading cachedPose_{};
  bool poseFresh_ = false;   // poseFresh()'s backing field

  // Rate-limit bookkeeping — see readDue()/tick()'s own comments.
  uint64_t lastReadUs_ = 0;  // [us] time of the most recent REAL bus read
  bool hasRead_ = false;     // true once at least one real bus read has been attempted

  // Staged setPose() re-anchor cell — see setPose()/tick()'s own comments.
  bool posePending_ = false;
  float pendingX_ = 0.0f;        // [mm]
  float pendingY_ = 0.0f;        // [mm]
  float pendingHeading_ = 0.0f;  // [rad]

  // Register addresses — ported from source_old/hal/real/OtosSensor.h via
  // otos_odometer.h.
  static constexpr uint8_t kRegProductId        = 0x00;
  static constexpr uint8_t kRegLinearScalar     = 0x04;
  static constexpr uint8_t kRegAngularScalar    = 0x05;
  static constexpr uint8_t kRegImuCalibration   = 0x06;
  static constexpr uint8_t kRegReset            = 0x07;
  static constexpr uint8_t kRegSignalProcessCfg = 0x0E;
  static constexpr uint8_t kRegOffsetXl         = 0x10;
  static constexpr uint8_t kRegPositionXl       = 0x20;
  static constexpr uint8_t kRegVelocityXl       = 0x26;

  static constexpr uint8_t kExpectedProductId = 0x5F;

  // IMU calibration sample count written to REG_IMU_CALIBRATION by init()
  // (fire-and-forget — see file header). 255 samples ~= 0.77s, chip-internal.
  static constexpr uint8_t kImuCalibSamples = 255;

  // LSB scale factors shared by every 6-byte int16-triple pose-domain
  // register block (POSITION_XL, OFFSET_XL) this leaf touches. See
  // otos_odometer.h's own historical derivation note for VELOCITY_XL's
  // pre-existing (unfixed, out of this ticket's scope) reuse of the SAME
  // constants despite the chip's velocity registers documenting a different
  // native LSB scale — carried forward unchanged (a live twist-scaling
  // change needs its own bench-verifiable ticket, not a sim-only port pass).
  static constexpr float kPosMmPerLsb = 0.305f;                             // [mm/LSB]
  static constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f);  // [rad/LSB]

  // I2C settle window for every bus_.write()/bus_.read() call this leaf
  // makes — mirrors nezha_motor.cpp's writeMotorRun()/requestEncoder() fix
  // for the identical CODAL NRF52I2C::waitForStop() TWIM stall.
  static constexpr uint32_t kBusClearance = 4000;  // [us]

  // Minimum spacing between real OTOS bus reads inside tick().
  static constexpr uint64_t kReadPeriod = 20000;  // [us]

  // Convert a calibration scale multiplier (e.g. 1.05) to the chip's signed
  // int8 register scalar (0.1% per LSB), clamped to [-127, 127].
  static int8_t scaleToRegister(float scale);

  // Applies a staged setPose() request — see tick()'s "Drain order" comment.
  void applyPendingPose();

  // --- sensorToCentre()/centreToSensor() — OTOS lever-arm (mounting-offset)
  // compensation math.
  //
  // The chip reports the SENSOR's own pose (its physical position on the
  // chassis, offset from the robot's centre of rotation by offsetX/
  // offsetY); these two pure functions convert between that sensor pose and
  // the chassis CENTRE pose the rest of the firmware actually wants:
  //
  //   sensor = centre + R(centreHeading) * offset
  //   centre = sensor  - R(sensorHeading) * offset   (exact inverse, SAME-
  //                                                     INSTANT heading)
  //
  // *** SAME-INSTANT-HEADING CONTRACT — READ BEFORE CALLING ***
  // sensorToCentre()'s sensorHeading parameter MUST be the heading read in
  // the SAME I2C burst/sample as sensorX/sensorY — never a heading left
  // over from a previous tick. A past regression (commit db11b7c,
  // pre-rebuild tree) produced ~433mm of phantom translation on a pure spin
  // on hardware because the offset rotation used a heading that lagged the
  // live spin by a constant ~omega*dt: the residual is a lever-arm circle
  // proportional to spin rate, invisible at rest and severe during a fast
  // turn. Passing the same-instant heading makes the arc cancel exactly,
  // regardless of spin rate. tick() (below) and applyPendingPose() already
  // honor this; any NEW call site must too.
  static void sensorToCentre(float sensorX, float sensorY, float sensorHeading,
                              float offsetX, float offsetY,
                              float& centreXOut, float& centreYOut);
  // centre -> sensor (the exact inverse of sensorToCentre()).
  static void centreToSensor(float centreX, float centreY, float centreHeading,
                              float offsetX, float offsetY,
                              float& sensorXOut, float& sensorYOut);

  // Standalone register writes/reads below each carry kBusClearance: the
  // register-address write gets postClear=kBusClearance so any subsequent
  // transaction to this device waits out the settle window; each read
  // additionally carries preClear=kBusClearance so it is self-contained even
  // if called after some other 0x17 traffic.
  void writeReg8(uint8_t reg, uint8_t val);
  uint8_t readReg8(uint8_t reg);
  // Burst-reads all 12 bytes of the CONTIGUOUS position+velocity block
  // (kRegPositionXl, 6 bytes, immediately followed by kRegVelocityXl, 6
  // bytes) in a SINGLE I2C read. Returns true iff both the register-address
  // write and the 12-byte read succeeded.
  bool readPositionVelocity(int16_t& x, int16_t& y, int16_t& h,
                             int16_t& vx, int16_t& vy, int16_t& vh);
  // Burst-reads one plain 6-byte int16 pose-domain register triple (X_L X_H
  // Y_L Y_H H_L H_H) — getOffset()'s only caller (kRegOffsetXl); tick()'s own
  // hot path stays on readPositionVelocity()'s combined 12-byte burst.
  bool readXYH(uint8_t startReg, int16_t& x, int16_t& y, int16_t& h);
  // Burst-writes three signed int16 to a triple-register block.
  void writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h);
  // Shared clamp+scale+write tail for any plain int16 pose-domain register
  // triple (kRegPositionXl OR kRegOffsetXl) — applyPendingPose() lands here
  // after its own lever-arm/mounting-yaw inverse transform; setOffset()
  // calls it directly (no transform).
  void writePoseMm(uint8_t startReg, float xF, float yF, float hF);
};

}  // namespace Devices
