// otos_odometer.h — Hal::OtosOdometer: the real-hardware Hal::Odometer leaf
// for the SparkFun Optical Tracking Odometry Sensor (OTOS), I2C address
// 0x17 (ticket 086-006).
//
// This is the real-hardware counterpart to Hal::SimOdometer (source/hal/
// sim/sim_odometer.h, 081-003) — the first CONCRETE Hal::Odometer this
// program can actually reach hardware through (Subsystems::NezhaHardware::
// odometer() has returned the base's nullptr default since sprint 081-002;
// see subsystems/hardware.h's own file header for that history).
//
// Lives at source/hal/otos/ — a NEW top-level HAL device directory,
// parallel to hal/nezha/, hal/sim/, hal/capability/ (NOT nested under
// hal/nezha/): the OTOS sensor is not a Nezha-brand device, it just happens
// to be orchestrated by the same Subsystems::NezhaHardware owner in this
// single-hardware-owner tree (see nezha_hardware.h's own file header for why
// that owner class lives where it does).
//
// Register map / read sequencing ported (concept/math, not verbatim syntax)
// from source_old/hal/real/OtosSensor.{h,cpp}:
//   0x00  PRODUCT_ID       (read; expected 0x5F)
//   0x04  LINEAR_SCALAR    (signed int8, 0.1% resolution)
//   0x05  ANGULAR_SCALAR   (signed int8, 0.1% resolution)
//   0x06  IMU_CALIBRATION  (write N to start a background bias calibration;
//                           read = samples remaining, 0 once done)
//   0x07  RESET            (bit 0: reset Kalman tracking)
//   0x0E  SIGNAL_PROCESS_CFG (LUT=0x01, Accel=0x02, Rotation=0x04, Variance=0x08)
//   0x10  OFFSET_XL        (6 bytes, same int16 LE layout as POSITION_XL)
//   0x20  POSITION_XL      (6 bytes: X_L X_H Y_L Y_H H_L H_H, signed int16 LE)
//   0x26  VELOCITY_XL      (6 bytes, same layout)
//
// 092-003 update (SUC-003, faithful SparkFun library port): REG_OFFSET
// (0x10-0x15) is now written/read by setOffset()/getOffset() — the prior
// claim above (that this register silently ACKs the write and keeps
// reading back 0 on this hardware) came from source_old/hal/real/
// OtosSensor.cpp and was never re-verified against the upstream reference
// implementation's own write path. The upstream SparkFun driver
// (sfDevOTOS::setOffset()/getOffset(), sfTk/sfDevOTOS.cpp) writes/reads
// REG_OFFSET through the EXACT SAME writePoseRegs()/readPoseRegs() helper
// and the EXACT SAME int16 scaling (kMeterToInt16/kInt16ToMeter,
// kRadToInt16/kInt16ToRad) it uses for REG_POSITION (0x20) — a register
// this leaf already writes/reads successfully (writeXYH()/setPose()). This
// ticket ports that primitive faithfully (setOffset()/getOffset() below,
// sharing writeXYH()/kPosMmPerLsb/kHdgRadPerLsb with the position path —
// see those methods' own comments). Mounting-offset (lever-arm)
// compensation was, at the time this paragraph was written, STILL applied
// HOST-SIDE via a standalone source/hal/lever_arm.h — whether this project
// would switch to chip-native REG_OFFSET compensation (retiring the
// host-side lever arm) depended on a real bench re-test of whether THIS
// chip honors the write. Ticket 004 (below) is that re-test and its
// outcome.
//
// 092-004 update (SUC-004, bench re-test + lever-arm disposition): the
// REG_OFFSET bench re-test above could NOT be run this sprint — the
// robot's serial port was held by an unrelated process and the radio relay
// was unavailable, so no bench evidence either way exists yet. Per
// architecture-update.md Decision 7, the default disposition when the
// bench cannot be run or is inconclusive is FOLD, never DELETE — deleting
// a possibly-still-needed compensation on an unconfirmed assumption risks
// a live-hardware regression (the db11b7c phantom-translation signature,
// see sensorToCentre()'s own comment below), while folding on an
// inconclusive result costs only a small amount of code a later sprint can
// clean up once the bench is achievable again. So: source/hal/lever_arm.h
// no longer exists as a standalone file — its two functions
// (LeverArm::sensorToCentre()/centreToSensor()) are folded directly into
// this class as the private sensorToCentre()/centreToSensor() methods
// below (its one production consumer, tick()/setPose()). This is a pure
// relocation, not a behavior change: the host-side compensation still
// happens exactly as before, on every tick()/setPose() call, with the
// same same-instant-heading contract. Whether THIS chip actually honors a
// REG_OFFSET write remains UNCONFIRMED — carried forward by a fresh
// clasi/issues/ follow-on (see that issue for the re-test this ticket
// deferred) rather than left unresolved.
//
// Deliberate deviation from source_old: OtosSensor::init() BLOCK-POLLED
// (fiber_sleep-based busy-wait, up to ~0.77s) for the IMU bias calibration
// to finish before returning. This tree's main loop has no fiber_sleep-style
// scheduler-yield primitive (HOST_BUILD has none at all), and blocking the
// whole dev loop — comms, motor control, telemetry — for the better part of
// a second on every OI command (not just boot) is exactly the class of
// stall sprints 078/079 spent real stand time eliminating from the Nezha
// path (see nezha_motor.cpp's requestEncoder()/writeMotorRun() comments).
// This leaf instead only WRITES REG_IMU_CALIBRATION (a fire-and-forget
// kick-off, matching the chip's own documented async behavior — source_old's
// OtosSensor.h: "Calibration runs asynchronously") and does not poll for
// completion. The chip finishes calibrating in its own background timer
// regardless of whether anything waits for it.
//
// Bus safety: reuses I2CBus's existing per-device preClear/postClear lazy-
// clearance mechanism (already generic over any 7-bit address) for every
// read/write here — no second, hand-rolled busy-wait is introduced. OTOS
// (0x17) and the Nezha motor bus (0x10, Subsystems::NezhaHardware's brick
// flip-flop) are different device-slot addresses, so this leaf's own I2C
// traffic never contends with or is scheduled by the flip-flop sequencer —
// dev_loop.cpp drives this leaf's tick()/pose() on its own, once per pass,
// entirely outside NezhaHardware::tick().
//
// 086-007 HITL fix: ticket 006 documented the above but never actually
// PASSED any preClear/postClear to bus_.write()/bus_.read() — every call
// carried the default (0, 0), i.e. no clearance at all. Combined with
// dev_loop.cpp calling tick() unconditionally every main-loop pass
// (~470 Hz observed) and tick() issuing 4 back-to-back I2C transactions
// (two readXYH() bursts) with zero settle time, this reproduced — on real
// hardware, 4/4 gdb halts during the 086 stand campaign — the exact CODAL
// NRF52I2C::waitForStop() multi-second TWIM stall that 079-006 eliminated
// from the Nezha motor path (see nezha_motor.cpp's writeMotorRun()/
// requestEncoder() comments for that precedent). When it stalls, the ENTIRE
// main loop freezes — motors, comms, and radio alike. The fix has three
// parts, all in otos_odometer.cpp: (1) every bus_.write()/bus_.read() call
// in this leaf now passes kBusClearance the same way the Nezha path does;
// (2) tick()'s former two 6-byte readXYH() bursts (position, then velocity)
// are combined into ONE 12-byte burst read (readPositionVelocity()) since
// kRegPositionXl and kRegVelocityXl are contiguous registers, halving the
// transaction count; (3) tick()'s own real bus read is rate-limited to
// kReadPeriod — see tick()'s own doc comment.
//
// Construction: takes the ticket 086-005 boot-config values
// (Config::OtosBootConfig — mounting offset + linear/angular scale
// multipliers) directly. There is no msg::-shaped equivalent (unlike
// msg::MotorConfig for Hal::NezhaMotor) because this data is deliberately
// NOT a live wire surface — see boot_config.h's own OtosBootConfig doc
// comment. Config:: has no Hal:: dependency of its own, so this one-
// directional Hal -> Config include introduces no cycle.
#pragma once

#include <stdint.h>

#include "com/i2c_bus.h"
#include "config/boot_config.h"
#include "hal/capability/odometer.h"
#include "messages/common.h"
#include "messages/odometer.h"

namespace Hal {

// 7-bit I2C address of the SparkFun OTOS chip — a different device slot
// from kNezhaDeviceAddr (0x10, nezha_motor.h), so this leaf's own I2CBus
// clearance timers never interact with the Nezha flip-flop's.
constexpr uint8_t kOtosDeviceAddr = 0x17;

class OtosOdometer : public Odometer {
 public:
  OtosOdometer(I2CBus& bus, const Config::OtosBootConfig& config);

  // Detect (PRODUCT_ID read) and, if found: enable signal processing +
  // reset Kalman tracking + kick off IMU bias calibration (this class's
  // init(), the OI primitive's effect — see file header for why this does
  // NOT block-poll for calibration completion), apply the boot-config
  // linear/angular scale multipliers (converted to the chip's raw int8
  // register scalar — scaleToRegister()), and zero the OTOS position AND
  // heading (mirrors OtosSensor::begin(): the chip retains its tracked pose
  // across a micro:bit reset/reflash, so without this the very first tick()
  // would report a stale pose against the encoders' fresh (0,0,0) origin).
  // Sets connected() accordingly; a failed product-ID detect leaves this
  // leaf permanently un-initialized (no further bus traffic — mirrors
  // source_old's is_initialized() gate) since there is nothing to recover
  // from an absent/never-detected chip.
  void begin() override;

  // Returns the cached pose computed by the most recent tick() — a cheap
  // accessor, never issues I2C traffic (mirrors Hal::SimOdometer::pose() /
  // NezhaMotor::position()'s tick()-caches-then-getters-read-cache
  // contract). Defaults to a zero pose with stamp.valid == false before the
  // first successful tick().
  msg::PoseEstimate pose() const override;

  // True once PRODUCT_ID was detected at begin() AND the most recent tick()
  // (or begin()'s own probe, before the first tick()) completed its I2C
  // burst without error.
  bool connected() const override;

  // True once (and permanently, never re-evaluated after) begin()'s
  // PRODUCT_ID detect succeeded -- unlike connected() above, which is
  // re-evaluated every tick() call from that call's own bus-read result and
  // can go false from a single transient bus glitch. present() answers "was
  // a chip ever detected at this address at all" -- the fact a CALLER
  // deciding whether to schedule this leaf a bus slot at all needs
  // (Subsystems::NezhaHardware::tick()'s scheduled-slot branch), never
  // "is the chip healthy right now" (connected()'s own job). See
  // clasi/sprints/099-restore-pose-estimation-otos-encoders-and-delayed-
  // camera-fixes/architecture-update-r1.md Decision 2 for the regression
  // this distinction fixes: gating a caller's scheduling decision on
  // connected() instead would let one transient I2C read failure on an
  // otherwise-healthy chip permanently stop it from ever being scheduled
  // again (only tick() ever re-evaluates connected_, and gating the CALL to
  // tick() on connected() would mean tick() is never called again to
  // recover it).
  bool present() const override;

  // True if a real bus read is due: either no real read has ever happened
  // (hasRead_ false, e.g. before begin(), or before begin()'s successful
  // detect on a chip that was never begin()'d/never detected -- present()
  // stays false forever in that case too, so a CALLER should always check
  // present() before readDue(), never readDue() alone -- see
  // Subsystems::NezhaHardware::tick()'s own call site), or at least
  // kReadPeriod has elapsed since the last real read (signed-cast
  // rollover-safe, matching this project's established uint32-ms-
  // subtraction convention -- e.g. motion/segment_executor.cpp's own
  // `static_cast<int32_t>(now - deadline)` idiom). A pure function of this
  // leaf's own existing hasRead_/lastReadMs_ fields (tick()'s own rate-limit
  // bookkeeping, otos_odometer.h's private section below) -- no new state,
  // and deliberately NOT itself gated on present()/initialized_ (that is
  // the caller's own, separate conjunct -- see present()'s own comment
  // above for why the two concerns stay independent). now: [ms].
  bool readDue(uint32_t now) const;

  // Burst-reads POSITION_XL and VELOCITY_XL TOGETHER in a single 12-byte I2C
  // read (086-007 — see readPositionVelocity()'s own comment: the two
  // registers are contiguous, so one read replaces the former two), applies
  // the mounting-yaw rotation (config's offsetYaw) and the lever-arm
  // compensation (this class's own private sensorToCentre(), 092-004 —
  // folded from the former standalone source/hal/lever_arm.h) using the
  // SAME-INSTANT heading from THIS burst — never a heading left over from a
  // previous tick (see sensorToCentre()'s own comment below for the
  // same-instant-heading contract; a stale heading here is the exact
  // db11b7c phantom-translation failure mode). Caches the result
  // for pose(); a burst failure holds the previously-cached pose but marks
  // it stale (stamp.valid = false) so Subsystems::PoseEstimator::tick()
  // skips fusion this pass (pose_estimator.cpp checks otosObs->stamp.valid).
  //
  // 086-007 rate limiting: dev_loop.cpp calls tick() unconditionally every
  // main-loop pass (~470 Hz observed on hardware). The OTOS does not need
  // reading anywhere near that often, and reading it that often — with the
  // per-transaction I2C clearance this fix also adds (see readReg8()/
  // readPositionVelocity()'s own comments) — would itself reintroduce a
  // main-loop cadence problem. So a REAL bus read only happens at most every
  // kReadPeriod; a tick() call that arrives sooner is a no-op on the bus and
  // marks THIS sample stale (stamp.valid = false) so PoseEstimator does not
  // re-fuse the same reading every pass (over-weighting the EKF). Every
  // tick() call that DOES perform a real read attempts it regardless of the
  // previous call's outcome — a transient bus glitch does not permanently
  // disable further attempts (mirrors Hal::NezhaMotor::tick()'s own
  // always-retry connected_ semantics). No-op entirely (no bus traffic, no
  // rate-limit bookkeeping) if begin() never detected the chip. now: [ms].
  void tick(uint32_t now) override;

  // --- Hal::Odometer's primitive setters — each a no-op if begin() never
  // detected the chip (mirrors source_old's is_initialized() guard on every
  // one of these). ---
  void init() override;                              // OI
  void resetTracking() override;                      // OR
  void setPose(const msg::Pose2D& pose) override;     // OZ (all-zero) / OV
  // OL/OA operate on the chip's raw int8 register scalar directly
  // (docs/protocol-v2.md §11 — "int8_t" register value, not a 1.0-based
  // multiplier), matching the wire contract otos_commands.cpp's handleOL/
  // handleOA already implement. The boot-config linear/angular SCALE
  // multipliers (Config::OtosBootConfig) are a different domain, converted
  // once at begin() via scaleToRegister() before being handed to these same
  // setters — see begin()'s own comment.
  void setLinearScalar(float scalar) override;        // OL
  void setAngularScalar(float scalar) override;        // OA

  // --- 092-003 (SUC-003) additions: faithful port of upstream primitives
  // beyond the Hal::Odometer virtual interface above — OTOS-specific, no
  // wire command dispatches to these yet. 092-004's bench re-test could not
  // run this sprint (see this file's own header, "092-004 update"), so
  // begin() does NOT call setOffset() — host-side compensation
  // (sensorToCentre()/centreToSensor() below) remains the live path;
  // setOffset()/getOffset() stay available, tested primitives for the
  // deferred bench re-test and any future wire surface. Each a no-op / zero
  // return if begin() never detected the chip, matching every primitive
  // above. ---

  // setOffset()/getOffset() — REG_OFFSET (0x10-0x15), the chip's own
  // mounting-offset compensation register. Shares the EXACT SAME
  // writeXYH()/kPosMmPerLsb/kHdgRadPerLsb-scaled int16 path kRegPositionXl
  // already uses (Decision 6, architecture-update.md) — upstream's own
  // sfDevOTOS::setOffset()/getOffset() write/read REG_OFFSET through the
  // identical writePoseRegs()/readPoseRegs() helper and scale constants it
  // uses for REG_POSITION (sfTk/sfDevOTOS.cpp). Deliberately NO lever-arm
  // or mounting-yaw transform here (unlike setPose()): this writes/reads
  // the mounting-offset VALUE ITSELF (config_.offsetX/offsetY/offsetYaw's
  // own domain — mm/mm/rad), not a world/chassis-centre pose that must be
  // converted THROUGH the lever arm the way setPose() converts one.
  void setOffset(const msg::Pose2D& offset);
  msg::Pose2D getOffset();

  // setSignalProcessConfig()/signalProcessConfig() — REG_SIGNAL_PROCESS_CFG
  // (0x0E) raw register value (LUT=0x01, Accel=0x02, Rotation=0x04,
  // Variance=0x08). init() already writes 0x0F (all four enabled) via this
  // same setter but, before this ticket, had no way to read the value back
  // or write anything else — upstream's own getSignalProcessConfig()/
  // setSignalProcessConfig() pair (sfDevOTOS.cpp) closes that gap.
  void setSignalProcessConfig(uint8_t config);
  uint8_t signalProcessConfig();

  // imuCalibrationSamplesRemaining() — REG_IMU_CALIBRATION (0x06)
  // read-back. init() already fire-and-forget WRITES this register to kick
  // off calibration (see file header for why it deliberately does not
  // block-poll); upstream's calibrateImu()/getImuCalibrationProgress() pair
  // splits exactly the same way — write to start, read to poll — so this
  // adds the read half without introducing any blocking wait of its own.
  // Returns the RAW register value (samples remaining until calibration
  // completes; 0 once done) — a sample count, not a physical unit, so no
  // `// [unit]` tag applies (coding-standards.md's dimensionless-fields
  // rule).
  uint8_t imuCalibrationSamplesRemaining();

 private:
  I2CBus& bus_;
  Config::OtosBootConfig config_;

  // True once PRODUCT_ID matched at begin() — gates ALL further bus traffic
  // (mirrors source_old's is_initialized()); never re-probed after begin().
  bool initialized_ = false;

  // Live per-tick bus-health flag — see tick()'s doc comment for why this
  // is retried every call rather than latching permanently false.
  bool connected_ = false;

  msg::PoseEstimate cachedPose_{};

  // 086-007 rate-limit bookkeeping — see tick()'s doc comment.
  uint32_t lastReadMs_ = 0;  // [ms] main-loop time of the most recent REAL bus read
  bool hasRead_ = false;     // true once at least one real bus read has been attempted

  // Register addresses — ported from source_old/hal/real/OtosSensor.h.
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
  // (fire-and-forget — see file header). Matches source_old's
  // kImuCalibSamples (255 samples ≈ 0.77 s at ~3 ms/sample, chip-internal).
  static constexpr uint8_t kImuCalibSamples = 255;

  // LSB scale factors — position, velocity, and (unread by this leaf)
  // acceleration all share the same 6-byte int16-triple register LAYOUT
  // (see source_old/hal/real/OtosSensor.cpp's "OTOS LSB Scale Factors"
  // comment block for the SparkFun-library derivation this restates).
  // kPosMmPerLsb/kHdgRadPerLsb below are CONFIRMED against the upstream
  // reference (sfTk/sfDevOTOS.cpp, 092-003 port pass) to equal its
  // kInt16ToMeter/kInt16ToRad EXACTLY (10 m full range / 32768, and
  // pi rad full range / 32768) — the constants setOffset()/getOffset()
  // and setPose()/tick()'s POSITION+HEADING conversions all correctly
  // share.
  //
  // 092-003 FINDING (out of THIS ticket's scope, not fixed here): upstream
  // uses a DIFFERENT LSB scale for the VELOCITY registers than for
  // position/offset -- kMpsToInt16 = 32768 / 5 (5 m/s full range), i.e.
  // kInt16ToMps = 5/32768 (~0.1526 mm/s per LSB) for linear velocity, and
  // an angular-rate scale (2000 dps full range) for omega -- roughly
  // HALF (linear) and an order of magnitude different (angular) from
  // kPosMmPerLsb/kHdgRadPerLsb. tick() below (see its own comment) applies
  // kPosMmPerLsb/kHdgRadPerLsb to the VELOCITY burst's rvx/rvy/rvh too --
  // this is a pre-existing behavior this ticket's port did NOT change
  // (Decision 6: ADD primitives, do not alter existing tick() math without
  // a dedicated, bench-verifiable ticket of its own -- a live twist-scaling
  // change is exactly the kind of hardware-behavior risk this ticket's
  // sim-only, no-bench scope cannot responsibly re-verify). Flagged here
  // for a future ticket, not silently carried forward unnoticed.
  static constexpr float kPosMmPerLsb = 0.305f;                            // [mm/LSB]
  static constexpr float kHdgRadPerLsb = 0.00549f * (3.14159265f / 180.0f); // [rad/LSB]

  // 086-007: I2C settle window for every bus_.write()/bus_.read() call this
  // leaf makes — mirrors the 079-006 Nezha-motor-path fix's value exactly
  // (nezha_motor.cpp's writeMotorRun()/requestEncoder()). See the file
  // header's "086-007 HITL fix" section and tick()'s doc comment for the
  // CODAL NRF52I2C::waitForStop() stall this eliminates.
  static constexpr uint32_t kBusClearance = 4000;  // [us]

  // 086-007: minimum spacing between real OTOS bus reads inside tick() — the
  // sensor does not need reading at dev_loop.cpp's ~470 Hz main-loop rate;
  // ~50 Hz is ample for pose fusion. See tick()'s doc comment.
  static constexpr uint32_t kReadPeriod = 20;  // [ms]

  // Convert a calibration scale multiplier (e.g. 1.05) to the chip's signed
  // int8 register scalar (0.1% per LSB), clamped to [-127, 127]. Ported
  // from OtosSensor::scaleToInt8().
  static int8_t scaleToRegister(float scale);

  // --- sensorToCentre()/centreToSensor() -- OTOS lever-arm (mounting-offset)
  // compensation math, 092-004: folded here (private, pure, stateless) from
  // the former standalone source/hal/lever_arm.h -- this class was always
  // its one production consumer (tick()/setPose()), and the standalone
  // file's original "future shared sim leaf" rationale never materialized
  // (YAGNI). This is a pure relocation, NOT a behavior change: the same two
  // formulas, called from the same two call sites, with the same
  // same-instant-heading contract below. See otos_odometer_harness.cpp for
  // this math's dedicated coverage (folded from the former
  // lever_arm_harness.cpp/test_lever_arm.py, since these are now private
  // and no longer independently unit-testable from outside this class).
  //
  // The chip reports the SENSOR's own pose (its physical position on the
  // chassis, offset from the robot's centre of rotation by offsetX/
  // offsetY); these two pure functions convert between that sensor pose and
  // the chassis CENTRE pose the rest of the firmware (and the EKF) actually
  // wants:
  //
  //   sensor = centre + R(centreHeading) * offset
  //   centre = sensor  - R(sensorHeading) * offset   (exact inverse, SAME-
  //                                                     INSTANT heading --
  //                                                     see below)
  //
  // *** SAME-INSTANT-HEADING CONTRACT -- READ BEFORE CALLING ***
  // sensorToCentre()'s sensorHeading parameter MUST be the heading read in
  // the SAME I2C burst/sample as sensorX/sensorY -- never a heading left
  // over from a previous tick or a separately-fused estimate. A past
  // regression (commit db11b7c, pre-rebuild tree) produced ~433 mm of
  // phantom translation on a pure spin on hardware because the offset
  // rotation used a heading that lagged the live spin by a constant
  // ~omega*dt: the residual is a lever-arm circle proportional to spin
  // rate, invisible at rest and severe during a fast turn. Passing the
  // same-instant heading makes the arc cancel exactly, regardless of spin
  // rate. Do not reintroduce this bug -- tick() (above) and setPose()
  // (below) both already honor this; any NEW call site must too.

  // sensor -> centre. sensorX/sensorY: the sensor's own reported position
  // (already mount-yaw-rotated / upside-down-flip corrected into a world-
  // oriented frame, but NOT yet lever-arm-compensated). sensorHeading: the
  // SAME-INSTANT heading [rad] the sensor reading was taken at -- see the
  // same-instant-heading contract above. offsetX/offsetY: mounting offset
  // from the chassis centre to the sensor [mm] (config_.offsetX/offsetY).
  static void sensorToCentre(float sensorX, float sensorY, float sensorHeading,
                              float offsetX, float offsetY,
                              float& centreXOut, float& centreYOut);
  // centre -> sensor (the exact inverse of sensorToCentre() -- same rotation
  // angle, offset added instead of subtracted). centreX/centreY/
  // centreHeading: the chassis centre pose (world frame); centreHeading
  // [rad] and sensorToCentre()'s sensorHeading are the SAME value (the
  // mounting offset never affects heading, only position), so a caller
  // round-tripping through both functions passes one heading reading
  // straight through both calls. offsetX/offsetY: same mounting offset as
  // sensorToCentre(). Returns the sensor's own world-frame position at that
  // centre pose/heading.
  static void centreToSensor(float centreX, float centreY, float centreHeading,
                              float offsetX, float offsetY,
                              float& sensorXOut, float& sensorYOut);

  // Standalone register writes/reads below each carry kBusClearance
  // (086-007): the register-address write gets postClear=kBusClearance so
  // any subsequent transaction to this device waits out the settle window;
  // readReg8()'s own read additionally carries preClear=kBusClearance so it
  // waits for ITS OWN register-select write's settle before issuing (the
  // write's postClear already covers this, but the explicit preClear keeps
  // the read self-contained if ever called after some other 0x17 traffic).
  void writeReg8(uint8_t reg, uint8_t val);
  uint8_t readReg8(uint8_t reg);
  // Burst-reads all 12 bytes of the CONTIGUOUS position+velocity block
  // (kRegPositionXl, 6 bytes, immediately followed by kRegVelocityXl, 6
  // bytes) in a SINGLE I2C read — 086-007 replaces the former two separate
  // 6-byte readXYH() bursts (position, then velocity) with this one 12-byte
  // burst, halving tick()'s transaction count (and thus its clearance
  // cost). Returns true iff both the register-address write and the 12-byte
  // read succeeded.
  bool readPositionVelocity(int16_t& x, int16_t& y, int16_t& h,
                             int16_t& vx, int16_t& vy, int16_t& vh);
  // Burst-reads one plain 6-byte int16 pose-domain register triple (X_L X_H
  // Y_L Y_H H_L H_H) — 092-003's getOffset() (kRegOffsetXl) is this helper's
  // only caller. tick()'s own hot path stays on readPositionVelocity()'s
  // combined 12-byte burst (086-007, Decision 6 — untouched); this is a
  // separate, narrower helper for a register block tick() never reads.
  // Mirrors upstream's shared readPoseRegs() helper, which backs
  // getOffset()/getPosition()/getVelocity()/getAcceleration() alike
  // (sfDevOTOS.cpp) — this leaf keeps its dedicated combined-burst read for
  // the hot path and adds this one only for the new getOffset().
  bool readXYH(uint8_t startReg, int16_t& x, int16_t& y, int16_t& h);
  // Burst-writes three signed int16 to a triple-register block.
  void writeXYH(uint8_t startReg, int16_t x, int16_t y, int16_t h);
  // Shared clamp+scale+write tail for any plain int16 pose-domain register
  // triple (kRegPositionXl OR kRegOffsetXl) — setPose() lands here after
  // its own lever-arm/mounting-yaw inverse transform; setOffset() calls it
  // directly (no transform — see setOffset()'s own comment). Mirrors
  // upstream's shared writePoseRegs() helper (sfDevOTOS.cpp), which backs
  // setOffset()/setPosition() alike through one function.
  void writePoseMm(uint8_t startReg, float xF, float yF, float hF);
};

}  // namespace Hal
