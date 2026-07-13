// device_bus_hardware.h — Subsystems::DeviceBusHardware / DeviceBusMotor /
// DeviceBusOdometer: the COMPLETE CUTOVER bridge that makes Devices::DeviceBus
// (source/devices/, the fiber-owned device subsystem) the live device layer
// of the real firmware, replacing Subsystems::NezhaHardware.
//
// Ticket 100-DBX (clasi/sprints/100-.../device-bus-cutover-ticket.md), the
// "adapter approach" that section calls for: the whole motion stack
// (Subsystems::Drivetrain, Subsystems::PoseEstimator, Rt::MainLoop) talks to
// Subsystems::Hardware / Hal::Motor / Hal::Odometer exactly as before -- only
// this file's three classes change what backs those interfaces. NezhaHardware/
// Hal::NezhaMotor/Hal::OtosOdometer are LEFT ON DISK, parked, unreferenced by
// main.cpp after the cutover (a later cleanup ticket removes them).
//
// --- Isolation note ---
// This file (and its leaves) is deliberately the ONE place that includes BOTH
// devices/*.h (the DeviceBus side) and msg::/hal/subsystems (the motion-stack
// side). It lives under source/subsystems/, NOT source/devices/, so
// test_devices_isolation.py (which guards source/devices/) is not violated --
// source/devices/ itself gains no messages:://hal:: include from this ticket.
//
// --- DeviceBusMotor: pure passthrough, no owned control state ---
// setVelocity()/setDutyCycle()/setNeutral() relay directly onto the
// Devices::Motor handle's own setters; position()/velocity()/appliedDuty()/
// connected() are plain reads of the handle's latest published sample.
// tick(now) is a NO-OP for the ordinary control path -- Devices::DeviceBus's
// own fiber (device_bus.cpp's runCycleOnce(): request -> settle -> collect ->
// PID -> armored write, once per ~16ms fiber cycle) already runs the real
// collect+PID+armored-write cycle. A leaf that ALSO ran Hal::Motor's own
// armoredWrite()/PID dispatch from ITS tick() would double-drive the motor
// against two independently-timed control loops -- the double-PID trap this
// ticket's own Verify section calls out by name. See "Known limitations"
// below for the one narrow, deliberate exception tick() takes to full no-op.
//
// PID on/off routing: Devices::NezhaMotor (the internal DeviceBus-side leaf,
// source/devices/nezha_motor.h) defaults pidEnabled_ = true and picks PID vs.
// raw duty at its OWN tick() time from that flag, not from which setter was
// last called (that leaf's own "OQ2" note). So DUTY_CYCLE mode must
// explicitly call handle_.setPidEnabled(false) before handle_.setDuty(), or
// the embedded PID keeps overwriting the staged raw duty every fiber cycle;
// VELOCITY mode explicitly re-arms setPidEnabled(true) so a motor that was
// left in DUTY mode and is now commanded VELOCITY resumes PID control.
//
// --- Known limitations (deliberate, documented -- not silently papered over)
// ---
// 1. wedged()/wedgeSuspect()/hardResetCount()/acceleration() -- Hal::Motor
//    declares these NON-VIRTUAL (capability/motor.h): they read base-owned
//    protected fields (wedgeLatched_, hardResetCount_, acceleration_) that
//    only the base's OWN updateWedgeDetector()/processResetIfPending()/
//    trackAcceleration() ever mutate, from a leaf's tick(). Since
//    DeviceBusMotor::tick() does not call updateWedgeDetector()/
//    trackAcceleration() (that would be exactly the "second armor" this
//    class must not run -- the DeviceBus-side leaf already computes its OWN,
//    correct wedge/glitch signal, reachable via handle_.wedged()/
//    handle_.wedgeSuspect()/handle_.encGlitchCount()), these NON-VIRTUAL
//    accessors structurally CANNOT be redirected to the handle's real
//    values from this leaf -- msg::MotorState.wedged/wedge_suspect/
//    acceleration report false/0.0 for a DeviceBusHardware-backed robot.
//    encGlitchCount()/sampleTime() ARE virtual and ARE correctly overridden
//    below to forward to the handle. softResetCount() DOES work correctly
//    (see limitation 2's tick() exception). A follow-up ticket that makes
//    these accessors virtual (touching every Hal::Motor leaf) would close
//    this gap; out of THIS ticket's adapter-only scope.
// 2. resetPosition() / DEV M <n> RESET -- Hal::Motor::resetPosition() is
//    also NON-VIRTUAL: it only stages resetPending_ = true; the ONLY
//    consumer is processResetIfPending(now), normally called from a leaf's
//    own tick(). A truly NO-OP tick() would make RESET commands silently
//    inert forever (resetPending_ set, never drained). DeviceBusMotor::
//    tick() therefore takes ONE narrow, deliberate exception: it calls
//    ONLY processResetIfPending(now) -- never armoredWrite()/
//    updateWedgeDetector()/updateRestTracking()/trackAcceleration(), so no
//    second PID or armor runs. Because updateRestTracking() never runs
//    either, restTicks_ stays 0 forever, so processResetIfPending() always
//    takes its "not verified at rest" branch and calls softRebaseline()
//    (never hardReset()) -- both are implemented identically below (forward
//    to handle_.resetPosition(), the DeviceBus-side leaf's own staged,
//    at-rest-guarded reset), so the hard/soft distinction is immaterial
//    here; softRebaseline() increments the base's softResetCount_, so
//    Motor::softResetCount() DOES correctly report a live count.
// 3. configureDevice() -- Devices::Motor's public handle (handles.h) exposes
//    NO live-reconfigure primitive at all (only setVelocity/setDuty/
//    setNeutral/resetPosition/setPidEnabled); the real calibration (travel
//    calib, fwd_sign, vel_gains, ...) is baked once into the DeviceBus's own
//    Devices::MotorConfig at construction (this class's own ctor, from boot
//    config). A live `DEV M <n> CFG ...` delta (Rt::Configurator's
//    kMotor path, source/runtime/configurator.cpp) reaches this leaf's
//    configureDevice() but has nowhere on the handle to go, so it is a
//    documented no-op -- NOT a regression versus NezhaHardware, whose own
//    Hardware::motorConfig() getter already only ever returns the
//    CONSTRUCTION-time snapshot regardless of any live configure() call
//    (Subsystems::NezhaHardware::motorConfigs_[] is likewise never updated
//    by a live configure()) -- see this file's own DeviceBusHardware::
//    motorConfig() for the matching precedent.
// 4. DeviceBusOdometer's init()/resetTracking()/setLinearScalar()/
//    setAngularScalar() -- the public Devices::Odometer handle (handles.h)
//    exposes exactly ONE setter, setPose(x,y,heading) -- DB-007's own scope
//    (the bring-up image's DEV grammar only ever calls "ODO SETPOSE"). The
//    other four OdometerCommand/OdometerConfig primitives have no handle
//    path to reach the DeviceBus-side Otos leaf (which DOES implement all
//    five internally) -- OI/OR/OL/OA wire commands become accepted-but-inert
//    no-ops under this cutover; only OZ/OV/SI (all routed through setPose())
//    keep working. Linear/angular scale ARE still applied once, at boot,
//    via this class's Devices::OtosConfig conversion (see
//    otosBootConfigToDeviceBus()).
// 5. Color/line sensors are OUT OF SCOPE for this ticket: Subsystems::
//    Hardware's own interface (hardware.h) has no color()/line() seam at
//    all (only motor()/odometer()) -- nothing in source/subsystems|commands
//    reads a color/line sensor today (grep confirms zero references outside
//    source/devices/). DeviceBus's own color_/line_ leaves still run inside
//    the fiber (detected, sampled, published to their own rings) but are
//    not bridged to any wire command by this ticket.
//
// --- DeviceBusOdometer: fusableThisPass()'s freshness-derived design ---
// mirrors Hal::OtosOdometer::fusableThisPass() SEMANTICS (one-shot,
// read-and-clear, single-caller-per-pass contract -- hal/capability/
// odometer.h's own doc comment) via a DIFFERENT MECHANISM: OtosOdometer
// never overrides fusableThisPass() at all -- it inherits the base's
// reset-flag bookkeeping (Odometer::apply()'s resetAppliedThisPass_,
// odometer.h) for free, and additionally rate-limits itself inside its OWN
// tick() (kReadPeriod, otos_odometer.h) so a same-reading pose() reports
// stamp.valid=false between real reads. DeviceBusOdometer's tick() is a
// NO-OP (the fiber owns all I/O, asynchronously, at its own ~48ms-per-OTOS-
// turn cadence -- device_bus.h's 3-way perception round robin), so there is
// no leaf-side tick() left to rate-limit pose() the way OtosOdometer does --
// and the main loop's own pass rate is no longer bus-bound (the old I2C
// flip-flop that throttled it is gone), so it may call fusableThisPass()
// many times between two real fiber-side OTOS publishes. Without a
// dedicated gate, PoseEstimator would re-fuse the SAME stale OTOS reading
// every one of those passes, over-weighting the EKF -- precisely the
// failure mode otos_odometer.h's own header worries about. fusableThisPass()
// below closes that gap by comparing the handle's ring publish stamp
// (updatedAt(), [us]) against the stamp last reported fusable: unchanged
// -> false (nothing new since the last fusable pass); changed -> true, and
// remember the new stamp (the "clear" half of read-and-clear). This is the
// STRUCTURAL equivalent of OtosOdometer's kReadPeriod rate limit, expressed
// against the fiber's own actual publish cadence instead of a fixed
// wall-clock window.
//
// --- async fiber vs. the synchronous Hardware::tick() contract ---
// Devices::DeviceBus's fiber runs continuously and independently on its own
// CODAL fiber, started once by DeviceBusHardware::begin() (deviceBus_.
// start(), returns immediately -- device_bus.h's own "detection retries
// never block boot" contract). Each ~20ms main-loop pass, DeviceBusHardware::
// tick() does NOT pump any bus I/O of its own (the old NezhaHardware flip-
// flop's REQUEST_DUE/COLLECT_DUE state machine is GONE) -- it is a pure
// no-op, and every Hal::Motor/Hal::Odometer read this pass sees is simply
// whatever the fiber's own last ~16ms cycle most recently published into its
// measurement rings (handles.h's own "getters serve the MOST RECENT
// PUBLISHED sample and NEVER touch the bus" contract). The old "flush-
// staged-then-collect, one-pass latency" model (Rt::MainLoop::tick()'s own
// header comment: "a setpoint staged THIS pass is flushed the FOLLOWING
// pass") LOOSENS under the cutover: a setpoint staged this pass is picked up
// by the fiber's OWN next cycle (as fast as ~16ms, independent of and
// generally FASTER than the main loop's own pass-to-pass latency), not
// deferred an entire main-loop pass. This is a documented behavior change,
// not a regression -- the fiber is faster than the main loop it replaces as
// the bus-I/O owner. The measurement rings (Devices::MeasurementRing<T>,
// single-writer/multi-reader gap-write buffers) make every handle read
// snapshot-safe across this fiber/main-loop boundary with no additional
// synchronization needed here.
#pragma once

#ifndef HOST_BUILD
#include "MicroBit.h"
#endif
#include <array>
#include <stdint.h>

#include "config/boot_config.h"
#include "devices/device_bus.h"
#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/handles.h"
#include "devices/measurement_ring.h"
#include "hal/capability/hal_command.h"
#include "hal/capability/motor.h"
#include "hal/capability/odometer.h"
#include "messages/common.h"
#include "messages/motor.h"
#include "subsystems/hardware.h"

namespace Subsystems {

// --- Conversion helpers -----------------------------------------------
// Pure functions, declared here so a host harness (tests/sim/unit/
// device_bus_hardware_harness.cpp) can exercise them directly without
// constructing a DeviceBus/DeviceBusHardware at all. No bus/fiber/CODAL
// dependency of their own -- msg::/Devices:: struct field copies only.

// Devices::MotorConfig -> msg::MotorConfig -- the Devices-local boot config
// (device_config.h) rendered back into wire/message shape for
// Hardware::motorConfig()'s getter. Inverse of msgToDeviceBusMotorConfig().
msg::MotorConfig deviceBusMotorConfigToMsg(const Devices::MotorConfig& cfg);

// msg::MotorConfig -> Devices::MotorConfig -- the boot msg::MotorConfig
// (source/config/boot_config.cpp, the SAME per-robot calibration
// NezhaHardware's own boot path already bakes) converted into the
// Devices-local shape DeviceBus's constructor requires. Inverse of
// deviceBusMotorConfigToMsg().
Devices::MotorConfig msgToDeviceBusMotorConfig(const msg::MotorConfig& cfg);

// Config::OtosBootConfig -> Devices::OtosConfig -- identical field set on
// both sides (offsetX/offsetY/offsetYaw/linearScale/angularScale); a plain
// 1:1 copy across the isolation boundary.
Devices::OtosConfig otosBootConfigToDeviceBus(const Config::OtosBootConfig& cfg);

// Devices::Sample<Devices::PoseReading> -> msg::PoseEstimate -- the
// DeviceBusOdometer::pose() conversion, factored out so a host test can feed
// a synthetic Sample<PoseReading> directly (no DeviceBus/fiber needed).
// stamp.valid mirrors the ring sample's own valid flag (false until the
// OTOS ring's first publish -- Devices::MeasurementRing<T>'s own "a freshly
// constructed ring has no history yet" contract, measurement_ring.h);
// stamp.last_upd is the ring's [us] publish stamp converted to [ms] (the
// fiber's Devices::Clock and the main loop's uBit.systemTime() share the
// same underlying CODAL microsecond clock).
msg::PoseEstimate deviceBusPoseToEstimate(const Devices::Sample<Devices::PoseReading>& sample);

// msg::Neutral -> Devices::Neutral -- the two Neutral vocabularies
// (messages/common.h vs. device_types.h; the isolation invariant forbids
// Devices:: from including messages/common.h directly, so this leaf owns
// the translation) mapped by NAME, not by underlying int value (the two
// enums do not share a numbering).
Devices::Neutral msgNeutralToDeviceBus(msg::Neutral mode);

// ---------------------------------------------------------------------------
// DeviceBusMotor -- Hal::Motor leaf, pure passthrough to a Devices::Motor
// handle. See this file's own header for the full design/limitations.
// ---------------------------------------------------------------------------
class DeviceBusMotor : public Hal::Motor {
 public:
  explicit DeviceBusMotor(Devices::Motor& handle) : handle_(handle) {}

  // --- Primitive setters (Hal::Motor) ---
  void setDutyCycle(float dutyCycle) override;   // [-1, 1]
  void setVoltage(float voltage) override;       // [V] unsupported -- capabilities().voltage == false
  void setVelocity(float velocity) override;     // [mm/s] signed
  void setPosition(float position) override;     // [deg] unsupported -- handle has no position-move primitive
  void setNeutral(msg::Neutral mode) override;
  void setFeedforward(float feedforward) override;   // [V] unsupported -- handle has no feedforward primitive

  // --- Primitive getters -- plain reads of the handle's latest published
  // sample; never touch the bus (handles.h's own contract). ---
  float position() const override;      // [mm]
  float velocity() const override;      // [mm/s] signed
  float appliedDuty() const override;   // [-1, 1]
  bool connected() const override;
  uint32_t encGlitchCount() const override;
  uint32_t sampleTime() const override;   // [ms]

  // tick() -- see this file's own header, "Known limitations" #2, for the
  // ONE narrow exception this takes to full no-op (RESET-command plumbing).
  void tick(uint32_t now) override;   // [ms]
  msg::MotorCapabilities capabilities() const override;

 protected:
  void writeRawDuty(float duty) override;   // [-1, 1] structurally unreachable (see file header); forwards defensively
  void hardReset() override;
  void softRebaseline() override;
  void configureDevice(const msg::MotorConfig& config) override;   // see file header, "Known limitations" #3

 private:
  Devices::Motor& handle_;
};

// ---------------------------------------------------------------------------
// DeviceBusOdometer -- Hal::Odometer leaf, pure passthrough to a
// Devices::Odometer handle. See this file's own header for fusableThisPass()'s
// freshness-derived design and the init()/resetTracking()/setLinearScalar()/
// setAngularScalar() limitation.
// ---------------------------------------------------------------------------
class DeviceBusOdometer : public Hal::Odometer {
 public:
  explicit DeviceBusOdometer(Devices::Odometer& handle) : handle_(handle) {}

  msg::PoseEstimate pose() const override;
  bool connected() const override;
  // present() -- NOT overridden; inherits Hal::Odometer's `true` convenience
  // default. The public Devices::Odometer handle exposes no present()-
  // equivalent signal (only connected()), and the fiber's OWN detection
  // preamble runs asynchronously in the background on real hardware (never
  // blocking begin()/boot) -- a boot-time snapshot read immediately after
  // begin() (main.cpp's bb.otosPresent seed) would almost always observe
  // "not yet detected" regardless of how this were implemented, permanently
  // suppressing TLM's otos= field for the rest of the session. Reporting
  // "a device slot is wired up" (true, unconditionally -- this class always
  // owns exactly one Otos leaf) matches Hal::SimOdometer's own precedent for
  // "no real boot-time detection step to report" leaves (odometer.h's own
  // present() doc comment).

  void tick(uint32_t now) override;   // [ms] NO-OP -- the fiber owns all I/O

  void init() override;
  void resetTracking() override;
  void setPose(const msg::Pose2D& pose) override;
  void setLinearScalar(float scalar) override;
  void setAngularScalar(float scalar) override;

  bool fusableThisPass() override;

 private:
  Devices::Odometer& handle_;
  uint64_t lastFusedStamp_ = 0;   // [us] freshness one-shot bookkeeping -- see fusableThisPass()
};

// ---------------------------------------------------------------------------
// DeviceBusHardware -- Subsystems::Hardware owner, backed by one owned
// Devices::DeviceBus. Constructor mirrors Subsystems::NezhaHardware's own
// signature as closely as the underlying bus type allows (msg::MotorConfig
// configs[kMotorCount] + Config::OtosBootConfig) so main.cpp's swap stays
// minimal -- see device_bus_hardware.cpp's own file header for the
// #ifndef HOST_BUILD split (Devices::DeviceBus's real constructor takes a
// raw MicroBitI2C&, not the project's own I2CBus wrapper NezhaHardware used).
// ---------------------------------------------------------------------------
class DeviceBusHardware : public Hardware {
 public:
#ifndef HOST_BUILD
  DeviceBusHardware(MicroBitI2C& i2c, const msg::MotorConfig configs[kMotorCount],
                     const Config::OtosBootConfig& otosConfig = Config::OtosBootConfig());
#else
  DeviceBusHardware(const msg::MotorConfig configs[kMotorCount],
                     const Config::OtosBootConfig& otosConfig = Config::OtosBootConfig());
#endif

  // Starts the DeviceBus fiber (deviceBus_.start()) -- returns immediately on
  // real hardware (detection retries run asynchronously in the background).
  void begin() override;

  // NO-OP -- the fiber owns all bus scheduling asynchronously; the old
  // NezhaHardware flip-flop's REQUEST_DUE/COLLECT_DUE pump is gone. See this
  // file's own header, "async fiber vs. the synchronous Hardware::tick()
  // contract".
  void tick(uint32_t now) override;   // [ms]

  Hal::Motor& motor(uint32_t i) override;

  void apply(const Hal::CommandProcessorToHardwareCommand& cmd) override;
  void apply(const Hal::DrivetrainToHardwareCommand& cmd) override;

  Hal::Odometer* odometer() override;

  msg::MotorConfig motorConfig(uint32_t i) const override;
  msg::MotorState motorState(uint32_t i) const override;

 private:
  static uint32_t clampIndex(uint32_t i) { return (i < kMotorCount) ? i : kMotorCount - 1; }

  // ---- Declaration order IS construction order: deviceBus_ must be fully
  // constructed before motors_/odometer_ (both hold references into it,
  // obtained via deviceBus_.motor(port)/deviceBus_.odometer()). ----
  Devices::DeviceBus deviceBus_;
  std::array<DeviceBusMotor, kMotorCount> motors_;
  DeviceBusOdometer odometer_;

  // motorConfig()'s backing store -- a verbatim copy of the constructor's
  // configs[] argument, mirroring Subsystems::NezhaHardware::motorConfigs_
  // exactly (same construction-time-only snapshot semantics -- see this
  // file's header, "Known limitations" #3). Ports 3/4 (indices 2/3) have no
  // real backing DeviceBusMotor leaf of their own (Devices::DeviceBus::
  // motor(port) resolves any port other than 2 to its FIRST channel -- an
  // already-documented DeviceBus contract, device_bus.h/.cpp), but their
  // boot msg::MotorConfig is still copied here verbatim for getter parity
  // with NezhaHardware (which reports the SAME bench-placeholder values for
  // these two unpolled, physically-absent-on-Tovez ports today).
  msg::MotorConfig motorConfigs_[kMotorCount];
};

}  // namespace Subsystems
