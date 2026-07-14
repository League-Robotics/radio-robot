// sim_api.cpp — extern "C" C ABI wrapper over the new source/ tree's
// firmware (sprint 081-004, rewired for sprint 087 ticket 007's real
// cyclic-executive Rt::MainLoop, gutted per sprint 093's architecture-
// update.md Step 5 -- see runtime/main_loop.h). Loaded from Python via
// ctypes (host/robot_radio/io/sim_conn.py; tests/_infra/sim/firmware.py,
// ticket 081-005).
//
// SimHandle owns Subsystems::SimHardware + Subsystems::Drivetrain + one
// Rt::Blackboard, one Rt::CommandRouter, and one Rt::MainLoop -- the SAME
// Rt::MainLoop::tick(bb, now) method main.cpp calls, run here instead of
// being hand-mirrored (mirrors ticket 081-004's own "no hand-mirrored
// second copy" precedent, applied to the new transport; the 1:1-mirror
// invariant covers this shared mandatory+commit pass, not the two entry
// points below, which are sim-only testing conveniences with no equivalent
// in main.cpp -- see each one's own doc comment). There is no runtime
// config-application authority here any more (093: boot config is applied
// once, directly, at construction; no runtime SET/GET path to serve).
//
// Two reply-store instances (architecture-update.md (081) Decision 3,
// extended by architecture-update.md (088) Decision 5):
//   - syncStoreSerial / syncStoreRadio -- the ONE command's own reply during
//     sim_command()/sim_command_on(): each of Rt::CommandRouter's two reply
//     channels is wired to its OWN ReplyStore instance (088-006 -- before
//     this, BOTH channels were wired to the SAME single store, so no test
//     could prove a command dispatched/replied on RADIO specifically, only
//     that a Channel::RADIO-tagged field could be set).
//     Rt::CommandRouter::route() picks exactly one of the two reply
//     functions from the inbound command's returnPath, so only the
//     TARGETED channel's store is ever written to by a given call.
//   (093, architecture-update.md Decision 4): the third store,
//   `asyncStore` -- Rt::MainLoop's own loop-originated reply sink -- is
//   REMOVED along with the four reply-sink parameters MainLoop's
//   constructor used to take (nothing produces loop-originated output any
//   more). sim_get_async_evts() below is KEPT as a permanent no-op stub
//   (always 0 bytes) so the ctypes binding needs no matching change this
//   sprint.
//
// sim_tick(h, now) — advances real (simulated) time: ONE Rt::MainLoop::
// tick() pass (mandatory + commit). No config-drain loop remains (093:
// there is no runtime config-application authority left to drain).
//
// sim_command_on(h, line, channel, reply, size) — the dt=0 synchronous-
// command trick (Decision 4), extended (088-006) so a caller picks which
// channel (Subsystems::Channel's own enum values: 1=SERIAL, 2=RADIO) the
// command's returnPath carries, and reads the reply back from THAT
// channel's own ReplyStore (see "Two reply-store instances" above): routes
// ONE command (mirrors one slack sub-iteration with a command present),
// THEN — since a real slack window would keep spinning for ~20ms with no
// further command, ample time for anything this route() call just posted
// (bb.driveIn/…) to be drained by the NEXT mandatory tick -- replays
// Rt::MainLoop::tick() at the SAME `now` as the most recent sim_tick() call
// (SimHandle::lastTickNow). Safe ONLY because Subsystems::SimHardware::
// tick()'s own re-entry guard treats a repeated same-`now` hardware.tick()
// as a complete no-op for the PLANT integration. (093/094 teardown) There
// is no motorIn[]/motorResetIn[] drain here any more -- Hardware no longer
// receives commands through the Blackboard at all (blackboard.h's file
// header). This keeps a `sim.command("S …")` immediately followed
// by a SEPARATE `sim.command("STOP")` (no intervening sim_tick()) wire-
// observably equivalent to two real, non-zero-latency serial commands --
// exactly what ticket 006's own dt=0 trick already established as a
// sim-only convenience (this ticket extends it, it does not invent the
// pattern). Both ReplyStores are reset at the start of every call, so the
// store NOT targeted by `channel` is always left empty for this call
// (088-006's channel-isolation requirement, verified by
// tests/sim/unit/test_sim_command_channel.py).
//
// sim_command(h, line) — thin SERIAL-only wrapper over sim_command_on()
// (088-006): every pre-088-006 call site (~183 test functions across
// tests/sim/unit/) is source-compatible and behaves identically.
#include "hal/sim/sim_setters.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "runtime/blackboard.h"
#include "runtime/command_router.h"
#include "runtime/configurator.h"
#include "runtime/main_loop.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/sim_hardware.h"
#include "telemetry/telemetry_tick.h"
#include "types/clock.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace {

// ---------------------------------------------------------------------------
// ReplyStore — a fixed-size reply accumulator. Two independent instances
// live in SimHandle (see file header); this struct is intentionally the
// same shape for both. sim_conn.py's own comment documents 2048 as the
// long-standing convention for this buffer's capacity.
// ---------------------------------------------------------------------------
constexpr int kReplyBufSize = 2048;

struct ReplyStore {
    char buf[kReplyBufSize];
    int written = 0;

    void reset() { buf[0] = '\0'; written = 0; }

    void append(const char* msg) {
        if (!msg || written >= kReplyBufSize - 1) return;
        int remaining = kReplyBufSize - written - 1;
        int n = snprintf(buf + written, static_cast<size_t>(remaining), "%s\n", msg);
        if (n > 0 && n < remaining) written += n;
    }
};

void storeReply(const char* msg, void* ctx) {
    static_cast<ReplyStore*>(ctx)->append(msg);
}

// ---------------------------------------------------------------------------
// Boot configuration — sim_api.cpp cannot call Config::defaultMotorConfigs()/
// Config::defaultDrivetrainConfig() (source/config/boot_config.cpp): that
// generated file is baked from the active robot JSON and is deliberately
// ABSENT from this ticket's explicit source list. These are sane,
// self-contained sim defaults instead.
// ---------------------------------------------------------------------------
struct MotorConfigSet {
    msg::MotorConfig cfg[Subsystems::Hardware::kMotorCount];
};

MotorConfigSet defaultMotorConfigSet() {
    MotorConfigSet set;

    // Velocity-PID gains CALIBRATED TO THE SIM PLANT (2026-07-11 fix). The
    // sim plant is exactly linear: wheel velocity = duty * kNominalMaxSpeed
    // (physics_world.cpp update() sub-step A), so the exact feed-forward is
    // kff = 1/kNominalMaxSpeed -- duty = kff*|target| reproduces the target
    // 1:1 and kp/ki only clean up residuals. The previous hand-typed
    // kff = 0.0038 (a stale bench-tuned value for the REAL motor's duty
    // scale) overdrove this plant's feed-forward by 52% (0.0038 * 400 =
    // 1.52), and kp/ki were too weak to pull the error back within a
    // segment: every wheel ran ~1.22-1.28x its commanded setpoint, so every
    // pivot over-rotated ~22% (RT 90 -> ~110 deg) while the executor's own
    // emitted plan integrated to the target EXACTLY. Encoder-bounded
    // STOP_DISTANCE masked the same overdrive on translate legs.
    msg::Gains velGains;
    velGains.kp = 0.0005f;
    velGains.ki = 0.0005f;
    velGains.kff = 1.0f / Hal::PhysicsWorld::kNominalMaxSpeed;   // = 0.0025
    velGains.i_max = 0.3f;

    for (uint32_t i = 0; i < Subsystems::Hardware::kMotorCount; ++i) {
        set.cfg[i] = msg::MotorConfig();
        set.cfg[i].setPort(i + 1);
        set.cfg[i].setFwdSign(1);
        set.cfg[i].setVelGains(velGains);
        set.cfg[i].setVelFiltAlpha(1.0f);
        // 091-002: I2C flip-flop poll-schedule membership -- true for the
        // drive pair (indices 0/1, physical ports 1/2, matching
        // defaultSimDrivetrainConfig()'s left_port=1/right_port=2), false
        // otherwise. Subsystems::SimHardware itself ignores this (it ticks
        // all four motors every pass unconditionally -- sim_hardware.h's
        // own file header), but text_channel.cpp's DEV DUTY/VEL/POS `ERR nodev`
        // gate reads bb.motorConfig[idx].polled regardless of which
        // Hardware owner is behind it -- this is the config every
        // pytest-collected sim test actually runs against, so getting it
        // right here keeps every existing `DEV M 1|2 DUTY/VEL/POS` sim test
        // passing unchanged.
        set.cfg[i].setPolled(i == 0 || i == 1);
    }
    return set;
}

msg::DrivetrainConfig defaultSimDrivetrainConfig() {
    msg::DrivetrainConfig cfg;
    cfg.setTrackwidth(Hal::PhysicsWorld::kDefaultTrackwidth);   // [mm]
    cfg.setLeftPort(1);
    cfg.setRightPort(2);
    return cfg;
}

// defaultSimMotionConfig -- 094-005: mirrors source/main.cpp's own
// defaultMotionConfig() exactly (no Config::defaultPlannerConfig()
// generator exists for either build -- sim_api.cpp cannot call
// Config::defaultMotorConfigs()/Config::defaultDrivetrainConfig() either,
// see defaultMotorConfigSet()'s own comment above). Applied once, at
// construction, via drivetrain.configureMotion() below.
msg::PlannerConfig defaultSimMotionConfig() {
    msg::PlannerConfig cfg;
    cfg.a_max = 800.0f;         // [mm/s^2]
    cfg.a_decel = 800.0f;       // [mm/s^2]
    // v_body_max: capped to the SIM PLANT's own capability (2026-07-11) --
    // kNominalMaxSpeed = 400 mm/s less 5% saturation headroom -- rather than
    // main.cpp's 1000. A plan that cruises above what the plant can execute
    // saturates the duty at 1.0 and silently accrues a travel deficit the
    // divergence replan then has to chase (a D 345 planned at 465 mm/s
    // landed 8-13mm short depending on replan thresholds). Like the
    // velocity-PID gains above, the motion ceiling is a PLANT-SPECIFIC
    // quantity: plans must never ask the actuator for more than it has.
    cfg.v_body_max = 380.0f;    // [mm/s] = kNominalMaxSpeed * 0.95
    cfg.yaw_rate_max = 6.0f;    // [rad/s]
    cfg.yaw_acc_max = 20.0f;    // [rad/s^2]
    cfg.j_max = 5000.0f;        // [mm/s^3]
    cfg.yaw_jerk_max = 100.0f;  // [rad/s^3]

    // Drive::Limits/tracker fields (100-007, THE CUTOVER) -- PlannerConfig
    // fields 10 (min_speed) and 15-31, plant-specific like v_body_max above
    // (not the real robot's tovez.json-measured values -- scripts/
    // gen_boot_config.py's own MIN_SPEED_DEFAULT/*_DEFAULT constants are
    // the firmware-generic starting points; sim_api.cpp cannot call the
    // generator either, see this function's own file header). vWheelMax is
    // capped well under kNominalMaxSpeed (headroom for the trim law to
    // still command a positive correction without saturating the plant);
    // trimVMax/trimOmegaMax are scaled down from tovez.json's own
    // bench-measured 120.0/2.0 to match this tighter wheel budget --
    // headroom = trimVMax + trimOmegaMax*trackwidth/2 must leave a
    // meaningfully positive wheelBudget = vWheelMax - headroom for
    // Drivetrain::plan() to ever produce Verdict::OK (drivetrain.cpp's own
    // ceiling fold) -- 60 + 1.0*64 = 124, wheelBudget = 350-124 = 226 mm/s,
    // comfortably below kNominalMaxSpeed=400. track_k_s/track_k_theta/
    // track_k_cross are the issue's own Kanayama trim-law table values,
    // unscaled (dimensionless-ish gains, not plant-capacity-dependent).
    // min_speed=10 avoids the min_speed==0.0f pivot-mode-never-fires bug
    // (see gen_boot_config.py's own MIN_SPEED_DEFAULT comment for the full
    // derivation).
    cfg.v_wheel_max = 350.0f;      // [mm/s]
    cfg.wheel_step_max = 150.0f;   // [mm/s]
    cfg.track_k_s = 2.0f;          // [1/s]
    cfg.track_k_theta = 6.0f;      // [1/s]
    cfg.track_k_cross = 1.5e-5f;   // [rad/mm^2]
    cfg.trim_v_max = 60.0f;        // [mm/s]
    cfg.trim_omega_max = 1.0f;     // [rad/s]
    cfg.min_speed = 10.0f;         // [mm/s]
    return cfg;
}

// ---------------------------------------------------------------------------
// SimHandle — one self-contained simulation instance allocated per
// sim_create() call. Member declaration order IS construction order (C++
// initializes members in declaration order regardless of the initializer
// list's own order) -- motorConfigs before hardware (which reads it),
// hardware/drivetrain/poseEstimator before configurator (Rt::Configurator's
// own constructor takes references to all three, plus reads back
// hardware.motorConfig(i) for its per-port boot seed), hardware/drivetrain
// before loop (Rt::MainLoop's own constructor takes references to the same
// two subsystems), bb before router (router's own construction is
// bb-agnostic, but bb must exist before router. setReplyChannels() runs in
// the constructor body).
//
// poseEstimator/configurator (096-004, TEST-ONLY): main.cpp/this file's own
// mandatory tick (Rt::MainLoop::tick()) still have "no runtime
// config-application authority" (093/094's own established design,
// unchanged by this ticket -- see main.cpp's matching comment and
// runtime/configurator.h's class comment, neither of which this ticket
// edits). Ticket 096-004 is the FIRST thing since then that can ever reach
// bb.configIn/bb.streamWatchdogWindowIn at all (BinaryChannel's new binary
// `config` arm) -- without a drain, bb.configIn's posted Rt::ConfigDelta
// entries would sit forever and `get` would only ever read boot-time
// defaults, so the ticket's own round-trip acceptance criterion (`config`
// then `get` on the matching target) would be untestable. Rather than
// duplicate Configurator::applyOne()'s field-masked fold logic a THIRD time
// (config_commands.cpp's ConfigCandidate and configurator.cpp's foldXXX()
// are the other two) or have BinaryChannel write bb.*Config cells directly
// (bypassing Drivetrain::configure()/Hal::Motor::configure()/
// PoseEstimator::configure() -- silently shipping a `config` arm that
// updates what `get` reports without ever reaching the simulated hardware),
// this instantiates the SAME, unmodified Rt::Configurator class
// runtime/configurator.cpp/subsystems/pose_estimator.cpp are ALREADY
// compiled into this exact shared library for (CMakeLists.txt) --
// previously linked-but-unused here, exercised end to end by its own
// separate tests/sim/unit/configurator_harness.cpp harness. This does NOT
// revive Configurator wiring in the real firmware (main.cpp is untouched);
// it only lets THIS test harness prove BinaryChannel's translation is
// correct using the real fold+apply+publish machinery, drained after every
// sim_tick()/sim_command_on() call (see drainConfig() below) -- never in
// sim_route_no_tick(), which keeps peeking bb.segmentIn/bb.replaceIn before
// any drain, unaffected by this addition.
// ---------------------------------------------------------------------------
struct SimHandle {
    MotorConfigSet motorConfigs;
    Subsystems::SimHardware hardware;
    Subsystems::Drivetrain drivetrain;
    Subsystems::PoseEstimator poseEstimator;
    Rt::Configurator configurator;
    Rt::Blackboard bb;
    Rt::CommandRouter router;
    Rt::MainLoop loop;

    ReplyStore syncStoreSerial;  // sim_command()/sim_command_on()'s SERIAL-channel reply (see file header)
    ReplyStore syncStoreRadio;   // sim_command_on()'s RADIO-channel reply (see file header)

    // [ms] the most recent `now` passed to sim_tick(); sim_command() replays
    // Rt::MainLoop::tick() at this SAME now (the dt=0 synchronous-command
    // trick, Decision 4 — see file header).
    uint32_t lastTickNow = 0;

    SimHandle();
};

SimHandle::SimHandle()
    : motorConfigs(defaultMotorConfigSet()),
      hardware(motorConfigs.cfg),
      drivetrain(hardware),   // 094-005: Drivetrain now HOLDS a Hardware& -- hardware above
                               // must (and does) construct first.
      configurator(drivetrain, poseEstimator, hardware,
                   defaultSimDrivetrainConfig(), defaultSimMotionConfig()),
      loop(hardware, drivetrain, poseEstimator)
{
    // Primes all four ports' encoders — parity with main.cpp's
    // hardware.begin() call, before the Drivetrain is configured.
    hardware.begin();
    // 099-002 (architecture-update-r1.md Decision 2): bb.otosPresent is a
    // boot-time, never-changing hardware-identity fact (blackboard.h's own
    // comment on this field) -- seeded ONCE here, immediately after
    // hardware.begin() above, mirroring main.cpp's own identical seed line.
    // Hal::Odometer::present() defaults `true` for any leaf with no real
    // boot-time detection step of its own (Hal::SimHardware's
    // Hal::SimOdometer, here) -- see odometer.h's own doc comment.
    bb.otosPresent = hardware.odometer()->present();

    msg::DrivetrainConfig dtConfig = defaultSimDrivetrainConfig();
    drivetrain.configure(dtConfig);
    // 099-008: seed poseEstimator the SAME as drivetrain immediately above --
    // a pre-existing gap (present since ticket 099-004 first constructed
    // poseEstimator here) this ticket closes: without this call,
    // EkfTiny::init() is never reached at boot, so its q/r noise matrices
    // stay at their C++ zero-default forever and EVERY EkfTiny update
    // channel silently no-ops via the numerically-singular-S safety guard.
    // Mirrors source/main.cpp's own identical fix.
    poseEstimator.configure(dtConfig);
    // 096-002: mirrors main.cpp's own one-time bb.drivetrainConfig seed
    // (see that file's own comment on this line for the full rationale) --
    // no runtime Configurator is wired here either, so without this,
    // Telemetry::tick()'s bb.drivetrainConfig.left_port/right_port-derived
    // bb.motors[] index underflows to UINT32_MAX and reads wildly out of
    // bounds the moment STREAM/SNAP first actually emits a frame.
    bb.drivetrainConfig = dtConfig;
    // 096-004 (TEST-ONLY): seeds bb.motorConfig[]/bb.plannerConfig/
    // bb.odometerConfig too (both previously always zero -- nothing else in
    // this file ever set them), from the SAME `configurator` instance the
    // new binary `config`/`get` round-trip test drains into below. Also
    // re-writes bb.drivetrainConfig with the identical value the line above
    // already set (harmless -- configurator's own boot copy is built from
    // the SAME defaultSimDrivetrainConfig() call).
    configurator.publish(bb);
    // 094-005: boot-only jerk-limit defaults for the Drivetrain-owned
    // Motion::SegmentExecutor -- mirrors main.cpp's own
    // drivetrain.configureMotion(defaultMotionConfig()) call exactly (the
    // 1:1-mirror invariant).
    drivetrain.configureMotion(defaultSimMotionConfig());

    // Rt::CommandRouter (087-006, channel-distinct since 088-006): each
    // reply channel resolves to its OWN sync store -- see file header's
    // "Two reply-store instances".
    router.setReplyChannels(storeReply, &syncStoreSerial, storeReply, &syncStoreRadio);

    // Start the host fake clock at 0, mirroring main.cpp's boot moment (093:
    // there is no watchdog left to feed here).
    Types::setHostClockNow(0);
}

// drainConfig (096-004, TEST-ONLY) -- see SimHandle's own class comment for
// the full rationale. Drains every currently-pending bb.configIn entry
// through the real (unmodified) Rt::Configurator::applyOne() -- fold,
// conditionally call the target subsystem's own configure(), publish onto
// the matching bb.*Config cell -- exactly what a future main.cpp Configurator
// wiring will do per pass, just run to exhaustion here rather than
// one-per-pass (a test harness has no per-pass CPU budget to protect).
// Also drains bb.streamWatchdogWindowIn -> bb.streamWatchdogWindow directly
// (sTimeout is NOT one of the Configurator's four targets, Open Question 4 --
// there is no live StreamingDriveWatchdog instance here to feed, so this
// mirrors only the "publish the window" half of what that class's owner
// would do, matching the now-deleted StreamingDriveWatchdog's own (git
// history, formerly commands/motion_commands.h -- 097-006 deleted the
// class outright) setWindow()/window() shape without instantiating the
// class itself).
void drainConfig(SimHandle& s) {
    while (s.configurator.pending(s.bb)) {
        s.configurator.applyOne(s.bb);
    }
    if (!s.bb.streamWatchdogWindowIn.empty()) {
        s.bb.streamWatchdogWindow = s.bb.streamWatchdogWindowIn.take();
    }
}

}  // namespace

extern "C" {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void* sim_create() {
    return new SimHandle();
}

void sim_destroy(void* h) {
    delete static_cast<SimHandle*>(h);
}

// ---------------------------------------------------------------------------
// Tick / command dispatch
// ---------------------------------------------------------------------------

// Advance the sim by one ordinary pass: one Rt::MainLoop::tick() (hardware
// tick, drivetrain tick, commit -- 094-005 deletes the former routeOutputs()
// step; Drivetrain now stages its own wheel writes directly through
// hardware's motor refs), plus tickTelemetry() (096-002's loop-owned
// periodic STREAM emission -- a no-op unless STREAM has armed
// bb.telemetryPeriod), mirroring main.cpp's own peer call exactly (the
// "hardware and sim call the identical function" invariant). The REAL
// firmware (main.cpp) still has no config-drain loop (093: no runtime
// config-application authority left to drain) -- drainConfig() below is
// TEST-ONLY (096-004, see SimHandle's own class comment).
void sim_tick(void* h, uint32_t now) {
    SimHandle* s = static_cast<SimHandle*>(h);
    Types::setHostClockNow(now);
    s->lastTickNow = now;
    s->loop.tick(s->bb, now);
    tickTelemetry(s->bb, s->router, now);
    drainConfig(*s);
}

// Dispatch one NUL-terminated command line synchronously on a caller-chosen
// channel. Copies `line` into a Subsystems::CommunicatorToCommandProcessorCommand
// (an OWNED char[256] buffer -- subsystems/wire_command.h) whose returnPath
// is set from `channel` (Subsystems::Channel's own enum values: 1=SERIAL,
// 2=RADIO -- 0=NONE is accepted too, though no test targets it and it is
// treated the same as SERIAL for reply-store selection below), routes it,
// then replays Rt::MainLoop::tick() at the SAME `now` as the most recent
// sim_tick() call (the dt=0 synchronous-command trick — see file header).
// Both ReplyStores are reset up front, so the reply is read back from the
// ONE store that matches `channel` -- the other is guaranteed empty for
// this call (see file header's "Two reply-store instances"). Returns the
// number of reply bytes written into `reply` (not counting the final NUL),
// matching sim_conn.py's ctypes.c_int expectation.
int sim_command_on(void* h, const char* line, int channel, char* reply, int size) {
    SimHandle* s = static_cast<SimHandle*>(h);

    s->syncStoreSerial.reset();
    s->syncStoreRadio.reset();

    Subsystems::CommunicatorToCommandProcessorCommand cmd;
    cmd.returnPath = static_cast<Subsystems::Channel>(channel);
    cmd.line[0] = '\0';
    if (line) {
        std::strncpy(cmd.line, line, sizeof(cmd.line) - 1);
        cmd.line[sizeof(cmd.line) - 1] = '\0';
    }

    Types::setHostClockNow(s->lastTickNow);

    // Slack-phase command ingestion (architecture-update-r1.md Reference
    // code), mirroring main.cpp's own ingest step -- 093: no watchdog left
    // to feed here.
    s->router.route(cmd, s->bb);

    // Sim-only synchronous settle (see file header): let whatever this
    // command just posted (driveIn/…) be consumed by the next mandatory
    // tick, at the unchanged `now`.
    s->loop.tick(s->bb, s->lastTickNow);
    // 096-004 (TEST-ONLY): drain any bb.configIn/bb.streamWatchdogWindowIn
    // entry the routed command just posted (BinaryChannel's new binary
    // `config` arm) -- see SimHandle's own class comment and drainConfig()'s
    // above. A same-call `get` (a SEPARATE sim_command_on()) then reads the
    // just-published bb.*Config cell.
    drainConfig(*s);

    ReplyStore& store = (cmd.returnPath == Subsystems::Channel::RADIO)
                             ? s->syncStoreRadio
                             : s->syncStoreSerial;

    int n = store.written;
    if (reply && size > 0) {
        int copy = (n < size - 1) ? n : size - 1;
        memcpy(reply, store.buf, static_cast<size_t>(copy));
        reply[copy] = '\0';
        n = copy;
    }
    store.reset();
    return n;
}

// sim_command() -- thin SERIAL-only wrapper over sim_command_on() (088-006):
// every pre-088-006 call site (~183 test functions across tests/sim/unit/)
// is source-compatible and behaves identically.
int sim_command(void* h, const char* line, char* reply, int size) {
    return sim_command_on(h, line, static_cast<int>(Subsystems::Channel::SERIAL), reply, size);
}

// ---------------------------------------------------------------------------
// sim_route_no_tick (095-007, TEST-ONLY) -- identical to sim_command_on()
// EXCEPT it skips the trailing Rt::MainLoop::tick() replay. Lets
// test_binary_channel.py peek bb.segmentIn/bb.replaceIn's raw just-posted
// Motion::Segment (via sim_peek_segment_in()/sim_peek_replace_in() below)
// BEFORE Drivetrain::tick() drains it into its own ring_/executor_ --
// proving BinaryChannel's segment/replace translation field-by-field,
// independent of any physics/timing inference. A caller that also wants
// the drive-through-tick behavior test_bare_loop_move_and_tlm.py's own
// suite exercises can still call sim_tick()/tick_for() afterward -- this
// entry point only omits the ONE tick sim_command_on() replays inline.
// ---------------------------------------------------------------------------
int sim_route_no_tick(void* h, const char* line, int channel, char* reply, int size) {
    SimHandle* s = static_cast<SimHandle*>(h);

    s->syncStoreSerial.reset();
    s->syncStoreRadio.reset();

    Subsystems::CommunicatorToCommandProcessorCommand cmd;
    cmd.returnPath = static_cast<Subsystems::Channel>(channel);
    cmd.line[0] = '\0';
    if (line) {
        std::strncpy(cmd.line, line, sizeof(cmd.line) - 1);
        cmd.line[sizeof(cmd.line) - 1] = '\0';
    }

    Types::setHostClockNow(s->lastTickNow);
    s->router.route(cmd, s->bb);

    ReplyStore& store = (cmd.returnPath == Subsystems::Channel::RADIO)
                             ? s->syncStoreRadio
                             : s->syncStoreSerial;

    int n = store.written;
    if (reply && size > 0) {
        int copy = (n < size - 1) ? n : size - 1;
        memcpy(reply, store.buf, static_cast<size_t>(copy));
        reply[copy] = '\0';
        n = copy;
    }
    store.reset();
    return n;
}

// ---------------------------------------------------------------------------
// sim_peek_segment_in (100-007, THE CUTOVER: retyped from Motion::Segment to
// Drive::Goal) -- non-destructive read of a just-ADMITTED Drive::Goal
// (bb.segmentIn's WorkQueue::peek(idx), already non-destructive by design --
// queue.h). Writes the 3 float fields into out3[] in Drive::Goal's own
// declared order (source/drive/drivetrain.h): arcLength, deltaHeading,
// exitSpeed. *presentOut is set to 1 if a Goal was found at that position,
// 0 otherwise (out3 left untouched when absent -- caller must check
// presentOut first).
// ---------------------------------------------------------------------------
void writeGoalOut(const Drive::Goal& goal, float* out3) {
    out3[0] = goal.arcLength;
    out3[1] = goal.deltaHeading;
    out3[2] = goal.exitSpeed;
}

void sim_peek_segment_in(void* h, int idx, float* out3, int* presentOut) {
    SimHandle* s = static_cast<SimHandle*>(h);
    const Drive::Goal* goal = s->bb.segmentIn.peek(static_cast<uint32_t>(idx));
    if (!goal) {
        *presentOut = 0;
        return;
    }
    writeGoalOut(*goal, out3);
    *presentOut = 1;
}

// ---------------------------------------------------------------------------
// sim_peek_replace_in (100-008: retyped from Drive::Goal to
// Rt::MoverRequest, mirroring bb.replaceIn's own retype -- see blackboard.h/
// commands.h's doc comments) -- non-destructive read of a just-posted
// Rt::MoverRequest (bb.replaceIn's Mailbox::peek(), already non-destructive
// by design -- queue.h). Writes the 3 float fields into out3[] in
// MoverRequest's own declared order (source/runtime/commands.h): v (target's
// v_x), omega (target's omega), deadman. *presentOut is set to 1 if a
// MoverRequest was found, 0 otherwise (out3 left untouched when absent --
// caller must check presentOut first).
// ---------------------------------------------------------------------------
void sim_peek_replace_in(void* h, float* out3, int* presentOut) {
    SimHandle* s = static_cast<SimHandle*>(h);
    const Rt::MoverRequest* request = s->bb.replaceIn.peek();
    if (!request) {
        *presentOut = 0;
        return;
    }
    out3[0] = request->target.v_x;
    out3[1] = request->target.omega;
    out3[2] = request->deadman;
    *presentOut = 1;
}

// ---------------------------------------------------------------------------
// sim_post_segment (100-007, THE CUTOVER: retyped from Motion::Segment to
// Drive::Goal) -- posts one Drive::Goal directly to bb.segmentIn, BYPASSING
// wire admission entirely (no admit() check, no bb.chainTail advance -- a
// test wanting to exercise real wire admission must go through
// sim_command()/sim_command_on() with a `segment`/`replace` CommandEnvelope
// instead, tests/sim/unit/_binary_envelope.py's send_segment()/
// send_replace()). Retained for tests that isolate the ADAPTER's own
// pop/plan/step behavior from admission (mirrors the pre-cutover
// 094-005/095-007 test-only precedent this function already established).
// Returns 1 if segmentIn accepted it (false only if segmentIn is already at
// its 8-slot cap), 0 otherwise.
// ---------------------------------------------------------------------------
int sim_post_segment(void* h, float arcLength, float deltaHeading, float exitSpeed) {
    SimHandle* s = static_cast<SimHandle*>(h);
    Drive::Goal goal;
    goal.arcLength = arcLength;
    goal.deltaHeading = deltaHeading;
    goal.exitSpeed = exitSpeed;
    return s->bb.segmentIn.post(goal) ? 1 : 0;
}

// ---------------------------------------------------------------------------
// sim_get_chain_tail / sim_get_last_event (100-007, THE CUTOVER, TEST-ONLY)
// -- direct bb.chainTail/bb.lastEvent peeks, mirroring sim_get_active()'s
// own "zero-cost bb-cell peek" precedent. Lets a tier-1 test observe
// ChainTail's wire-admission advance/abort-reanchor and the adapter's own
// populated EventNotify on an ABORT_* without a wire-level EVT transport
// (that lands in ticket 100-009 -- see blackboard.h's own doc comment on
// bb.lastEvent).
// ---------------------------------------------------------------------------
void sim_get_chain_tail(void* h, float* x, float* y, float* heading, float* exitSpeed,
                         float* kappa) {
    SimHandle* s = static_cast<SimHandle*>(h);
    const Drive::ChainTail& tail = s->bb.chainTail;
    *x = tail.pose.x;
    *y = tail.pose.y;
    *heading = tail.pose.h;
    *exitSpeed = tail.exitSpeed;
    *kappa = tail.kappa;
}

void sim_get_last_event(void* h, uint32_t* segSeq, int* status, float* eFinalPos,
                         float* eFinalTheta) {
    SimHandle* s = static_cast<SimHandle*>(h);
    const msg::EventNotify& evt = s->bb.lastEvent;
    *segSeq = evt.seg_seq;
    *status = static_cast<int>(evt.status);
    *eFinalPos = evt.e_final_pos;
    *eFinalTheta = evt.e_final_theta;
}

// ---------------------------------------------------------------------------
// Reply-store introspection (088-006, test-only) -- read a channel's
// CURRENT ReplyStore length WITHOUT draining or routing anything, so a test
// can call sim_command_on() on one channel and then confirm the OTHER
// channel's store is still empty: proves CommandRouter's two reply channels
// are backed by genuinely distinct ReplyStore instances, not the pre-
// 088-006 single shared sink. Not used by sim_command()/sim_command_on()
// themselves -- test support only.
// ---------------------------------------------------------------------------
int sim_get_reply_store_len(void* h, int channel) {
    SimHandle* s = static_cast<SimHandle*>(h);
    return (static_cast<Subsystems::Channel>(channel) == Subsystems::Channel::RADIO)
               ? s->syncStoreRadio.written
               : s->syncStoreSerial.written;
}

// ---------------------------------------------------------------------------
// sim_peek_reply_store (096-002, test-only) -- non-destructive read of one
// channel's CURRENT ReplyStore content, companion to
// sim_get_reply_store_len() above. Added so a test can drain the periodic
// TLM frames tickTelemetry() (telemetry/telemetry_tick.cpp) appends into
// a channel's sync store across a run of sim_tick() calls -- neither
// sim_command() nor sim_command_on() can be used for this: both reset
// (clear) BOTH stores before routing anything, which would silently wipe
// out whatever tickTelemetry() had already accumulated during the preceding
// sim_tick() calls before a test ever got to read it. Returns the number of
// bytes written into `store_out` (not counting the final NUL) -- the same
// convention sim_command()/sim_command_on() use. Does NOT reset/drain the
// store; a caller wanting a clean slate issues any sim_command()/
// sim_command_on() call afterward (which resets both stores as a side
// effect of routing).
// ---------------------------------------------------------------------------
int sim_peek_reply_store(void* h, int channel, char* store_out, int size) {
    SimHandle* s = static_cast<SimHandle*>(h);
    ReplyStore& store = (static_cast<Subsystems::Channel>(channel) == Subsystems::Channel::RADIO)
                             ? s->syncStoreRadio
                             : s->syncStoreSerial;
    int n = store.written;
    if (store_out && size > 0) {
        int copy = (n < size - 1) ? n : size - 1;
        memcpy(store_out, store.buf, static_cast<size_t>(copy));
        store_out[copy] = '\0';
        n = copy;
    }
    return n;
}

// ---------------------------------------------------------------------------
// sim_drain_reply_store (097, SimConnection binary transport -- test-only) --
// DESTRUCTIVE read of one channel's CURRENT ReplyStore content: returns
// exactly what sim_peek_reply_store() above would, then resets (clears)
// THAT ONE channel's store. Added for host/robot_radio/io/sim_conn.py's
// SimConnection.drain_binary_tlm(): neither existing accessor is enough on
// its own -- sim_peek_reply_store() is non-destructive, so a caller that
// only ever peeks lets tickTelemetry()'s periodic frames accumulate in the
// store until it silently overflows (ReplyStore::append()'s own
// once-full-every-further-append-is-a-no-op behavior, this file's own
// ReplyStore struct above); sim_command()/sim_command_on() DO reset a
// store, but only as an incidental side effect of routing an unrelated
// command, and they reset BOTH channels' stores unconditionally (this
// file's "Two reply-store instances" note), which would also wipe out
// whatever the OTHER channel had pending. This entry point resets only the
// ONE channel it drains, with no command routed at all.
// ---------------------------------------------------------------------------
int sim_drain_reply_store(void* h, int channel, char* store_out, int size) {
    SimHandle* s = static_cast<SimHandle*>(h);
    ReplyStore& store = (static_cast<Subsystems::Channel>(channel) == Subsystems::Channel::RADIO)
                             ? s->syncStoreRadio
                             : s->syncStoreSerial;
    int n = store.written;
    if (store_out && size > 0) {
        int copy = (n < size - 1) ? n : size - 1;
        memcpy(store_out, store.buf, static_cast<size_t>(copy));
        store_out[copy] = '\0';
        n = copy;
    }
    store.reset();
    return n;
}

// ---------------------------------------------------------------------------
// Async EVT access — DELIBERATE NO-OP STUB (093, architecture-update.md
// Decision 4): Rt::MainLoop no longer produces any loop-originated output
// (no watchdog/motion/telemetry EVTs left to drain -- see file header).
// Kept, not deleted, so the existing ctypes binding
// (host/robot_radio/io/sim_conn.py) needs no matching change this sprint;
// always writes 0 bytes and returns 0.
// ---------------------------------------------------------------------------
int sim_get_async_evts(void* /*h*/, char* evts_buf, int evts_len) {
    if (evts_buf && evts_len > 0) evts_buf[0] = '\0';
    return 0;
}

// ---------------------------------------------------------------------------
// Ground-truth reads — Hal::PhysicsWorld's TRUE (unslipped, unerrored)
// accumulators. Reached through Subsystems::SimHardware's concrete plant()
// accessor directly, never through the abstract Subsystems::Hardware* base.
// ---------------------------------------------------------------------------

float sim_get_true_pose_x(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().truePoseX(); }
float sim_get_true_pose_y(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().truePoseY(); }
float sim_get_true_pose_h(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().truePoseH(); }

// exact_pose — legacy aliases for the same true-pose reads.
float sim_get_exact_pose_x(void* h) { return sim_get_true_pose_x(h); }
float sim_get_exact_pose_y(void* h) { return sim_get_true_pose_y(h); }
float sim_get_exact_pose_h(void* h) { return sim_get_true_pose_h(h); }

float sim_get_true_enc_l(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().trueEncL(); }
float sim_get_true_enc_r(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().trueEncR(); }

float sim_get_true_vel_l(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().trueVelL(); }
float sim_get_true_vel_r(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().trueVelR(); }

void sim_set_true_wheel_travel(void* h, float encL, float encR) {
    static_cast<SimHandle*>(h)->hardware.plant().setTrueWheelTravel(encL, encR);
}

void sim_set_true_pose(void* h, float x, float y, float heading) {   // [mm] [mm] [rad]
    static_cast<SimHandle*>(h)->hardware.plant().setTruePose(x, y, heading);
}

// ---------------------------------------------------------------------------
// Errored-observation reads.
//
// sim_get_enc_l/r read Hal::PhysicsWorld's REPORTED accumulator (slip +
// noise + scale error already applied) — the plant's own LEFT/RIGHT channel
// abstraction, independent of which port happens to be bound to it.
//
// sim_get_vel_l/r read the two DEFAULT plant-bound Hal::SimMotor instances'
// own filtered, encoder-derived velocity() — port 1 = LEFT, port 2 = RIGHT.
//
// sim_get_pwm_l/r read the plant's own raw commanded actuator value
// ([-100, 100] — a plant channel, port-independent, like the encoder reads
// above).
// ---------------------------------------------------------------------------

float sim_get_enc_l(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().reportedEncL(); }
float sim_get_enc_r(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().reportedEncR(); }

// index 0 = LEFT, index 1 = RIGHT (the default plant-bound drive pair,
// physical ports 1/2 -- 0-based motor indices, OOP refactor).
float sim_get_vel_l(void* h) { return static_cast<SimHandle*>(h)->hardware.simMotor(0).velocity(); }
float sim_get_vel_r(void* h) { return static_cast<SimHandle*>(h)->hardware.simMotor(1).velocity(); }

// sim_get_active (097-008, TEST-ONLY) -- bb.drivetrain.busy directly,
// bypassing the telemetry wire entirely. Added when the deleted one-shot
// text TLM verb's own tests (tests/sim/unit/test_bare_loop_move_and_tlm.py)
// were re-pointed at the binary `stream` arm: most of those tests tolerate
// the one extra tick_for() pass a wire read now costs (there is no more
// dt=0 one-shot TLM -- tickTelemetry() only ever runs from a real
// sim_tick() pass), but test_pivot_completes_promptly_single_peaked polls
// "is it idle yet" on nearly every iteration of a tight per-tick loop -- an
// extra tick per read would silently double the plant's effective
// simulated time per iteration there, corrupting the single-peak/
// prompt-idle timing that test exists to verify (and the ReplyStore the
// wire path writes into is a small fixed-size buffer with no wraparound,
// so polling it every iteration over a multi-second test would also
// silently overflow and freeze it -- see _binary_envelope.py's
// read_tlm_now() for the full rationale). bb.drivetrain.busy is exactly
// the value Telemetry::tick() copies into TlmFrameInput.active
// (source/telemetry/tlm_frame.cpp) -- this is the SAME value, read
// directly, with the same zero-cost-peek posture sim_get_vel_l()/
// sim_get_enc_l() above already established for exactly this reason.
int sim_get_active(void* h) {
    return static_cast<SimHandle*>(h)->bb.drivetrain.busy ? 1 : 0;
}

float sim_get_pwm_l(void* h) {
    return static_cast<float>(static_cast<SimHandle*>(h)->hardware.plant().pwmL());
}
float sim_get_pwm_r(void* h) {
    return static_cast<float>(static_cast<SimHandle*>(h)->hardware.plant().pwmR());
}

float sim_get_otos_x(void* h) { return static_cast<SimHandle*>(h)->hardware.simOdometer().odomX(); }
float sim_get_otos_y(void* h) { return static_cast<SimHandle*>(h)->hardware.simOdometer().odomY(); }
float sim_get_otos_h(void* h) { return static_cast<SimHandle*>(h)->hardware.simOdometer().odomH(); }

// sim_get_enc_pose_x/y/h (099-008, TEST-ONLY) -- bb.encoderPose.pose direct
// peek, mirroring sim_get_active()'s "zero-cost bb-cell peek" precedent.
// Subsystems::PoseEstimator::encoderPose() is never wire-visible (encpose=
// was trimmed from Telemetry, 096-001) -- test_pose_fix_end_to_end.py needs
// a way to prove a delayed camera-fix leaves bb.encoderPose completely
// untouched (only bb.fusedPose/the EKF is corrected), which nothing on the
// wire can show.
float sim_get_enc_pose_x(void* h) { return static_cast<SimHandle*>(h)->bb.encoderPose.pose.x; }
float sim_get_enc_pose_y(void* h) { return static_cast<SimHandle*>(h)->bb.encoderPose.pose.y; }
float sim_get_enc_pose_h(void* h) { return static_cast<SimHandle*>(h)->bb.encoderPose.pose.h; }

// ---------------------------------------------------------------------------
// Error-knob setters — each forwards to EXACTLY ONE hal/sim/sim_setters.h
// free function; no knob logic is duplicated here beyond the ctypes
// marshalling itself.
// ---------------------------------------------------------------------------

void sim_set_enc_scale_error(void* h, int side, float err) {
    Hal::setSimMotorScaleError(static_cast<SimHandle*>(h)->hardware.plant(), side, err);
}

void sim_set_enc_slip(void* h, int side, float fraction) {
    Hal::setSimMotorSlip(static_cast<SimHandle*>(h)->hardware.plant(), side, fraction);
}

void sim_set_enc_noise(void* h, int side, float sigma) {
    Hal::setSimMotorNoise(static_cast<SimHandle*>(h)->hardware.plant(), side, sigma);
}

void sim_set_stiction(void* h, int side, float pwm) {
    Hal::setSimStiction(static_cast<SimHandle*>(h)->hardware.plant(), side, pwm);
}

void sim_set_motor_lag(void* h, int side, float tau) {   // [ms]
    Hal::setSimMotorLag(static_cast<SimHandle*>(h)->hardware.plant(), side, tau);
}

void sim_set_nominal_max_speed(void* h, float speed) {   // [mm/s]
    Hal::setSimNominalMaxSpeed(static_cast<SimHandle*>(h)->hardware.plant(), speed);
}

void sim_set_coulomb_friction(void* h, int side, float decel) {   // [mm/s^2]
    Hal::setSimCoulombFriction(static_cast<SimHandle*>(h)->hardware.plant(), side, decel);
}

void sim_set_trackwidth(void* h, float trackwidth) {
    Hal::setSimTrackwidth(static_cast<SimHandle*>(h)->hardware.plant(), trackwidth);
}

void sim_set_body_rotational_scrub(void* h, float scrub) {
    Hal::setSimBodyRotationalScrub(static_cast<SimHandle*>(h)->hardware.plant(), scrub);
}

void sim_set_body_linear_scrub(void* h, float scrub) {
    Hal::setSimBodyLinearScrub(static_cast<SimHandle*>(h)->hardware.plant(), scrub);
}

void sim_set_otos_linear_noise(void* h, float sigma) {
    Hal::setSimOtosLinearNoise(static_cast<SimHandle*>(h)->hardware.simOdometer(), sigma);
}

void sim_set_otos_yaw_noise(void* h, float sigma) {
    Hal::setSimOtosYawNoise(static_cast<SimHandle*>(h)->hardware.simOdometer(), sigma);
}

void sim_set_otos_linear_scale_error(void* h, float err) {
    Hal::setSimOtosLinearScaleError(static_cast<SimHandle*>(h)->hardware.simOdometer(), err);
}

void sim_set_otos_angular_scale_error(void* h, float err) {
    Hal::setSimOtosAngularScaleError(static_cast<SimHandle*>(h)->hardware.simOdometer(), err);
}

void sim_set_otos_linear_drift(void* h, float drift) {
    Hal::setSimOtosLinearDrift(static_cast<SimHandle*>(h)->hardware.simOdometer(), drift);
}

void sim_set_otos_yaw_drift(void* h, float drift) {
    Hal::setSimOtosYawDrift(static_cast<SimHandle*>(h)->hardware.simOdometer(), drift);
}

}  // extern "C"
