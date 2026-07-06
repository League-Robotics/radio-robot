// sim_api.cpp — extern "C" C ABI wrapper over the new source/ tree's
// dev-loop firmware (sprint 081-004), compiled into tests/_infra/sim/'s
// libfirmware_host by this directory's CMakeLists.txt. Loaded from Python
// via ctypes (host/robot_radio/io/sim_conn.py; tests/_infra/sim/firmware.py,
// ticket 081-005).
//
// SimHandle owns Subsystems::SimHardware + Subsystems::Drivetrain +
// CommandProcessor + source/dev_loop.h's DevLoop -- the SAME
// devLoopTick(loop, now, statement) function main.cpp calls, run here
// instead of being hand-mirrored (see dev_loop.h's file header and
// clasi/sprints/081-.../architecture-update.md's Step 2 responsibility
// table for why a second, drifting copy of the loop body is exactly the
// risk this design avoids).
//
// Two separate reply sinks (architecture-update.md (081) Decision 3):
//   - syncStore  -- the DevLoopStatement's own replyFn/replyCtx during
//     sim_command(): captures ONLY that command's own OK/ERR/… reply.
//   - asyncStore -- DevLoop's defaultReply/defaultReplyCtx, the
//     loop-originated reply sink devLoopTick() uses for output it
//     generates ITSELF rather than in response to a statement (today, only
//     the watchdog-fire `EVT dev_watchdog`) -- drained by
//     sim_get_async_evts().
//
// The dt=0 synchronous-command trick (Decision 4): sim_command() replays
// devLoopTick() at the SAME `now` as the most recent sim_tick() call,
// captured in SimHandle::lastTickNow -- never a fresh timestamp. This is
// safe ONLY because Subsystems::SimHardware::tick()'s own re-entry guard
// (ticket 003) treats a repeated same-`now` hardware.tick() as a complete
// no-op; the statement dispatch, outbox drain, and watchdog check inside
// devLoopTick() still run normally on every call.
#include "commands/command_processor.h"
#include "commands/config_commands.h"
#include "commands/dev_commands.h"
#include "commands/motion_commands.h"
#include "commands/system_commands.h"
#include "commands/telemetry_commands.h"
#include "dev_loop.h"
#include "hal/sim/sim_setters.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/planner.h"
#include "subsystems/pose_estimator.h"
#include "subsystems/sim_hardware.h"
#include "types/clock.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

// ---------------------------------------------------------------------------
// ReplyStore — a fixed-size reply accumulator. Two independent instances
// live in SimHandle (see file header); this struct is intentionally the
// same shape for both, just as tests_old/_infra/sim/sim_api.cpp's single
// combined ReplyStore was — sim_conn.py's own comment documents 2048 as the
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
// ABSENT from this ticket's explicit source list (it is a real-robot boot
// concern, not a build/ABI concern — see the CMakeLists.txt file header's
// "Absent" list). These are sane, self-contained sim defaults instead: the
// same bench-tuned velocity-PID gains boot_config.cpp's generator bakes in,
// and a trackwidth matched to Hal::PhysicsWorld's own default so the
// Drivetrain's kinematics and the plant's true chassis geometry agree with
// no artificial calibration mismatch.
// ---------------------------------------------------------------------------
struct MotorConfigSet {
    msg::MotorConfig cfg[Subsystems::Hardware::kPortCount];
};

MotorConfigSet defaultMotorConfigSet() {
    MotorConfigSet set;

    msg::Gains velGains;
    velGains.kp = 0.0022f;
    velGains.ki = 0.0018f;
    velGains.kff = 0.0038f;
    velGains.i_max = 0.3f;

    for (uint32_t i = 0; i < Subsystems::Hardware::kPortCount; ++i) {
        set.cfg[i] = msg::MotorConfig();
        set.cfg[i].setPort(i + 1);
        set.cfg[i].setFwdSign(1);
        set.cfg[i].setVelGains(velGains);
        set.cfg[i].setVelFiltAlpha(0.3f);
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

// defaultSimPlannerConfig — 084-002: mirrors source/main.cpp's own
// defaultPlannerConfig() (no Config::defaultPlannerConfig() generator exists
// for either build — see that function's doc comment); same generous,
// headroom-above-the-wire-range ramp limits so a sim test's `D`/`T`/`S`
// converges to its commanded speed in a bounded, predictable number of
// ticks instead of the ramp being clamped to zero (an all-zero, never-
// configured msg::PlannerConfig — see tests/sim/unit/planner_harness.cpp's
// own "a_max == 0 (never configured) means the ramp never leaves zero"
// scenario).
msg::PlannerConfig defaultSimPlannerConfig() {
    msg::PlannerConfig cfg;
    cfg.a_max = 800.0f;              // [mm/s^2]
    cfg.a_decel = 800.0f;            // [mm/s^2]
    cfg.v_body_max = 1000.0f;        // [mm/s]
    cfg.yaw_rate_max = 6.0f;         // [rad/s]
    cfg.yaw_acc_max = 20.0f;         // [rad/s^2]
    cfg.j_max = 0.0f;
    cfg.yaw_jerk_max = 0.0f;
    cfg.arrive_tol = 25.0f;          // [mm]
    cfg.turn_in_place_gate = 35.0f;
    cfg.min_speed = 0.0f;
    return cfg;
}

// ---------------------------------------------------------------------------
// buildAndWireCommandTable — wires DevLoopState's hardware/drivetrain/
// watchdog pointers (devCommands()'s own doc comment requires state.watchdog
// be set before it is called — DEV WD dereferences it), wires
// TelemetryState's hardware/drivetrain/poseEstimator pointers (082-004,
// telemetryCommands()'s own doc comment requires the same before any call),
// wires MotionLoopState's poseEstimator pointer (084-002, motionCommands()'s
// own doc comment requires the same before any call), and returns the full
// command table (liveness + DEV + telemetry + motion), mirroring main.cpp's
// own systemCommands()+devCommands()+telemetryCommands()+motionCommands()
// assembly exactly. Packaged as a function (rather than inline in main.cpp's
// style) so it can run from SimHandle's member-initializer list, wiring
// devState/telemetryState/motionState as a side effect at the exact point
// CommandProcessor needs the finished table.
// ---------------------------------------------------------------------------
std::vector<CommandDescriptor> buildAndWireCommandTable(
    DevLoopState& devState,
    TelemetryState& telemetryState,
    MotionLoopState& motionState,
    ConfigCommandState& configState,
    Subsystems::Hardware& hardware,
    Subsystems::Drivetrain& drivetrain,
    Subsystems::PoseEstimator& poseEstimator,
    Subsystems::Planner& planner,
    SerialSilenceWatchdog& watchdog) {
    devState.hardware = &hardware;
    devState.drivetrain = &drivetrain;
    devState.watchdog = &watchdog;

    telemetryState.hardware = &hardware;
    telemetryState.drivetrain = &drivetrain;
    telemetryState.poseEstimator = &poseEstimator;
    // 084-005: mode='s sole source (Decision 6) -- see telemetry_commands.h's
    // file header comment.
    telemetryState.planner = &planner;

    motionState.poseEstimator = &poseEstimator;

    // 084-006: SET/GET's own config-plane shadow -- an independent struct,
    // NOT devState's (architecture-update.md (084) Decision 7). Pointer
    // wiring only here; the shadow fields themselves are seeded from
    // SimHandle's own boot configs in the constructor below, mirroring
    // devState.motorConfigShadow[]/drivetrainConfigShadow's seeding.
    configState.hardware = &hardware;
    configState.drivetrain = &drivetrain;
    configState.poseEstimator = &poseEstimator;
    configState.planner = &planner;
    configState.sTimeoutWatchdog = &motionState.sTimeout;

    std::vector<CommandDescriptor> all = systemCommands();
    std::vector<CommandDescriptor> dev = devCommands(devState);
    all.insert(all.end(), dev.begin(), dev.end());
    std::vector<CommandDescriptor> telemetry = telemetryCommands(telemetryState);
    all.insert(all.end(), telemetry.begin(), telemetry.end());
    std::vector<CommandDescriptor> motion = motionCommands(motionState);
    all.insert(all.end(), motion.begin(), motion.end());
    std::vector<CommandDescriptor> config = configCommands(configState);
    all.insert(all.end(), config.begin(), config.end());
    return all;
}

// ---------------------------------------------------------------------------
// SimHandle — one self-contained simulation instance allocated per
// sim_create() call. Member declaration order IS construction order (C++
// initializes members in declaration order regardless of the initializer
// list's own order) — motorConfigs before hardware (which reads it),
// hardware/drivetrain/watchdog/devState before processor (whose
// initializer wires devState's pointers and builds the command table that
// needs them already valid).
// ---------------------------------------------------------------------------
struct SimHandle {
    MotorConfigSet motorConfigs;
    Subsystems::SimHardware hardware;
    Subsystems::Drivetrain drivetrain;
    Subsystems::PoseEstimator poseEstimator;   // 082-003: wired into loop below
    Subsystems::Planner planner;               // 084-002: wired into loop below
    SerialSilenceWatchdog watchdog;
    DevLoopState devState;
    TelemetryState telemetryState;   // 082-004: wired into loop below
    MotionLoopState motionState;     // 084-002: wired into loop below
    ConfigCommandState configState;  // 084-006: SET/GET's own config shadow
    CommandProcessor processor;
    DevLoop loop;

    ReplyStore syncStore;    // sim_command()'s synchronous reply (see file header)
    ReplyStore asyncStore;   // devLoopTick()'s loop-originated output (watchdog EVT)

    // [ms] the most recent `now` passed to sim_tick(); sim_command() replays
    // devLoopTick() at this SAME now (the dt=0 synchronous-command trick,
    // Decision 4 — see file header).
    uint32_t lastTickNow = 0;

    SimHandle();
};

SimHandle::SimHandle()
    : motorConfigs(defaultMotorConfigSet()),
      hardware(motorConfigs.cfg),
      processor(buildAndWireCommandTable(devState, telemetryState, motionState, configState,
                                          hardware, drivetrain, poseEstimator, planner, watchdog))
{
    // Primes all four ports' encoders — parity with main.cpp's
    // hardware.begin() call, before the Drivetrain is configured.
    hardware.begin();

    msg::DrivetrainConfig dtConfig = defaultSimDrivetrainConfig();
    drivetrain.configure(dtConfig);
    // 082-003: PoseEstimator reads the SAME dtConfig drivetrain.configure()
    // just took -- one shared boot-config source, mirroring main.cpp's own
    // wiring (source/main.cpp).
    poseEstimator.configure(dtConfig);

    // 084-002: Planner configured with defaultSimPlannerConfig() (above) --
    // mirrors main.cpp's own planner.configure(defaultPlannerConfig()) boot
    // call.
    planner.configure(defaultSimPlannerConfig());

    // Seed the CFG-delta shadows the same way main.cpp does (DevLoopState's
    // own field comment): the first `DEV M <n> CFG ...`/`DEV DT CFG ...`
    // must merge onto the SAME calibration the motors/drivetrain were
    // actually constructed/configured with, not an all-zero blank.
    for (uint32_t i = 0; i < Subsystems::Hardware::kPortCount; ++i) {
        devState.motorConfigShadow[i] = motorConfigs.cfg[i];
    }
    devState.drivetrainConfigShadow = dtConfig;

    // 084-006: seed configState's OWN, independent config-plane shadow the
    // same way -- SET/GET's first delta must merge onto the SAME
    // calibration the motors/drivetrain/planner were actually constructed/
    // configured with, not an all-zero blank (config_commands.h's file
    // header; mirrors devState's seeding immediately above).
    for (uint32_t i = 0; i < Subsystems::Hardware::kPortCount; ++i) {
        configState.motorShadow[i] = motorConfigs.cfg[i];
    }
    configState.drivetrainShadow = dtConfig;
    configState.plannerShadow = defaultSimPlannerConfig();

    // Prime the capabilities cache for the default DEV DT PORTS binding —
    // read back via ports() (not a local copy), mirroring main.cpp exactly.
    Subsystems::DrivetrainPorts bootPorts = drivetrain.ports();
    drivetrain.setMotorCapabilities(hardware.motor(bootPorts.left).capabilities(),
                                     hardware.motor(bootPorts.right).capabilities());

    loop.hardware = &hardware;
    loop.drivetrain = &drivetrain;
    loop.poseEstimator = &poseEstimator;
    loop.telemetry = &telemetryState;
    loop.processor = &processor;
    loop.watchdog = &watchdog;
    loop.devState = &devState;
    loop.planner = &planner;
    loop.motionState = &motionState;
    // The loop-originated reply sink (Decision 3) — devLoopTick()'s watchdog-
    // fire EVT goes here, never into syncStore (which belongs solely to the
    // statement currently being dispatched by sim_command(), if any).
    loop.defaultReply = storeReply;
    loop.defaultReplyCtx = &asyncStore;

    // Start the watchdog window from sim t=0 and the host fake clock at 0,
    // mirroring main.cpp's watchdog.feed(uBit.systemTime()) boot call.
    watchdog.feed(0);
    Types::setHostClockNow(0);
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

// Advance the sim by one ordinary pass: devLoopTick(loop, now, nullptr) --
// no statement, so only the hardware tick, Drivetrain governance (if
// active), and the watchdog check run this pass.
void sim_tick(void* h, uint32_t now) {
    SimHandle* s = static_cast<SimHandle*>(h);
    Types::setHostClockNow(now);
    s->lastTickNow = now;
    devLoopTick(s->loop, now, nullptr);
}

// Dispatch one NUL-terminated command line synchronously. Copies `line`
// into a DevLoopStatement whose replyFn/replyCtx point at SimHandle's own
// syncStore, then calls devLoopTick() at the SAME `now` as the most recent
// sim_tick() (the dt=0 synchronous-command trick — see file header).
// Returns the number of reply bytes written into `reply` (not counting the
// final NUL), matching sim_conn.py's ctypes.c_int expectation.
int sim_command(void* h, const char* line, char* reply, int size) {
    SimHandle* s = static_cast<SimHandle*>(h);

    s->syncStore.reset();

    DevLoopStatement stmt;
    stmt.line = line;
    stmt.replyFn = storeReply;
    stmt.replyCtx = &s->syncStore;

    Types::setHostClockNow(s->lastTickNow);
    devLoopTick(s->loop, s->lastTickNow, &stmt);

    int n = s->syncStore.written;
    if (reply && size > 0) {
        int copy = (n < size - 1) ? n : size - 1;
        memcpy(reply, s->syncStore.buf, static_cast<size_t>(copy));
        reply[copy] = '\0';
        n = copy;
    }
    s->syncStore.reset();
    return n;
}

// ---------------------------------------------------------------------------
// Async EVT access — drains the loop-originated reply sink (see file
// header). Returns the number of bytes written into evts_buf.
// ---------------------------------------------------------------------------
int sim_get_async_evts(void* h, char* evts_buf, int evts_len) {
    SimHandle* s = static_cast<SimHandle*>(h);
    if (!evts_buf || evts_len <= 0) return 0;
    int n = s->asyncStore.written;
    if (n >= evts_len) n = evts_len - 1;
    memcpy(evts_buf, s->asyncStore.buf, static_cast<size_t>(n));
    evts_buf[n] = '\0';
    s->asyncStore.reset();
    return n;
}

// ---------------------------------------------------------------------------
// Ground-truth reads — Hal::PhysicsWorld's TRUE (unslipped, unerrored)
// accumulators. Reached through Subsystems::SimHardware's concrete plant()
// accessor directly, never through the abstract Subsystems::Hardware* base
// (architecture-update.md (081) Decision 2's Consequences).
// ---------------------------------------------------------------------------

float sim_get_true_pose_x(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().truePoseX(); }
float sim_get_true_pose_y(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().truePoseY(); }
float sim_get_true_pose_h(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().truePoseH(); }

// exact_pose — legacy aliases for the same true-pose reads (pre-081
// sim_conn.py names these get_exact_pose_*); kept as a second entry point
// onto the identical data rather than forcing an immediate host-side rename
// in this build/ABI-only ticket.
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
// own filtered, encoder-derived velocity() — port 1 = LEFT, port 2 = RIGHT,
// Subsystems::SimHardware's documented default binding (subsystems/
// sim_hardware.h's file header) — the errored/filtered observation a real
// firmware consumer (DEV M STATE, Drivetrain::governRatio) actually reads,
// as opposed to Hal::PhysicsWorld::trueVelL/R() (already exposed above as
// ground truth). Subsystems::SimHardware::rebindPlantPorts() is not itself
// exposed over this ABI (not required by this ticket's acceptance
// criteria); a future ticket that needs these two reads to track a rebound
// pair would add that entry point then.
//
// sim_get_pwm_l/r read the plant's own raw commanded actuator value
// ([-100, 100] — a plant channel, port-independent, like the encoder reads
// above).
// ---------------------------------------------------------------------------

float sim_get_enc_l(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().reportedEncL(); }
float sim_get_enc_r(void* h) { return static_cast<SimHandle*>(h)->hardware.plant().reportedEncR(); }

float sim_get_vel_l(void* h) { return static_cast<SimHandle*>(h)->hardware.simMotor(1).velocity(); }
float sim_get_vel_r(void* h) { return static_cast<SimHandle*>(h)->hardware.simMotor(2).velocity(); }

float sim_get_pwm_l(void* h) {
    return static_cast<float>(static_cast<SimHandle*>(h)->hardware.plant().pwmL());
}
float sim_get_pwm_r(void* h) {
    return static_cast<float>(static_cast<SimHandle*>(h)->hardware.plant().pwmR());
}

float sim_get_otos_x(void* h) { return static_cast<SimHandle*>(h)->hardware.simOdometer().odomX(); }
float sim_get_otos_y(void* h) { return static_cast<SimHandle*>(h)->hardware.simOdometer().odomY(); }
float sim_get_otos_h(void* h) { return static_cast<SimHandle*>(h)->hardware.simOdometer().odomH(); }

// ---------------------------------------------------------------------------
// Error-knob setters — each forwards to EXACTLY ONE hal/sim/sim_setters.h
// free function (ticket 003's canonical call site for that knob); no knob
// logic is duplicated here beyond the ctypes marshalling itself.
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
