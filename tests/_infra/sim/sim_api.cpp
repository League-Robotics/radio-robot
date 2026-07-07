// sim_api.cpp — extern "C" C ABI wrapper over the new source/ tree's
// dev-loop firmware (sprint 081-004, rewired for sprint 087 ticket 006's
// Rt::Blackboard/Rt::Configurator/Rt::CommandRouter transport). Loaded from
// Python via ctypes (host/robot_radio/io/sim_conn.py;
// tests/_infra/sim/firmware.py, ticket 081-005).
//
// SimHandle owns Subsystems::SimHardware + Subsystems::Drivetrain +
// Subsystems::PoseEstimator + Subsystems::Planner + one Rt::Blackboard, one
// Rt::Configurator, one Rt::CommandRouter, and dev_loop.h's LoopContext --
// the SAME runLoopPass(loop, bb, now, statement) function main.cpp calls,
// run here instead of being hand-mirrored (mirrors ticket 081-004's own
// "no hand-mirrored second copy" precedent, applied to the new transport).
//
// Two separate reply sinks (architecture-update.md (081) Decision 3,
// unchanged in shape by this rewrite):
//   - syncStore  -- the ONE statement's own reply during sim_command():
//     wired as BOTH of Rt::CommandRouter's reply channels (serial AND
//     radio -- the sim has no real transport distinction, so both resolve
//     to the same sink; only Rt::CommandRouter::route() ever picks between
//     them, and it always sees the same target either way).
//   - asyncStore -- LoopContext's own serialReply/serialCtx (and
//     radioReply/radioCtx), the loop-originated reply sink runLoopPass()
//     uses for output it generates ITSELF rather than in response to a
//     statement (the watchdog-fire EVT, motion-done EVT, safety_stop EVT,
//     and -- new since this ticket -- periodic TLM emission bound to
//     whichever channel issued STREAM) -- drained by sim_get_async_evts().
//
// The dt=0 synchronous-command trick (Decision 4, unchanged): sim_command()
// replays runLoopPass() at the SAME `now` as the most recent sim_tick()
// call, captured in SimHandle::lastTickNow -- never a fresh timestamp. Safe
// ONLY because Subsystems::SimHardware::tick()'s own re-entry guard treats a
// repeated same-`now` hardware.tick() as a complete no-op; the statement
// dispatch, queue drains, and watchdog check inside runLoopPass() still run
// normally on every call.
#include "commands/dev_commands.h"
#include "dev_loop.h"
#include "hal/sim/sim_setters.h"
#include "messages/drivetrain.h"
#include "messages/motor.h"
#include "messages/planner.h"
#include "runtime/blackboard.h"
#include "runtime/command_router.h"
#include "runtime/configurator.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
#include "subsystems/planner.h"
#include "subsystems/pose_estimator.h"
#include "subsystems/sim_hardware.h"
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
// for either build).
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
// SimHandle — one self-contained simulation instance allocated per
// sim_create() call. Member declaration order IS construction order (C++
// initializes members in declaration order regardless of the initializer
// list's own order) -- motorConfigs before hardware (which reads it),
// hardware/drivetrain/poseEstimator/planner before configurator (whose
// constructor reads hardware.config(port) and needs live subsystem
// references), bb before router (router's own construction is bb-agnostic,
// but bb must exist before configurator.publish(bb)/router.setReplyChannels()
// run in the constructor body).
// ---------------------------------------------------------------------------
struct SimHandle {
    MotorConfigSet motorConfigs;
    Subsystems::SimHardware hardware;
    Subsystems::Drivetrain drivetrain;
    Subsystems::PoseEstimator poseEstimator;
    Subsystems::Planner planner;
    Rt::Blackboard bb;
    Rt::Configurator configurator;
    Rt::CommandRouter router;
    LoopContext loop;

    ReplyStore syncStore;    // sim_command()'s synchronous reply (see file header)
    ReplyStore asyncStore;   // runLoopPass()'s loop-originated output (watchdog/motion/telemetry)

    // [ms] the most recent `now` passed to sim_tick(); sim_command() replays
    // runLoopPass() at this SAME now (the dt=0 synchronous-command trick,
    // Decision 4 — see file header).
    uint32_t lastTickNow = 0;

    SimHandle();
};

SimHandle::SimHandle()
    : motorConfigs(defaultMotorConfigSet()),
      hardware(motorConfigs.cfg),
      configurator(drivetrain, poseEstimator, planner, hardware,
                   defaultSimDrivetrainConfig(), defaultSimPlannerConfig())
{
    // Primes all four ports' encoders — parity with main.cpp's
    // hardware.begin() call, before the Drivetrain is configured.
    hardware.begin();

    msg::DrivetrainConfig dtConfig = defaultSimDrivetrainConfig();
    drivetrain.configure(dtConfig);
    // 082-003: PoseEstimator reads the SAME dtConfig drivetrain.configure()
    // just took -- one shared boot-config source, mirroring main.cpp's own
    // wiring.
    poseEstimator.configure(dtConfig);

    // 084-002: Planner configured with defaultSimPlannerConfig() (above) --
    // mirrors main.cpp's own planner.configure(defaultPlannerConfig()) boot
    // call.
    planner.configure(defaultSimPlannerConfig());

    // Rt::Configurator (087-005): seed bb's current-config cells from boot
    // config -- mirrors main.cpp's configurator.publish(bb) call.
    configurator.publish(bb);

    // Rt::CommandRouter (087-006): both reply channels resolve to the SAME
    // sync store -- the sim has no real serial/radio distinction for a
    // per-statement reply (see file header).
    router.setReplyChannels(storeReply, &syncStore, storeReply, &syncStore);

    // Boot-time hardware-identity snapshots (blackboard.h's file header):
    // never rewritten after this one-time seed.
    for (uint32_t port = 1; port <= Rt::kPortCount; ++port) {
        bb.motorCaps[port - 1] = hardware.motor(port).capabilities();
    }
    bb.otosPresent = (hardware.odometer() != nullptr);

    // Prime the capabilities cache for the default DEV DT PORTS binding —
    // read back via ports() (not a local copy), mirroring main.cpp exactly.
    Subsystems::DrivetrainPorts bootPorts = drivetrain.ports();
    drivetrain.setMotorCapabilities(hardware.motor(bootPorts.left).capabilities(),
                                     hardware.motor(bootPorts.right).capabilities());

    loop.hardware = &hardware;
    loop.drivetrain = &drivetrain;
    loop.poseEstimator = &poseEstimator;
    loop.planner = &planner;
    loop.router = &router;
    loop.configurator = &configurator;
    // The loop-originated reply sink (Decision 3) — runLoopPass()'s
    // watchdog/motion/telemetry EVTs go here, never into syncStore (which
    // belongs solely to the statement currently being dispatched by
    // sim_command(), if any). No real radio/serial distinction in the sim,
    // so both resolve to the same store.
    loop.serialReply = storeReply;
    loop.serialCtx = &asyncStore;
    loop.radioReply = storeReply;
    loop.radioCtx = &asyncStore;

    // Start the watchdog window from sim t=0 and the host fake clock at 0,
    // mirroring main.cpp's watchdog.feed(uBit.systemTime()) boot call.
    loop.watchdog.feed(0);
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

// Advance the sim by one ordinary pass: runLoopPass(loop, bb, now, nullptr)
// -- no statement, so only the hardware tick, Drivetrain governance (if
// active), and the watchdog check run this pass.
void sim_tick(void* h, uint32_t now) {
    SimHandle* s = static_cast<SimHandle*>(h);
    Types::setHostClockNow(now);
    s->lastTickNow = now;
    runLoopPass(s->loop, s->bb, now, nullptr);
}

// Dispatch one NUL-terminated command line synchronously. Copies `line`
// into a Subsystems::CommunicatorToCommandProcessorStatement (an OWNED
// char[256] buffer -- subsystems/statement.h) whose returnPath is SERIAL
// (arbitrary -- Rt::CommandRouter's two reply channels are wired to the
// same sync store either way, see file header), then calls runLoopPass() at
// the SAME `now` as the most recent sim_tick() (the dt=0 synchronous-command
// trick — see file header). Returns the number of reply bytes written into
// `reply` (not counting the final NUL), matching sim_conn.py's ctypes.c_int
// expectation.
int sim_command(void* h, const char* line, char* reply, int size) {
    SimHandle* s = static_cast<SimHandle*>(h);

    s->syncStore.reset();

    Subsystems::CommunicatorToCommandProcessorStatement stmt;
    stmt.returnPath = Subsystems::Channel::SERIAL;
    stmt.line[0] = '\0';
    if (line) {
        std::strncpy(stmt.line, line, sizeof(stmt.line) - 1);
        stmt.line[sizeof(stmt.line) - 1] = '\0';
    }

    Types::setHostClockNow(s->lastTickNow);
    runLoopPass(s->loop, s->bb, s->lastTickNow, &stmt);

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
