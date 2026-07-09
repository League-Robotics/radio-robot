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
// (bb.motorIn[]/bb.driveIn/…) to be drained by the NEXT mandatory tick --
// replays Rt::MainLoop::tick() at the SAME `now` as the most recent
// sim_tick() call (SimHandle::lastTickNow). Safe ONLY because Subsystems::
// SimHardware::tick()'s own re-entry guard treats a repeated same-`now`
// hardware.tick() as a complete no-op for the PLANT integration (it still
// drains bb.motorIn[]/bb.motorResetIn[] every call, staging any freshly
// routed command). This keeps a `sim.command("S …")` immediately followed
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
#include "runtime/blackboard.h"
#include "runtime/command_router.h"
#include "runtime/main_loop.h"
#include "subsystems/drivetrain.h"
#include "subsystems/hardware.h"
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
        // 091-002: I2C flip-flop poll-schedule membership -- true for the
        // drive pair (ports 1/2, matching defaultSimDrivetrainConfig()'s
        // left_port=1/right_port=2), false otherwise. Subsystems::SimHardware
        // itself ignores this (it ticks all four ports every pass
        // unconditionally -- sim_hardware.h's own file header), but
        // dev_commands.cpp's DUTY/VEL/POS `ERR nodev` gate reads
        // bb.motorConfig[port-1].polled regardless of which Hardware owner
        // is behind it -- this is the config every pytest-collected sim test
        // actually runs against, so getting it right here keeps every
        // existing `DEV M 1|2 DUTY/VEL/POS` sim test passing unchanged.
        set.cfg[i].setPolled(i + 1 == 1 || i + 1 == 2);
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

// ---------------------------------------------------------------------------
// SimHandle — one self-contained simulation instance allocated per
// sim_create() call. Member declaration order IS construction order (C++
// initializes members in declaration order regardless of the initializer
// list's own order) -- motorConfigs before hardware (which reads it),
// hardware/drivetrain before loop (Rt::MainLoop's own constructor takes
// references to the same two subsystems), bb before router (router's own
// construction is bb-agnostic, but bb must exist before router.
// setReplyChannels() runs in the constructor body).
// ---------------------------------------------------------------------------
struct SimHandle {
    MotorConfigSet motorConfigs;
    Subsystems::SimHardware hardware;
    Subsystems::Drivetrain drivetrain;
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
      loop(hardware, drivetrain)
{
    // Primes all four ports' encoders — parity with main.cpp's
    // hardware.begin() call, before the Drivetrain is configured.
    hardware.begin();

    msg::DrivetrainConfig dtConfig = defaultSimDrivetrainConfig();
    drivetrain.configure(dtConfig);

    // Rt::CommandRouter (087-006, channel-distinct since 088-006): each
    // reply channel resolves to its OWN sync store -- see file header's
    // "Two reply-store instances".
    router.setReplyChannels(storeReply, &syncStoreSerial, storeReply, &syncStoreRadio);

    // Prime the capabilities cache for the default DEV DT PORTS binding —
    // read back via ports() (not a local copy), mirroring main.cpp exactly.
    Subsystems::DrivetrainPorts bootPorts = drivetrain.ports();
    drivetrain.setMotorCapabilities(hardware.motor(bootPorts.left).capabilities(),
                                     hardware.motor(bootPorts.right).capabilities());

    // Start the host fake clock at 0, mirroring main.cpp's boot moment (093:
    // there is no watchdog left to feed here).
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

// Advance the sim by one ordinary pass: one Rt::MainLoop::tick() (hardware
// tick, drivetrain tick, commit, routeOutputs -- no command, so no routing
// happens since none is being fed). No config-drain loop remains (093:
// there is no runtime config-application authority left to drain).
void sim_tick(void* h, uint32_t now) {
    SimHandle* s = static_cast<SimHandle*>(h);
    Types::setHostClockNow(now);
    s->lastTickNow = now;
    s->loop.tick(s->bb, now);
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
    // command just posted (motorIn[]/driveIn) be consumed by the next
    // mandatory tick, at the unchanged `now`.
    s->loop.tick(s->bb, s->lastTickNow);

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
