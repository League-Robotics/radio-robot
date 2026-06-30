// sim_api.cpp — extern "C" C ABI wrapper over a self-contained simulation.
//
// Provides an opaque SimHandle that owns SimHardware + Robot + CommandProcessor.
// (040-002) SimHardware replaces MockHAL; PhysicsWorld is the single source of
// ground truth and the Sim* observation models read it.
// Python test code (ticket 020-004) loads this shared library via ctypes.
//
// Build: cmake -S . -B host_tests/build && cmake --build host_tests/build
// Load:  python3 -c "import ctypes; ctypes.CDLL('./host_tests/build/libfirmware_host.dylib')"

// Sprint 050, Ticket 004: EKFTiny must be included BEFORE robot/Robot.h so that
// EKF_N / EKF_M are defined from EKFTiny.h before tinyekf.h is pulled in by any
// other header.  EKFTiny.h itself guards the defines with #ifndef, so including it
// first is safe even if Robot.h or its transitive headers also include tinyekf.h.
#define EKF_N 5
#define EKF_M 2
#include "state/EKFTiny.h"

#include "robot/Robot.h"
#include "commands/CommandProcessor.h"
#include "commands/CommandQueue.h"
#include "commands/DebugCommands.h"
#include "hal/sim/SimHardware.h"
#include "hal/sim/PhysicsWorld.h"
#include "hal/sim/WorldView.h"
#include "hal/real/BenchOtosSensor.h"
#include "types/Config.h"
#include "types/Inputs.h"
#include "superstructure/MotionController.h"
#include "commands/MotionCommand.h"
#include "control/HaltController.h"
#include "robot/LoopTickOnce.h"
#include "types/CommandTypes.h"
#include "commands/ArgParse.h"

#include <cstring>
#include <cstdio>
#include <utility>

// Sim-injected clock.  Robot.cpp::system_timer_current_time() reads this so
// that Robot::systemTime() returns sim time instead of real wall-clock time.
// Updated at the top of sim_tick() and at the start of sim_command() so that
// time-based stop conditions (T ms=..., HALT TIME, watchdog) stay in the same
// epoch as driveAdvance(now_ms) and evaluate(now_ms).
uint32_t g_sim_now_ms = 0;

// ---------------------------------------------------------------------------
// ReplyStore — a heap-allocated reply accumulator.
//
// When MotionCommands (D, VW, R, …) complete during sim_tick(), they call
// the reply function stored in their reply sink.  In sim_api, the reply sink
// MUST point to a heap-allocated buffer (not a stack variable) so that
// driveAdvance() can safely fire EVTs at any tick after the command was issued.
//
// sim_command() writes both synchronous (OK/ERR) and async (EVT) replies into
// the same ReplyStore, then copies the accumulated bytes to the caller's buffer.
// ---------------------------------------------------------------------------
static constexpr int kReplyBufSize = 2048;

struct ReplyStore {
    char buf[kReplyBufSize];
    int  written = 0;

    void reset() { buf[0] = '\0'; written = 0; }

    void append(const char* msg) {
        if (!msg || written >= kReplyBufSize - 1) return;
        int remaining = kReplyBufSize - written - 1;
        int n = snprintf(buf + written, (size_t)remaining, "%s\n", msg);
        if (n > 0 && n < remaining) written += n;
    }
};

static void storeReply(const char* msg, void* ctx)
{
    static_cast<ReplyStore*>(ctx)->append(msg);
}

// ---------------------------------------------------------------------------
// SimHandle — one self-contained simulation instance allocated per test.
//
// Construction order is load-bearing:
//   1. cfg        — RobotConfig value from defaultRobotConfig()
//   2. hal        — SimHardware(cfg) (owns the PhysicsWorld plant + Sim* models)
//   3. robot      — Robot(hal, cfg), wires motorController/odometry/etc.
//   4. _queue     — CommandQueue (wired into cmd + robot._motionCtx)
//   5. cmd        — CommandProcessor with the full command table
//
// (040-002) hal is now SimHardware, not MockHAL.  PhysicsWorld is the single
// source of ground truth; the Sim* observation models read it.  cfg is declared
// BEFORE hal because SimHardware(const RobotConfig&) needs it at construction.
//
// Queue wiring: sim_api mirrors LoopScheduler's constructor wiring exactly —
//   cmd.setQueue(&_queue) and robot.setMotionQueue(&_queue) — so
//   converter commands (S, T, D, G, R, TURN) travel through the queue path,
//   not the direct begin*() fallback, matching firmware behaviour.
//   (sprint 026-002: replaced robot.motionController.setQueue() with
//   robot.setMotionQueue() since MotionCtx now lives in Robot.)
//
// replyStore: heap-allocated persistent reply buffer.  All sim_command calls
//   and async EVTs fired during sim_tick() accumulate here.  sim_command()
//   copies from replyStore into the caller's out_buf, then resets the store.
// ---------------------------------------------------------------------------
struct SimHandle {
    RobotConfig      cfg;
    SimHardware      hal;
    Robot            robot;
    // (040-003) WorldView — the canonical read-only bridge from PhysicsWorld
    // truth into the C ABI.  Holds references to hal.plant() (ground truth) and
    // robot.state.actual (firmware pose estimate), so it always sees current
    // state.  Declared AFTER hal and robot because it binds references to both.
    WorldView        _worldView;
    CommandQueue     _queue;
    // DebugCommands wired with robot; sched and bus are nullptr in sim.
    // CODAL-dependent handlers (DBG WEDGE, I2CW, etc.) reply ERR noimpl in
    // HOST_BUILD.  DBG OTOS BENCH and DBG OTOS work via HOST_BUILD paths.
    DebugCommands dbg;
    CommandProcessor cmd;
    ReplyStore       replyStore;

    // Bench OTOS sensor (sprint 031): standalone BenchOtosSensor for host-sim
    // tests.  In the real firmware NezhaHAL owns this; in the sim we own it
    // directly since NezhaHAL (CODAL) is excluded from HOST_BUILD.  Sim tests
    // drive it via sim_bench_otos_tick() and read it via sim_get_bench_otos_*.
    BenchOtosSensor  benchOtos;

    // Per-tick state: watchdog, last-run timestamps, active reply sink,
    // fuseOtos flag.  Replaces the former standalone watchdogMs field and
    // the hand-mirrored watchdog block in sim_tick().
    LoopTickState    _ts;

    SimHandle()
        : cfg(defaultRobotConfig())
        , hal(cfg)
        , robot(hal, cfg)
        , _worldView(hal.plant(), robot.state.actual)
        , _queue()
        // 044-003 (Phase F): DbgCtx gained busAccess; host build leaves both
        // busDiag and busAccess null (DebugCommands's I2C handlers are
        // #ifndef HOST_BUILD, so the null bus path is never exercised host-side).
        , dbg(DbgCtx{nullptr, nullptr, nullptr, &robot})
        , cmd(robot.buildCommandTable(&dbg, nullptr))
        , benchOtos()
    {
        // Initialize the bench OTOS sensor (sets _initialized = true; no I2C).
        benchOtos.begin();

        // Wire robot geometry into SimHardware so the OTOS sim model integrates
        // correctly.  SimHardware's ctor already set this from cfg; the explicit
        // call is harmless (idempotent) and documents the dependency.
        hal.setTrackwidth(cfg.trackwidthMm);

        // Wire the queue into both cmd and Robot's MotionCtx — mirrors LoopScheduler's
        // constructor wiring so converter commands (S, T, D, G, R, TURN, RT) travel
        // the queue path on the next sim_tick(), not the direct begin*() fallback.
        // Sprint 026-002: replaced robot.motionController.setQueue() with
        // robot.setMotionQueue() since MotionCtx now lives in Robot.
        cmd.setQueue(&_queue);
        robot.setMotionQueue(&_queue);

        // Set default reply sink in _ts so the watchdog and halt blocks have a
        // valid sink from the first command.  sim_command() will also set this
        // to storeReply / &replyStore on each command call.
        _ts.activeFn    = storeReply;
        _ts.activeTlmFn = storeReply;
        _ts.activeCtx   = &replyStore;

        // (045-002) Wire the wedge-EVT sink — mirrors main.cpp:229
        //   robot.motorController.setEvtSink(&sched.activeFn, &sched.activeCtx);
        // In the host sim, _ts plays the role of LoopScheduler's per-tick state,
        // and its activeFn/activeCtx are refreshed to storeReply/&replyStore at
        // the top of every sim_tick() / sim_command(). Binding the MotorController's
        // EVT sink to &_ts.activeFn / &_ts.activeCtx lets EVT enc_wedged flow into
        // replyStore (read by sim_get_async_evts), exactly as the firmware routes
        // it through sched.activeFn/Ctx. Without this the wedge latch still sets
        // (sim_get_wheel_wedged_*) but the EVT line is never emitted in sim.
        robot.motorController.setEvtSink(&_ts.activeFn, &_ts.activeCtx);
    }
};

// ---------------------------------------------------------------------------
// C ABI
// ---------------------------------------------------------------------------
extern "C" {

// ---- Lifecycle ----

void* sim_create()
{
    // Reset the injected clock so each SimHandle starts from t=0.
    // g_sim_now_ms is a global; without this reset, stale values from a
    // prior SimHandle would corrupt time-based stop conditions (HALT TIME,
    // watchdog) in the new instance.
    g_sim_now_ms = 0;
    return new SimHandle();
}

void sim_destroy(void* h)
{
    delete static_cast<SimHandle*>(h);
}

// ---- Tick ----

// Advance simulation by one control tick.
// hal.tick() drives MockMotor physics (integrates encoder mm from speed).
// controlCollectSplitPhase() reads encoders and runs the velocity PID.
// loopTickOnce() runs the single shared firmware loop body: dequeueOne,
// watchdog, halt, driveAdvance, odometry, OTOS fusion (when enabled),
// line/colour/ports/TLM timed blocks.
//
// EVTs fired during loopTickOnce() (e.g. EVT done D, EVT safety_stop) are
// written into SimHandle::replyStore via storeReply, which remains valid for
// the life of the SimHandle.
void sim_tick(void* h, uint32_t now_ms)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    // Keep the injected clock in sync so Robot::systemTime() returns sim time.
    g_sim_now_ms = now_ms;
    // (034-005) Upgraded to two-arg overload so the HAL plant receives the
    // current actuator commands.  Ordering invariant preserved: this call runs
    // BEFORE controlCollectSplitPhase so the encoders it reads are already
    // updated.  loopTickOnce will call hal.tick(now,cmds) again with the same
    // timestamp; MockHAL's dt==0 guard makes that second call a no-op.
    s->hal.tick(now_ms, s->robot.state.outputs);
    // (039-002) Sensor tick: promote the integrated encoder position into each
    // MockMotor's positionMm() accessor.  The outlier filter + velocity PID +
    // wedge push (formerly controlCollectSplitPhase) now run at the top of
    // loopTickOnce, reading positionMm().
    s->hal.tick(now_ms);

    // Ensure _ts has the current reply sink before each tick so that watchdog
    // and halt events go to replyStore.
    s->_ts.activeFn    = storeReply;
    s->_ts.activeTlmFn = storeReply;
    s->_ts.activeCtx   = &s->replyStore;

    // Run the shared firmware tick body: dequeue, watchdog, halt, drive,
    // odometry, OTOS/line/colour/ports/TLM blocks.
    loopTickOnce(s->robot, s->cmd, s->_queue, s->_ts, now_ms);
}

// ---- Command dispatch ----

// Process one NUL-terminated command line.
// All replies (synchronous OK/ERR and async EVTs from future ticks) are
// written into SimHandle::replyStore via the persistent storeReply callback.
// After cmd.process() returns, the synchronous portion is copied into out_buf
// and the store is reset.  Any async EVTs fired AFTER this call (during
// subsequent sim_tick() calls) will accumulate in replyStore and are
// accessible via sim_get_async_evts().
//
// Returns the number of synchronous bytes written (not counting the final NUL).
int sim_command(void* h, const char* line, char* out_buf, int out_len)
{
    SimHandle* s = static_cast<SimHandle*>(h);

    // Sync the injected clock to the current sim time before processing the
    // command.  Command handlers call robot->systemTime() (→ g_sim_now_ms) to
    // stamp time-based baselines (T ms=..., HALT TIME, watchdog).  Using sim
    // time here ensures those baselines are in the same epoch as driveAdvance.
    // g_sim_now_ms was last updated by the most-recent sim_tick() call.

    // Reset the store before the command so we capture only this command's
    // synchronous reply and not leftover async EVTs from prior ticks.
    s->replyStore.reset();

    // Set the active reply sink for this command.
    s->_ts.activeFn    = storeReply;
    s->_ts.activeTlmFn = storeReply;
    s->_ts.activeCtx   = &s->replyStore;

    // Process the command; all replies go into the persistent replyStore.
    // storeReply + &s->replyStore are passed as the reply sink to cmd.process()
    // and will also be captured by any MotionCommand that calls setReplySink().
    //
    // With the queue wired, cmd.process() enqueues the command rather than
    // dispatching it immediately.  We drain the queue right here so that
    // sim_command() remains synchronous — the caller receives the OK/ERR reply
    // before returning, exactly as before the queue was wired.
    //
    // Drain twice: once for the command itself (e.g. T handler → pushes VW),
    // once for any VW pushed by a converter handler (T/S/D/G/R/TURN/RT →
    // handleVW → beginTimed/beginStream/…).  A non-converter command (PING,
    // HALT, SET, …) will find the queue empty on the second call — no-op.
    s->cmd.process(line, storeReply, &s->replyStore);
    s->cmd.dequeueOne(s->_queue);  // dispatch the command
    s->cmd.dequeueOne(s->_queue);  // dispatch any VW pushed by a converter

    // Reset system watchdog on every inbound command — mirrors LoopScheduler's
    // resetWatchdog(now). Reset to the CURRENT sim time so keepalives actually
    // extend the window: the watchdog fires sTimeoutMs after the LAST command,
    // not the first. g_sim_now_ms==0 (a command before the first tick) maps to
    // the sentinel 1 so the timer stays armed (0 means "disarmed / none yet").
    s->_ts.watchdogMs = (g_sim_now_ms == 0) ? 1u : g_sim_now_ms;

    // Copy the synchronous reply into the caller's buffer.
    int n = s->replyStore.written;
    if (out_buf && out_len > 0) {
        int copy = (n < out_len - 1) ? n : out_len - 1;
        memcpy(out_buf, s->replyStore.buf, (size_t)copy);
        out_buf[copy] = '\0';
        n = copy;
    }

    // Reset written count so subsequent EVTs from driveAdvance() accumulate
    // from position 0 (overwriting the already-copied synchronous reply).
    s->replyStore.reset();

    return n;
}

// ---- Async EVT access ----

// Read async EVT replies accumulated in replyStore since the last
// sim_command() call.  Returns the number of bytes written into evts_buf.
int sim_get_async_evts(void* h, char* evts_buf, int evts_len)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    if (!evts_buf || evts_len <= 0) return 0;
    int n = s->replyStore.written;
    if (n >= evts_len) n = evts_len - 1;
    memcpy(evts_buf, s->replyStore.buf, (size_t)n);
    evts_buf[n] = '\0';
    // Drain the store so subsequent calls only see new EVTs from future ticks.
    // Callers (SimConnection._get_evts) own the returned bytes; the store is
    // refilled on the next sim_tick() when new EVTs fire.
    s->replyStore.reset();
    return n;
}

// ---- Encoder reads (accumulated mm from Robot::state.actual) ----

// Array convention: [0]=FR=R, [1]=FL=L — see ActualState.h.
float sim_get_enc_l(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.actual.encMm[1];  // FL = index 1
}

float sim_get_enc_r(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.actual.encMm[0];  // FR = index 0
}

// ---- Velocity reads (mm/s from Robot::state.actual) ----

float sim_get_vel_l(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.actual.velMms[1];  // FL = index 1
}

float sim_get_vel_r(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.actual.velMms[0];  // FR = index 0
}

// ---- PWM reads (from Robot::state.outputs) ----

float sim_get_pwm_l(void* h)
{
    return static_cast<float>(static_cast<SimHandle*>(h)->robot.state.outputs.pwm[1]);  // FL = index 1
}

float sim_get_pwm_r(void* h)
{
    return static_cast<float>(static_cast<SimHandle*>(h)->robot.state.outputs.pwm[0]);  // FR = index 0
}

// ---- Pose reads (fused estimate from Robot::state.actual.fused — 047-002) ----

float sim_get_pose_x(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.actual.fused.pose.x;
}

float sim_get_pose_y(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.actual.fused.pose.y;
}

float sim_get_pose_h(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.actual.fused.pose.h;
}

// ---- EKF diagnostics ----

// Cumulative EKF gate rejection count (all channels: position, heading, velocity).
// Exposed for the N1 regression test: assert ekf_rej == 0 after a D command when
// fusion is ON — the atomic encoder reset prevents the spurious negative delta that
// previously caused Mahalanobis-gate rejections for ~10 ticks post-D. (030-001)
int sim_get_ekf_rej_count(void* h)
{
    return static_cast<SimHandle*>(h)->robot.estimate.ekfRejectCount();
}

// ---- State injection ----

// Inject encoder position directly into the plant (040-003 FIX).
//
// HISTORY: 040-002 left this writing only state.actual.encLMm (plus a SimMotor
// reset) — the "lying" bug: state.actual.encLMm was overwritten on the next tick
// by the value promoted from positionMm(), so the injected value did not flow
// through the plant truth.
//
// FIX (040-003): set BOTH the TRUE wheel travel (ground truth, read by
// sim_get_true_*) AND the REPORTED encoder accumulator (read by
// SimMotor::positionMm() → loopTickOnce → state.actual.encLMm) directly in the
// plant.  Now the injected value survives the next tick: plant.update() ADDS
// vel*dt to the reported accumulator (0 at 0 PWM), tick() promotes it into
// positionMm(), and loopTickOnce writes it back to state.actual.encLMm.
// state.actual is also patched here to keep the current tick in sync before the
// next sim_tick() runs.
void sim_set_enc_l(void* h, float mm)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    PhysicsWorld& p = s->hal.plant();
    p.setTrueWheelTravel(mm, p.trueEncRMm());     // TRUE travel (ground truth)
    p.setReportedEncoder(0, mm);                  // REPORTED accumulator (side 0 = L)
    s->robot.state.actual.encMm[1] = mm;          // FL = index 1: keep state in sync this tick
}

void sim_set_enc_r(void* h, float mm)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    PhysicsWorld& p = s->hal.plant();
    p.setTrueWheelTravel(p.trueEncLMm(), mm);     // TRUE travel (ground truth)
    p.setReportedEncoder(1, mm);                  // REPORTED accumulator (side 1 = R)
    s->robot.state.actual.encMm[0] = mm;          // FR = index 0: keep state in sync this tick
}

// Inject an OTOS pose reading into the SimOdometer.  The injected pose is
// returned by SimOdometer::readTransformed() on the next otosCorrect() call.
void sim_set_otos_pose(void* h, float x, float y, float hrad)
{
    static_cast<SimHandle*>(h)->hal.simOdometer().setInjectedPose(x, y, hrad);
}

// Inject a per-wheel speed offset factor (1.0 = symmetric) into the plant.
// side: 0 = left, 1 = right, other = both.
void sim_set_motor_offset(void* h, int side, float factor)
{
    static_cast<SimHandle*>(h)->hal.plant().setOffsetFactor(side, factor);
}

// ---- True pose (plant ground truth, not the EKF/dead-reckoning estimate) ----
// (040-003) The canonical truth accessors.  WorldView::truePose* reads the
// PhysicsWorld plant — the single source of ground truth that replaced
// ExactPoseTracker.
float sim_get_true_pose_x(void* h) {
    return static_cast<SimHandle*>(h)->_worldView.truePoseX();
}
float sim_get_true_pose_y(void* h) {
    return static_cast<SimHandle*>(h)->_worldView.truePoseY();
}
float sim_get_true_pose_h(void* h) {
    return static_cast<SimHandle*>(h)->_worldView.truePoseH();
}

// ---- Exact pose (oracle ground truth) — FORMAL ALIAS of sim_get_true_pose_* ----
// (040-003) Back-compat: these now route through WorldView (the canonical
// accessor), identical to sim_get_true_pose_*.  The 040-002 temp alias read
// hal.plant() directly; this formalizes it via WorldView.
float sim_get_exact_pose_x(void* h) {
    return static_cast<SimHandle*>(h)->_worldView.truePoseX();
}
float sim_get_exact_pose_y(void* h) {
    return static_cast<SimHandle*>(h)->_worldView.truePoseY();
}
float sim_get_exact_pose_h(void* h) {
    return static_cast<SimHandle*>(h)->_worldView.truePoseH();
}

// ---- True wheel travel / velocity (plant ground truth) ----
// (040-003) Direct reads of the unslipped true accumulators / velocities.
float sim_get_true_enc_l(void* h) {
    return static_cast<SimHandle*>(h)->_worldView.trueEncLMm();
}
float sim_get_true_enc_r(void* h) {
    return static_cast<SimHandle*>(h)->_worldView.trueEncRMm();
}
float sim_get_true_vel_l(void* h) {
    return static_cast<SimHandle*>(h)->_worldView.trueVelLMms();
}
float sim_get_true_vel_r(void* h) {
    return static_cast<SimHandle*>(h)->_worldView.trueVelRMms();
}

// ---- Set true plant state directly (isolation tests) ----
// (040-003) Set the plant's ground-truth pose.  The next sim_tick does NOT
// overwrite it unless that tick integrates the actuator path (non-zero PWM +
// dt>0).  At 0 PWM, update() adds 0 to encoders and integrates 0 chassis motion,
// so an injected pose persists.
void sim_set_true_pose(void* h, float x, float y, float h_rad) {
    static_cast<SimHandle*>(h)->hal.plant().setTruePose(x, y, h_rad);
}

// Set the plant's TRUE wheel travel accumulators directly (ground truth).  Unlike
// sim_set_enc_l/r this touches ONLY the true accumulators (not the reported path
// or state.actual) — for pure plant-truth isolation tests.
void sim_set_true_wheel_travel(void* h, float enc_l_mm, float enc_r_mm) {
    static_cast<SimHandle*>(h)->hal.plant().setTrueWheelTravel(enc_l_mm, enc_r_mm);
}

// Set the plant's TRUE per-wheel velocity directly (ground truth, mm/s).
void sim_set_true_velocity(void* h, float vel_l_mms, float vel_r_mms) {
    static_cast<SimHandle*>(h)->hal.plant().setTrueVelocity(vel_l_mms, vel_r_mms);
}

// ---- Estimation error: firmware estimate vs. plant truth ----
// (040-003) WorldView crosses the plant/estimate boundary.  XY is the Euclidean
// distance (mm) between true pose and the firmware's fused/dead-reckoned pose;
// H is the heading error (rad) wrapped to [-pi, pi].  Both are 0 when the robot
// has not moved (estimate == truth == origin).
float sim_get_estimation_error_xy(void* h) {
    return static_cast<SimHandle*>(h)->_worldView.estimationErrorXY();
}
float sim_get_estimation_error_h(void* h) {
    return static_cast<SimHandle*>(h)->_worldView.estimationErrorH();
}

// ---- Reset all observation-model error layers to no-op (perfect sensors) ----
// (040-003) After this call every Sim* observation model is fresh-perfect: no
// freeze/dropout, no read failure, no noise/drift.  Mirrors the "fresh sensor is
// PERFECT" invariant (each error setter defaults to no-op at construction).
void sim_set_perfect(void* h) {
    SimHandle* s = static_cast<SimHandle*>(h);
    // Drive-wheel encoders: clear freeze + zero per-side noise.
    s->hal.simMotorL().setFrozen(false);
    s->hal.simMotorR().setFrozen(false);
    s->hal.simMotorL().setNoiseSigma(0.0f);
    s->hal.simMotorR().setNoiseSigma(0.0f);
    // OTOS odometer: clear read failure + lift; zero linear/yaw noise.
    s->hal.simOdometer().setReadFailure(false);
    s->hal.simOdometer().setLift(false);
    s->hal.simOdometer().setLinearNoiseSigma(0.0f);
    s->hal.simOdometer().setYawNoiseSigma(0.0f);
    // Line / color sensors: clear freeze.
    s->hal.simLineSensor().setFrozen(false);
    s->hal.simColorSensor().setFrozen(false);
    // Plant dynamics-error: zero encoder slip and noise (true == reported).
    s->hal.plant().setSlip(0.0f, 0.0f);
    s->hal.plant().setEncoderNoise(2, 0.0f);
}

// ---- Encoder noise/slip (side: 0=left, 1=right, 2=both) ----
// (040-002) Re-pointed to the plant's reported-encoder slip/noise model
// (OQ-1 Option A — legacy MockMotor encoder-step model preserved bit-for-bit).
void sim_set_motor_slip(void* h, int side, float straight, float turn_extra) {
    // PhysicsWorld applies one shared slip pair to the reported encoder; the
    // field-profile fixture always calls side=2, so a single set matches the
    // retired per-motor behaviour.
    (void)side;
    static_cast<SimHandle*>(h)->hal.plant().setSlip(straight, turn_extra);
}
void sim_set_encoder_noise(void* h, int side, float sigma_mm) {
    static_cast<SimHandle*>(h)->hal.plant().setEncoderNoise(side, sigma_mm);
}

// ---- OTOS sim model ----
void sim_enable_otos_model(void* h) {
    static_cast<SimHandle*>(h)->hal.simOdometer().enableSimModel(true);
}
// Enable/disable the firmware OTOS EKF correction inside sim_tick().
// Also marks the SimOdometer initialised so Robot::otosCorrect() does not
// early-return on its is_initialized() guard.
void sim_set_otos_fusion(void* h, int on) {
    SimHandle* s = static_cast<SimHandle*>(h);
    s->_ts.fuseOtos = (on != 0);
    if (s->_ts.fuseOtos) s->hal.simOdometer().begin();
}
void sim_set_otos_linear_noise(void* h, float sigma_fraction) {
    static_cast<SimHandle*>(h)->hal.simOdometer().setLinearNoise(sigma_fraction);
}
void sim_set_otos_yaw_noise(void* h, float sigma_fraction) {
    static_cast<SimHandle*>(h)->hal.simOdometer().setYawNoise(sigma_fraction);
}
float sim_get_otos_x(void* h) {
    return static_cast<SimHandle*>(h)->hal.simOdometer().odomX();
}
float sim_get_otos_y(void* h) {
    return static_cast<SimHandle*>(h)->hal.simOdometer().odomY();
}
float sim_get_otos_h(void* h) {
    return static_cast<SimHandle*>(h)->hal.simOdometer().odomH();
}

// ---- N2 queue-invariant helper (030-002) ----

// Returns 1 if the CommandProcessor has a queue attached (cmd.hasQueue()),
// 0 otherwise.  Used by the boot/queue-invariant regression test to assert
// that cmd._queue survives a Phase-3-style reassignment.
//
// In the sim, the queue is wired in SimHandle's constructor and is never
// reassigned, so this always returns 1 after sim_create().  The regression
// test is therefore a structural canary: if CommandProcessor's move-assign
// ever silently clears the queue pointer again (e.g. a future refactor
// re-introduces the Phase-3 pattern), this accessor will catch it.
int sim_get_queue_wired(void* h)
{
    return static_cast<SimHandle*>(h)->cmd.hasQueue() ? 1 : 0;
}

// ---- D10 telemetry test helpers (028-005) ----

// Returns 1 if the robot's TLM channel is bound (_tlmBoundCtx != nullptr),
// 0 otherwise.  Used by channel-binding tests to verify handleStream stored
// the caller's reply ctx.  (In sim, _tlmBoundFn is not set since runCommsIn
// is not called; binding is signalled via _tlmBoundCtx alone.)
int sim_get_tlm_bound(void* h)
{
    return (static_cast<SimHandle*>(h)->robot._tlmBoundCtx != nullptr) ? 1 : 0;
}

// Force the _tlmBoundIsRadio flag so tests can exercise the radio TLM-rate cap
// in telemetryEmit.  In sim, runCommsIn (which normally resolves the channel
// type) is not called, so this flag is otherwise always false.
void sim_set_tlm_bound_radio(void* h, int on)
{
    static_cast<SimHandle*>(h)->robot._tlmBoundIsRadio = (on != 0);
}

// Advance simulation by total_ms in step_ms increments and collect all TLM
// lines emitted by telemetryEmit into out_buf.  Returns the number of TLM
// lines collected (not bytes).  out_buf receives concatenated lines separated
// by '\n'; it is NUL-terminated.
//
// Note: sim_tick() resets _ts.activeTlmFn = storeReply each tick, so TLM
// frames from telemetryEmit go into replyStore.  After each tick we drain
// replyStore into out_buf, counting the TLM lines.
int sim_tick_collect_tlm(void* h, uint32_t start_ms, uint32_t total_ms,
                         uint32_t step_ms, char* out_buf, int out_len)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    char evtBuf[2048];
    int  tlmCount = 0;
    int  outPos   = 0;

    uint32_t end_ms = start_ms + total_ms;
    for (uint32_t t = start_ms; t < end_ms; t += step_ms) {
        g_sim_now_ms = t;
        // (034-005) Upgraded to two-arg overload — same ordering invariant as
        // sim_tick: plant runs before controlCollectSplitPhase; loopTickOnce's
        // subsequent hal.tick(t,cmds) is idempotent via the dt==0 guard.
        s->hal.tick(t, s->robot.state.outputs);
        // (039-002) Sensor tick — see sim_tick comment.
        s->hal.tick(t);

        s->_ts.activeFn    = storeReply;
        s->_ts.activeTlmFn = storeReply;
        s->_ts.activeCtx   = &s->replyStore;

        loopTickOnce(s->robot, s->cmd, s->_queue, s->_ts, t);

        // Drain replyStore: look for TLM lines.
        int n = s->replyStore.written;
        if (n > 0) {
            char* p = s->replyStore.buf;
            char* end = p + n;
            while (p < end) {
                char* nl = p;
                while (nl < end && *nl != '\n') ++nl;
                // p..nl is one line (without the '\n').
                int lineLen = (int)(nl - p);
                if (lineLen >= 3 && p[0] == 'T' && p[1] == 'L' && p[2] == 'M') {
                    ++tlmCount;
                    // Append to out_buf if room.
                    if (out_buf && outPos + lineLen + 2 < out_len) {
                        memcpy(out_buf + outPos, p, (size_t)lineLen);
                        outPos += lineLen;
                        out_buf[outPos++] = '\n';
                    }
                }
                p = (nl < end) ? nl + 1 : end;
            }
            s->replyStore.reset();
        }
    }

    if (out_buf && out_len > 0) out_buf[outPos] = '\0';
    return tlmCount;
}

// ---- N7 queue-overflow test helpers (030-005) ----

// Returns the current number of items in the CommandQueue.
// Used by overflow regression tests to verify the queue fills and that
// subsequent enqueues (via sim_command) produce ERR full replies.
int sim_queue_size(void* h)
{
    return static_cast<SimHandle*>(h)->_queue.size();
}

// Fill the CommandQueue to capacity (COMMAND_QUEUE_CAPACITY = 4) with
// dummy no-op ParsedCommand entries.  Returns the number of items pushed.
// After this call, any sim_command() that routes through dispatchTable()
// (queue path) will fail with ERR full — except sim_command itself also
// drains two items, so the test should call sim_fill_queue() then send
// one more command WITHOUT calling dequeueOne.  This is done by calling
// sim_command_no_drain() below.
//
// The dummy entries use a nullptr desc so dequeueOne() is a safe no-op
// (it guards pc.desc != nullptr before calling handlerFn).
int sim_fill_queue(void* h)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    int pushed = 0;
    ParsedCommand dummy;
    dummy.desc    = nullptr;   // no-op: dequeueOne guards desc != nullptr
    dummy.replyFn = nullptr;
    dummy.replyCtx = nullptr;
    dummy.args.count = 0;
    dummy.corrId[0] = '\0';
    while (s->_queue.push_back(dummy)) ++pushed;
    return pushed;
}

// Process one command WITHOUT the two post-process dequeueOne drains.
// This is needed by the overflow test: we want dispatchTable() to find a
// full queue and reply ERR full, without draining the queue first.
// Returns the number of synchronous bytes written into out_buf.
int sim_command_no_drain(void* h, const char* line, char* out_buf, int out_len)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    s->replyStore.reset();
    s->_ts.activeFn    = storeReply;
    s->_ts.activeTlmFn = storeReply;
    s->_ts.activeCtx   = &s->replyStore;
    s->cmd.process(line, storeReply, &s->replyStore);
    // NOTE: no dequeueOne() calls — intentional, allows testing overflow.
    int n = s->replyStore.written;
    if (out_buf && out_len > 0) {
        int copy = (n < out_len - 1) ? n : out_len - 1;
        memcpy(out_buf, s->replyStore.buf, (size_t)copy);
        out_buf[copy] = '\0';
        n = copy;
    }
    s->replyStore.reset();
    return n;
}

// ---- N8 sensor-freshness helpers (030-008) ----

// Initialize (begin) the MockLineSensor so Robot::lineRead() considers it
// present and emits line= in TLM.  Call before tests that need line data.
void sim_init_line_sensor(void* h)
{
    static_cast<SimHandle*>(h)->hal.simLineSensor().begin();
}

// Initialize (begin) the SimColorSensor so Robot::colorRead() considers it
// present and emits color= in TLM.  Call before tests that need color data.
void sim_init_color_sensor(void* h)
{
    static_cast<SimHandle*>(h)->hal.simColorSensor().begin();
}

// Freeze / unfreeze the SimLineSensor.  When frozen, readValues() returns
// false so Robot::lineRead() never updates lineVS.lastUpdMs — after ~2×lagMs
// the TLM freshness gate drops the line= field from TLM frames.
void sim_set_line_frozen(void* h, int frozen)
{
    static_cast<SimHandle*>(h)->hal.simLineSensor().setFrozen(frozen != 0);
}

// Freeze / unfreeze the SimColorSensor.  When frozen, pollRGBC() returns
// false so Robot::colorRead() never updates colorVS.lastUpdMs — after ~2×lagMs
// the TLM freshness gate drops the color= field from TLM frames.
void sim_set_color_frozen(void* h, int frozen)
{
    static_cast<SimHandle*>(h)->hal.simColorSensor().setFrozen(frozen != 0);
}

// ---- (045-003) Fixed sensor-value injection (line / color) ----
//
// The Sim* line/color sensors read from an internal schedule table, NOT from
// the PhysicsWorld plant (PhysicsWorld::setTrueLineRaw/setTrueColorRGBC set
// plant truth that these sensors do not consult).  To inject a constant value
// that flows into HardwareState (line[]/colorR/G/B/C) — and thus into
// StopCondition::evaluate via the SENSOR/COLOR/LINE_ANY kinds — install a
// single-row schedule.  With _scheduleRows == 1 the sensor's currentRow() is
// always 0, so readValues()/pollRGBC() return the injected row every tick.
//
// Callers must first sim_init_line_sensor()/sim_init_color_sensor() (begin())
// so the LineSensor/ColorSensor subsystem periodics actually read the sensor.

void sim_set_line_values(void* h, uint16_t l0, uint16_t l1,
                         uint16_t l2, uint16_t l3)
{
    uint16_t row[1][4] = {{ l0, l1, l2, l3 }};
    static_cast<SimHandle*>(h)->hal.simLineSensor().setSchedule(row, 1);
}

void sim_set_color_rgbc(void* h, uint16_t r, uint16_t g,
                        uint16_t b, uint16_t c)
{
    uint16_t row[1][4] = {{ r, g, b, c }};
    static_cast<SimHandle*>(h)->hal.simColorSensor().setSchedule(row, 1);
}

// ---- N9 same-tick OTOS failure helper (030-008) ----

// Inject / clear an OTOS read failure.  When set, MockOtosSensor::readTransformed
// and readVelocityTransformed return false and emit {0,0,0}/{0,0}.
// Robot::otosCorrect() must detect this same-tick failure via the return value
// and skip EKF fusion — the fusedV/poseX/Y/H state must remain unchanged.
void sim_set_otos_read_failure(void* h, int fail)
{
    static_cast<SimHandle*>(h)->hal.simOdometer().setReadFailure(fail != 0);
}

// Read fusedV from state.actual.fused (EKF body-frame linear speed, mm/s).
// Used by N9 test to assert the fused velocity is not dragged to zero on
// a same-tick OTOS read failure.
// 047-002: re-pointed from legacy fusedV scalar to fused.twist.vx_mmps.
float sim_get_fused_v(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.actual.fused.twist.vx_mmps;
}

// Read fusedOmega from state.actual.fused (EKF yaw rate, rad/s).
// 047-002: re-pointed from legacy fusedOmega scalar to fused.twist.omega_rads.
float sim_get_fused_omega(void* h)
{
    return static_cast<SimHandle*>(h)->robot.state.actual.fused.twist.omega_rads;
}

// 033-003: set the encoder-omega health gate.  When healthy=0, predict()
// suppresses the encoder yaw-rate observation (simulating a wedged wheel) — the
// hook the wedge detector (033-005) will drive.  Used to verify that a wedged
// encoder cannot inject phantom omega into the fused state.
void sim_set_enc_omega_healthy(void* h, int healthy)
{
    static_cast<SimHandle*>(h)->robot.estimate.setEncOmegaHealthy(healthy != 0);
}

// N11: inject a dead-reckoning pose into state.actual directly.
// Used by test_n11 to place the robot "past" a G target so the PURSUE
// backtrack re-gate fires on the next few ticks.
// 047-004: writes only canonical fused.pose (compat scalars removed).
void sim_set_pose(void* h, float x, float y, float hrad)
{
    SimHandle* s = static_cast<SimHandle*>(h);
    s->robot.state.actual.fused.pose.x = x;
    s->robot.state.actual.fused.pose.y = y;
    s->robot.state.actual.fused.pose.h = hrad;
}

// N15: read one diagonal entry of the EKF covariance matrix P.
// Returns P[idx][idx] where idx in [0..4]:
//   0=x, 1=y, 2=theta, 3=v, 4=omega.
// Used by N15 test to verify Q effect is invariant to loop rate.
float sim_get_ekf_p_diag(void* h, int idx)
{
    if (idx < 0 || idx > 4) return -1.0f;
    return static_cast<SimHandle*>(h)->robot.estimate.ekfPDiag(idx);
}

// ---- Bench OTOS sim hooks (sprint 031-002) ----
//
// In firmware, NezhaHAL owns BenchOtosSensor and benchOtosTick() drives it.
// In the host sim, NezhaHAL is excluded (CODAL), so the bench sensor is owned
// directly by SimHandle.  These hooks let Python tests exercise the integrator
// without going through NezhaHAL.
//
// Usage pattern:
//   1. sim_bench_otos_tick(h, velL, velR, trackwidth, dt_ms) — integrate one step.
//   2. sim_get_bench_otos_x/y/h(h)                          — read ideal pose.
//   3. sim_bench_otos_reset(h)                               — zero accumulators.
//   4. sim_bench_otos_set_noise(h, noiseXY, noiseH, drift)  — set error model.

// Manually tick the bench OTOS sensor with explicit velocities.
// This mirrors what Robot::benchOtosTick() does in firmware via NezhaHAL,
// but operates on SimHandle::benchOtos directly.
void sim_bench_otos_tick(void* h, float vel_l, float vel_r,
                         float trackwidth_mm, uint32_t dt_ms)
{
    static_cast<SimHandle*>(h)->benchOtos.tick(vel_l, vel_r, trackwidth_mm, dt_ms);
}

// Read the noiseless ideal accumulator (ground truth).
float sim_get_bench_otos_x(void* h) {
    return static_cast<SimHandle*>(h)->benchOtos.idealX();
}
float sim_get_bench_otos_y(void* h) {
    return static_cast<SimHandle*>(h)->benchOtos.idealY();
}
float sim_get_bench_otos_h(void* h) {
    return static_cast<SimHandle*>(h)->benchOtos.idealH();
}

// Read the errored accumulator (what readTransformed returns).
float sim_get_bench_otos_errored_x(void* h) {
    return static_cast<SimHandle*>(h)->benchOtos.otosX();
}
float sim_get_bench_otos_errored_y(void* h) {
    return static_cast<SimHandle*>(h)->benchOtos.otosY();
}
float sim_get_bench_otos_errored_h(void* h) {
    return static_cast<SimHandle*>(h)->benchOtos.otosH();
}

// Reset both accumulators.
void sim_bench_otos_reset(void* h) {
    static_cast<SimHandle*>(h)->benchOtos.reset();
}

// Set error model parameters.
void sim_bench_otos_set_noise(void* h, float noise_xy, float noise_h,
                               float drift_rad_per_sec)
{
    static_cast<SimHandle*>(h)->benchOtos.setNoise(noise_xy, noise_h,
                                                    drift_rad_per_sec);
}

// ---- 033-005 wedge-defense sim hooks ----

// Read the per-wheel wedge latch state from MotorController (033-005e).
// Returns 1 if the wedge EVT latch is set (wheel is wedged), 0 otherwise.
int sim_get_wheel_wedged_l(void* h)
{
    return static_cast<SimHandle*>(h)->robot.motorController.wheelWedgedL() ? 1 : 0;
}
int sim_get_wheel_wedged_r(void* h)
{
    return static_cast<SimHandle*>(h)->robot.motorController.wheelWedgedR() ? 1 : 0;
}

// Read the odometry wedge-active gate (033-005e).
// Returns 1 when Odometry::_wedgeActive is true (dTheta suppressed in predict).
int sim_get_odometry_wedge_active(void* h)
{
    return static_cast<SimHandle*>(h)->robot.estimate.wedgeActive() ? 1 : 0;
}

// Read the odometry encoder-omega health gate (033-003 / 033-005e).
// Returns 1 when healthy (omega fused), 0 when suppressed (wedged).
int sim_get_odometry_enc_omega_healthy(void* h)
{
    return static_cast<SimHandle*>(h)->robot.estimate.encOmegaHealthy() ? 1 : 0;
}

// ---- Three-estimate pose reads (047-002) ----
//
// encoder : pure dead-reckoning accumulator — never touched by the EKF.
// optical : raw OTOS observation captured before EKF correction.
// fused   : EKF output — authoritative belief (same as sim_get_pose_x/y/h).
//
// These let tests compare the three estimates side by side (ticket 005
// fusion-validation test will use enc vs fused divergence as the key check).

float sim_get_enc_pose_x(void* h) {
    return static_cast<SimHandle*>(h)->robot.state.actual.encoder.pose.x;
}
float sim_get_enc_pose_y(void* h) {
    return static_cast<SimHandle*>(h)->robot.state.actual.encoder.pose.y;
}
float sim_get_enc_pose_h(void* h) {
    return static_cast<SimHandle*>(h)->robot.state.actual.encoder.pose.h;
}

float sim_get_otos_pose_x(void* h) {
    return static_cast<SimHandle*>(h)->robot.state.actual.optical.pose.x;
}
float sim_get_otos_pose_y(void* h) {
    return static_cast<SimHandle*>(h)->robot.state.actual.optical.pose.y;
}
float sim_get_otos_pose_h(void* h) {
    return static_cast<SimHandle*>(h)->robot.state.actual.optical.pose.h;
}

float sim_get_fused_pose_x(void* h) {
    return static_cast<SimHandle*>(h)->robot.state.actual.fused.pose.x;
}
float sim_get_fused_pose_y(void* h) {
    return static_cast<SimHandle*>(h)->robot.state.actual.fused.pose.y;
}
float sim_get_fused_pose_h(void* h) {
    return static_cast<SimHandle*>(h)->robot.state.actual.fused.pose.h;
}

// ---------------------------------------------------------------------------
// Sprint 050, Ticket 004 — standalone EKFTiny sim API
//
// These functions create and exercise an EKFTiny object independently from the
// live Robot/Odometry stack.  They are used by test_ekf.py to run the same
// numerical test suite against EKFTiny as is run against the Python EKF mirror,
// establishing parity between the two implementations.
//
// Pattern: sim_ekftiny_create() returns an opaque void* owning an EKFTiny
// instance.  Callers must call sim_ekftiny_destroy() when done.  This is a
// separate lifecycle from the main SimHandle.
// ---------------------------------------------------------------------------

// Opaque handle for a standalone EKFTiny instance (not wired into Robot).
void* sim_ekftiny_create()
{
    return new EKFTiny();
}

void sim_ekftiny_destroy(void* h)
{
    delete static_cast<EKFTiny*>(h);
}

// Mirrors EKFTiny::init().
void sim_ekftiny_init(void* h,
                      float q_xy, float q_theta, float q_v, float q_omega,
                      float r_otos_xy, float r_otos_v, float r_enc_v)
{
    static_cast<EKFTiny*>(h)->init(q_xy, q_theta, q_v, q_omega,
                                   r_otos_xy, r_otos_v, r_enc_v);
}

// Mirrors EKFTiny::setPose().
void sim_ekftiny_set_pose(void* h, float x, float y, float theta)
{
    static_cast<EKFTiny*>(h)->setPose(x, y, theta);
}

// Mirrors EKFTiny::predict().
void sim_ekftiny_predict(void* h, float dCenter, float dTheta,
                         float theta_before, float dt_s)
{
    static_cast<EKFTiny*>(h)->predict(dCenter, dTheta, theta_before, dt_s);
}

// Mirrors EKFTiny::updatePosition().
void sim_ekftiny_update_position(void* h, float x_otos, float y_otos)
{
    static_cast<EKFTiny*>(h)->updatePosition(x_otos, y_otos);
}

// Mirrors EKFTiny::updateVelocity().
void sim_ekftiny_update_velocity(void* h, float v_meas, float omega_meas,
                                 float r_v, float r_omega)
{
    static_cast<EKFTiny*>(h)->updateVelocity(v_meas, omega_meas, r_v, r_omega);
}

// Mirrors EKFTiny::updateHeading().
void sim_ekftiny_update_heading(void* h, float theta_meas, float r_theta)
{
    static_cast<EKFTiny*>(h)->updateHeading(theta_meas, r_theta);
}

// State accessors.
float sim_ekftiny_x(void* h)     { return static_cast<EKFTiny*>(h)->x(); }
float sim_ekftiny_y(void* h)     { return static_cast<EKFTiny*>(h)->y(); }
float sim_ekftiny_theta(void* h) { return static_cast<EKFTiny*>(h)->theta(); }
float sim_ekftiny_v(void* h)     { return static_cast<EKFTiny*>(h)->v(); }
float sim_ekftiny_omega(void* h) { return static_cast<EKFTiny*>(h)->omega(); }

int sim_ekftiny_rejected_count(void* h)
{
    return static_cast<EKFTiny*>(h)->rejectedCount();
}

int sim_ekftiny_rej_head_streak(void* h)
{
    return static_cast<EKFTiny*>(h)->rejHeadStreak();
}

int sim_ekftiny_rej_pos_streak(void* h)
{
    return static_cast<EKFTiny*>(h)->rejPosStreak();
}

// P diagonal accessor — index 0..4.
float sim_ekftiny_p_diag(void* h, int idx)
{
    return static_cast<EKFTiny*>(h)->pDiag(idx);
}

// Row accessor: return one row of the 5x5 P matrix as 5 floats into out[].
// Needed by tests that inspect off-diagonal entries (e.g. TestSetPose, block-
// decoupling tests).  out must point to a buffer of at least 5 floats.
void sim_ekftiny_p_row(void* h, int row, float* out)
{
    EKFTiny* e = static_cast<EKFTiny*>(h);
    if (row < 0 || row > 4 || !out) return;
    for (int col = 0; col < 5; ++col)
        out[col] = e->pEntry(row, col);
}

// Direct state injection — needed by tests that set _x[3] or _x[4] directly
// (e.g. TestPredictVelocity, TestUpdateVelocity, TestMahalanobisGating).
void sim_ekftiny_set_x(void* h, int idx, float val)
{
    static_cast<EKFTiny*>(h)->setXEntry(idx, val);
}

// Direct P injection — needed by tests that set P entries directly
// (e.g. TestUpdateHeading: e._P[2][2] = 0.1, TestPositionGateRecovery:
// e._rej_pos_streak = 0).
void sim_ekftiny_set_p(void* h, int row, int col, float val)
{
    static_cast<EKFTiny*>(h)->setPEntry(row, col, val);
}

// Direct streak injection — needed by tests that reset streaks (pre-005 logic
// simulation in TestPositionGateRecovery::test_pre_005_logic_does_not_converge).
void sim_ekftiny_set_rej_pos_streak(void* h, int streak)
{
    static_cast<EKFTiny*>(h)->setRejPosStreak(streak);
}

void sim_ekftiny_set_rej_head_streak(void* h, int streak)
{
    static_cast<EKFTiny*>(h)->setRejHeadStreak(streak);
}

// ---------------------------------------------------------------------------
// ArgParse C-ABI test hooks (051-002)
//
// sim_parse_schema — invoke parseSchema with a fully described schema and
// token/KV arrays, and return the ParseResult through output pointers.
//
// Parameters (in):
//   tokens     — array of ntokens C strings (positional tokens).
//   ntokens    — number of tokens.
//   kv_keys    — array of nkv key C strings (KV pairs).
//   kv_vals    — array of nkv value C strings (parallel to kv_keys).
//   nkv        — number of KV pairs.
//   def_names  — array of ndefs arg names (ArgDef.name).
//   def_kinds  — array of ndefs ArgKind values as int (0=INT,1=FLOAT,2=STR).
//   def_ranged — array of ndefs ranged flags (0 or 1).
//   def_lo     — array of ndefs lo bounds (meaningful when ranged).
//   def_hi     — array of ndefs hi bounds (meaningful when ranged).
//   ndefs      — number of ArgDef entries.
//   min_tokens — schema.minTokens.
//   variadic   — schema.variadic (0 or 1).
//   pack_kv    — schema.packKv key string, or nullptr for none.
//
// Parameters (out — caller provides arrays of capacity >= MAX_ARGS):
//   out_ok           — 1 on success, 0 on failure.
//   out_count        — number of args in result (undefined on failure).
//   out_arg_types    — arg type per slot (0=INT,1=FLOAT,2=STR).
//   out_arg_ivals    — ival per slot.
//   out_arg_fvals    — fval per slot.
//   out_arg_svals    — flat buffer; each arg uses 32 bytes at offset i*32.
//   out_err_detail   — on failure: pointer to detail string (may be null).
//                      Written as a C string into err_detail_buf[64].
//   err_detail_buf   — caller-provided 64-byte buffer for error detail string.
//
// Returns: 1 if ok, 0 if not ok.
// ---------------------------------------------------------------------------
int sim_parse_schema(
    const char* const* tokens, int ntokens,
    const char* const* kv_keys, const char* const* kv_vals, int nkv,
    const char* const* def_names, const int* def_kinds,
    const int* def_ranged, const int* def_lo, const int* def_hi, int ndefs,
    int min_tokens, int variadic, const char* pack_kv,
    int* out_ok, int* out_count,
    int* out_arg_types, int* out_arg_ivals, float* out_arg_fvals,
    char* out_arg_svals,            // flat: slot i occupies svals[i*32..i*32+31]
    char* err_detail_buf)           // 64-byte output for error detail
{
    // Build KVPair array on the stack.
    KVPair kvs[MAX_ARGS];
    int kv_count = (nkv < MAX_ARGS) ? nkv : MAX_ARGS;
    for (int i = 0; i < kv_count; ++i) {
        kvs[i].key   = kv_keys  ? kv_keys[i]  : nullptr;
        kvs[i].value = kv_vals  ? kv_vals[i]  : nullptr;
    }

    // Build ArgDef array on the stack.
    ArgDef defs[MAX_ARGS];
    int def_count = (ndefs < MAX_ARGS) ? ndefs : MAX_ARGS;
    for (int i = 0; i < def_count; ++i) {
        defs[i].name   = def_names  ? def_names[i]  : "";
        defs[i].kind   = static_cast<ArgKind>(def_kinds[i]);
        defs[i].ranged = def_ranged ? (def_ranged[i] != 0) : false;
        defs[i].lo     = def_lo     ? def_lo[i]     : 0;
        defs[i].hi     = def_hi     ? def_hi[i]     : 0;
    }

    ArgSchema schema;
    schema.defs       = defs;
    schema.ndefs      = def_count;
    schema.minTokens  = min_tokens;
    schema.variadic   = (variadic != 0);
    schema.packKv     = pack_kv;

    ParseResult r = parseSchema(tokens, ntokens,
                                kv_count > 0 ? kvs : nullptr, kv_count,
                                schema);

    *out_ok = r.ok ? 1 : 0;

    if (r.ok) {
        *out_count = r.args.count;
        for (int i = 0; i < r.args.count; ++i) {
            out_arg_types[i] = static_cast<int>(r.args.args[i].type);
            out_arg_ivals[i] = r.args.args[i].ival;
            out_arg_fvals[i] = r.args.args[i].fval;
            char* dst = out_arg_svals + i * 32;
            int j = 0;
            while (r.args.args[i].sval[j] && j < 31) {
                dst[j] = r.args.args[i].sval[j];
                ++j;
            }
            dst[j] = '\0';
        }
        if (err_detail_buf) err_detail_buf[0] = '\0';
    } else {
        *out_count = 0;
        // Write error detail into caller's buffer.
        if (err_detail_buf) {
            if (r.err.detail) {
                int k = 0;
                while (r.err.detail[k] && k < 63) {
                    err_detail_buf[k] = r.err.detail[k];
                    ++k;
                }
                err_detail_buf[k] = '\0';
            } else {
                err_detail_buf[0] = '\0';
            }
        }
    }

    return r.ok ? 1 : 0;
}

} // extern "C"
