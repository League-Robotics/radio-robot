// ---------------------------------------------------------------------------
// RobotTelemetry.cpp — Robot::buildTlmFrame and Robot::telemetryEmit
//
// Split from Robot.cpp (sprint 035 A3). These are Robot:: member definitions
// only; class layout and Robot.h are unchanged.
// ---------------------------------------------------------------------------

#include "Robot.h"
#include <cstdio>
#include <cmath>

// ---------------------------------------------------------------------------
// buildTlmFrame — assemble the unified TLM frame; returns length.
//
// Reads drive2.state(), sensors.state(), config, motionController.mode().
// Shared by the periodic STREAM (telemetryEmit) and the synchronous SNAP command.
//
// 060-001: rewired from state.actual (legacy HardwareState) to the new
// message-contract subsystem state accessors: drive-related fields from
// drive2.state() (msg::DrivetrainState) and sensor fields from sensors.state()
// (subsystems::SensorsState). Wire format (field names, snprintf calls) unchanged.
// ---------------------------------------------------------------------------

int Robot::buildTlmFrame(char* buf, int len)
{
    uint32_t t_sample = systemTime();
    // Array convention: [0]=R (FR), [1]=L (FL) — same as ActualState.h enc arrays.
    // drive2.state().enc() returns the per-wheel encoder accumulator array.
    const msg::DrivetrainState& ds = drive2.state();
    const subsystems::SensorsState& ss = sensors.state();
    int32_t encL = static_cast<int32_t>(ds.enc()[1]);
    int32_t encR = static_cast<int32_t>(ds.enc()[0]);

    // 060-001: pose read from drive2.state().fused.pose (msg::Pose2D: x,y mm, h rad).
    // Convert to integer mm (x,y) and centidegrees (h) matching the legacy output.
    // RAD_TO_CDEG = 18000/pi ≈ 5729.578 — same constant Odometry::getPose uses.
    static constexpr float kRadToCdeg = 5729.5779513f;
    int32_t pose_x = 0, pose_y = 0, pose_h = 0;
    if (config.tlmFields & TLM_FIELD_POSE) {
        pose_x = static_cast<int32_t>(ds.fused.pose.x);
        pose_y = static_cast<int32_t>(ds.fused.pose.y);
        pose_h = static_cast<int32_t>(ds.fused.pose.h * kRadToCdeg);
    }
    // N8 (030-008): gate line/color on freshness, not just the sticky valid bit.
    // A sensor that wedges after boot keeps valid=true forever; consult the
    // last_upd / lag envelope instead: fresh = now - last_upd <= 2*lag.
    // lag is 0 until the first valid read (last_upd stays 0 too), so the
    // subtraction wraps and the gate is never met -- correct for "never read".
    bool haveLine = line.is_initialized() &&
                    ss.line.stamp.get_valid() &&
                    (t_sample - ss.line.stamp.get_last_upd()
                         <= 2u * ss.line.stamp.get_lag()) &&
                    (config.tlmFields & TLM_FIELD_LINE);
    bool haveColor = colorSensor.is_initialized() &&
                     ss.color.stamp.get_valid() &&
                     (t_sample - ss.color.stamp.get_last_upd()
                          <= 2u * ss.color.stamp.get_lag()) &&
                     (config.tlmFields & TLM_FIELD_COLOR);
    bool haveVel = (config.tlmFields & TLM_FIELD_VEL) != 0;
    // Array convention: [0]=R (FR), [1]=L (FL) — same as ActualState.h vel arrays.
    float velL = haveVel ? ds.vel()[1] : 0.0f;
    float velR = haveVel ? ds.vel()[0] : 0.0f;
    bool haveTwist = (config.tlmFields & TLM_FIELD_TWIST) != 0;

    char modeChar = 'I';
    switch (motionController.mode()) {
        case DriveMode::STREAMING: modeChar = 'S'; break;
        case DriveMode::DISTANCE:  modeChar = 'D'; break;
        case DriveMode::GO_TO:     modeChar = 'G'; break;
        case DriveMode::VELOCITY:  modeChar = 'V'; break;
        // N13 (030-010): TIMED removed -- T command runs as VELOCITY; mode=T
        // was unreachable in firmware. Host parser handles mode=T gracefully
        // for backward-compatibility with old logs.
        default:                   modeChar = 'I'; break;
    }

    int pos = 0, rem = len;
    int n = snprintf(buf + pos, (size_t)rem, "TLM t=%lu mode=%c seq=%u",
                     (unsigned long)t_sample, modeChar, (unsigned)_tlmSeq++);
    if (n > 0 && n < rem) { pos += n; rem -= n; }
    if (config.tlmFields & TLM_FIELD_ENC) {
        n = snprintf(buf + pos, (size_t)rem, " enc=%d,%d", (int)encL, (int)encR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (config.tlmFields & TLM_FIELD_POSE) {
        n = snprintf(buf + pos, (size_t)rem, " pose=%d,%d,%d",
                     (int)pose_x, (int)pose_y, (int)pose_h);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveVel) {
        n = snprintf(buf + pos, (size_t)rem, " vel=%d,%d", (int)velL, (int)velR);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveTwist) {
        // fusedV is body linear speed in mm/s (integer).
        // fusedOmega is yaw rate in rad/s; convert to mrad/s (integer) matching
        // the omega_mrads convention used by VW command and NezhaProtocol.vw().
        // 060-001: read velocity from drive2.state().fused.twist (msg::BodyTwist3:
        // v_x mm/s, omega rad/s). Semantics identical to legacy estimate.getVelocity.
        float fV     = ds.fused.twist.v_x;
        float fOmega = ds.fused.twist.omega;
        n = snprintf(buf + pos, (size_t)rem, " twist=%d,%d",
                     (int)fV,
                     (int)(fOmega * 1000.0f));
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    // N8 (030-008): gate raw otos= on freshness -- same 2*lag rule as
    // line/color above.  otos.valid stays true after the first success; if the
    // sensor goes dark the last-good pose would be emitted forever without
    // the freshness check.
    // 060-001: read from drive2.state().otos (msg::ValueSet) and
    // drive2.state().optical.pose (msg::Pose2D: x,y mm, h rad).
    if ((config.tlmFields & TLM_FIELD_OTOS) &&
        ds.otos.get_valid() &&
        (t_sample - ds.otos.get_last_upd()
             <= 2u * ds.otos.get_lag())) {
        // Raw OTOS pose (pre-fusion): x,y mm and heading in centidegrees,
        // matching the pose= field encoding. Lets the host plot the raw OTOS
        // sensor track alongside enc-derived and fused pose. 18000/pi cdeg/rad.
        n = snprintf(buf + pos, (size_t)rem, " otos=%d,%d,%d",
                     (int)ds.optical.pose.x,
                     (int)ds.optical.pose.y,
                     (int)(ds.optical.pose.h * kRadToCdeg));
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveLine) {
        // 060-001: read from sensors.state().line.raw() array (msg::LineSensorState).
        n = snprintf(buf + pos, (size_t)rem, " line=%u,%u,%u,%u",
                     (unsigned)ss.line.raw()[0], (unsigned)ss.line.raw()[1],
                     (unsigned)ss.line.raw()[2], (unsigned)ss.line.raw()[3]);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (haveColor) {
        // 060-001: read from sensors.state().color (msg::ColorSensorState).
        n = snprintf(buf + pos, (size_t)rem, " color=%u,%u,%u,%u",
                     (unsigned)ss.color.r, (unsigned)ss.color.g,
                     (unsigned)ss.color.b, (unsigned)ss.color.c);
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    if (config.tlmFields & TLM_FIELD_EKFREJ) {
        // Cumulative EKF gate rejection count -- all channels (pos, heading, velocity).
        // Sprint 024-005: emitted as ekf_rej=<n> for divergence visibility.
        n = snprintf(buf + pos, (size_t)rem, " ekf_rej=%d",
                     estimate.ekfRejectCount());
        if (n > 0 && n < rem) { pos += n; rem -= n; }
    }
    buf[pos] = '\0';
    return pos;
}

// ---------------------------------------------------------------------------
// telemetryEmit -- gate and emit the periodic TLM frame.
//
// D10 idle-rate change (028-005): the stream no longer goes silent when the
// robot is stopped.  When idle, the effective period is max(tlmPeriodMs, 500)
// so the host can distinguish "robot idle" from "serial dropped."
// The clamp (tlmPeriodMs < 20 -> 20) is enforced in handleStream, not here;
// telemetryEmit must NOT write to config.
// ---------------------------------------------------------------------------

void Robot::telemetryEmit(uint32_t now_ms, ReplyFn fn, void* ctx)
{
    if (config.tlmPeriodMs <= 0) return;

    // N3 null guard (030-003): _tlmBoundFn stays nullptr until STREAM binds the
    // channel.  SET tlmPeriod without a prior STREAM must not reach fn(...) -- a
    // null fn-pointer call is a HardFault on the micro:bit.  Silent suppression
    // matches the Robot.h:164-169 comment ("nullptr means TLM is suppressed").
    if (fn == nullptr) return;

    // Idle-rate: when stopped, slow down to max(period, 500 ms) so the stream
    // stays alive but doesn't flood the link with idle noise.
    static constexpr uint32_t kIdleMinMs = 500;
    static constexpr uint32_t kGraceMs   = 400;
    if (motionController.mode() != DriveMode::IDLE) _lastActiveMs = now_ms;
    bool stopped = ((now_ms - _lastActiveMs) > kGraceMs);

    uint32_t effectivePeriod = stopped
        ? ((uint32_t)config.tlmPeriodMs > kIdleMinMs
               ? (uint32_t)config.tlmPeriodMs
               : kIdleMinMs)
        : (uint32_t)config.tlmPeriodMs;

    // Radio rate cap: the radio/relay link sustains only ~5 Hz of TLM cleanly —
    // bench-measured 2026-06-14, STREAM 200 (5 Hz) delivered ~100% during motion,
    // but 10 Hz dropped ~85% and 20 Hz ~100%.  When TLM is bound to the radio
    // channel, floor the period at kRadioMinMs so motion frames actually arrive.
    // Serial keeps the full requested rate (no cap).  At rest the idle throttle
    // (>= kIdleMinMs) already exceeds this cap, so this only bites during motion.
    static constexpr uint32_t kRadioMinMs = 200;
    if (_tlmBoundIsRadio && effectivePeriod < kRadioMinMs) {
        effectivePeriod = kRadioMinMs;
    }

    if ((now_ms - _lastTlmMs) < effectivePeriod) return;

    char tlmBuf[160];
    buildTlmFrame(tlmBuf, sizeof(tlmBuf));
    fn(tlmBuf, ctx);
    _lastTlmMs = now_ms;
}
