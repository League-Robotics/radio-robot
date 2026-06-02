#pragma once
#include <stdint.h>
#include <math.h>
#include "Config.h"
#include "Protocol.h"

class MotorController;
class Odometry;

/**
 * DriveController — owns and advances the S/T/D/G drive state machines,
 * S-mode watchdog, streaming encoder counter, and odometry delta tracking.
 *
 * Calls MotorController for wheel control and reads Odometry for pose.
 * Does not own sensors. Does not parse commands.
 * Emits completion and telemetry strings through the injected ReplyFn.
 */
class DriveController {
public:
    DriveController(MotorController& mc, Odometry& odo, const RobotConfig& cfg);

    // Entry points — called from Robot drive methods.
    void beginStream(float leftMms, float rightMms, uint32_t now_ms);
    void beginTimed(float leftMms, float rightMms, uint32_t durationMs, uint32_t now_ms);
    void beginDistance(float leftMms, float rightMms, int32_t targetMm, uint32_t now_ms);
    void beginGoTo(float tx, float ty, float speedMms, uint32_t now_ms);
    void stop(uint32_t now_ms, ReplyFn fn, void* ctx);

    // Advance all state machines. Call once per main-loop iteration.
    // now_ms: current system time. fn/ctx: reply callback.
    void tick(uint32_t now_ms, ReplyFn fn, void* ctx);

    DriveMode mode() const { return _mode; }

    // Access timing state (used by CommandProcessor transitional wiring).
    uint32_t lastTickMs() const { return _lastTickMs; }
    uint32_t currentTimeMs() const { return _currentTimeMs; }

private:
    MotorController&   _mc;
    Odometry&          _odo;
    const RobotConfig& _cfg;

    // Drive mode
    DriveMode _mode;

    // S-mode watchdog
    uint32_t _lastSMs;

    // Current speed targets (kept for internal use only)
    float _tgtL;
    float _tgtR;

    // T-command termination
    uint32_t _tEndMs;

    // D-command termination
    int32_t  _dEncStartL;
    int32_t  _dEncStartR;
    int32_t  _dTargetMm;
    uint32_t _dTimeoutMs;

    // G go-to state machine
    enum class GPhase { IDLE, PRE_ROTATE, ARC };
    GPhase _gPhase;
    float  _gTargetX;
    float  _gTargetY;
    float  _gSpeed;
    float  _gArcLeftMm;
    float  _gArcRightMm;
    float  _gArcStartL;
    float  _gArcStartR;

    // Streaming state
    int32_t _encTickCount;

    // Tick timing
    uint32_t _lastTickMs;

    // Updated at top of tick(); used internally and exposed for transitional wiring.
    uint32_t _currentTimeMs;

    // Previous encoder positions for odometry delta computation
    int32_t _prevOdoEncL;
    int32_t _prevOdoEncR;

    // Internal helpers
    void fullStop(ReplyFn fn, void* ctx);
    void reportEncoders(ReplyFn fn, void* ctx);
    void reportOdo(ReplyFn fn, void* ctx);

    static void computeArc(float tx, float ty, float trackwidthMm,
                           float& leftMm, float& rightMm);
};
