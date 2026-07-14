#pragma once
#include <stdint.h>

struct RobotConfig;
class IPositionMotor;

/**
 * IVelocityMotor — drive-wheel capability (039-001).
 *
 * Phase A introduces this header as the canonical name for the drive-wheel
 * interface.  During the transition (T1) the body is the verbatim former
 * IMotor interface, and `source/hal/IMotor.h` becomes a `using IMotor =
 * IVelocityMotor;` shim so every existing consumer (Motor, MockMotor,
 * MotorController, Robot) compiles unchanged.  The capability-typed method
 * rename (setOutput / position / velocityMmps / tick) and the split-phase
 * move into the Motor impl land in T2/T3 — bodies are not changed here.
 *
 * Allows MotorController and Robot to be written against the interface
 * rather than the concrete Motor class, enabling test doubles and future
 * alternative hardware back-ends.
 */
class IVelocityMotor {
public:
    virtual ~IVelocityMotor() = default;

    // Prime/initialize the motor at boot (e.g. encoder readback). Default no-op.
    // Concrete implementations (Motor) override this to prime the hardware encoder
    // so the first read after boot returns valid data, not the frozen-at-zero
    // value the Nezha 0x46 register exhibits before its first atomic read.
    virtual void begin() {}

    // Set speed as signed percentage (-100..100). Positive = logical forward.
    virtual void setSpeed(int8_t pct) = 0;

    // ---- Per-loop encoder tick + cheap state accessors (039-002) ----
    //
    // tick(now_ms): called once per cooperative-loop iteration (via
    // Hardware::tick(now_ms)) BEFORE loopTickOnce.  The concrete Motor performs
    // the split-phase encoder read here and caches the last-collected position
    // (mm) and a differentiated velocity (mm/s).  After tick() the control layer
    // reads those cached values through position() / velocityMmps() without
    // issuing any further I2C.  Default no-op so non-encoder test doubles (and
    // the MockMotor's separate integration path) need no special handling.
    //
    // NOTE: the speed-scaled outlier filter, velocity differentiation used by the
    // PID, and the wedge push into Odometry remain in the control layer
    // (MotorController::controlTick / loopTickOnce) per Sprint 039 Open Question 2
    // resolution (b) — only the request/collect I2C read moves into the impl.
    virtual void tick(uint32_t now_ms) { (void)now_ms; }

    // position(): last-collected cumulative encoder position [mm] (float).
    // Cheap accessor — returns the value cached by the most recent tick(); no I2C.
    virtual float position() const = 0;

    // velocityMmps(): last-differentiated wheel velocity in mm/s.
    // Cheap accessor — returns the value cached by the most recent tick(); no I2C.
    virtual float velocityMmps() const = 0;

    // Split-phase encoder I/O, phase 1: issue the 0x46 write and return.
    virtual void requestEncoder() = 0;

    // Split-phase encoder I/O, phase 2: read back the 4-byte response.
    virtual int32_t collectEncoder() const = 0;

    // High-resolution encoder read in mm as float (used by velocity loop).
    // readEncoder is the generic name; concrete variants below for
    // specific timing contexts.
    virtual float readEncoder(const RobotConfig& cfg) const = 0;

    // Atomic single-shot encoder read (~8 ms, safe outside control loop).
    virtual float readEncoderAtomic(const RobotConfig& cfg) const = 0;

    // Settle-only encoder read (~4 ms, safe inside fixed-rate control loop).
    virtual float readEncoderSettle(const RobotConfig& cfg) const = 0;

    // Zero this motor's encoder accumulator (software offset reset).
    virtual void resetEncoder() = 0;

    // ---- Software-only rebaseline (064-003) ----
    //
    // rebaselineSoft(): zero this motor's encoder accumulator WITHOUT issuing
    // any I2C transaction — folds the already-tick-cached position back into
    // the software offset instead of firing the hardware atomic-read burst
    // resetEncoder() uses. MotorController::resetEncoderAccumulators() calls
    // this instead of resetEncoder() whenever the drivetrain is not at rest,
    // because firing the atomic 0x46 burst while the wheels are rotating
    // latches the Nezha encoder readback (see clasi/sprints/064-.../issues/
    // encoder-reset-while-moving-latches-readback.md). Pure — both current
    // implementers (Motor, SimMotor) are updated alongside this interface
    // change, so there is no default body.
    virtual void rebaselineSoft() = 0;

    // hardResetCount() / softResetCount(): cumulative count of resetEncoder()
    // / rebaselineSoft() calls respectively, for testability (064-003). Both
    // default to 0 so any OTHER implementer of this interface (outside
    // Motor/SimMotor) keeps compiling unmodified.
    virtual uint32_t hardResetCount() const { return 0; }
    virtual uint32_t softResetCount() const { return 0; }

    // ---- Secondary-capability discovery (039-003) ----
    //
    // asPositionMotor(): RTTI-free downcast to the position-move capability.
    // Firmware is compiled -fno-rtti, so dynamic_cast is unavailable; this
    // virtual accessor lets the control layer ask whether a given drive motor
    // also supports on-chip move-to-angle (Nezha 0x5D/0x70) without a cast.
    //
    // Default returns nullptr — a plain drive motor (and the sim MockMotor)
    // does NOT expose position control.  The concrete Motor overrides this to
    // return a non-null IPositionMotor* backed by an inner adapter that forwards
    // to moveToAngle() / timedMove().
    virtual IPositionMotor* asPositionMotor() { return nullptr; }
};

// 044-004 (Phase F): the former `source/io/IMotor.h` alias shim is deleted; its
// `using IMotor = IVelocityMotor;` alias is folded in here so every consumer that
// still names IMotor (Motor, MotorController, Robot, Drive) compiles unchanged.
// Behaviour-preserving rename housekeeping — no wire bytes change.
using IMotor = IVelocityMotor;
