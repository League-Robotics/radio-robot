#pragma once
#include <stdint.h>
#include "hal/capability/IVelocityMotor.h"
#include "PhysicsWorld.h"

struct RobotConfig;

/**
 * SimMotor — observation model for one drive wheel (Sprint 040 Phase B, 040-002).
 *
 * Implements IVelocityMotor.  Holds a `const PhysicsWorld&` (the single source of
 * ground truth) and a MotorSide (LEFT or RIGHT).  Replaces the retired MockMotor,
 * which fused plant integration + slip + noise into one class.  SimMotor owns ONLY
 * the observation cache + (forward-compat) error setters; the plant integration
 * lives in PhysicsWorld::update (driven by SimHardware::tick(now,cmds)).
 *
 * Control law stays ABOVE the device line (Case B): MotorController computes the
 * per-wheel PI+FF PWM and calls setOutput()/setSpeed(); SimMotor stores it (and
 * forwards it to the plant so single-wheel injection works) — there is NO second
 * controller here.
 *
 * Read path:
 *   - tick(now_ms) promotes plant.reportedEncL/R() into _lastPosition and
 *     computes a position-difference velocity into _lastVelocityMmps.  This is a
 *     COPY only (no re-integration) — bit-identical to MockMotor::tick.
 *   - position() / velocityMmps() return the cached values (optionally errored).
 *
 * The reported encoder (PhysicsWorld OQ-1 Option A) carries the legacy encoder-step
 * slip + noise model, so the golden-TLM byte-exact canary and the slip-fence tests
 * reproduce MockMotor exactly.  Every error setter defaults to a no-op, so a fresh
 * SimMotor is PERFECT (the fidelity dial is at zero).
 *
 * No CODAL dependency.  Compiles with plain clang++ -std=c++11 -I source.
 */
class SimMotor : public IVelocityMotor {
public:
    enum class Side : int { LEFT = 0, RIGHT = 1 };

    SimMotor(const PhysicsWorld& plant, PhysicsWorld& mutablePlant, Side side)
        : _plant(plant), _mut(mutablePlant), _side(side) {}

    // IVelocityMotor interface -----------------------------------------------

    // begin()/setNeutralMode() — no-ops (the sim has no hardware to prime).
    void begin() override {}

    // Set speed as signed percentage (-100..100).  Stores PWM only and forwards
    // to the plant for this one wheel (so single-wheel state injection works);
    // the authoritative plant tick uses setActuators(cmds.pwmL, cmds.pwmR).
    void setSpeed(int8_t pct) override;

    // Per-loop sensor tick: promote plant.reportedEnc*Mm() into the accessor
    // cache and compute a differentiated velocity.  COPY only — no integration.
    void    tick(uint32_t now_ms) override;
    float   position()     const override { return _lastPosition; }   // [mm]
    float   velocityMmps() const override { return _lastVelocityMmps; }

    // Split-phase encoder I/O — no-ops in the sim (encoder always ready).
    void    requestEncoder() override {}
    int32_t collectEncoder() const override;
    float   readEncoder(const RobotConfig& cfg) const override;
    float   readEncoderAtomic(const RobotConfig& cfg) const override;
    float   readEncoderSettle(const RobotConfig& cfg) const override;
    void    resetEncoder() override;

    // rebaselineSoft() (064-003): sim has no I2C timing race to avoid, so
    // this performs the SAME effect resetEncoder() already does here (zero
    // the reported accumulator). Only the reset-kind counter differs
    // (softResetCount, not hardResetCount) — see hardResetCount()/
    // softResetCount() below, the testable surface for MotorController's
    // at-rest DECISION.
    void    rebaselineSoft() override;

    // Cumulative reset-kind counters (064-003), incremented by resetEncoder()
    // / rebaselineSoft() respectively.
    uint32_t hardResetCount() const override { return _hardResetCount; }
    uint32_t softResetCount() const override { return _softResetCount; }

    // Drive-wheel only — no on-chip position-move capability.
    IPositionMotor* asPositionMotor() override { return nullptr; }

    // Error setters (all default no-op → a fresh sensor is PERFECT) -----------

    // Gaussian encoder noise standard deviation (mm per tick), applied to the
    // plant's reported-encoder accumulator for this side.
    void setNoiseSigma(float sigma);   // [mm]

    // Frozen encoder (simulates a wedged sensor / dropout): tick() stops
    // promoting new plant values, so position() holds its last cached value.
    void setFrozen(bool frozen) { _frozen = frozen; }

    // (064-005) I2C read-failure injection, mirroring SimOdometer::
    // setReadFailure / sim_set_otos_read_failure — the SimMotor-side
    // counterpart to the real Motor's hold-last-value fix (CR-03). When
    // injected: tick() does not promote a fresh reportedEnc() (holds
    // _lastPosition, same early-return as _frozen), and collectEncoder() /
    // readEncoder() / readEncoderAtomic() / readEncoderSettle()
    // likewise return the last cached value instead of a live plant read —
    // validating the downstream contract (Drive::_runOutlierFilter →
    // MotorController::controlTick → Odometry/EKF) that the real firmware's
    // fix exists to protect. Defaults to false (a fresh SimMotor is PERFECT).
    void setReadFailure(bool fail) { _readFailure = fail; }

    // Encoder error injection (ticket 058-001): per-wheel scale error and slip,
    // forwarded to the plant's reported-encoder error model.  Defaults to zero
    // (no-op) so a fresh SimMotor is PERFECT.
    //   err:      fractional scale error (0 = perfect, 0.05 = 5% over-report).
    //   fraction: fraction of motion not registered (0 = perfect, 0.05 = 5% slip).
    void setScaleError(float err);
    void setSlip(float fraction);

    // Test accessors
    int8_t cmdSpeed() const { return _cmdSpeed; }

private:
    int sideIdx() const { return static_cast<int>(_side); }

    // Returns this side's reported (slipped/noisy) encoder from the plant.
    float reportedEnc() const;   // [mm]

    const PhysicsWorld& _plant;   // ground-truth read access
    PhysicsWorld&       _mut;     // for setActuator / noise config / reset
    Side                _side;

    int8_t  _cmdSpeed         = 0;

    // tick() cache — promoted from reportedEnc() by tick(now_ms).
    float    _lastPosition     = 0.0f;   // [mm]
    float    _lastVelocityMmps = 0.0f;
    uint32_t _lastTickMs       = 0;
    bool     _hasLastTick      = false;

    bool     _frozen           = false;

    // (064-005) I2C read-failure injection — see setReadFailure() above.
    bool     _readFailure      = false;

    // Cumulative reset-kind counters (064-003): resetEncoder() increments
    // _hardResetCount; rebaselineSoft() increments _softResetCount.
    uint32_t _hardResetCount   = 0;
    uint32_t _softResetCount   = 0;
};
