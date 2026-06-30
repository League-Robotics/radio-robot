#pragma once
#include <stdint.h>

#ifdef HOST_BUILD
#include <random>
#endif

/**
 * PhysicsWorld — the single source of ground truth for the simulated chassis.
 *
 * Sprint 040 (Phase B) consolidates the three currently-fused / triplicated
 * midpoint-arc integrators —
 *   - MockMotor::integrate   (encoder accumulation + slip + noise)
 *   - ExactPoseTracker::update (pre-slip oracle pose, in MockHAL.h)
 *   - MockOtosSensor::tick    (OTOS odom pose, with noise)
 * — into ONE canonical plant integrated by ONE update(dt) call.
 *
 * PhysicsWorld owns ONLY ground truth — no observation error of any kind lives
 * here.  Observation error (quantization, noise, drift, dropout) belongs to the
 * Sim* observation models (added in T2), each of which reads `const PhysicsWorld&`.
 *
 * Two ways in:
 *   1. Evolve mode:  setActuators(pwmL, pwmR) + update(dt_ms) — advances the
 *      chassis from PWM commands.
 *   2. Truth-injection mode:  setTruePose / setTrueWheelTravel / setTrueVelocity /
 *      setTrueSensorValues — sets ground truth directly for isolation tests.
 *      A set value is NOT overwritten by a subsequent update() unless that
 *      update() integrates the actuator path (which overwrites encoder/velocity).
 *
 * Zero-heap, single-threaded, value-member ownership.  Compiles host-side
 * (HOST_BUILD) with no CODAL dependency.
 *
 * GOLDEN-TLM CONSTRAINT: update() sub-step A (encoder accumulation) MUST match
 * MockMotor::integrate bit-for-bit for zero-slip / zero-noise / offset-factor-1.0
 * inputs.  See the comment in PhysicsWorld.cpp — do NOT simplify that expression.
 *
 * OQ-1 Option A (040-002): PhysicsWorld tracks BOTH a true (unslipped) encoder
 * accumulator AND a reported (legacy MockMotor encoder-step slip + noise)
 * accumulator.  SimMotor::positionMm() returns the REPORTED value, so the
 * sim_field_profile / slip-fence behaviour is preserved bit-for-bit vs. the old
 * MockMotor model (encoder over-reports arc by 26% on turns).  trueEncL/RMm()
 * remains the unslipped ground truth for T3's sim_get_true_* / setTrueWheelTravel.
 * In the golden-TLM fixture (zero slip, zero noise, offset-factor 1.0) the
 * reported and true accumulators are bit-identical AND equal to the value the
 * retired MockMotor::integrate produced — so the byte-exact canary is unaffected.
 */
class PhysicsWorld {
public:
    // Default dynamics parameters — match MockMotor / MockHAL defaults.
    static constexpr float kNominalMaxMms = 400.0f;   // MockMotor::kNominalMaxMms
    static constexpr float kDefaultTrackwidthMm = 150.0f;

    PhysicsWorld() = default;

    // --- Evolve mode ---------------------------------------------------------

    // Store the per-wheel PWM commands [-100, 100]; consumed by the next update().
    void setActuators(int8_t pwmL, int8_t pwmR) {
        _pwmL = pwmL;
        _pwmR = pwmR;
    }

    // Store ONE wheel's PWM command (SimMotor::setOutput forwards here; the
    // authoritative plant tick uses setActuators(cmds.pwmL, cmds.pwmR)).
    // side: 0 = left, 1 = right.
    void setActuator(int side, int8_t pwm) {
        if (side == 0) _pwmL = pwm;
        else           _pwmR = pwm;
    }

    // Advance the chassis one step of dt_ms milliseconds.  Two structurally
    // separate sub-steps: (A) encoder accumulation (golden-TLM bit-exact path),
    // (B) chassis pose integration (slip applied here, not on the TLM path).
    void update(uint32_t dt_ms);

    // --- Truth-injection mode (isolation tests) ------------------------------

    void setTruePose(float x, float y, float h) {
        _truePoseX = x;
        _truePoseY = y;
        _truePoseH = h;
    }

    void setTrueWheelTravel(float encL, float encR) {
        _trueEncLMm = encL;
        _trueEncRMm = encR;
    }

    void setTrueVelocity(float velL, float velR) {
        _trueVelLMms = velL;
        _trueVelRMms = velR;
    }

    // Set true auxiliary sensor values (line / color / port raw truth).
    void setTrueLineRaw(const uint16_t line[4]) {
        for (int i = 0; i < 4; ++i) _lineRaw[i] = line[i];
    }
    void setTrueColorRGBC(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
        _colorRGBC[0] = r; _colorRGBC[1] = g; _colorRGBC[2] = b; _colorRGBC[3] = c;
    }
    void setTruePort(int ch, uint16_t val) {
        if (ch >= 0 && ch < 4) _port[ch] = val;
    }

    // Convenience: set all auxiliary sensor values at once.
    void setTrueSensorValues(const uint16_t line[4],
                             uint16_t r, uint16_t g, uint16_t b, uint16_t c,
                             const uint16_t port[4]) {
        setTrueLineRaw(line);
        setTrueColorRGBC(r, g, b, c);
        for (int i = 0; i < 4; ++i) _port[i] = port[i];
    }

    // Zero all state.
    void reset();

    // --- Dynamics configuration ----------------------------------------------

    // Set fractional slip.  Matches the MockMotor::setSlip API so sim_api.cpp can
    // forward sim_set_motor_slip calls here in T2.  In PhysicsWorld the slip is
    // applied at the chassis body-rotation step (sub-step B), parallel to
    // Odometry::predict (architecture-update.md §"Slip moves to chassis
    // body-rotation step").  rotationalSlip = straight + turnExtra contributes the
    // configured factor; effectiveSlip() clamps/maps it (0/unset → 1.0 = no slip).
    void setSlip(float straight, float turnExtra) {
        _slipStraight  = straight;
        _slipTurnExtra = turnExtra;
        _rotationalSlip = straight + turnExtra;
    }

    // Per-wheel offset factor.  side: 0 = left, 1 = right, 2 = both.
    void setOffsetFactor(int side, float f) {
        if (side == 0)      { _offsetFactorL = f; }
        else if (side == 1) { _offsetFactorR = f; }
        else                { _offsetFactorL = f; _offsetFactorR = f; }
    }

    // OQ-1 Option A — reported-encoder (legacy MockMotor) noise config.  side:
    // 0 = left, 1 = right, 2 = both.  Gaussian encoder noise (mm per tick) added
    // to the REPORTED encoder accumulator only; the true accumulator is unaffected.
    void setEncoderNoise(int side, float sigmaMm) {
        if (side == 0 || side > 1) _encNoiseSigmaL = sigmaMm;
        if (side == 1 || side > 1) _encNoiseSigmaR = sigmaMm;
    }

    void setTrackwidth(float mm)    { _trackwidthMm = mm; }
    void setNominalMaxMms(float v)  { _nominalMaxMms = v; }

    // --- Read accessors (const ground-truth) ---------------------------------

    float truePoseX() const { return _truePoseX; }
    float truePoseY() const { return _truePoseY; }
    float truePoseH() const { return _truePoseH; }

    // Ground-truth accessors (ticket 057-005): return the plant's authoritative
    // integrated pose.  Alias to truePose* — ground truth IS the integrated state.
    float groundTruthX() const { return _truePoseX; }
    float groundTruthY() const { return _truePoseY; }
    float groundTruthH() const { return _truePoseH; }

    // Ideal (error-free) pose: currently identical to ground truth.  Diverges only
    // if wheel-encoder error / slip is introduced for the OTOS path separately.
    float idealX() const { return _truePoseX; }
    float idealY() const { return _truePoseY; }
    float idealH() const { return _truePoseH; }

    float trueEncLMm() const { return _trueEncLMm; }
    float trueEncRMm() const { return _trueEncRMm; }

    // OQ-1 Option A — reported (legacy MockMotor encoder-step slip + noise)
    // encoder accumulators.  SimMotor::positionMm() reads these.  Equal to the
    // true accumulators when slip == 0 and noise == 0 (golden-TLM fixture).
    float reportedEncLMm() const { return _reportedEncLMm; }
    float reportedEncRMm() const { return _reportedEncRMm; }

    // OQ-1 Option A — directly set the reported encoder accumulator (back-compat
    // for sim_set_enc_l/r which resets+rebuilds the legacy MockMotor encoder).
    void setReportedEncoder(int side, float mm) {
        if (side == 0) _reportedEncLMm = mm;
        else           _reportedEncRMm = mm;
    }

    // OQ-1 Option A — zero one side's reported encoder accumulator (mirrors
    // MockMotor::resetEncoder, which zeros _encoderMm for that one wheel).  The
    // true accumulator is the ground truth and is NOT reset here.
    void resetReportedEncoder(int side) {
        if (side == 0) _reportedEncLMm = 0.0f;
        else           _reportedEncRMm = 0.0f;
    }

    float trueVelLMms() const { return _trueVelLMms; }
    float trueVelRMms() const { return _trueVelRMms; }

    uint16_t lineRaw(int ch) const {
        return (ch >= 0 && ch < 4) ? _lineRaw[ch] : 0;
    }
    void colorRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) const {
        r = _colorRGBC[0]; g = _colorRGBC[1]; b = _colorRGBC[2]; c = _colorRGBC[3];
    }
    uint16_t port(int ch) const {
        return (ch >= 0 && ch < 4) ? _port[ch] : 0;
    }

    // Dynamics-parameter accessors (for tests / observation models).
    float trackwidthMm()   const { return _trackwidthMm; }
    float nominalMaxMms()  const { return _nominalMaxMms; }
    float rotationalSlip() const { return _rotationalSlip; }
    int8_t pwmL()          const { return _pwmL; }
    int8_t pwmR()          const { return _pwmR; }

private:
    // --- Commanded actuator state ---
    int8_t _pwmL = 0;
    int8_t _pwmR = 0;

    // --- True chassis pose ---
    float _truePoseX = 0.0f;
    float _truePoseY = 0.0f;
    float _truePoseH = 0.0f;

    // --- True per-wheel travel / velocity (unslipped ground truth) ---
    float _trueEncLMm  = 0.0f;
    float _trueEncRMm  = 0.0f;
    float _trueVelLMms = 0.0f;
    float _trueVelRMms = 0.0f;

    // --- Reported per-wheel travel (OQ-1 Option A: legacy MockMotor model) ---
    // encoder-step slip + Gaussian noise; read by SimMotor::positionMm().
    float _reportedEncLMm = 0.0f;
    float _reportedEncRMm = 0.0f;

    // --- True auxiliary sensor values (zero-initialized) ---
    uint16_t _lineRaw[4]   = {0, 0, 0, 0};
    uint16_t _colorRGBC[4] = {0, 0, 0, 0};   // r, g, b, c
    uint16_t _port[4]      = {0, 0, 0, 0};

    // --- Dynamics parameters ---
    float _trackwidthMm  = kDefaultTrackwidthMm;
    float _nominalMaxMms = kNominalMaxMms;
    float _rotationalSlip = 0.0f;             // 0/unset → effectiveSlip → 1.0
    float _slipStraight  = 0.0f;
    float _slipTurnExtra = 0.0f;

    // --- Per-wheel offset factors (default symmetric) ---
    float _offsetFactorL = 1.0f;
    float _offsetFactorR = 1.0f;

    // --- Reported-encoder error model (OQ-1 Option A) ---
    // Per-tick turn rate in [0, 1] (set by SimHardware before update()); drives
    // the encoder-step slip term (slip = slipStraight + slipTurnExtra * turnRate)
    // exactly as MockHAL::advance fed MockMotor::setTurnRate.
    float _turnRate       = 0.0f;
    float _encNoiseSigmaL = 0.0f;
    float _encNoiseSigmaR = 0.0f;

public:
    // Per-tick turn rate (set by SimHardware before update()); used only by the
    // reported-encoder slip term (sub-step A').
    void setTurnRate(float r) { _turnRate = r; }

private:
#ifdef HOST_BUILD
    // Two independent generators so the LEFT/RIGHT encoder-noise draw streams
    // match the retired MockMotor (each MockMotor owned its own std::mt19937{42u}).
    std::mt19937 _rngL{42u};
    std::mt19937 _rngR{42u};
#endif
};
