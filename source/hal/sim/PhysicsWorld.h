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
 *   1. Evolve mode:  setActuators(pwmL, pwmR) + update(dt) — advances the
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
 * accumulator.  SimMotor::position() returns the REPORTED value, so the
 * sim_field_profile / slip-fence behaviour is preserved bit-for-bit vs. the old
 * MockMotor model (encoder over-reports arc by 26% on turns).  trueEncL/R()
 * remains the unslipped ground truth for T3's sim_get_true_* / setTrueWheelTravel.
 * In the golden-TLM fixture (zero slip, zero noise, offset-factor 1.0) the
 * reported and true accumulators are bit-identical AND equal to the value the
 * retired MockMotor::integrate produced — so the byte-exact canary is unaffected.
 */
class PhysicsWorld {
public:
    // Default dynamics parameters — match MockMotor / MockHAL defaults.
    static constexpr float kNominalMaxSpeed = 400.0f;  // [mm/s] MockMotor::kNominalMaxMms
    static constexpr float kDefaultTrackwidth = 150.0f;

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

    // Advance the chassis one step of dt milliseconds.  Two structurally
    // separate sub-steps: (A) encoder accumulation (golden-TLM bit-exact path),
    // (B) chassis pose integration (slip applied here, not on the TLM path).
    void update(uint32_t dt);  // [ms]

    // --- Truth-injection mode (isolation tests) ------------------------------

    void setTruePose(float x, float y, float h) {
        _truePoseX = x;
        _truePoseY = y;
        _truePoseH = h;
    }

    void setTrueWheelTravel(float encL, float encR) {
        _trueEncL = encL;
        _trueEncR = encR;
    }

    void setTrueVelocity(float velL, float velR) {
        _trueVelL = velL;
        _trueVelR = velR;
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
    // body-rotation step").
    //
    // 073-002 (Decision 2): _rotationalSlip (body truth) is derived from
    // `straight` ONLY — `turnExtra` no longer contributes to it. `turnExtra`
    // is an encoder-report-only knob (see _slipTurnExtra / sub-step A′,
    // still set below, unaffected); the only current caller of a nonzero
    // turnExtra (the TestGUI's slip_turn_extra control) was, before this
    // change, able to perturb body truth by accident, relying on
    // effectiveSlip()'s <=0 clamp to neutralize a negated value rather than
    // the channel being structurally unreachable. Every other caller that
    // wants a genuine body-truth effect via this channel already passes
    // turnExtra=0.0 (test_sim_otos_lever_arm.py, test_physics_world_basic.py,
    // test_physics_world_body_scrub.py), so this narrowing is arithmetically
    // a no-op for them. effectiveSlip() clamps/maps _rotationalSlip
    // (0/unset → 1.0 = no slip).
    void setSlip(float straight, float turnExtra) {
        _slipStraight  = straight;
        _slipTurnExtra = turnExtra;
        _rotationalSlip = straight;
    }

    // Body-truth scrub (069-002): independent, wire-settable rotational/linear
    // efficiency applied in sub-step B, MULTIPLICATIVELY combined with (not
    // replacing) the effectiveSlip(_rotationalSlip) term above — see
    // PhysicsWorld.cpp's sub-step B and architecture-update.md §4b/Decision 4.
    // Default 1.0 = no-op: every existing test that never calls these setters
    // observes byte-identical sub-step B output. Unlike _rotationalSlip (a
    // test-infra encoder-defect knob with a migration-safe 0-means-unset
    // history), these are brand-new fields with no such history, so they are
    // clamped by the new, simpler clampScrub() (range (0, 1]) in
    // PhysicsWorld.cpp — deliberately NOT effectiveSlip() (Decision 2).
    void setBodyRotationalScrub(float f) { _bodyRotationalScrub = f; }
    void setBodyLinearScrub(float f)     { _bodyLinearScrub = f; }

    // Per-wheel offset factor.  side: 0 = left, 1 = right, 2 = both.
    void setOffsetFactor(int side, float f) {
        if (side == 0)      { _offsetFactorL = f; }
        else if (side == 1) { _offsetFactorR = f; }
        else                { _offsetFactorL = f; _offsetFactorR = f; }
    }

    // OQ-1 Option A — reported-encoder (legacy MockMotor) noise config.  side:
    // 0 = left, 1 = right, 2 = both.  Gaussian encoder noise (mm per tick) added
    // to the REPORTED encoder accumulator only; the true accumulator is unaffected.
    void setEncoderNoise(int side, float sigma) {  // [mm] noise sigma per tick
        if (side == 0 || side > 1) _encNoiseSigmaL = sigma;
        if (side == 1 || side > 1) _encNoiseSigmaR = sigma;
    }

    // Encoder error injection (ticket 058-001): per-wheel scale error and slip
    // applied to the REPORTED encoder accumulator only.  The true accumulator
    // (_trueEncL / _trueEncR) and chassis pose remain unaffected — ground
    // truth is preserved and golden-TLM parity holds when both are zero (default).
    //
    // side: 0 = left, 1 = right, 2 = both.
    // err:  fractional scale error (0 = perfect, 0.05 = 5% over-report).
    // slip: fraction of motion not registered (0 = perfect, 0.05 = 5% under-report).
    void setEncoderScaleError(int side, float err) {
        if (side == 0 || side > 1) _encScaleErrL = err;
        if (side == 1 || side > 1) _encScaleErrR = err;
    }
    void setEncoderSlip(int side, float fraction) {
        if (side == 0 || side > 1) _encSlipL = fraction;
        if (side == 1 || side > 1) _encSlipR = fraction;
    }

    void setTrackwidth(float trackwidth)  { _trackwidth = trackwidth; }  // [mm]
    void setNominalMaxSpeed(float speed)  { _nominalMaxSpeed = speed; }    // [mm/s]

    // Motor stiction/breakaway gate (072-001): a stateless per-tick PWM
    // dead-zone. |pwm| < stictionPwm => this tick's target velocity is
    // forced to 0, regardless of the wheel's velocity on any previous tick
    // (no "was moving" memory) -- see PhysicsWorld.cpp sub-step A and
    // architecture-update.md Step 4b / Decision 3. Default 0 => the gate
    // condition (fabsf(pwm) < 0) is never true for any representable pwm,
    // so this is a no-op by construction. side: 0 = left, 1 = right,
    // 2 = both (mirrors setEncoderNoise()'s side convention).
    void setStictionPwm(int side, float pwm) {  // [PWM units, 0-100]
        if (side == 0 || side > 1) _stictionPwmL = pwm;
        if (side == 1 || side > 1) _stictionPwmR = pwm;
    }

    // Optional first-order motor response lag (072-001): per-wheel time
    // constant. tau <= 0 (default) takes a no-exp()-call path so the output
    // velocity equals the (possibly stiction-gated) target velocity
    // bit-for-bit; tau > 0 converges the reported velocity toward the target
    // exponentially over successive ticks -- see PhysicsWorld.cpp sub-step A.
    // side: 0 = left, 1 = right, 2 = both.
    void setMotorLag(int side, float tauMs) {  // [ms]
        if (side == 0 || side > 1) _motorLagL = tauMs;
        if (side == 1 || side > 1) _motorLagR = tauMs;
    }

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

    float trueEncL() const { return _trueEncL; }
    float trueEncR() const { return _trueEncR; }

    // OQ-1 Option A — reported (legacy MockMotor encoder-step slip + noise)
    // encoder accumulators.  SimMotor::position() reads these.  Equal to the
    // true accumulators when slip == 0 and noise == 0 (golden-TLM fixture).
    float reportedEncL() const { return _reportedEncL; }
    float reportedEncR() const { return _reportedEncR; }

    // OQ-1 Option A — directly set the reported encoder accumulator (back-compat
    // for sim_set_enc_l/r which resets+rebuilds the legacy MockMotor encoder).
    void setReportedEncoder(int side, float position) {  // [mm]
        if (side == 0) _reportedEncL = position;
        else           _reportedEncR = position;
    }

    // OQ-1 Option A — zero one side's reported encoder accumulator (mirrors
    // MockMotor::resetEncoder, which zeros _encoderMm for that one wheel).  The
    // true accumulator is the ground truth and is NOT reset here.
    void resetReportedEncoder(int side) {
        if (side == 0) _reportedEncL = 0.0f;
        else           _reportedEncR = 0.0f;
    }

    float trueVelL() const { return _trueVelL; }
    float trueVelR() const { return _trueVelR; }

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
    float trackwidth()    const { return _trackwidth; }
    float nominalMaxSpeed() const { return _nominalMaxSpeed; }  // [mm/s]
    float rotationalSlip()  const { return _rotationalSlip; }
    int8_t pwmL()          const { return _pwmL; }
    int8_t pwmR()          const { return _pwmR; }

    // Body-truth scrub (069-002) — see setBodyRotationalScrub()/setBodyLinearScrub().
    float bodyRotationalScrub() const { return _bodyRotationalScrub; }
    float bodyLinearScrub()     const { return _bodyLinearScrub; }

    // Per-wheel offset factor accessors (069-003) — mirror the existing
    // rotationalSlip() accessor shape, for SimCommands' SIMGET row.
    float offsetFactorL() const { return _offsetFactorL; }
    float offsetFactorR() const { return _offsetFactorR; }

    // Per-wheel encoder-report-error accessors (069-004) — mirror the
    // existing setEncoderScaleError()/setEncoderSlip()/setEncoderNoise()
    // setters (058-001 lineage), write-only until now. encoderNoiseL/R()
    // are not named in architecture-update.md's Step 5 getter list, but
    // SUC-002 requires all six per-wheel keys to be SIMGET-able, so they
    // follow the same mirror-the-setter pattern as the other four.
    float encoderScaleErrL() const { return _encScaleErrL; }
    float encoderScaleErrR() const { return _encScaleErrR; }
    float encoderSlipL()     const { return _encSlipL; }
    float encoderSlipR()     const { return _encSlipR; }
    float encoderNoiseL()    const { return _encNoiseSigmaL; }
    float encoderNoiseR()    const { return _encNoiseSigmaR; }

    // Stiction/breakaway + optional lag accessors (072-001) — mirror the
    // setStictionPwm()/setMotorLag() setters above, for SimCommands' SIMGET rows.
    float stictionPwmL() const { return _stictionPwmL; }
    float stictionPwmR() const { return _stictionPwmR; }
    float motorLagL()    const { return _motorLagL; }
    float motorLagR()    const { return _motorLagR; }

private:
    // --- Commanded actuator state ---
    int8_t _pwmL = 0;
    int8_t _pwmR = 0;

    // --- True chassis pose ---
    float _truePoseX = 0.0f;
    float _truePoseY = 0.0f;
    float _truePoseH = 0.0f;

    // --- True per-wheel travel / velocity (unslipped ground truth) ---
    float _trueEncL  = 0.0f;
    float _trueEncR  = 0.0f;
    float _trueVelL = 0.0f;
    float _trueVelR = 0.0f;

    // --- Reported per-wheel travel (OQ-1 Option A: legacy MockMotor model) ---
    // encoder-step slip + Gaussian noise; read by SimMotor::position().
    float _reportedEncL = 0.0f;
    float _reportedEncR = 0.0f;

    // --- True auxiliary sensor values (zero-initialized) ---
    uint16_t _lineRaw[4]   = {0, 0, 0, 0};
    uint16_t _colorRGBC[4] = {0, 0, 0, 0};   // r, g, b, c
    uint16_t _port[4]      = {0, 0, 0, 0};

    // --- Dynamics parameters ---
    float _trackwidth    = kDefaultTrackwidth;
    float _nominalMaxSpeed = kNominalMaxSpeed;  // [mm/s]
    float _rotationalSlip = 0.0f;             // 0/unset → effectiveSlip → 1.0
    float _slipStraight  = 0.0f;
    float _slipTurnExtra = 0.0f;

    // --- Body-truth scrub (069-002) — independent, multiplicative with the
    // effectiveSlip(_rotationalSlip) term above; default 1.0 = no-op. See
    // setBodyRotationalScrub()/setBodyLinearScrub() above for full rationale.
    float _bodyRotationalScrub = 1.0f;
    float _bodyLinearScrub     = 1.0f;

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

    // Encoder error injection (ticket 058-001): per-wheel scale error and slip
    // applied to the REPORTED encoder accumulator only.  Default zero = no effect.
    float _encScaleErrL = 0.0f;  // fractional over/under-report (0 = perfect)
    float _encScaleErrR = 0.0f;
    float _encSlipL     = 0.0f;  // fraction of motion not registered (0 = perfect)
    float _encSlipR     = 0.0f;

    // --- Stiction/breakaway gate + optional lag (072-001) ---
    // _stictionPwmL/R: per-wheel PWM dead-zone threshold, default 0 => gate
    // never fires (no-op). _motorLagL/R: per-wheel first-order response time
    // constant [ms], default 0 => no-op (no exp() call). _lagVelL/R: the
    // filter's persistent state (last output velocity); zeroed in reset().
    // See setStictionPwm()/setMotorLag() above and PhysicsWorld.cpp sub-step A.
    float _stictionPwmL = 0.0f;
    float _stictionPwmR = 0.0f;
    float _motorLagL    = 0.0f;  // [ms]
    float _motorLagR    = 0.0f;  // [ms]
    float _lagVelL      = 0.0f;  // [mm/s] persistent lag-filter state
    float _lagVelR      = 0.0f;  // [mm/s] persistent lag-filter state

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
