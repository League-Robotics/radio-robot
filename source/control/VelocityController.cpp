// VelocityController.cpp — per-wheel PI + feed-forward velocity controller.
//
// See docs/kinematics-model.md §2.1 for control law derivation.
// Sprint 010, Ticket 003.  Sprint 049-003: integral/anti-windup core
// replaced with a composed cmon-pid backcalculation_t<pid_bwe_exposed>.

#include "VelocityController.h"
#include <math.h>

// Nominal time step used for initial cmon-pid gain configuration (seconds).
// The actual dt_s passed to update() is used for all real ticks.
// 24 ms is the typical real-time loop period.
static constexpr float kNominalDt = 0.024f;

// Tiny derivative filter time-constant (s) for ParallelPid().
// ParallelPid() requires Tf > 0 to avoid division by zero.
// With Kd=0 and Tf this small:
//   A1 = Tf/(h+Tf) ≈ 0           (D-filter pole near zero)
//   A3 = -(Ki*Tf - Kp)*h/(h+Tf) → Kp   (D register carries kP*e)
//   B3 = Ki*h                     (integrator step = Ki*dt)
//   C3 = Kd/Tf = 0               (no direct D path)
// So Update(e) = D_new + I_new = Kp*e + (I_old + Ki*h*e)
// = Kp*e + I_new.  The PI output uses the UPDATED integrator.
// The wrapper reads I_old before Update() and constructs output with I_old
// to match the original hand-rolled order exactly.
static constexpr float kTinyTf = 1e-6f;

// ---------------------------------------------------------------------------
// _configurePid — set up _pid coefficients from current public gain fields.
//
// ParallelPid(h, Kp, Ki, Kd=0, Tf=kTinyTf):
//   Pure PI (Kd=0).  Kp and Ki map directly from public fields.
//
// Backcalculation(min, max, Tw, h):
//   min/max = ±iMax  (PI-output limit that bounds the integrator).
//   Tw      = 1/kAw  (tracking time; kAw=0 → Tw large → cW≈0, no bleed).
//
// Called from constructor and reconfigurePid() (via updateVelGains()).
// Also called at the start of each update() with the actual dt_s so the
// B3 = Ki*h coefficient reflects the true elapsed time.
// ---------------------------------------------------------------------------
void VelocityController::_configurePid(float dt_s)
{
    if (dt_s <= 0.0f) dt_s = kNominalDt;
    float Tw = (kAw > 0.0f) ? (1.0f / kAw) : 1e6f;
    _pid.ParallelPid(dt_s, kP, kI, 0.0f, kTinyTf);
    _pid.Backcalculation(-iMax, iMax, Tw, dt_s);
}

VelocityController::VelocityController(float kFF_, float kP_, float kI_,
                                         float iMax_, float minWheelSpeed_, float kAw_)
    : kFF(kFF_), kP(kP_), kI(kI_), iMax(iMax_), minWheelSpeed(minWheelSpeed_),
      kAw(kAw_), integral(0.0f)
{
    _pid.SteadyStateInit(0.0f);
    _configurePid(kNominalDt);
}

float VelocityController::update(float setpoint, float measured, float dt_s)
{
    if (dt_s <= 0.0f) return 0.0f;

    // Error: positive when measured is slower than setpoint.
    float err = setpoint - measured;

    // Feed-forward: drives proportional to |setpoint|, signed by setpoint direction.
    float spAbs  = fabsf(setpoint);
    float spSign = (setpoint >= 0.0f) ? 1.0f : -1.0f;
    float ff     = kFF * spAbs;

    // Reconfigure cmon-pid with actual dt_s so B3 = Ki*h is exact each tick.
    _configurePid(dt_s);

    // Read the integrator BEFORE advancing so we compute output with I_old —
    // matching the hand-rolled ordering (rawPwm uses old integral; then update).
    float I_old = _pid.getI();

    // Raw (pre-clamp) command using the OLD integrator value, then actual output.
    float rawPwm = spSign * ff + kP * err + I_old;
    float output = clamp(rawPwm, -100.0f, 100.0f);

    // Advance integrator via cmon-pid (backcalculation_t::Update handles the
    // anti-windup bleed when PI output exceeds ±iMax).
    // Deadband: don't advance the integrator at very low commanded speed.
    bool inDeadband = (spAbs < minWheelSpeed);
    if (!inDeadband) {
        // Drive backcalculation_t with the error.  It calls pid_bwe::Update(err)
        // which steps I by B3*err = Ki*dt*err, then applies anti-windup correction
        // when I+Kp*err is outside [-iMax, iMax].
        _pid.Update(err);
    } else {
        // Frozen: keep I where it is, just refresh the D filter at zero error
        // so the filter state stays consistent.
        // ReInit(e=0, u=I_old) keeps I at I_old with the filter consistent.
        _pid.ReInit(0.0f, I_old);
    }

    // Sync public integral field from updated cmon-pid state.
    integral = _pid.getI();

    return output;
}

void VelocityController::reset()
{
    _pid.SteadyStateInit(0.0f);
    integral = 0.0f;
}

void VelocityController::reconfigurePid()
{
    _configurePid(kNominalDt);
}

float VelocityController::clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
