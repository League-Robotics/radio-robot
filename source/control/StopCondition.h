#pragma once
#include <stdint.h>
#include "Inputs.h"

// ---------------------------------------------------------------------------
// MotionBaseline — snapshot of state variables at motion-command start.
//
// Captured by MotionCommand::start(); passed by const-ref to evaluate().
// ---------------------------------------------------------------------------
struct MotionBaseline {
    uint32_t t0;            // [ms] system time at command start
    float    enc0;          // [mm] (encL + encR) * 0.5 at start
    float    encDiff0;      // [mm] (encR - encL) at start — for ROTATION stop
    float    heading0;      // [rad] pose heading at start
    float    pose0X;        // pose X at start, mm
    float    pose0Y;        // pose Y at start, mm

    // Commanded-direction signs (072-002), captured by MotionCommand::start()
    // from the command's commanded v/omega at the moment start() is called.
    // ±1.0, or 0.0 if the commanded velocity component is exactly zero.
    // DISTANCE/ROTATION multiply the raw (unsigned-by-nature) encoder delta
    // by these signs to gate on travel in the COMMANDED direction instead of
    // a direction-blind fabsf() magnitude — see architecture-update.md
    // Decision 1 (why these live here, not as a StopCondition param or a
    // call-site pre-negation trick). SAFETY_MARGIN also reads vSign.
    //
    // 0.0 (unset) is not just the degenerate MotionCommand v/omega==0 case:
    // it is also the value every OTHER caller of these Kinds' evaluate() gets
    // by construction when it builds its OWN MotionBaseline with no notion of
    // a "commanded direction" at all (e.g. HaltController's HALT DIST watches
    // — a deliberately direction-agnostic magnitude watch, independent of
    // whatever verb happens to be driving at registration time). DISTANCE and
    // ROTATION's evaluate() branches treat 0.0 as "fall back to the original
    // undirected |delta| magnitude" for exactly this reason — a caller that
    // never had a commanded direction to report gets the pre-072-002 behavior
    // back, rather than a signed gate that would silently lock it to one
    // arbitrary direction.
    float    vSign;          // ±1.0 (0.0 = no commanded direction; magnitude fallback)
    float    omegaSign;      // ±1.0 (0.0 = no commanded direction; magnitude fallback)
};

// ---------------------------------------------------------------------------
// StopCondition — POD tagged struct for a single termination condition.
//
// A MotionCommand owns a fixed array of up to kMaxStopConds of these.
// evaluate() is called once per tick; returns true when the condition fires.
// Conditions are OR-combined: the first one to fire terminates the command.
//
// Kind + Cmp enums are uint8_t to keep the struct compact (no padding waste).
//
// Param layout per Kind (from architecture-update.md §StopCondition):
//
//   KIND          | a              | b          | ax          | sensor | cmp
//   --------------|----------------|------------|-------------|--------|----
//   NONE          | —              | —          | —           | —      | —
//   TIME          | threshold ms   | —          | —           | —      | —
//   DISTANCE      | threshold mm   | —          | —           | —      | —
//   HEADING       | target Δrad    | eps rad    | —           | —      | —
//   POSITION      | target Y mm    | radius mm  | target X mm | —      | —
//   SENSOR        | threshold      | —          | —           | ch     | GE/LE
//   SAFETY_MARGIN | margin mm      | —          | —           | —      | —
//
// POSITION param note: `ax` = target X and `a` = target Y; `b` = radius.
// Although `ax`/`a` (X/Y) seems reversed from convention, it matches the
// architecture field names exactly. Callers should use the named helpers
// makePositionStop(targetX, targetY, radius) to avoid confusion.
//
// DISTANCE/ROTATION/SAFETY_MARGIN direction-awareness (072-002): these three
// Kinds all gate on the SIGNED delta (raw * base.vSign or raw * base.omegaSign),
// not a direction-blind fabsf() magnitude — see MotionBaseline's vSign/omegaSign
// doc comment above.
// ---------------------------------------------------------------------------
struct StopCondition {
    enum class Kind : uint8_t {
        NONE, TIME, DISTANCE, HEADING, POSITION, SENSOR,
        COLOR,         // fires when HSV distance from target <= ax
        LINE_ANY,      // fires when any line[0..3] satisfies threshold/cmp
        ROTATION,      // fires when per-wheel encoder arc (from the differential) >= a
        SAFETY_MARGIN  // fires when signed travel crosses -a (runaway safety net, D only)
    };
    enum class Cmp  : uint8_t { GE, LE };

    Kind    kind   = Kind::NONE;
    float   a      = 0.0f;   // primary param (TIME: ms threshold; DISTANCE: mm threshold;
                              //   HEADING: target delta rad; POSITION: target Y mm;
                              //   SENSOR/LINE_ANY: threshold; COLOR: target hue [0,360))
    float   b      = 0.0f;   // secondary param (HEADING: eps rad; POSITION: radius mm;
                              //   COLOR: target saturation [0,1])
    float   ax     = 0.0f;   // POSITION only: target X mm; COLOR: HSV distance threshold
    float   ay     = 0.0f;   // COLOR only: target value/brightness [0,1]
    uint8_t sensor = 0;      // SENSOR: channel selector (index into HardwareState fields)
    Cmp     cmp    = Cmp::GE; // SENSOR/LINE_ANY: comparison direction

    /**
     * evaluate — test whether this stop condition is satisfied this tick.
     *
     * @param s     Current hardware state.
     * @param now   Current system time, ms.
     * @param base  Motion baseline captured at command start.
     * @return      true when condition fires (command should terminate).
     */
    bool evaluate(const HardwareState& s, uint32_t now,   // [ms]
                  const MotionBaseline& base) const;
};

// ---------------------------------------------------------------------------
// Factory helpers — create named StopConditions for readability.
// ---------------------------------------------------------------------------

/** Stop after duration milliseconds. */
inline StopCondition makeTimeStop(float duration)   // [ms]
{
    StopCondition c;
    c.kind = StopCondition::Kind::TIME;
    c.a    = duration;
    return c;
}

/** Stop when average encoder travel reaches distance (absolute value). */
inline StopCondition makeDistanceStop(float distance)   // [mm]
{
    StopCondition c;
    c.kind = StopCondition::Kind::DISTANCE;
    c.a    = distance;
    return c;
}

/** Stop when per-wheel encoder arc (|Δ(encR-encL)|/2) reaches arc.
 *  Used by RT / beginRotation for spin-in-place dead reckoning. */
inline StopCondition makeRotationStop(float arc)   // [mm]
{
    StopCondition c;
    c.kind = StopCondition::Kind::ROTATION;
    c.a    = arc;
    return c;
}

/**
 * Stop (safety-class) when signed travel crosses more than margin mm
 * NEGATIVE relative to the commanded direction — i.e. the robot is
 * demonstrably moving the WRONG way during a directed D (072-002).
 *
 * `margin` is a positive threshold, mm. Fires when
 * (raw traveled) * base.vSign <= -margin.
 *
 * MotionCommand::tick() special-cases this Kind: forced HARD teardown and
 * the emitted EVT label forced to "EVT safety_stop" (reason=runaway),
 * regardless of the command's configured stop style / done-EVT label.
 */
inline StopCondition makeSafetyMarginStop(float margin)   // [mm]
{
    StopCondition c;
    c.kind = StopCondition::Kind::SAFETY_MARGIN;
    c.a    = margin;
    return c;
}

/** Stop when heading reaches headingDelta within eps tolerance. */
inline StopCondition makeHeadingStop(float headingDelta, float eps)   // [rad], [rad]
{
    StopCondition c;
    c.kind = StopCondition::Kind::HEADING;
    c.a    = headingDelta;
    c.b    = eps;
    return c;
}

/**
 * Stop when the robot pose is within radius of (targetX, targetY).
 *
 * Named parameters avoid the `ax` = X, `a` = Y ambiguity in raw field access.
 */
inline StopCondition makePositionStop(float targetX, float targetY, float radius)   // [mm]
{
    StopCondition c;
    c.kind = StopCondition::Kind::POSITION;
    c.ax   = targetX;   // target X, mm
    c.a    = targetY;   // target Y, mm
    c.b    = radius;    // arrival radius, mm
    return c;
}

/**
 * Stop when sensor channel compares against threshold per cmp.
 *
 * SENSOR channel selector constants (index into HardwareState):
 *   0–3: line[0..3]
 *   4:   colorR
 *   5:   colorG
 *   6:   colorB
 *   7:   colorC
 *   8–11: analogIn[0..3]
 */
inline StopCondition makeSensorStop(uint8_t channel, float threshold,
                                     StopCondition::Cmp cmp)
{
    StopCondition c;
    c.kind   = StopCondition::Kind::SENSOR;
    c.a      = threshold;
    c.sensor = channel;
    c.cmp    = cmp;
    return c;
}

/**
 * Stop when the color sensor HSV reading is within distThreshold of the
 * target HSV colour (h [0,360), s [0,1], v [0,1]).
 *
 * The match uses wrap-aware hue distance:
 *   dist = sqrt(hDist^2 + (s_sensor - s_target)^2 + (v_sensor - v_target)^2)
 *   fires when dist <= distThreshold.
 *
 * Param layout:
 *   a  = target hue, degrees [0,360)
 *   b  = target saturation [0,1]
 *   ay = target value/brightness [0,1]
 *   ax = HSV distance threshold
 */
inline StopCondition makeColorStop(float targetH, float targetS, float targetV,
                                    float distThreshold)
{
    StopCondition c;
    c.kind = StopCondition::Kind::COLOR;
    c.a    = targetH;
    c.b    = targetS;
    c.ay   = targetV;
    c.ax   = distThreshold;
    return c;
}

/**
 * Stop when ANY of line[0..3] satisfies the threshold/cmp condition.
 *
 * cmp = GE: fires when any channel >= threshold.
 * cmp = LE: fires when any channel <= threshold.
 */
inline StopCondition makeLineAnyStop(float threshold, StopCondition::Cmp cmp)
{
    StopCondition c;
    c.kind = StopCondition::Kind::LINE_ANY;
    c.a    = threshold;
    c.cmp  = cmp;
    return c;
}
