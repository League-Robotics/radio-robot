#pragma once
#include <stdint.h>
#include "RobotState.h"

// ---------------------------------------------------------------------------
// MotionBaseline — snapshot of state variables at motion-command start.
//
// Captured by MotionCommand::start(); passed by const-ref to evaluate().
// ---------------------------------------------------------------------------
struct MotionBaseline {
    uint32_t t0Ms;          // system time at command start, ms
    float    enc0Mm;        // (encLMm + encRMm) * 0.5 at start, mm
    float    heading0Rad;   // pose heading at start, rad
    float    pose0X;        // pose X at start, mm
    float    pose0Y;        // pose Y at start, mm
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
//   KIND     | a              | b          | ax          | sensor | cmp
//   ---------|----------------|------------|-------------|--------|----
//   NONE     | —              | —          | —           | —      | —
//   TIME     | threshold ms   | —          | —           | —      | —
//   DISTANCE | threshold mm   | —          | —           | —      | —
//   HEADING  | target Δrad    | eps rad    | —           | —      | —
//   POSITION | target Y mm    | radius mm  | target X mm | —      | —
//   SENSOR   | threshold      | —          | —           | ch     | GE/LE
//
// POSITION param note: `ax` = target X and `a` = target Y; `b` = radius.
// Although `ax`/`a` (X/Y) seems reversed from convention, it matches the
// architecture field names exactly. Callers should use the named helpers
// makePositionStop(targetX, targetY, radiusMm) to avoid confusion.
// ---------------------------------------------------------------------------
struct StopCondition {
    enum class Kind : uint8_t { NONE, TIME, DISTANCE, HEADING, POSITION, SENSOR };
    enum class Cmp  : uint8_t { GE, LE };

    Kind    kind   = Kind::NONE;
    float   a      = 0.0f;   // primary param (TIME: ms threshold; DISTANCE: mm threshold;
                              //   HEADING: target delta rad; POSITION: target Y mm;
                              //   SENSOR: threshold)
    float   b      = 0.0f;   // secondary param (HEADING: eps rad; POSITION: radius mm)
    float   ax     = 0.0f;   // POSITION only: target X mm
    uint8_t sensor = 0;      // SENSOR: channel selector (index into HardwareState fields)
    Cmp     cmp    = Cmp::GE; // SENSOR: comparison direction

    /**
     * evaluate — test whether this stop condition is satisfied this tick.
     *
     * @param s       Current hardware state.
     * @param now_ms  Current system time, ms.
     * @param base    Motion baseline captured at command start.
     * @return        true when condition fires (command should terminate).
     */
    bool evaluate(const HardwareState& s, uint32_t now_ms,
                  const MotionBaseline& base) const;
};

// ---------------------------------------------------------------------------
// Factory helpers — create named StopConditions for readability.
// ---------------------------------------------------------------------------

/** Stop after durationMs milliseconds. */
inline StopCondition makeTimeStop(float durationMs)
{
    StopCondition c;
    c.kind = StopCondition::Kind::TIME;
    c.a    = durationMs;
    return c;
}

/** Stop when average encoder travel reaches distanceMm (absolute value). */
inline StopCondition makeDistanceStop(float distanceMm)
{
    StopCondition c;
    c.kind = StopCondition::Kind::DISTANCE;
    c.a    = distanceMm;
    return c;
}

/** Stop when heading reaches headingDeltaRad within epsRad tolerance. */
inline StopCondition makeHeadingStop(float headingDeltaRad, float epsRad)
{
    StopCondition c;
    c.kind = StopCondition::Kind::HEADING;
    c.a    = headingDeltaRad;
    c.b    = epsRad;
    return c;
}

/**
 * Stop when the robot pose is within radiusMm of (targetX, targetY).
 *
 * Named parameters avoid the `ax` = X, `a` = Y ambiguity in raw field access.
 */
inline StopCondition makePositionStop(float targetX, float targetY, float radiusMm)
{
    StopCondition c;
    c.kind = StopCondition::Kind::POSITION;
    c.ax   = targetX;   // target X, mm
    c.a    = targetY;   // target Y, mm
    c.b    = radiusMm;  // arrival radius, mm
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
