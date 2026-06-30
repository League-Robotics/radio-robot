// =============================================================================
// Sensors.cpp — Sensors subsystem facade implementation (ticket 057-003)
//
// C++11 / -fno-rtti / -fno-exceptions / no heap / no STL containers.
// =============================================================================

#include "Sensors.h"

namespace subsystems {

// ---------------------------------------------------------------------------
// tick(now) — drive both sensors when their respective lag gates fire.
//
// Line lag gate: mirrors LineSensor::periodic() but owns the timer here so
// we can call updateInputs() directly without a LoopTickState dependency.
// After a successful updateInputs() call, copies the HardwareState line[]
// fields into _state.line.
//
// Color lag gate: same pattern for ColorSensor / colorR/G/B/C.
// ---------------------------------------------------------------------------
void Sensors::tick(uint32_t now)
{
    // ---- Line sensor --------------------------------------------------------
    uint32_t lagLine = _lineCfg.get_lag_line_ms();
    if (lagLine > 0 &&
        (int32_t)(now - _lastLineTick) >= (int32_t)lagLine)
    {
        _line.updateInputs(now);
        _lastLineTick = now;

        // Project HardwareState → msg::LineSensorState.
        // connected: true if the underlying ILineSensor was initialized and
        //            has returned at least one successful reading.
        // raw_[] / normalized_[]: copy the four 16-bit channel values as
        //   uint32_t.  The existing subsystems::LineSensor writes the same
        //   values to both (SimLineSensor::readNormalized delegates to
        //   readValues), so raw and normalized are identical here.
        _state.line.connected = _hw.lineVS.valid;
        for (int i = 0; i < 4; ++i) {
            _state.line.raw_[i]        = static_cast<uint32_t>(_hw.line[i]);
            _state.line.normalized_[i] = static_cast<uint32_t>(_hw.line[i]);
        }
        _state.line.raw_count          = 4;
        _state.line.normalized_count   = 4;
        _state.line.stamp.valid        = _hw.lineVS.valid;
        _state.line.stamp.last_upd_ms  = _hw.lineVS.lastUpdMs;
    }

    // ---- Color sensor -------------------------------------------------------
    uint32_t lagColor = _colorCfg.get_lag_color_ms();
    if (lagColor > 0 &&
        (int32_t)(now - _lastColorTick) >= (int32_t)lagColor)
    {
        _color.updateInputs(now);
        _lastColorTick = now;

        // Project HardwareState → msg::ColorSensorState.
        _state.color.connected      = _hw.colorVS.valid;
        _state.color.r              = static_cast<uint32_t>(_hw.colorR);
        _state.color.g              = static_cast<uint32_t>(_hw.colorG);
        _state.color.b              = static_cast<uint32_t>(_hw.colorB);
        _state.color.c              = static_cast<uint32_t>(_hw.colorC);
        _state.color.stamp.valid       = _hw.colorVS.valid;
        _state.color.stamp.last_upd_ms = _hw.colorVS.lastUpdMs;
    }
}

// ---------------------------------------------------------------------------
// configure(lc, cc) — store both configs. The next tick() picks up the
// updated lag values and thresholds.
// ---------------------------------------------------------------------------
void Sensors::configure(const msg::LineSensorConfig& lc,
                        const msg::ColorSensorConfig& cc)
{
    _lineCfg  = lc;
    _colorCfg = cc;
}

}  // namespace subsystems
