// sensors_api.cpp — extern "C" C-ABI shims for the Sensors subsystem facade.
//
// (ticket 057-003) Provides an opaque SensorsHandle that owns a self-contained
// Sensors subsystem constructed on SimHardware, with its own local HardwareState.
// Python tests (test_sensors_subsystem.py) load this via ctypes and call these
// functions directly.
//
// Construction order:
//   1. cfg         — RobotConfig from defaultRobotConfig()
//   2. hal         — SimHardware(cfg) — owns PhysicsWorld + Sim* devices
//   3. hw          — HardwareState (defaultInputs(cfg).actual)
//   4. line        — subsystems::LineSensor(hal.lineSensor(), hw, cfg)
//   5. color       — subsystems::ColorSensor(hal.colorSensor(), hw, cfg)
//   6. sensors     — subsystems::Sensors(line, color, hw)
//
// The sim sensors start un-initialized (begin() not called). Call
// sensors_api_init_sensors() to call begin() on both underlying devices,
// matching the sim_init_line_sensor / sim_init_color_sensor pattern.

#include "types/Config.h"
#include "types/Inputs.h"
#include "hal/sim/SimHardware.h"
#include "subsystems/sensors/LineSensor.h"
#include "subsystems/sensors/ColorSensor.h"
#include "subsystems/sensors/Sensors.h"
#include "subsystems/sensors/SensorsConfig.h"
#include "messages/sensors.h"

// ---------------------------------------------------------------------------
// SensorsHandle — opaque handle owning a self-contained Sensors subsystem.
// ---------------------------------------------------------------------------
struct SensorsHandle {
    RobotConfig                cfg;
    SimHardware                hal;
    HardwareState              hw;
    subsystems::LineSensor     line;
    subsystems::ColorSensor    color;
    subsystems::Sensors        sensors;

    SensorsHandle()
        : cfg(defaultRobotConfig())
        , hal(cfg)
        , hw(defaultInputs(cfg).actual)
        , line(hal.lineSensor(), hw, cfg)
        , color(hal.colorSensor(), hw, cfg)
        , sensors(line, color, hw)
    {}
};

extern "C" {

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

void* sensors_api_create()
{
    return new SensorsHandle();
}

void sensors_api_destroy(void* h)
{
    delete static_cast<SensorsHandle*>(h);
}

// ---------------------------------------------------------------------------
// Init — call begin() on both underlying sim sensor devices so that
// updateInputs() considers them initialized (is_initialized() → true).
// ---------------------------------------------------------------------------
void sensors_api_init_sensors(void* h)
{
    SensorsHandle* s = static_cast<SensorsHandle*>(h);
    s->hal.simLineSensor().begin();
    s->hal.simColorSensor().begin();
}

// ---------------------------------------------------------------------------
// Configure — apply lag_line_ms and lag_color_ms from projection functions.
// sensors_api_configure_lag(h, lag_ms) sets BOTH line and color lags to the
// same value (convenience for the lag-test in test_sensors_subsystem.py).
// ---------------------------------------------------------------------------
void sensors_api_configure(void* h, uint32_t lag_line_ms, uint32_t lag_color_ms)
{
    SensorsHandle* s = static_cast<SensorsHandle*>(h);
    msg::LineSensorConfig  lc = subsystems::toLineSensorConfig(s->cfg);
    msg::ColorSensorConfig cc = subsystems::toColorSensorConfig(s->cfg);
    lc.lag_line  = lag_line_ms;
    cc.lag_color = lag_color_ms;
    s->sensors.configure(lc, cc);
}

// Convenience: set lag for both sensors to the same value.
void sensors_api_configure_lag(void* h, uint32_t lag_ms)
{
    sensors_api_configure(h, lag_ms, lag_ms);
}

// ---------------------------------------------------------------------------
// Tick
// ---------------------------------------------------------------------------
void sensors_api_tick(void* h, uint32_t now_ms)
{
    SensorsHandle* s = static_cast<SensorsHandle*>(h);
    // Advance the SimHardware clock so Sim* sensors update their schedules.
    s->hal.tick(now_ms);
    s->sensors.tick(now_ms);
}

// ---------------------------------------------------------------------------
// State reads — delegate to sensors.state() getters.
// ---------------------------------------------------------------------------

int sensors_api_line_connected(void* h)
{
    return static_cast<SensorsHandle*>(h)->sensors.state().line.get_connected() ? 1 : 0;
}

int sensors_api_color_connected(void* h)
{
    return static_cast<SensorsHandle*>(h)->sensors.state().color.get_connected() ? 1 : 0;
}

// normalized_[idx]: raw line sensor value at channel idx (0..3).
// (The sim sensor returns the same values from readValues / readNormalized.)
uint32_t sensors_api_line_normalized(void* h, int idx)
{
    const subsystems::SensorsState& st =
        static_cast<SensorsHandle*>(h)->sensors.state();
    if (idx < 0 || idx >= 4) return 0;
    return st.line.normalized()[idx];
}

// raw_[idx]: raw line sensor value (same as normalized in this projection).
uint32_t sensors_api_line_raw(void* h, int idx)
{
    const subsystems::SensorsState& st =
        static_cast<SensorsHandle*>(h)->sensors.state();
    if (idx < 0 || idx >= 4) return 0;
    return st.line.raw()[idx];
}

uint32_t sensors_api_color_r(void* h)
{
    return static_cast<SensorsHandle*>(h)->sensors.state().color.get_r();
}

uint32_t sensors_api_color_g(void* h)
{
    return static_cast<SensorsHandle*>(h)->sensors.state().color.get_g();
}

uint32_t sensors_api_color_b(void* h)
{
    return static_cast<SensorsHandle*>(h)->sensors.state().color.get_b();
}

uint32_t sensors_api_color_c(void* h)
{
    return static_cast<SensorsHandle*>(h)->sensors.state().color.get_c();
}

} // extern "C"
