#pragma once
#include <stdint.h>
#include "hal/capability/IColorSensor.h"
#include "PhysicsWorld.h"

/**
 * SimColorSensor — observation model for the RGBC color sensor (040-002).
 *
 * Implements IColorSensor.  Holds a `const PhysicsWorld&` for forward-compat truth
 * access; for now it reproduces the retired MockColorSensor schedule cycling
 * verbatim so every test that drives sim_init_color_sensor / sim_set_color_frozen
 * passes unchanged.  setFrozen(bool) default-off → a fresh sensor is PERFECT.
 *
 * No CODAL dependency.  Compiles with plain clang++ -std=c++11 -I source.
 */
class SimColorSensor : public IColorSensor {
public:
    static constexpr int      kScheduleRows  = 4;
    static constexpr uint32_t kRowDurationMs = 250;  // 4 rows * 250 ms = 1000 ms

    explicit SimColorSensor(const PhysicsWorld& plant) : _plant(plant) {}

    // IColorSensor interface -------------------------------------------------
    bool readRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) override;
    bool pollRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) override;

    // Sensor interface -------------------------------------------------------
    bool begin() override { _initialized = true; return true; }

    // Simulation control -----------------------------------------------------
    void tick(uint32_t dt_ms);
    void setSchedule(const uint16_t table[][4], int rows);
    void reset() { _elapsedMs = 0; }

    // Frozen sensor (wedge dropout): pollRGBC() returns false so colorRead()
    // never updates lastUpdMs and the TLM freshness gate drops the field.
    void setFrozen(bool frozen) { _frozen = frozen; }

private:
    const PhysicsWorld& _plant;     // forward-compat truth access
    bool     _frozen       = false;

    uint16_t _table[kScheduleRows][4] = {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
    };
    int      _scheduleRows = kScheduleRows;
    uint32_t _elapsedMs    = 0;

    int currentRow() const;
    void fillOutputs(int row, uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c) const;
};
