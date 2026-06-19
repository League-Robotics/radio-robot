#pragma once
#include <stdint.h>
#include "io/capability/ILineSensor.h"
#include "PhysicsWorld.h"

/**
 * SimLineSensor — observation model for the 4-channel line sensor (040-002).
 *
 * Implements ILineSensor.  Holds a `const PhysicsWorld&` for forward-compat truth
 * access; for now it reproduces the retired MockLineSensor schedule cycling
 * verbatim so every test that drives sim_init_line_sensor / sim_set_line_frozen
 * passes unchanged.  setFrozen(bool) default-off → a fresh sensor is PERFECT.
 *
 * No CODAL dependency.  Compiles with plain clang++ -std=c++11 -I source.
 */
class SimLineSensor : public ILineSensor {
public:
    static constexpr int      kScheduleRows  = 5;
    static constexpr uint32_t kRowDurationMs = 600;  // 5 rows * 600 ms = 3000 ms

    explicit SimLineSensor(const PhysicsWorld& plant) : _plant(plant) {}

    // ILineSensor interface --------------------------------------------------
    bool readValues(uint16_t out[4]) const override;
    bool readNormalized(uint16_t out[4]) override;

    // Sensor interface -------------------------------------------------------
    bool begin() override { _initialized = true; return true; }

    // Simulation control -----------------------------------------------------
    void tick(uint32_t dt_ms);
    void setSchedule(const uint16_t table[][4], int rows);
    void reset() { _elapsedMs = 0; }

    // Frozen sensor (wedge dropout): readValues() returns false so lineRead()
    // never updates lastUpdMs and the TLM freshness gate drops the field.
    void setFrozen(bool frozen) { _frozen = frozen; }

private:
    const PhysicsWorld& _plant;     // forward-compat truth access
    bool     _frozen       = false;

    uint16_t _table[kScheduleRows][4] = {
        {1000, 1000, 1000, 1000},
        {   0, 1000, 1000, 1000},
        {   0,    0,    0,    0},
        {1000, 1000, 1000,    0},
        {1000, 1000, 1000, 1000},
    };
    int      _scheduleRows = kScheduleRows;
    uint32_t _elapsedMs    = 0;

    int currentRow() const;
};
