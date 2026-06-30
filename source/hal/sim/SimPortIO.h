#pragma once
#include <stdint.h>
#include "hal/capability/IPortIO.h"
#include "PhysicsWorld.h"

/**
 * SimPortIO — observation model for the four sensor/actuator port pins (040-002).
 *
 * Implements IPortIO.  Holds a `const PhysicsWorld&` for forward-compat truth
 * access; for now it reproduces the retired MockPortIO last-written-value store
 * verbatim.  Out-of-range port: setDigital/setAnalog are no-ops; read returns -1.
 *
 * No CODAL dependency.  Compiles with plain clang++ -std=c++11 -I source.
 */
class SimPortIO : public IPortIO {
public:
    static constexpr uint8_t kMinPort = 1;
    static constexpr uint8_t kMaxPort = 4;

    explicit SimPortIO(const PhysicsWorld& plant) : _plant(plant) {}

    // IPortIO interface ------------------------------------------------------
    void setDigital(uint8_t port, bool high) override;
    int  readDigital(uint8_t port) const override;
    void setAnalog(uint8_t port, uint16_t val) override;
    int  readAnalog(uint8_t port) const override;

private:
    const PhysicsWorld& _plant;     // forward-compat truth access

    // Indexed 0..3 for ports 1..4.
    bool     _digital[4] = {false, false, false, false};
    uint16_t _analog[4]  = {0, 0, 0, 0};

    bool valid(uint8_t port) const {
        return port >= kMinPort && port <= kMaxPort;
    }
    int idx(uint8_t port) const { return static_cast<int>(port) - 1; }
};
