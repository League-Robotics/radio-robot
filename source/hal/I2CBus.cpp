#include "I2CBus.h"
#include "codal_target_hal.h"   // target_disable_irq() / target_enable_irq()
#include "MicroBit.h"           // system_timer_current_time_us()
#include <cstdio>

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

I2CBus::I2CBus(MicroBitI2C& bus)
    : _bus(bus)
    , _inUse(false)
    , _inFlightAddr(0)
    , _reentryViolations(0)
    , _reentryInFlightAddr(0)
    , _reentryNewAddr(0)
    , _deviceCount(0)
    , _logHead(0)
    , _logTotal(0)
    , _logOn(false)
{
    for (int i = 0; i < kMaxDevices; ++i) {
        _devices[i].addr     = 0;
        _devices[i].txnCount = 0;
        _devices[i].errCount = 0;
        _devices[i].lastErr  = 0;
    }
    for (int i = 0; i < kLogSize; ++i) {
        _log[i] = TxnLog{0, 0, 0, 0, 0, 0, 0};
    }
}

// ---------------------------------------------------------------------------
// I2C forwarding
// ---------------------------------------------------------------------------

int I2CBus::write(uint16_t address, uint8_t* data, int len, bool repeated)
{
    // address is the 8-bit wire address (7-bit addr << 1).
    uint16_t addr7 = (uint16_t)(address >> 1);

    // --- Re-entrancy guard: atomic check-and-set ---
    // Disable IRQs for the MINIMUM window needed to check+set the flag.
    // The full I2C transaction is NOT inside the critical section — only
    // the 3-instruction flag read-modify-write is.
    target_disable_irq();
    bool alreadyInUse = _inUse;
    if (alreadyInUse) {
        // Capture violation: keep the existing _inFlightAddr, record new addr.
        ++_reentryViolations;
        _reentryInFlightAddr = _inFlightAddr;
        _reentryNewAddr      = (uint16_t)address;
    } else {
        _inUse        = true;
        _inFlightAddr = (uint16_t)address;
    }
    target_enable_irq();
    // --- End critical section ---

    int status = _bus.write(address, data, len, repeated);

    // Clear the flag only if we were the one who set it.
    if (!alreadyInUse) {
        _inUse = false;
    }

    record(addr7, status);
    logTxn(addr7, 0, len, data, status);
    return status;
}

int I2CBus::read(uint16_t address, uint8_t* data, int len, bool repeated)
{
    uint16_t addr7 = (uint16_t)(address >> 1);

    // --- Re-entrancy guard: atomic check-and-set ---
    target_disable_irq();
    bool alreadyInUse = _inUse;
    if (alreadyInUse) {
        ++_reentryViolations;
        _reentryInFlightAddr = _inFlightAddr;
        _reentryNewAddr      = (uint16_t)address;
    } else {
        _inUse        = true;
        _inFlightAddr = (uint16_t)address;
    }
    target_enable_irq();
    // --- End critical section ---

    int status = _bus.read(address, data, len, repeated);

    if (!alreadyInUse) {
        _inUse = false;
    }

    record(addr7, status);
    logTxn(addr7, 1, len, data, status);
    return status;
}

// ---------------------------------------------------------------------------
// Transaction log (diagnostic ring buffer)
// ---------------------------------------------------------------------------

void I2CBus::logTxn(uint16_t addr7, uint8_t rw, int len, const uint8_t* data, int status)
{
    if (!_logOn) return;
    TxnLog& e = _log[_logHead];
    e.t_us   = (uint32_t)system_timer_current_time_us();
    e.addr   = addr7;
    e.rw     = rw;
    e.len    = (uint8_t)(len > 255 ? 255 : (len < 0 ? 0 : len));
    e.b0     = (data && len > 0) ? data[0] : 0;
    e.b1     = (data && len > 1) ? data[1] : 0;
    e.status = (int16_t)status;
    _logHead = (_logHead + 1) % kLogSize;
    ++_logTotal;
}

void I2CBus::dumpRecent(void (*fn)(const char*, void*), void* ctx) const
{
    if (!fn || !ctx) return;
    // Walk the ring oldest→newest. If we've wrapped, oldest is at _logHead;
    // otherwise the buffer filled 0.._logHead-1.
    int count = (_logTotal < (uint32_t)kLogSize) ? (int)_logTotal : kLogSize;
    int start = (_logTotal < (uint32_t)kLogSize) ? 0 : _logHead;
    char line[96];
    uint32_t prev_us = 0;
    for (int i = 0; i < count; ++i) {
        const TxnLog& e = _log[(start + i) % kLogSize];
        uint32_t dt = (i == 0) ? 0 : (e.t_us - prev_us);   // us since previous txn
        prev_us = e.t_us;
        snprintf(line, sizeof(line),
                 "I2CLOG +%luus 0x%02X %s len=%u b=%02X,%02X st=%d\r\n",
                 (unsigned long)dt, (unsigned)e.addr, e.rw ? "RD" : "WR",
                 (unsigned)e.len, (unsigned)e.b0, (unsigned)e.b1, (int)e.status);
        fn(line, ctx);
    }
}

// ---------------------------------------------------------------------------
// Per-device statistics
// ---------------------------------------------------------------------------

uint32_t I2CBus::txnCount(uint16_t addr) const
{
    for (int i = 0; i < _deviceCount; ++i) {
        if (_devices[i].addr == addr) return _devices[i].txnCount;
    }
    return 0;
}

uint32_t I2CBus::errCount(uint16_t addr) const
{
    for (int i = 0; i < _deviceCount; ++i) {
        if (_devices[i].addr == addr) return _devices[i].errCount;
    }
    return 0;
}

int I2CBus::lastErr(uint16_t addr) const
{
    for (int i = 0; i < _deviceCount; ++i) {
        if (_devices[i].addr == addr) return _devices[i].lastErr;
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------

void I2CBus::resetStats()
{
    _reentryViolations   = 0;
    _reentryInFlightAddr = 0;
    _reentryNewAddr      = 0;
    _inUse               = false;
    _inFlightAddr        = 0;

    for (int i = 0; i < kMaxDevices; ++i) {
        _devices[i].addr     = 0;
        _devices[i].txnCount = 0;
        _devices[i].errCount = 0;
        _devices[i].lastErr  = 0;
    }
    _deviceCount = 0;

    _logHead  = 0;
    _logTotal = 0;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

int I2CBus::findOrAdd(uint16_t addr7)
{
    // Linear scan: return existing slot if found.
    for (int i = 0; i < _deviceCount; ++i) {
        if (_devices[i].addr == addr7) return i;
    }

    // Allocate a new slot if there is room (leave the last slot as
    // the "other" bucket so we never exceed the array bounds).
    if (_deviceCount < kMaxDevices - 1) {
        int idx = _deviceCount++;
        _devices[idx].addr     = addr7;
        _devices[idx].txnCount = 0;
        _devices[idx].errCount = 0;
        _devices[idx].lastErr  = 0;
        return idx;
    }

    // Table full: use the last slot as the "other" overflow bucket.
    // Mark it addr=0xFFFF so queries for unknown addresses don't
    // collide with real devices (real 7-bit addrs are 0x00..0x7F).
    int overflow = kMaxDevices - 1;
    if (_devices[overflow].addr == 0) {
        _devices[overflow].addr = 0xFFFF;
        _deviceCount = kMaxDevices;
    }
    return overflow;
}

void I2CBus::record(uint16_t addr7, int status)
{
    int idx = findOrAdd(addr7);
    ++_devices[idx].txnCount;
    if (status != MICROBIT_OK) {
        ++_devices[idx].errCount;
        _devices[idx].lastErr = status;
    }
}
