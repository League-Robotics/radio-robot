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
    , _irqGuard(true)
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
    const bool guard = _irqGuard;

    // Always mask IRQs for the flag check-and-set. When _irqGuard is on we KEEP
    // them masked through the whole _bus transaction (nRF52 TWIM errata fix —
    // see I2CBus.h / NRF52I2C::waitForStop); when off, we re-enable before the
    // transaction (original narrow-guard behaviour).
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
    if (!guard) target_enable_irq();

    int status = _bus.write(address, data, len, repeated);

    if (!alreadyInUse) {
        _inUse = false;
    }
    if (guard) target_enable_irq();

    record(addr7, status);
    logTxn(addr7, 0, len, data, status);
    return status;
}

int I2CBus::read(uint16_t address, uint8_t* data, int len, bool repeated)
{
    uint16_t addr7 = (uint16_t)(address >> 1);
    const bool guard = _irqGuard;

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
    if (!guard) target_enable_irq();

    int status = _bus.read(address, data, len, repeated);

    if (!alreadyInUse) {
        _inUse = false;
    }
    if (guard) target_enable_irq();

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
    // For a Nezha WRITE the meaningful byte is the command at frame byte[4]
    // ("FF F9 id dir <CMD> ..." — 0x46=read-angle-request, 0x60=move, 0x47=read-
    // speed). The header byte[0] is always 0xFF, useless. For a READ, byte[0] is
    // the low data byte. So log byte[4] on writes, byte[0] on reads.
    e.b0     = (rw == 0 && len > 4 && data) ? data[4]
             : (data && len > 0) ? data[0] : 0;
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
    // Emit the WHOLE ring as ONE line — multiple lines overflow the async serial
    // TX buffer (~255 B) and garble. One ≤255-char line (like DBG I2C) is safe.
    // Token: <addr><R/W><b0>.<dt_us>  e.g. "10W60.0 10R46.4012 43RA6.250"
    char line[256];
    uint32_t prev_us = 0;
    int pos = snprintf(line, sizeof(line), "I2CLOG ");
    for (int i = 0; i < count; ++i) {
        const TxnLog& e = _log[(start + i) % kLogSize];
        uint32_t dt = (i == 0) ? 0 : (e.t_us - prev_us);
        prev_us = e.t_us;
        int w = snprintf(line + pos, sizeof(line) - pos, "%02X%c%02X.%lu ",
                         (unsigned)e.addr, e.rw ? 'R' : 'W',
                         (unsigned)e.b0, (unsigned long)dt);
        if (w <= 0 || w >= (int)sizeof(line) - pos) break;   // out of room
        pos += w;
    }
    snprintf(line + pos, sizeof(line) - pos, "\r\n");
    fn(line, ctx);
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
