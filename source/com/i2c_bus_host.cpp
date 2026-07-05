// i2c_bus_host.cpp — HOST_BUILD scripted-fake implementation of I2CBus.
//
// Compiled ONLY when HOST_BUILD is defined, and NEVER linked alongside the
// real source/com/i2c_bus.cpp (both files define the same I2CBus:: symbols;
// linking both into one binary is a build-configuration error, not a
// supported dual-build). This gives ticket 004's HAL scheduler tests (and
// any future host test) a dependency-free I2CBus to script against — no
// MicroBitI2C, no CODAL, no wall clock.
//
// Scripting model: a test pre-loads expected (address, status) tuples for
// writes and (address, bytes, status) tuples for reads via scriptWrite()/
// scriptRead(), in the exact order production code is expected to call
// write()/read(). Each call pops the next scripted entry (FIFO); an
// unscripted call, or one whose address doesn't match, returns a distinct
// mismatch status instead of crashing the test process (see kScriptMismatch
// below) — this fails the specific assertion the test cares about (a
// status/errCount() check) rather than aborting the whole harness.
//
// Clock: HOST_BUILD has no wall clock, so the exact lastEnd/readyAt
// clearance-timer bookkeeping in write()/read() (identical logic to the
// real fork — see i2c_bus.cpp) runs against a static, test-settable
// counter (I2CBus::setClock()/advanceClock()/clock() — architecture-
// update.md's "static test-settable counter" shape). A live entry-spin
// self-advances the counter by 1us per iteration so a scripted preClear/
// postClear deadline always terminates even if a test forgets to advance
// the clock itself — this ticket's own tests never trigger a live spin
// (every scripted scenario either uses the defaults, or advances the clock
// explicitly before the next call), so the self-advance is a safety net for
// future callers, not exercised behavior this ticket asserts on.
#include "i2c_bus.h"
#include <cstdio>

namespace {
// Distinct from any real CODAL status — a scripted call with no queued
// script, or a wrong-address script, returns this rather than silently
// returning "OK" or crashing.
constexpr int kScriptMismatch = -100;

// HOST_BUILD fake clock — see the "Clock" section above. A single counter
// shared by every I2CBus instance in the process; starts at 0.
uint64_t g_fakeClockUs = 0;   // [us]
}  // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

I2CBus::I2CBus()
    : _inUse(false)
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
        _devices[i].lastEnd  = 0;
        _devices[i].readyAt  = 0;
    }
    for (int i = 0; i < kLogSize; ++i) {
        _log[i] = TxnLog{0, 0, 0, 0, 0, 0, 0};
    }
}

// ---------------------------------------------------------------------------
// Clock
// ---------------------------------------------------------------------------

uint64_t I2CBus::clockUs()
{
    return g_fakeClockUs;
}

void I2CBus::setClock(uint64_t us)
{
    g_fakeClockUs = us;
}

void I2CBus::advanceClock(uint64_t us)
{
    g_fakeClockUs += us;
}

uint64_t I2CBus::clock()
{
    return g_fakeClockUs;
}

// ---------------------------------------------------------------------------
// I2C forwarding — scripted fake, no MicroBitI2C
// ---------------------------------------------------------------------------

int I2CBus::write(uint16_t address, uint8_t* data, int len, bool repeated,
                   uint32_t preClear, uint32_t postClear)
{
    (void)repeated;
    (void)data;
    (void)len;

    // address is the 8-bit wire address (7-bit addr << 1) — same convention
    // as the real fork.
    uint16_t addr7 = static_cast<uint16_t>(address >> 1);

    // Same lazy-clearance entry spin as the real fork (i2c_bus.cpp), against
    // the fake clock instead of system_timer_current_time_us(). See the
    // file header for why this self-advances rather than truly blocking.
    int idx = findOrAdd(addr7);
    uint64_t entryDeadline = _devices[idx].readyAt;
    uint64_t preDeadline = _devices[idx].lastEnd + static_cast<uint64_t>(preClear);
    if (preDeadline > entryDeadline) entryDeadline = preDeadline;
    while (clockUs() < entryDeadline) { advanceClock(1); }

    int status = kScriptMismatch;
    if (!_scriptedWrites.empty()) {
        ScriptedWrite expected = _scriptedWrites.front();
        _scriptedWrites.pop_front();
        status = (expected.addr == address) ? expected.status : kScriptMismatch;
    }

    record(addr7, status);
    logTxn(addr7, 0, len, data, status);

    _devices[idx].lastEnd = clockUs();
    _devices[idx].readyAt = _devices[idx].lastEnd + static_cast<uint64_t>(postClear);

    return status;
}

int I2CBus::read(uint16_t address, uint8_t* data, int len, bool repeated,
                  uint32_t preClear, uint32_t postClear)
{
    (void)repeated;

    uint16_t addr7 = static_cast<uint16_t>(address >> 1);

    int idx = findOrAdd(addr7);
    uint64_t entryDeadline = _devices[idx].readyAt;
    uint64_t preDeadline = _devices[idx].lastEnd + static_cast<uint64_t>(preClear);
    if (preDeadline > entryDeadline) entryDeadline = preDeadline;
    while (clockUs() < entryDeadline) { advanceClock(1); }

    int status = kScriptMismatch;
    if (!_scriptedReads.empty()) {
        ScriptedRead expected = _scriptedReads.front();
        _scriptedReads.pop_front();
        if (expected.addr == address) {
            status = expected.status;
            int copyLen = (len < static_cast<int>(expected.data.size()))
                              ? len
                              : static_cast<int>(expected.data.size());
            for (int i = 0; i < copyLen; ++i) {
                data[static_cast<size_t>(i)] = expected.data[static_cast<size_t>(i)];
            }
        }
    }

    record(addr7, status);
    logTxn(addr7, 1, len, data, status);

    _devices[idx].lastEnd = clockUs();
    _devices[idx].readyAt = _devices[idx].lastEnd + static_cast<uint64_t>(postClear);

    return status;
}

// ---------------------------------------------------------------------------
// Scripting
// ---------------------------------------------------------------------------

void I2CBus::scriptWrite(uint16_t address, int status)
{
    _scriptedWrites.push_back(ScriptedWrite{address, status});
}

void I2CBus::scriptRead(uint16_t address, const uint8_t* data, int len, int status)
{
    ScriptedRead entry;
    entry.addr = address;
    entry.status = status;
    if (data && len > 0) {
        entry.data.assign(data, data + len);
    }
    _scriptedReads.push_back(entry);
}

// ---------------------------------------------------------------------------
// Transaction log (diagnostic ring buffer) — identical logic to the real
// fork (i2c_bus.cpp), against the fake clock.
// ---------------------------------------------------------------------------

void I2CBus::logTxn(uint16_t addr7, uint8_t rw, int len, const uint8_t* data, int status)
{
    if (!_logOn) return;
    TxnLog& e = _log[_logHead];
    e.t      = (uint32_t)clockUs();   // [us]
    e.addr   = addr7;
    e.rw     = rw;
    e.len    = (uint8_t)(len > 255 ? 255 : (len < 0 ? 0 : len));
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
    int count = (_logTotal < (uint32_t)kLogSize) ? (int)_logTotal : kLogSize;
    int start = (_logTotal < (uint32_t)kLogSize) ? 0 : _logHead;
    char line[256];
    uint32_t prevTime = 0;   // [us]
    int pos = snprintf(line, sizeof(line), "I2CLOG ");
    for (int i = 0; i < count; ++i) {
        const TxnLog& e = _log[(start + i) % kLogSize];
        uint32_t dt = (i == 0) ? 0 : (e.t - prevTime);
        prevTime = e.t;
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
// Per-device statistics — identical logic to the real fork.
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
// Lazy per-device clearance timers — non-spinning peek
// ---------------------------------------------------------------------------

bool I2CBus::clear(uint16_t addr7) const
{
    for (int i = 0; i < _deviceCount; ++i) {
        if (_devices[i].addr == addr7) {
            return clockUs() >= _devices[i].readyAt;
        }
    }
    return true;   // never transacted with — nothing to wait for
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
        _devices[i].lastEnd  = 0;
        _devices[i].readyAt  = 0;
    }
    _deviceCount = 0;

    _logHead  = 0;
    _logTotal = 0;
}

// ---------------------------------------------------------------------------
// Private helpers — identical logic to the real fork.
// ---------------------------------------------------------------------------

int I2CBus::findOrAdd(uint16_t addr7)
{
    for (int i = 0; i < _deviceCount; ++i) {
        if (_devices[i].addr == addr7) return i;
    }

    if (_deviceCount < kMaxDevices - 1) {
        int idx = _deviceCount++;
        _devices[idx].addr     = addr7;
        _devices[idx].txnCount = 0;
        _devices[idx].errCount = 0;
        _devices[idx].lastErr  = 0;
        _devices[idx].lastEnd  = 0;
        _devices[idx].readyAt  = 0;
        return idx;
    }

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
    if (status != 0) {   // 0 == OK; HOST_BUILD has no MicroBit.h MICROBIT_OK,
                          // but CODAL's own convention is MICROBIT_OK == 0.
        ++_devices[idx].errCount;
        _devices[idx].lastErr = status;
    }
}
