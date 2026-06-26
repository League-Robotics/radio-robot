#pragma once
#include "MicroBit.h"
#include <stdarg.h>

/**
 * SerialPort — line-buffered 115200-baud serial over USB.
 *
 * Non-blocking: readLine() drains the CODAL ASYNC receive buffer each call
 * and returns true only when a complete '\n'-terminated line is ready.
 * Never calls uBit.sleep() or any blocking CODAL primitive.
 */
class SerialPort {
public:
    explicit SerialPort(NRF52Serial& serial);

    // Configure buffers (256 bytes each) and set baud rate to 115200.
    // Must be called once before readLine() / send() / sendf().
    void begin();

    // Non-blocking. Accumulates bytes from ASYNC read; returns true when
    // a complete '\n'-terminated line is ready. buf is null-terminated;
    // newline stripped. len includes the NUL terminator.
    bool readLine(char* buf, uint16_t len);

    void send(const char* msg);          // ASYNC, drop-on-full — for telemetry
    void sendReliable(const char* msg);  // bounded-wait for room — for replies/EVT
    void sendf(const char* fmt, ...);  // snprintf into 256-byte stack buffer

    // Retune the UART baud at runtime. Drains TX first so an already-queued
    // reply (sent at the OLD baud) clocks out fully before the switch. Supported
    // rates: 115200 (default), 230400, 921600, 1000000. The host must change its
    // own baud to match WITHOUT reopening the port (reopening pulses DTR → reset).
    void setBaud(uint32_t baud);

private:
    NRF52Serial& _serial;
    char     _rxBuf[256];   // holds up to a 250-byte line (RAW250 message size)
    uint16_t _rxLen;
};
