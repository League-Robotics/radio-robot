#include "serial_port.h"
#include <string.h>
#include <stdio.h>

SerialPort::SerialPort(NRF52Serial& serial)
    : _serial(serial), _rxLen(0)
{
    memset(_rxBuf, 0, sizeof(_rxBuf));
}

void SerialPort::begin() {
    _serial.setRxBufferSize(255);
    // TX buffer size is a uint8_t in CODAL — 255 is the max (1024 wraps to 0!).
    // Keep replies to a single line; don't fire many sends back-to-back.
    _serial.setTxBufferSize(255);
    _serial.setBaud(115200);
}

bool SerialPort::readLine(char* buf, uint16_t len) {
    int c;
    while ((c = _serial.read(ASYNC)) != MICROBIT_NO_DATA) {
        if (c == '\r') continue;
        if (c == '\n') {
            _rxBuf[_rxLen] = '\0';
            uint16_t copy = (_rxLen < len - 1) ? _rxLen : (len - 1);
            memcpy(buf, _rxBuf, copy);
            buf[copy] = '\0';
            _rxLen = 0;
            return true;
        }
        if (_rxLen < sizeof(_rxBuf) - 1)
            _rxBuf[_rxLen++] = (char)c;
    }
    return false;
}

void SerialPort::send(const char* msg) {
    // ASYNC: queue what fits in the TX buffer and return IMMEDIATELY, never
    // blocking the loop. Drop-on-full — a frame may be silently truncated
    // under a flood. For must-arrive lines use sendReliable() instead.
    _serial.send(ManagedString(msg) + ManagedString("\r\n"), ASYNC);
}

void SerialPort::sendReliable(const char* msg) {
    // Like send(), but bounded-waits for TX-buffer room so the WHOLE line
    // fits before handing off to ASYNC. 5 ms cap: a dead/absent reader can't
    // hang the loop — falls through and sends anyway, dropping the overflow
    // exactly as pure ASYNC would.
    ManagedString s = ManagedString(msg) + ManagedString("\r\n");
    const int len = s.length();
    const uint64_t deadline = system_timer_current_time_us() + 5000;   // [us]
    while ((250 - _serial.txBufferedSize()) < len &&
           system_timer_current_time_us() < deadline) {
        // spin briefly; the UART's DMA drains the buffer in the background
    }
    _serial.send(s, ASYNC);
}

void SerialPort::setBaud(uint32_t baud) {
    // Drain the TX buffer (bounded) so the just-sent reply, at the OLD baud,
    // clocks out fully before retuning — otherwise its trailing bytes garble.
    const uint64_t drainDeadline = system_timer_current_time_us() + 20000;   // [us] 20 ms cap
    while (_serial.txBufferedSize() > 0 &&
           system_timer_current_time_us() < drainDeadline) { /* spin */ }
    // Software buffer empty, but the UART shift register + DAPLink still need
    // a moment to push the final bytes out — brief settle before retuning.
    const uint64_t settleDeadline = system_timer_current_time_us() + 4000;    // [us] ~4 ms
    while (system_timer_current_time_us() < settleDeadline) { /* spin */ }
    _serial.setBaud((int)baud);
}

void SerialPort::sendf(const char* fmt, ...) {
    char tmp[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    send(tmp);
}
