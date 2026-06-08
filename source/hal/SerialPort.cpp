#include "SerialPort.h"
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
    // Bursts must therefore fit in 255 bytes: keep replies to a single line and
    // don't fire many sends back-to-back (the firmware blocks/loses output when
    // the TX buffer can't absorb a burst).
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
    // ASYNC: queue what fits in the TX buffer and return IMMEDIATELY. Never block
    // the cooperative loop. Used for the TELEMETRY flood — under a flood (host not
    // reading fast enough, or the IRQ-guard stalling the TX drain during a drive)
    // a frame may be dropped (host sees a truncated line and skips it). Acceptable
    // for telemetry; a stalled control loop is not. For rare must-arrive lines
    // (command replies, EVT done) use sendReliable() instead.
    _serial.send(ManagedString(msg) + ManagedString("\r\n"), ASYNC);
}

void SerialPort::sendReliable(const char* msg) {
    // Like send(), but waits (bounded) for TX-buffer room so the WHOLE line fits
    // before handing it to the ASYNC path — so a momentarily-full buffer doesn't
    // silently drop it (a lost EVT done, a truncated reply). ONLY for rare lines
    // (replies, EVT) — never the telemetry stream: the wait would stall the
    // control loop every tick when the buffer is under pressure during a drive.
    //
    // When the host is reading (the normal case) the buffer drains in well under
    // 1 ms, so there is effectively no wait. The 5 ms cap means a dead/absent
    // reader can NOT hang the loop: we fall through and send anyway, dropping the
    // overflow exactly as pure ASYNC would.
    ManagedString s = ManagedString(msg) + ManagedString("\r\n");
    const int len = s.length();
    const uint64_t deadlineUs = system_timer_current_time_us() + 5000;
    while ((250 - _serial.txBufferedSize()) < len &&
           system_timer_current_time_us() < deadlineUs) {
        // spin briefly; the UART's DMA drains the buffer in the background
    }
    _serial.send(s, ASYNC);
}

void SerialPort::sendf(const char* fmt, ...) {
    char tmp[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(tmp, sizeof(tmp), fmt, args);
    va_end(args);
    send(tmp);
}
