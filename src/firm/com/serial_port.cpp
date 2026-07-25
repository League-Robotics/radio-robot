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

SerialPort::FrameKind SerialPort::readLine(char* buf, uint16_t cap, uint16_t* outLen) {
    int c;
    while ((c = _serial.read(ASYNC)) != MICROBIT_NO_DATA) {
        if (c == 0x00) {
            // Binary frame delimiter -- deliver the accumulated bytes
            // AS-IS (no '\r' stripping -- a COBS+CRC frame may legitimately
            // carry 0x0D as content, unlike a text-plane line).
            uint16_t copy = (_rxLen < cap - 1) ? _rxLen : (cap - 1);
            memcpy(buf, _rxBuf, copy);
            // Safe to NUL-terminate even for binary content: COBS-encoded
            // bytes never contain 0x00 by construction, so this terminator
            // can never collide with real frame content.
            buf[copy] = '\0';
            if (outLen) *outLen = copy;
            _rxLen = 0;
            return FrameKind::kBinary;
        }
        if (c == '\n') {
            // Text-plane line -- strip a single trailing '\r' (the
            // "\r\n" text-line convention), matching pre-123 behavior for
            // HELLO/PING.
            uint16_t n = _rxLen;
            if (n > 0 && _rxBuf[n - 1] == '\r') --n;
            uint16_t copy = (n < cap - 1) ? n : (cap - 1);
            memcpy(buf, _rxBuf, copy);
            buf[copy] = '\0';
            if (outLen) *outLen = copy;
            _rxLen = 0;
            return FrameKind::kText;
        }
        if (_rxLen < sizeof(_rxBuf) - 1)
            _rxBuf[_rxLen++] = (char)c;
    }
    return FrameKind::kNone;
}

void SerialPort::send(const uint8_t* data, uint16_t len) {
    // ASYNC: queue the WHOLE frame and return IMMEDIATELY, never blocking
    // the loop. 123-006: drop-on-full now means drop the ENTIRE frame, not
    // whatever prefix happens to fit -- CODAL's ASYNC send() (setTxInterrupt())
    // copies bytes until the buffer fills and then silently stops, which used
    // to hand a truncated frame to the wire. A truncated frame always fails
    // the host's CRC (malformed), and if the dropped tail included the
    // trailing 0x00 delimiter, the leftover bytes prefix the NEXT frame and
    // corrupt it too. Checking free space FIRST and refusing to send at all
    // when the frame doesn't fit costs one honest, countable seq gap instead
    // -- never a corrupt or merged frame. For must-arrive lines use
    // sendReliable() instead.
    //
    // 123-002: binary frame body + the trailing 0x00 delimiter, via the
    // raw uint8_t*/len send() overload (NOT ManagedString -- the content is
    // not a NUL-terminated C string and may not be printable ASCII).
    uint8_t framed[256];
    uint16_t n = (len < sizeof(framed) - 1) ? len : (uint16_t)(sizeof(framed) - 1);
    memcpy(framed, data, n);
    framed[n] = 0x00;

    const int frameLen = static_cast<int>(n) + 1;
    if (kTxBufferCapacity - _serial.txBufferedSize() < frameLen) {
        // Not enough room for the WHOLE frame -- drop it atomically. Nothing
        // is written to the UART; the frame is simply never sent.
        return;
    }
    _serial.send(framed, frameLen, ASYNC);
}

void SerialPort::sendReliable(const char* msg) {
    // Like send(), but bounded-waits for TX-buffer room so the WHOLE line
    // fits before handing off to ASYNC. 5 ms cap: a dead/absent reader can't
    // hang the loop — falls through and sends anyway, dropping the overflow
    // exactly as pure ASYNC would. Text-plane only (HELLO/PING replies) --
    // still "\r\n"-terminated, unchanged from pre-123.
    ManagedString s = ManagedString(msg) + ManagedString("\r\n");
    const int len = s.length();
    const uint64_t deadline = system_timer_current_time_us() + 5000;   // [us]
    while ((kTxBufferCapacity - _serial.txBufferedSize()) < len &&
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
    sendReliable(tmp);
}
