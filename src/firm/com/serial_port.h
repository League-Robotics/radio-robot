#pragma once
#include "MicroBit.h"
#include <stdarg.h>

/**
 * SerialPort — binary-clean 115200-baud serial over USB.
 * Design/rationale: DESIGN.md.
 *
 * 123-002 (COBS+CRC framer integration): readLine() demuxes a 0x00-
 * delimited COBS+CRC binary frame from a '\r'?'\n'-terminated text line
 * (HELLO/PING) on the SAME accumulated byte stream -- see readLine()'s own
 * doc comment for the exact contract. Bytes are accumulated UNFILTERED
 * (no longer dropping every '\r' on sight, since a binary frame may
 * legitimately carry 0x0D as content) until one of the two terminators
 * ends the frame. `SerialPort::FrameKind` is this class's OWN plain enum
 * (no dependency on `app/`, per this directory's own "com/ has no
 * dependency on app/, messages/, or any wire-schema type" invariant --
 * com/DESIGN.md) -- `app/comms.h`'s `SerialTransport` adapter maps this
 * onto `App::FrameKind` at the one seam that is allowed to know both.
 *
 * Non-blocking: readLine() drains the CODAL ASYNC receive buffer each call
 * and returns FrameKind::kNone until a complete frame is ready.
 * Never calls uBit.sleep() or any blocking CODAL primitive.
 */
class SerialPort {
public:
    enum class FrameKind : uint8_t { kNone = 0, kText = 1, kBinary = 2 };

    explicit SerialPort(NRF52Serial& serial);

    // Configure buffers (256 bytes each) and set baud rate to 115200.
    // Must be called once before readLine() / send() / sendReliable().
    void begin();

    // Non-blocking. Accumulates bytes from ASYNC read (unfiltered -- see
    // this class's own file header) until either terminator ends a
    // complete frame: 0x00 ends a BINARY frame (FrameKind::kBinary, buf
    // holds *outLen raw bytes, delimiter consumed/not included); '\n' ends
    // a TEXT frame (FrameKind::kText, a trailing '\r' stripped, buf
    // NUL-terminated, *outLen == strlen(buf)). Returns FrameKind::kNone
    // (buf/*outLen untouched) when nothing complete is ready yet. `cap`
    // bounds buf's capacity (including the NUL terminator this call
    // always writes, even for a binary frame -- safe because COBS-encoded
    // content is 0x00-free by construction).
    FrameKind readLine(char* buf, uint16_t cap, uint16_t* outLen);

    // ASYNC, drop-on-full — for telemetry. `data`/`len` is a COBS+CRC
    // frame body (0x00-free by construction, per App::Transport::send()'s
    // own contract); this appends the trailing 0x00 delimiter itself and
    // sends the raw buffer (never a ManagedString/strlen-based path --
    // the content is not necessarily printable ASCII).
    void send(const uint8_t* data, uint16_t len);

    void sendReliable(const char* msg);  // bounded-wait for room — for replies/EVT (text-plane only, "\r\n"-terminated, unchanged)
    void sendf(const char* fmt, ...);  // snprintf into 256-byte stack buffer, then sendReliable() (text-plane only)

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
