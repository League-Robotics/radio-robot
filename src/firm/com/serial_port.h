#pragma once
#include "MicroBit.h"
#include <stdarg.h>

/**
 * SerialPort — binary-clean 115200-baud serial over USB.
 * Design/rationale: DESIGN.md.
 *
 * readLine() reads to an UNCONDITIONAL '\n' terminator -- no heuristic, no
 * recognizer, no text/binary distinction at this layer at all (Core::Comms
 * decides text-vs-binary from the parsed `<COMMAND>` prefix, once the
 * transport hands it a complete line -- see comms.h's own file header).
 * This is safe because COBS is keyed on 0x0A (wire_runtime.h item 8): a
 * binary frame's own bytes never contain a literal 0x0A, so '\n' is a
 * genuine, unconditional terminator in both directions. Bytes are
 * accumulated UNFILTERED (no '\r' stripping here -- under one uniform rule
 * '\r' is legal binary content; a caller that has already classified a
 * line as cleartext strips it, Core::Comms::dispatchLine()). The host's
 * mirror, wire_codec.py's `ByteStreamDemuxer`, splits on '\n' alone.
 *
 * Non-blocking: readLine() drains the CODAL ASYNC receive buffer each call
 * and returns false until a complete line is ready.
 * Never calls uBit.sleep() or any blocking CODAL primitive.
 */
class SerialPort {
public:
    // Usable TX ring-buffer capacity assumed by both backpressure checks in
    // this class: send()'s whole-frame-fits gate and sendReliable()'s
    // bounded wait. begin() configures the underlying CODAL UART TX buffer
    // to 255 bytes (the uint8_t max); a circular buffer of N bytes holds at
    // most N-1 bytes before head catches tail (ambiguous with empty), so the
    // true ceiling is 254 -- this constant keeps a small extra margin below
    // that, and is the single source of truth so send()'s drop check and
    // sendReliable()'s spin-wait can never assume two different capacities.
    static constexpr uint16_t kTxBufferCapacity = 250;  // [bytes]

    explicit SerialPort(NRF52Serial& serial);

    // Configure buffers (256 bytes each) and set baud rate to 115200.
    // Must be called once before readLine() / send() / sendReliable().
    void begin();

    // Non-blocking. Accumulates bytes from ASYNC read (unfiltered -- see
    // this class's own file header) until an UNCONDITIONAL '\n' (0x0A)
    // ends a complete line -- buf holds *outLen raw bytes (the delimiter
    // itself consumed, not included), NUL-terminated as a convenience
    // (safe: a binary line's COBS-encoded content is 0x00-free by
    // construction, so this terminator can never collide with real
    // content). Returns false (buf/*outLen untouched) when nothing
    // complete is ready yet. `cap` bounds buf's capacity, including that
    // trailing NUL.
    bool readLine(char* buf, uint16_t cap, uint16_t* outLen);

    // ASYNC, WHOLE-FRAME drop-on-full — for telemetry. `data`/`len` is a
    // COBS+CRC frame body (0x00-free by construction, per
    // Core::Transport::send()'s own contract); this appends the trailing
    // 0x00 delimiter itself and sends the raw buffer (never a
    // ManagedString/strlen-based path -- the content is not necessarily
    // printable ASCII). Checks free TX-buffer space FIRST -- if the
    // full framed length (body + delimiter) does not fit, the ENTIRE frame
    // is dropped and nothing is written. Never hands CODAL's ASYNC send() a
    // frame it might only partially copy: a partial write both corrupts
    // this frame (fails the host's CRC) and, when the dropped tail includes
    // the trailing delimiter, merges into and corrupts the NEXT frame too.
    // Still fully non-blocking -- no spin-wait, unlike sendReliable().
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
