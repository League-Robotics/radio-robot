#include "radio.h"
#include <string.h>

Radio* Radio::_instance = nullptr;

Radio::Radio(MicroBitRadio& radio, MessageBus& bus)
    : _radio(radio), _bus(bus),
      _reasmLen(0), _reasmActive(false), _msgLen(0), _msgReady(false), _txSeq(0)
{
    memset(_reasm, 0, sizeof(_reasm));
    memset(_msg, 0, sizeof(_msg));
}

void Radio::begin(int channel) {
    _instance = this;
    _channel = channel;
    _radio.enable();
    // CODAL does not default to band 0 — must set explicitly or the robot
    // and relay sit on different frequencies and never hear each other.
    _radio.setFrequencyBand(channel);
    _radio.setGroup(10);
    _radio.setTransmitPower(7);
    _bus.listen(DEVICE_ID_RADIO, MICROBIT_RADIO_EVT_DATAGRAM, onData);
}

int Radio::setChannel(int channel) {
    int rc = _radio.setFrequencyBand(channel);
    if (rc == MICROBIT_OK) {
        _channel = channel;
    }
    return rc;
}

// Reassemble §5 fragments in place. Runs in the radio datagram ISR context.
// Binary-clean by construction (raw memcpy, no byte-level interpretation).
void Radio::onData(MicroBitEvent) {
    Radio* self = _instance;
    if (!self) return;
    PacketBuffer pkt = self->_radio.datagram.recv();
    int n = pkt.length();
    if (n < FRAME_HEADER) return;

    const uint8_t* b = pkt.getBytes();
    uint8_t flags = b[1];
    int plen = b[2];
    if (plen > n - FRAME_HEADER) plen = n - FRAME_HEADER;

    if (flags & FLAG_ACK) return;               // ACK frame: nothing to assemble

    if (flags & FLAG_START) {
        self->_reasmLen = 0;
        self->_reasmActive = true;
    }
    if (self->_reasmActive && plen > 0) {
        int space = REASM_MAX - 1 - self->_reasmLen;
        int copy = (plen < space) ? plen : space;
        if (copy > 0) {
            memcpy(self->_reasm + self->_reasmLen, b + FRAME_HEADER, copy);
            self->_reasmLen += copy;
        }
    }
    if (flags & FLAG_END) {
        // Publish only if the previous message has been consumed; otherwise drop.
        if (self->_reasmActive && !self->_msgReady) {
            memcpy(self->_msg, self->_reasm, self->_reasmLen);
            self->_msg[self->_reasmLen] = '\0';
            self->_msgLen = self->_reasmLen;
            self->_msgReady = true;
        }
        self->_reasmActive = false;
        self->_reasmLen = 0;
    }
}

bool Radio::poll(char* buf, uint16_t cap, uint16_t* outLen) {
    if (!_msgReady) return false;

    // UNCONDITIONAL terminator -- every sender (send(), the sole outbound
    // path) appends exactly one trailing '\n'; strip it here. No '\r'
    // stripping -- a binary line may legitimately carry 0x0D as
    // content; Core::Comms::dispatchLine() strips a trailing '\r' only once
    // it has classified the line as cleartext (see this class's own file
    // header).
    int contentLen = _msgLen;
    if (contentLen > 0 && _msg[contentLen - 1] == '\n') --contentLen;

    uint16_t out = static_cast<uint16_t>(contentLen);
    if (out >= cap) out = cap - 1;
    memcpy(buf, _msg, out);
    buf[out] = '\0';
    if (outLen) *outLen = out;
    _msgReady = false;   // release the slot for the next message
    return true;
}

// Shared fragmentation body -- `payload[0..payloadLen)` already carries its
// own trailing '\n' delimiter (the one terminator every outbound line
// uses, appended by send() itself) as its LAST byte; this function only
// knows about RAW250 fragment framing, never about what the trailing
// byte means.
void Radio::sendFragmented(const uint8_t* payload, int payloadLen) {
    int off = 0;
    bool first = true;
    uint8_t frame[FRAME_HEADER + MTU];

    do {
        int chunk = payloadLen - off;
        if (chunk > MTU) chunk = MTU;

        uint8_t flags = 0;
        if (first) flags |= FLAG_START;
        if (off + chunk < payloadLen) flags |= FLAG_MORE;
        else                          flags |= FLAG_END;

        frame[0] = _txSeq++;
        frame[1] = flags;
        frame[2] = (uint8_t)chunk;
        if (chunk > 0) memcpy(frame + FRAME_HEADER, payload + off, chunk);
        _radio.datagram.send(frame, FRAME_HEADER + chunk);

        off += chunk;
        first = false;
    } while (off < payloadLen);
}

void Radio::send(const uint8_t* data, uint16_t len) {
    // `data`/`len` is the full wire LINE content (a command-prefixed COBS
    // body, or a cleartext reply -- Core::Comms builds either shape the
    // same way before handing it here) + a single trailing '\n' delimiter
    // -- the ONE terminator every outbound line uses (see this class's
    // own file header). Payload buffer generously covers
    // Core::kMaxLineBytes (207) + 1 delimiter with headroom;
    // truncates (rather than overflows) on an over-length caller,
    // mirroring SerialPort::send()'s own defensive truncation.
    uint8_t payload[256];
    uint16_t n = (len < sizeof(payload) - 1) ? len : (uint16_t)(sizeof(payload) - 1);
    if (n > 0) memcpy(payload, data, n);
    payload[n] = '\n';
    sendFragmented(payload, static_cast<int>(n) + 1);
}
