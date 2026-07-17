#include "radio.h"
#include <string.h>

Radio* Radio::_instance = nullptr;

Radio::Radio(MicroBitRadio& radio, MessageBus& bus)
    : _radio(radio), _bus(bus),
      _reasmLen(0), _reasmActive(false), _msgReady(false), _txSeq(0)
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
            self->_msgReady = true;
        }
        self->_reasmActive = false;
        self->_reasmLen = 0;
    }
}

bool Radio::poll(char* buf, uint16_t len) {
    if (!_msgReady) return false;
    uint16_t out = (uint16_t)strlen(_msg);
    if (out >= len) out = len - 1;
    memcpy(buf, _msg, out);
    buf[out] = '\0';
    _msgReady = false;   // release the slot for the next message
    return true;
}

void Radio::send(const char* msg) {
    // Terminate every message with '\n' (mirrors SerialPort::send's "\r\n") —
    // required so the host's line reader can split consecutive messages after
    // !GO; see DESIGN.md. The '\n' is the last payload byte (msgLen counts
    // it) so it survives reassembly.
    int msgLen = (int)strlen(msg) + 1;   // +1 for the trailing '\n'
    int off = 0;
    bool first = true;
    uint8_t frame[FRAME_HEADER + MTU];

    do {
        int chunk = msgLen - off;
        if (chunk > MTU) chunk = MTU;

        uint8_t flags = 0;
        if (first) flags |= FLAG_START;
        if (off + chunk < msgLen) flags |= FLAG_MORE;
        else                      flags |= FLAG_END;

        frame[0] = _txSeq++;
        frame[1] = flags;
        frame[2] = (uint8_t)chunk;
        for (int i = 0; i < chunk; ++i) {
            int idx = off + i;
            // All but the final byte come from msg; the final byte is '\n'.
            frame[FRAME_HEADER + i] =
                (idx < msgLen - 1) ? (uint8_t)msg[idx] : (uint8_t)'\n';
        }
        _radio.datagram.send(frame, FRAME_HEADER + chunk);

        off += chunk;
        first = false;
    } while (off < msgLen);
}
