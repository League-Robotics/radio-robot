#pragma once
#include "MicroBit.h"

/**
 * Radio — micro:bit radio driver speaking the RadioRelay RAW250 framing.
 * Design/rationale: DESIGN.md.
 *
 * Wire framing (RadioRelay §5): every on-air packet is a fragment
 *     [SEQ:1][FLAGS:1][LEN:1][payload:LEN]
 * carried as the CODAL datagram payload (no MakeCode/PXT header in RAW250).
 * FLAGS: START=0x01, MORE=0x02, END=0x04, ACK=0x10. A message is split into
 * fragments of up to MTU (=247) payload bytes; the receiver reassembles from
 * START through END. A single-fragment message is flagged START|END (0x05).
 * Firmware MUST be built with MICROBIT_RADIO_MAX_PACKET_SIZE=250 (codal.json)
 * so the on-air nRF MAXLEN matches the relay.
 *
 * Only one Radio instance may call begin(). _instance is a static singleton
 * pointer used by the static ISR callback.
 *
 * 123-002 (COBS+CRC framer integration): fragment reassembly (`onData`) was
 * ALREADY binary-clean (raw memcpy, no byte-level interpretation) -- what
 * changes here is the trailing byte convention and `poll()`'s demux.
 * `send()` (binary, COBS+CRC frame body) appends a trailing 0x00; the NEW
 * `sendText()` (text-plane HELLO/PING replies) appends a trailing '\n', the
 * same terminator `send()` used for everything pre-123. `poll()` looks at
 * the reassembled message's OWN trailing byte (whichever the sender
 * appended) to decide which of the two it just received -- no dependency
 * on `app/`, per this directory's own "com/ has no dependency on app/,
 * messages/, or any wire-schema type" invariant (com/DESIGN.md);
 * `app/comms.h`'s `RadioTransport` adapter maps `Radio::FrameKind` onto
 * `App::FrameKind` at the one seam that is allowed to know both.
 */
class Radio {
public:
    enum class FrameKind : uint8_t { kNone = 0, kText = 1, kBinary = 2 };

    explicit Radio(MicroBitRadio& radio, MessageBus& bus);

    // enable(), setFrequencyBand(channel), setGroup(10), setTransmitPower(7),
    // register the datagram ISR.  `channel` is the nRF frequency band (0..83);
    // the group is always 10 to match the RadioRelay.
    void begin(int channel = 0);

    // Re-tune to a new channel (frequency band) at runtime. Group is unchanged.
    // Returns MICROBIT_OK on success, or a CODAL error on an invalid band.
    // NOTE: re-tuning over the radio drops the link (the relay stays on the old
    // channel) — the caller must send any reply BEFORE calling this.
    int setChannel(int channel);

    // The channel (frequency band) currently in use.
    int channel() const { return _channel; }

    // Non-blocking. Returns FrameKind::kNone until a complete reassembled
    // message is ready. FrameKind::kBinary: buf holds *outLen raw bytes (the
    // COBS+CRC frame body, trailing 0x00 delimiter consumed/not included).
    // FrameKind::kText: buf holds a NUL-terminated ASCII line (trailing
    // '\n'/'\r' stripped), *outLen == strlen(buf). Only one message is
    // buffered — a second message completing before poll() drains the
    // first is dropped.
    FrameKind poll(char* buf, uint16_t cap, uint16_t* outLen);

    // Fragment a COBS+CRC binary frame body into RAW250 frames and
    // transmit each one, appending a trailing 0x00 delimiter as the FINAL
    // payload byte (mirrors SerialPort::send()'s own delimiter-appending
    // contract). `data`/`len` is 0x00-free by construction (App::Transport::
    // send()'s own contract) so the 0x00 this appends is unambiguous.
    void send(const uint8_t* data, uint16_t len);

    // Fragment a text-plane reply (NUL-terminated ASCII -- HELLO/PING) into
    // RAW250 frames, appending a trailing '\n' as the FINAL payload byte --
    // this is what `send()` did for EVERYTHING pre-123 (RadioRelay §5
    // framing alone delimits a message on the wire, but after `!GO` the
    // link becomes a transparent byte pipe with no per-message boundary of
    // its own; without the embedded newline, consecutive host-bound text
    // replies concatenate and the host's line reader can't split them).
    void sendText(const char* msg);

private:
    MicroBitRadio& _radio;
    MessageBus&    _bus;
    int            _channel = 0;   // nRF frequency band in use (group is always 10)

    // RadioRelay §5 fragment framing.
    static constexpr uint8_t FLAG_START = 0x01;
    static constexpr uint8_t FLAG_MORE  = 0x02;
    static constexpr uint8_t FLAG_END   = 0x04;
    static constexpr uint8_t FLAG_ACK   = 0x10;
    static constexpr int FRAME_HEADER = 3;
    static constexpr int MAX_FRAME    = MICROBIT_RADIO_MAX_PACKET_SIZE; // 250
    static constexpr int MTU          = MAX_FRAME - FRAME_HEADER;       // 247
    static constexpr int REASM_MAX    = 512;   // v2 GET dump can reach ~290 bytes

    // Reassembly accumulator (ISR-owned).
    char _reasm[REASM_MAX];
    int  _reasmLen;
    bool _reasmActive;

    // Completed message published to poll(). _msgReady gates the handoff and is
    // the single synchronization point between the ISR and the main loop.
    // _msgLen is the EXACT reassembled byte count (set by the ISR alongside
    // _msgReady) -- poll() trusts this instead of strlen(_msg) so a stray
    // embedded 0x00 in a corrupt/fault-injected binary frame can never
    // truncate the length the demux reasons about.
    char          _msg[REASM_MAX];
    int           _msgLen;
    volatile bool _msgReady;

    uint8_t _txSeq;           // rolling §5 sequence number

    // Shared fragmentation body for send()/sendText(): fragments
    // `payload[0..payloadLen)` into RAW250 frames. Used by both public
    // sends -- the only difference between them is the trailing delimiter
    // byte the caller already appended into `payload`.
    void sendFragmented(const uint8_t* payload, int payloadLen);

    static void onData(MicroBitEvent);
    static Radio* _instance;
};
