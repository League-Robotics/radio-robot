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
 */
class Radio {
public:
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

    // Non-blocking. Returns true and fills buf (NUL-terminated) when a complete
    // reassembled message is ready. Only one message is buffered — a second
    // message completing before poll() drains the first is dropped.
    bool poll(char* buf, uint16_t len);

    // Fragment msg into RAW250 frames and transmit each one.
    void send(const char* msg);

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
    char          _msg[REASM_MAX];
    volatile bool _msgReady;

    uint8_t _txSeq;           // rolling §5 sequence number

    static void onData(MicroBitEvent);
    static Radio* _instance;
};
