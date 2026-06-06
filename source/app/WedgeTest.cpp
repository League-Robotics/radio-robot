// WedgeTest.cpp — minimal encoder-wedge bench harness.
//
// Distilled to ONLY the two things that demonstrably reduced the Nezha encoder
// wedge, with everything that didn't help removed (no modes, no bus/gap/reset/
// settle knobs):
//
//   1. Read BOTH encoders every tick, motor 1 (right) FIRST, then motor 2.
//   2. Write the motor command ONLY when the commanded speed changes.
//
// The loop is an explicit fixed-rate scheduler: each tick fires when the clock
// passes the next deadline (deadline = tick-start + period). The actual loop
// rate is measured and printed once a second so the cadence is verified — you
// can see for certain whether it is really running at 10 Hz (or whatever).
//
// Stops on any serial byte, or when a wedge is detected.
//
// Nezha V2 wire protocol (8-byte writes to I2C 0x10):
//   move/coast : FF F9 <id> <dir> 60 <speed 0-100> F5 00     (dir CW=1, CCW=2)
//   enc read   : FF F9 <id> 00 46 00 F5 00  then read 4 bytes (LE int32, 0.1 deg)
// Motor ids: right = 1, left = 2.

#include "WedgeTest.h"
#include "MicroBit.h"
#include <cstdio>
#include <cstdint>

namespace {

constexpr uint16_t ADDR_W    = 0x10 << 1;   // 8-bit wire address
constexpr uint8_t  M_RIGHT   = 1;
constexpr uint8_t  M_LEFT    = 2;
constexpr uint8_t  DIR_CW    = 1;
constexpr uint8_t  DIR_CCW   = 2;
// Only the POST-write settle is kept (the chip needs time to prepare its 4-byte
// response). The old PRE-idle wait is dropped: the fixed-rate loop already
// leaves the bus idle between ticks, so it was redundant.
constexpr uint32_t SETTLE_US = 4000;        // post-0x46-write settle before the read

inline uint64_t nowUs() { return system_timer_current_time_us(); }

inline void busyUs(uint32_t us)
{
    uint64_t deadline = nowUs() + us;
    while (nowUs() < deadline) {}
}

// One 0x60 motor command. pct in [-100, 100]; 0 = coast.
void writeMotor(MicroBitI2C& i2c, uint8_t id, int pct)
{
    if (pct >  100) pct =  100;
    if (pct < -100) pct = -100;
    uint8_t dir = (pct >= 0) ? DIR_CW : DIR_CCW;
    uint8_t sp  = (pct >= 0) ? (uint8_t)pct : (uint8_t)(-pct);
    uint8_t buf[8] = { 0xFF, 0xF9, id, dir, 0x60, sp, 0xF5, 0x00 };
    i2c.write(ADDR_W, buf, 8, false);
}

// One 0x46 encoder read: write the angle-request, wait the post-write settle,
// read 4 bytes. No pre-idle wait (see SETTLE_US note).
int32_t readEnc(MicroBitI2C& i2c, uint8_t id)
{
    uint8_t cmd[8] = { 0xFF, 0xF9, id, 0x00, 0x46, 0x00, 0xF5, 0x00 };
    i2c.write(ADDR_W, cmd, 8, false);
    busyUs(SETTLE_US);
    uint8_t r[4] = { 0, 0, 0, 0 };
    i2c.read(ADDR_W, r, 4, false);
    return (int32_t)(((uint32_t)r[3] << 24) | ((uint32_t)r[2] << 16) |
                     ((uint32_t)r[1] << 8) | (uint32_t)r[0]);
}

constexpr int32_t JUMP = 4000;          // |delta| above this = implausible read
constexpr int     RETRIES = 6;          // re-reads to confirm a suspicious value

// Read an encoder, but if the value is an implausible jump from `prev`, RE-READ
// up to RETRIES times. If any re-read returns a sane value (the encoder is still
// counting), that's a transient glitch — count it and return the good value. If
// every re-read is still implausible, the encoder has truly latched: set
// *latched and return the (bad) value.
int32_t readConfirmed(MicroBitI2C& i2c, uint8_t id, int32_t prev,
                      uint32_t& glitches, bool& latched)
{
    int32_t e = readEnc(i2c, id);
    int32_t d = e - prev;
    if (d <= JUMP && d >= -JUMP) return e;          // looks normal — done
    for (int k = 0; k < RETRIES; ++k) {             // suspicious — confirm
        int32_t e2 = readEnc(i2c, id);
        int32_t d2 = e2 - prev;
        if (d2 <= JUMP && d2 >= -JUMP) { ++glitches; return e2; }  // recovered
        e = e2;
    }
    latched = true;                                 // never recovered — real wedge
    return e;
}

// Drive pattern: hold (l, r) for `ticks` ticks, then advance to the next phase.
struct Phase { int l; int r; uint16_t ticks; const char* name; };
const Phase PHASES[] = {
    { +90, +90, 40, "fwd"  }, {   0,   0, 12, "coast" },
    { -90, -90, 40, "back" }, {   0,   0, 12, "coast" },
    { +95, -95, 25, "spin" }, { +100, +75, 40, "arc" }, { 0, 0, 12, "coast" },
};
constexpr int NPH = (int)(sizeof(PHASES) / sizeof(PHASES[0]));

}  // namespace

void runWedgeTest(MicroBit& uBit, int rateHz)
{
    MicroBitI2C& i2c = uBit.i2c;
    char line[160];

    if (rateHz < 1)   rateHz = 1;
    if (rateHz > 200) rateHz = 200;
    const uint32_t periodUs = 1000000u / (uint32_t)rateHz;

    i2c.setFrequency(100000);               // 100 kHz bus (restored at exit)

    snprintf(line, sizeof(line),
        "WEDGETEST start: read-both(M1-first)+write-on-change @ %d Hz "
        "(period %u us, bus 100kHz, settle %u us). Any byte stops.\r\n",
        rateHz, (unsigned)periodUs, (unsigned)SETTLE_US);
    uBit.serial.send(line);

    // --- state ---
    int32_t prevR = readEnc(i2c, M_RIGHT);
    int32_t prevL = readEnc(i2c, M_LEFT);
    int lastL = 0x7fff, lastR = 0x7fff;     // sentinel: force the first write
    int stuckR = 0, stuckL = 0;             // consecutive frozen (delta 0) reads
    uint32_t glitches = 0;                  // transient bad reads that re-read OK
    int pi = 0; uint16_t pk = 0;            // phase index, tick within phase
    uint32_t ticks = 0, cycles = 0;
    const char* verdict = "stopped";

    // --- explicit fixed-rate scheduler + rate measurement ---
    uint64_t nextTickUs   = nowUs();        // first tick fires immediately
    uint64_t reportAtUs   = nowUs() + 1000000u;
    uint32_t ticksAtReport = 0;

    bool stop = false;
    while (!stop) {
        uint64_t now = nowUs();
        if (now < nextTickUs) {             // not time for the next tick yet
            if (uBit.serial.isReadable()) stop = true;
            continue;                       // spin until the deadline
        }
        nextTickUs = now + periodUs;        // schedule the next tick

        // ---- one tick: drive (write-on-change) + read BOTH encoders ----
        const Phase& ph = PHASES[pi];
        if (ph.l != lastL || ph.r != lastR) {
            writeMotor(i2c, M_RIGHT, ph.r);
            writeMotor(i2c, M_LEFT,  ph.l);
            lastL = ph.l; lastR = ph.r;
        }
        // Read both, confirming any suspicious value with re-reads (motor 1 first).
        bool latched = false;
        int32_t eR = readConfirmed(i2c, M_RIGHT, prevR, glitches, latched);
        int32_t eL = readConfirmed(i2c, M_LEFT,  prevL, glitches, latched);
        ++ticks;

        // ---- wedge detection ----
        // (a) latched: a glitch value that never recovered across the re-reads.
        // (b) frozen: a plausible value that simply stops changing while we are
        //     commanding motion (confirmed over many ticks, not one read).
        int32_t dR = eR - prevR, dL = eL - prevL;
        prevR = eR; prevL = eL;
        if (ph.r != 0 && dR == 0) ++stuckR; else stuckR = 0;
        if (ph.l != 0 && dL == 0) ++stuckL; else stuckL = 0;
        if (latched || stuckR >= 30 || stuckL >= 30) {
            verdict = latched ? "WEDGE-LATCHED" : "WEDGE-FROZEN";
            snprintf(line, sizeof(line),
                "%s at tick=%lu cyc=%lu phase=%s  R=%ld L=%ld stuckR=%d stuckL=%d "
                "(after %lu glitches)\r\n",
                verdict, (unsigned long)ticks, (unsigned long)cycles, ph.name,
                (long)eR, (long)eL, stuckR, stuckL, (unsigned long)glitches);
            uBit.serial.send(line);
            break;
        }

        // ---- advance the drive pattern ----
        if (++pk >= ph.ticks) { pk = 0; if (++pi >= NPH) { pi = 0; ++cycles; } }

        // ---- verify the rate: print the MEASURED loop rate once a second ----
        if (now >= reportAtUs) {
            uint32_t dticks = ticks - ticksAtReport;   // ticks in the last ~1 s
            snprintf(line, sizeof(line),
                "  rate=%lu Hz  cyc=%lu  glitches=%lu  R=%ld L=%ld\r\n",
                (unsigned long)dticks, (unsigned long)cycles,
                (unsigned long)glitches, (long)eR, (long)eL);
            uBit.serial.send(line);
            ticksAtReport = ticks;
            reportAtUs += 1000000u;
        }
    }

    writeMotor(i2c, M_RIGHT, 0);
    writeMotor(i2c, M_LEFT, 0);
    i2c.setFrequency(400000);               // restore default bus for the rest of the firmware
    snprintf(line, sizeof(line),
             "WEDGETEST end (%s, %lu ticks, %lu cycles, %lu glitches)\r\n",
             verdict, (unsigned long)ticks, (unsigned long)cycles, (unsigned long)glitches);
    uBit.serial.send(line);
}
