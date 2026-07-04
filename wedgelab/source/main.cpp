// ============================================================================
// WEDGELAB v2 — standalone, from-scratch Nezha V2 encoder-latch laboratory.
// Four-motor edition: ports M1..M4 driven and monitored simultaneously.
// Bench config 2026-07-03: M1/M2 = OLD (latch-prone) motors, M3/M4 = fresh.
//
// This program shares NO code with the production firmware. It talks to the
// Nezha V2 motor brick (I2C addr 0x10) with its own 8-byte-frame functions,
// re-implemented from the vendor wire protocol (vendor/pxt-nezha2/main.ts).
//
// Console (USB serial, 115200):
//   set <knob> <value> | get           knobs, no reflash needed
//   run legs|slam|burst|combo|native|spin N
//   heal | recover | stat | stop       (stop is encoder-verified)
//
// Detection is ENCODER-TRUTH, per wheel, both directions:
//   commanded + armed and readback constant  -> LATCH (+ ring dump)
//   commanded and readback NEVER moved       -> LATCH-BLIND
//   not commanded and readback moving        -> RUNAWAY / MOTION-AT-REST
//
// Wire protocol (verified byte-for-byte against vendor TS):
//   All commands: 8-byte write to 0x10: FF F9 <motor> <b3> <CMD> <b5> <b6> <b7>
//   0x60 run: [.. mm dir 60 speed F5 00]   dir 1=CW 2=CCW, speed 0-100
//   0x46 angle req: [.. mm 00 46 00 F5 00] then read 4B LE int32 (0.1 deg)
//   0x47 speed req: [.. mm 00 47 00 F5 00] then read 2B LE uint16
//   0x77 global speed: [.. 00 00 77 hi 00 lo] (speed*9)
//   0x70 move: [.. mm dir 70 hi mode lo]   mode 2 = degrees
//   Vendor timing: >=4 ms idle before read-requests, >=4 ms request->read.
//
// SAFETY LESSONS BAKED IN (2026-07-03 runaway post-mortem,
// clasi/reflections/2026-07-03-ignored-eyewitness-runaway.md):
//   - decel steps toward zero RELATIVE TO CURRENT VALUE, never a fixed
//     direction; bounded dither window; termination guaranteed (host-sim
//     proven in scripts + tests below src/test/).
//   - PWM invariant: a leg aborts loudly if it ever commands > cruise+dither.
//   - every pattern ends in stopVerified(); idle watchdog force-stops and
//     ring-dumps on any un-commanded motion.
// ============================================================================

#include "MicroBit.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

// Production motor layer, copied VERBATIM from source/ (2026-07-04, Eric's
// bisection directive): if the latch reproduces through these classes but
// not through the raw driver below, the difference lives in this layer.
#include "I2CBus.h"
#include "Motor.h"
#include "Config.h"

MicroBit uBit;

// Production-driver objects (constructed in main() after uBit.init()).
static RobotConfig gCfg = {};
static I2CBus*     gBus = nullptr;
static Motor*      gMot[4] = {nullptr, nullptr, nullptr, nullptr};

// ---------------------------------------------------------------------------
// Small utilities
// ---------------------------------------------------------------------------

static inline uint64_t nowUs() { return system_timer_current_time_us(); }

static void busyUs(uint32_t us)
{
    // Busy-wait: no fiber yield, so nothing else can slip onto the bus.
    uint64_t dl = nowUs() + us;
    while (nowUs() < dl) {}
}

static char gLine[240];
#define P(...) do { snprintf(gLine, sizeof(gLine), __VA_ARGS__); \
                    uBit.serial.send(gLine); } while (0)

// ---------------------------------------------------------------------------
// Knobs
// ---------------------------------------------------------------------------

struct Knob { const char* name; int* val; const char* help; };

static int kTickUs      = 20000;  // control tick period [us] (4 wheels x ~4.3ms reads)
static int kPreIdleUs   = 0;      // idle BEFORE a 0x46/0x47 request write (vendor: 4000)
static int kSettleUs    = 4000;   // request-write -> data-read gap (vendor: 4000)
static int kPostWriteUs = 0;      // idle AFTER any 0x60 write
static int kBusKhz      = 100;    // I2C bus speed
static int kCruise      = 32;     // cruise PWM percent
static int kSlamPct     = 80;     // slam reversal amplitude
static int kSlewCap     = 25;     // max |dPWM| per 0x60 write; 0 = unlimited
static int kWriteMode   = 1;      // 0=on-change 1=every-tick 2=throttle 40ms
static int kDither      = 1;      // +-N PWM dither during decel (PID mimicry)
static int kZeroDwellMs = 0;      // forced 0x60-speed-0 dwell at sign crossings
static int kReadMode    = 0;      // 0=all wheels 1=one-per-tick 2=none 3=0x47
static int kDecelSkip   = 0;      // 1 = skip encoder reads while pwm is changing
static int kLatchN      = 15;     // consecutive identical armed reads => latch
static int kArmRaw      = 50;     // raw 0.1deg movement to arm detection
static int kHealAuto    = 1;      // auto re-prime at rest between legs
static int kRestMs      = 400;    // inter-leg rest
static int kNWheels     = 4;      // active motor ports (1..kNWheels)
static int kRatchet     = 0;      // 1 = D-terminal stall/reverse/lunge tail on legs
static int kSensors     = 0;      // 1 = production-mimic OTOS read (0x17) each tick
static int kDriver      = 1;      // 1 = production Motor class, 0 = raw lab driver
static int kVerbose     = 0;      // 1 = per-leg lines during runs

static const Knob kKnobs[] = {
    {"tickus",    &kTickUs,      "tick period us"},
    {"preidle",   &kPreIdleUs,   "idle before read-request us (vendor 4000)"},
    {"settle",    &kSettleUs,    "request->read gap us (vendor 4000)"},
    {"postwrite", &kPostWriteUs, "idle after 0x60 write us"},
    {"buskhz",    &kBusKhz,      "i2c bus khz"},
    {"cruise",    &kCruise,      "cruise pwm pct"},
    {"slam",      &kSlamPct,     "slam amplitude pct"},
    {"slewcap",   &kSlewCap,     "max dPWM per write, 0=off"},
    {"writemode", &kWriteMode,   "0=onchange 1=everytick 2=throttle40"},
    {"dither",    &kDither,      "decel dither pct"},
    {"zerodwell", &kZeroDwellMs, "coast dwell at sign crossing ms"},
    {"readmode",  &kReadMode,    "0=all 1=oneper-tick 2=none 3=speed47"},
    {"decelskip", &kDecelSkip,   "1=no reads while pwm changing"},
    {"latchn",    &kLatchN,      "identical reads to declare latch"},
    {"armraw",    &kArmRaw,      "raw movement to arm detector"},
    {"healauto",  &kHealAuto,    "auto re-prime at rest"},
    {"restms",    &kRestMs,      "inter-leg rest ms"},
    {"nwheels",   &kNWheels,     "active motor ports 1..N (max 4)"},
    {"ratchet",   &kRatchet,     "1 = stall/reverse/lunge decel tail"},
    {"sensors",   &kSensors,     "1 = OTOS 0x17 read each tick (bus mix)"},
    {"driver",    &kDriver,      "1 = production Motor class, 0 = raw"},
    {"verbose",   &kVerbose,     "1 = per-leg lines"},
};
static const int kNumKnobs = sizeof(kKnobs) / sizeof(kKnobs[0]);

static inline int nW() { return (kNWheels < 1) ? 1 : (kNWheels > 4 ? 4 : kNWheels); }

// ---------------------------------------------------------------------------
// Bus-op ring (micro I2CLOG) + counters
// ---------------------------------------------------------------------------

struct Op { uint32_t t; uint8_t code; uint8_t m; int16_t st; };
static Op       gRing[32];
static int      gRingHead = 0;
static uint32_t gWrites = 0, gReads = 0, gErrs = 0;

static void ringPush(uint8_t code, uint8_t m, int st)
{
    gRing[gRingHead] = Op{(uint32_t)nowUs(), code, m, (int16_t)st};
    gRingHead = (gRingHead + 1) % 32;
}

static void ringDump()
{
    // One line, oldest->newest: <code><motor>.<dt_us>  e.g. "W1.0 Q1.9 R1.4012"
    char out[240];
    int pos = snprintf(out, sizeof(out), "RING ");
    uint32_t prev = 0;
    for (int i = 0; i < 32; ++i) {
        const Op& e = gRing[(gRingHead + i) % 32];
        if (e.t == 0) continue;
        uint32_t dt = prev ? (e.t - prev) : 0;
        prev = e.t;
        int w = snprintf(out + pos, sizeof(out) - pos, "%c%d%s.%lu ",
                         (char)e.code, (int)e.m, e.st ? "!" : "",
                         (unsigned long)dt);
        if (w <= 0 || w >= (int)sizeof(out) - pos) break;
        pos += w;
    }
    snprintf(out + pos, sizeof(out) - pos, "\r\n");
    uBit.serial.send(out);
}

// ---------------------------------------------------------------------------
// Nezha wire functions (self-contained; addr 0x10; motors = ports 1..4)
// ---------------------------------------------------------------------------

static const uint16_t NZ = 0x10 << 1;   // 8-bit wire address

static int nzWrite8(uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5,
                    uint8_t b6, uint8_t b7)
{
    uint8_t f[8] = {0xFF, 0xF9, b2, b3, b4, b5, b6, b7};
    int st = uBit.i2c.write(NZ, f, 8, false);
    ++gWrites; if (st != MICROBIT_OK) ++gErrs;
    return st;
}

// 0x60 direct PWM. pct in [-100,100]; 0 = coast (NEVER 0x5F here).
static int      gLastPwm[5]  = {0, 0, 0, 0, 0};   // indexed by port 1..4
static uint64_t gLastWrUs[5] = {0, 0, 0, 0, 0};

static void nzPwmRaw(uint8_t motor, int pct)
{
    uint8_t dir = (pct >= 0) ? 1 : 2;
    uint8_t spd = (uint8_t)(pct >= 0 ? pct : -pct);
    nzWrite8(motor, dir, 0x60, spd, 0xF5, 0x00);
    ringPush('W', motor, 0);
    gLastPwm[motor] = pct;
    gLastWrUs[motor] = nowUs();
    if (kPostWriteUs > 0) busyUs((uint32_t)kPostWriteUs);
}

// Policy-shaped PWM write: write-mode, slew cap, zero dwell.
static void nzPwm(uint8_t motor, int pct)
{
    if (pct >  100) pct =  100;
    if (pct < -100) pct = -100;
    int last = gLastPwm[motor];

    if (kWriteMode == 0 && pct == last) return;                 // on-change
    if (kWriteMode == 2 && pct != 0 &&
        (pct > 0) == (last > 0) &&
        (nowUs() - gLastWrUs[motor]) < 40000u) return;          // throttle

    if (kSlewCap > 0) {
        int d = pct - last;
        if (d >  kSlewCap) pct = last + kSlewCap;
        if (d < -kSlewCap) pct = last - kSlewCap;
    }
    if (kZeroDwellMs > 0 && last != 0 && pct != 0 &&
        ((pct > 0) != (last > 0))) {
        nzPwmRaw(motor, 0);                                     // dwell at 0
        uint64_t dl = nowUs() + (uint64_t)kZeroDwellMs * 1000u;
        while (nowUs() < dl) {}
    }
    if (pct == last && kWriteMode == 0) return;
    nzPwmRaw(motor, pct);
}

// Apply one commanded pct to all active wheels (policy-shaped).
// driver=1: production Motor::setSpeed — write-on-change, 40 ms throttle,
//           MotorSlew clamp, stop/reversal exemptions — the REAL policy.
// driver=0: this lab's knob-shaped raw policy (nzPwm).
static int gReqPwm[5] = {0, 0, 0, 0, 0};   // commanded intent per port

static void pwmAll(int pct)
{
    for (int m = 1; m <= nW(); ++m) {
        gReqPwm[m] = pct;
        if (kDriver && gMot[m - 1]) gMot[m - 1]->setSpeed((int8_t)pct);
        else                        nzPwm((uint8_t)m, pct);
    }
}

static void pwmAllRaw(int pct)
{
    for (int m = 1; m <= nW(); ++m) {
        gReqPwm[m] = pct;
        if (kDriver && gMot[m - 1]) gMot[m - 1]->setSpeed((int8_t)pct);
        else                        nzPwmRaw((uint8_t)m, pct);
    }
}

// Commanded intent for detector gating (driver-1's written value is private
// to Motor; intent is the right gate anyway — production's detector gates on
// target, and a latched wheel is latched regardless of slew position).
static int cmdPwm(uint8_t motor) { return kDriver ? gReqPwm[motor] : gLastPwm[motor]; }

// 0x46 encoder read.  ok=false on any bus error (value then meaningless).
static int32_t nzReadAngle(uint8_t motor, bool& ok, bool applyPreIdle = true)
{
    if (applyPreIdle && kPreIdleUs > 0) busyUs((uint32_t)kPreIdleUs);
    int st = nzWrite8(motor, 0x00, 0x46, 0x00, 0xF5, 0x00);
    ringPush('Q', motor, st);
    busyUs((uint32_t)kSettleUs);
    uint8_t r[4] = {0, 0, 0, 0};
    int st2 = uBit.i2c.read(NZ, r, 4, false);
    ++gReads; if (st2 != MICROBIT_OK) ++gErrs;
    ringPush('R', motor, st2);
    ok = (st == MICROBIT_OK && st2 == MICROBIT_OK);
    return (int32_t)(((uint32_t)r[3] << 24) | ((uint32_t)r[2] << 16) |
                     ((uint32_t)r[1] <<  8) |  (uint32_t)r[0]);
}

// 0x47 speed read (unsigned magnitude).
static int32_t nzReadSpeed(uint8_t motor, bool& ok)
{
    if (kPreIdleUs > 0) busyUs((uint32_t)kPreIdleUs);
    int st = nzWrite8(motor, 0x00, 0x47, 0x00, 0xF5, 0x00);
    ringPush('q', motor, st);
    busyUs((uint32_t)kSettleUs);
    uint8_t r[2] = {0, 0};
    int st2 = uBit.i2c.read(NZ, r, 2, false);
    ++gReads; if (st2 != MICROBIT_OK) ++gErrs;
    ringPush('r', motor, st2);
    ok = (st == MICROBIT_OK && st2 == MICROBIT_OK);
    return (int32_t)(((uint16_t)r[1] << 8) | (uint16_t)r[0]);
}

// Vendor-native regime: 0x77 board speed then 0x70 degree move.
static void nzGlobalSpeed(uint8_t pct)
{
    uint16_t enc = (uint16_t)pct * 9;
    nzWrite8(0x00, 0x00, 0x77, (uint8_t)(enc >> 8), 0x00, (uint8_t)(enc & 0xFF));
    ringPush('G', 0, 0);
}

static void nzMoveDegrees(uint8_t motor, uint8_t dir, uint16_t deg)
{
    nzWrite8(motor, dir, 0x70, (uint8_t)(deg >> 8), 2 /*degrees*/,
             (uint8_t)(deg & 0xFF));
    ringPush('M', motor, 0);
}

// ---------------------------------------------------------------------------
// Latch detector (motion-armed) + episode bookkeeping — per port
// ---------------------------------------------------------------------------

struct Wheel {
    uint8_t  id;
    int32_t  last;        // last raw value
    int32_t  cmdStart;    // raw value at command onset
    bool     armed;       // moved since command onset
    int      same;        // consecutive identical reads while commanded+armed
    bool     latched;
    int32_t  latchVal;
    uint32_t episodes;
};
static Wheel gW[4] = {{1,0,0,false,0,false,0,0}, {2,0,0,false,0,false,0,0},
                      {3,0,0,false,0,false,0,0}, {4,0,0,false,0,false,0,0}};
static uint32_t gGlitches  = 0;     // bus-error reads (excluded from latch logic)
static uint32_t gPersist   = 0;
static uint32_t gRunaway   = 0;     // watchdog catches (motion w/o command)
static uint32_t gStopRetry = 0;     // stop-verify re-issues

static void wheelCmdOnset(Wheel& w)
{
    w.cmdStart = w.last; w.armed = false; w.same = 0; w.latched = false;
}

static void onsetAll()
{
    for (int i = 0; i < nW(); ++i) wheelCmdOnset(gW[i]);
}

// BLIND-latch check: a wheel ALREADY latched at command onset never arms
// (its readback never moves), so wheelFeed() cannot see it. Call after the
// wheel has been commanded at cruise long enough that a healthy motor MUST
// have moved.
static void wheelBlindCheck(Wheel& w, const char* phase, uint32_t tick)
{
    if (!w.armed && !w.latched) {
        w.latched = true; w.latchVal = w.last; ++w.episodes;
        P("LATCH-BLIND M%d raw=%ld tick=%lu phase=%s (never armed)\r\n",
          (int)w.id, (long)w.last, (unsigned long)tick, phase);
        ringDump();
    }
}

static void blindCheckAll(const char* phase, uint32_t tick)
{
    if (kReadMode == 0 || kReadMode == 1)
        for (int i = 0; i < nW(); ++i) wheelBlindCheck(gW[i], phase, tick);
}

// Feed one good raw reading; returns true when a NEW latch is declared.
static bool wheelFeed(Wheel& w, int32_t raw, int cmdPct, const char* phase,
                      uint32_t tick)
{
    bool newLatch = false;
    int mag = cmdPct >= 0 ? cmdPct : -cmdPct;
    if (mag >= 5) {
        int32_t moved = raw - w.cmdStart;
        if (moved < 0) moved = -moved;
        if (!w.armed && moved >= kArmRaw) w.armed = true;
        if (w.armed) {
            if (raw == w.last) {
                if (++w.same == kLatchN && !w.latched) {
                    w.latched = true; w.latchVal = raw; ++w.episodes;
                    newLatch = true;
                    P("LATCH M%d raw=%ld tick=%lu phase=%s pwm=%d\r\n",
                      (int)w.id, (long)raw, (unsigned long)tick, phase, cmdPct);
                    ringDump();
                }
            } else {
                if (w.latched)
                    P("UNLATCH M%d raw=%ld (was %ld) tick=%lu\r\n",
                      (int)w.id, (long)raw, (long)w.latchVal,
                      (unsigned long)tick);
                w.same = 0; w.latched = false;
            }
        }
    } else {
        w.same = 0;   // not commanded: freeze checks meaningless
    }
    w.last = raw;
    return newLatch;
}

// Production bus-mix mimic: the real firmware reads the SparkFun OTOS
// (addr 0x17) every 10 ms tick, BETWEEN motor transactions — the brick's
// I2C slave sees every one of those STARTs/address bytes. Register 0x20 =
// pose burst (14 bytes) in the production driver. NACKs (no OTOS powered)
// still put real traffic on the wire; count errors separately.
static uint32_t gSensorReads = 0, gSensorErrs = 0;

static void sensorTraffic()
{
    uint8_t reg = 0x20;
    int st = uBit.i2c.write(0x17 << 1, &reg, 1, false);
    uint8_t buf[14];
    int st2 = uBit.i2c.read(0x17 << 1, buf, 14, false);
    ringPush('S', 0, (st != MICROBIT_OK || st2 != MICROBIT_OK) ? 1 : 0);
    ++gSensorReads;
    if (st != MICROBIT_OK || st2 != MICROBIT_OK) ++gSensorErrs;
}

// Read wheels per readmode; feeds detector. tick used for one-per-tick mode.
static void readWheels(const char* phase, uint32_t tick, bool pwmChanging)
{
    if (kSensors) sensorTraffic();
    if (kReadMode == 2) return;
    if (kDecelSkip && pwmChanging) return;
    bool ok;
    if (kReadMode == 3) {                      // 0x47 during motion
        for (int i = 0; i < nW(); ++i) {
            (void)nzReadSpeed(gW[i].id, ok); if (!ok) ++gGlitches;
        }
        return;                                // latch detector is 0x46-based
    }
    if (kReadMode == 1) {                      // one wheel per tick, rotating
        Wheel& w = gW[tick % (uint32_t)nW()];
        int32_t v = nzReadAngle(w.id, ok);
        if (ok) wheelFeed(w, v, cmdPwm(w.id), phase, tick); else ++gGlitches;
        return;
    }
    // mode 0: all active wheels, M1 first (production order, extended)
    for (int i = 0; i < nW(); ++i) {
        Wheel& w = gW[i];
        if (kDriver && gMot[i]) {
            // Production per-tick read: Motor::tick -> readEncoderMmFSettle
            // (0x46 write, 4 ms busy settle, 4-byte read) + position cache.
            // Detector fed in tenths-of-mm (position()*10) — a latch is
            // EXACT constancy, preserved by the linear conversion.
            gMot[i]->tick(uBit.systemTime());
            int32_t v = (int32_t)(gMot[i]->position() * 10.0f);
            wheelFeed(w, v, cmdPwm(w.id), phase, tick);
        } else {
            int32_t v = nzReadAngle(w.id, ok);
            if (ok) wheelFeed(w, v, cmdPwm(w.id), phase, tick); else ++gGlitches;
        }
    }
}

// ---------------------------------------------------------------------------
// Encoder-truth safety net (see 2026-07-03 post-mortem)
// ---------------------------------------------------------------------------

static void nzPwmGuarded(uint8_t motor, int pct)
{
    busyUs(4000);
    nzPwmRaw(motor, pct);
}

// Guarded double-read standstill probe over all active wheels: max |delta|
// across the window. ok=false if any read errored.
static int32_t standstillDelta(int windowMs, bool& ok)
{
    int32_t a[4] = {0, 0, 0, 0}, b[4] = {0, 0, 0, 0};
    bool oks = true; bool o;
    for (int i = 0; i < nW(); ++i) {
        busyUs(4000); a[i] = nzReadAngle(gW[i].id, o, false); oks = oks && o;
    }
    uBit.sleep(windowMs);
    for (int i = 0; i < nW(); ++i) {
        busyUs(4000); b[i] = nzReadAngle(gW[i].id, o, false); oks = oks && o;
        gW[i].last = b[i];
    }
    ok = oks;
    int32_t worst = 0;
    for (int i = 0; i < nW(); ++i) {
        int32_t d = b[i] - a[i]; if (d < 0) d = -d;
        if (d > worst) worst = d;
    }
    return worst;
}

// Stop ALL motors and prove it with encoders. Re-issues the stop until the
// wheels are physically still (or attempts exhaust). Cross-checks 0x47 so a
// latched 0x46 cannot fake a standstill.
static bool stopVerified(const char* why)
{
    for (int attempt = 0; attempt < 6; ++attempt) {
        for (int m = 1; m <= nW(); ++m) nzPwmGuarded((uint8_t)m, 0);
        if (attempt > 0) ++gStopRetry;
        uBit.sleep(attempt == 0 ? 400 : 250);      // coast-down
        bool ok;
        int32_t d = standstillDelta(250, ok);
        bool spd0 = true;
        if (ok && d <= 3) {
            bool os;
            for (int i = 0; i < nW(); ++i) {
                busyUs(4000);
                int32_t s = nzReadSpeed(gW[i].id, os);
                if (os && s > 2) spd0 = false;
            }
            if (spd0) {
                P("STOPV ok (%s) att=%d d=%ld\r\n", why, attempt, (long)d);
                return true;
            }
        }
        P("STOPV retry (%s) att=%d d=%ld ok=%d spd0=%d\r\n",
          why, attempt, (long)d, (int)ok, (int)spd0);
    }
    P("STOPV FAILED (%s) — WHEELS MAY BE TURNING, CUT POWER\r\n", why);
    return false;
}

// ---------------------------------------------------------------------------
// Heal / recover
// ---------------------------------------------------------------------------

// At-rest re-prime: stop, settle, 3 atomic reads per wheel w/ vendor guards,
// then a nudge to verify the readback responds. Ends encoder-verified still.
static bool healAtRest(bool report)
{
    for (int m = 1; m <= nW(); ++m) nzPwmGuarded((uint8_t)m, 0);
    uBit.sleep(300);
    bool ok, healthy = true;
    for (int i = 0; i < nW(); ++i) {
        Wheel& w = gW[i];
        int32_t a = 0, b = 0;
        for (int k = 0; k < 3; ++k) { busyUs(4000); a = nzReadAngle(w.id, ok); }
        nzPwmGuarded(w.id, 25); uBit.sleep(250);
        nzPwmGuarded(w.id, 0);  uBit.sleep(300);
        busyUs(4000); b = nzReadAngle(w.id, ok);
        bool moved = ok && (b != a);
        if (report) P("HEAL M%d %s (a=%ld b=%ld)\r\n", (int)w.id,
                      moved ? "ok" : "STUCK", (long)a, (long)b);
        if (!moved) healthy = false;
        w.last = b; wheelCmdOnset(w);
        w.latched = false;
    }
    stopVerified("heal-end");
    return healthy;
}

// Escalating recovery ladder for a persistent latch.
static void recoverLadder()
{
    P("RECOVER rung1: at-rest re-prime\r\n");
    if (healAtRest(true)) { P("RECOVER ok at rung1\r\n"); return; }
    P("RECOVER rung2: 20x spaced atomic reads\r\n");
    bool ok;
    for (int k = 0; k < 20; ++k)
        for (int i = 0; i < nW(); ++i) { busyUs(8000); nzReadAngle(gW[i].id, ok); }
    if (healAtRest(true)) { P("RECOVER ok at rung2\r\n"); return; }
    P("RECOVER rung3: bus re-init (setFrequency)\r\n");
    uBit.i2c.setFrequency((uint32_t)kBusKhz * 1000u);
    uBit.sleep(50);
    if (healAtRest(true)) { P("RECOVER ok at rung3\r\n"); return; }
    P("RECOVER rung4: 0x77 global-speed poke + re-prime\r\n");
    nzGlobalSpeed(50); uBit.sleep(50);
    if (healAtRest(true)) { P("RECOVER ok at rung4\r\n"); return; }
    ++gPersist;
    P("RECOVER FAILED — persistent latch (power-cycle territory)\r\n");
}

// ---------------------------------------------------------------------------
// Patterns.  All: deadline-scheduled ticks; any serial byte aborts; all end
// with an encoder-verified stop.
// ---------------------------------------------------------------------------

static bool userAbort()
{
    return uBit.serial.isReadable() > 0;
}

static void printEpisodes(const char* pat, int n)
{
    P("RESULT %s n=%d ep=%lu,%lu,%lu,%lu glitch=%lu errs=%lu persist=%lu\r\n",
      pat, n,
      (unsigned long)gW[0].episodes, (unsigned long)gW[1].episodes,
      (unsigned long)gW[2].episodes, (unsigned long)gW[3].episodes,
      (unsigned long)gGlitches, (unsigned long)gErrs,
      (unsigned long)gPersist);
}

// One production-like leg: slew to +/-cruise, hold, decel toward zero with a
// bounded dither window, stop. Direction alternates per leg.
static void runLegs(int n)
{
    uint32_t tick = 0;
    for (int leg = 0; leg < n && !userAbort(); ++leg) {
        int sign = (leg & 1) ? -1 : 1;
        int target = kCruise * sign;
        onsetAll();
        const char* phase = "ramp";
        int pwm = 0;
        int ditherLeft = 0;
        uint64_t phaseEnd = 0;
        bool holdSet = false;
        uint64_t next = nowUs();
        while (!userAbort()) {
            uint64_t t = nowUs();
            if (t < next) continue;
            next = t + (uint64_t)kTickUs;
            ++tick;
            int prev = pwm;
            if (phase[0] == 'r') {                       // ramp
                pwm = target;
                bool allAt = true;
                for (int m = 1; m <= nW(); ++m)
                    if (gLastPwm[m] != target) allAt = false;
                if (allAt) { phase = "hold"; holdSet = false; }
            } else if (phase[0] == 'h') {                // hold
                if (!holdSet) { phaseEnd = t + 400000u; holdSet = true; }
                pwm = target;
                if (t >= phaseEnd) {
                    // end of a full-speed hold: a healthy wheel MUST have
                    // moved by now — catch pre-onset (blind) latches.
                    blindCheckAll("hold", tick);
                    phase = "decel";
                }
            } else if (phase[0] == 'd') {                // decel
                // Step |pwm| toward zero relative to the CURRENT value —
                // never a fixed direction. (v1 stepped a fixed -1 on
                // forward legs; one -1 dither write escaped the band and
                // walked to -100. See the 2026-07-03 post-mortem.)
                int cur = gLastPwm[1];
                int mag = (cur >= 0) ? cur : -cur;
                if (mag > kDither) {
                    pwm = (cur > 0) ? cur - 1 : cur + 1;  // toward zero
                    ditherLeft = 12;    // PID-mimic sign-noise window at end
                } else if (ditherLeft > 0) {
                    --ditherLeft;
                    pwm = (ditherLeft == 0) ? 0
                        : ((tick & 1) ? kDither : -kDither);
                    if ((tick % 6) == 0) pwm = 0;
                } else {
                    pwm = 0;
                    if (cur == 0) phase = "stop";
                }
            } else if (phase[0] == 't') {                // ratchet tail
                // D-drive terminal instability mimic (docs/knowledge
                // 2026-07-02): stall at 0, brief REVERSE kick, then a
                // forward LUNGE, then hard 0. Where the 0.63/leg article
                // actually latched (enc = decel landing point).
                if (ditherLeft > 8)      pwm = 0;                    // stall
                else if (ditherLeft > 5) pwm = (sign > 0) ? -12 : 12; // kick
                else if (ditherLeft > 2) pwm = (sign > 0) ?  14 : -14; // lunge
                else                     pwm = 0;
                if (--ditherLeft <= 0) { pwmAllRaw(0); break; }
            } else {                                      // stop
                if (kRatchet) { phase = "tail"; ditherLeft = 11; continue; }
                pwmAllRaw(0);
                break;
            }
            // COMMAND INVARIANT: a leg may never command beyond its own
            // cruise magnitude (+ dither; ratchet tail allowed its fixed
            // +-14 kick/lunge). If the state machine produces more, that is
            // a lab bug — stop everything loudly.
            int lim = (kCruise > 0 ? kCruise : -kCruise) + kDither;
            if (phase[0] == 't' && lim < 14) lim = 14;
            if (pwm > lim || pwm < -lim) {
                P("PWM-INVARIANT-VIOLATION pwm=%d lim=%d leg=%d phase=%s "
                  "tick=%lu — aborting run\r\n",
                  pwm, lim, leg, phase, (unsigned long)tick);
                stopVerified("pwm-invariant");
                return;
            }
            int before = gLastPwm[1];
            pwmAll(pwm);
            bool changing = (gLastPwm[1] != before) || (prev != pwm);
            readWheels(phase, tick, changing);
            if (phase[0] == 'd') {
                bool allZero = (pwm == 0);
                for (int m = 1; m <= nW(); ++m)
                    if (gLastPwm[m] != 0) allZero = false;
                if (allZero) phase = "stop";
            }
        }
        // inter-leg rest; auto-heal if latched
        bool anyLatched = false;
        for (int i = 0; i < nW(); ++i) if (gW[i].latched) anyLatched = true;
        if (anyLatched && kHealAuto) healAtRest(false);
        // rest-quiet invariant: nothing commanded => encoders still.
        for (int m = 1; m <= nW(); ++m) nzPwmGuarded((uint8_t)m, 0);
        uBit.sleep(kRestMs > 250 ? kRestMs - 250 : 0);
        bool stillOk;
        int32_t d = standstillDelta(200, stillOk);
        if (stillOk && d > 5) {
            ++gRunaway;
            P("MOTION-AT-REST leg=%d d=%ld — forcing stop\r\n", leg, (long)d);
            ringDump();
            stopVerified("rest");
        }
        if (kVerbose)
            P("LEG %d/%d ep=%lu,%lu,%lu,%lu\r\n", leg + 1, n,
              (unsigned long)gW[0].episodes, (unsigned long)gW[1].episodes,
              (unsigned long)gW[2].episodes, (unsigned long)gW[3].episodes);
    }
    stopVerified("legs-end");
    printEpisodes("legs", n);
}

// Full reversals every 1.2 s (stress-matrix arm 5), all wheels together.
static void runSlam(int n)
{
    onsetAll();
    uint32_t tick = 0;
    for (int i = 0; i < n && !userAbort(); ++i) {
        int pct = (i & 1) ? -kSlamPct : kSlamPct;
        pwmAllRaw(pct);                            // slam = unslewed by design
        onsetAll();
        uint64_t end = nowUs() + 1200000u, next = nowUs();
        while (nowUs() < end && !userAbort()) {
            if (nowUs() < next) continue;
            next = nowUs() + (uint64_t)kTickUs;
            ++tick;
            readWheels("slam", tick, false);
        }
        blindCheckAll("slam", tick);
    }
    stopVerified("slam-end");
    printEpisodes("slam", n);
}

// Cruise + a 3-read atomic burst (no pre-idle) every 1.2 s (arm 3).
static void runBurst(int n)
{
    pwmAllRaw(kCruise);
    onsetAll();
    uint32_t tick = 0;
    for (int i = 0; i < n && !userAbort(); ++i) {
        uint64_t end = nowUs() + 1200000u, next = nowUs();
        while (nowUs() < end && !userAbort()) {
            if (nowUs() < next) continue;
            next = nowUs() + (uint64_t)kTickUs;
            ++tick;
            readWheels("burst", tick, false);
        }
        bool ok;
        for (int k = 0; k < 3; ++k)               // trigger: burst while moving
            for (int w = 0; w < nW(); ++w)
                (void)nzReadAngle(gW[w].id, ok, false);
        blindCheckAll("burst", tick);
    }
    stopVerified("burst-end");
    printEpisodes("burst", n);
}

// Slam + burst together (arms 1-2; ESCALATES to persistent — use sparingly).
static void runCombo(int n)
{
    onsetAll();
    uint32_t tick = 0;
    for (int i = 0; i < n && !userAbort(); ++i) {
        int pct = (i & 1) ? -kSlamPct : kSlamPct;
        pwmAllRaw(pct);
        onsetAll();
        bool ok;
        for (int k = 0; k < 3; ++k)
            for (int w = 0; w < nW(); ++w)
                (void)nzReadAngle(gW[w].id, ok, false);
        uint64_t end = nowUs() + 1200000u, next = nowUs();
        while (nowUs() < end && !userAbort()) {
            if (nowUs() < next) continue;
            next = nowUs() + (uint64_t)kTickUs;
            ++tick;
            readWheels("combo", tick, false);
        }
        blindCheckAll("combo", tick);
    }
    stopVerified("combo-end");
    printEpisodes("combo", n);
}

// Vendor-regime legs: board speed + 0x70 degree moves; read only at rest.
static void runNative(int n)
{
    nzGlobalSpeed((uint8_t)kCruise);
    bool ok;
    for (int i = 0; i < n && !userAbort(); ++i) {
        uint8_t dir = (i & 1) ? 2 : 1;
        int32_t before[4];
        for (int w = 0; w < nW(); ++w)
            before[w] = nzReadAngle(gW[w].id, ok, true);
        for (int w = 0; w < nW(); ++w)
            nzMoveDegrees(gW[w].id, dir, 540);
        busyUs(4000);                             // vendor post-command wait
        int waitMs = 540000 / (kCruise * 9) + 500;
        uBit.sleep(waitMs);
        busyUs(4000);
        for (int w = 0; w < nW(); ++w) {
            int32_t after = nzReadAngle(gW[w].id, ok, true);
            int32_t d = after - before[w]; if (d < 0) d = -d;
            if (d < kArmRaw) {
                ++gW[w].episodes;
                P("NATIVE-STALE M%d leg=%d d=%ld\r\n", (int)gW[w].id, i, (long)d);
            }
        }
        uBit.sleep(kRestMs);
    }
    stopVerified("native-end");
    printEpisodes("native", n);
}

// Production-faithful RESET-WHILE-MOVING (stress-matrix arm 3, done right):
// cruise, and every 800 ms fire a full Motor::resetEncoder-equivalent burst
// per wheel — 3x [4ms idle + 0x46 + 4ms settle + read] + a verify read —
// then IMMEDIATELY flip the command (a D-preemption does reset+new-target).
// Arm 3 produced 13 transient latches / 10 cycles on the old drivetrain.
static void runReset(int n)
{
    int pct = kCruise;
    pwmAllRaw(pct);
    onsetAll();
    uint32_t tick = 0;
    for (int i = 0; i < n && !userAbort(); ++i) {
        uint64_t end = nowUs() + 800000u, next = nowUs();
        while (nowUs() < end && !userAbort()) {
            if (nowUs() < next) continue;
            next = nowUs() + (uint64_t)kTickUs;
            ++tick;
            readWheels("reset", tick, false);
        }
        // blind check belongs HERE — end of the cruise window, while this
        // window's arming state is still valid. (First version checked
        // right after the flip's onsetAll(): "never armed" fired every
        // cycle by construction — 20/20 false positives, caught because
        // heal showed live readbacks. Detector bugs get post-mortems too.)
        blindCheckAll("reset", tick);
        // the trigger: full guarded atomic-read reset burst, wheels MOVING.
        // driver=1: the REAL Motor::resetEncoder() — median-of-3 atomic reads
        // + readback-verify; on a MOVING wheel the verify fails, so it
        // retries up to 3 full attempts = up to ~16 atomic reads. This is
        // the exact production D-preemption burst, much heavier than the
        // raw mimic below.
        bool ok;
        for (int w = 0; w < nW(); ++w) {
            if (kDriver && gMot[w]) {
                gMot[w]->resetEncoder();
            } else {
                for (int k = 0; k < 3; ++k) { busyUs(4000); nzReadAngle(gW[w].id, ok, false); }
                busyUs(4000); (void)nzReadAngle(gW[w].id, ok, false);   // verify read
            }
        }
        // ...followed immediately by a new command (alternate direction),
        // exactly like a D-preemption reset+retarget.
        pct = -pct;
        pwmAllRaw(pct);
        onsetAll();
    }
    stopVerified("reset-end");
    printEpisodes("reset", n);
}

// Constant-speed spin with reads (control condition — historically clean).
static void runSpin(int sec)
{
    pwmAllRaw(kCruise);
    onsetAll();
    uint64_t end = nowUs() + (uint64_t)sec * 1000000u, next = nowUs();
    uint32_t tick = 0;
    while (nowUs() < end && !userAbort()) {
        if (nowUs() < next) continue;
        next = nowUs() + (uint64_t)kTickUs;
        ++tick;
        readWheels("spin", tick, false);
    }
    blindCheckAll("spin", tick);
    stopVerified("spin-end");
    printEpisodes("spin", sec);
}

// ---------------------------------------------------------------------------
// Serial console
// ---------------------------------------------------------------------------

static void resetCounters()
{
    for (int i = 0; i < 4; ++i) gW[i].episodes = 0;
    gGlitches = gErrs = 0; gWrites = gReads = 0; gPersist = 0;
}

static void printKnobs()
{
    for (int i = 0; i < kNumKnobs; ++i)
        P("K %-10s %6d  %s\r\n", kKnobs[i].name, *kKnobs[i].val,
          kKnobs[i].help);
}

static void handleLine(char* s)
{
    char* tok[4] = {nullptr, nullptr, nullptr, nullptr};
    int nt = 0;
    for (char* p = s; *p && nt < 4;) {
        while (*p == ' ') ++p;
        if (!*p) break;
        tok[nt++] = p;
        while (*p && *p != ' ') ++p;
        if (*p) *p++ = 0;
    }
    if (nt == 0) return;

    if (!strcmp(tok[0], "get")) { printKnobs(); return; }
    if (!strcmp(tok[0], "set") && nt >= 3) {
        for (int i = 0; i < kNumKnobs; ++i) {
            if (!strcmp(tok[1], kKnobs[i].name)) {
                *kKnobs[i].val = atoi(tok[2]);
                if (kKnobs[i].val == &kBusKhz)
                    uBit.i2c.setFrequency((uint32_t)kBusKhz * 1000u);
                P("OK %s=%d\r\n", kKnobs[i].name, *kKnobs[i].val);
                return;
            }
        }
        P("ERR unknown knob %s\r\n", tok[1]);
        return;
    }
    if (!strcmp(tok[0], "run") && nt >= 3) {
        int n = atoi(tok[2]);
        resetCounters();
        P("RUN %s %d (any byte aborts)\r\n", tok[1], n);
        if      (!strcmp(tok[1], "legs"))   runLegs(n);
        else if (!strcmp(tok[1], "slam"))   runSlam(n);
        else if (!strcmp(tok[1], "burst"))  runBurst(n);
        else if (!strcmp(tok[1], "combo"))  runCombo(n);
        else if (!strcmp(tok[1], "native")) runNative(n);
        else if (!strcmp(tok[1], "reset"))  runReset(n);
        else if (!strcmp(tok[1], "spin"))   runSpin(n);
        else P("ERR unknown pattern %s\r\n", tok[1]);
        while (uBit.serial.isReadable() > 0) uBit.serial.read(ASYNC);
        return;
    }
    if (!strcmp(tok[0], "heal"))    { healAtRest(true);  return; }
    if (!strcmp(tok[0], "recover")) { recoverLadder();   return; }
    if (!strcmp(tok[0], "stop"))    { stopVerified("user"); return; }
    if (!strcmp(tok[0], "stat")) {
        P("STAT2 sens=%lu senserr=%lu\r\n",
          (unsigned long)gSensorReads, (unsigned long)gSensorErrs);
        P("STAT w=%lu r=%lu err=%lu glitch=%lu ep=%lu,%lu,%lu,%lu persist=%lu "
          "runaway=%lu stopretry=%lu\r\n",
          (unsigned long)gWrites, (unsigned long)gReads, (unsigned long)gErrs,
          (unsigned long)gGlitches,
          (unsigned long)gW[0].episodes, (unsigned long)gW[1].episodes,
          (unsigned long)gW[2].episodes, (unsigned long)gW[3].episodes,
          (unsigned long)gPersist, (unsigned long)gRunaway,
          (unsigned long)gStopRetry);
        return;
    }
    if (!strcmp(tok[0], "ping")) { P("PONG wedgelab\r\n"); return; }
    P("ERR unknown cmd %s\r\n", tok[0]);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main()
{
    uBit.init();
    uBit.serial.setTxBufferSize(250);
    uBit.serial.setRxBufferSize(64);
    uBit.serial.setBaud(115200);
    uBit.i2c.setFrequency((uint32_t)kBusKhz * 1000u);
    uBit.sleep(1000);                       // brick power-up settle

    // Production motor layer: verbatim I2CBus wrapper (IRQ-guarded
    // transactions) + Motor instances, one per port. fwdSign +1 everywhere
    // (direction convention is irrelevant on the stand); wheelTravelCalib
    // nonzero so position() is a faithful linear image of the raw encoder.
    gCfg.fwdSignL = 1; gCfg.fwdSignR = 1;
    gCfg.wheelTravelCalibL = 0.7f;
    gCfg.wheelTravelCalibR = 0.7f;
    static I2CBus bus(uBit.i2c);
    static Motor m1(bus, 1, 1, gCfg), m2(bus, 2, 1, gCfg),
                 m3(bus, 3, 1, gCfg), m4(bus, 4, 1, gCfg);
    gBus = &bus;
    gMot[0] = &m1; gMot[1] = &m2; gMot[2] = &m3; gMot[3] = &m4;
    // Production boot prime (NezhaHAL::begin does exactly this per motor).
    for (int i = 0; i < 4; ++i) gMot[i]->begin();

    P("\r\nWEDGELAB v3 (4-motor, dual-driver) built %s %s\r\n",
      __DATE__, __TIME__);
    P("driver=%d (1=production Motor class, 0=raw lab)\r\n", kDriver);
    P("ports: M1/M2 = old (latch-prone), M3/M4 = fresh. nwheels=%d\r\n", nW());
    P("commands: get | set k v | run legs|slam|burst|combo|native|spin N | "
      "heal | recover | stat | stop\r\n");

    // prime the readback registers (vendor boot behavior), all ports
    bool ok;
    for (int i = 0; i < nW(); ++i) {
        busyUs(4000); (void)nzReadAngle(gW[i].id, ok, true);
        busyUs(4000); gW[i].last = nzReadAngle(gW[i].id, ok, true);
    }
    P("BOOT enc=%ld,%ld,%ld,%ld ok=%d\r\n",
      (long)gW[0].last, (long)gW[1].last, (long)gW[2].last, (long)gW[3].last,
      (int)ok);

    char buf[80];
    int  len = 0;
    uint64_t nextWatch = nowUs() + 500000u;
    while (true) {
        // ---- idle runaway watchdog: encoders must be still at console ----
        if (nowUs() >= nextWatch) {
            nextWatch = nowUs() + 500000u;
            bool wok;
            int32_t d = standstillDelta(150, wok);
            if (wok && d > 20) {
                ++gRunaway;
                P("RUNAWAY-AT-IDLE d=%ld — forcing stop\r\n", (long)d);
                ringDump();
                stopVerified("idle-watchdog");
            }
        }
        int c = uBit.serial.read(ASYNC);
        if (c == MICROBIT_NO_DATA || c < 0) { uBit.sleep(5); continue; }
        if (c == '\r') continue;
        if (c == '\n') {
            buf[len] = 0;
            if (len > 0) handleLine(buf);
            len = 0;
            continue;
        }
        if (len < (int)sizeof(buf) - 1) buf[len++] = (char)c;
    }
}
