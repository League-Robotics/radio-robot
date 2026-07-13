// bringup_main.cpp — DB-009: the dedicated DeviceBus HITL bring-up firmware
// image. `Devices::DeviceBus` is the ONLY thing running here — this is the
// "own dedicated main" resolution of clasi/issues/device-bus-fiber-owned-
// self-contained-device-subsystem.md's open question 5 ("greenfield
// directory, tested via its own dedicated main where DeviceBus is the only
// thing running, driven directly via DEV commands. No consumer cutover in
// the same sprint, and no runtime coexistence with the legacy stack, ever").
//
// This translation unit's own dependencies are, deliberately, ONLY
// `Devices::` (source/devices/*.h) and CODAL/`MicroBit.h` — device-bus-
// tickets.md's DB-009 description: "Keep it minimal + self-contained (uses
// only Devices:: + CODAL)." It never includes source/main.cpp, anything
// under source/subsystems|commands|com|hal|messages|config, or
// source/com/serial_port.h — even though the tiny line-buffered serial
// reader below looks a lot like that file, it is a fresh, local
// re-implementation against the same CODAL primitives (NRF52Serial::
// read(ASYNC)/send(), MICROBIT_NO_DATA), not a #include of it, so this
// file's include list stays exactly what the isolation invariant
// (device-bus-tickets.md's "Standing isolation invariant") already holds
// source/devices/*.{h,cpp} to. Building this file into an image alongside
// source/main.cpp would violate "no runtime coexistence with the legacy
// stack, ever" (both would define `main()`, which will not even link) —
// codal.devicebus.json (repo root) + the CMakeLists.txt "application_entry"
// switch it sets are what select THIS file as the image entry point
// instead of source/main.cpp; see CMakeLists.txt's own comment on that
// block and this ticket's own report for the exact build command.
//
// --- What runs ---
// One `Devices::DeviceBus`, constructed against the real hardware I2C bus
// (`uBit.i2c`) and started once at boot — `bus.start()` spawns the real
// `CodalFiberRunner` fiber (device_bus.cpp/fiber_runner.h, DB-008), which
// runs the detection preamble then the straight-line request/settle/collect/
// perceive/publish cycle (device_bus.h's own header comment) forever, fully
// asynchronously from this file's own foreground loop. This file's
// foreground loop does exactly one more thing: pump a tiny line-buffered
// serial reader and dispatch each line as one DEV command against the
// SAME DeviceBus handles (`bus.motor(port)`/`bus.color()`/`bus.line()`/
// `bus.odometer()`) any other consumer would use — it never reaches past
// the handle API into a leaf or the bus directly (device-bus-tickets.md's
// own "no coexistence" spirit extends to this file's own conduct, not just
// what it links).
//
// --- The DEV command grammar (this file's own bespoke, minimal parser —
// NOT docs/protocol-v2.md's `DEV ...` family, which is the legacy stack's
// text_channel.cpp surface; this bring-up image has no CommandProcessor at
// all) ---
//   PING                              -> OK pong
//   RUNNING                           -> OK running=<0|1>
//   STOP                              -> OK  (both motors -> Neutral::Coast)
//   M <port:1|2> VEL <mm/s>           -> OK  (Motor::setVelocity)
//   M <port> DUTY <-1..1>             -> OK  (Motor::setDuty, PID-off only)
//   M <port> PID <0|1>                -> OK  (Motor::setPidEnabled)
//   M <port> NEUTRAL <C|B>            -> OK  (Motor::setNeutral)
//   M <port> RESET                    -> OK  (Motor::resetPosition)
//   M <port> STATE                    -> OK pos= vel= applied= t= valid= conn= wedged= suspect= glitch=
//   M <port> RING <age 0..4>          -> OK age= pos= vel= applied= t= valid=
//   COLOR                             -> OK r= g= b= c= t= conn=
//   COLOR RING <age 0..4>             -> OK age= r= g= b= c= t= valid=
//   LINE                              -> OK r0= r1= r2= r3= n0= n1= n2= n3= t= conn=
//   LINE RING <age 0..4>              -> OK age= r0.. n3.. t= valid=
//   ODO                               -> OK x= y= h= vx= vy= w= t= conn=
//   ODO SETPOSE <x mm> <y mm> <h rad> -> OK  (Odometer::setPose)
//   ODO RING <age 0..4>               -> OK age= x= y= h= vx= vy= w= t= valid=
//   anything else                     -> ERR unknown / ERR badport / ERR badarg / ...
//
// Any line may end with a trailing " #<digits>" correlation-id suffix
// (matching host/robot_radio/io/serial_conn.py's `SerialConnection.send()`,
// which appends one to every command and blocks on a reply carrying the
// SAME suffix back) — stripped before dispatch, echoed back verbatim on the
// reply. tests/bench/device_bus_bringup.py drives this exact grammar.
//
// --- Why floats are never formatted with printf's %f here ---
// This target's C library is newlib-nano (--specs=nano.specs): %f/%e/%g
// silently emit NOTHING (not even a garbled number — an empty field), a
// project finding documented from a prior debugging session (the codebase's
// `newlib-nano-no-printf-float` knowledge note: "firmware %f emits NOTHING
// (sim fine)... wire floats via integer tenths"). Every float this file
// serializes goes through formatFixed() below instead, which uses only
// integer conversions (%ld/%0*ld) — safe under nano-spec newlib.
#include "MicroBit.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "devices/device_bus.h"
#include "devices/device_config.h"
#include "devices/device_types.h"
#include "devices/measurement_ring.h"

static MicroBit uBit;

namespace {

// ---------------------------------------------------------------------------
// BringupSerial — a minimal, self-contained line-buffered 115200-baud
// serial reader/writer over uBit.serial (NRF52Serial&). Deliberately NOT
// source/com/serial_port.h (this file's own header comment) — the same
// ASYNC-read / MICROBIT_NO_DATA / ManagedString-send primitives that file
// wraps, reproduced locally so this translation unit's includes stay
// Devices:: + CODAL only.
// ---------------------------------------------------------------------------
class BringupSerial {
 public:
  explicit BringupSerial(NRF52Serial& serial) : serial_(serial) {}

  void begin() {
    serial_.setRxBufferSize(255);
    serial_.setTxBufferSize(255);
    serial_.setBaud(115200);
  }

  // Non-blocking. Accumulates bytes from the ASYNC receive buffer; returns
  // true once a complete '\n'-terminated line is ready. `buf` is
  // NUL-terminated with the newline stripped; `len` includes the NUL.
  bool readLine(char* buf, uint16_t len) {
    int c;
    while ((c = serial_.read(ASYNC)) != MICROBIT_NO_DATA) {
      if (c == '\r') continue;
      if (c == '\n') {
        rxBuf_[rxLen_] = '\0';
        uint16_t copy = (rxLen_ < len - 1) ? rxLen_ : static_cast<uint16_t>(len - 1);
        memcpy(buf, rxBuf_, copy);
        buf[copy] = '\0';
        rxLen_ = 0;
        return true;
      }
      if (rxLen_ < sizeof(rxBuf_) - 1) rxBuf_[rxLen_++] = static_cast<char>(c);
    }
    return false;
  }

  void send(const char* msg) {
    serial_.send(ManagedString(msg) + ManagedString("\r\n"), ASYNC);
  }

 private:
  NRF52Serial& serial_;
  char rxBuf_[256] = {};
  uint16_t rxLen_ = 0;
};

// formatFixed — serializes `value` as a fixed-point decimal with `decimals`
// fractional digits, WITHOUT printf's %f conversion. See this file's own
// header comment ("Why floats are never formatted with printf's %f here").
void formatFixed(char* out, size_t cap, float value, int decimals) {
  long scale = 1;
  for (int i = 0; i < decimals; ++i) scale *= 10;
  bool neg = value < 0.0f;
  float mag = neg ? -value : value;
  long scaled = static_cast<long>(mag * static_cast<float>(scale) + 0.5f);
  long intPart = scaled / scale;
  long fracPart = scaled % scale;
  if (decimals > 0) {
    snprintf(out, cap, "%s%ld.%0*ld", neg ? "-" : "", intPart, decimals, fracPart);
  } else {
    snprintf(out, cap, "%s%ld", neg ? "-" : "", intPart);
  }
}

// formatU64 — serializes a uint64_t as decimal WITHOUT printf's %llu
// conversion. newlib-nano (--specs=nano.specs) does not implement the %ll
// length modifier and silently emits the literal "lu" for it — the same
// nano-spec gap this file's header documents for %f, and the cause of the
// `t=lu` fields the DB-009 bench observed. 64-bit integer *arithmetic* (the
// /10 and %10 below, via libgcc's __udivdi3/__umoddi3) IS supported — only
// the %ll format conversion is missing — so manual digit extraction is safe
// and keeps the full 64-bit [us] stamp range (OQ3 chose uint64 for
// wrap-freedom; truncating in the formatter would defeat that).
void formatU64(char* out, size_t cap, uint64_t value) {
  if (cap == 0) return;
  char tmp[20];  // uint64_t max is 20 decimal digits
  int i = 0;
  do {
    tmp[i++] = static_cast<char>('0' + static_cast<int>(value % 10));
    value /= 10;
  } while (value != 0 && i < static_cast<int>(sizeof(tmp)));
  size_t j = 0;
  while (i > 0 && j + 1 < cap) out[j++] = tmp[--i];  // reverse into out
  out[j] = '\0';
}

// ---------------------------------------------------------------------------
// ReplyBuilder — appends "OK"/"ERR ..." plus " key=value" fields into a
// fixed caller-owned buffer. Every numeric append is bounds-checked against
// the buffer's remaining room via snprintf's own return value.
// ---------------------------------------------------------------------------
class ReplyBuilder {
 public:
  ReplyBuilder(char* buf, size_t cap) : buf_(buf), cap_(cap) { buf_[0] = '\0'; }

  void raw(const char* s) { append(s); }

  void kvFloat(const char* key, float value, int decimals = 3) {
    char valStr[32];
    formatFixed(valStr, sizeof(valStr), value, decimals);
    char field[48];
    snprintf(field, sizeof(field), " %s=%s", key, valStr);
    append(field);
  }

  void kvU32(const char* key, uint32_t value) {
    char field[40];
    snprintf(field, sizeof(field), " %s=%lu", key, static_cast<unsigned long>(value));
    append(field);
  }

  void kvU64(const char* key, uint64_t value) {
    char valStr[24];
    formatU64(valStr, sizeof(valStr), value);  // NOT %llu — nano-spec gap
    char field[48];
    snprintf(field, sizeof(field), " %s=%s", key, valStr);
    append(field);
  }

  void kvInt(const char* key, int value) {
    char field[32];
    snprintf(field, sizeof(field), " %s=%d", key, value);
    append(field);
  }

 private:
  void append(const char* s) {
    if (pos_ + 1 >= cap_) return;  // no room left even for a NUL
    int n = snprintf(buf_ + pos_, cap_ - pos_, "%s", s);
    if (n <= 0) return;
    size_t written = static_cast<size_t>(n);
    size_t avail = cap_ - pos_ - 1;
    pos_ += (written < avail) ? written : avail;
  }

  char* buf_;
  size_t cap_;
  size_t pos_ = 0;
};

// ---------------------------------------------------------------------------
// Config builders — Devices-local MotorConfig/OtosConfig literals mirroring
// data/robots/tovez.json's calibration.*/control.vel_* values (the SAME
// numbers source/config/boot_config.cpp bakes for the legacy stack's boot
// path), duplicated here rather than included: the isolation invariant this
// file's own header comment describes forbids `#include "config/
// boot_config.h"` from source/devices/, and this file holds itself to the
// identical rule. This is a bounded, deliberate duplication for bring-up
// purposes only — the bring-up image is a diagnostic tool for DeviceBus's
// OWN mechanics (bench gates: request/collect pipelining, fiber_sleep(4)
// latency, reversal-stress armor, serial health, flash/RAM delta), not a
// calibration source of truth; DEV commands (`M <port> VEL/DUTY/PID`) let a
// bench operator drive either motor directly regardless of what these
// defaults are. ColorConfig/LineConfig are left at their Devices-local
// zero-valued defaults on purpose — color_sensor.cpp/line_sensor.cpp (DB-006)
// both auto-substitute ship defaults for every zero-valued field that has
// one (kDefaultLagColor/kDefaultIntegration/kDefaultGain/kDefaultLagLine/
// kDefaultCalMax), the same pattern MotorConfig's own reversalDwell/
// outputDeadband/slewRate zero-defaults already rely on below.
Devices::MotorConfig buildMotorConfig(uint32_t port, int32_t fwdSign, float travelCalib) {
  Devices::MotorConfig cfg;
  cfg.port = port;
  cfg.fwdSign = fwdSign;
  cfg.wheelTravelCalib = travelCalib;  // [mm/deg]
  cfg.velGains.kp = 0.0014f;
  cfg.velGains.ki = 0.005f;
  cfg.velGains.kff = 0.00135f;
  cfg.velGains.iMax = 0.3f;
  cfg.velGains.kaw = 20.0f;
  cfg.velFiltAlpha = 0.3f;
  // velDeadband, slewRate, reversalDwell, outputDeadband: left at
  // Devices::MotorConfig's zero/unset defaults on purpose (see this
  // function's own header comment) -- NezhaMotor/MotorArmor auto-substitute
  // their own ship defaults (nezha_motor.cpp's `config_.slewRate <= 0.0f`
  // fallback to kDefaultSlewRate; motor_armor.h's `Opt<float>.has` fallback
  // to kDefaultReversalDwell/kDefaultOutputDeadband) whenever these are left
  // unset, exactly as the legacy boot path's own generated defaults
  // (source/config/boot_config.cpp) do.
  cfg.polled = true;
  return cfg;
}

Devices::OtosConfig buildOtosConfig() {
  Devices::OtosConfig cfg;
  cfg.offsetX = -47.7f;  // [mm]
  cfg.offsetY = 3.5f;    // [mm]
  cfg.offsetYaw = 0.0f;  // [rad]
  cfg.linearScale = 1.067f;
  cfg.angularScale = 0.987f;
  return cfg;
}

// ---------------------------------------------------------------------------
// dispatch — parses one already corr-id-stripped, space-tokenized command
// line and fills `reply`. See this file's own header comment for the full
// grammar. Never touches the bus directly -- only ever calls through the
// `bus.motor()/color()/line()/odometer()` handle API (device_bus.h),
// mirroring how any other DeviceBus consumer would.
// ---------------------------------------------------------------------------
void dispatchRing(const Devices::Sample<Devices::MotorReading>& s, int age, ReplyBuilder& reply) {
  reply.raw("OK");
  reply.kvInt("age", age);
  reply.kvFloat("pos", s.value.position);
  reply.kvFloat("vel", s.value.velocity);
  reply.kvFloat("applied", s.value.appliedDuty, 3);
  reply.kvU64("t", s.stamp);
  reply.kvInt("valid", s.valid ? 1 : 0);
}

// servoPin -- map a bring-up SERVO <pin> selector to a micro:bit edge pin
// (101-001). The OTOS-servo signal on the rig is a direct micro:bit PWM pin
// (independent of the DeviceBus I2C), so it is driven with CODAL's
// setServoValue() directly, resurrecting source_old/hal/real/Servo.cpp's one
// line. Selectors follow source_old/hal/real/PortIO.cpp's Nezha port->pin
// table so the rig's J1/S1 can be found empirically by sweeping selectors and
// watching OTOS heading (ODO) respond: 1..4 = the digital pins of Nezha ports
// 1..4, 5..8 = the analog pins of Nezha ports 1..4, 0 = P0.
MicroBitPin* servoPin(int sel) {
  switch (sel) {
    case 0: return &uBit.io.P0;
    case 1: return &uBit.io.P8;    // Nezha port 1 digital (J1)
    case 2: return &uBit.io.P12;   // port 2 digital
    case 3: return &uBit.io.P14;   // port 3 digital
    case 4: return &uBit.io.P16;   // port 4 digital
    case 5: return &uBit.io.P1;    // Nezha port 1 analog (J1/S1 candidate)
    case 6: return &uBit.io.P2;    // port 2 analog
    case 7: return &uBit.io.P13;   // port 3 analog
    case 8: return &uBit.io.P15;   // port 4 analog
    default: return nullptr;
  }
}

void dispatch(Devices::DeviceBus& bus, char* line, ReplyBuilder& reply) {
  char* tok = strtok(line, " ");
  if (tok == nullptr) {
    reply.raw("ERR empty");
    return;
  }

  if (strcmp(tok, "ODIAG") == 0) {
    // OTOS I2C diagnosis (101-001): report the OTOS leaf's detect state and
    // the I2CBus transaction stats for address 0x17 -- reads counters only, no
    // new bus traffic (safe against the fiber). txn=0 => the begin() probe
    // never issued a transaction; err>0/lasterr!=0 => the OTOS NAK'd (not on
    // the bus / not powered); conn=0 with err=0 => it read but returned the
    // wrong product id.
    Devices::DeviceBus::OtosProbeDiag d = bus.otosProbeDiag();
    reply.raw("OK");
    reply.kvInt("conn", d.connected ? 1 : 0);
    reply.kvInt("present", d.present ? 1 : 0);
    reply.kvInt("txn", static_cast<int>(d.txnCount));
    reply.kvInt("err", static_cast<int>(d.errCount));
    reply.kvInt("lasterr", d.lastErr);
    reply.kvInt("id", d.lastProbeId);
    return;
  }

  if (strcmp(tok, "SERVO") == 0) {
    // SERVO <pin> <angle> -- drive setServoValue(angle) on the selected edge
    // pin (servoPin() above). angle [deg] 0..180 (a 360 continuous-rotation
    // servo takes 90 = stop, <90 / >90 = rotate each way). Used to command
    // the OTOS servo and to hunt for its pin (sweep <pin>, watch ODO heading).
    char* pinTok = strtok(nullptr, " ");
    char* angTok = strtok(nullptr, " ");
    if (pinTok == nullptr || angTok == nullptr) {
      reply.raw("ERR args");
      return;
    }
    int sel = atoi(pinTok);
    int ang = atoi(angTok);
    if (ang < 0) ang = 0;
    if (ang > 180) ang = 180;
    MicroBitPin* pin = servoPin(sel);
    if (pin == nullptr) {
      reply.raw("ERR badpin");
      return;
    }
    pin->setServoValue(ang);
    reply.raw("OK");
    reply.kvInt("pin", sel);
    reply.kvInt("angle", ang);
    return;
  }

  if (strcmp(tok, "PING") == 0) {
    reply.raw("OK pong");
    return;
  }

  if (strcmp(tok, "RUNNING") == 0) {
    reply.raw("OK");
    reply.kvInt("running", bus.running() ? 1 : 0);
    return;
  }

  if (strcmp(tok, "STOP") == 0) {
    // Lightweight safety stop: neutralize both wheels through the ordinary
    // staged-setter path (armored write next cycle top) WITHOUT tearing
    // down the fiber -- bus.stop() is reserved for a real shutdown (it
    // joins and permanently exits the cycle loop; this bring-up image never
    // calls it on its own).
    bus.motor(1).setNeutral(Devices::Neutral::Coast);
    bus.motor(2).setNeutral(Devices::Neutral::Coast);
    reply.raw("OK");
    return;
  }

  if (strcmp(tok, "M") == 0) {
    char* portTok = strtok(nullptr, " ");
    int port = portTok ? atoi(portTok) : 0;
    if (port != 1 && port != 2) {
      reply.raw("ERR badport");
      return;
    }
    Devices::Motor& m = bus.motor(static_cast<uint8_t>(port));

    char* verb = strtok(nullptr, " ");
    if (verb == nullptr) {
      reply.raw("ERR noverb");
      return;
    }

    if (strcmp(verb, "VEL") == 0) {
      char* v = strtok(nullptr, " ");
      if (!v) {
        reply.raw("ERR noval");
        return;
      }
      m.setVelocity(strtof(v, nullptr));
      reply.raw("OK");
    } else if (strcmp(verb, "DUTY") == 0) {
      char* v = strtok(nullptr, " ");
      if (!v) {
        reply.raw("ERR noval");
        return;
      }
      m.setDuty(strtof(v, nullptr));
      reply.raw("OK");
    } else if (strcmp(verb, "PID") == 0) {
      char* v = strtok(nullptr, " ");
      if (!v) {
        reply.raw("ERR noval");
        return;
      }
      m.setPidEnabled(atoi(v) != 0);
      reply.raw("OK");
    } else if (strcmp(verb, "NEUTRAL") == 0) {
      char* v = strtok(nullptr, " ");
      Devices::Neutral mode = Devices::Neutral::Coast;
      if (v != nullptr && (v[0] == 'B' || v[0] == 'b')) mode = Devices::Neutral::Brake;
      m.setNeutral(mode);
      reply.raw("OK");
    } else if (strcmp(verb, "RESET") == 0) {
      m.resetPosition();
      reply.raw("OK");
    } else if (strcmp(verb, "STATE") == 0) {
      Devices::Sample<Devices::MotorReading> s = m.latest();
      reply.raw("OK");
      reply.kvFloat("pos", s.value.position);
      reply.kvFloat("vel", s.value.velocity);
      reply.kvFloat("applied", s.value.appliedDuty, 3);
      reply.kvU64("t", s.stamp);
      reply.kvInt("valid", s.valid ? 1 : 0);
      reply.kvInt("conn", m.connected() ? 1 : 0);
      reply.kvInt("wedged", m.wedged() ? 1 : 0);
      reply.kvInt("suspect", m.wedgeSuspect() ? 1 : 0);
      // glitch= (encGlitchCount(), a cumulative, never-reset counter) is the
      // bench script's own signal for two of the issue's "Bench gates":
      // gate 1 (dual-per-motorId encoder-request pipelining probe -- a
      // corrupted/mispaired pipelined request would show up as rejected-
      // sample outlier growth here) and gate 3 (reversal-stress armor
      // re-verification -- wedged=/suspect= above cover the latch signal,
      // glitch= covers the softer "rejected samples crept up" signal).
      reply.kvU32("glitch", m.encGlitchCount());
    } else if (strcmp(verb, "RING") == 0) {
      char* ageTok = strtok(nullptr, " ");
      int age = ageTok ? atoi(ageTok) : -1;
      if (age < 0 || age > 4) {
        reply.raw("ERR badage");
        return;
      }
      dispatchRing(m.sample(static_cast<uint8_t>(age)), age, reply);
    } else {
      reply.raw("ERR unknownverb");
    }
    return;
  }

  if (strcmp(tok, "COLOR") == 0) {
    char* sub = strtok(nullptr, " ");
    Devices::ColorSensor& c = bus.color();
    if (sub != nullptr && strcmp(sub, "RING") == 0) {
      char* ageTok = strtok(nullptr, " ");
      int age = ageTok ? atoi(ageTok) : -1;
      if (age < 0 || age > 4) {
        reply.raw("ERR badage");
        return;
      }
      Devices::Sample<Devices::ColorReading> s = c.sample(static_cast<uint8_t>(age));
      reply.raw("OK");
      reply.kvInt("age", age);
      reply.kvU32("r", s.value.r);
      reply.kvU32("g", s.value.g);
      reply.kvU32("b", s.value.b);
      reply.kvU32("c", s.value.c);
      reply.kvU64("t", s.stamp);
      reply.kvInt("valid", s.valid ? 1 : 0);
    } else {
      Devices::Sample<Devices::ColorReading> s = c.latest();
      reply.raw("OK");
      reply.kvU32("r", s.value.r);
      reply.kvU32("g", s.value.g);
      reply.kvU32("b", s.value.b);
      reply.kvU32("c", s.value.c);
      reply.kvU64("t", s.stamp);
      reply.kvInt("conn", c.connected() ? 1 : 0);
    }
    return;
  }

  if (strcmp(tok, "LINE") == 0) {
    char* sub = strtok(nullptr, " ");
    Devices::LineSensor& l = bus.line();
    if (sub != nullptr && strcmp(sub, "RING") == 0) {
      char* ageTok = strtok(nullptr, " ");
      int age = ageTok ? atoi(ageTok) : -1;
      if (age < 0 || age > 4) {
        reply.raw("ERR badage");
        return;
      }
      Devices::Sample<Devices::LineReading> s = l.sample(static_cast<uint8_t>(age));
      reply.raw("OK");
      reply.kvInt("age", age);
      char key[4];
      for (int i = 0; i < 4; ++i) {
        snprintf(key, sizeof(key), "r%d", i);
        reply.kvU32(key, s.value.raw[i]);
      }
      for (int i = 0; i < 4; ++i) {
        snprintf(key, sizeof(key), "n%d", i);
        reply.kvU32(key, s.value.normalized[i]);
      }
      reply.kvU64("t", s.stamp);
      reply.kvInt("valid", s.valid ? 1 : 0);
    } else {
      Devices::Sample<Devices::LineReading> s = l.latest();
      reply.raw("OK");
      char key[4];
      for (int i = 0; i < 4; ++i) {
        snprintf(key, sizeof(key), "r%d", i);
        reply.kvU32(key, s.value.raw[i]);
      }
      for (int i = 0; i < 4; ++i) {
        snprintf(key, sizeof(key), "n%d", i);
        reply.kvU32(key, s.value.normalized[i]);
      }
      reply.kvU64("t", s.stamp);
      reply.kvInt("conn", l.connected() ? 1 : 0);
    }
    return;
  }

  if (strcmp(tok, "ODO") == 0) {
    char* sub = strtok(nullptr, " ");
    Devices::Odometer& o = bus.odometer();
    if (sub != nullptr && strcmp(sub, "SETPOSE") == 0) {
      char* xt = strtok(nullptr, " ");
      char* yt = strtok(nullptr, " ");
      char* ht = strtok(nullptr, " ");
      if (!xt || !yt || !ht) {
        reply.raw("ERR noval");
        return;
      }
      o.setPose(strtof(xt, nullptr), strtof(yt, nullptr), strtof(ht, nullptr));
      reply.raw("OK");
    } else if (sub != nullptr && strcmp(sub, "RING") == 0) {
      char* ageTok = strtok(nullptr, " ");
      int age = ageTok ? atoi(ageTok) : -1;
      if (age < 0 || age > 4) {
        reply.raw("ERR badage");
        return;
      }
      Devices::Sample<Devices::PoseReading> s = o.sample(static_cast<uint8_t>(age));
      reply.raw("OK");
      reply.kvInt("age", age);
      reply.kvFloat("x", s.value.x);
      reply.kvFloat("y", s.value.y);
      reply.kvFloat("h", s.value.heading, 4);
      reply.kvFloat("vx", s.value.v_x);
      reply.kvFloat("vy", s.value.v_y);
      reply.kvFloat("w", s.value.omega, 4);
      reply.kvU64("t", s.stamp);
      reply.kvInt("valid", s.valid ? 1 : 0);
    } else {
      Devices::Sample<Devices::PoseReading> s = o.latest();
      reply.raw("OK");
      reply.kvFloat("x", s.value.x);
      reply.kvFloat("y", s.value.y);
      reply.kvFloat("h", s.value.heading, 4);
      reply.kvFloat("vx", s.value.v_x);
      reply.kvFloat("vy", s.value.v_y);
      reply.kvFloat("w", s.value.omega, 4);
      reply.kvU64("t", s.stamp);
      reply.kvInt("conn", o.connected() ? 1 : 0);
    }
    return;
  }

  reply.raw("ERR unknown");
}

// stripCorrId — finds a trailing " #<digits>" suffix (matching
// SerialConnection.send()'s own `f"{message} #{corr_id}"` framing),
// NUL-terminates `line` before it, trims the trailing space left behind,
// and copies the digits (without the '#') into `corrIdOut`. `corrIdOut[0]`
// is '\0' if no suffix was found.
void stripCorrId(char* line, char* corrIdOut, size_t corrIdCap) {
  corrIdOut[0] = '\0';
  char* hash = strrchr(line, '#');
  if (hash == nullptr) return;
  if (hash != line && hash[-1] != ' ') return;  // '#' mid-token -> not a corr-id suffix
  size_t i = 0;
  for (char* p = hash + 1; *p != '\0' && i + 1 < corrIdCap; ++p) {
    corrIdOut[i++] = *p;
  }
  corrIdOut[i] = '\0';
  *hash = '\0';
  size_t len = strlen(line);
  while (len > 0 && line[len - 1] == ' ') line[--len] = '\0';
}

}  // namespace

// ---------------------------------------------------------------------------
// bringupMain — construct the ONE DeviceBus, start its fiber, then pump the
// DEV command line reader forever. Mirrors source/main.cpp's own
// hardware_main()/main() split (uBit.init() first, function-local `static`
// long-lived objects, a bare foreground for(;;) with a uBit.sleep(1) yield
// per pass) without depending on anything that file includes beyond
// MicroBit.h.
// ---------------------------------------------------------------------------
int bringupMain() {
  uBit.init();

  // Port 1 = left (fwdSign +1), port 2 = right (fwdSign -1) -- matches
  // data/robots/tovez.json's calibration.fwd_sign_left/right and
  // source/config/boot_config.cpp's defaultMotorConfigs() port1/port2
  // mapping (see buildMotorConfig()'s own header comment for why these
  // numbers are duplicated here instead of included).
  Devices::MotorConfig motor1Config = buildMotorConfig(1, /*fwdSign=*/1, /*travelCalib=*/0.7165f);
  Devices::MotorConfig motor2Config = buildMotorConfig(2, /*fwdSign=*/-1, /*travelCalib=*/0.7077f);
  Devices::OtosConfig otosConfig = buildOtosConfig();
  Devices::ColorConfig colorConfig{};  // zero-valued -- ColorSensorLeaf auto-defaults (see above)
  Devices::LineConfig lineConfig{};    // zero-valued -- LineSensorLeaf auto-defaults (see above)

  static Devices::DeviceBus bus(uBit.i2c, motor1Config, motor2Config, otosConfig, colorConfig,
                                 lineConfig);
  bus.start();  // spawns the real CodalFiberRunner fiber (DB-008) -- returns immediately

  static BringupSerial serial(uBit.serial);
  serial.begin();

  char line[256];
  char corrId[16];
  char replyBuf[220];

  for (;;) {
    if (serial.readLine(line, sizeof(line))) {
      stripCorrId(line, corrId, sizeof(corrId));

      ReplyBuilder reply(replyBuf, sizeof(replyBuf));
      dispatch(bus, line, reply);

      if (corrId[0] != '\0') {
        reply.raw(" #");
        reply.raw(corrId);
      }
      serial.send(replyBuf);
    }

    uBit.sleep(1);  // yield: lets the DeviceBus fiber (and radio/other
                     // fibers, if any) run -- same per-pass discipline
                     // source/main.cpp's own foreground loop uses.
  }

  return 0;
}

int main() { bringupMain(); }
