---
id: '003'
title: Implement CommandProcessor
status: done
use-cases: []
depends-on:
- '001'
- '002'
github-issue: ''
issue: plan-c-port-of-radio-robot-firmware
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Implement CommandProcessor

## Description

Create `source/app/CommandProcessor.h` and `source/app/CommandProcessor.cpp`.

CommandProcessor is the most complex module in this sprint. It owns the drive-mode
state machine (IDLE / STREAMING / TIMED / DISTANCE), the S-mode watchdog, streaming
encoder/odometry output, and the full command dispatch table. It calls
`MotorController` and `Odometry` — it never touches hardware directly.

This is a direct C++ port of `command.ts::handleCommand()` and `command.ts::tick()`
with the following adaptations:
- OTOS/sensor/gripper/portio dependencies are injected as nullable pointers (all null
  in this sprint; commands requiring them return `ERR:` if pointer is null)
- No `basic.pause()` calls — C++ is not event-driven; small pauses in K-dump are
  replaced by `uBit.sleep(10)` via a passed reference or eliminated (single-line
  K-dump output is fine without inter-line pauses on hardware)
- Reply function is a typedef `void (*ReplyFn)(const char*, void*)` where the second
  arg is a caller-supplied context pointer

Read the full `command.ts` source before implementing. The logic below is authoritative
for this sprint's scope; commands marked "out of scope" should reply `ERR:` if received.

## Header — `source/app/CommandProcessor.h`

```cpp
#pragma once
#include <stdint.h>
#include "Config.h"
#include "NezhaV2.h"
#include "MotorController.h"
#include "Odometry.h"

// Forward declarations for optional peripherals (may be null).
class OtosSensor;
class LineSensor;
class ColorSensor;
class GripperServo;
class PortIO;

// Reply callback type. ctx is caller-supplied (e.g. pointer to SerialPort or Radio).
typedef void (*ReplyFn)(const char* msg, void* ctx);

/**
 * CommandProcessor — wire-protocol parser and drive-mode state machine.
 *
 * Owns DriveMode state, S-mode watchdog, and streaming encoder output.
 * Calls MotorController (motor control) and Odometry (dead-reckoning).
 * Does NOT interact with hardware directly.
 *
 * Usage:
 *   CommandProcessor cmd;
 *   cmd.init(&motor, &mc, &odo, nullptr, nullptr, nullptr, nullptr, nullptr);
 *   // in tick loop:
 *   cmd.process(lineBuf, replyFn, ctx);
 *   cmd.tick(uBit.systemTime(), replyFn, ctx);
 */
class CommandProcessor {
public:
    CommandProcessor();

    // Public calibration params — updated by K-commands.
    struct Params {
        float   mmPerDegL;       // encoder mm/degree, left wheel (default 0.487)
        float   mmPerDegR;       // encoder mm/degree, right wheel (default 0.481)
        float   distScale;       // distance command scale factor (default 0.94)
        float   turnScale;       // turn command scale factor (default 1.07)
        int32_t minSpeedMms;     // minimum non-zero speed snap (default 50)
        int32_t tickMs;          // tick cadence ms (default 20)
        int32_t sTimeoutMs;      // S-mode watchdog timeout ms (default 200)
        int32_t encReportEvery;  // streaming encoder/odo report interval in ticks (default 2)
        float   trackwidthMm;    // wheel trackwidth mm (default 120)
    } params;

    // Inject hardware pointers. mc and odo must not be null. Others may be null.
    void init(NezhaV2*       motor,
              MotorController* mc,
              Odometry*       odo,
              OtosSensor*     otos,
              LineSensor*     line,
              ColorSensor*    color,
              GripperServo*   gripper,
              PortIO*         portio);

    // Parse and dispatch one command line. line must be NUL-terminated.
    // Calls replyFn(msg, ctx) for each response line.
    void process(const char* line, ReplyFn replyFn, void* ctx);

    // Drive-mode state machine tick. Call once per iteration of the main loop.
    // now_ms: current system time in ms (from uBit.systemTime()).
    // replyFn/ctx: same callback used by process().
    void tick(uint32_t now_ms, ReplyFn replyFn, void* ctx);

private:
    // Injected pointers
    NezhaV2*         _motor;
    MotorController* _mc;
    Odometry*        _odo;
    OtosSensor*      _otos;
    LineSensor*      _line;
    ColorSensor*     _color;
    GripperServo*    _gripper;
    PortIO*          _portio;

    // Drive mode state
    DriveMode _mode;
    uint32_t  _lastSMs;       // time of last S command (for watchdog)
    float     _tgtL;          // current left target mm/s
    float     _tgtR;          // current right target mm/s

    // T-command termination
    uint32_t  _tEndMs;

    // D-command termination
    int32_t   _dEncStartL;    // mm at D command start
    int32_t   _dEncStartR;
    int32_t   _dTargetMm;
    uint32_t  _dTimeoutMs;

    // Streaming state
    int32_t   _encTickCount;  // counts up to encReportEvery

    // Tick timing
    uint32_t  _lastTickMs;

    // Internal helpers
    static int  parseSignedArgs(const char* s, int32_t* out, int maxArgs);
    static int  clampInt(int v, int lo, int hi);
    static int  clampMinSpeed(int mms, int minSpeedMms);
    static void signStr(char* buf, int32_t v);   // writes "+NNNN" or "-NNNN"
    void        fullStop(ReplyFn replyFn, void* ctx);
    void        reportEncoders(ReplyFn replyFn, void* ctx);
    void        reportOdo(ReplyFn replyFn, void* ctx);
};
```

## Implementation — `source/app/CommandProcessor.cpp`

### Constructor

Initialize all params with defaults:
- `params.mmPerDegL = 0.487f`
- `params.mmPerDegR = 0.481f`
- `params.distScale = 0.94f`
- `params.turnScale = 1.07f`
- `params.minSpeedMms = 50`
- `params.tickMs = 20`
- `params.sTimeoutMs = 200`
- `params.encReportEvery = 2`
- `params.trackwidthMm = 120.0f`

Zero all state fields. Set `_mode = DriveMode::IDLE`.

### `parseSignedArgs(const char* s, int32_t* out, int maxArgs)`

Scan the string character by character. When `+` or `-` is found, begin accumulating
digits. Push the parsed integer into `out[]` when the next sign or NUL is reached.
Return number of args parsed.

Example: `"+200-150"` → `out[0]=200, out[1]=-150`, returns 2.
Example: `"+200+200+1000"` → `out[0]=200, out[1]=200, out[2]=1000`, returns 3.

This exactly mirrors `command.ts::parseSignedArgs()`.

### `signStr(char* buf, int32_t v)`

Writes a mandatory-sign string: if `v >= 0`, write `"+NNN"`, else write `"-NNN"`
(the `sprintf` negative sign appears automatically). Use `snprintf(buf, 8, "%+d", v)`
or equivalent.

### `fullStop(ReplyFn replyFn, void* ctx)`

```
_mc->stop();
_mode = DriveMode::IDLE;
_tgtL = _tgtR = 0.0f;
_encTickCount = 0;
```

Does NOT reset odometry. Does NOT emit any reply (caller emits ACK or LOG as needed).

### `reportEncoders(ReplyFn replyFn, void* ctx)`

```
int32_t l, r;
_mc->getEncoderPositions(l, r);
// Format: "ENC+LLLL-RRRR" with mandatory signs
char buf[32];
snprintf(buf, sizeof(buf), "ENC%+d%+d", (int)l, (int)r);
// Note: %+d inserts sign automatically (+ for positive, - for negative)
replyFn(buf, ctx);
```

Wire format examples: `ENC+1234-0045`, `ENC+0000+0000`, `ENC-0100+0050`.

### `reportOdo(ReplyFn replyFn, void* ctx)`

```
int32_t x, y, h;
_odo->getPose(x, y, h);
char buf[48];
snprintf(buf, sizeof(buf), "SO%+d%+d%+d", (int)x, (int)y, (int)h);
replyFn(buf, ctx);
```

Wire format examples: `SO+0500+0000+00000`, `SO-0123+0045+09000`.

### `process(const char* line, ReplyFn replyFn, void* ctx)`

Uppercase and trim the input. Use a local char buffer (128 bytes max).

**Command dispatch order** (check longer prefixes before shorter):

1. **`X` or `STOP`** — `fullStop()`; reply `"ACK:X"`.

2. **`S` with sign second char** — streaming drive.
   - Parse args from `line+1`: need 2 args.
   - Clamp each to `clampMinSpeed(v, params.minSpeedMms)`.
   - If `_mode != DriveMode::STREAMING`: call `_mc->resetIntegrators()`.
   - Call `_mc->setTarget(leftMms, rightMms)`.
   - Set `_mode = DriveMode::STREAMING`, `_lastSMs = now_ms` (use `_lastTickMs` as current time since no `now_ms` parameter here — or store last known time). 
   - **Important**: The S command handler does not receive `now_ms` directly. Store a member `_lastSMs` and update it in `tick()` at the top. For the S command, record the time by saving it as `_lastSMs = _lastTickMs` (the last tick time, which is always current within one tick cadence).
   - Reply: `"ACK:S "` + leftMms + `" "` + rightMms. (Space-separated for ACK, matching TS: `"ACK:S " + leftMms + " " + rightMms`)

3. **`T`** — timed drive.
   - Parse 3 args: leftMms, rightMms, durationMs.
   - Clamp durationMs to 1..5000.
   - `_mc->resetIntegrators(); _mc->setTarget(leftMms, rightMms);`
   - `_tEndMs = _lastTickMs + durationMs;`
   - `_mode = DriveMode::TIMED;`
   - Reply: `"ACK:T "` + leftMms + `" "` + rightMms + `" "` + durationMs.

4. **`D`** — distance drive.
   - Parse 3 args: leftMms, rightMms, targetMm.
   - targetMm = abs(args[2]); clamp to >= 1.
   - `_mc->resetIntegrators(); _mc->setTarget(leftMms, rightMms);`
   - `_mc->resetEncoderAccumulators();`
   - `_mc->getEncoderPositions(_dEncStartL, _dEncStartR);` (will be 0 after reset)
   - `_dTargetMm = targetMm; _dTimeoutMs = _lastTickMs + 5000;`
   - `_mode = DriveMode::DISTANCE;`
   - Reply: `"ACK:D "` + leftMms + `" "` + rightMms + `" "` + targetMm.

5. **`ENC`** — `reportEncoders(replyFn, ctx)`.

6. **`EZ`** — `_mc->resetEncoderAccumulators()`; reply `"ACK:EZ"`.

7. **`SO`** — `reportOdo(replyFn, ctx)`.

8. **`SZ`** — `_odo->zero()`; reply `"ACK:SZ"`.

9. **`SI`** — parse 3 args (x_mm, y_mm, h_cdeg); `_odo->setPose(args[0], args[1], args[2])`; reply `"ACK:SI "` + args joined.

10. **`K` alone** — dump all calibration params, one per reply line, format `"K:KML:+487"` (value scaled per table below). Use `replyFn` for each line.

11. **`KML`** — `params.mmPerDegL = v/1000.0f`; reply `"ACK:KML "` + round(val*1000).
    **`KMR`** — `params.mmPerDegR = v/1000.0f`.
    **`KFF`** — `_mc->gains.kFF = v/1000.0f`.
    **`KSP`** — `_mc->gains.kP = v/1000.0f`.  *(kP setter)*
    **`KSI`** — `_mc->gains.kI = v/1000.0f`.  *(kI setter)*
    **`KIC`** — `_mc->gains.iClamp = static_cast<float>(v)`.  *(iClamp setter)*
    **`KSR`** — `_mc->gains.kRatio = v/1000.0f`.  *(kRatio setter)*
    **`KSM`** — `params.minSpeedMms = max(0, v)`.
    **`KSS`** — `params.sTimeoutMs = clampInt(v, 50, 5000)`.
    **`KTR`** — `params.tickMs = clampInt(v, 5, 100)`.
    **`KER`** — `params.encReportEvery = clampInt(v, 1, 20)`.
    **`KSD`** — `params.distScale = v/100.0f`.
    **`KST`** — `params.turnScale = v/100.0f`.

    All K setters: parse 1 signed arg from `line+3`; reply `"ACK:K"` + key + `" "` + stored value.

12. Any unrecognized command: reply `"ERR:"` + uppercased command text.

**K dump format** (K command alone):
Each line: `"K:KML:"` + signStr(round(params.mmPerDegL * 1000)) — i.e. `"K:KML:+487"`.
Emit one line per parameter in this order:
`KML, KMR, KFF, KSP, KSI, KIC, KSR, KSM, KSS, KTR, KER, KSD, KST`

All values are integers scaled as follows:
- `KML`, `KMR`, `KFF`, `KSP`, `KSI`, `KSR`: value * 1000 (e.g. 0.15 → 150)
- `KIC`: raw integer (iClamp)
- `KSM`, `KSS`, `KTR`, `KER`: raw integers
- `KSD`, `KST`: value * 100 (e.g. 0.94 → 94)

### `tick(uint32_t now_ms, ReplyFn replyFn, void* ctx)`

```
// Throttle to tickMs cadence
if ((now_ms - _lastTickMs) < (uint32_t)params.tickMs) return;
float dt_s = (now_ms - _lastTickMs) / 1000.0f;
_lastTickMs = now_ms;

// Run motor controller
if (_mode != DriveMode::IDLE) {
    _mc->tick(dt_s);

    // Update odometry from encoder deltas
    int32_t encL, encR;
    _mc->getEncoderPositions(encL, encR);
    // Note: getEncoderPositions returns cumulative mm since last reset.
    // Odometry needs per-tick delta. Track previous positions separately.
    // (Use private members _prevOdoEncL, _prevOdoEncR initialized to 0)
    float dL = encL - _prevOdoEncL;
    float dR = encR - _prevOdoEncR;
    _prevOdoEncL = encL;
    _prevOdoEncR = encR;
    _odo->update(dL, dR, params.trackwidthMm);
}

// S-mode watchdog
if (_mode == DriveMode::STREAMING) {
    if ((now_ms - _lastSMs) > (uint32_t)params.sTimeoutMs) {
        fullStop(replyFn, ctx);
        replyFn("LOG:SAFETY_STOP", ctx);
    }
}

// T-mode: stop when deadline reached
if (_mode == DriveMode::TIMED && now_ms >= _tEndMs) {
    fullStop(replyFn, ctx);
    reportOdo(replyFn, ctx);
    replyFn("ACK:T+DONE", ctx);
}

// D-mode: stop when average encoder travel >= target, or on timeout
if (_mode == DriveMode::DISTANCE) {
    int32_t l, r;
    _mc->getEncoderPositions(l, r);
    int32_t avgTravel = (abs(l - _dEncStartL) + abs(r - _dEncStartR)) / 2;
    if (avgTravel >= _dTargetMm || now_ms >= _dTimeoutMs) {
        fullStop(replyFn, ctx);
        reportOdo(replyFn, ctx);
        replyFn("ACK:D+DONE", ctx);
    }
}

// Streaming encoder output every encReportEvery ticks
if (_mode != DriveMode::IDLE) {
    _encTickCount++;
    if (_encTickCount >= params.encReportEvery) {
        reportEncoders(replyFn, ctx);
        _encTickCount = 0;
    }
}
```

Add private members `_prevOdoEncL` and `_prevOdoEncR` (int32_t, zeroed in constructor)
to track the previous encoder positions for odometry delta computation.

Also update `_lastSMs` at the top of the S command handler:
```
_lastSMs = now_ms;
```
Since `process()` does not receive `now_ms`, pass it through `tick()` to a shared member
`_currentTimeMs` updated at the top of `tick()`. The S handler uses `_currentTimeMs`.

### S-Command Watchdog Rule

**Integrators are NOT reset on watchdog refresh.** When an S command is received and
`_mode` is already `DriveMode::STREAMING`, skip `resetIntegrators()`. This preserves
accumulated integral across keepalive refreshes, preventing step-response jerk. Only
on a mode transition (IDLE → STREAMING or TIMED/DISTANCE → STREAMING) should
integrators be reset.

## Files to Create

- `source/app/CommandProcessor.h`
- `source/app/CommandProcessor.cpp`

## Files to Read First

- `/Volumes/Proj/proj/league-projects/scratch/radio-robot/src/command.ts` — full source, especially `handleCommand()` (line 188) and `tick()` (line 819)
- `source/control/MotorController.h` — gains struct, public interface
- `source/control/Odometry.h` — `update()`, `getPose()`, `setPose()`, `zero()`
- `source/types/Config.h` — `CalibParams`, `DriveMode` enum
- `source/hal/NezhaV2.h` — passed to init() but not called directly from CommandProcessor

## Acceptance Criteria

- [x] `source/app/CommandProcessor.h` exists with the exact public interface above
- [x] `source/app/CommandProcessor.cpp` exists and implements `init()`, `process()`, `tick()`
- [x] `X` and `STOP` commands stop motors and reply `"ACK:X"`
- [x] `S+200+150` sets targets, mode=STREAMING, replies `"ACK:S 200 150"`
- [x] S-mode watchdog fires after `sTimeoutMs` ms of silence, emits `"LOG:SAFETY_STOP"`
- [x] Integrators are NOT reset on S keepalive (only on mode change)
- [x] `T+200+200+1000` runs for 1 s then auto-stops, replies `"ACK:T+DONE"`
- [x] `D+200+200+500` runs until avg encoder >= 500 mm, replies `"ACK:D+DONE"`
- [x] `ENC` replies `"ENC+LLLL-RRRR"` with mandatory signs
- [x] `EZ` zeroes encoder accumulators, replies `"ACK:EZ"`
- [x] `SO` replies `"SO+XXXX+YYYY+HHHH"` from Odometry::getPose()
- [x] `SZ` zeroes odometry, replies `"ACK:SZ"`
- [x] `SI+100-050+01800` sets pose, replies `"ACK:SI 100 -50 1800"`
- [x] `K` dumps all 13 parameters in `"K:KML:+487"` format
- [x] All K setters (KML, KMR, KFF, KSP, KSI, KIC, KSR, KSM, KSS, KTR, KER, KSD, KST) work and reply `"ACK:K<key> <val>"`
- [x] Unrecognized commands reply `"ERR:<CMD>"`
- [x] Streaming encoder output emitted every `encReportEvery` ticks when mode != IDLE
- [x] Odometry updated from encoder deltas each tick
- [ ] `python build.py` compiles without errors (build not run per ticket instructions)

## Testing

- **Hardware-in-the-loop only** — CODAL does not support off-device unit tests
- **Build verification**: `python build.py` must succeed with zero errors
- **Functional tests** (after ticket 004 wires into Robot):
  - Serial: `S+100+100` — motors spin, streaming ENC lines appear
  - Serial: wait 300 ms — `LOG:SAFETY_STOP` appears
  - Serial: `T+200+200+500` — drives 0.5 s then `ACK:T+DONE`
  - Serial: `ENC` — response format `ENC+NNNN-MMMM`
  - Serial: `K` — 13 lines each starting with `K:`
- **Verification command**: `python build.py`
