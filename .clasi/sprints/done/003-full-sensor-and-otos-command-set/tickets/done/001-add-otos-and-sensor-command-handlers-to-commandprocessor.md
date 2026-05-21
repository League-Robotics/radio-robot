---
id: '001'
title: Add OTOS and sensor command handlers to CommandProcessor
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Add OTOS and sensor command handlers to CommandProcessor

## Description

After Sprint 2, the firmware drives and reports dead-reckoning odometry, but approximately
15 commands remain unimplemented: all OTOS commands (O, OI, OK, OZ, OR, OP, OV, OL, OA),
line sensor (LS), color sensor (CS), gripper (G 1-arg form), and port I/O (P, PA).

This ticket adds all of those command handlers to `source/app/CommandProcessor.cpp` and adds
the `_currentGripperAngle` private member to `source/app/CommandProcessor.h`. The HAL
drivers (`OtosSensor`, `LineSensor`, `ColorSensor`, `GripperServo`, `PortIO`) are already
written and injected as nullable pointers via `init()` from Sprint 1. This ticket only plugs
in the missing command dispatch cases and streaming output.

No new files or classes. All changes are confined to `CommandProcessor.h` and
`CommandProcessor.cpp`.

## HAL Driver Interface Reference

These are the actual C++ method signatures. Do not invent methods; call only what is
declared in the headers.

**OtosSensor** (`source/hal/OtosSensor.h`):
- `bool begin()` — probe I2C; returns false if not connected
- `void init()` — enable all signal processing (0x0F) and resetTracking
- `void calibrateImu(uint8_t samples)` — write N to IMU_CALIBRATION (async)
- `void resetTracking()` — write 0x01 to REG_RESET (resets Kalman filters, not position)
- `void getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const`
- `void setPositionRaw(int16_t x, int16_t y, int16_t h)`
- `void getVelocityRaw(int16_t& x, int16_t& y, int16_t& h) const`
- `int8_t getLinearScalar() const` / `void setLinearScalar(int8_t val)`
- `int8_t getAngularScalar() const` / `void setAngularScalar(int8_t val)`

**LineSensor** (`source/hal/LineSensor.h`):
- `bool readValues(uint16_t out[4]) const` — fills out[0..3], 0=white, 255=black approx

**ColorSensor** (`source/hal/ColorSensor.h`):
- `bool readRGBC(uint16_t& r, uint16_t& g, uint16_t& b, uint16_t& c)` — 16-bit raw counts

**GripperServo** (`source/hal/GripperServo.h`):
- `void setAngle(uint8_t degrees)` — clamps to 0..180 internally

**PortIO** (`source/hal/PortIO.h`):
- `void setDigital(uint8_t port, bool high)` — port 1..4
- `int  readDigital(uint8_t port) const` — returns 0 or 1, or -1 for invalid port
- `int  readAnalog(uint8_t port) const` — returns 0..1023, or -1 for invalid port

## OTOS LSB Conversion Constants

From the OTOS register map (confirmed in `otos.ts`):
- Position X/Y: 1 raw LSB = 0.305 mm → `x_mm = (int32_t)(raw_x * 0.305f)`
- Heading: 1 raw LSB = 0.00549° = 0.549 centidegrees → `h_cdeg = (int32_t)(raw_h * 0.549f)`
- Velocity X/Y: 1 raw LSB ≈ 0.153 mm/s → `vx_mms = (int32_t)(raw_vx * 0.153f)`
- Velocity H: 1 raw LSB ≈ 0.061 °/s → `vh_cdps = (int32_t)(raw_vh * 6.1f)` (centideg/s)

Inverse (for OV set-position): `x_raw = (int16_t)(x_mm / 0.305f)`, `h_raw = (int16_t)(h_cdeg / 0.549f)`

## Files to Modify

### `source/app/CommandProcessor.h`

Add one private member after `_prevOdoEncR`:

```cpp
int32_t   _currentGripperAngle;  // last angle sent to gripper (degrees, 0..180)
```

### `source/app/CommandProcessor.cpp`

Add includes after existing includes:

```cpp
#include "OtosSensor.h"
#include "LineSensor.h"
#include "ColorSensor.h"
#include "GripperServo.h"
#include "PortIO.h"
```

Add `_currentGripperAngle(0)` to the constructor initializer list.

Add command handlers in `process()` before the final ERR fallthrough, after the `K` block.
Add CS and LS streaming output in `tick()`.

## Command Handlers to Implement

### Null-guard pattern

Every handler checks its peripheral pointer first. Example for _otos:

```cpp
if (!_otos) { char e[32]; snprintf(e, sizeof(e), "ERR:%s", buf); replyFn(e, ctx); return; }
```

### Dispatch order (critical — follow exactly to avoid prefix collisions)

Place handlers in this order. PA must precede P. O-subcommands must precede bare O.

1. `OI` — len==2 && memcmp(buf,"OI",2)==0
2. `OK` — len>=2 && buf[0]=='O' && buf[1]=='K'
3. `OZ` — len==2 && memcmp(buf,"OZ",2)==0
4. `OR` — len==2 && memcmp(buf,"OR",2)==0
5. `OP` — len==2 && memcmp(buf,"OP",2)==0
6. `OV` — len>2  && buf[0]=='O' && buf[1]=='V'
7. `OL` — len>=2 && buf[0]=='O' && buf[1]=='L'
8. `OA` — len>=2 && buf[0]=='O' && buf[1]=='A'
9. `O`  — len==1 && buf[0]=='O'   (LAST among O-family)
10. `LS` — len==2 && memcmp(buf,"LS",2)==0
11. `CS` — len==2 && memcmp(buf,"CS",2)==0
12. `G`  — buf[0]=='G' && (len==1 || buf[1]=='+' || buf[1]=='-')
13. `PA` — len>=3 && buf[0]=='P' && buf[1]=='A' && (buf[2]=='+'||buf[2]=='-')   (BEFORE P)
14. `P`  — buf[0]=='P' && len>1  && (buf[1]=='+'||buf[1]=='-')

### O — OTOS init + calibrate shortcut

```
Condition: len==1 && buf[0]=='O'
Action:    _otos->begin(); _otos->init(); _otos->calibrateImu(255);
Reply:     "ACK:O"
Null:      "ERR:O"
```

### OI — OTOS init only

```
Condition: len==2 && memcmp(buf,"OI",2)==0
Action:    _otos->begin(); _otos->init();
Reply:     "ACK:OI"
Null:      "ERR:OI"
```

### OK — calibrate IMU

```
Condition: buf[0]=='O' && buf[1]=='K'
Action:    parse optional arg from buf+2; default samples=255; clamp 1..255
           _otos->calibrateImu((uint8_t)samples);
Reply:     "ACK:OK"
Null:      "ERR:OK"
```

### OZ — reset tracking

```
Condition: len==2 && memcmp(buf,"OZ",2)==0
Action:    _otos->resetTracking();
Reply:     "ACK:OZ"
Null:      "ERR:OZ"
```

### OR — get velocity

```
Condition: len==2 && memcmp(buf,"OR",2)==0
Action:    int16_t vx,vy,vh; _otos->getVelocityRaw(vx,vy,vh);
           int32_t vx_mms = (int32_t)(vx * 0.153f);
           int32_t vy_mms = (int32_t)(vy * 0.153f);
           int32_t vh_cdps = (int32_t)(vh * 6.1f);
           clamp vx_mms/vy_mms to -9999..9999; vh_cdps to -99999..99999
Reply:     snprintf(r, sizeof(r), "OR%+d%+d%+d", vx_mms, vy_mms, vh_cdps)
Null:      "ERR:OR"
```

### OP — get position

```
Condition: len==2 && memcmp(buf,"OP",2)==0
Action:    int16_t x,y,h; _otos->getPositionRaw(x,y,h);
           int32_t x_mm   = (int32_t)(x * 0.305f);
           int32_t y_mm   = (int32_t)(y * 0.305f);
           int32_t h_cdeg = (int32_t)(h * 0.549f);
           clamp x_mm/y_mm to -9999..9999; h_cdeg to -18000..18000
Reply:     snprintf(r, sizeof(r), "OP%+d%+d%+d", x_mm, y_mm, h_cdeg)
Null:      "ERR:OP"
```

### OV — set position

```
Condition: len>2 && buf[0]=='O' && buf[1]=='V'
Action:    parse 3 args from buf+2; if n<3 → ERR
           int16_t xr = (int16_t)(args[0] / 0.305f);
           int16_t yr = (int16_t)(args[1] / 0.305f);
           int16_t hr = (int16_t)(args[2] / 0.549f);
           _otos->setPositionRaw(xr, yr, hr);
Reply:     "ACK:OV"
Null:      "ERR:OV"
```

### OL — linear scalar get/set

```
Condition: len>=2 && buf[0]=='O' && buf[1]=='L'
Query (len==2):  snprintf(r, sizeof(r), "OL%+d", (int)_otos->getLinearScalar()); reply
Set (len>2):     parse 1 arg; clamp -128..127; _otos->setLinearScalar((int8_t)v);
                 snprintf(r, sizeof(r), "ACK:OL %d", v); reply
Null:            snprintf(e, sizeof(e), "ERR:%s", buf); reply
```

### OA — angular scalar get/set

Same pattern as OL using `getAngularScalar()` / `setAngularScalar()`.
Query reply format: `"OA%+d"`. Set reply: `"ACK:OA %d"`.

### LS — line sensor

```
Condition: len==2 && memcmp(buf,"LS",2)==0
Action:    uint16_t out[4]={0,0,0,0}; _line->readValues(out);
Reply:     snprintf(r, sizeof(r), "LS%+d%+d%+d%+d", out[0], out[1], out[2], out[3])
Null:      "ERR:LS"
```

### CS — color sensor

```
Condition: len==2 && memcmp(buf,"CS",2)==0
Action:    uint16_t r,g,b,c; _color->readRGBC(r,g,b,c);
Reply:     snprintf(rbuf, sizeof(rbuf), "CS%+d%+d%+d%+d", (int)r,(int)g,(int)b,(int)c)
Null:      "ERR:CS"
```

### G — gripper (1-arg form; sprint 5 replaces with 3-arg go-to)

```
Condition: buf[0]=='G' && (len==1 || buf[1]=='+' || buf[1]=='-')
  No-arg (len==1):
    Null: "ERR:G"
    Reply: snprintf(r, sizeof(r), "G%+d", (int)_currentGripperAngle)
  1-arg (parse from buf+1):
    If n<1: "ERR:G"
    Null: "ERR:G"
    deg = clampInt(args[0], 0, 180)
    _gripper->setAngle((uint8_t)deg); _currentGripperAngle = deg;
    snprintf(r, sizeof(r), "ACK:G %d", deg)
```

If Sprint 2 left any existing G handler, replace it entirely with this implementation.

### PA — analog port read

```
Condition: len>=3 && buf[0]=='P' && buf[1]=='A' && (buf[2]=='+'||buf[2]=='-')
Action:    parse 1 arg from buf+2; if n<1 → ERR
           int val = _portio->readAnalog((uint8_t)args[0]);
Reply:     snprintf(r, sizeof(r), "PA%+d%+d", (int)args[0], val)
Null:      snprintf(e, sizeof(e), "ERR:%s", buf); reply
```

### P — digital port I/O

```
Condition: buf[0]=='P' && len>1 && (buf[1]=='+'||buf[1]=='-')
Action:    parse up to 2 args from buf+1; if n<1 → ERR
  2 args (set):
    _portio->setDigital((uint8_t)args[0], args[1] != 0);
    snprintf(r, sizeof(r), "ACK:P %d %d", (int)args[0], args[1]!=0 ? 1 : 0)
  1 arg (read):
    int val = _portio->readDigital((uint8_t)args[0]);
    snprintf(r, sizeof(r), "P%+d%+d", (int)args[0], val)
Null:      snprintf(e, sizeof(e), "ERR:%s", buf); reply
```

## Streaming in tick()

In `CommandProcessor::tick()`, inside the `_encTickCount >= params.encReportEvery` block
(after the existing `reportEncoders` and `reportOdo` calls), add:

```cpp
if (_color) {
    uint16_t sr, sg, sb, sc;
    _color->readRGBC(sr, sg, sb, sc);
    char sbuf[48];
    snprintf(sbuf, sizeof(sbuf), "CS%+d%+d%+d%+d",
             (int)sr, (int)sg, (int)sb, (int)sc);
    replyFn(sbuf, ctx);
}
if (_line) {
    uint16_t lo[4] = {0, 0, 0, 0};
    _line->readValues(lo);
    char sbuf[48];
    snprintf(sbuf, sizeof(sbuf), "LS%+d%+d%+d%+d",
             (int)lo[0], (int)lo[1], (int)lo[2], (int)lo[3]);
    replyFn(sbuf, ctx);
}
```

These run only when the sensor pointer is non-null and unconditionally within the
encReportEvery cadence — there are no separate streamColor/streamLine flags in the C++
port; the null pointer serves as the enable/disable gate.

## Acceptance Criteria

- [x] `_currentGripperAngle` declared in `CommandProcessor.h` private section, initialized to 0
- [x] Five HAL includes added to `CommandProcessor.cpp`
- [x] `O` → begin()+init()+calibrateImu(255) → "ACK:O"; null → "ERR:O"
- [x] `OI` → begin()+init() → "ACK:OI"; null → "ERR:OI"
- [x] `OK` → calibrateImu(255 default or parsed arg clamped 1..255) → "ACK:OK"; null guard
- [x] `OZ` → resetTracking() → "ACK:OZ"; null guard
- [x] `OR` → velocity with 0.153f/6.1f conversion → "OR+vx+vy+vh"; null guard
- [x] `OP` → position with 0.305f/0.549f conversion → "OP+x+y+h"; null guard
- [x] `OV+x+y+h` → inverse conversion → setPositionRaw → "ACK:OV"; null guard
- [x] `OL` (no arg) → "OL+<scalar>"; `OL+<n>` → set → "ACK:OL <n>"; null guard
- [x] `OA` (no arg) → "OA+<scalar>"; `OA+<n>` → set → "ACK:OA <n>"; null guard
- [x] `LS` → readValues → "LS+v0+v1+v2+v3"; null → "ERR:LS"
- [x] `CS` → readRGBC → "CS+r+g+b+c"; null → "ERR:CS"
- [x] `G+<deg>` → setAngle, store _currentGripperAngle → "ACK:G <deg>"; null guard
- [x] `G` (no arg) → "G+<_currentGripperAngle>"; null guard
- [x] `P+<port>+<val>` → setDigital → "ACK:P <port> <val>"; null guard
- [x] `P+<port>` (1 arg) → readDigital → "P+<port>+<val>"; null guard
- [x] `PA+<port>` → readAnalog → "PA+<port>+<val>"; null guard
- [x] tick() streaming emits CS and LS when respective pointers are non-null
- [x] PA handler precedes P handler in dispatch sequence
- [x] O-subcommands precede bare O in dispatch sequence
- [x] Existing commands (S, T, D, ENC, EZ, SO, SZ, SI, K, X) unchanged — no regressions
- [ ] `python build.py` succeeds with no errors

## Implementation Notes

- All snprintf buffer sizes: use at least 48 bytes for sensor replies (4 values × up to
  6 chars each + prefix = ~30 chars; 48 is safe).
- The `%+d` format inserts a leading `+` for non-negative integers, matching the
  sign-prefixed protocol used throughout the firmware.
- Do not add space between command prefix and values in sign-prefixed replies (e.g.
  `"OP%+d%+d%+d"` not `"OP %+d %+d %+d"`). Check existing handlers like `reportOdo()`
  and `reportEncoders()` for the exact format convention.
- For `ACK:` replies where a value follows with a space (e.g. `"ACK:G <deg>"`), use a
  space separator as shown. This matches the existing pattern in `ACK:S`, `ACK:T`, etc.
