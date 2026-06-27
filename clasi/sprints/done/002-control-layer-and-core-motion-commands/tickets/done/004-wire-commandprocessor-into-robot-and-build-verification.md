---
id: '004'
title: Wire CommandProcessor into Robot and build verification
status: done
use-cases: []
depends-on:
- '003'
github-issue: ''
issue: plan-c-port-of-radio-robot-firmware
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Wire CommandProcessor into Robot and build verification

## Description

Modify `source/app/Robot.h` and `source/app/Robot.cpp` to add `MotorController`,
`Odometry`, and `CommandProcessor` members, call `init()` in the constructor, and
route all incoming commands through `CommandProcessor::process()` and
`CommandProcessor::tick()` in the `run()` loop.

This ticket also performs the full build-and-deploy verification for Sprint 2.

The `run()` loop currently has placeholder comments from Sprint 1 (`// sprint 2:
_cmd.dispatch(...)`) — replace those with the real calls.

## Changes to `source/app/Robot.h`

Add includes and new member declarations:

```cpp
#include "MotorController.h"
#include "Odometry.h"
#include "CommandProcessor.h"
```

Add three new private members after `_cal`:

```cpp
MotorController  _mc;
Odometry         _odo;
CommandProcessor _cmd;
```

Declaration order: `_mc` before `_odo` before `_cmd`, because `_cmd` is initialized
last and depends on the others being fully constructed.

## Changes to `source/app/Robot.cpp`

### Constructor member initializer list

Add `_mc` and `_odo` to the initializer list after `_cal`:

```cpp
_mc(_motor, _cal),
_odo(),
_cmd(),
```

`CommandProcessor` has a default constructor; its `init()` is called in the body.

### Constructor body

After `_radio.begin()` and the sensor probing, call:

```cpp
_cmd.init(
    &_motor,
    &_mc,
    &_odo,
    _otosPresent  ? &_otos   : nullptr,
    _linePresent  ? &_line   : nullptr,
    _colorPresent ? &_color  : nullptr,
    _gripperPresent ? &_gripper : nullptr,
    &_portio
);
```

### `run()` loop

Replace the current placeholder loop with:

```cpp
void Robot::run() {
    bool isRelayed;
    while (true) {
        // Process incoming serial lines
        while (_serial.readLine(_buf, sizeof(_buf))) {
            bool handled = _announcer.handle(_buf);
            if (!handled) {
                _cmd.process(_buf, serialReply, &_serial);
            }
        }
        // Process incoming radio packets
        while (_radio.poll(_buf, sizeof(_buf), isRelayed)) {
            bool handled = _announcer.handle(_buf);
            if (!handled) {
                _cmd.process(_buf, radioReply, &_radio);
            }
        }
        // Drive-mode tick
        _cmd.tick(uBit.systemTime(), serialReply, &_serial);
        // No explicit sleep — tick() self-throttles to tickMs cadence
    }
}
```

### Reply function stubs

Define two static free functions (or lambdas) before `Robot::run()` to adapt the
`ReplyFn` signature to `SerialPort` and `Radio`:

```cpp
static void serialReply(const char* msg, void* ctx) {
    reinterpret_cast<SerialPort*>(ctx)->writeLine(msg);
}

static void radioReply(const char* msg, void* ctx) {
    reinterpret_cast<Radio*>(ctx)->send(msg);
}
```

Check that `SerialPort` has `writeLine(const char*)` and `Radio` has `send(const char*)`;
adjust method names to match their actual APIs if different.

### Tick reply routing

The `_cmd.tick()` call above passes `serialReply` and `&_serial`. This means streaming
encoder output and watchdog messages go to the serial port by default. This is correct
for direct-cable sessions. Radio relay responses will be added in a later sprint if
needed.

## Build Verification

After making all changes:

1. Run `python build.py` from the project root.
2. Fix any compile errors that arise (type mismatches, missing includes, API name
   mismatches between the ticket spec and actual class interfaces). The ticket body
   describes intent; actual method names on SerialPort/Radio may differ — read their
   headers and use the real names.
3. Once the build is clean, deploy with:
   ```
   python scripts/deploy.py --usb-mount "/Volumes/MICROBIT 1"
   ```
   IMPORTANT: always include `--usb-mount "/Volumes/MICROBIT 1"` to target the robot's
   micro:bit. Omitting it risks overwriting the radio relay on `/Volumes/MICROBIT`.

## Hardware-in-the-Loop Tests

After deploy, connect a serial terminal at 115200 baud and verify:

1. **Motor forward**: send `S+100+100` — both wheels spin forward.
2. **Stop**: send `X` — motors stop, reply `ACK:X`.
3. **Differential**: send `S+100-100` — robot spins in place.
4. **Timed drive**: send `T+200+200+500` — drives ~0.5 s, then `ACK:T+DONE`.
5. **Encoder format**: send `ENC` — reply matches `ENC+NNNN-MMMM` with mandatory signs.
6. **Encoder zero**: send `EZ` then `ENC` — reply is `ENC+0+0` (or `ENC+0000+0000`).
7. **Odometry**: drive forward, send `SO` — reply `SO+XXXX+YYYY+HHHH` with non-zero x.
8. **Watchdog**: send `S+100+100`, wait 300 ms without refresh — `LOG:SAFETY_STOP` appears.
9. **Calibration dump**: send `K` — 13 lines, each starting with `K:`.
10. **Streaming ENC**: while mode is STREAMING, ENC lines appear every `encReportEvery` ticks.

## Files to Modify

- `source/app/Robot.h`
- `source/app/Robot.cpp`

## Files to Read First

- `source/app/Robot.h` — current state before edits
- `source/app/Robot.cpp` — current state, note existing sprint-1 placeholder comments
- `source/app/CommandProcessor.h` — exact `init()` and `tick()` signatures (from ticket 003)
- `source/hal/SerialPort.h` — find the actual `writeLine()` method name
- `source/hal/Radio.h` — find the actual `send()` method name
- `source/control/MotorController.h` — constructor signature: `MotorController(NezhaV2&, const CalibParams&)`
- `source/control/Odometry.h` — default constructor

## Acceptance Criteria

- [x] `Robot.h` includes `MotorController.h`, `Odometry.h`, `CommandProcessor.h`
- [x] `Robot.h` declares `_mc`, `_odo`, `_cmd` as private members in that order
- [x] `Robot.cpp` initializer list constructs `_mc(_motor, _cal)` and `_odo()`
- [x] `Robot.cpp` constructor body calls `_cmd.init(...)` with correct pointers
- [x] `run()` loop calls `_cmd.process()` for every serial and radio line not handled by Announcer
- [x] `run()` loop calls `_cmd.tick(uBit.systemTime(), ...)` on every iteration
- [x] `python build.py` succeeds with zero errors
- [ ] `S+100+100` over serial drives both motors forward on hardware
- [ ] `X` stops motors on hardware
- [ ] `LOG:SAFETY_STOP` appears after 300 ms without S refresh
- [ ] `ENC` returns correctly formatted response
- [ ] `K` returns 13 calibration lines

## Testing

- **Build verification**: `python build.py`
- **Deploy command**: `python scripts/deploy.py --usb-mount "/Volumes/MICROBIT 1"`
- **Serial terminal**: 115200 baud, any terminal app (screen, minicom, etc.)
- **Verification command**: `python build.py && python scripts/deploy.py --usb-mount "/Volumes/MICROBIT 1"`
