---
title: Protocol & Commands
blurb: The protocol v2 wire format and the most-used command verbs at a glance.
order: 30
tags: [protocol, commands, serial, radio]
---

# Protocol & Commands

The host and robot speak **protocol v2**: newline-terminated ASCII lines with
sign-prefixed integer arguments. This page is the gist; the authoritative,
exhaustive reference is
[docs/protocol-v2.md](https://github.com/League-Robotics/radio-robot/blob/master/docs/protocol-v2.md)
and the [feature specification](https://github.com/League-Robotics/radio-robot/blob/master/docs/specification.md).

## Wire format

- **Serial:** 115200 baud, bidirectional over the micro:bit's USB serial.
- **Radio:** micro:bit radio group 10. A command relayed over radio carries a
  `>` prefix; responses come back with `<`. This lets the host run wirelessly
  through a second micro:bit acting as a USB-serial bridge.
- **Numbers:** every numeric value is sign-prefixed — `+1234`, `-42`, `+0`. No
  floating point on the wire; fixed-point scaling is used where needed.
- **Responses:** `OK` / `ERR` / `EVT` / `TLM` / `CFG` / `ID`.

## Most-used commands

| Command | Format | Description |
|---|---|---|
| `S` | `S+LS+RS` | Set left/right motor speeds; runs until stopped (sends watchdog keepalives) |
| `T` | `T+LS+RS+DUR` | Drive at speeds for `DUR` ms |
| `D` | `D+LS+RS+DIST` | Drive at speeds for `DIST` mm (encoder-based, blocking) |
| `G` | `G+X+Y+Speed` | Arc-navigate to relative XY target; emits `G+DONE` |
| `X` / `STOP` | `X` | Stop immediately |
| `ENC` | `ENC` | Query encoder positions → `ENC+L+R` |
| `EZ` | `EZ` | Zero encoder counts |
| `SO` / `SZ` / `SI` | — | Query / zero / set dead-reckoning odometry |
| `OP` | `OP` | Query OTOS pose → `OP+X+Y+H` |
| `LS` / `CS` | — | Read line sensor / color sensor |
| `HELLO` | `HELLO` | Trigger device announcement (`DEVICE:+name+version`) |

### Two ways to drive

- **Discrete moves** — `D`/`T`/`G` are blocking, no watchdog; the host waits for
  the `EVT done` event.
- **Continuous driving** — `S` runs open-ended; the host must send periodic
  keepalives. The firmware safety watchdog window is 500 ms by default.

## Calibration parameters

Motor scale, PID gains, trackwidth, and navigation tolerances are all set live
with `K*` commands (e.g. `KCP`, `KCI`, `KTW`, `KGD`). The full table of `K`
parameters, units, and defaults is in the
[specification](https://github.com/League-Robotics/radio-robot/blob/master/docs/specification.md#calibration-parameters-table).

## Don't hand-roll the protocol

Use the [`robot_radio` host library](https://github.com/League-Robotics/radio-robot/blob/master/host/robot_radio/README.md);
it owns the serial port, framing, and command formatting. The layered API
(`NezhaProtocol` → `Nezha` → sensors/nav) is the supported way to talk to the
robot.
