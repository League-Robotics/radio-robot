---
title: Overview
blurb: What the radio-robot is, how the firmware and host fit together, and where to dig deeper.
order: 10
tags: [robot, firmware, microbit, nezha]
---

# Radio Robot

Radio Robot is the firmware and host software for the **DFRobot QBot Pro** — a
micro:bit V2 paired with a Nezha V2 motor board. The robot runs a C++
[CODAL](https://github.com/lancaster-university/codal) firmware that takes
movement and sensor commands over USB serial and micro:bit radio, executes
them on the hardware, and streams telemetry back to a Python host.

This page is the quick tour. Deeper material lives in the repository and is
linked throughout.

## How the pieces fit together

```
  Python host  ──serial / radio──►  micro:bit V2  ──I2C──►  Nezha V2 motors,
  (robot_radio)   protocol v2          (firmware)            OTOS, color, servo
       ▲                                                          │
       └──────────────── telemetry (OK / EVT / TLM / ID) ◄────────┘
```

- **Firmware** (`source/`) — C++14, no heap in the hot path, layered into
  `types / hal / control / app / nav`. It owns the command processor, the
  ratio-PID motor control, dead-reckoning odometry, and the pluggable
  path-following / pose-provider interfaces.
- **Host library** (`host/robot_radio/`) — the canonical, tested Python package
  for all host-side robot interaction. Everything that talks to the robot goes
  through it: motion, config, sensors, navigation, path planning, kinematics.
- **Wire protocol** — newline-terminated ASCII commands with sign-prefixed
  integer arguments (`+1234`, `-42`). Serial runs at 115200 baud; radio uses
  group 10 with `>`/`<` relay prefixes for wireless operation through a second
  micro:bit.

## What makes it interesting

- **Ratio-PID motor control.** Instead of plain velocity PI, the firmware
  tracks cumulative encoder distance per wheel and runs a PID controller on the
  normalized distance *ratio* between wheels, which kills long-run drift.
- **Arc-to-goal `G` command.** Computes an arc from the current pose to a
  relative XY target (optionally pre-rotating on large heading error) and drives
  it with encoder targets — point-to-point navigation without continuous pose
  feedback.
- **Pluggable navigation.** `PathFollower` (PurePursuit, Stanley) and
  `PoseProvider` (OTOS, dead-reckoning, future external camera) are pure-virtual
  interfaces selected at runtime.

## Start here

| You want to… | Go to |
|---|---|
| Build, flash, and test | [Getting Started](getting-started.md) |
| Understand the command set & wire format | [Protocol & Commands](protocol.md) |
| Read the full architecture | [docs/architecture.md](https://github.com/League-Robotics/radio-robot/blob/master/docs/architecture.md) |
| Read the feature spec | [docs/specification.md](https://github.com/League-Robotics/radio-robot/blob/master/docs/specification.md) |
| Use the Python host library | [host/robot_radio/README.md](https://github.com/League-Robotics/radio-robot/blob/master/host/robot_radio/README.md) |
| Debug firmware over SWD | [.claude/rules/debugging.md](https://github.com/League-Robotics/radio-robot/blob/master/.claude/rules/debugging.md) |

> **Source of truth lives in the repo.** This wiki summarizes and points back to
> [League-Robotics/radio-robot](https://github.com/League-Robotics/radio-robot);
> the linked files are always the authoritative version.
