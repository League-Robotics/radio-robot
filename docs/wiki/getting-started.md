---
title: Getting Started
blurb: Set up the toolchain, build the firmware, flash the robot, and run the test suite.
order: 20
tags: [build, deploy, testing]
---

# Getting Started

This is the short path from a fresh clone to a running robot. For the full
detail — toolchain links, Docker, Yotta, IntelliSense — see the
[repository README](https://github.com/League-Robotics/radio-robot/blob/master/README.md).

## Prerequisites

- GNU Arm Embedded Toolchain (`gcc-arm-embedded`)
- Git, CMake, Python 3
- [`uv`](https://github.com/astral-sh/uv) for Python environments

On macOS, the clean setup is:

```sh
brew install --cask gcc-arm-embedded
brew install uv
```

(See the README for the symlink fix if `arm-none-eabi-gcc` isn't on `PATH`.)

## Python setup

From the repo root:

```sh
uv venv
uv sync
```

## Build the firmware

```sh
uv run python3 build.py        # produces MICROBIT.hex in the repo root
```

## Flash the robot — mbdeploy

`mbdeploy` is the standalone deploy tool. Install it once:

```sh
pipx install --editable ./mbdeploy      # or: just mbd-install
```

Then:

```sh
mbdeploy list                  # show connected micro:bit devices
mbdeploy deploy                # auto-detect the robot and flash MICROBIT.hex
mbdeploy deploy --build 1      # build, then flash device #1
```

A target can be an enum (`1`), a 5-char board name (`ZUVUB`), a serial path, or
a full pyOCD UID. The deployer reads each device's `DEVICE:` type so it never
flashes the radio relay.

> **Tip:** always do a `--clean` build before bench-flashing. Stale incremental
> builds can flash binaries that compile and pass tests but read garbage at
> runtime.

## Run the tests

The full suite (firmware-logic + host library, ~1000 tests, about a second):

```sh
uv run --with pytest python -m pytest -q
```

## Drive the robot

Talk to the robot through the host library / `rogo` CLI rather than ad-hoc
scripts:

```sh
uv run rogo --help
```

The closed-loop linear calibration tool (bench, hardware required):

```sh
uv run python tests/calibrate/calibrate_linear.py
```

## Where to go next

- [Protocol & Commands](protocol.md) — the wire format and command reference
- [Overview](overview.md) — how firmware and host fit together
- [docs/debugging.md](https://github.com/League-Robotics/radio-robot/blob/master/docs/debugging.md) — SWD/pyOCD debugging and chip recovery
- [docs/hardware-bench-testing.md](https://github.com/League-Robotics/radio-robot/blob/master/docs/hardware-bench-testing.md) — bench verification gate
