---
status: pending
---

# On-chip fake/simulated OTOS test device for the bench (synthetic OTOS pose, heading from encoders)

## Context

During bench bring-up (2026-07-20) we confirmed there is **no firmware-side
fake OTOS today**. `MICROBIT.hex` is always the real image talking to the real
I2C OTOS chip; `SimPlant` (the honest simulated I2C/OTOS source) is **host-only**
C++ compiled into `libfirmware_host.dylib` and can never run on the micro:bit.

On the bench the real OTOS is on a servo port, not the I2C bus, so
`otos.present()` is false and `App::HeadingSource` falls back to
encoder-derived heading automatically (`heading_source: "auto"`). For the
current bench tour bring-up we **accept that AUTO fallback** — this issue is the
follow-up to do it properly.

## What we want

A firmware-side, **build-selectable fake OTOS "test version"** — a
`Devices::Otos`-compatible test device that synthesizes an OTOS pose on-chip
instead of reading the I2C chip, so the robot can **report a synthetic OTOS
pose** (`frame.otos`) and drive heading from it.

Per stakeholder: it is **used for heading, and the heading is derived from the
encoders** — i.e. the synthetic OTOS pose is generated from the wheel/encoder
kinematic model (the same forward kinematics `App::Odometry` and the host
`SimPlant` use). This gives the bench a self-consistent **OTOS-present** regime
(matching how the sim closure gate validates with OTOS heading) without a
physical OTOS.

## Scope / design notes (for planning)

- **Build seam is missing.** The ARM composition root hard-wires the real bus:
  `src/firm/main.cpp:91` constructs `MicroBitI2CBus` and `main.cpp:122-128`
  constructs `Devices::Otos` against it, with no way to substitute a fake.
  Needs either a compile-time build variant (e.g. a `FAKE_OTOS` / bench-test
  macro) or a constructor seam.
- **Pose source.** The fake OTOS derives its pose from encoder odometry
  (`BodyKinematics::forward` over real encoder deltas), so heading = encoder
  heading — surfaced through the normal OTOS/`HeadingSource` path
  (`usingOtos_ = true`) rather than the encoder-fallback branch.
- **Keep production untouched.** The real `Devices::Otos` path must stay
  unchanged for production; this is a test/bench build only.
- **Related code:** `App::HeadingSource` encoder fallback
  (`src/firm/app/heading_source.{h,cpp}`), `App::Odometry`
  (`src/firm/app/odometry.cpp`), host `SimPlant` (`src/sim/sim_plant.*`).
- **Do not reuse** the stale worktree `.claude/worktrees/bench-otos/` — it is
  the pre-rebuild `source/` layout, not the live tree.

## Priority

Normal — **not blocking** the current bench tour bring-up (AUTO fallback covers
that). This makes the bench OTOS-present and self-consistent for future work.
