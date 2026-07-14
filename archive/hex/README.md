# archive/hex — pre-single-loop rollback hexes (sprint 102, ticket 004)

This directory is the reversibility safety net for sprint 102's P2 delete
(ticket 005), which removes the entire "Elite" orchestration stack
(`source/runtime/`, `source/subsystems/`, `source/commands/`,
`source/drive/`, `source/telemetry/`, most of `source/hal/`,
`source/com/i2c_bus*`, `source/estimation/`, `codal.devicebus.json`, and
the vendored `ruckig`/`tinyekf`/`cmon-pid` libraries) and replaces
`source/main.cpp` with a banner-only stub.

Both hexes below were built from, and reflash-proven against, the git tag
**`pre-single-loop`** — the commit where both P0 de-risking spikes
(001: relay sustained-push telemetry measurement, 003: wire-frame budget
dry run) are recorded. That tag is the source-level rollback path
(`git checkout pre-single-loop` restores the full tree); these hexes are
the *binary* rollback path for when the source no longer builds the
devicebus-bringup image at all (ticket 005 deletes its only source,
`codal.devicebus.json`).

## Artifacts

| File | Image | Version | Source commit | Config |
|---|---|---|---|---|
| `default-v0.20260714.3-1e2ec366.hex` | Default production firmware (`source/main.cpp`, the full legacy Elite stack) | `0.20260714.3` | `1e2ec366327cfe7789434855ccd258efd2070d1a` (tag `pre-single-loop`) | `codal.json` (unmodified, `application: "source"`, no `application_entry`) |
| `devicebus-bringup-v0.20260714.3-1e2ec366.hex` | DeviceBus HITL bring-up image (`source/devices/bringup_main.cpp` only) | `0.20260714.3` | `1e2ec366327cfe7789434855ccd258efd2070d1a` (tag `pre-single-loop`) | `codal.devicebus.json` swapped over `codal.json` (`application_entry: "devices"`) — **this config file is deleted by ticket 005; this hex is the ONLY way to recover this image after that lands.** |

SHA-256:

```
1816c848f9e517dcec0dd159b665913d7e3fe821c7cc5582dc60dba33312133f  default-v0.20260714.3-1e2ec366.hex
d2e1804bbba729fe27984a8dc47192b841c17f601ca7a45548d17d42cd777259  devicebus-bringup-v0.20260714.3-1e2ec366.hex
```

## Build provenance

Both were produced by `just build-clean` (never an incremental build —
see `.claude/rules/hardware-bench-testing.md`'s stale-incremental-build
gotcha) at commit `1e2ec366`, on a clean working tree (`git status`
verified clean before each build).

- **Default**: `just build-clean` with the repo's checked-in `codal.json`
  as-is (`application: "source"`). Firmware summary: `v0.20260714.3
  (bench, BENCH_OTOS_ENABLED)`. FLASH 348452 B / 364 KB (93.48%), RAM
  120768 B / 122816 B (98.33%).
- **Devicebus-bringup**: `codal.devicebus.json` copied over `codal.json`
  (`cp codal.devicebus.json codal.json`), then `just build-clean`.
  Firmware summary: `v0.20260714.3 (bench, BENCH_OTOS_ENABLED)`. FLASH
  145212 B / 364 KB (38.96%), RAM 120768 B / 122816 B (98.33%) —
  substantially smaller, consistent with this image containing only
  `Devices::DeviceBus` + its own dedicated `main()`, per
  `source/devices/bringup_main.cpp`'s header comment. `codal.json` was
  restored to its original (default) content immediately after the build
  (`git diff --stat -- codal.json` verified empty afterward — no residual
  modification).

## Flash instructions

Flash by full UID with `mbdeploy` — never blind-`cp` to
`/Volumes/MICROBIT` (the robot and the RELAY dongle can share that mount
point). Confirm `mbdeploy list`'s ROLE column shows the target as the
robot (`NEZHA2`), not the relay (`RADIOBRIDGE`), before flashing.

```bash
mbdeploy list   # confirm ROLE=NEZHA2 for the target UID

# Default (production) image:
mbdeploy deploy <robot-UID> --hex archive/hex/default-v0.20260714.3-1e2ec366.hex

# Devicebus-bringup image:
mbdeploy deploy <robot-UID> --hex archive/hex/devicebus-bringup-v0.20260714.3-1e2ec366.hex
```

A locked/protected nRF recovers automatically via `mbdeploy`'s CTRL-AP
mass-erase-and-retry on a failed flash — no manual step needed (both
flashes in this ticket's reflash-proof hit this path and recovered
cleanly).

## Reflash-proof (sprint 102 ticket 004, 2026-07-14)

Both hexes were flashed once to the bench robot
(UID `9906360200052820a8fdb5e413abb276000000006e052820`,
`/dev/cu.usbmodem2121102`) and confirmed to boot and self-identify over
serial (115200 baud):

- **Devicebus-bringup**: `PING` → `OK pong` (interleaved in the
  continuous unsolicited `TLM`/`STLM` telemetry stream, confirming the
  DeviceBus fiber is running); `RUNNING` → `OK running=1`.
- **Default**: `HELLO` → `DEVICE:NEZHA2:robot:tovez:2314287040`; `PING` →
  `OK pong t=16132`.

The robot was left flashed with the **default** image afterward (default
reflashed last), so the bench stays on production firmware. Motors were
never commanded during this proof (`PING`/`HELLO`/`RUNNING` are read-only
verbs) — the rig's wheels/drums never moved.

## Why this directory also holds `source_old/`, `source_parked/`, `src/`, `tests_old/`, `wedgelab/`

Those predate this ticket and are unrelated parked-source archives from
earlier sprints (the greenfield rebuild). This `hex/` subdirectory is
sprint 102 ticket 004's own addition and does not touch or supersede
them.
