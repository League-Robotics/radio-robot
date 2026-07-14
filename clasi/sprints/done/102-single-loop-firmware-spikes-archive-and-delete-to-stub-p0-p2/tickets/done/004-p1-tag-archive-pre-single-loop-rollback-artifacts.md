---
id: '004'
title: 'P1 tag + archive: pre-single-loop rollback artifacts'
status: done
use-cases:
- SUC-004
depends-on:
- '001'
- '003'
github-issue: ''
issue: single-loop-firmware-de-fiber-delete-the-elite-plumbing-telemetry-only-return-path.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# P1 tag + archive: pre-single-loop rollback artifacts

## Description

Build the reversibility safety net the stakeholder specified in place of
parked source code: an annotated git tag at the pre-deletion commit, plus
two archived, reflash-proven flashable hexes. This ticket must complete
**before** ticket 005 (P2 delete) runs, because ticket 005 deletes
`codal.devicebus.json` — the only source that builds the devicebus-bringup
image — so the devicebus-bringup hex must be built and archived from
working source while that source still exists.

Depends on tickets 001 and 003 (both P0 spikes) because the tag should mark
the commit where the spike verdicts are already recorded (the true
"pre-single-loop, fully de-risked" point), not an earlier one. (Ticket 002,
a serial baud-ceiling spike, was dropped by stakeholder decision 2026-07-14
— see sprint.md — so it is not a dependency.)

## Acceptance Criteria

- [x] Annotated git tag `pre-single-loop` created at the commit where P0's
      two spike verdicts are recorded (i.e., after tickets 001 and 003
      land), and pushed to the remote.
- [x] Default `MICROBIT.hex` built via `just build-clean` (never an
      incremental build for an artifact meant to be trusted — per the
      project's stale-incremental-build gotcha) and archived under
      `archive/` with its build version (from `types/version_generated.h`
      or equivalent) and flash notes (what config produced it, what it
      does/doesn't do).
- [x] Devicebus-bringup hex built from `codal.devicebus.json` (the config
      ticket 005 will delete) via a clean build, archived under `archive/`
      with the same documentation (build version + flash notes + the fact
      that its source config will no longer exist after ticket 005).
- [x] Each archived hex reflashed once onto the bench robot/rig using
      `mbdeploy`, confirmed to boot and identify itself correctly (banner
      or `HELLO`/`DEVICE:` response as applicable to that image) — proving
      the ARTIFACT is good, not just that the build succeeded.
- [x] Before each flash, `mbdeploy list`'s ROLE column checked to confirm
      the target is the robot (not the RELAY dongle) — they can share
      `/Volumes/MICROBIT`.
- [x] Archive notes are discoverable (e.g. `archive/README.md` or per-hex
      sidecar notes) stating: what each hex is, what commit/tag it was
      built from, and that it is the rollback path for sprint 102's P2
      delete.

## Completion Notes (2026-07-14)

- **Tag**: `pre-single-loop` (annotated), created and pushed at
  `1e2ec366327cfe7789434855ccd258efd2070d1a` — HEAD at the time of
  execution, i.e. the commit immediately after ticket 001 (`chore(102):
  move ticket 001 to done`), with ticket 003 already landed earlier in
  the branch history. Both P0 spike verdicts are recorded on this commit.
  Pushed with `git push origin pre-single-loop`.
- **Default hex**: built with `just build-clean` at `1e2ec366` on a clean
  tree, version `0.20260714.3` (`bench, BENCH_OTOS_ENABLED`), FLASH
  348452 B/364 KB (93.48%). Archived as
  `archive/hex/default-v0.20260714.3-1e2ec366.hex`
  (sha256 `1816c848f9e517dcec0dd159b665913d7e3fe821c7cc5582dc60dba33312133f`).
- **Devicebus-bringup hex**: built by copying `codal.devicebus.json` over
  `codal.json`, then `just build-clean`, same commit/version. FLASH
  145212 B/364 KB (38.96%). Archived as
  `archive/hex/devicebus-bringup-v0.20260714.3-1e2ec366.hex`
  (sha256 `d2e1804bbba729fe27984a8dc47192b841c17f601ca7a45548d17d42cd777259`).
  `codal.json` was restored immediately after the build; `git diff
  --stat -- codal.json` confirmed empty (unmodified) before committing.
- **Reflash-proof** (robot UID
  `9906360200052820a8fdb5e413abb276000000006e052820`,
  `/dev/cu.usbmodem2121102`; `mbdeploy list` ROLE column confirmed
  `NEZHA2`/robot, not the relay's `RADIOBRIDGE`, before each flash):
  - Devicebus-bringup flashed first via
    `mbdeploy deploy <UID> --hex archive/hex/devicebus-bringup-...hex`
    (one flash attempt hit a locked-flash error and self-recovered via
    mbdeploy's automatic CTRL-AP mass-erase-and-retry, as documented).
    Serial proof: `PING` -> `OK pong`; `RUNNING` -> `OK running=1`,
    interleaved with continuous unsolicited `TLM`/`STLM` telemetry
    confirming the DeviceBus fiber was running.
  - Default reflashed last (mass-erase-and-retry recovery again on the
    first attempt). Serial proof: `HELLO` ->
    `DEVICE:NEZHA2:robot:tovez:2314287040`; `PING` -> `OK pong t=16132`.
  - Robot left flashed with the **default** image, so the bench stays on
    production firmware. Only read-only verbs (`PING`/`HELLO`/`RUNNING`)
    were sent; motors were never commanded and the rig never moved.
- **Archive notes**: `archive/hex/README.md` documents both artifacts —
  version, source commit/tag, build config, SHA-256, flash instructions,
  and the reflash-proof evidence above.
- **Pytest**: `uv run python -m pytest` run after all artifact work — see
  ticket's Testing section outcome below; no source changes were made by
  this ticket (archive/ additions + ticket frontmatter only), so no
  regression was expected or found (the pre-existing
  `tests/testgui/test_canvas.py` asset-path failure is expected/known).

## Implementation Plan

**Approach**: This ticket produces artifacts, not source changes (beyond an
`archive/` directory). Sequence: (1) confirm tickets 001 and 003 are done and
their commits are on this sprint's branch; (2) tag; (3) clean-build the
default target and the devicebus target; (4) copy both hexes into
`archive/` with documentation; (5) reflash each once on the bench rig and
confirm it boots.

**Files to create/modify**:
- `archive/` — new directory (or reused if one already exists) holding the
  two hex files plus a notes file (`archive/README.md` or per-hex
  `.md`/`.txt` sidecars) documenting version, source commit/tag, and flash
  notes for each.
- No `source/` changes.

**Testing plan**: The reflash-and-boot check for each hex IS the test —
this ticket's acceptance criteria require a real flash, not just a
successful build. No new pytest needed.

**Documentation updates**: `archive/README.md` (or equivalent) documenting
both artifacts; this is the primary deliverable of the ticket alongside the
tag itself.

## Verification (hardware bench gate)

Per `.claude/rules/hardware-bench-testing.md`: robot bench-mounted, wheels
off the ground. Reflashing and confirming boot/banner is itself the bench
gate for this ticket — no drive/motor verification is required for the
archived-artifact proof (booting and self-identifying is sufficient; full
sensor/wheel verification is not the point of an archival ticket for
already-known-good firmware).

## Testing

- **Existing tests to run**: `uv run python -m pytest` (surviving suite —
  should still be green; this ticket doesn't touch source).
- **New tests to write**: none; the reflash-and-boot proof is a hardware
  verification step, not an automated test.
- **Verification command**: `uv run pytest`
