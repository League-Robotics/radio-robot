---
status: pending
---

# Watchdog ConfigDelta acks ok but the value does not stick

## Evidence (2026-07-10 team-lead bench session, post-gut firmware v0.20260710.5)

Over the binary plane against real hardware:

- `CommandEnvelope{config: ConfigDelta{watchdog: 1500}}` → reply body **ok**.
- Immediately-following `CommandEnvelope{get: ConfigGet{target:
  CONFIG_WATCHDOG}}` → snapshot with `watchdog: 0`.
- Same via the high-level helpers: `NezhaProtocol.set_config(sTimeout=1500)`
  returns `{'sTimeout': '1500'}` (applied-echo), but
  `get_config("sTimeout")` reads back `'0'`. Retested ~100 ms later (many
  loop ticks) — still 0, so this is not a post-queue apply race.

The firmware accepts and acks the watchdog patch but either never applies it
to the Blackboard config, or the snapshot path reads a different variable
than the delta writes.

## Notes

- Found while bench-gating the rogo translator proxy (097-004); surfaced to
  the client as `GET sTimeout` → `CFG sTimeout=0` after a successful-looking
  `SET`.
- Whole-config context: after the mass-erase + reflash, the full CFG dump is
  all zeros except `tw=128` (defaults) — calibration must be re-pushed by the
  host, which is expected; the watchdog non-stick is the only observed
  set-then-read mismatch.
- Check whether the other ConfigDelta arms (drivetrain/motor/planner) apply
  correctly on hardware — only watchdog was caught misbehaving; a sweep
  set→get of every key over the binary plane on the stand would bound this.
