# Recordings (`data/recordings`)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-21 · **Status:** stable

---

## 1. Purpose

`data/recordings/` holds recorded telemetry logs (currently just
`latest.jsonl`) written by host-side tooling during bench/playfield runs.
It has **no architecturally significant content** — it is a data
directory, not source: a place captured JSONL telemetry lands, not code
or design worth its own Purpose/Orientation/Design sections. It is kept
as its own directory (rather than nested under `src/tests/bench/data/`
or similar) so recording output has one stable, well-known path
independent of which tool wrote it.

## 2. Interfaces

### Exposes

- **`latest.jsonl`** — the most recent recorded telemetry stream (one
  JSON object per line), written by whichever host-side capture tool ran
  last (see [`../../src/host/robot_radio/DESIGN.md`](../../src/host/robot_radio/DESIGN.md)
  for the writer(s) — e.g. `tlm_log.py` in
  [`../../src/tests/DESIGN.md`](../../src/tests/DESIGN.md)'s `bench/` tooling).

### Consumes

Nothing — this directory is a write target, not a dependency of
anything.

No further sections apply.
