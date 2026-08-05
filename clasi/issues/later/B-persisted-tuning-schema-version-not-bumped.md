---
status: pending
priority: medium
---

# `pid.kff`/`pid.kaw` were repurposed into a different unit domain without a schema bump

## Description

2026-08-02 post-130 review, Part 1 **MAJOR**; sprint-130 midpoint finding #14
(filed then as a NOTE, "a live-tuning trap"). Verified today.

130-005 repointed two persisted tuning keys to new meanings:

| wire key | was | now |
|---|---|---|
| `pid.kff` | duty-domain feedforward | `kaff` **[s]** — the plant time constant |
| `pid.kaw` | duty-domain anti-windup | `pidMax` **[mm/s]** — a velocity clamp |

`src/firm/config/configurator.cpp:141-147` plumbs the new meanings consistently.
But `Config::kConfigSchemaVersion` is still **2**
(`src/firm/config/persisted_tuning.h:63`) — violating the file's own documented
rule at `:51`: *"bumped whenever a persisted field's meaning ... changes."*

## The failure

A robot carrying pre-130 values in flash boots duty-domain numbers into
mm/s-domain fields. `pidMax` in particular gates whether Stage B's output is
clamped **and** whether the deficit fault can latch at all (see
[[B-observability-contract-is-inert-as-shipped]]) — so a stale flash snapshot
silently arms Stage B with a wrong-domain limit, on a controller whose gains are
otherwise deliberately zero.

Nothing warns. There is also no config read-back to notice it after the fact
([[A-no-firmware-to-host-config-readback]]) — the two failures compound: wrong
values, silently applied, unobservable.

## What to do

1. Bump `kConfigSchemaVersion` to 3. The mechanism exists precisely for this.
2. Ideally also **NACK an old-shape snapshot loudly** rather than discarding it
   silently, so a robot that had tuning wiped says so.
3. While in here: the repurposed keys deserve real names at the next wire
   revision — `pid.kff`/`pid.kaw` now mean neither kff nor kaw. That is a
   protocol change, so file it against the next wire rev rather than doing it
   here (midpoint finding 14's own recommendation).

## Verification

- A robot with a v2 snapshot in flash boots with tuning **rejected**, not
  reinterpreted, and reports that it did.
- Push a `pid.kaff`, power-cycle, confirm it survives and reads back under the
  new schema version.
- `persisted_tuning.h`'s bump rule and the actual version agree.

## Related

- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` Part 1, first
  MAJOR.
- `docs/code_review/2026-08-02-sprint-130-midpoint.md` finding 14.
