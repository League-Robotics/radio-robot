---
status: pending
priority: medium
---

# Sprint 132 leftovers: two capability gaps, five broken bench scripts, one untested robot

## Description

Sprint 132 retired the old config surface wholesale. Four things did not make
the crossing, and one robot changed behaviour without being tested. None blocked
the sprint; all need an owner.

## Cause

### 1. The stream-watchdog window has no live replacement

`CONFIG_WATCHDOG` died when ticket 013 deleted `config.proto` wholesale. Ticket
015 confirmed nothing took over. This is a capability that quietly left, not
dead code that was cleaned up. **Decide: restore it, or confirm it was already
vestigial and record that.**

### 2. `OI` (OTOS chip re-init) has no successor

The OTOS re-initialisation command has no equivalent in the new schema. Flagged
by ticket 014 rather than papered over. Note the surrounding constraints: OTOS
`init()` requires the robot stationary and has no completion poll, and `begin()`
cannot be re-run without teleporting the odometry origin — so whatever replaces
it needs care.

### 3. Five HITL bench scripts still call the deleted `.config(**…)`

Not pytest-collected, so nothing catches them; they will fail the next time
someone reaches for them at the bench:

- `src/tests/bench/plant_id.py`
- `src/tests/bench/velocity_step_response.py`
- `src/tests/bench/wheel_controller_ab_bench.py`
- `src/tests/bench/move_protocol_bench.py`
- `src/tests/dev/rig_dev.py`

Migrate onto `set_config_field()` / `set_config_group()` / `get_config()`.
**`src/tests/bench/velocity_profile_gate.py` is confirmed clean** — it was the
132-019 acceptance harness and does not use the deleted API.

### 4. `togov`'s effective `duty_per_speed` changed ~58%, untested

Ticket 009 reversed the 2026-07-31 "MEASURED, NOT CONFIGURED" decision, so
`duty_per_speed` now comes from each robot's own JSON instead of one hardcoded
constant. For `tovez` this was verified behaviour-preserving (`tovez.json` was
corrected to `0.001182`, matching the old constant, and the regenerated
`boot_config.cpp` diff touched only those literals).

**`togov` genuinely changes**: it now gets its own measured `0.00187325` rather
than the shared `0.001182` — about 58% more duty for the same commanded speed.
Arguably a fix, since the old constant baked one robot's plant gain onto every
robot. But it is **untested** — `togov` was not connected during sprint 132, and
nothing in the sprint's acceptance exercises it. **Verify on the stand before
driving it in anger.** If `0.00187325` is stale rather than measured, it will
overdrive.

### 5. `config_parity_capi.cpp` is a third touchpoint

Sprint 132's headline economic claim is "one new config field goes from 16
touchpoints to 2" (the `.proto` and the robot JSON). Ticket 018's demonstration
found `src/firm/config/config_parity_capi.cpp` is hand-maintained, making it a
genuine third touchpoint — for the parity *guard* specifically, not the core
wiring path. Small, but the claim should be stated accurately, and generating
that file would make it true as written.

## Verification

- Watchdog and `OI`: a decision recorded either way.
- Bench scripts: each runs against `tovez` on the stand.
- `togov`: a `velocity_profile_gate.py` run on the stand, compared against its
  pre-132 behaviour.
- Parity capi: either generated, or the 16→2 claim restated as 16→3.

## Related

- [[the-configuration-object]] — the design these fell out of.
- [[A-live-config-push-is-wiped-by-the-next-reconnect]]
- [[A-sprint-132-turn-accuracy-regression]]
