---
id: '011'
title: GetConfig/ConfigSnapshot wire read-back + host get_config()
status: open
use-cases:
- SUC-003
depends-on:
- '007'
github-issue: ''
issue: the-configuration-object.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# GetConfig/ConfigSnapshot wire read-back + host get_config()

## Description

Add the `GetConfig`/`ConfigSnapshot` wire round trip, per-`ConfigTarget`
(the 240 B envelope cannot carry the whole object at once — see
sprint.md Design Rationale Decision 3). Firmware: `robot_loop.cpp`'s
`processMessage()` gains a `GetConfig` branch that reads
`Configurator::config()`'s relevant group and replies with a
`ConfigSnapshot` carrying that group's current values (reusing the same
generated group codec's encode path `applyGroup` used to decode). Host:
`NezhaProtocol.get_config(target)` sends `GetConfig` and returns the
parsed `ConfigSnapshot`.

## Acceptance Criteria

- [ ] A `GetConfig(target)` request returns a `ConfigSnapshot` carrying
      that group's current values from `Configurator::config()`.
- [ ] `get_config(target)` round-trips a value just pushed to that same
      target via `applyGroup` (push then get shows the pushed value, not
      a stale baked one) — verified in sim.
- [ ] `NezhaProtocol.get_config(target)` exists on the host and returns a
      typed result (using the generated pydantic group model from ticket
      002, not a raw dict).
- [ ] Compiles under `HOST_BUILD`.

## Testing

- **Existing tests to run**: unaffected — new capability, no existing
  call site changes.
- **New tests to write**: the push-then-get round-trip test above, for
  at least `DRIVE` and one boot-only target (confirming GET still works
  for boot-only groups even though SET is rejected — read-back is not
  gated by re-appliability, only writes are).
- **Verification command**: `uv run python -m pytest
  <configurator/protocol test paths> -q`.

## Implementation Plan

**Approach**: Add `GetConfig`/`ConfigSnapshot` handling in
`robot_loop.cpp`'s `processMessage()`, encoding `config_`'s relevant
group via the same generated encode path `install`'s wire equivalent
would use. Host side: a `get_config()` method on `NezhaProtocol`
mirroring the shape of its existing request/reply methods (e.g.
`PING`/`PONG`'s round-trip pattern).

**Files to modify**: `src/firm/app/robot_loop.cpp`,
`src/firm/app/configurator.{h,cpp}` (if `config()` needs a per-group
accessor rather than just the whole struct),
`src/host/robot_radio/robot/protocol.py`.

**Testing plan**: as above.

**Documentation updates**: `docs/protocol-v5.md` gets a short addendum
documenting `GetConfig`/`ConfigSnapshot` as part of the CONFIG binary
arm — confirm this is expected under this project's protocol-doc
maintenance convention before skipping it.
