---
id: '006'
title: Sim command harness channel extension (SERIAL vs RADIO)
status: done
use-cases:
- SUC-006
depends-on:
- '004'
github-issue: ''
issue: full-command-smoke-test-suite.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sim command harness channel extension (SERIAL vs RADIO)

## Description

`tests/_infra/sim/sim_api.cpp`'s `sim_command()` hardcodes
`stmt.returnPath = Subsystems::Channel::SERIAL`, and `CommandRouter`'s two
reply channels are both wired to the same shared `ReplyStore`
(`router.setReplyChannels(storeReply, &syncStore, storeReply, &syncStore)`,
confirmed at `sim_api.cpp:230`) — so no existing test can prove a command
actually dispatches correctly on radio versus serial, only that a
`Channel::RADIO`-tagged field can be set. This is the prerequisite
extension the full command smoke suite (ticket 007) needs. This ticket
depends on ticket 004 (the statement→command/message rename) because
`sim_api.cpp` names `Subsystems::CommunicatorToCommandProcessorStatement`
directly — it should be written against the final, renamed type.

## Implementation Plan

**Approach**: Give `SimHandle` a second `ReplyStore` (rename `syncStore` →
`syncStoreSerial`, add `syncStoreRadio`); wire
`router.setReplyChannels(storeReply, &syncStoreSerial, storeReply, &syncStoreRadio)`
instead of both channels pointing at one store. Add
`int sim_command_on(void* h, const char* line, int channel, char* reply, int size)`
that sets `stmt.returnPath` from the `channel` argument (matching
`Subsystems::Channel`'s enum values: 1=SERIAL, 2=RADIO) and reads the
reply from the matching store. Keep `sim_command()` as a thin SERIAL-only
wrapper: `return sim_command_on(h, line, /*SERIAL*/ 1, reply, size);` —
zero changes required at any existing test call site (~183 test functions
across `tests/sim/unit/` share this fixture entry point). In
`host/robot_radio/io/sim_conn.py`, add the `sim_command_on` ctypes binding
(`argtypes`/`restype` matching `sim_command`'s pattern plus one `c_int`
channel argument) and `CHANNEL_SERIAL = 1` / `CHANNEL_RADIO = 2` constants.

**Files to create/modify**: `tests/_infra/sim/sim_api.cpp`,
`host/robot_radio/io/sim_conn.py`.

**Testing plan**: a new test proving channel isolation — send a command on
RADIO and assert the SERIAL store stays empty (and vice versa), and that
each store's reply is well-formed for its own send.

**Documentation updates**: none beyond the code's own doc comments.

## Acceptance Criteria

- [x] `sim_command_on(h, line, channel, reply, size)` exists and lets a
      test select SERIAL or RADIO as the return channel for one command.
- [x] `CommandRouter`'s two reply sinks are backed by distinct `ReplyStore`
      instances in the harness; a reply sent on one channel is NOT
      observable via the other channel's store.
- [x] `sim_command()` becomes a thin SERIAL-only wrapper over
      `sim_command_on()`; every existing test call site is
      source-compatible and requires zero changes.
- [x] `host/robot_radio/io/sim_conn.py` exposes the new entry point via
      ctypes, matching the existing `sim_command()` binding pattern, plus
      `CHANNEL_SERIAL`/`CHANNEL_RADIO` constants.
- [x] A new test proves channel selection works end-to-end (send on
      RADIO, assert the SERIAL store is empty; send on SERIAL, assert the
      RADIO store is empty).

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/sim -q` — green
  with zero existing test-file edits (proves the wrapper is truly backward
  compatible): 264 passed, 4 xfailed (baseline 260 passed/4 xfailed + this
  ticket's 4 new tests). The repo-wide `uv run python -m pytest` (all
  `testpaths`, including `tests/testgui`) has 2 pre-existing failures
  unrelated to this ticket (`test_set_origin.py`/`test_tour1_geometry.py`,
  multi-leg-motion-via-headless-GUI issues already tracked outside this
  ticket's scope) — confirmed unchanged by stashing this ticket's diff and
  re-running those two tests, which fail identically without this ticket's
  changes.
- **New tests written**: `tests/sim/unit/test_sim_command_channel.py` — 4
  tests: RADIO reply lands only in the RADIO store, SERIAL reply lands only
  in the SERIAL store, `sim_command()` stays SERIAL-only, and back-to-back
  RADIO/SERIAL calls don't cross-contaminate. Verified via a non-draining
  peek entry point added for test observability
  (`sim_get_reply_store_len(h, channel)` / `sim.reply_store_len(channel)`,
  not itself used by `sim_command()`/`sim_command_on()`).
- **Verification command**: `uv run python -m pytest tests/sim -q`.
- **Implementation note**: in addition to the two files named above,
  `tests/_infra/sim/firmware.py` (the `Sim` wrapper backing the `sim`
  pytest fixture that `tests/sim/unit/*.py` — including the new test file
  above — actually runs against) gained the same channel-aware surface
  (`command_on()`, `reply_store_len()`, `CHANNEL_SERIAL`/`CHANNEL_RADIO`)
  as `sim_conn.py`, so the new test could be written the normal way (via
  the `sim` fixture) rather than reaching into ctypes directly.
