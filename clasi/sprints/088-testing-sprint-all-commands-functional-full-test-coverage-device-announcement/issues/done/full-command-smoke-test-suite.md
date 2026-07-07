---
status: done
sprint: 088
tickets:
- 088-006
- 088-007
---

# Full command smoke-test suite: one test per registered command, exercising serial + radio, asserting an effect

## Context

Stakeholder request (2026-07-07): a comprehensive **smoke test** covering **every
command registered in the system**, so that "full test" proves the entire command
surface is functional. Requirements as stated:

- A collection of tests, one **function per registered command**.
- Each function verifies:
  1. the command can be sent through the **serial** port,
  2. the command can be sent through the **radio**,
  3. the command **produces an effect**.
- Smoke-level only — not exhaustive per-command behavior, just "this command is
  registered, dispatches on both channels, and does something."

The registered command surface (dev build) is assembled by
[command_router.cpp:24-41](source/runtime/command_router.cpp#L24-L41): `systemCommands`
(PING VER HELP ECHO ID) + `devCommands` (DEV family) + `telemetryCommands` (STREAM
SNAP) + `motionCommands` (S T D R TURN RT G STOP) + `configCommands` (SET GET) +
`poseCommands` (SI ZERO) + `otosCommands` (OI OZ OR OP OV OL OA). The suite's command
list must equal this registered table (see completeness guard below).

## Prerequisite: the sim harness has no serial/radio distinction yet

The command-driving harness is `sim.command()` →
[sim_api.cpp `sim_command`](tests/_infra/sim/sim_api.cpp#L296) (ctypes via
`host/robot_radio/io/sim_conn.py`), used by existing sim/unit tests
([test_protocol_roundtrips.py](tests/sim/unit/test_protocol_roundtrips.py),
`test_motion_commands.py`, `test_pose_commands.py`, `test_otos_commands.py`).

Today it **cannot** satisfy requirements 1+2 as written: `sim_command()` hardcodes
`stmt.returnPath = Subsystems::Channel::SERIAL`
([sim_api.cpp:302](tests/_infra/sim/sim_api.cpp#L302)), and the router's two reply
channels are both wired to the *same* sync sink — "the sim has no real transport
distinction, so both resolve to the same sink"
([sim_api.cpp:19-23](tests/_infra/sim/sim_api.cpp#L19-L23)).

So the first piece of work is a **harness extension**: let a test choose the return
channel (SERIAL vs RADIO) and observe the reply that comes back *on that channel* —
e.g. a `sim_command_on(h, line, channel, …)` entry (or a channel arg to
`sim_command`), with the router's serial and radio reply sinks kept **distinct** so
a test can assert the reply arrived on the channel it was sent on. Only then can each
smoke test genuinely exercise both channels.

## Open questions for planning

- **Where does it live?** The stakeholder said `tests/unit`. The sim command harness
  and the always-run pytest gate live under `tests/sim/` (`pyproject.toml` `testpaths`
  collects `tests/sim/` only; [tests/CLAUDE.md](tests/CLAUDE.md)). Recommend
  `tests/sim/unit/` (alongside the existing command tests, already collected);
  confirm, or extend `testpaths` if it must be `tests/unit/`.
- **"Send through the radio" — sim or real?** In pure sim, "radio" means routing a
  `Channel::RADIO` return path through the harness (automatable in the CI gate). A
  *true* over-the-air radio-relay send is HITL and belongs in `tests/bench/` (not
  pytest-collected). The `tests/unit` + "unit test" framing points at the sim
  interpretation; confirm whether a real-radio bench smoke is also wanted.
- **"Produces an effect" for hardware-gated verbs.** On the sim, some verbs reply
  `ERR unsupported` / `ERR nodev` by design (e.g. OTOS `OI/OL/OA`, motor `POS/VOLT`
  on `SimMotor` — [test_protocol_roundtrips.py:31-41](tests/sim/unit/test_protocol_roundtrips.py#L31-L41)).
  A smoke test should treat a **well-formed reply** (`OK …` or a *defined* `ERR …`)
  as "registered and dispatches," i.e. it proves reachability, not hardware presence.
- **DEV family granularity.** Decide whether `DEV` is one smoke test or one per
  subcommand (`DEV M`, `DEV DT`, `DEV WD`, `DEV STATE`, …). Enumerate by the same
  unit the registered table uses.

## Completeness guard

Add a meta-test that enumerates the registered command table and **fails if any
registered verb lacks a smoke test** (and vice versa), so the suite cannot silently
fall behind as command families are added. This dovetails with the separate
[HELP-enumeration issue](clasi/issues/help-should-reflect-registered-commands.md):
once `HELP` reports the live table, the guard can cross-check against it.

## Acceptance

- One smoke-test function per registered command, each sending the command on
  **both** channels (via the extended channel-aware harness) and asserting a
  well-formed effect (reply and/or observable state change).
- The harness extension lands: a test can select SERIAL vs RADIO and observe the
  reply on the chosen channel.
- The completeness meta-test passes and would fail if a registered verb had no smoke
  test.
- `uv run python -m pytest` runs the suite green as part of the standard gate.
