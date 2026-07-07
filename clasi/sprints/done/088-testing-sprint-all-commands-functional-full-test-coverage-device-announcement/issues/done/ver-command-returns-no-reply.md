---
status: done
sprint: 088
tickets:
- 088-001
---

# `VER` command returns no reply / host times out

## Context

Stakeholder reported (2026-07-07) that the `VER` statement "isn't working": sending
`VER` produces **no reply** and the host times out waiting for it.

`VER` is not a missing verb — it is registered and has a handler that looks
correct on its face:

- Registered: [system_commands.cpp:132](source/commands/system_commands.cpp#L132)
  — `cmds.push_back(makeCmd("VER", nullptr, handleVer, nullptr, "badarg"));`
- Handler: [system_commands.cpp:49-55](source/commands/system_commands.cpp#L49-L55)
  — builds `OK ver fw=<ver> proto=<n>` via `CommandProcessor::replyOK`.
- Documented: [docs/protocol-v2.md:217-231](docs/protocol-v2.md#L217-L231)
  (`VER [#id] → OK ver fw=<ver> proto=2 [#id]`).
- Host callers: [protocol.py:514-515](host/robot_radio/robot/protocol.py#L514-L515)
  (`send("VER", read_timeout=500)`), TestGUI
  [__main__.py:1696-1703](host/robot_radio/testgui/__main__.py#L1696-L1703)
  (logs `[WARN] VER query failed` / `VER gave no parseable reply (link drop?)`).

So the verb is wired and the reply string is right — yet nothing comes back. The
symptom (silence/timeout, not `ERR`, not wrong value) points at the **firmware
dispatch / registration / build wiring**, or the reply never reaching the wire —
not at a host-side parse failure.

## Investigation leads (systematic-debugging)

- **Fastest bisect: does `PING` work but `VER` not?** `PING`
  ([:131](source/commands/system_commands.cpp#L131)) has the *identical* shape —
  `makeCmd(prefix, nullptr, handler, nullptr, "badarg")`, `parseFn=nullptr`,
  replies via `replyOK`. If `PING` replies and `VER` is silent, diff the two
  `makeCmd` rows and the two handlers — the difference is the bug.
- **Dispatch / prefix matching.** Confirm the dispatcher matches the token `VER`
  and isn't shadowing/mis-routing it (e.g. a prefix collision, or `parseFn=nullptr`
  + `errKind="badarg"` silently dropping when a correlation id or stray arg is
  present).
- **Correlation id.** Check `VER` with and without a trailing `#id` — does the
  reply path fire in both cases?
- **Build-variant wiring.** Confirm the `system_commands` table is actually
  installed in the build under test (dev-loop `ROBOT_DEV_BUILD` vs full). Note the
  comment at [telemetry_commands.cpp:76](source/commands/telemetry_commands.cpp#L76):
  "PING/VER register (system_commands.cpp). Replies on its OWN dispatch" — verify
  that dispatch path is present and reached.
- **Channel.** Does `VER` fail on serial, radio, or both? (`VER` is a synchronous
  reply, so the relay's async-drop behavior should not affect it — if it fails
  only over the relay, that reframes the bug.)

## Acceptance

- Root cause identified and fixed; `VER` reliably returns
  `OK ver fw=<ver> proto=2` (with `[#id]` echoed when supplied) over the tested
  transport.
- Regression coverage: a sim/host test that sends `VER` and asserts the `OK ver`
  reply (parallel to any existing `PING` test).
- **HITL bench (on the stand):** `VER` returns a well-formed reply over the real
  link (serial at the bench, and over the radio/relay path).
