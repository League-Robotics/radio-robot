---
id: '001'
title: VER command dispatch/wiring bug fix
status: done
use-cases:
- SUC-002
depends-on: []
github-issue: ''
issue: ver-command-returns-no-reply.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# VER command dispatch/wiring bug fix

## Resolution (2026-07-07): NOT A DEFECT — VER verified working, closed with no code change

On-hardware verification against the currently-flashed firmware
(`fw=0.20260706.20`, robot on `/dev/cu.usbmodem2121102`, direct serial) shows
`VER` works correctly:

```
>>> 'VER'      <<< 'OK ver fw=0.20260706.20 proto=2\r\n'
>>> 'VER #7'   <<< 'OK ver fw=0.20260706.20 proto=2 #7\r\n'   (correlation id echoed)
```

`PING`, `ID`, `HELP` all reply correctly too, and the host-side parser
(`protocol.py:get_ver()`, checks `r.tokens[0] == "ver"`) is also correct. The
firmware handler and the host path both work; the original "no reply" report was
a transient (likely a momentary link/relay hiccup at the time), not a reproducible
defect. **Stakeholder confirmed VER works and directed this ticket closed** with no
fix. Regression coverage for `VER` on both channels is carried by the ticket-007
smoke suite (no redundant standalone test added here).

## Description

`VER` is registered (`system_commands.cpp:132`) and its handler
(`handleVer`, `system_commands.cpp:49-55`) builds a well-formed
`OK ver fw=<ver> proto=<n>` reply via `CommandProcessor::replyOK` — but
sending `VER` produces no reply and the host times out. `PING`
(`system_commands.cpp:131`, `handlePing`) has the *identical* shape
(`makeCmd(prefix, nullptr, handler, nullptr, "badarg")`, `parseFn=nullptr`,
replies via `replyOK`) and works. Sprint planning's own static read of the
tokenizer, `prefixMatchLen`/`dispatchTable`, and both handler bodies found
no defect (see `architecture-update.md`'s Grounding section) — this ticket
must find the actual root cause, which may require on-target bisection.

## Implementation Plan

**Approach**: Follow the issue's investigation leads in order:
1. Diff `handleVer` against `handlePing` and their `makeCmd(...)` rows
   again with fresh eyes — the difference, if any, is the bug.
2. Confirm the dispatcher matches the token `VER` and isn't
   shadowed/mis-routed (check `prefixMatchLen` against `VER` specifically,
   including with and without a trailing `#id`).
3. Confirm the `ROBOT_DEV_BUILD` command table genuinely includes `VER` in
   the build under test — verify the assembled table reaches the dispatch
   path described at `telemetry_commands.cpp:76`'s comment.
4. Test on both serial and radio — if only one channel fails, that
   reframes the bug (e.g. a reply-channel resolution issue, not a
   dispatch issue).
5. If steps 1-4 find nothing, escalate to on-target debugging per
   `.claude/rules/debugging.md` (batch-mode `arm-none-eabi-gdb`, non-
   interactive) — do not guess-and-check on hardware.
Use the `systematic-debugging` skill's four-phase protocol if the first
few attempts don't converge.

**Files likely touched**: `source/commands/system_commands.cpp` (if the
defect is local to `VER`'s own handler/registration) or, if broader,
`source/commands/command_processor.cpp` / `source/runtime/command_router.cpp`.

**Testing plan**: add a sim/unit test mirroring an existing `PING` test,
asserting `VER`'s `OK ver` reply. HITL bench spot-check on both channels.

**Documentation updates**: none expected unless the root cause reveals a
gap in `docs/protocol-v2.md`'s `VER` contract.

## Acceptance Criteria

- [x] Root cause investigated: **no defect** — VER verified working on hardware
      (see Resolution). Original report was a non-reproducible transient.
- [x] `VER` reliably returns `OK ver fw=<ver> proto=2` (with `[#id]` echoed when
      supplied) on serial — verified on hardware.
- [~] `VER` on radio: not separately re-verified; stakeholder confirmed VER works
      and directed closure. Radio spot-check folded into ticket 009 if desired.
- [x] Regression coverage: carried by the ticket-007 smoke suite (VER on both
      channels), rather than a redundant standalone test.
- [x] HITL bench: `VER` returns `OK ver fw=0.20260706.20 proto=2` over the real
      serial link.

## Testing

- **Existing tests to run**: whichever `tests/sim/unit/` file already
  covers `PING`/liveness commands (e.g. `test_protocol_roundtrips.py`),
  plus the full suite.
- **New tests to write**: a `VER`-specific sim test in the same
  file/pattern as the existing `PING` test.
- **Verification command**: `uv run python -m pytest tests/sim/unit/test_protocol_roundtrips.py`
  then the full `uv run python -m pytest`.
