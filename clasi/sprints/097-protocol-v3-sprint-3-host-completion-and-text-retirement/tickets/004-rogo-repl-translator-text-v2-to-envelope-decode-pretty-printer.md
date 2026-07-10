---
id: '004'
title: 'rogo translator proxy: text-v2 PTY bridge fronting the real binary robot connection'
status: in-progress
use-cases:
- SUC-004
depends-on:
- '002'
- '003'
github-issue: ''
issue: protocol-v3-schema-driven-binary-command-plane-protobuf.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# rogo translator proxy: text-v2 PTY bridge fronting the real binary robot connection

## Description

**REWRITTEN — see `architecture-update-r2.md` Decision 9 (supersedes r1's
Decision 8 and this ticket's own original "rogo REPL translator" scope in
full).** Decision 9's M5 sketch is itself further revised by two
same-day stakeholder decisions (2026-07-10), recorded as a short addendum
at the end of `architecture-update-r2.md` and detailed in full in the
authoritative implementation spec **`clasi/issues/rogo-translator-proxy-
text-v2-binary-bridge-on-a-pty.md` — READ THAT ISSUE FIRST.** It carries
the complete verb-routing table, PTY-lifecycle edge cases, and firmware
format-string file:line citations; this ticket file summarizes the
resulting scope and states acceptance criteria, it does not duplicate the
spec.

The original plan (extend `rogo send` to translate one typed line at a
time for a human at a terminal) is superseded by a bigger, different
shape: `rogo` gains a **`proxy` mode** that is a persistent, standalone
text-v2-speaking bridge fronting the real, binary-only robot connection —
the host's own answer to "how does a legacy text client keep working once
the firmware only speaks binary" (tickets 006/007/008 gut the firmware
text plane unconditionally this sprint; this proxy is what makes that safe
to do without migrating every legacy consumer first).

**What it does**:

1. Creates a **PTY** (`os.openpty()`) — NOT a Unix-domain socket as r2
   Decision 9 originally chose. Publishes a stable symlink to the PTY
   slave device path (default `~/.rogo/robot-pty`, overridable via
   `--link`). Every legacy consumer already opens its serial port as a
   device path (`serial.Serial(path)` / `SerialConnection(port)`), so a
   PTY is a **zero-code-change drop-in** — a socket would have forced a
   code change into every consumer, recreating the migration problem the
   proxy exists to avoid. Single-client: documented in the module
   docstring and `--help`, not policed by the proxy itself. The routing
   core (steps 3-6 below) is transport-agnostic, so an additive
   `AF_UNIX` listener remains cheap to add later if multi-client need
   materializes.

   *Socket rationale, superseded*: r2 Decision 9 chose `AF_UNIX` for
   OS-buffered bidirectional multi-client framing and `nc -U`
   pokability. Superseded 2026-07-10: PTY wins because it is a
   zero-code-change drop-in for every existing legacy consumer — the
   decisive property, since the whole point of the proxy is "worry
   about the consumer later." `nc -U` pokability is replaced by
   `screen <pty-path>`.

2. Owns the REAL robot connection (`SerialConnection`, serial or relay) —
   exactly one instance.
3. Reads text-v2 lines from the single connected client (the SAME
   grammar a human or legacy tool would type/send today: `S 200 200`,
   `D 200 200 300`, `SET tw=128`, `STREAM 50`, `PING`, etc.) via a
   pty-reader thread.
4. Tokenizes and routes each line via `legacy_verbs.py`'s dispatch
   tables to the matching `CommandEnvelope` oneof arm, or to a local
   (non-wire) handler for verbs with no binary counterpart. This covers
   every verb the original ticket listed (`S`/`D`/`T`/`RT`/`MOVE`/
   `MOVER`/`ECHO`/`PING`/`ID`/`HELLO`/`HELP`/`STOP`/`SET`/`GET`/
   `STREAM`/`SNAP`) PLUS additions surfaced by the issue spec: `TLM`
   one-shot (renders an `OK tlm ...` bench body from a binary snap);
   `QLEN`/`G`/`R`/`TURN`/`GRIP`/`DEV *`/unknown → local typed
   `ERR unsupported <verb>` (never a hang, never a silent drop);
   relay-control lines (`!MODE`/`!CG`/`!P`/`!ECHO`/`!GO`/`?`) swallowed
   locally with a `# ok` comment reply; `+` keepalive forwarded via
   `send_fast` (feeds the firmware watchdog); `HELLO` answered locally
   from a startup-cached binary `DeviceId` (upstream passthrough can't
   work — the reader drops `DEVICE:` lines); `HELP` answered locally;
   pose/otos verbs (`SI/ZERO/OI/OZ/OR/OP/OV/OL/OA`) gated behind a
   `_POSE_OTOS_BINARY=False` flag → `ERR unsupported` until sprint 098
   lands binary pose/otos arms. Full routing table and ERR-code mapping:
   the issue spec.
5. Sends the resulting `CommandEnvelope` to the real robot via
   `SerialConnection.send_envelope()`.
6. Renders the binary reply (`Ack`/`Error`/`DeviceId`/`Telemetry`/
   `ConfigSnapshot`) back into the equivalent text-v2 reply line via
   `legacy_render.py` — the reverse of step 4's mapping, transcribed
   from the firmware's own format strings with file:line citations.
7. Unsolicited binary `Telemetry` push frames (ticket 001's
   `_binary_tlm_queue`) are forwarded to the **single connected client**
   as text `TLM ...` lines **when a stream is armed** — by that
   client's own `STREAM n`, or internally by the EVT watcher (item 8, at
   `--watch-period` when the client hasn't armed one; internal-only
   frames are never forwarded to the PTY). This replaces r2's "every
   connected client" fan-out language, which no longer applies under a
   single-client PTY transport.
8. **Synthesizes `EVT done <VERB> [#id] reason=idle`.** Current firmware
   emits **no EVT at all** (`CommandProcessor::emitEvent` has zero
   producers — verified), yet legacy calibration scripts block on
   `EVT done D/T`. This is new host-side scope, not a translation of an
   existing firmware signal. A `_EvtWatcher` state machine (owned by the
   tlm-pump thread) watches the binary `Telemetry.active` flag
   (unconditionally present in every binary frame) — **not** the `mode`
   char; nothing writes `bb.planner.mode`, it is always `I`. Ack for
   `T`/`D`/`RT`/`MOVE` arms the watch (`WAIT_BUSY`, 2 s cap); `active`
   going true moves to `BUSY`; `active` going false — or the 2 s cap
   expiring while still `WAIT_BUSY` — emits the event and disarms;
   `STOP` clears the pending watch silently. **Gap, flagged plainly**:
   `EVT safety_stop` is NOT synthesizable — there is no binary
   watchdog-stop signal to watch. This is not a regression: firmware
   emits no EVT today either.

`rogo binary <arm>` (095/096) and every other existing `rogo` subcommand
are UNAFFECTED — `proxy` is a new, additive mode, not a replacement.

**Explicitly deferred, NOT this ticket's job** (Eric: "worry about the
consumer later"): pointing TestGUI, `robot_mcp.py`, `calibration/
linear.py`/`angular.py`, `gamepad_teleop.py`, or any bench script AT the
proxy socket. This ticket only BUILDS the proxy and PROVES it translates
correctly with a test client. Wiring real consumers to it is a follow-up
(tracked by `realign-host-tooling-to-gutted-four-verb-wire-surface.md`,
whose scope this revision narrows from "migrate to `NezhaProtocol`
directly" to "point at the proxy socket").

## Acceptance Criteria

- [ ] `rogo proxy` creates a PTY (`os.openpty()`), publishes a symlink at
      `~/.rogo/robot-pty` by default (`--link` to override), and serves
      exactly one connected client at a time (documented, not policed).
- [ ] `legacy_verbs.py` tokenizes and dispatches EVERY verb in the issue
      spec's verb-routing table to its `CommandEnvelope` oneof arm or
      local handler — including the additions beyond the original
      step-4 list: `TLM` one-shot; `QLEN`/`G`/`R`/`TURN`/`GRIP`/`DEV *`/
      unknown → local typed `ERR unsupported <verb>` (never a hang or
      silent drop); relay-control lines swallowed locally with `# ok`;
      `+` keepalive forwarded via `send_fast`; `HELLO` answered locally
      from a startup-cached binary `DeviceId`; `HELP` answered locally;
      pose/otos verbs gated behind `_POSE_OTOS_BINARY=False` →
      `ERR unsupported` until sprint 098.
- [ ] A TEST CLIENT opens `serial.Serial(<pty-slave-path>)` (pyserial)
      against the proxy's PTY — NOT `nc -U` (superseded by the PTY
      transport decision) — writes a text line for EACH covered verb,
      and receives the correct translated text reply, while the proxy
      itself speaks ONLY binary (`*B<base64>`) to a fake/sim robot
      connection underneath — verified by asserting on the bytes the
      proxy's OWN `SerialConnection` writes, not just the client-visible
      reply.
- [ ] A test exercises unsolicited `TLM` forwarding to the single
      connected client: arm binary streaming on the underlying
      (fake/sim) connection, confirm the client receives text
      `TLM ...` lines when a stream is armed (client `STREAM n`, or the
      internal watch-period stream while an EVT watch is pending). This
      replaces the old multi-client TLM fan-out criterion, dropped by
      the PTY (single-client) transport decision.
- [ ] `_EvtWatcher` synthesizes `EVT done <VERB> [#id] reason=idle` off
      the binary `Telemetry.active` flag: a test exercises
      WAIT_BUSY→BUSY→idle (event fires once), the 2 s WAIT_BUSY cap
      firing anyway, `STOP` clearing the pending watch silently, and a
      new motion verb superseding a pending one. Completion notes state
      plainly that `EVT safety_stop` is not synthesizable (no binary
      signal) and this is not a regression (firmware emits no EVT
      today).
- [ ] The committed-but-RED `tests/unit/test_cli_send_translator.py`
      (pins `cli._tokenize_send_line`/`cli._SEND_RUMP_VERBS`/
      `cli._decode_reply_body`/`cmd_send`) goes GREEN via thin `cli.py`
      aliases over `legacy_verbs.py` — delivering the ticket's original
      `rogo send` REPL scope at near-zero incremental cost.
- [ ] `rogo binary <arm>` and every other existing `rogo` subcommand are
      byte-for-byte unaffected by this ticket's diff.
- [ ] `tests/sim` stays green (host-only ticket; sanity check).
- [ ] `tests/unit` is green, including all new
      `legacy_verbs`/`legacy_render`/proxy tests.
- [ ] Hardware bench gate (per `.claude/rules/hardware-bench-testing.md`,
      robot on stand): flagship test is the unmodified
      `calibration/linear.py --port ~/.rogo/robot-pty --direct` running
      end-to-end through the proxy with zero code changes, plus
      `gamepad_teleop.py` over the PTY at 20 Hz with healthy `q=` flow
      control. Executed by the team-lead after the firmware-gut lane
      (ticket 008, running in parallel) lands — tracked as this
      ticket's bench-gate step, not blocking the ticket's own
      code-complete state.
- [ ] Completion notes state plainly: consumer rewiring (TestGUI, MCP,
      calibration scripts, bench/demo scripts) to point at this proxy is
      OUT of this ticket's scope, deferred to
      `realign-host-tooling-to-gutted-four-verb-wire-surface.md`.

## Implementation Plan

### Approach

1. `host/robot_radio/robot/legacy_verbs.py` (NEW, pure): tokenizer
   (`tokenize_send_line`, `split_corr_id`, `kvfloat`) + verb-to-envelope
   builders + dispatch tables (`BINARY_DISPATCH`, `RUMP_VERBS`,
   `PROTOCOL_VERBS`) + `decode_reply_body()` pretty-printer — thin
   wrappers over the EXISTING `legacy_translate.py` motion builders
   (ticket 002, reused not rewritten). Makes
   `tests/unit/test_cli_send_translator.py` green via the `cli.py`
   aliases (see Files).
2. `host/robot_radio/robot/legacy_render.py` (NEW, pure): the reverse
   direction — every renderer transcribed from the firmware's own
   format strings with file:line citations (`render_tlm_line`,
   `render_ok`/`render_err`, per-verb `ok_body_for`, `ERR_CODE_TEXT`,
   `render_id_line`/`render_ver_body`/`render_device_banner`,
   `render_cfg_line`, `render_evt_done`).
3. `host/robot_radio/io/proxy.py` (NEW): `ProtocolBridge` — PTY
   lifecycle (`os.openpty()`, symlink publish/cleanup, non-blocking
   master fd, write lock with drop-on-`BlockingIOError` for TLM lines /
   short retry for replies and EVT), pty-reader thread, tlm-pump thread,
   `_handle_client_line` routing, `_EvtWatcher` (pure state machine,
   separately unit-testable).
4. `host/robot_radio/io/cli.py` (MODIFIED): `rogo proxy` subcommand
   (`cmd_proxy`, options `--link`, `--watch-period`, `--no-evt`); thin
   `cmd_send` + module aliases over `legacy_verbs` to satisfy the
   committed test.
5. Corr-id handling: single in-flight command (the pty-reader processes
   lines serially) — client's trailing `#<digits>` is stripped/saved
   and re-attached to the rendered reply as a local variable, no map
   needed.
6. Full per-verb routing, ERR-code mapping, and PTY-lifecycle edge cases
   (macOS-vs-Linux client-close behavior, the `tcdrain` invariant) are
   specified in `clasi/issues/rogo-translator-proxy-text-v2-binary-
   bridge-on-a-pty.md` — follow it as the implementation spec rather
   than re-deriving any of the above from scratch.

### Files to modify

- `host/robot_radio/robot/legacy_verbs.py` — NEW. Pure tokenizer/
  dispatch/pretty-printer module. **Deviation from this ticket's
  original "extend `legacy_translate.py` both directions" instruction**:
  `legacy_translate.py`'s existing pure numeric builders (ticket 002)
  stay untouched; tokenizing and dispatch live in this new sibling
  module instead — module-per-concern, an implementer-naming choice
  this ticket already permitted.
- `host/robot_radio/robot/legacy_render.py` — NEW. Pure reverse-direction
  (binary-reply-to-text) renderers — the other half of the "both
  directions" translator coverage, kept out of `legacy_translate.py` for
  the same module-per-concern reason.
- `host/robot_radio/io/proxy.py` — NEW. `ProtocolBridge` daemon: PTY
  lifecycle + routing + `_EvtWatcher`.
- `host/robot_radio/io/cli.py` — MODIFIED. New `proxy` subcommand + thin
  `cmd_send`/aliases over `legacy_verbs`.

### Testing plan

- `tests/unit/test_cli_send_translator.py` — already committed
  (currently RED); goes green via the `cli.py` aliases over
  `legacy_verbs.py`.
- New `tests/unit/test_legacy_render.py` — golden lines vs. firmware
  formats (TLM full/minimal frame, heading truncation, omega scaling,
  OK/ERR spacing variants, ERR code map, CFG key order and per-key
  int-vs-3-decimal formatting, ID/VER/banner, EVT).
- New `tests/unit/test_bridge_routing.py` — `_FakeConn` double: per-verb
  envelope differential vs. `legacy_translate`, corr-id round trip, STOP
  clears the EVT watch, relay-verb swallow, unknown verb → typed ERR
  with no wire call, SET badkey local, GET fan-out targets, SNAP
  restores prior stream state, `_EvtWatcher` transitions (idle→busy→idle
  fires once, WAIT_BUSY timeout, supersede).
- New `tests/unit/test_bridge_pty_e2e.py` — real `os.openpty()`,
  `_FakeConn` upstream, client = `serial.Serial(slave_path)`:
  S/PING/HELLO/D + synthetic telemetry frames → `EVT done` appears;
  GET → one CFG line.
- Run `tests/unit` (host suite) and `tests/sim` (sanity — unaffected, no
  firmware files touched).
- Hardware bench gate (see Acceptance Criteria) — executed by the
  team-lead on the physical robot after ticket 008 lands; tracked here
  but not part of this ticket's own automated test run.

### Documentation updates

- `rogo proxy --help`'s own usage text.
- `docs/protocol-v3.md` (ticket 009) documents the proxy as the
  recommended path for any tool that still needs to speak text-v2
  against a now-binary-only firmware, including the PTY symlink path
  and the EVT-synthesis caveat (`EVT safety_stop` not synthesizable).
- Module docstrings in `proxy.py` state the single-client contract
  plainly.
