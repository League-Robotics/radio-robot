---
id: '004'
title: 'rogo translator proxy: text-v2 socket server fronting the real binary robot connection'
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

# rogo translator proxy: text-v2 socket server fronting the real binary robot connection

## Description

**REWRITTEN — see `architecture-update-r2.md` Decision 9 (supersedes r1's
Decision 8 and this ticket's own original "rogo REPL translator" scope in
full).** The original plan (extend `rogo send` to translate one typed
line at a time for a human at a terminal) is superseded by a bigger,
different shape: `rogo` gains a **`proxy`/`serve` mode** that is a
persistent, standalone text-v2-speaking server fronting the real,
binary-only robot connection — the host's own answer to "how does a
legacy text client keep working once the firmware only speaks binary"
(tickets 006/007/008 gut the firmware text plane unconditionally this
sprint; this proxy is what makes that safe to do without migrating every
legacy consumer first).

**What it does**:

1. Opens a **Unix-domain socket** (`SOCK_STREAM`) at a well-known path
   (e.g. `/tmp/rogo-proxy-<port-or-robot-id>.sock`, or a `--socket-path`
   flag). Chosen over a FIFO: the proxy needs bidirectional, OS-buffered,
   multi-client-tolerant framing (a client writes a command line, reads a
   reply line, and unsolicited `TLM` pushes must reach every connected
   client regardless of request/reply timing) — a `SOCK_STREAM` socket's
   standard `accept()`-per-client model gives this for free; a FIFO is
   simplex/single-reader and would need hand-rolled multiplexing to match.
   A Unix-domain socket is also directly pokeable with `nc -U`/`socat`/
   Python's `socket.AF_UNIX`, useful for the acceptance test and for a
   human operator.
2. Owns the REAL robot connection (`SerialConnection`, serial or relay) —
   exactly one instance, shared across all connected proxy clients.
3. For each connected client, reads text-v2 lines (the SAME grammar a
   human or legacy tool would type/send today: `S 200 200`, `D 200 200
   300`, `SET tw=128`, `STREAM 50`, `PING`, etc.).
4. Translates each line via the **EXTENDED Legacy Verb Translator**
   (`host/robot_radio/robot/legacy_translate.py`, built in ticket 002) —
   extend it in THIS ticket to cover EVERY text verb a legacy client might
   still send, not just `timed`/`distance`: `S`, `D`, `T`, `RT`, `MOVE`,
   `MOVER`, `ECHO`, `PING`, `ID`, `HELLO`, `HELP`, `STOP`, `SET`, `GET`,
   `STREAM`, `SNAP` — each maps to its matching `CommandEnvelope` oneof
   arm (or, for `HELLO`/`HELP`, to whatever the retained text rump ends up
   being, per `architecture-update-r2.md`'s flagged open rump question —
   coordinate with ticket 006's final rump size). Verbs with NO binary arm
   at all (`R`, `TURN`, `G`) reply with a clear, typed error (e.g. `ERR
   unsupported R` ) — the proxy cannot manufacture a binary capability the
   firmware never implements; do not silently drop or hang on these.
5. Sends the resulting `CommandEnvelope` to the real robot via
   `SerialConnection.send_envelope()`.
6. Translates the binary reply (`Ack`/`Error`/`DeviceId`/`Telemetry`/
   `ConfigSnapshot`) back into the equivalent text-v2 reply line
   (`OK ...`/`ERR ...`/`ID ...`/`TLM ...`/`CFG ...`) for that client —
   the REVERSE of step 4's mapping.
7. Unsolicited binary `Telemetry` push frames (ticket 001's
   `_binary_tlm_queue`) are forwarded to EVERY connected client as text
   `TLM ...` lines, independent of which client (if any) armed the
   `stream` — matching the pre-097 firmware behavior where any client on
   the wire could see STREAM output.

`rogo binary <arm>` (095/096) and every other existing `rogo` subcommand
are UNAFFECTED — `proxy`/`serve` is a new, additive mode, not a
replacement.

**Explicitly deferred, NOT this ticket's job** (Eric: "worry about the
consumer later"): pointing TestGUI, `robot_mcp.py`, `calibration/
linear.py`/`angular.py`, `gamepad_teleop.py`, or any bench script AT the
proxy socket. This ticket only BUILDS the proxy and PROVES it translates
correctly with a test client. Wiring real consumers to it is a follow-up
(tracked by `realign-host-tooling-to-gutted-four-verb-wire-surface.md`,
whose scope this revision narrows from "migrate to `NezhaProtocol`
directly" to "point at the proxy socket").

## Acceptance Criteria

- [ ] `rogo proxy`/`rogo serve` (name at implementer's discretion, matching
      existing `rogo` subcommand naming conventions) opens a Unix-domain
      socket and accepts client connections.
- [ ] `legacy_translate.py` covers EVERY verb listed in step 4 above,
      mapping each to its `CommandEnvelope` oneof arm; `R`/`TURN`/`G`
      produce a clear, typed error reply, not a hang or silent drop.
- [ ] A TEST CLIENT (new host test) connects to the socket, writes a text
      line for EACH covered verb, and receives the correct translated
      text reply, while the proxy itself speaks ONLY binary
      (`*B<base64>`) to a fake/sim robot connection underneath — verified
      by asserting on the bytes the proxy's OWN `SerialConnection` writes,
      not just the client-visible reply.
- [ ] A test exercises the unsolicited `TLM` forwarding path: arm binary
      streaming on the underlying (fake/sim) connection, confirm a
      connected proxy client receives text `TLM ...` lines without having
      sent `STREAM` itself.
- [ ] `rogo binary <arm>` and every other existing `rogo` subcommand are
      byte-for-byte unaffected by this ticket's diff.
- [ ] `tests/sim` stays green (host-only ticket; sanity check).
- [ ] `tests/unit` is green, including the new proxy tests.
- [ ] Completion notes state plainly: consumer rewiring (TestGUI, MCP,
      calibration scripts, bench/demo scripts) to point at this proxy is
      OUT of this ticket's scope, deferred to
      `realign-host-tooling-to-gutted-four-verb-wire-surface.md`.

## Implementation Plan

### Approach

1. Extend `host/robot_radio/robot/legacy_translate.py` (ticket 002) with
   translator functions for every verb in step 4 above, reusing/extending
   whatever parsing helpers ticket 002 already built for `timed`/
   `distance`. For verbs with a direct 1:1 binary arm (`PING`/`ECHO`/`ID`/
   `STOP`/`SET`/`GET`/`STREAM`/`SNAP`), the translation is a thin
   text-token-to-message-field mapping; for `S`/`D`/`T`/`RT`/`MOVE`/
   `MOVER`, reuse the sign/distance/segment-shape logic ticket 002 already
   ported from `BodyKinematics::forward()`/the firmware's own parse
   functions.
2. Build the reverse (binary-reply-to-text-line) translator alongside it —
   symmetric to step 1, driven by the `ReplyEnvelope`'s populated oneof
   body.
3. Implement the socket server: `socket.AF_UNIX`/`SOCK_STREAM`, one
   accept-loop thread (or asyncio, implementer's choice — document
   whichever), one reader per connected client, a shared writer path for
   `TLM` fan-out to all clients.
4. Wire it into `cli.py` as a new `rogo proxy`/`serve` subcommand.
5. Write the test client + fake/sim-robot-backed tests described in
   Acceptance Criteria.

### Files to modify

- `host/robot_radio/robot/legacy_translate.py` — extended translator
  coverage (both directions: text-to-envelope and envelope-to-text).
- `host/robot_radio/io/cli.py` — new `proxy`/`serve` subcommand.
- New file(s) for the socket server implementation itself (e.g.
  `host/robot_radio/io/proxy.py` or similar — implementer's naming
  choice, follow the project's existing module-per-concern convention in
  `host/robot_radio/io/`).

### Testing plan

- New host unit/integration tests: a test client against the proxy
  socket, a fake/sim robot underneath, covering every translated verb
  both directions, the `R`/`TURN`/`G` error case, and the `TLM` fan-out
  case.
- Run `tests/unit` (host suite) and `tests/sim` (sanity — unaffected, no
  firmware files touched).

### Documentation updates

- `rogo proxy --help`'s own usage text. `docs/protocol-v3.md` (ticket
  009) documents the proxy as the recommended path for any tool that
  still needs to speak text-v2 against a now-binary-only firmware.
