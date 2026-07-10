---
sprint: "097"
status: in-progress
revises: architecture-update-r1.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Architecture Update r2 -- Sprint 097: Pure-binary firmware + a rogo translator proxy replaces consumer-gated preservation

This is a **focused revision**, triggered by a direct stakeholder redirect
(Eric, 2026-07-10) reversing r1's consumer-gated conservatism. It does not
restate `architecture-update.md` or `architecture-update-r1.md` — read
both first. This document adds **Decision 9**, which supersedes r1's
Decision 8 in full. `architecture-update.md` and `architecture-update-r1.md`
are both preserved unmodified as the calibration record of what was
planned and why it changed twice. This revision was directly stakeholder-
approved in the redirect message itself — no separate architecture-review
or stakeholder-approval gate re-run was required or performed.

## The redirect

r1 found that essentially every motion/config/telemetry text verb had a
live, unmigrated production consumer (TestGUI's manual command panel and
connect-time telemetry probe, the MCP server's calibration push, `rogo`'s
own text subcommands, two calibration scripts, several bench/demo
scripts) and, per the issue's own "consumers migrated" rule, deferred
essentially all firmware text deletion to a future sprint gated on
`realign-host-tooling-to-gutted-four-verb-wire-surface.md`.

Eric's redirect rejects that trade explicitly: **don't preserve firmware
text for legacy consumers — gut the firmware text plane entirely, and
solve the legacy-consumer problem on the HOST side instead**, via a new
`rogo` translator proxy that speaks text-v2 to legacy clients and binary
to the firmware. This inverts r1's whole premise: instead of the firmware
carrying both planes until every consumer migrates, the firmware carries
ONE plane (binary), and text compatibility becomes a host-only, one-time
build (the proxy), not a per-consumer migration project gating the
firmware.

## Decision 9 -- Pure-binary firmware; a rogo translator proxy is the
host's own text-compatibility story; consumer migration is explicitly
deferred (not a firmware-deletion gate)

**Context**: r1's Decision 8 gated every firmware deletion on its
consumers having migrated to `NezhaProtocol`'s binary methods. That
gate is real but expensive: it makes 097's firmware-deletion scope
depend on migrating TestGUI's command panel, the MCP server's calibration
push, two raw-pyserial calibration scripts, and several bench/demo
scripts — work that belongs to a different, already-filed issue
(`realign-host-tooling-to-gutted-four-verb-wire-surface.md`) and is
substantial enough to be its own sprint. Eric's redirect observes that the
gate is solving the wrong problem: the CONSUMERS don't need the firmware
to speak text — they need SOMETHING to speak text to. A host-side proxy
satisfies that need without the firmware carrying a second protocol
implementation at all.

**Alternatives considered**:
1. **Keep r1's consumer-gated deferral** (do nothing further this
   sprint beyond `ParsedCommand`). *Rejected by the stakeholder*: leaves
   the firmware carrying two protocol implementations indefinitely, with
   no forcing function to ever finish the migration — exactly the
   "duplication this whole program exists to remove" the original issue
   was filed to eliminate.
2. **Migrate every named consumer to `NezhaProtocol` THIS sprint**, then
   gut. *Rejected*: this is `realign-host-tooling`'s full scope
   (TestGUI's command panel, `robot_mcp.py`, two raw-pyserial calibration
   scripts needing new transport-level binary capability, several bench
   scripts) plus new work discovered along the way (`SimTransport`'s
   ctypes ABI, the `encpose` schema question) — not a bounded addition to
   097, a second sprint's worth of work.
3. **Gut the firmware text plane unconditionally; build a rogo translator
   proxy as the host's own compatibility layer; defer WIRING each
   consumer to the proxy to a later effort.** *Chosen* (Eric's explicit
   redirect).

**Why the chosen alternative**: it decouples two previously-conflated
problems. "Does the firmware need to speak two protocols forever?" — no,
and alternative 3 ends that immediately. "Do all the legacy consumers
need to be rewired to a new endpoint before that's safe?" — the redirect's
answer is also no, PROVIDED a compatible endpoint (the proxy) exists for
them to be rewired to LATER, at whatever pace. The firmware gutting and
the consumer rewiring become independent, parallelizable tracks instead
of one blocking the other. This is a deliberate, explicit, stakeholder-
accepted trade: **every currently-live text consumer identified in r1
(TestGUI, `robot_mcp.py`, `rogo turn`/`sync-cal`, `calibration/linear.py`/
`angular.py`, `gamepad_teleop.py`, the bench demo scripts) WILL BREAK the
moment the firmware they point at is rebuilt with 097's changes, and stays
broken until each is individually rewired to point at the proxy** — that
rewiring is explicitly NOT this sprint's job ("worry about the consumer
later," Eric's own words). This is not an oversight; it is the accepted
cost of the trade, and every ticket below states it plainly rather than
implying continuity.

**Consequences**:

- **M5 (rogo Translator Proxy, supersedes "M5 -- rogo REPL Translator")**:
  boundary substantially expands. `rogo` gains a `proxy`/`serve` mode: it
  opens a Unix-domain socket (chosen over a FIFO — see rationale below),
  owns the REAL robot connection (`SerialConnection`, serial or relay),
  and for every connected client: reads a text-v2 line, translates it via
  an EXTENDED Legacy Verb Translator (`host/robot_radio/robot/
  legacy_translate.py`, built in ticket 002, now covering every verb a
  legacy client might still send — not just `timed`/`distance`), sends
  the resulting `CommandEnvelope` to the real robot, and translates the
  binary reply (`Ack`/`Error`/`DeviceId`/`Telemetry`/`ConfigSnapshot`)
  back to the matching text-v2 reply line for that client. Unsolicited
  binary `Telemetry` push frames (ticket 001's `_binary_tlm_queue`) are
  forwarded to every connected client as text `TLM ...` lines. Verbs with
  no binary arm at all (`R`/`TURN`/`G`) reply with a clear, typed error —
  the proxy cannot manufacture a binary capability the firmware never
  implements. `rogo binary <arm>` and every other existing `rogo`
  subcommand are unaffected — the proxy is an ADDITIVE mode, not a
  replacement for direct binary access.

  **Transport choice: Unix-domain socket**, not a FIFO. A FIFO is
  simplex and single-reader — `rogo`'s proxy needs bidirectional,
  multi-client-tolerant framing (a client writes a command line, reads a
  reply line, and unsolicited `TLM` pushes must reach every connected
  client independent of request/reply timing), which a `SOCK_STREAM`
  Unix-domain socket gives for free (standard `accept()`-per-client
  connection model, full duplex, OS-buffered) where a FIFO would need
  hand-rolled multiplexing (separate FIFOs per direction, no natural
  multi-client fan-out for push frames) to achieve the same. A Unix-domain
  socket is also directly compatible with `nc -U`/`socat`/Python's own
  `socket.AF_UNIX` for the acceptance test and for a human operator
  poking at it by hand, without inventing a bespoke framing protocol.

- **M6/M7/M8 (firmware retirement modules) lose their M2/M3 dependency
  edge.** r1 (and the original document) gated firmware deletion on host
  conversion having landed and been verified. Under Decision 9, the
  firmware's own text handlers are deleted UNCONDITIONALLY — their
  deletion no longer depends on any particular host consumer's migration
  state, because the proxy (M5), not `NezhaProtocol` directly, is now the
  host's answer to "what talks text to legacy clients." M6/M7/M8 depend
  only on the pre-existing, already-proven binary arms (095/096) — not on
  097's own host tickets at all. The dependency graph is now: M1 (queue
  fix) and M4 (translator) feed M2/M3/M5 independently of M6/M7/M8; M6/M7/
  M8 depend on nothing new this sprint.

- **Every family r1 preserved for "live consumer" reasons is now GUT
  target**: `S`/`D`/`T`/`RT`/`MOVE`/`MOVER`/`QLEN` (motion),
  `SET`/`GET` (config), `STREAM`/`SNAP` (telemetry), and — see the
  flagged rump question below — the liveness family. `otos_commands.cpp`/
  `pose_commands.cpp` (preserved for a DIFFERENT reason, sprint 098's
  transcription reference — original Decision 6, untouched by Decision 8
  OR 9) and `dev_commands.cpp` (096 Decision 3's boundary, a debug surface
  with no binary path planned) are **NOT** part of this redirect — Eric's
  message scopes the gut to `motion_commands.cpp`/`system_commands.cpp`
  (ticket 006), `config_commands.{h,cpp}` (ticket 007), and
  `telemetry_commands.{h,cpp}`/`tlm_frame.{h,cpp}` (ticket 008)
  specifically. They remain preserved, unchanged, for their own original
  reasons.

- **R/TURN/G and the stop-clause text grammar are now ALSO gut targets** —
  a necessary, forced consequence, not a separate decision. Ticket 006's
  own instruction to delete "the stop-clause text grammar" and to
  "preserve only what the BINARY path structurally reuses" cannot be
  satisfied while leaving `parseR`/`handleR`/`parseTURN`/`handleTURN`/
  `parseG`/`handleG` in place — they are the grammar's ONLY callers, and
  none of them structurally feed the binary path (unlike `Motion::Segment`/
  the `SegmentExecutor`, which live outside `motion_commands.cpp` entirely
  and are untouched regardless). r1's original rationale for preserving
  R/TURN/G (095's own r1 deferred the binary `motion` arm design
  indefinitely, so nothing PROVEN replaces them) is explicitly overridden
  by Decision 9's "no consumer-gating, no preservation" ethos — the
  absence of a binary replacement is no longer, on its own, a reason to
  keep dead code around. `bb.motionIn`/`Rt::MotionCommand`
  (`runtime/commands.h`/`blackboard.h`) become fully unreferenced plumbing
  once R/TURN/G's handlers are gone (they were already the only
  producers, and `Subsystems::Planner`, the only conceivable consumer, has
  been parked with zero wiring since 093/094) — cleaning up the
  Blackboard/queue declarations themselves is explicitly OUT of ticket
  006's file scope (`motion_commands.{h,cpp}`/`system_commands.cpp`/
  `command_types.h`/`command_router.cpp` only); flagged as Open Question 1
  below, not actioned here.

- **`handleTlm` (one-shot `TLM`) and `handleQlen`** — r1's Decision 7
  preservation (bench-diagnostic, no proven substitute) is explicitly
  overridden. Both are named for deletion in Eric's own ticket rewrites.

## Open decision: the text safety rump — flagged, not resolved silently

The ORIGINAL protocol-v3 issue (2026-07-09) said retain PING/ID/HELLO/
HELP/STOP for bare-terminal use. Eric's 2026-07-10 redirect says "gut
everything" but simultaneously says "do NOT gut STOP without explicit
confirmation" and defaults to a 2-verb rump (STOP + PING). Both
instructions are Eric's own, in the same message, and are in tension with
each other and with the original 5-verb rump — this is exactly the kind
of conflict the redirect itself asks to be flagged, not silently resolved
either way.

**This document defaults to a 3-verb rump — STOP, PING, and HELLO —
one verb wider than Eric's stated 2-verb default, for a reason grepped
from the firmware's own source, not inferred:**

`source/subsystems/communicator.cpp`'s own boot-announcement comment says,
verbatim: *"Radio is fire-and-forget: a missed boot banner (no relay
listening yet) is not a failure -- **HELLO re-requests it**, and HELLO's
handler uses this same `formatDeviceAnnouncement()` so the banner matches
byte-for-byte."* `host/robot_radio/io/serial_conn.py`'s own `connect()`
docstring confirms the host side depends on exactly this: `_banner_classify()`
sends `HELLO` repeatedly (up to ~10 times, ~200ms apart) specifically to
catch a `DEVICE:<ROLE>:...` banner it may have missed — the RECONNECT
case (no fresh boot event fires a new automatic banner) has NO OTHER way
to obtain that banner once `HELLO` the command stops existing. Deleting
`HELLO` does not merely remove a diagnostic convenience the way `ECHO`/
`VER`/`ID`/`HELP` do — it removes the one re-request path
`SerialConnection.connect()`'s OWN documented handshake protocol
structurally depends on for any reconnect. This is a materially different
risk class than the other four liveness verbs, found by reading the
actual connection-handshake code, not assumed.

**Flagged for Eric to confirm or override, exactly as the STOP question
was flagged**:
- Keep the 3-verb rump (STOP + PING + HELLO) — this document's default,
  pending confirmation.
- OR: confirm the 2-verb rump (STOP + PING only) and accept that
  `SerialConnection.connect()`'s reconnect path (used by literally every
  host tool that isn't the proxy, INCLUDING the proxy itself unless it is
  separately hardened) loses its documented re-request mechanism and must
  rely solely on catching the one automatic boot-time banner within
  whatever window it happens to be listening — a real regression in
  connection reliability, not just a lost diagnostic.
- OR: confirm gutting STOP too (truly zero rump), per the redirect's own
  most literal "gut everything" reading.

`ID`, `HELP`, `VER`, `ECHO` have no equivalent structural finding — they
are gut targets under EITHER rump size, per Eric's default.

**Ticket 006 implements the 3-verb default (STOP/PING/HELLO preserved;
ID/HELP/VER/ECHO gutted) and states this flag explicitly in its own
Description**, so a human reviewing the diff sees the open question
before it ships, not after.

## Open Questions

1. **`bb.motionIn`/`Rt::MotionCommand` become fully dead Blackboard
   plumbing** once R/TURN/G's handlers are deleted (they were the only
   producers; `Subsystems::Planner`, the only conceivable consumer, has
   been parked since 093/094). Cleaning up `blackboard.h`/
   `runtime/commands.h` themselves is out of ticket 006's file scope this
   sprint — flagged for a future cleanup pass, not actioned here.
2. **The rump size** (2 vs. 3 vs. 0 verbs) — see above. Ticket 006
   implements the 3-verb default; if Eric confirms 2 or 0, ticket 006's
   own diff needs one more small edit (delete `HELLO`'s registration and,
   for the 0-verb reading, `STOP`'s) before it ships.
3. **Consumer rewiring to the proxy** is explicitly deferred (not this
   sprint). `realign-host-tooling-to-gutted-four-verb-wire-surface.md`
   should be updated (a follow-up to this revision, not done inline here
   since Eric's redirect did not ask for it the way the r1 revision did)
   to note the proxy now exists as the rewiring TARGET, changing that
   issue's scope from "migrate each consumer to `NezhaProtocol` directly"
   to "point each consumer at the proxy socket" — a smaller, more
   mechanical task than before.
4. **`_legacy_tlm_text.py`** (the frozen bridge module ticket 003 built
   for the four `SerialConnection`-unreachable telemetry consumers) is
   harmless dead code under this redirect — those four consumers will
   break anyway once ticket 008 lands (their underlying transports don't
   go through `NezhaProtocol`/the proxy either), so the bridge module
   neither helps nor hurts. Left in place; noted for a future cleanup pass
   alongside Open Question 1.

## Status

This revision is directly stakeholder-approved (Eric's redirect message
itself constitutes the approval — no separate gate re-run performed, per
explicit instruction). Tickets 004/005/006/007/008/009/010 are rewritten
to carry out Decision 9; none have been executed. Tickets 001/002/003
remain unaffected (already done, and 002's Legacy Verb Translator is
extended, not replaced, by ticket 004).
