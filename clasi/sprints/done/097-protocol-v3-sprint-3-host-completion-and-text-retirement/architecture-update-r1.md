---
sprint: "097"
status: in-progress
revises: architecture-update.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Architecture Update r1 -- Sprint 097: Text retirement is consumer-gated; full firmware text deletion deferred to realign-host-tooling

This is a **focused revision**, triggered by a structural discovery made
during ticket 003's implementation and confirmed by the team-lead's own
follow-up grep sweep. It does not restate `architecture-update.md` — read
that document first for the full Sprint 097 design (Steps 1-7, Decisions
1-7). This document adds **Decision 8**, which supersedes part of the
original's Step 5 ("What Changed"/"Impact on Existing Components") and
Decisions 1/5/7 insofar as they assumed the motion/config/telemetry text
families would be fully deletable once host parity landed. It is now the
active planning artifact for tickets 006/007/008's actual scope.
`architecture-update.md` itself is preserved unmodified as the calibration
record of what was originally planned.

## The discovery

Ticket 003 (`NezhaProtocol` telemetry conversion + 9-file consumer sweep)
fully swept 6 of the 9 `parse_tlm()` consumers found by the original
document's Step 1 grep, converted `stream()`/`snap()` to binary, and
deleted `parse_tlm`/`parse_cfg`. It could NOT sweep three files —
`calibration/linear.py`, `calibration/angular.py`,
`calibration/fit_sim_error_model.py` — plus `testgui/transport.py`'s
`SimTransport` half, for two independent, structural reasons documented in
the ticket's own Resolution section:

1. **No `SerialConnection` in play at all.** `calibration/linear.py`/
   `angular.py` talk to the robot over `calibration/_conn_helpers.py`'s
   `RelaySerial`/`DirectSerial` — a raw pyserial wrapper chosen
   deliberately for relay-handshake/DTR timing control
   `SerialConnection` doesn't expose. `SimTransport` talks over
   `robot_radio.io.sim_conn.SimConnection`, a ctypes ABI. Neither
   transport owns (or can cheaply be given) ticket 001's
   `_binary_tlm_queue`.
2. **The data itself is gone from the binary wire schema.**
   `fit_sim_error_model.py`'s `_residual_vector()` structurally depends on
   `TLMFrame.encpose`, which `telemetry.proto`'s `Telemetry` message never
   carries (096-001 Decision 6 trimmed it to fit the 186-byte envelope
   budget). No transport fix restores this — the field cannot be adapted
   from the binary plane at all as currently schemed.

Ticket 003 preserved all four via a new, frozen, clearly-labeled bridge
module (`host/robot_radio/robot/_legacy_tlm_text.py`,
`parse_historical_tlm_line()` — a field-for-field copy of the deleted
`parse_tlm()`, deliberately not resurrected as a general-purpose
replacement) and left its own ticket `status: in-progress` rather than
silently declaring the nine-file sweep complete. `tests/unit` (93),
`tests/sim` (600), and `tests/testgui` (16 pre-existing failures,
identical A/B) are all unaffected.

**Team-lead's own follow-up grep, run before revising tickets 006-008,
found the SAME live-consumer pattern on the COMMAND-SEND side** — a
different half of the wire than ticket 003's telemetry-receive scope, and
not something ticket 003 was ever asked to check:

- `testgui/commands.py`'s `COMMANDS` table (S/T/D/R/TURN/RT/G) +
  `build_wire_string()`, wired into TestGUI's manual command panel
  (`testgui/__main__.py`): sends raw text lines via `transport.command()`
  for ANY connected transport (Serial/Relay/Sim alike) — completely
  bypassing `NezhaProtocol`. TestGUI also sends a hardcoded `"STREAM 50"`
  on every connect (`testgui/__main__.py`).
- `host/robot_radio/io/robot_mcp.py`: `push_calibration(_robot._proto,
  _config)` — since `NezhaProtocol` has no `push_calibration` method,
  `calibration/push.py`'s own documented fallback always takes the
  "extract `_conn`, send raw text `SET`" path. **This is the MCP server**,
  one of the sprint's own explicitly-protected consumers.
  `robot_mcp.py`.
- `host/robot_radio/io/cli.py`: `cmd_turn`'s DEFAULT (non-`--open-loop`)
  path sends raw text `RT <cdeg> #<corr>` via `proto.send()` directly (not
  through M4's translator); `_push_calibration()` (a live, called
  function, `rogo sync-cal`) sends raw text `SET`/`OI`/`OL`/`OA`.
- `calibration/push.py`, `host/calibrate_verify.py`: raw text `SET`/`GET`.
- `tests/bench/gamepad_teleop.py`: raw text `MOVER`.
- `tests/bench/dtr_drive_demo.py`, `random_segment_demo.py`: raw text
  `MOVE`.

Every verb the original document's Decision 1/6/7 (and Step 5's "What
Changed") assumed tickets 006/007/008 could delete — `S`/`D`/`T`/`RT`/
`MOVE`/`MOVER` (motion), `SET`/`GET` (config), `STREAM`/`SNAP` (telemetry),
and `VER` (TestGUI's own connect-time firmware-version check,
`testgui/__main__.py`, also sends raw text `VER`) — has at least one live,
non-test, production consumer that has not migrated to the binary plane.
`ECHO`'s only found consumer is a bench protocol-verification script
(`tests/bench/comms_plane_verify.py`); no production tool was found to
depend on it, but given the density of findings elsewhere, it is grouped
with the conservative preservation below rather than singled out as safe.

This is precisely the scenario the issue's own rule anticipates and the
team-lead's dispatch brief explicitly warned about: **"a text family is
deleted only after its binary replacement is bench-proven AND its
consumers migrated."** The binary replacements ARE proven (095/096). Their
consumers are NOT migrated — that migration is the separate
`realign-host-tooling-to-gutted-four-verb-wire-surface.md` issue's own,
already-filed scope, not something 097 can absorb without becoming a
different, much larger sprint.

## Decision 8 -- Text retirement is consumer-gated; firmware text deletion
of live-consumer families is deferred to `realign-host-tooling`, not
performed in 097

**Context**: The original document's Decisions 1, 5, and 7 (and Step 5's
"What Changed"/"Impact on Existing Components") planned tickets 006/007/008
to delete the motion (`S`/`D`/`T`/`RT`/`MOVE`/`MOVER`), liveness
(`ECHO`/`VER`), and telemetry (`STREAM`/`SNAP`) text families once
`NezhaProtocol`'s own host-reachable methods (tickets 002/003) proved
parity. That framing implicitly assumed `NezhaProtocol` is the ONLY host
path to these verbs. The discovery above disproves that assumption: at
least six other code paths — TestGUI's manual command panel and
connect-time STREAM, the MCP server's calibration push, `rogo`'s `turn`/
`sync-cal` subcommands, two calibration scripts, and several bench/demo
scripts — reach the wire directly, independent of `NezhaProtocol`, and
remain text-only.

**Alternatives considered**:
1. **Expand 097 to also migrate every one of these newly-found consumers
   to the binary plane**, then proceed with the original deletion plan.
   *Rejected*: this is not a bounded addition — it is the ENTIRE scope of
   the separate, already-filed `realign-host-tooling-to-gutted-four-verb-
   wire-surface.md` issue (TestGUI's command panel, `robot_mcp.py`'s
   calibration push, `rogo`'s remaining text subcommands), plus new scope
   that issue doesn't yet cover (`calibration/linear.py`/`angular.py`'s
   raw-pyserial transport, `SimTransport`'s ctypes ABI, the `encpose`
   schema question). Absorbing it here would silently more-than-double
   this sprint's real size and blow past `sprint.md`'s own "no new binary
   functionality" scope boundary (migrating `calibration/linear.py`/
   `angular.py` would need SOME transport-level binary capability neither
   `RelaySerial` nor `DirectSerial` currently has).
2. **Delete the text families anyway**, accepting that TestGUI's command
   panel, the MCP server's calibration push, `rogo turn`, and the
   calibration/bench scripts listed above will break. *Rejected
   outright*: this directly violates the issue's own stated rule and the
   sprint's own explicit "TestGUI, gamepad teleop, bench scripts, and the
   MCP server change zero call sites" success criterion — breaking the
   MCP server's calibration push and TestGUI's manual command panel is not
   a defensible reading of "zero call sites," it is the opposite of it.
3. **Defer deletion of every family with a live, unmigrated consumer;
   delete only what is confirmed to have NO live consumer anywhere in the
   tree.** *Chosen.*

**Why the chosen alternative**: it is the only option consistent with the
issue's own stated deletion rule, the sprint's own success criteria, and
the team-lead's explicit conservative-deletion posture already applied
once in this document (Decision 5, R/TURN/G). It keeps 097 bounded to what
it can actually verify and land safely, and it does not foreclose the real
goal — full text retirement — it defers the REMAINING part of it to the
issue that already owns "realign host tooling to the current wire
surface," which is the correct owner: that issue already scopes "TestGUI /
robot_radio still drive removed verbs" and "the config family... decide
with the stakeholder whether it comes back on the wire" as its own
mandate.

**Consequences** (concrete re-scope of tickets 006/007/008, detailed in
each ticket's own revised file):

- **Ticket 006** (motion + liveness): the ONLY deletion with zero live
  consumers found anywhere is `ParsedCommand` (`command_types.h`,
  grep-confirmed zero references both originally and now). `S`/`D`/`T`/
  `RT`/`MOVE`/`MOVER`/`ECHO`/`VER` are ALL preserved this sprint — every
  one has at least one live, non-test consumer (TestGUI's command panel:
  `S`/`T`/`D`/`RT`; calibration scripts: `D`/`T`; `rogo turn`: `RT`;
  `gamepad_teleop.py`: `MOVER`; bench demos: `MOVE`; TestGUI's connect-time
  check: `VER`; `ECHO` grouped in conservatively despite its weaker
  evidence). `R`/`TURN`/`G` and the shared stop-clause grammar remain
  preserved per the original Decision 5 (unchanged, still correct, now
  doubly justified — they too are in TestGUI's command panel).
- **Ticket 007** (config): `SET`/`GET` are preserved in full — live via
  the MCP server's `push_calibration`, `rogo sync-cal`
  (`_push_calibration`), `calibration/push.py`, `calibrate_verify.py`'s
  `GET`, and TestGUI's own test suite. `config_commands.{h,cpp}` is NOT
  deleted this sprint.
- **Ticket 008** (telemetry): `STREAM`/`SNAP` text handlers and
  `Telemetry::buildTlmFrame()` are preserved in full — live via TestGUI's
  connect-time `STREAM 50` and `calibration/linear.py`/`angular.py`'s
  `SNAP`/`STREAM`. Already the team-lead's own expectation before this
  revision; now confirmed with direct evidence rather than inferred.
- **Ticket 010** (closure): the flash-reduction expectation drops from the
  issue's own "15-30 KB reclaimed" estimate to whatever a single
  zero-reference struct deletion (`ParsedCommand`) is worth — effectively
  negligible this sprint. The real flash win the issue was written around
  remains achievable, but only after `realign-host-tooling` migrates the
  remaining consumers and a FUTURE sprint (not 097) completes the
  deletion. Ticket 010's report must say this plainly, not imply the
  original estimate was met.
- **`realign-host-tooling-to-gutted-four-verb-wire-surface.md`** is
  updated (this revision's own task, done directly in the issue file) to
  explicitly own: migrating TestGUI's command panel + connect-time
  `STREAM`, `robot_mcp.py`'s calibration push, `rogo`'s remaining text
  subcommands (`turn`, `sync-cal`), `calibration/linear.py`/`angular.py`'s
  raw-pyserial transport, and `SimTransport`'s ctypes transport, to the
  binary plane — and noting that FULL firmware text retirement (097's
  original, now-deferred goal for these families) is gated on that
  migration landing first.
- **The protocol-v3 issue itself**
  (`protocol-v3-schema-driven-binary-command-plane-protobuf.md`) gets a
  short note (this revision's own task) recording that Sprint 3's text
  retirement is PARTIAL: host completion is done; firmware text retirement
  of the motion/config/telemetry families is deferred pending
  `realign-host-tooling`.

No change to Decisions 1-4, 6 (M1-M5's host-side design, `otos_commands.
cpp`/`pose_commands.cpp` preservation, `handleTlm`/`QLEN` preservation) —
those stand exactly as originally decided. Decision 5 (R/TURN/G
preservation) stands and is reinforced, not revised. Decision 7
(bench-diagnostic rump) stands unchanged.

## Status

This revision is the team-lead's own resolution call, recorded here per
the Exception Protocol's spirit (ticket 003 correctly surfaced a
structural finding rather than silently working around it or silently
declaring success). No further architecture self-review round is required
for this scope of change — Decision 8 narrows tickets 006/007/008's own
deletion scope without touching M1-M10's boundaries, the dependency graph,
or any other module's design; the original document's Quality Checks still
hold. Tickets 003, 006, 007, 008, and 010 are revised separately to carry
out the concrete changes above; `realign-host-tooling-to-gutted-four-verb-
wire-surface.md` and the protocol-v3 issue are updated directly.
