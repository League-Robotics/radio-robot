---
status: pending
---

# rogo Translator Proxy — text-v2 ⇄ binary bridge on a PTY

## Context

Protocol v3 replaced the firmware's hand-rolled text command plane with a schema-driven binary envelope plane (`*B<base64(protobuf)>`). Sprint 097's r2 redirect (Decision 9, stakeholder-approved 2026-07-10) guts the firmware text plane **unconditionally** — legacy text consumers (TestGUI, MCP server calibration push, calibration scripts, gamepad teleop, bench demos) are handled on the **host** side instead: a standing `rogo` translator proxy speaks protocol-v2 text to legacy clients and binary to the robot. Consumers are never individually migrated to binary — they just repoint their serial-port path at the proxy (deferred, owned by `realign-host-tooling-to-gutted-four-verb-wire-surface.md`).

This plan is the implementation spec for the rewritten **ticket 097-004** ("rogo translator proxy"), with **two stakeholder decisions made today (2026-07-10, this session) that revise the ticket as written**:

1. **Transport: PTY, not Unix socket.** Ticket 004/r2 chose `AF_UNIX`. Eric chose PTY after seeing the trade-off: every legacy consumer opens its port with a plain device path (`serial.Serial(path)` or `SerialConnection(port)`), so a PTY is a **zero-code-change drop-in** — a socket would force a code change into every consumer, recreating the migration problem the proxy exists to avoid. Cost accepted: single-client (documented), no `nc -U` pokability (use `screen <pty>` instead). The routing core is transport-agnostic, so an additive socket listener later is cheap if multi-client need materializes.
2. **EVT synthesis included.** Calibration scripts block on `EVT done D/T`; current firmware emits **no EVT at all** (verified: `emitEvent` has zero producers). The proxy synthesizes `EVT done <verb>` by watching the binary `Telemetry.active` flag.

Also decided: full v2 verb surface; the proxy **replaces** per-consumer binary migration permanently.

**Verified facts this plan corrects from earlier artifacts** (do not re-litigate):
- Ticket 097-004's old REPL helpers (`cli._tokenize_send_line` etc.) **do not exist** — only the test file `tests/unit/test_cli_send_translator.py` is committed and is **currently RED** on the branch. This plan makes it green.
- EVT must key off `Telemetry.active` (field 22, unconditionally present in every binary frame — `source/telemetry/tlm_frame.cpp:275`), **not** the `mode` char — nothing writes `bb.planner.mode`; mode is always `I`.
- PTY + pyserial verified empirically on this Mac: opens cleanly, DTR ioctl swallowed by pyserial — **no SerialConnection guard needed**. `tcdrain` deadlocks if the master isn't being read → invariant: the proxy never stops reading the master. Client close does not EOF the master on macOS (blocks until reopen); Linux raises EIO → catch `OSError`, sleep-retry.
- HELLO passthrough can't work: the upstream `SerialConnection` reader drops `DEVICE:` lines. Proxy answers HELLO **locally** from a cached binary `DeviceId`.
- Binary `Telemetry` carries only `now/mode/seq/enc/vel/cmd/pose/otos(+otos_connected)/twist` + the bench block (`acc/active/conn/glitch/ts`). `encpose/line/color/wedge/otos_health/ekf_rej` are **not on the binary wire** — the proxy cannot emit them (accepted; their consumers are broken by the gut regardless).
- Binary POSE/OTOS arms reply `ERR_UNIMPLEMENTED` today (sprint 098 lands them).

## Architecture

```
legacy client (pyserial / SerialConnection, unchanged code)
      │  text-v2 lines                 ~/.rogo/robot-pty (symlink → PTY slave)
      ▼
┌─ rogo proxy ────────────────────────────────────────────────┐
│ pty-reader thread: line-split → route:                      │
│   local (HELLO/HELP/!*/unknown) ── fake-ack / typed ERR     │
│   binary (everything else) ── legacy_verbs → send_envelope  │
│   → legacy_render → write PTY                               │
│ tlm-pump thread: read_binary_tlm → EvtWatcher(active)       │
│   → synthesized EVT done / text TLM lines → write PTY       │
└──────────────┬───────────────────────────────────────────────┘
               │  *B<base64> only          one SerialConnection
               ▼
        robot (serial or radio relay) — binary-only firmware
```

Upstream is **binary-only**, so the proxy works against today's dual-stack firmware AND the post-gut firmware — it can be built and bench-proven before/parallel to gut tickets 006–008 (its only deps, 002/003, are done).

## Files

| File | Action | Contents |
|---|---|---|
| `host/robot_radio/robot/legacy_verbs.py` | NEW (pure) | `tokenize_send_line(raw) -> (verb, positional, kv)` mirroring firmware `parseTokens`/`parseKV`; `split_corr_id(raw)`; `kvfloat`; `envelope_for_drive/timed/distance/rt/move/mover/echo(pos, kv) -> CommandEnvelope` (thin wrappers over the existing `legacy_translate` builders); dispatch tables `BINARY_DISPATCH`, `RUMP_VERBS`, `PROTOCOL_VERBS`; `decode_reply_body(reply)` pretty-printer |
| `host/robot_radio/robot/legacy_render.py` | NEW (pure) | reverse direction, every renderer transcribed from the firmware format strings **with file:line citations**: `render_tlm_line`, `render_ok`/`render_err` (4 spacing variants, `command_processor.cpp:284-322`), per-verb `ok_body_for` table, `ERR_CODE_TEXT` map, `render_id_line`/`render_ver_body`/`render_device_banner`, `render_cfg_line` (kAllKeys order, per-key int vs 3-decimal `formatFixed` formats), `render_evt_done` |
| `host/robot_radio/io/proxy.py` | NEW | `ProtocolBridge` class: PTY lifecycle, pty-reader + tlm-pump threads, `_handle_client_line` routing, `_write_pty` (non-blocking + drop policy), `_EvtWatcher` (pure state machine, separately testable) |
| `host/robot_radio/io/cli.py` | MODIFIED | `cmd_proxy` + `sub.add_parser("proxy", ...)` + `commands["proxy"]` entry (dict at ~line 1913); plus thin `cmd_send` + module aliases (`_tokenize_send_line = legacy_verbs.tokenize_send_line`, `_SEND_RUMP_VERBS`, `_decode_reply_body`, …) — **makes the committed-red `test_cli_send_translator.py` green** and delivers the original `rogo send` REPL for free |
| ticket `097-004` file | REVISED first | transport section socket→PTY (cite today's stakeholder decision), add EVT-synthesis acceptance criterion, single-client acceptance, test client = pyserial on the PTY (not `nc -U`) |

Reused, not rewritten: `legacy_translate.py` (all six motion builders), `SerialConnection.send_envelope/send/read_binary_tlm/drain_binary_tlm`, `protocol.py`'s `_TARGET_FOR_KEY`/`_ALL_GET_KEYS`/`_read_config_snapshot_value` (GET fan-out/fan-in) and `_DRIVE_MODE_CHAR`, `connection.make_robot` (port resolution).

## Verb routing (client line → action), against the post-gut wire surface

| Client sends | Route | Rendered reply |
|---|---|---|
| `S l r` | binary `{drive}` | `OK drive l=<l> r=<r> #id` |
| `T l r ms` / `D l r mm` | binary `{segment}`; arm EVT watch | `OK drive l= r= ms=/mm=` |
| `RT cdeg` | binary `{segment}`; arm EVT watch | `OK rt rot=` |
| `MOVE …` | binary `{segment}`; arm EVT watch | `OK move dist= dir= fh= q=<Ack.q> rem=<int(Ack.rem)>` |
| `MOVER …` | binary `{replace}` (no EVT watch — teleop polls `q=`) | `OK mover t= v= w= q=<Ack.q>` (gamepad regex `OK mover .*q=(\d+)` satisfied) |
| `STOP` | binary `{stop}`; clears pending EVT watch silently | `OK stop` |
| `ECHO text` | binary `{echo}` | `OK echo <payload>` |
| `PING` | binary `{ping}` | `OK pong t=<Ack.t>` |
| `ID` / `VER` | binary `{id}` | `ID model= name= serial= fw= proto=` / `OK ver fw= proto=` |
| `HELLO` | **local** (cached DeviceId fetched once at startup) | `DEVICE:NEZHA2:robot:<name>:<serial>` |
| `HELP` | **local** | short proxy help text (firmware HELP is gutted) |
| `SET k=v …` | binary fan-out: one `ConfigDelta` per target (reuse protocol.py key maps); unknown key → local `ERR badkey <k>` before any wire traffic | `OK set <k=v …> #id` |
| `GET [keys]` | binary fan-out/fan-in: one `ConfigGet` per distinct target, merge | ONE `CFG k=v … #id` line, kAllKeys order, firmware per-key formats |
| `STREAM n` | binary `{stream: StreamControl{binary:true, period:n}}`; sets client-stream flag | `OK stream period=<0 or max(20,n)>` (proxy computes the clamp) |
| `SNAP` | binary arm-wait-disarm (restore prior stream state — do NOT blindly `stream(0)`) | exactly one bare `TLM …` line, no OK |
| `TLM` (one-shot) | binary snap → render bench body (binary carries `acc/active/conn/glitch/ts` unconditionally) | `OK tlm enc=… acc=… conn=… #id` |
| `SI/ZERO/OI/OZ/OR/OP/OV/OL/OA` | binary `{pose}`/`{otos}` behind a module flag `_POSE_OTOS_BINARY=False` → until 098 lands: local `ERR unsupported <verb>` | flip flag when 098's arms go live |
| `QLEN`, `G`, `R`, `TURN`, `GRIP`, `DEV *`, unknown | **local** typed `ERR unsupported <verb>` (ticket requirement: no hang, no silent drop) | |
| `!MODE/!CG/!P/!ECHO/!GO/?` (relay-control from clients that think they face a dongle) | **local** swallow → `# ok` comment reply (verified: `RelaySerial.configure()` never checks these) | |
| `+` (keepalive) | forward `conn.send_fast("+")` (feeds firmware watchdog) | none |
| `*B…` (binary-native client) | local `ERR unsupported proxy-is-text-only` (binary tools use the real port) | |

ERR mapping: `Error{code, field}` → `ERR <text-code> <field-name> #id` with `{UNKNOWN:"unknown", BADARG:"badarg", RANGE:"range", FULL:"full", DECODE:"badarg", UNIMPLEMENTED:"unsupported", OVERSIZE:"unsupported"}`; field number → name via `CommandEnvelope.DESCRIPTOR.fields_by_number`.

Corr-id: client's trailing `#<digits>` is stripped and saved; `send_envelope()` uses its own counter internally; the proxy re-attaches the client's id to the rendered reply. Single in-flight command (pty-reader processes lines serially) makes this a local variable — no map. At 20 Hz MOVER the blocking round trip (~10 ms wire at 115200) fits the budget; the same cadence already runs in gamepad flow control.

## EVT synthesis (`_EvtWatcher`, owned by the tlm-pump thread)

- On Ack for T/D/RT/MOVE: `_pending = (verb, client_corr_id)`, state `WAIT_BUSY`, 2 s cap. If no client stream is armed, arm an internal upstream stream at `--watch-period` (default 50 ms); its frames feed the watcher only, never the PTY.
- `WAIT_BUSY` → `active==True` → `BUSY`; `BUSY` → `active==False` → emit `EVT done <VERB> [#id] reason=idle`, disarm (and drop the internal stream if the client has none). Cap expiry in `WAIT_BUSY` → emit anyway (short segment finished between frames; late beats missing).
- `STOP` clears `_pending` silently (v2 spec: STOP emits no event). A new motion verb supersedes the pending one (documented).
- Gap, flagged: `EVT safety_stop` is not synthesizable (no binary watchdog-stop signal). No regression — firmware emits no EVT today either.

## PTY lifecycle

`os.openpty()`; `tty.setraw(slave)`; proxy keeps its own slave fd open for its lifetime (pins termios; avoids Linux EIO churn); symlink `os.symlink(os.ttyname(slave), link)` at `--link` (default `~/.rogo/robot-pty`), removed in `finally` + signal handler. Master fd `os.set_blocking(False)`; writes under a lock — on `BlockingIOError`, TLM lines drop (counted), replies/EVT retry 10 ms up to ~1 s. Reader loop: `OSError` → sleep 0.2 s, continue. Single-client contract in module docstring + `--help`.

## CLI

`rogo proxy` — options: global `--port`/`-v` (exist), new `--link`, `--watch-period` (ms, default 50), `--no-evt`. `cmd_proxy(args)`: `_make_robot(args)` → `ProtocolBridge(conn, …)` → run until SIGINT/SIGTERM → clean shutdown. Startup prints the PTY path + symlink (the operator's copy-paste line).

## Implementation order

1. Revise ticket 097-004 (transport → PTY, EVT criterion, acceptance edits) — team-lead records the stakeholder decision; coordinate with the parallel 097 session (see Risks).
2. `legacy_verbs.py` + cli.py `cmd_send`/aliases → committed `test_cli_send_translator.py` goes green.
3. `legacy_render.py` + `tests/unit/test_legacy_render.py` (golden lines vs firmware formats: TLM full/minimal frame, heading truncation `int(h*5729.5779513)`, omega ×1000, OK/ERR spacing variants, ERR code map, CFG order + `tw`/`minSpeed` int vs 3-dec `lroundf` rounding, ID/VER/banner, EVT).
4. `io/proxy.py` (`ProtocolBridge`, `_EvtWatcher`) + `tests/unit/test_bridge_routing.py` with a `_FakeConn` double: per-verb envelope differential vs `legacy_translate`; corr-id round trip; STOP clears EVT; relay-verb swallow; unknown → typed ERR with no wire call; SET badkey local; GET fan-out targets; SNAP restores stream state; `_EvtWatcher` transitions (idle→busy→idle fires once; WAIT_BUSY timeout; supersede).
5. cli.py `cmd_proxy` registration.
6. PTY e2e test (`tests/unit/test_bridge_pty_e2e.py`): real `os.openpty`, FakeConn upstream, client = `serial.Serial(slave_path)`: S/PING/HELLO/D + synthetic frames → EVT done appears; GET → one CFG line.
7. Hardware bench gate (below).
8. Docs: `docs/protocol-v3.md` (ticket 009) proxy section; module docstrings.

## Verification (hardware bench, robot on stand — per `.claude/rules/hardware-bench-testing.md`)

1. `mbdeploy probe`; `rogo proxy -v`; confirm symlink + identity banner at startup.
2. `screen ~/.rogo/robot-pty`: `PING`→`OK pong t=`, `ID`, `VER`, `HELLO`→`DEVICE:NEZHA2:…`, `GET`→one CFG line matching direct-port `GET` byte-for-byte, `SET sTimeout=1500`→`OK set`.
3. Motion: `S 200 200` (wheels spin) → `STOP`; `D 150 150 300 #42` → `OK drive … #42` then `EVT done D #42 reason=idle` after the wheels stop; `RT 9000` → `EVT done RT`.
4. Telemetry: `STREAM 100` → text TLM ~10 Hz with `enc=` climbing while driving; `SNAP` → exactly one line; `STREAM 0` → silence.
5. **Flagship legacy-client test**: `uv run python -m robot_radio.calibration.linear --port ~/.rogo/robot-pty --direct` — unmodified raw-pyserial text client end-to-end through the proxy.
6. `gamepad_teleop.py --port ~/.rogo/robot-pty` — 20 Hz MOVER cadence, `q=` flow control healthy.
7. Client churn: kill the client, reconnect, repeat step 2.
8. Relay-upstream variant: `rogo proxy --port <relay>`; repeat 2–3.

## Risks / open items

- **Parallel session**: another active session owns sprint 097 execution (it wrote r2 and the ticket rewrite today). Exactly one flow must execute ticket 004 — hand this plan to that flow or run it here, not both. Ticket-004 revision (step 1) is the synchronization point.
- **Rump size open question** (r2: 2 vs 3 vs 0 text verbs) does **not** block the proxy — it answers HELLO/HELP locally and uses binary PING/STOP, so it works under any rump outcome.
- `encpose/line/color` fields don't exist on the binary wire; their consumers (`fit_sim_error_model.py`, line/color TLM users) stay broken until the `encpose` schema question is settled — accepted cost, already recorded in the realign issue.
- Single-client PTY; multi-client fan-out (ticket's old socket criterion) is dropped by the transport decision; additive `AF_UNIX` listener later if needed.
- STOP waits behind at most one in-flight round trip (≤ ~300 ms); phase-2 fast-path (scan backlog for STOP) if bench objects.

## Process note

CLASI repo: on approval this plan is executed through the sprint-097 flow (revised ticket 004 → programmer dispatch), not implemented ad hoc from this approval. Team-lead cannot edit sprint artifacts via improvisation — the ticket revision is recorded first, then execution proceeds under the sprint's execution lock.
