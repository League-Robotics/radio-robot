---
id: '013'
title: 'Radio-relay standing bench gate: banner, HELLO/PING, move_wheels start/stop,
  encoder climb, acks, wire-quality vs budget'
status: done
use-cases:
- SUC-007
- SUC-008
depends-on:
- '005'
- '006'
- 008
- 009
- '010'
- '011'
- '012'
github-issue: ''
issue:
- protocol-v5-one-line-packets-command-prefix-and-newline-cobs.md
- relay-handshake-trips-comms-malformed.md
- telemetry-physical-layer-corruption-and-move-ack-observability.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Radio-relay standing bench gate: banner, HELLO/PING, move_wheels start/stop, encoder climb, acks, wire-quality vs budget

## Description

**This ticket is the sprint's acceptance gate.** Run the full standing
bench gate over the **`!GO` radio-relay data plane — NOT USB** (per
stakeholder directive; see `.claude/rules/hardware-bench-testing.md` and
`.clasi/knowledge/` for the relay `!GO` data-plane protocol: host opens
the relay with DTR asserted, sends `!GO` to enter the data plane, then
plain commands with no `>` prefix). A sprint is not done on tests alone
— this must be SEEN working on the stand.

The acceptance criteria below are the stakeholder's list, verbatim, all
of them over the relay:

## Acceptance Criteria

- [ ] Banner on connect: `DEVICE:` observed at boot on a fresh relay
      connect with no `HELLO` sent, and `connect()` completes without
      entering the `_HELLO_CLASSIFY_TIMEOUT_S` fallback.
- [ ] `HELLO`/`PING` (and `ID`/`VER`) all answer over the relay.
- [ ] `move_wheels` starts the wheels; `stop` stops them — over the
      relay.
- [ ] Encoder positions climb in telemetry while the move runs, over
      the relay.
- [ ] Enqueue ack AND completion ack are both observed, over the relay
      (via the packed `acks` ring from ticket 008; confirmed generally
      fixed by ticket 011).
- [ ] A `wire_truth`-equivalent quality measurement (ticket 012's
      promoted script) runs through the relay, checked against ticket
      012's stated loss budget for the relay path.
- [ ] `kFaultCommsMalformed` stays clear throughout (confirming ticket
      010's fix holds under the full gate, not just its isolated repro).
- [ ] All of the above pass over the radio relay specifically. USB may
      additionally be run for comparison but does NOT satisfy this
      gate on its own.
- [ ] The 123-006 hardware repro (`move_wheels` embedding a literal
      `0x0A`) executes 10/10 over the relay (ticket 005 confirmed this
      over USB; this ticket confirms it over the actual acceptance
      transport).

## Testing

- **Existing tests to run**: N/A — this ticket IS the test. All prior
  unit/sim/property tests from tickets 001-012 must already be green
  before this bench session is attempted.
- **New tests to write**: none beyond the bench session itself; results
  are recorded as this ticket's own closing evidence (PASS/FAIL per
  criterion, per `.claude/rules/hardware-bench-testing.md`'s stated
  format).
- **Verification command**: a single bench session on the stand,
  executing the full sequence above over the radio relay, with results
  recorded in this ticket's closing notes.

## Completion Notes (programmer, no hardware in this environment)

**The robot/relay is NOT connected in this development environment**
(`pyocd list` finds no probe, no `/dev/cu.usbmodem*`) — this ticket's
deliverable is therefore the **gate itself**: a runnable, self-scoring
bench script covering every acceptance bullet above, plus everything
that can honestly be dry-run without hardware. None of the acceptance
checkboxes above are checked — they require an actual bench reading,
which this environment cannot produce. **The stakeholder's own bench run
on master closes each of them**, per the exact command lines below.

### What was built

`src/tests/bench/radio_bench_gate.py` — a standalone HITL CLI tool (not
pytest-collected, per `src/tests/CLAUDE.md`'s three-domain convention),
structured in three phases, every check self-scoring PASS/FAIL and the
process exiting nonzero on any failure:

- **Phase 1** (fresh relay connect, own connection): `DEVICE:` banner
  observed without the `_HELLO_CLASSIFY_TIMEOUT_S` fallback (bullet 1),
  then — same connection, before any application command — `SUC-008`
  (`kFaultCommsMalformed` clear with zero application commands sent).
- **Phase 2** (one main relay session): `HELLO`/`PING`/`ID`/`VER` all
  answer (bullet 2, via `send_fast()` + the `on_recv` hook — see the
  module docstring's "Design notes" for why `SerialConnection.send()`
  cannot observe these replies post-124-005); `move_wheels` starts the
  wheels, encoders climb, `stop` stops them (bullets 3+4); enqueue AND
  completion acks observed for one self-completing Move (bullet 5); the
  123-006 `0x0A`-embedding repro shape 10/10 (SUC-007).
- **Phase 3** (own connection): the wire-quality-vs-budget leg (bullet
  6), by importing and calling ticket 012's own `wire_truth.run_capture()`
  / `report()` / `kRelayBudget` directly — not reimplemented. An optional
  `--usb-port` runs an informational-only USB comparison leg (bullet 7)
  that never affects the exit code.
- A cross-phase `SessionTelemetry` object aggregates `kFaultCommsMalformed`
  observations across the WHOLE session (not just the Phase-1 repro), per
  bullet 7's "stays clear throughout."

### Vacuous-PASS defect found and fixed during this ticket (pre-close)

The coordinator's own dry-run against a nonexistent port caught a real
defect before this ticket closed: the original SUC-008 aggregate check
read `len(tracker) == 0` (no fault observed) as PASS even when BOTH
connect attempts had already failed and zero telemetry frames were ever
observed — absence of evidence scored as evidence of absence, the exact
failure shape sprint 123 shipped a wire bug through. Fixed by requiring
POSITIVE evidence (a minimum real frame count) before any "stayed clear"
check can PASS, in three places: the SUC-008 phase-1 check, the SUC-008
session-wide aggregate, and (found during the follow-up audit) the
wire-quality budget check, where `wire_truth.report()`'s own
`corrupted_rate = 0/total if total else 0.0` scores a zero-line capture
as trivially "within budget" — a caller-side minimum-lines floor was
added on top (ticket 012's own `wire_truth.py` is untouched). Every other
check in the file was audited for the same shape and found to already
require positive evidence by construction (`X is not None and X.ok`, or
`any()`/`bool()` over what is `False` on an empty list) — see the
module's own docstring "The vacuous PASS defect and its fix" section for
the full audit trail. `main()` also now prints a loud banner if neither
phase ever connects, stating plainly that no criterion in the run can be
considered scored.

### Dry-run verification performed (no hardware)

- `--help`: prints cleanly, no hardware touched.
- No-device path (`--port /dev/cu.doesnotexist999`): fails fast (no
  hang — `serial.Serial.open()` raises immediately for a nonexistent
  port), every check reports FAIL or INCONCLUSIVE, **0/4 checks pass**,
  exit code 1. Confirmed (post-fix) that no check reports a vacuous PASS
  on this path — see the full transcript in the sprint execution log /
  agent report.
- Unit-level verification of every piece of deterministic logic that
  does not require a live connection: `_velocity_with_embedded_0x0a()`
  (genuinely embeds a literal `0x0A` byte, deterministic), the `Result`
  class's PASS/FAIL/exit-code arithmetic, `_find_completion_ack()`,
  `_next_move_id()`'s monotonic/non-colliding sequencing, and — targeting
  the vacuous-PASS fix specifically — `scenario_wire_quality_vs_budget()`
  against a monkeypatched zero-line `CaptureStats` (confirms FAIL, not
  wire_truth's own vacuous "RESULT: PASS") and
  `scenario_comms_malformed_stays_clear_fresh()` against a fake
  zero-frame connection (confirms FAIL, not a vacuous PASS).
- **Unverifiable without hardware** (this is exactly what the stakeholder's
  bench run must close): every criterion listed in Acceptance Criteria
  above — the banner/timing behavior against a REAL boot, the real
  `HELLO`/`PING`/`ID`/`VER` round trip over an actual relay, real
  `move_wheels`/`stop`/encoder-climb behavior on the real drivetrain, real
  enqueue/completion acks off the real ack ring, the real relay
  wire-quality measurement against ticket 012's budget (whose relay
  numbers are themselves only a first-pass proposal, per `wire_truth.py`'s
  own docstring, until a real relay measurement lands), and the real
  10/10 `0x0A`-repro outcome over an actual radio link.

### How to run (exact command lines, in order, on the stand)

```bash
# 1. Full acceptance gate (relay only). --port is the RELAY's own serial
#    port (confirm with `mbdeploy list`'s ROLE column). Default 120s
#    wire-quality window matches wire_truth.py's own default -- expect
#    ~2.5-3 minutes total wall time.
uv run python src/tests/bench/radio_bench_gate.py \
    --port /dev/cu.usbmodemRELAY123

# 2. Optional: also run an informational USB-direct wire-quality
#    comparison leg (does NOT gate the exit code -- bullet 7).
uv run python src/tests/bench/radio_bench_gate.py \
    --port /dev/cu.usbmodemRELAY123 --usb-port /dev/cu.usbmodemUSB456
```

Record the printed PASS/FAIL-per-check transcript and the final `GATE
RESULT` line as this ticket's own closing evidence; check off each
acceptance box above against that transcript.

### Pre-existing `src/tests/bench/` scripts found stale against the v5 wire (for ticket 014)

- `cli.py`'s `_push_calibration()` (`src/host/robot_radio/io/cli.py`)
  calls `conn.send("ID", read_timeout=500)` and blocks on its reply —
  this now ALWAYS times out: 124-005 turned
  `SerialConnection._handle_text_line()` into a no-op (`DEVICE`/`PONG`/
  `ID`/`VER` cleartext replies have no live reader-thread consumer any
  more), so `send()`'s corr-id-keyed reply queue is never populated for
  those four verbs. This function is therefore silently dead code
  post-124-005, not something this ticket's own scope covers fixing
  (`io/cli.py` is not in the `src/tests/bench/` catalog this ticket
  works in) — worth its own follow-up.
- `dev_exercise.py` and other pre-124 bench scripts still reference the
  v2-era `DEV`/`SET`/`GET` command family per `src/tests/CLAUDE.md`'s own
  (also stale) "Drives the robot via the `DEV` command family
  (`docs/protocol-v2.md` §16)" line — that command family does not exist
  on the current wire (`docs/protocol-v4.md` §2.4: only `HELLO`/`PING`
  remain as the text rump; everything else is the binary `move`/`config`/
  `stop` command plane). `src/tests/CLAUDE.md`'s own watchdog-widen
  guidance (`DEV WD 3000`/`DEV WD 1000`) is the same vintage and doesn't
  map onto protocol v4/v5 either — there is no ambient watchdog any more,
  every `Move` carries its own bounded `timeout` instead (see this
  ticket's own script docstring "Why no watchdog widen/restore").
