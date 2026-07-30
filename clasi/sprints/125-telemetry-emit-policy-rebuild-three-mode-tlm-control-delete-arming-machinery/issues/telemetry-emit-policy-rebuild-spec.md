---
status: in-progress
sprint: '125'
tickets:
- 125-001
- 125-002
- 125-003
- 125-004
- 125-005
- 125-006
- 125-008
---

# Telemetry emit-policy rebuild: delete the arming machinery, make the on/off policy explicit

**Stakeholder directive (2026-07-29, code review of the telemetry gating
path).** This is a SPECIFICATION, not a discussion document. It states
precisely what is deleted, what is fixed at its true source, and how the
rebuilt system operates. It supersedes the incremental gating added across
commits `4370055a`..`eea996da` plus the uncommitted boot-settle work
(`tickBootSettle`/`kBootStableCycles`).

**Amended 2026-07-29 (same day, stakeholder discussion):** the emit policy
is now a three-state, host-controllable mode (`TLM:OFF` / `AUTO` /
`TLM:ON`) rather than a single automatic policy — Parts 3, 4, and 7 below.
The governing principle agreed in that discussion: **every frame on the
wire is either a reply to something the host sent, or part of a stream the
host turned on** (explicitly via `TLM:ON`, or implicitly by commanding a
move while in `AUTO`). Nothing unsolicited ever appears outside those two.

## Why (one paragraph)

The stakeholder's model is three sentences: boot is silent except the
`DEVICE` banner and `READY`; a frame is sent when there is an ack to carry
(and while motion is running); when the robot is parked with nothing to say,
the link is quiet. What was built instead is an emergent policy spread over
four booleans (`everEmittedPrimary_`, `changeReportingArmed_`,
`bootSettling_`, `force`) and a four-arm predicate, patched five times in
two days. It contains one structural defect: the "fresh THIS frame" bits
(`kFlagLinePresent`/`kFlagColorPresent`) toggle every cycle by design
(RobotLoop's line/color alternation), so on a robot with a connected line or
color sensor the flags word is never stable for `kBootStableCycles`,
report-on-change never arms, and the coast-down arm it gates is permanently
dead — while, had it armed, the same toggling would defeat idle silence and
stream at 25 Hz forever. The rebuild removes the entire arming lifecycle and
replaces the predicate with one that matches the three-sentence spec.

## Part 1 — Deletions (exact)

Delete the following. Each item lists the symbol, its current location, and
why it goes. Do not keep any of these behind a flag or comment them out —
remove them.

### `src/firm/app/telemetry.h` / `telemetry.cpp`

1. **`kBootStableCycles`** (constant, telemetry.h) and the whole
   "wait for the flags word to hold still" mechanism. It is a timing
   heuristic (5 quiet cycles) whose correctness depends on how long bits
   happen to wiggle; on a robot with line/color sensors it deadlocks
   (never arms). Heuristics patched onto heuristics are exactly what this
   rebuild removes.
2. **`markBootComplete()`** (declaration + definition). Nothing remains for
   it to do: boot silence falls out of the new emit predicate (Part 3), not
   out of an arming call.
3. **`tickBootSettle()`** (declaration + definition) and its call site in
   `RobotLoop::cycle()` (currently immediately after `tlm_.update(state_)`).
4. **Members `changeReportingArmed_`, `bootSettling_`, `stableCycles_`,
   `lastSeenFlags_`** — the entire hidden lifecycle. After this change,
   `App::Telemetry` has NO arming state: a freshly constructed instance
   behaves identically to one on a robot that has been up for an hour,
   which also deletes the boot-reenactment boilerplate from
   `app_telemetry_harness.cpp` (`markBootComplete()` +
   `2 * kBootStableCycles` warm-up ticks).
5. **Member `lastEmittedFlags_`** including its `0xFFFFFFFFu` seed and the
   now-false comment claiming the seed makes "the FIRST frame always send",
   and the write to it at the tail of `emitPrimary()`. It exists only to
   serve report-on-change (arm 4), which is deleted below.
6. **`hasSomethingToSay()` arm 4** (report-on-change: `changeReportingArmed_
   && flags_ != lastEmittedFlags_`). Flag-change push was never in the
   stakeholder spec. Faults do not need push delivery: the flags word rides
   every frame that goes out for any other reason (every ack-carrying
   frame, every motion frame), and is queryable on demand via `STATUS`
   (cleartext) or a bare `TLM` (full frame). If push-on-fault is ever
   genuinely wanted, it comes back as a NEW issue specifying a masked
   comparison over latching fault bits only (bits 7, 9, 15–18) — never
   freshness bits (0, 13, 14), never event bits. Not part of this rebuild.
7. **The `changeReportingArmed_` gate on arm 2** (the coast-down/wheel-
   velocity arm). The gate coupled encoder-sample validity to the boot
   lifecycle to suppress a symptom whose real fix is Part 2. Arm 2 itself
   is replaced by the activity hold-off in Part 3.
8. **`kFlagEventBootReady`** (bit 11) — the constant, its
   `setLiveFlag(kFlagEventBootReady, true)` call at `RobotLoop::boot()`'s
   tail, and its bullet in the flags layout comment and in
   `setLiveFlag()`'s doc comment. It is set once, latched forever, and its
   transition is unobservable by construction (boot ends before any frame
   that could carry the edge). The `READY` line already announces this, in
   cleartext, which is what the stakeholder actually asked for. Bit 11
   becomes RESERVED, same treatment as bit 5.

### `src/firm/app/robot_loop.cpp`

9. **The stale "Arm report-on-change now" comment block** above the
   `markBootComplete()` call (the call itself goes per item 2). The
   replacement boot() tail is: `sendBanner()`, `sendReady()`, nothing else.
10. **The factually wrong DTR claim** in boot()'s "NOT forced" comment
    ("opening a serial port asserts DTR and resets the board"). Bench-
    established fact (`.clasi/knowledge/`, serial-monitor-never-shows-the-
    banner): the board does NOT reset on port open or DTR pulse. Keep the
    un-forced emit and its real rationale (no per-probe frame flood on
    power-on); delete the invented justification.

### What is explicitly KEPT (do not "clean up" these)

- The single-assembly `update(const Types::RobotState&)` projection and
  `setLiveFlag()`'s narrow two-bit escape hatch (minus the boot-ready
  bullet).
- The bounded ack ring, `ackSends_[]`, and `kAckRepeats = 3` (redundancy
  sized against sprint 123's measured residual wire loss — a reasoned
  choice, not a hack). Reminder of the load-bearing architectural fact:
  **protocol v5 has no separate ack message — the telemetry frame IS the
  ack's only vehicle** (the ack ring rides inside it). Every mode in Part 3
  therefore delivers ack frames; suppressing them is never an option.
- `primaryDue()` / `kPrimaryPeriod` cadence gating, `everEmittedPrimary_`.
- The bare-`TLM` request path (`Comms::takeTelemetryRequest()` → `force`) —
  extended, not replaced, by Part 4's command surface.
- `READY`/`STATUS`/`HELP` verbs, host `wire_commands.py` mirror, the
  banner→`READY` ordering, and `STATUS` answering from the same `state_`
  the projection used.
- The unforced `emit()` call inside the boot loop (see Part 6 for the
  documented consequence).

## Part 2 — Source-level fix (the bug the gates papered over)

**Motor velocity must read 0 until it is real.** The reason arm 2 was gated
at all is that the first encoder read after power-on can report a bogus
nonzero velocity on a robot that has never moved. That is a
`Devices::Motor` sampling defect and is fixed there: `velocity()` returns
0.0f until the device has collected at least two valid samples (a velocity
is a difference quotient; with fewer than two samples there is no velocity,
and fabricating one from an uninitialized baseline is the defect). Add a
unit test: construct, connect, collect one sample → `velocity() == 0`;
second sample → real value. No telemetry-side guard may be added for this
condition.

## Part 3 — The rebuilt emit policy: three modes (normative)

Telemetry operates in one of three host-controllable modes. The mode is a
single member with a single writer (the `TLM` command handler, Part 4), and
it resets to `kAuto` at every boot — it is never persisted:

```cpp
// telemetry.h -- the whole policy surface.
enum class TlmMode : uint8_t { kOff, kAuto, kOn };
...
TlmMode mode_ = TlmMode::kAuto;  // reset by construction each boot; one writer
```

| Mode | Unsolicited frames | Command → TLM-as-ack | Bare `TLM`/`TLM:NOW` → one frame |
|---|---|---|---|
| `kOff` | never | yes | yes |
| `kAuto` (default) | during moves + coast-down | yes | yes |
| `kOn` | every cadence tick | yes (subsumed) | yes (subsumed) |

The emit predicate must read as the following and nothing more:

```cpp
// telemetry.cpp -- the whole policy. due = cadence gate, unchanged.
const bool activity = (flags_ & kFlagActive) ||
                      (everMoved_ && (now - lastActivity_) < kCoastHoldoff);
bool unsolicited = false;
switch (mode_) {
  case TlmMode::kOff:  unsolicited = false;    break;
  case TlmMode::kAuto: unsolicited = activity; break;
  case TlmMode::kOn:   unsolicited = true;     break;
}
if (primaryDue(now) && (force || unsolicited || pendingAckDeliveries())) emitPrimary(now);
```

The three reasons to emit, in detail:

1. **Request** (`force`): a bare `TLM` (or `TLM:NOW`) line arrived. One
   frame, honored in EVERY mode including `kOff`, still cadence-gated so a
   request storm cannot outrun the wire.
2. **Unsolicited, per mode:**
   - `kOff` — never. For the client that genuinely does not want
     telemetry, and for a human silencing a streaming robot to type at a
     serial monitor. Polling still works (reason 1), and command replies
     still arrive (reason 3) — a polling client in `kOff` rarely even
     needs bare `TLM`, since every ack frame is also a full state
     snapshot.
   - `kAuto` — the activity window. Streaming is needed during moves
     precisely because there is no command bandwidth to ask for frames
     mid-motion; at rest there is plenty, so the host polls instead.
     New state replacing old arms 1+2:
     - `uint32_t lastActivity_` `// [ms]` — refreshed to `now` whenever
       `kFlagActive` is set, and whenever (`everMoved_` AND either wheel's
       staged velocity is nonzero AND the window is currently open). The
       window-open condition on the velocity refresh means coasting wheels
       keep the window alive, but wheels alone can never OPEN a closed
       window.
     - `bool everMoved_` — set true the first time `kFlagActive` is seen
       this power cycle, never cleared. This is what makes power-on
       silence unconditional: before the first commanded motion, wheel
       velocity (bogus sample, hand-spun wheel on the stand) cannot wake
       the link. Belt-and-suspenders with Part 2, by design.
     - `constexpr uint32_t kCoastHoldoff = 2000;  // [ms]` — sized to
       cover the bench-observed ~1.2 s post-STOP settle with margin. The
       ONE tunable, replacing `kBootStableCycles`, the arming machine, and
       both gated arms.
     - Net behavior: `STOP` mid-motion → `kFlagActive` drops, window is
       open, coasting velocities keep refreshing it, frames continue
       through the deceleration (the "velocity frozen at 366 mm/s" harness
       bug stays fixed) → wheels reach 0 → no more refreshes → silence
       within `kCoastHoldoff`.
   - `kOn` — every cadence tick, motion or not. This mode exists for the
     characterization scripts that watch a PARKED robot
     (`otos_drift.py`, `tlm_log.py`, `velocity_step_response.py`) — under
     `kAuto` they would have to spam requests, which is exactly the
     command-clog this design avoids.
3. **Pending acks** (`pendingAckDeliveries()` — rename of the existing
   arm-3 loop into a named private helper): any ring entry with
   `ackSends_[i] < kAckRepeats`. **Honored in EVERY mode, never gated by
   anything.** The telemetry frame is the ack's only vehicle (Part 1 KEEP
   list); `kOff` suppressing ack frames would reintroduce "acked but
   nothing happened", the failure class that cost a bench session. It
   costs the human-at-a-terminal case nothing: humans type cleartext
   verbs, and cleartext replies come back cleartext (`f26b3010`); binary
   acks only answer binary commands. `kOff` therefore means "no
   UNSOLICITED frames": a bounded `kAckRepeats` burst per command still
   delivers, then silence. A move commanded in `kOff` runs normally and
   produces only its ack frames (enqueue + completion), no stream.

There is NO fourth reason. In particular: no flag-change push, no arming
state, no boot-completion signal into `Telemetry`. The class has no
lifecycle beyond `mode_` — construction is the only initialization, and
construction already yields the correct boot state (`kAuto`, silent at
rest).

## Part 4 — The `TLM` command surface (normative)

Protocol v5 grammar is `<COMMAND>[':' <data>]` — the mode controls are
therefore colon-spelled. `TLM ON` with a space is NOT valid and gets no
special-case parse. The full inbound `TLM` surface:

| Line | Effect | Reply |
|---|---|---|
| `TLM` | one frame now (existing behavior, kept) | one binary telemetry frame |
| `TLM:NOW` | alias for bare `TLM` | one binary telemetry frame |
| `TLM:ON` | `mode_ = kOn` | the `STATUS` line (carries the new mode) |
| `TLM:AUTO` | `mode_ = kAuto` | the `STATUS` line |
| `TLM:OFF` | `mode_ = kOff` | the `STATUS` line |
| `TLM:<anything else>` | no mode change | the `HELP` line |

Decisions embedded in that table:

- **Overloading `TLM:<data>` is safe and deliberate:** a real binary
  telemetry frame only ever travels robot→host; an inbound robot-ward
  `TLM:` line is unambiguous. The arguments are matched case-insensitively
  as the cleartext tokens `NOW`/`ON`/`AUTO`/`OFF`.
- **Mode changes reply with the `STATUS` line, not a bespoke ack.** A
  human sees immediate cleartext confirmation; a machine parses the same
  line it already knows. No new reply shape is invented.
- **`STATUS` gains a `tlm=` field**: `tlm=off|auto|on`, appended to the
  existing `STATUS:k=v` formatter in `comms.cpp`, so anyone can always
  discover the current mode. (Wire-visible key string — spelled here
  normatively, per the coding-standards exclusion for wire keys.)
- **`HELP` is updated** to list `TLM[:NOW|ON|AUTO|OFF]`.
- **Plumbing** follows the existing bare-`TLM` pattern: `Comms` parses the
  argument and stages a pending request/mode-change; `RobotLoop::cycle()`
  consumes it at the same point it consumes `takeTelemetryRequest()`
  today, calling `tlm_.setMode(...)` and/or forcing the emit. `Telemetry`
  itself never parses wire text.
- **No persistence**: nothing is written to config; power-cycle returns to
  `kAuto`. A robot that could boot into a stale `kOff` is a debugging
  trap, and one that boots into `kOn` violates the Part 6 power-on
  contract.

Host mirror: `wire_commands.py` already carries the `TLM` verb; no verb
table change is needed. `NezhaProtocol` (protocol.py) gains three trivial
helpers — `tlmOn()`, `tlmOff()`, `tlmNow()` — thin wrappers sending the
lines above (Python naming per project convention).

## Part 5 — Flags-word semantics (documentation, one decision)

The flags layout comment in telemetry.h must classify every bit into
exactly one of three declared classes, as a labeled section header per
class, so the next reader cannot repeat the category error:

- **State bits** (level, meaningful across frames): 1, 2 (`kFlagActive`),
  3, 4, 6, 7, 8, 9, 12, 16, 17, 18.
- **Freshness bits** (valid-THIS-frame qualifiers for a payload field,
  toggle by design, carry no cross-frame information): 0
  (`kFlagOtosPresent`), 13, 14. These may NEVER participate in any
  change-detection or stability logic.
- **Event bits** (transition-cycle-only): 10 (`kFlagEventDeadmanExpired`),
  15 (`kFlagFaultMoveTimeout` — rides the completing frame). Bit 11 is
  deleted (Part 1 item 8); bit 5 stays RESERVED.

No wire change: bit positions and meanings on the wire are untouched
(bit 11 simply never sets any more — hosts do not consume it; the ack ring
and `READY` carry its information).

## Part 6 — Boot sequence (normative, mostly unchanged)

1. Boot loop: probe → build throwaway `bootState` (connectivity bits only)
   → `update()` → **unforced** `emit()` → pace. Under the Part 3 predicate
   this emits nothing unless a host transmits during boot (mode is `kAuto`,
   `everMoved_` false, no activity possible).
2. Early commands are NACKed (`ERR_NOT_CONFIGURED`) via the ack ring,
   unchanged. **Documented consequence (decision, not accident):** a host
   that transmits during boot receives binary ack-carrying frames BEFORE
   the banner. This is correct — the host asked, and stranding a NACK
   reintroduces the "acked but nothing happened" bench-session class. A
   silent host sees zero binary bytes. State this in boot()'s comment.
3. Boot tail: `sendBanner()` then `sendReady()`. Nothing between them, no
   telemetry calls. `READY` is the one and only "telemetry may now start"
   signal, and it signals it to the HOST, not to the Telemetry class.

Resulting power-on contract with a silent host: exactly two cleartext lines
(`DEVICE:...` banner, `READY`), zero binary frames, indefinitely, until the
host speaks or commands motion. Mode is `kAuto`; any prior session's
`TLM:ON`/`TLM:OFF` is forgotten.

## Part 7 — Bench-script migration (required, same change)

Under `kAuto`, a parked robot emits nothing — so every script that today
relies on always-on streaming from a stationary robot MUST bracket its
capture with the mode commands or it silently records zero frames:

- `src/tests/bench/otos_drift.py`, `tlm_log.py`,
  `velocity_step_response.py` (and any other parked-capture script found
  by grepping `src/tests/bench/` for telemetry consumption): send `TLM:ON`
  after connect, `TLM:OFF` (or rely on the next power cycle) at teardown.
- Scripts that only consume telemetry DURING commanded moves
  (`twist_drive.py`, `move_protocol_bench.py`, `radio_bench_gate.py`,
  square-tour runners) need no change — `kAuto` streams for them — but
  must be RUN as part of acceptance to prove it.

## Part 8 — Acceptance criteria

Sim (extend `app_telemetry_harness.cpp` / `app_robot_loop_harness.cpp`):

1. Fresh boot, silent host → banner + `READY` only; `primaryEmitCount() == 0`
   after N cycles of parked idling.
2. **Regression for the structural defect:** parked robot WITH line and
   color sensors delivering fresh alternating readings → still zero frames.
   (This is the case the old design could not pass.)
3. Command to parked robot → its ack rides a frame within 2×`kPrimaryPeriod`;
   exactly `kAckRepeats` frames carry it; then silence again.
4. Move in `kAuto` → frames at cadence while active; `STOP` → frames
   continue while staged wheel velocity is nonzero; silence within
   `kCoastHoldoff` of velocities reaching 0.
5. Hand-spun-wheel / bogus-velocity case: nonzero wheel velocity with
   `everMoved_ == false` → zero frames.
6. Bare `TLM` on a parked robot → exactly one frame; `TLM:NOW` behaves
   identically.
7. `TLM:OFF`, then a Move → the move executes, ONLY ack frames appear
   (enqueue + completion, `kAckRepeats` each), no stream; bare `TLM`
   mid-move still answers with one frame.
8. `TLM:ON` on a parked robot → frames at cadence; `TLM:OFF` → stream
   stops within one `kPrimaryPeriod`; `TLM:AUTO` restores mode-2 behavior.
9. Every `TLM:ON|AUTO|OFF` is answered by a `STATUS` line whose `tlm=`
   field shows the NEW mode; `TLM:<garbage>` is answered by `HELP` and
   changes nothing.
10. Mode is not persistent: set `kOn` (or `kOff`), simulate reboot →
    `kAuto`.
11. Motor unit test per Part 2 (velocity 0 before two samples).
12. The harness boot-boilerplate (`markBootComplete` + settle ticks) is
    GONE; no test may re-add a Telemetry lifecycle call.

Bench (standing gate, robot on the stand):

13. Power-on with a passive serial capture already attached → verify
    contract of Part 6 (two cleartext lines, zero binary bytes over ≥30 s).
14. Run `src/tests/bench/radio_bench_gate.py` and the quick smoke sequence
    (`twist_drive.py`) — all existing PASS lines hold under `kAuto`;
    additionally observe silence-after-coast (~2 s).
15. Human-terminal check: with the robot parked, type `TLM:ON` in a serial
    monitor → binary streams; `TLM:OFF` → terminal is typeable again;
    `STATUS` → readable line including `tlm=off`.
16. One parked-capture script from Part 7 (e.g. `otos_drift.py`) runs
    end-to-end and records frames, proving the migration.

## Out of scope

- Push-on-fault via masked flag comparison (future issue if wanted).
- Any wire-format change to the telemetry frame itself; the only protocol
  additions are the cleartext `TLM:` arguments and the `STATUS` `tlm=`
  field (both additive).
- Renaming or restructuring the ack ring, `STATUS`/`HELP`, or the verbs
  table.
- Persisting the telemetry mode in config (deliberately rejected, Part 4).
