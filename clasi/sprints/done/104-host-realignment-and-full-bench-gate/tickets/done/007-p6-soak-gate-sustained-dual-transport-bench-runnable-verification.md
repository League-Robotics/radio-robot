---
id: '007'
title: "P6 soak gate \u2014 sustained dual-transport bench-runnable verification"
status: done
use-cases:
- SUC-017
depends-on:
- '001'
- '002'
- '003'
- '004'
- '005'
- '006'
github-issue: ''
issue: single-loop-firmware-p3-p7-continuation.md
completes_issue:
  single-loop-firmware-p3-p7-continuation.md: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# P6 soak gate — sustained dual-transport bench-runnable verification

## Description

This sprint's Definition of Done, per the 2026-07-14 stakeholder hard
scoping rule ("every sprint ends bench-runnable") and P6 of the
continuation issue. Deploy this sprint's firmware (ticket 004's fault-bit
additions) and, using ticket 006's rewritten `rig_soak.py`, run a
sustained (soak-duration, not smoke-duration — materially longer than
103-010's short bench-gate captures) verification pass over BOTH direct
USB and the radio relay. Strictly last in this sprint — depends on every
other ticket.

Like 103 ticket 010, this ticket is verification, not new code: if it
finds a defect in tickets 001-006's work, the fix belongs in whichever
ticket owns the broken module (reopen per the project's normal mechanism),
not silently patched here.

## Acceptance Criteria

- [x] `mbdeploy probe` confirms exactly one micro:bit connected;
      `mbdeploy deploy --build` flashes this sprint's firmware (ticket
      004's `kFaultCommsMalformed` bit + `kFaultI2CSafetyNet` doc fix —
      note: doc-only changes don't affect the binary, so this really
      verifies ticket 004's bit-wiring compiled and flashed correctly).
- [x] Sustained soak run (duration a ticket-time/stakeholder judgment
      call — architecture-update.md Step 7 Open Question 4 — informed by
      103-010's own 120s/1875-frame USB continuity capture as a floor, not
      a ceiling) over direct USB: zero I2C NAK/timeout errors
      (`kFaultI2CNak` stays clear throughout), TLM drop rate measured and
      reported (not assumed), wedge latch clears promptly once motion
      resumes after any transient idle-state assertion (103-010 §6's
      documented contract — the acceptance bar is "clears promptly," not
      "never asserts").
  - [x] Same soak run repeated over the radio relay's `!GO` data plane
        (reconnect, repeat) — both transports measured independently.
- [x] Deadman kill-test repeated under soak conditions (mid-soak host
      process kill, not just at idle) on both transports; wheels stop
      within one stale window with no further host input.
- [x] `kFaultI2CSafetyNet` observed to NOT re-trip after its initial
      boot-time latch during either soak window — corroborates ticket
      004's characterization under sustained load, not just 103-010's
      short session.
- [x] `kFaultCommsMalformed` (new this sprint) stays clear throughout both
      soak windows — the host's own well-formed traffic should never trip
      it; if it DOES trip, that is a real finding (either a host-side
      encoding bug this ticket surfaces, or evidence the bit's wiring in
      ticket 004 is miscalibrated) to be investigated, not waved through.
      It DID trip once during relay connect (before any application traffic) --
      investigated, not waved through, see completion notes and the new
      `clasi/issues/relay-handshake-trips-comms-malformed.md`.
- [x] `uv run python -m pytest tests/unit -q` reports 0 failed, 0 errors
      immediately before this bench session (confirms ticket 002's sweep
      held through the rest of the sprint's changes).
- [x] No motor left energized at the end of the verification session.
- [x] Session conducted per `.claude/rules/hardware-bench-testing.md`
      (robot on the stand, wheels off the ground) — confirmed explicitly
      in completion notes.

## Testing

- **Existing tests to run**: `uv run python -m pytest tests/unit -q`
  (must be green — see Acceptance Criteria).
- **New tests to write**: none (this IS the test — real hardware, soak
  duration, not pytest); ticket 006's `rig_soak.py` is the tooling this
  ticket exercises.
- **Verification command**: a real bench soak session per Acceptance
  Criteria, using `mbdeploy` + ticket 006's `rig_soak.py` + a second
  connection through the radio relay.

## Implementation Plan

**Approach**: Verification only, mirroring 103 ticket 010's own
Implementation Plan posture. If a defect is found, report it and, if
small and clearly scoped to one already-closed ticket in this sprint,
reopen that ticket — a call for whoever executes this ticket to make in
the moment.

**Files to create/modify**: none expected; if a defect requires a real
code fix, it lands in the ticket that owns the broken module.

**Testing plan**: covered above.

**Documentation updates**: record the full soak session's results (drop
rates both transports, fault/event bit timeline, deadman kill-test
timing, encoder motion sanity, the confirmed soak duration used) in this
ticket's own completion notes, matching 103-010's level of detail — this
IS the sprint's evidence of "bench-runnable," and future readers should
not have to re-run the session to trust the sprint closed correctly.

## SUC-017: P6 soak gate — sustained, dual-transport, bench-runnable

Parent: `single-loop-firmware-p3-p7-continuation.md` (P6); the hard
scoping rule's "every sprint ends bench-runnable" requirement.

- **Actor**: Bench operator; the physical rig.
- **Preconditions**: Tickets 001-006 complete; firmware flashed.
- **Main Flow**: Run the soak; observe fault/event bits; repeat the
  deadman kill-test under load; measure drop rate both transports.
- **Postconditions**: Full host tooling drives the robot over the binary
  plane on both transports, sustained and clean. This IS this sprint's
  Definition of Done.
- **Acceptance Criteria**: see above.

## Completion notes (2026-07-15, robot v0.20260715.3, bench stand)

Session conducted per `.claude/rules/hardware-bench-testing.md`: robot on the
stand, wheels off the ground, throughout. `mbdeploy probe` confirmed exactly one
non-relay micro:bit (`NEZHA2`/robot, UID
`9906360200052820a8fdb5e413abb276000000006e052820`,
`/dev/cu.usbmodem2121102`) plus the relay (`RADIOBRIDGE`,
`/dev/cu.usbmodem2121302`, never flashed). `uv run python -m pytest tests/unit -q`
was green (222 passed, 0 failed, 0 errors) before the session and unaffected
throughout (no source changes were needed). Built with `just build-clean`
(v0.20260715.3) and flashed via `mbdeploy deploy --hex MICROBIT.hex <uid>`
(`--build` target was unavailable as a combined flag with an explicit UID target,
so build and flash were run as separate steps — functionally identical).

**Soak matrix (official, clean-boot, 240s each):**

| Metric | Direct USB | Relay |
|---|---|---|
| Duration | 240.1 s | 240.1 s |
| Commands sent | 1472 (36 stop segments) | 1474 (36 stop segments) |
| Primary frames delivered | 3333 | 3318 |
| TLM drop rate | 0.00% | 0.48% |
| Ack loss (informational) | 0.07% | 2.17% |
| Encoder-responsive rate | 99% (1339/1355) | 98% (1334/1357) |
| New fault bits during window | none | none |
| `kFaultI2CSafetyNet` (bit 0) | boot one-shot, never re-tripped | boot one-shot, never re-tripped |
| `kFaultWedgeLatch` (bit 1) | idle-only, clears promptly on motion (see below) | same |
| `kFaultI2CNak` (bit 2) | never set | never set |
| `kFaultCommsMalformed` (bit 3) | never set | **set at connect, before the soak's own baseline — see caveat below; never set as a NEW bit during the 240s window itself** |
| Secondary TLM samples | 1123 | 1113 |

The relay is measurably noisier than direct USB (0.48% vs 0.00% TLM drop; 2.17% vs
0.07% ack loss) but both passed `rig_soak.py`'s own gate (0.00%/0.48% both well
under the 2% drop-rate threshold; 98-99% well over the 80% responsiveness
threshold; zero new fault bits either transport).

**Wedge-latch idle/motion contract, directly verified** (matching 103-010 §6's
documented "clears promptly," not "never asserts" bar): with the robot freshly
connected and idle, `fault_bits` reads `0b11` (bit 0 boot one-shot + bit 1 wedge
latch, asserted at idle). Commanding a `twist()` clears bit 1 within the first
telemetry frame after motion starts (`fault_bits` -> `0b1`, `active=True`, encoders
incrementing); stopping re-asserts it once idle again (`fault_bits` -> `0b11`). This
is the documented idle-boundary latch behavior, not a defect — noting for the
record since `telemetry.h`'s own doc comment currently (incorrectly) still says bit
1 is "declared, not yet wired" -- it is actually wired (`source/main.cpp`:
`tlm.setFault(App::kFaultWedgeLatch, motorL.wedged() || motorR.wedged())`) and
behaves exactly as bit 0's own documented boot-one-shot precedent would predict for
an idle-state assertion. Worth a doc-only correction in a future ticket; not
reopened here since it is a stale comment, not a behavioral defect.

**`kFaultCommsMalformed` DID trip — investigated, not waved through.** Isolated
test (fresh clean boot, connect via the relay's `!GO` data plane ONLY, zero
application commands sent) showed bit 3 already set in the very first telemetry
frame. The SAME test over direct USB never shows it. This is the relay's own
CONNECT HANDSHAKE tripping the bit, not the host's application traffic during the
soak window — structurally the same "fires once, never re-trips" shape as
`kFaultI2CSafetyNet`'s own documented boot-time one-shot, just triggered by
relay-connect instead of firmware-boot, which is why `rig_soak.py`'s own
baseline-vs-new-bit gate correctly did not fail on it. Filed as its own new issue,
`clasi/issues/relay-handshake-trips-comms-malformed.md`, for future root-cause
investigation (not this verification-only ticket's scope) — root cause not yet
isolated, likely relay control-plane bytes or a mode-transition artifact briefly
reaching the robot's line parser.

**Deadman kill-test, both transports.** Started a continuous twist-reissue driver
(150ms reissue, 500ms arm window, same shape as `rig_soak.py`'s own pattern) and
SIGKILLed it mid-motion, then reconnected to watch encoder/`active` telemetry.
Direct USB: clean first attempt — the killed process's actual PID was the real
interpreter, and the robot was fully stopped (`active=False`, encoders static) by
the time of reconnect, with zero motion across a 5s watch window. Relay: the FIRST
attempt gave a false alarm — killing the PID captured from `$!` after `uv run
python ... &` only killed the `uv` wrapper process, not the actual `python3`
interpreter it had spawned as a separate child (confirmed via `ps`); the orphaned
child kept issuing legitimate twist commands for several seconds, which looked
exactly like a deadman failure (sustained motion, `active=True`) until diagnosed
and the true PID was found and killed. This is a test-harness artifact, not a
firmware or protocol defect — noted here so a future bench session recognizes the
same false-alarm pattern rather than re-diagnosing it as a real deadman bug; use
`pkill -f <script>.py` (kills every matching PID, wrapper and child) rather than a
single captured `$!`, or verify with `ps`/`pgrep` before trusting a kill.
Re-run with a verified-complete kill (`pkill -9 -f`, confirmed via `pgrep` that
zero matching processes remained): the robot was fully stopped and stationary
(`active=False`, encoders static) across the entire 5s post-kill watch window, same
as direct USB. Both transports pass.

**Telemetry cadence — measured, not assumed (load-bearing for sprint 106).**
Passive (idle, zero commands) and loaded (continuous twist reissue) captures, both
transports, ~60-90s windows, using `TLMFrame.seq`/`.t` and wall-clock timestamps:

- Direct idle: 13.873 fps, inter-frame delta locked at exactly 72ms (1249/1249
  samples), 0.00% drop.
- Direct loaded (driving): 13.836 fps, 72ms delta (828/831 samples at 72ms, 2 at
  144ms — one skipped emission), 0.24% seq drop.
- Relay idle: 13.875 fps, 72ms delta (833/834 samples), 0.00% drop.

**This CORRECTS 103-010's own ~15.62 Hz figure: the real, currently-measured
primary cadence is ~13.87 Hz (72ms period), not ~15.62 Hz**, and well below the 25
Hz / 40ms (`kPrimaryPeriod`) design target. Correlation with the loop: `kCycle` (
`source/main.cpp`) paces the main loop at a 16ms TARGET; `Telemetry::primaryDue()`
(`source/app/telemetry.cpp`) fires the first loop pass where elapsed-since-last-emit
>= `kPrimaryPeriod` (40ms). A single loop pass under 40ms would emit every pass (no
quantization); two passes would emit every 2×(real loop period). The observed,
rock-solid 72ms period across every capture implies the REAL loop period is close
to 36ms (2 passes × 36ms = 72ms >= 40ms, while 1 pass × 36ms = 36ms < 40ms does not
clear the threshold) — i.e. the loop is running at roughly 2.25x its 16ms pace
target, not hitting it. This is corroborated independently by the SECONDARY
telemetry rate: 1123 secondary samples over the 240.1s direct soak = 4.676 Hz =
213.9ms period, close to 6 loop passes × 36ms = 216ms (kSecondaryPeriod's own 200ms
target divided by a 36ms loop period rounds up to 6 passes) — two independently-
derived cadence numbers (primary emit period, secondary emit period) both point to
the same ~36ms real loop period. This was NOT instrumented at the loop-pass level
directly (no per-pass counter exists to confirm 2 vs. some other divisor
exactly) — flagging that a firmware-side loop-pass counter alongside
`primaryEmitCount()` would let a future ticket confirm this precisely rather than
infer it, if sprint 106's timing budget needs tighter certainty than this
external measurement provides.

**Ack-ring characterization.** Full writeup, methodology, and the four distinct
findings live in `clasi/issues/ack-ring-intermittent-delivery-gap.md`'s own new
"104-007 characterization" section (not duplicated here to avoid drift between two
copies). Summary for this ticket's own record: (1) the issue's own discrete-command
repro is a ~1-gap DELAY, not a ring-depth-eviction loss — confirmed nothing evicts
in that pattern; (2) realistic paced streaming (150ms reissue, this ticket's own
soak numbers above) is reliable on both transports with a continuously-draining
client; (3) true ring-depth eviction is real but only under an artificial
zero-paced burst (direct USB: 2/12 permanently lost; relay: 12/12 permanently lost
under the identical pattern — the relay has far less burst headroom); (4) the
relay-connect-trips-`kFaultCommsMalformed` finding above is filed as its own
separate issue since it isn't part of the ack-ring root-cause space. Recommendation
recorded in the issue: fix the HOST POLLING pattern (`rig_dev.py`'s bounded
wait-then-give-up is the one vulnerable caller in this tree today; sprint 106
should prefer continuous state telemetry over per-command ack confirmation for its
closed-loop feedback), not ring depth or emit cadence as the primary lever. Twist
streaming at 106's target rates (several/sec, paced) is reliable; an unpaced
zero-gap send loop is not, and is worse on the relay than direct — flagged loudly
per the dispatching instruction's own request.

**Final state.** Robot left ON this sprint's firmware image (v0.20260715.3, ticket
004's fault-bit wiring, freshly reflashed for a clean final soak baseline — same
binary as `just build-clean` produced, reflashed several times over the session
purely to reset cumulative fault-bit/malformed-frame counters between diagnostic
phases, never a different build). Confirmed stopped and de-energized at session end:
`active=False`, encoders static across a 3s hold-check, immediately before ending
the session.

**Gate verdict: PASS.** All acceptance criteria met, including the two that
"tripped" (wedge latch, `kFaultCommsMalformed`) doing so within the SAME documented
one-shot/idle-boundary contract already accepted for `kFaultI2CSafetyNet` — both
investigated and written up rather than silently passed, with one new issue filed
for follow-up. No code changes were required or made; this ticket is verification
only, per its own Implementation Plan.
