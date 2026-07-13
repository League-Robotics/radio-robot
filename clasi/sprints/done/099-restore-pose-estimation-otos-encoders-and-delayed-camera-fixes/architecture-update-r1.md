---
sprint: 099
status: in-progress
revises: architecture-update.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Architecture Update r1 -- Sprint 099: Decision 2's OTOS scheduled-slot gate needs a boot-presence check, not just a phase check

This is a **focused revision**, triggered by a ticket 002 exception (`thrown_by:
programmer`, `surface: internal`) discovered during implementation, before any
code merged. It does not restate `architecture-update.md` — read that document
first for the full Sprint 099 design (Steps 1-7, Decisions 1-6). This document
revises **Decision 2 only**. `architecture-update.md` itself is preserved
unmodified as the calibration record of what was originally planned; this
document is now the active planning artifact for ticket 002.

## The discovery

Ticket 002's programmer implemented Decision 2's code block exactly as
written — `if (phase_ == Phase::REQUEST_DUE && otosOdometer_.readDue(now)) {
otosOdometer_.tick(now); return; }` at the top of `NezhaHardware::tick()` —
together with the ticket's own exact `readDue()` spec (`!hasRead_ || (now -
lastReadMs_) >= kReadPeriod`, "a pure function of its existing private
`hasRead_`/`lastReadMs_` fields, no new state"), then empirically tested it
against the real, unmodified `tests/sim/unit/nezha_flipflop_harness.cpp`
before writing anything else.

The combination has a real bug, verified by source reading and by a compiled
repro, not merely suspected:

- `Hal::OtosOdometer::tick()` only reaches `lastReadMs_ = now; hasRead_ =
  true;` **after** its `if (!initialized_) return;` gate
  (`source/hal/otos/otos_odometer.cpp:85`).
- `initialized_` is set once, in `begin()`, from a PRODUCT_ID I2C detect —
  `false` forever if the chip is absent or undetected.
- So on an odometer that was never `begin()`'d (every one of
  `nezha_flipflop_harness.cpp`'s 10 scenarios) **or** whose `begin()` ran but
  did not detect the chip — the current, project-memory-confirmed bench
  state ("OTOS currently connected=False") — `hasRead_` never becomes `true`,
  `readDue()` returns `true` on *every* call, forever, and D2's branch —
  placed **before** the existing flip-flop switch — intercepts and `return`s
  on every single `NezhaHardware::tick()` call where `phase_ ==
  REQUEST_DUE`. `phase_` never advances to `COLLECT_DUE`;
  `motors_[activeIndex_].requestSample()` never fires; **no duty write and
  no encoder poll ever reaches the 0x10 bus again.**
- Compiled repro: the real `nezha_hardware.cpp`/`otos_odometer.cpp` with D2's
  exact branch added, run against the real, unmodified
  `nezha_flipflop_harness.cpp` (baseline 10/10 pass) produces 16 assertion
  failures across 6/10 scenarios (e.g. "REQUEST_DUE issued exactly one
  transaction — expected 1, got 0").

This is strictly worse than the hazard ticket 002 exists to close: 098-004's
bug was mistimed OTOS traffic during an occasional settle window; D2-as-written
is **total, permanent loss of all motor command and encoder traffic** the
instant the OTOS is undetected — plausibly the actual current bench condition,
not a hypothetical edge case. It also directly violates the ticket's own
acceptance criterion ("the Nezha flip-flop's own request/collect cadence is
otherwise unchanged by the new branch"). Gating the branch on chip presence,
not just bus phase, is an upstream architecture correction (this document),
not an implementation choice within the ticket's discretion — exactly the
threshold the Exception Protocol exists for.

## Decision 2 (revised): the scheduled-slot branch gates on a permanent,
boot-time presence flag — never on the live, per-tick `connected()` flag

**Context**: D2's structural claim — "fold the OTOS's own scheduling decision
into the one class that already owns bus-scheduling policy for this bus,
gated on its own phase state" — is unchanged and correct; the flaw is narrower
than that claim: the branch has no guard at all for "was a chip ever detected
at this address," so an absent/undetected OTOS turns a scheduling gate into a
permanent lockout of the entire flip-flop.

**Alternatives considered**:

1. **Gate on `otosOdometer_.connected()`** (my own first instinct, and the
   fix the exception's dispatch brief proposed as a starting point):
   `if (phase_ == Phase::REQUEST_DUE && otosOdometer_.connected() &&
   otosOdometer_.readDue(now)) { otosOdometer_.tick(now); return; }`.
   This does close the reported hazard — `connected()` returns `initialized_
   && connected_`, and `connected_` is seeded to `initialized_` in `begin()`
   before any `tick()` runs, so an undetected chip has `connected()` false
   forever and the branch never fires.

   **Rejected on closer reading of `otos_odometer.cpp`.** `connected_` is
   **not** a boot-time-only flag — `tick()` reassigns it every call
   (`connected_ = ok;`, line 110) from that call's own bus-read result, and
   the class's own doc comments are explicit that this is deliberate: "Live
   per-tick bus health — always re-evaluated... a transient bus glitch does
   not permanently disable further attempts" (`otos_odometer.h:271-272`,
   `otos_odometer.cpp:108-110`). Gating the *caller* (`NezhaHardware::tick()`)
   on `connected()` breaks that documented retry contract one level up: if a
   *present, detected* chip has one single bus hiccup on some read
   (`ok == false` for one call), `connected_` flips to `false`, `connected()`
   goes `false`, and the very next `NezhaHardware::tick()` call no longer
   satisfies the gate — so `otosOdometer_.tick()` is never called again,
   `connected_` is never re-evaluated again (only `tick()` touches it), and
   the OTOS silently stops being read forever, from a single transient I2C
   error, even though the chip is fine. This is a strictly smaller hazard
   than the one this ticket closes (the Nezha flip-flop is completely
   unaffected — only the OTOS itself goes permanently stale), but it is a
   real, self-inflicted regression against `OtosOdometer`'s own documented
   invariant, introduced by naively reusing `connected()` as a caller-side
   gate rather than as the leaf's own per-tick bus-health report it was
   designed to be. Not selected.

2. **Add a new, narrow, permanent presence query — `bool present() const`
   — and gate on that instead.** `present()` returns `initialized_` alone:
   `true` once (and only once) `begin()`'s PRODUCT_ID detect succeeded, never
   reassigned afterward by `tick()` or anything else — the exact "was a chip
   ever detected at this address" fact the gate actually needs, cleanly
   separated from `connected()`'s live, retried-every-call bus-health
   semantics. *Chosen.*

**Why the chosen alternative**: `present()` and `connected()` answer two
different questions that `OtosOdometer` already, internally, tracks with two
different fields (`initialized_` vs `connected_`) for exactly this reason —
the class already has the right state, it just never exposed the
boot-permanent half of it as its own query. Gate 2 costs one trivial new
accessor (no new state, no new bus traffic, mirrors `readDue()`'s own "pure
function of existing private fields" shape) and gets both properties D2
needs simultaneously: structurally impossible for OTOS traffic to preempt an
undetected/absent chip's flip-flop slot forever (fixes the reported
exception), **and** structurally impossible for a transient bus error to
permanently stop OTOS polling once the chip is genuinely present (avoids
introducing a new, smaller latent regression). `readDue()` itself needs no
change and keeps its ticket-specified pure `!hasRead_ || (now - lastReadMs_)
>= kReadPeriod` shape — presence and cadence stay two independent,
separately-testable concerns, composed by `&&` at the one call site that
needs both, matching this document's `readDue()`-stays-pure preference
(rejecting the "fold presence into `readDue()`" alternative for the same
reason the original dispatch brief leaned against it: it would conflate "is
a read due" with "is the chip present," and would make the ticket's own
"true before any read has ever happened" unit-test wording state something
that is no longer literally true for an odometer that will never be
`begin()`'d).

**Revised code block** (`source/subsystems/nezha_hardware.cpp`,
`NezhaHardware::tick()`, same position — top, before the existing
`anyPolled()`/phase switch):

```cpp
if (phase_ == Phase::REQUEST_DUE && otosOdometer_.present() &&
    otosOdometer_.readDue(now)) {
    otosOdometer_.tick(now);
    return;   // this call's bus action; the Nezha flip-flop resumes next call
}
```

`Hal::OtosOdometer` gains one new public query alongside `readDue()`:

```cpp
// True once PRODUCT_ID matched at begin(); permanent for the life of this
// object (never re-probed, never reassigned by tick() or anything else) --
// unlike connected(), which is re-evaluated every tick() call and can go
// false from a single transient bus read failure. This is the query a
// CALLER should use to decide "is there a chip here worth scheduling a slot
// for at all," never connected() -- see architecture-update-r1.md Decision 2
// for why conflating the two caused a real regression.
bool present() const { return initialized_; }
```

(Declared in `otos_odometer.h` next to `connected()`; a one-line inline
accessor, or defined in `otos_odometer.cpp` next to `connected()`'s own
one-line definition — implementer's choice, matching the file's existing
style for trivial accessors either way.)

**Consequences**:

- `NezhaHardware`'s public interface is still unchanged (the fix is entirely
  inside `tick()`, as D2 originally claimed) — only `OtosOdometer` gains the
  one new query.
- An odometer that is never `begin()`'d, or whose `begin()` fails to detect
  the chip, now behaves exactly like D2's original intent describes for the
  *bus-collision* hazard, but additionally never touches the flip-flop's
  cadence at all — `present()` false is a permanent, harmless no-op for the
  branch, and every `nezha_flipflop_harness.cpp` scenario (none of which call
  `begin()`) is unaffected byte-for-byte, closing the exception.
- A chip that IS present keeps `OtosOdometer`'s own documented
  transient-failure-does-not-latch behavior intact at the caller level too:
  `present()` stays `true` regardless of any single bus read's `ok` result,
  so the branch keeps offering `otosOdometer_` a slot every `kReadPeriod`
  even after an isolated I2C hiccup — `connected()`/`connected_` continue to
  report that call's live bus health on `bb.otosConnected` exactly as D1
  already specifies, unaffected by this change.
- `bb.otosPresent`'s own seeding (ticket 002's AC, `source/main.cpp` /
  `tests/_infra/sim/sim_api.cpp`, immediately post-`begin()`) is **more
  precisely** expressed as `hardware.odometer()->present()` than as
  `hardware.odometer()->connected()`: at that exact call site the two
  happen to read the same value today (nothing has called `tick()` yet, so
  `connected_ == initialized_` still holds), but `present()` is the
  semantically exact, seed-once, "boot-time, never-changing hardware-identity"
  fact `blackboard.h`'s own comment on `otosPresent` already documents —
  using it removes an incidental coincidence the seeding line was silently
  relying on. This is a one-token AC correction, not a new requirement.
- `Subsystems::SimHardware` needs no matching change, for a different reason
  than D2 originally gave (see the sim-path note below) — its own `tick()`
  has no phase/schedule gate of any kind and never gained one.

## Sim-path verification (explicit, per this revision's mandate)

D2's original text asserted "`Subsystems::SimHardware` already ticks its own
odometer internally (081-003) — sim and hardware converge without a matching
sim-side change," without walking the sim source. Verified now, by reading:

- `SimHardware::tick()` (`source/subsystems/sim_hardware.cpp:32-55`) has no
  `readDue()`/`present()`/phase-gate of any kind on its odometer call —
  every motor ticks, `plant_.update(dt)` runs, then `odometer_.tick(now)`
  runs **unconditionally**, once per `SimHardware::tick()` call (guarded only
  by the class's own `dt == 0` re-entry no-op, which is a re-entrancy guard,
  not a presence/cadence gate). `SimHardware` and `NezhaHardware` are
  unrelated concrete classes under the common `Hardware` base — this
  revision's new `present()` gate lives inside `NezhaHardware::tick()`
  only and has **zero effect on `SimHardware`**, confirming D2's original
  claim was right, just previously unverified.
- `Hal::SimOdometer::connected()` (`source/hal/sim/sim_odometer.cpp:21`)
  is hardcoded `bool SimOdometer::connected() const { return true; }` —
  "always true — no I2C link to fail" per its own header comment
  (`sim_odometer.h:49`). `SimOdometer` has no `initialized_`/`present()`
  analog at all; it needs none, since nothing in `SimHardware::tick()`
  ever asks.

**Decision, stated explicitly**: the OTOS slot **fires every pass in sim**,
unconditionally — there is no sim-side scheduling slot to gate in the first
place, so ticket 007's fusion-enable work and any sim-side pose-fusion test
built on top of it see a live, fresh OTOS sample every `SimHardware::tick()`
call, exactly as they would have under the original (also-correct-here) D2
text. No sim-side ticket needs any change as a result of this revision.

## Interaction with ticket 007 (fusion enable) and `bb.otosPresent`

Checked explicitly, since both were named as things this revision must not
silently break:

- **Ticket 007** does not read `present()`, `connected()`, or any
  `NezhaHardware`-internal state directly — it reads
  `hardware_.odometer()->fusableThisPass()` and `hardware_.odometer()->
  pose()` once per pass from `Rt::MainLoop::tick()` (`Hal::Odometer` base
  interface, `hal/capability/odometer.h`), and gates `otosObs` on
  `otosSample.stamp.valid`. When the OTOS is undetected, `present()` is
  `false` forever, `OtosOdometer::tick()` is never called by `NezhaHardware`,
  `cachedPose_` never leaves its zero-initialized, `stamp.valid == false`
  default — so `pose()` keeps returning a stale-marked sample and ticket
  007's own existing gate (`otosObs->stamp.valid`) already, correctly,
  never fuses it. No change needed in ticket 007's own ticket file; this
  revision only strengthens the guarantee that an undetected OTOS produces
  an honestly-stale sample forever, rather than (pre-fix) a hung robot.
- **`bb.otosPresent`**: see Decision 2's Consequences above — the seeding
  expression tightens from `connected()` to `present()`, a same-value,
  more-precise substitution at the one call site that sets it, with no
  behavior change at boot (both read the same value there today) and a
  correctness improvement against future drift (e.g. if a future change
  ever reordered a `tick()` call ahead of the seed line, `connected()` could
  silently diverge from the intended "boot-time-only" meaning; `present()`
  cannot, by construction).

## Status

This revision narrows ticket 002's own scope by one accessor and one
guard-clause edit; it does not touch any other module's boundary, the
dependency graph, or Decisions 1/3-6. No full architecture self-review round
is required for this scope of change — Decision 2's structural claim (bus
scheduling policy lives in `NezhaHardware`, gated on its own phase state) is
unchanged and still correct; only its missing presence guard is added. Ticket
002 is revised separately (same file, AC/Implementation Plan updated
in-place) to carry out the concrete change above; its `status: exception` and
recorded `exception:` block are left untouched for the team-lead to review
and reopen.
