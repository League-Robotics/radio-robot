---
id: '002'
title: 'OTOS ticks live: readDue() scheduled slot + Blackboard commit'
status: done
use-cases:
- SUC-001
depends-on:
- '001'
github-issue: ''
issue: restore-pose-estimation-otos-encoders-delayed-camera-fixes.md
completes_issue: true
exception:
  thrown_by: programmer
  thrown_at: '2026-07-12T19:27:46.305118+00:00'
  attempted: 'Read the ticket, architecture-update.md''s Decision 2 (and its D1 pass
    pseudocode), and the referenced hazard doc (.clasi/knowledge/otos-per-pass-i2c-tick-wrecks-motion-timing.md
    -- note: this file does not exist anywhere in this checkout; only the ticket and
    architecture-update.md themselves reference it, confirmed by repo-wide grep --
    flagging separately, not the blocker itself). Read the real source: source/subsystems/nezha_hardware.{h,cpp}
    (confirmed Phase enum/phase_/activeIndex_/anyPolled() match D2''s code block exactly
    -- no naming mismatch), source/hal/otos/otos_odometer.{h,cpp} (confirmed tick()''s
    exact gating order: `if (!initialized_) return;` THEN the kReadPeriod rate-limit
    THEN `lastReadMs_=now; hasRead_=true;`), source/runtime/main_loop.{h,cpp}, source/main.cpp,
    tests/_infra/sim/sim_api.cpp, and source/subsystems/sim_hardware.cpp (confirmed
    SimHardware::tick() already unconditionally calls `odometer_.tick(now)` every
    pass -- no sim-side change needed, verified by reading not assumed). Then, before
    writing the real implementation, empirically tested D2''s exact code block: added
    `readDue(){ return !hasRead_ || (now-lastReadMs_)>=kReadPeriod; }` to OtosOdometer
    and the exact D2 branch (`if (phase_==Phase::REQUEST_DUE && otosOdometer_.readDue(now))
    { otosOdometer_.tick(now); return; }`) at the top of NezhaHardware::tick(), then
    compiled the REAL nezha_hardware.cpp/otos_odometer.cpp against the REAL, unmodified
    tests/sim/unit/nezha_flipflop_harness.cpp using the exact same host-build compile
    command test_nezha_flipflop.py uses, and ran it (baseline first, confirmed 10/10
    scenarios pass unmodified; then with the D2 branch added).'
  conflict: 'Architecture-update.md Decision 2''s own code block, implemented exactly
    as specified together with the ticket''s own exact readDue() spec, causes NezhaHardware::tick()''s
    Nezha (0x10) motor flip-flop to NEVER execute -- not "occasionally lose a slot,"
    but PERMANENTLY stuck at Phase::REQUEST_DUE -- whenever Hal::OtosOdometer was
    never begin()''d, or begin() ran but did not detect the chip (product-ID mismatch/absent).
    Root cause: OtosOdometer::tick() only reaches `lastReadMs_ = now; hasRead_ = true;`
    AFTER its `if (!initialized_) return;` gate (otos_odometer.cpp line ~85); readDue()
    is specified as a pure `!hasRead_ || (now-lastReadMs_)>=kReadPeriod` query with
    NO dependency on initialized_/connected() ("a pure function of its existing private
    hasRead_/lastReadMs_ fields, no new state" -- ticket''s own Implementation Plan).
    So an undetected/never-begun odometer has hasRead_ permanently false, readDue()
    permanently true, and D2''s branch -- placed BEFORE anyPolled()/the phase switch
    -- intercepts and `return`s on every single tick() call where phase_==REQUEST_DUE,
    forever. phase_ never advances to COLLECT_DUE; motors_[activeIndex_].requestSample()
    never fires; no duty write ever reaches the bus. Empirical proof: compiling this
    exact D2 branch into the real nezha_hardware.cpp and running the real, unmodified
    nezha_flipflop_harness.cpp (none of whose 10 scenarios call NezhaHardware::begin()
    -- the same "odometer never detected" state a real robot with OTOS unplugged/undetected
    would be in) produces 16 assertion failures across 6 of 10 previously-100%-passing
    scenarios (e.g. "REQUEST_DUE issued exactly one transaction -- expected 1, got
    0"). This directly violates this SAME ticket''s own acceptance criterion ("the
    Nezha flip-flop''s own request/collect cadence is otherwise unchanged by the new
    branch") and is a materially WORSE hazard than the one this ticket exists to close:
    098-004''s bug was mistimed OTOS traffic during an occasional settle window; this
    D2-as-written bug is total, permanent loss of ALL motor command/encoder traffic
    the instant OTOS is undetected -- and project knowledge (this ticket''s own dispatch
    context) states OTOS was reading connected()==False as of the most recent related
    sprint, i.e. this is not a hypothetical edge case but plausibly the CURRENT bench
    condition. I cannot resolve this without either (a) deviating from D2''s exact,
    explicitly-cited code block (e.g. gating the branch on connected()/initialized_
    too, or tracking a distinct "ever attempted" flag) or (b) leaving a known, verified
    total-motor-starvation regression in place -- both are upstream architecture calls,
    not implementation choices within this ticket''s discretion.'
  surface: internal
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# OTOS ticks live: readDue() scheduled slot + Blackboard commit

## Description

`Hal::OtosOdometer::begin()` runs at boot but `tick()` is never called by
the live loop — `bb.otos`/`bb.otosConnected`/`bb.otosPresent` sit at their
zero defaults forever. A prior attempt to tick it unconditionally,
per-pass, directly from the loop (ticket 098-004, mirroring the pre-093
`dev_loop.cpp` pattern) caused a real bus-hang-class regression (a live
turn over-rotated `-90 -> -192deg`) because OTOS (I2C 0x17) traffic could
start while an outstanding Nezha motor (I2C 0x10) 0x46 request was still
in its settle window — see architecture-update.md's Decision 2 for the
full incident writeup (the doc previously cited here,
`.clasi/knowledge/otos-per-pass-i2c-tick-wrecks-motion-timing.md`, does
not exist anywhere in this checkout — dropped as a dead cite; Decision 2's
own prose is the authoritative, in-repo record of the 098-004 incident).

This ticket implements D2 **as revised by architecture-update-r1.md**: the
OTOS tick becomes a **scheduled slot inside `NezhaHardware::tick()`**,
gated on BOTH bus phase and chip presence — structurally, not
probabilistically, ruled out from ever landing inside a 0x10
request/collect window, AND structurally ruled out from ever preempting
the flip-flop when no OTOS chip was ever detected — plus commits the
resulting raw OTOS state onto the Blackboard every pass.
`Telemetry::tick()`/`buildTelemetryMessage()` (`source/telemetry/
tlm_frame.cpp`) already read `bb.otos`/`bb.otosConnected` (gated on
`bb.otosPresent`) — this ticket needs **zero changes** to those files;
`otos=`/`otosconn=` light up on TLM automatically once these cells are
committed.

**Revision note (architecture-update-r1.md)**: the original D2 branch
(`phase_ == REQUEST_DUE && otosOdometer_.readDue(now)`) was implemented
and empirically tested exactly as specified by the previous attempt at
this ticket, and was found to permanently stall the Nezha flip-flop
(zero motor/encoder traffic, forever) whenever the OTOS was never
`begin()`'d or never detected — the current, project-memory-confirmed
bench state. See architecture-update-r1.md Decision 2 for the full
root-cause analysis and why the fix is a new `OtosOdometer::present()`
query (permanent, boot-time-only), not `connected()` (live, re-evaluated
every `tick()` call, and — per that document's own analysis — unsafe to
reuse for this gate since it would newly latch the OTOS itself
permanently unread after a single transient bus glitch on an otherwise
healthy, present chip).

This ticket does **not** touch `PoseEstimator` or `bb.otosValid` (the
fusion-gating flag `Hal::Odometer::fusableThisPass()` produces) — that is
ticket 004/007's job, once `PoseEstimator` is actually consuming an OTOS
observation. This ticket is purely: "the chip gets read safely, and its
raw reading + connection state are visible."

## Acceptance Criteria

- [x] `Hal::OtosOdometer` gains a new public `bool readDue(uint32_t now)
      const` query (`!hasRead_ || (now - lastReadMs_) >= kReadPeriod`,
      signed-cast rollover-safe, matching the project's established
      uint32-ms-subtraction convention). Pure function of existing private
      state — unchanged from the original spec; `readDue()` itself is NOT
      the fix (see next bullet).
- [x] **(architecture-update-r1.md)** `Hal::OtosOdometer` gains a second
      new public query, `bool present() const { return initialized_; }` —
      `true` once (and permanently, never re-evaluated after) `begin()`'s
      PRODUCT_ID detect succeeds. Distinct from `connected()`: `present()`
      never changes after `begin()`, `connected()` is re-evaluated every
      `tick()` call from that call's own bus-read result. Declare next to
      `connected()` in `otos_odometer.h`; define next to `connected()`'s
      own one-line definition in `otos_odometer.cpp`.
- [x] `NezhaHardware::tick()` gains one new branch at its top: when
      `phase_ == Phase::REQUEST_DUE && otosOdometer_.present() &&
      otosOdometer_.readDue(now)`, this call services the OTOS
      (`otosOdometer_.tick(now)`) and returns immediately — never entering
      the existing flip-flop switch this call. The existing flip-flop
      switch is otherwise byte-identical. **The `present()` conjunct is
      mandatory** — omitting it (the original, pre-revision spec) causes
      an undetected/never-`begin()`'d odometer to permanently stall the
      flip-flop; see architecture-update-r1.md Decision 2 for the verified
      root cause. Do not gate on `connected()` instead of `present()` — see
      the same document for why that substitution introduces its own
      (smaller, but real) regression.
- [x] `Rt::MainLoop::commit()` gains: `bb.otos =
      hardware_.odometer()->pose();` and `bb.otosConnected =
      hardware_.odometer()->connected();`, every pass. (Unchanged from the
      original spec — `bb.otosConnected` legitimately wants the live,
      re-evaluated-every-pass `connected()` value; only the
      `NezhaHardware`-internal scheduling gate needed `present()`.)
- [x] `bb.otosPresent` is seeded exactly once, at boot, immediately after
      `hardware.begin()` (both `main.cpp` and `tests/_infra/sim/
      sim_api.cpp`'s `SimHandle` constructor): `bb.otosPresent =
      hardware.odometer()->present();` **(architecture-update-r1.md: use
      `present()`, not `connected()`, here** — both happen to read the same
      value at this exact call site today since no `tick()` has run yet,
      but `present()` is the semantically exact, seed-once,
      boot-time-never-changing fact `blackboard.h`'s own comment on
      `otosPresent` documents; using it removes a coincidence this line
      would otherwise silently depend on).

      **Implementation-fill beyond r1's literal text**: this call site
      goes through `hardware.odometer()`, which returns the ABSTRACT
      `Hal::Odometer*` base pointer (not the concrete `OtosOdometer`) —
      matching this codebase's own established `bb.otosPresent`
      polymorphism precedent (`hardware.h`'s own file header: "main.cpp's
      bb.otosPresent snapshot" already went through this same base-pointer
      accessor pre-090-003). Since r1's own code block only added
      `present()` to `Hal::OtosOdometer` (never to the `Hal::Odometer`
      base), `hardware.odometer()->present()` did not compile as written —
      confirmed by actually building, not assumed. Resolved by adding
      `virtual bool present() const { return true; }` to `Hal::Odometer`
      (`hal/capability/odometer.h`, next to `connected()`) as a
      convenience default (mirrors `begin()`'s own existing
      "no caller needs polymorphic X semantics for every hypothetical
      owner" default) — `Hal::OtosOdometer` overrides it with the real
      `initialized_`-backed logic (unchanged from r1's own text);
      `Hal::NullOdometer` overrides it `false` (mirrors its own
      always-false `connected()`); `Hal::SimOdometer` needed **no file
      change at all**, inheriting the `true` default — which is also the
      semantically correct answer (no physical chip to ever fail to
      detect, the same rationale as its own hardcoded `connected()==true`)
      and keeps r1's own "`SimOdometer`/`sim_hardware.cpp` need no
      matching change" claim true in the sense that mattered (no sim
      FILE was touched; the literal "Files NOT to modify" list technically
      named `sim_odometer.{h,cpp}` too, but r1's OWN stated rationale for
      that line — "nothing in `SimHardware::tick()` ever asks" — was
      narrowly about the scheduling-gate concern and did not anticipate
      this base-pointer boot-seed compile requirement, which the ticket's
      own literal AC for this bullet independently demands for BOTH
      `main.cpp` and `sim_api.cpp`). Judged as implementation-fill, not an
      architecture override: additive-only, zero behavior change to any
      existing `tick()`/`connected()`/`pose()` call, and directly required
      to make this ticket's own explicit AC compile — not thrown as a
      second exception; flagged here and in the closing implementation
      summary for team-lead review.
- [x] `SimHardware`/`SimOdometer` need no matching change — confirmed by
      reading (not assuming) `sim_hardware.cpp`: `SimHardware::tick()`
      ticks its odometer unconditionally, with no phase/schedule gate of
      any kind (it is a different concrete class from `NezhaHardware` and
      shares none of this ticket's new gating logic); confirmed
      `SimOdometer::connected()` is hardcoded `true` (no I2C link to fail).
      The OTOS slot fires every pass in sim, unconditionally — this
      ticket's `present()`/`readDue()` gate has zero effect on sim. (See
      the implementation-fill note on the `bb.otosPresent` bullet above:
      `sim_odometer.{h,cpp}` themselves were NOT touched — `SimOdometer`
      inherits `Hal::Odometer`'s new `present()==true` convenience default
      unmodified.)
- [x] New/extended `otos_odometer_harness.cpp` case(s) for `readDue()`:
      false immediately after a real read, true once `kReadPeriod` has
      elapsed, true before any read has ever happened. (Unchanged by the
      revision — `readDue()`'s own implementation and unit-test AC are not
      touched by the `present()` fix.)
- [x] New/extended `otos_odometer_harness.cpp` case(s) for `present()`:
      `false` before `begin()` is ever called and after a `begin()` whose
      product-ID detect fails (mock returns a wrong ID); `true` after a
      `begin()` whose detect succeeds; stays `true` even after a
      subsequent `tick()` call whose own bus read fails (`present()` must
      NOT track `connected_`).
- [x] New/extended `nezha_flipflop_harness.cpp` case(s):
  - [x] **Regression test for the exact exception found**: an
        `otosOdometer_` that is never `begin()`'d (the harness's own
        existing default construction — matching every pre-existing
        scenario) must leave the flip-flop's request/collect cadence
        COMPLETELY unaffected — re-run the full existing 10-scenario suite
        unmodified and confirm 10/10 still pass with the new branch
        compiled in (this is the scenario that regressed to 6/10 passing
        under the pre-revision D2 branch). CONFIRMED: 10/10 still pass.
  - [x] The OTOS slot never fires while `phase_ == COLLECT_DUE`.
  - [x] At most one OTOS slot services per `kReadPeriod` window, when the
        odometer IS present (mock a successful `begin()`).
  - [x] **Transient-failure retry test**: with the odometer present
        (`begin()` succeeds) but a mocked bus read that fails on one
        `tick()` call (`connected()` goes `false` that call), confirm the
        OTOS branch still fires again on the next `kReadPeriod` boundary —
        i.e. `present()`-gating, unlike a `connected()`-gated design would,
        does not let one transient bus error permanently stop OTOS
        polling.
- [ ] **BENCH MANDATORY, DEFERRED**: sustained (>=10 minute) bench session
      with 0x17 (OTOS) and 0x10 (Nezha motor) traffic interleaved — zero
      bus hangs, verified via `robot_radio`'s `NezhaProtocol` (never
      lock-step pyserial, per prior bench-session lessons). Given the
      current bench state (OTOS reads `connected()==False`), this
      session's PRIMARY evidence is that the Nezha motor/encoder traffic
      runs completely normally throughout — the direct, on-hardware
      confirmation that the exception's failure mode does not reproduce.
      **Not run this session** — sim/unit coverage above (including the
      two new `nezha_flipflop_harness.cpp` scenarios that exercise both
      device addresses on one shared bus in exact chronological order) is
      the acceptance evidence for this pass; the bench gate remains open
      for a follow-up HITL session per `.claude/rules/hardware-bench-
      testing.md`.
- [ ] **DEFERRED**: The SAME session, with motion commands running
      throughout (binary `drive`/`segment`), shows no motion-timing
      regression versus a pre-ticket baseline — the 098-004 hazard class
      does not reproduce. Not run this session — see the bench-mandatory
      bullet above.
- [ ] **DEFERRED**: `TLM`/binary `stream` shows `otosconn=`/`otos=`
      live-updating (or a truthful `false`/omitted if no chip is detected)
      on the bench. Not run this session — see the bench-mandatory bullet
      above.

## Implementation Plan

**Approach** (revised per architecture-update-r1.md): (1) add the
`readDue()` query to `OtosOdometer` (`source/hal/otos/
otos_odometer.{h,cpp}`) — a pure function of its existing private
`hasRead_`/`lastReadMs_` fields, no new state; unchanged from the original
plan. (2) Add the `present()` query to `OtosOdometer`, alongside
`readDue()` and `connected()` — `return initialized_;`, no new state
either (it exposes an existing private field, it does not add one). (3)
Add the scheduled-slot branch to `NezhaHardware::tick()`
(`source/subsystems/nezha_hardware.cpp`) exactly as specified in
architecture-update-r1.md's revised Decision 2 code block — note the
`present()` conjunct, not present in the original architecture-update.md
text. (4) Extend `Rt::MainLoop::commit()` (`source/runtime/main_loop.cpp`)
with the two new `bb.otos*` assignments (unchanged from the original
plan — these use `pose()`/`connected()`, not `present()`). (5) Seed
`bb.otosPresent` once in `source/main.cpp` and `tests/_infra/
sim/sim_api.cpp`'s `SimHandle::SimHandle()`, immediately after
`hardware.begin()`, using `present()` (changed from `connected()` — see
AC).

**Files to modify**:
- `source/hal/otos/otos_odometer.h` — declare `readDue()` AND `present()`.
- `source/hal/otos/otos_odometer.cpp` — implement `readDue()` AND
  `present()`.
- `source/subsystems/nezha_hardware.cpp` — new top-of-`tick()` branch,
  three-way `&&` (`phase_ == REQUEST_DUE && present() && readDue(now)`).
- `source/runtime/main_loop.cpp` — `commit()` gains `bb.otos`/
  `bb.otosConnected`.
- `source/main.cpp` — seed `bb.otosPresent` once, post-`begin()`, from
  `present()`.
- `tests/_infra/sim/sim_api.cpp` — seed `bb.otosPresent` once,
  post-`hardware.begin()`, from `present()`, mirroring `main.cpp`.

**Files NOT to modify**: `source/telemetry/tlm_frame.{h,cpp}`,
`source/telemetry/telemetry_tick.cpp`, `protos/telemetry.proto` — already
correct (verified by reading, D9 in architecture-update.md).
`source/subsystems/sim_hardware.cpp`, `source/hal/sim/sim_odometer.{h,cpp}`
— confirmed by reading (architecture-update-r1.md's sim-path section):
`SimHardware::tick()` has no schedule gate at all and `SimOdometer::
connected()` is hardcoded `true`; this ticket's `present()`/`readDue()`
gate is `NezhaHardware`-only and has no sim-side counterpart to add.

**Testing plan**:
- Extend `tests/sim/unit/otos_odometer_harness.cpp` for `readDue()` (as
  originally planned) AND for `present()` (new — false pre-`begin()`/
  after a failed detect, true after a successful detect, stays true across
  a subsequent failed `tick()` read — see AC).
- Extend `tests/sim/unit/nezha_flipflop_harness.cpp` for: the
  never-fires-during-`COLLECT_DUE` and at-most-one-per-`kReadPeriod`
  invariants (as originally planned); the regression case for the
  exception (full existing 10-scenario suite, no `begin()` called, must
  stay 10/10 with the new branch compiled in); and the transient-failure
  retry case (present but one failed read does not permanently stop
  future OTOS slots) — see AC for all three.
- Bench session per the acceptance criteria above — this is the sprint's
  first mandatory bench gate; do not skip or shorten the >=10 minute
  window. Given the current bench state (OTOS not detected), this session
  now primarily verifies the Nezha motor/encoder path is completely
  unaffected by an undetected OTOS — the exact scenario the exception
  found broken pre-revision — in addition to the original bus-hang-timing
  hazard check.

**Documentation updates**: none required this ticket (the TLM field
semantics are unchanged, only their liveness). The dead
`.clasi/knowledge/otos-per-pass-i2c-tick-wrecks-motion-timing.md` citation
in this ticket's own Description has been replaced with a pointer to
architecture-update.md's Decision 2 prose, which already contains the full
098-004 incident writeup in-repo.
