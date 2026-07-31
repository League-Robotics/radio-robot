# Code Review Guidelines

**Status:** adopted 2026-07-30 (stakeholder-approved)
**Scope:** all code this project controls — firmware (`src/firm`, `src/motion`),
host (`src/host`), tests (`src/tests`), sim (`src/sim`), tooling.
**Companions:** `.claude/rules/coding-standards.md`, `.claude/rules/naming-and-style.md`,
`docs/reference/google-cppguide-condensed.md`. Those documents own mechanical style
(naming case, units-in-comments, header layout). This document owns **design
craftsmanship and correctness** — the things a linter can't see.

---

## 0. The standard

The bar is **obsessive craftsmanship**. Every class, and every connection between
classes, should be as close to textbook as we can manage — the kind of code you'd
put on a slide and be proud of. The tests are:

1. **Each class is understandable alone.** You can read one class, know its job,
   and reason about its correctness without opening three neighbors.
2. **The connections between classes are explicit and few.** You can draw the
   arrows. Each arrow carries one clearly-named kind of thing.
3. **Nothing is where it doesn't belong.** No orchestration in the wrong function,
   no policy in a leaf, no test scaffolding in the product tree.

Most of what this codebase's agents get wrong is not style — it is **accretion**:
solving problem upon problem by layering another fix on top instead of going back
and re-asking what the design is supposed to look like. The result is interfaces
that are dead on arrival — still present, still exported, but violated by their
own callers. A review's first job is to catch accretion before it merges;
its second is to name the refactor that removes it.

---

## 1. Crisp interfaces that don't bleed

An interface is a promise about who knows what. "Bleed" is any code that breaks
that promise while leaving the interface standing.

**Review questions:**

- **Does anyone reach around the interface?** If a class exposes `apply()`/`tick()`
  but a caller pokes its fields, calls its privates via a friend, or reads state
  out of its output buffer, the interface is dead — flag it, and flag the design
  that made reaching around easier than going through.
- **Does the interface leak its implementation?** Parameters or return types that
  only make sense if you know the internals (a raw index into a private array, a
  flag meaning "skip the thing I do internally") are bleed in the other direction.
- **Is there exactly one way in?** Two entry points to the same behavior — a real
  one and a "convenience" one added under pressure — is the signature of a layered
  hack. One Sim object, one dispatch path, one command plane. If a second path
  exists "temporarily," the review verdict is: pick one, delete the other.
  (History: three divergent sim capture paths in `turn_shape.py` gave three
  different answers and burned a day; the 2026-06-11 review found *three*
  independent go-to-point stacks and three pose estimators with no owner.)
- **Does data flow in one direction through it?** A serialization artifact (a
  telemetry frame, a wire message, a config snapshot) is assembled once, from
  primary sources, immediately before it crosses the boundary — never used as a
  scratchpad, never read back as an input to something upstream. If module B is
  fed by copying fields back out of module A's *output* buffer, that is
  laundering, not integration. (History: the 2026-07-25 `robot_loop.cpp`
  correction — the StateEstimator was being fed 16 fields copied out of the
  outbound TLM frame.)
- **Do layers point the right way?** Lower layers never include, name, or format
  for upper layers. Control code that snprintf's wire replies, a device leaf that
  knows the app's queue type — these are the A2-class inversions that turned into
  field defects (the duplicate-OK bug). The boundary interface pattern we use
  (`Motion::WheelSink`) is the model: one narrow, named seam, defined by the
  lower layer, implemented by the upper.

## 2. Code in the right place

The most common craftsmanship failure in this codebase: a class orchestrating
*another* class's business in its own body — usually because that was the file the
author happened to have open. It reads as: long call chains into a collaborator,
copies of the collaborator's decision logic, the same orchestration sequence
appearing in two or three call sites.

**Review questions:**

- **Feature envy.** Does a method spend most of its lines reading/steering one
  other object? Then it belongs on that object. Move it; don't wrap it.
- **Orchestration has one home.** A multi-step sequence over a class's primitives
  (configure → arm → run → settle) is itself a behavior, and it belongs *in that
  class or in one designated conductor* — never re-spelled at each call site.
  If the review finds the same three-call ritual in two places, that's a missing
  method, and the finding is "add the method, collapse the call sites."
- **Policy lives in the base tier; primitives live in the leaf.** The placement
  question is always: "is this about *this vendor's hardware*, or about *motors*?"
  Protection, health, retry, dwell, deadband — generic policy, base class. Bus
  transactions, register maps, vendor timing quirks — leaf. (Stakeholder ruling,
  sprint 078: wedge/reversal armor goes in `Hal::Motor`, not `NezhaMotor`.)
- **Setters are the primitives; verbs are built on them** — never the reverse.
  `apply(Command)` unpacks and calls setters; `state()` assembles from getters.
  A setter implemented as sugar over `apply()` is upside-down.
- **`apply()` stages, `tick()` executes.** No hardware I/O, no emission from
  `apply()`. One verb (`tick`) for "advance on a time interval," everywhere, at
  every scale.
- **Test and diagnostic code lives in `src/tests/`** (sim/bench/playfield
  domains), never inside the importable host package or the firmware product
  tree — even when `-m` import convenience tempts otherwise.
- **Counting lines is a valid review instrument.** Misplaced code is usually
  *more* code: orchestrating a foreign class from outside takes more lines than
  doing it inside. When a diff feels big for what it does, look for the class it
  should have been in.

## 3. Accretion: hacks layered on hacks

When a fix lands, the question is not "does it work" but "is this the code the
design would have produced?" A patch that special-cases its way around a design
problem leaves the problem in place and adds a liability on top.

**Review questions / signatures of accretion:**

- **A flag that selects between an old and a new behavior** in the same binary.
  We do not run two live systems behind runtime arbitration — "if both lines of
  code are compiled together, one of them is unwired." Cutover is complete: one
  swap, old tree parked or deleted.
- **A special case at the call site** that exists to compensate for a
  collaborator's known deficiency (an extra sleep, a re-read, a re-send, a
  clamp). The fix belongs in the collaborator; the call-site workaround is debt
  and must at minimum carry a finding.
- **Dead interfaces and dead code left standing after a replacement is proven.**
  When the new thing works, gut the old thing decisively — parsers, emitters,
  tables, the lot. Legacy consumers get one translation boundary (a proxy), not
  a preserved old surface. Half-retired code is worse than either state: it
  documents a design nobody follows.
- **Fixes that don't update the design docs.** If a change alters what
  `docs/design/design.md` or a subsystem `DESIGN.md` says, the change isn't done
  until the doc says the new truth.
- **The refactor question, always:** if this problem had been known at design
  time, what would the design be? If the diff and that answer differ, say so in
  the review — even when the diff is accepted for schedule reasons, the gap gets
  named and filed as an issue, not silently absorbed.

## 4. Explicitness at the load-bearing points

Where the code's behavior is safety- or timing-critical, the code must *read*
as what it does.

- **Critical waits are explicit lines in the loop** — `runAndWait(gap, body)` /
  `markTime()`/`sleepUntil()` — never a sleep hidden inside a function whose
  name says it does something else (`pump()`, `emit()`). Anchor the wait to the
  event that starts the physical clock, not to the work that fills the gap.
- **Halt paths use the panic verb and never swallow.** Every "stop the robot
  now" call site (geofence, Ctrl-C, watchdog) uses `estop()`, not the planned
  `stop()`; and a halt that raises must propagate — a silent halt failure is
  indistinguishable from a halt that worked (that exact swallow cost a day: a
  fence detected correctly and stopped nothing).
- **Flags, modes, and frame fields set in one place**, together, at the point
  they're committed — not sprinkled across the functions that happen to learn
  them first.
- **No magic timing constants without provenance.** A dwell, a settle, a
  timeout gets a `// [ms]` tag *and* a reason (what physical event it waits
  for, where it was measured).

## 5. Correctness: where things actually go wrong

Style review without bug review is half a review. For every diff, walk the
failure paths deliberately:

- **The unhappy path of every I/O call.** What happens on NAK, timeout, short
  read, garbage frame? The classic from this codebase: a NAK'd duty write that
  still latched `lastWrittenPct_`, so write-on-change suppressed every retry
  and a failed STOP was lost forever. Rule: **never record success before it
  happened.** State caches update on confirmed effect, not on attempt.
- **Errors must be loud or handled — never both absorbed and unrecorded.**
  Every `catch`/error-return that neither propagates nor increments a counter
  visible in telemetry is a finding. (The silent late-solve drop: segments
  ACKed and never executed, with the only trace a counter that had no getter.)
- **Stale-state hazards.** Anything cached across cycles — anchors, offsets,
  last-seen poses, "previous" values — ask: what re-anchors it, and what
  happens if the world moved while it didn't? (The frozen-anchor tracker that
  fought teleop to a stall at ~12°.)
- **Boundaries and wrap.** Angle wrap (the ~272° attractor), encoder width and
  software-offset projection, queue-full (`ERR_FULL` at depth 5), buffer
  truncation (SIMSET's silent >10-arg drop), the 0x0A-in-binary framing case.
  Every modular quantity gets a wrap-correct comparison; every bounded resource
  gets an explicit full/empty behavior the caller can see.
- **Time.** Who owns "now"? Mixed clock sources, `uint32_t` ms rollover,
  deadlines computed from a different epoch than they're compared against.
- **Concurrency & reentrancy in the loop model.** Nothing blocks the fiber
  (busy-spins in bus clearance blocked the scheduler for ~4 ms/cycle); IRQ-
  masked sections that can drop serial bytes; work functions must be bounded.
- **Checks that don't exist.** For each precondition the code assumes
  (configured device, valid pose, lights on, tag visible, port role), is it
  checked, and does the failure mode name the real cause? Fail-closed on
  unconfigured (sprint 114) is the model.
- **The measurement, before the mechanism.** When reviewing test/bench code:
  a result that varies run-to-run indicts the measurement design first (window,
  settling, lossy means) — never accept an unfalsifiable physical story
  (battery sag, thermal, "flaky link") as an explanation in code comments or
  reports.

## 6. Style and vocabulary (deltas only — the rules docs own the detail)

Reviewers enforce, per `.claude/rules/`:

- **No units in any identifier**; `// [unit]` tags, greppable, first token of
  the trailing comment.
- **UpperCamelCase types/namespaces, lowerCamelCase functions/variables** —
  functions never start uppercase. Members `trailing_`, math subscripts keep
  underscores (`v_x`).
- **Name the quantity precisely** — `speed` vs `velocity`, twist components
  (`v_x`, `v_y`, `omega`), never `position_1_mm`.
- **Edge types named by endpoints** (`<Producer>To<Consumer>Command`), never by
  mechanism or moment (`…Tick`, `…Output`). Long is fine; ambiguous is not.
- **`command` = wire-inbound, `message` = internal (`msg::*`).** No third term.
- **Comments state constraints the code can't show** — not narration of the
  next line, not review-time justification. Wire/serialized strings are never
  renamed as style cleanup.
- Touched legacy code is brought into conformance, not matched.

## 7. Review structure and output

Every review produces a dated report in `docs/code_review/` with this shape:

1. **Verdict up front** — one paragraph: merge / merge-with-findings / rework.
2. **Findings, ranked**, each with:
   - **Severity:** `CRITICAL` (wrong behavior, data loss, safety path) /
     `MAJOR` (design defect that will predictably cause failures or force
     rework) / `MINOR` (craftsmanship debt) / `NOTE` (observation, no action
     implied).
   - **Category:** `placement` / `interface-bleed` / `accretion` /
     `correctness` / `explicitness` / `style`.
   - **Evidence:** `file:line`, with the failing scenario spelled out for
     correctness findings (inputs/state → wrong outcome). No finding without
     a concrete mechanism — "this looks fragile" is not a finding.
   - **The design answer:** for placement/accretion findings, name where the
     code should live or what the interface should be — not just that it's
     wrong.
3. **What's good** — briefly. Calibration matters; a review that can't tell
   textbook from adequate can't defend either.
4. **Debatable taste points are excluded or clearly marked `NOTE`.** The
   report's authority comes from only asserting what it can defend.

**Findings are discussion items, not work orders.** The report is the input to
a conversation with the stakeholder; nobody starts refactoring off the back of
it without an explicit go-ahead. Condemnation opens a design discussion — the
stakeholder usually has a specific architecture in mind, and the review's job
is to inform it, not preempt it.

**Scope discipline:** a review of a diff reviews the diff plus the interfaces
it touches; a review of a subsystem reads the whole subsystem. In both cases,
if the root cause of a finding is outside scope, say so and point at it —
don't stretch the finding to fit the scope.
