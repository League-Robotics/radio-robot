# Radio-Robot-Elite Post-Mortem: How This Project Evolved, and What to Do Differently

**Date:** 2026-07-02 · **Scope:** sprints 001–066, 1,255 commits (2026-05-20 → 2026-07-02), all
CLASI artifacts (sprints, tickets, issues, reflections), the knowledge base, and code-review
documents. **Purpose:** a classic blameless post-mortem focused on the human–AI collaboration —
where progress was linear, where it looped, and what to change so the next project of this class
gets to its goals without the churn this one paid.

**Method.** Nine parallel readers extracted structured evidence from every sprint directory, the
issue backlog, reflections, and knowledge docs; the git log was mined independently for cadence,
churn, and commit-mix; hypotheses were then tested against both. Raw extracts are in
[`evidence/`](evidence/) and [`quantitative-evidence.md`](quantitative-evidence.md) — every claim
below is traceable to a quoted artifact.

---

## 1. The project in numbers

| Metric | Value |
|---|---|
| Wall-clock span | ~6 weeks (with a May 23–31 pause) |
| Sprints | 66 (64 done, 2 active) — **sprint duration is hours, not days** |
| Tickets | 372 (~5.6/sprint) |
| Commits | 1,255 — of which **419 (33%) are `chore: bump version`** |
| Code | firmware 2.9k → 24.9k LOC; host 0 → 24.5k; tests 0 → **73.5k (60% of the codebase)** |
| Peak cadence | 8 sprints in one day (038–045); the 7-sprint message-architecture cutover (055–061, 32 tickets) ran in **under 23 hours** |
| Churn hotspot | `source/types/Protocol.h` touched in **300 commits (24% of all commits)** |
| Rework signal | 24/66 sprint names contain fix/consolidate/eliminate/harden/replace — and that undercounts; by content, **roughly 60% of sprints are predominantly rework, consolidation, or recovery** |

Two framing facts temper everything below. First, the absolute output is far beyond what one
person writes in six weeks: a full C++/CODAL firmware port, a host control library, a physics
sim, an EKF, a message-based architecture, a PySide6 operator GUI, and a 73k-LOC test suite.
Second, the project was co-evolving with AprilCam and the FRC Elite Architecture — the largest
"regression" (the 055–061 message cutover) implemented a design that did not exist at project
start, and it is fair to grant it grace as planned evolution, not failure.

## 2. Timeline: six eras

1. **001–009 (May 20 – Jun 2): Port and un-port.** A fast greenfield port of the TypeScript
   firmware (001–004), followed immediately by systematic un-doing of its structure: 007 is an
   itemized stakeholder correction ("MicroBit is wrong-placed… CommandProcessor does far too
   much… the main loop is hidden"), 008 renames the HAL, 009 hard-deletes the wire protocol that
   002–004 had matched "exactly." Sprint 005 (on-device navigation) was silently abandoned —
   archived to `done/` with zero tickets and `status: open`, its design goal reversed with no
   written rationale.
2. **010–019 (Jun 2–9): The churn burst.** ~79 tickets in eight days; the motion/estimation stack
   built twice and the runtime model three times (fibers added mid-sprint in 013, abandoned in
   014, 014's own split-phase I2C reverted in-sprint). The encoder/I2C wedge is born here and
   never fully dies.
3. **020–029 (Jun 9–12): Sim-first build → field crisis → recovery roadmap.** Sprints 020–023
   built HAL/sim/EKF almost entirely offline, planting the fatal seeds (sim never wired to the
   command queue; noise model off-by-default with the wrong slip sign; heading fusion deferred
   twice). On **2026-06-11** the bill arrived — the "wild spin and cursing" field failure — and
   024–029 is the planned repayment. The pivotal fact: **sprint 024 fixed all six root causes,
   passed 1,434 tests, and still spun full-speed into the boards the same day.**
4. **030–037 (Jun 12–13): External audit + first real hardware contact.** An independent
   model review (030, "Fable round-2") found 16 correctness bugs the ticket loop missed — headlined
   by N2: *"sim wires+tests the queue path; firmware runs the immediate path."* The bench arc
   (031–033) found the validation machinery broken at every layer simultaneously, and 036's bench
   pass discovered `get_id()` and `refresh()` had **never worked on real hardware**.
5. **038–061 (Jun 19–30): The two great migrations.** Phase 0→F (038–045, one day) and the
   message-architecture cutover (055–061, 23 hours), separated by a consolidation batch (046–054)
   whose June-28 blitz ran 20 tickets in one afternoon on sim-only gates. The migrations were the
   process at its best *structurally* (canaries, parity gates, additive-then-delete) and at its
   most exposed *empirically* (every hardware gate deferred; two behavioral fragments silently
   dropped — the outlier-filter recovery and query-safety of `DBG IRQGUARD`).
6. **062–066 (Jun 30 – now): The cockpit and the archaeology.** The TestGUI made a human operate
   the system live for sustained periods for the first time — and defects poured out (063 grew
   from 3 planned tickets to 11 mid-sprint). 064–066 are regression archaeology with explicit
   provenance: "lost in the sprint-060 cutover," "arrived with the ArgSchema migration,"
   "reopens the exact failure the original D9 gate existed to prevent."

## 3. The regression catalog

The specific non-linear loops, with their arcs:

| # | Arc | Path | Cost pattern |
|---|---|---|---|
| R1 | **Encoder/I2C wedge** | 014 (deferred) → 015 (diagnose) → Jun-7 "eliminated" → 033 (detector) → 051 (query bug disarms guard) → 060 (recovery path dropped) → 064 (two new triggers; detector had **0% recall on ~18 real episodes**) | ~4 weeks recurring; the "RESOLVED" knowledge doc was contradicted by in-repo evidence within 10 days and actively misdirected the second investigation |
| R2 | **Chip velocity (0x47)** | built 008 with an invented **11×-off constant** → fixed 010 → fixed 012 → abandoned 013 | feature fixed twice then discarded |
| R3 | **Runtime model** | fibers added mid-sprint 013 → abandoned 014 → 014's split-phase I2C reverted inside 014 | three models in three days |
| R4 | **Motion stack** | 004 → 011 (`_vRamped`) → 017/018 (MotionCommand) → 020 (overhaul) → 026/027 (one path) → 052/053 (stop conditions) → 059–061 (Planner) | ~7 rebuilds; some planned, some reactive |
| R5 | **Command dispatch/parse** | 002–003 accretion → 019 table → 026 one-path → 051 ArgSchema — which introduced query-mutates-state (`DBG IRQGUARD` disarms the guard; bare `RF` retunes the radio to channel 0 **and persists it to flash**) | each rewrite fixed structure and shipped a new behavioral regression |
| R6 | **Navigation ownership** | 005 abandoned → three parallel go-to stacks grow → 029 tickets "closed unimplemented" → 035 finally consolidates | "Firmware fixes from sprints 024–027 have no effect when an agent uses the host-side navigator" — months of fixes bypassed |
| R7 | **Mecanum** | 046 builds it (8 tickets, HITL checkboxes all unchecked — "blocked: no mecanum robot on the bench") → `#ifdef` metastasizes to 81+ sites → 048 deletes the integration → later rebuilt (togov is live today) | full build→delete→rebuild cycle |
| R8 | **Config-consumption class** | `rotationalSlip` calibrated-but-dead (024 D2) → a8 drift lint (025) → `SET` not propagated to Planner's boot-time copy (open issue, root cause of the 90° over-rotation, 2026-07-02) | the same defect *class* three times; the lint guarded registration, not consumption |
| R9 | **Keepalive vs watchdog** | 002 S-watchdog → operator daemon defeats it (024 D4: "demoted the watchdog to dead-process detector") → docstring-recommended keepalive stomps active commands (027 D6) → CR-04/05: "the same 'watchdog silenced by keepalives' mechanism from the June wild-spin postmortem, **now structural**" (065) | same mechanism, June and July |
| R10 | **Sim honesty itself** | 021 noise model (wrong sign, off by default) → 040 PhysicsWorld → 058 dual-source fusion test (stakeholder-forced) → 066 sim-OTOS ground truth (planned) | four generations of making the sim stop lying; 066's thesis: "that agreement **was** the bug" |

## 4. Hypotheses, tested

**H1 — "The AI wrote bad code; that's why we looped." Verdict: real but secondary.**
There are unambiguous AI-quality defects: the invented 11×-off `lapsToMmScale`, the
`setTxBufferSize(1024)` uint8 wrap-to-zero, the union-aliasing bug that meant bench mode never
worked as shipped, a stale `static` debug block that manufactured a phantom bug, two conflicting
design drafts the stakeholder had to reconcile (017). But every era's dominant losses trace to
defects that *passed all gates in force at the time*. The code was mostly wrong in ways the
validation system was structurally unable to see. Fixing "AI writes better code" would have
recovered a fraction of the loss; fixing what "done" meant would have recovered most of it.

**H2 — "The validation surface systematically diverged from reality, and that divergence is the
single biggest cost driver." Verdict: strongly supported; this is the central finding.**
The evidence is overwhelming and spans the whole project: the sim tested a queue path the
firmware never ran (N2); "**Every sim test validates a system that does not exist on hardware**"
(026); the sim's libc had `%f` when the target printed nothing; protocol tests compared "static
string literals, not live firmware calls" (054); the sim OTOS "can never disagree with the
encoders except via injected noise" (066); `SET` replied `OK` while the consumer kept a boot-time
copy; a *query* disarmed the guard it queried, contaminating the very A/B experiments meant to
validate the fix. Meanwhile the one honest channel — hardware — was consulted rarely and late:
the field log holds five entries across three dates while ~30 sprints closed; "hardware
verification deferred to stakeholder" checkboxes sit unchecked inside *done* tickets in at least
ten sprints (004, 007, 009, 031, 033, 036, 037, 046, 059–061). Every major crisis in this project
— June 11, bench 032, post-roadmap 054, the CR-01..15 review — is the same event: **reality
issuing a correction to a backlog of sim-validated work.**

**H3 — "Sprint cadence outran the feedback loop." Verdict: supported; it's H2's multiplier.**
The AI can close eight sprints a day; the truth channel (stakeholder + bench + playfield) ran a
few times a week. So unvalidated change stacked: 048–053 (six sprints, 20+ tickets) merged on
sim-only gates, and the first bench contact afterward immediately found a wire regression (054).
055–061 merged a full replatform — including deleting the legacy path — with the reserved
physical parity run "deferred to stakeholder" out of every sprint's done-criteria; the cost
surfaced 24–48 hours later as field defects. The problem is not speed itself — it's that **the
process had no work-in-progress limit denominated in unvalidated layers.**

**H4 — "The architecture churn was waste." Verdict: rejected in part — churn came in two
distinct kinds with opposite economics.**
*Reactive* churn (eras 1–3: god-component unwinding, dual dispatch paths, three navigation
stacks, fiber whiplash) was expensive and largely avoidable — it came from building layer N+1 on
an unvalidated layer N, and from duplication-as-distrust ("host navigators grew because the
firmware path wasn't believed"). *Planned* churn (038–045, 049–050, 055–061) was strikingly
cheap and clean: the Phase 0→F migration ran seven phases in a day with byte-exact canaries and
zero reopened tickets; TinyEKF's parity gate correctly discovered mid-sprint that the "library
replacement" should keep all the hard-won custom logic. The FRC-derived designs deserve the grace
the stakeholder suggested — they could not have existed at project start, and both migrations
bought real capability (settable plant truth, subsystem isolation, single motor authority). The
caveat: even the clean migrations leaked *behavioral* fragments (outlier recovery, query-safety,
`ERR range`) precisely at the moments their own oracles were rebaselined.

**H5 — "Process overhead ate the gains." Verdict: partially supported.**
33% of all commits are version bumps; a one-ticket mechanical rename ran full sprint ceremony
(055, whose own issue said "should go through /oop"); Definition-of-Ready boxes sit unchecked in
a majority of closed sprints, meaning ceremony was *performed* but not *enforced* — the worst of
both. Against that: the artifact trail is what made 064-era regression archaeology (and this
post-mortem) possible at all. The overhead problem is real but is mostly a tuning problem —
automate the bumps, drop unenforced checkboxes, keep the provenance.

**H7 — "The compressed timeline eliminated human incubation ('shower thoughts') and code-reading,
which is where structural insight comes from." Verdict: supported — examined in detail in the
[incubation addendum](addendum-incubation.md).**
Raised by the stakeholder after the initial report. The record contains direct natural
experiments: an AI naming decision reversed by the first commit of the next morning (016's
AppContext → Robot, one sleep boundary later); the largest replatform of the project executing
16 tickets between 23:46 and 03:06 while the human slept; a burst–gap commit rhythm where every
calendar gap emitted the project's best planning artifacts and every burst stacked 10–30 tickets
of never-read surface on top of them; and three reactive slow-reads that harvested ~a third of
all confirmed defects at a desk. The addendum refines H6: the scarce resource is not approval
bandwidth but *absorbed attention*, and it adds recommendations R11–R15 (sleep boundaries before
irreversible merges, scheduled guided reads, cockpit-first legibility, bursts sized to human
absorption, "hard to follow" as a structural defect signal).

**H6 — "Human attention was the scarce resource, and it wasn't rationed to the highest-value
gates." Verdict: supported — and it's the most actionable finding.**
Every inflection toward linearity in this history is a stakeholder intervention at a choke
point: the June-11 review, the ALL-CAPS bench-transport directive, the 047 Q1–Q5 design review,
rejecting the vacuous fusion test (058 — corrected for ~25 minutes of wall clock), the 048
"supersede it, don't partial-fix" call, the golden-TLM "not an autonomous rubber-stamp" rule, the
2026-07-02 five-arm stand experiment that finally isolated the wedge triggers. Conversely, every
major loss ran through a gate the human was *supposed* to hold but that the process allowed to be
deferred: unchecked HITL checkboxes in done tickets, "auto-approve session" DoR entries, bench
runs owed after merge. The pattern is precise: **when the human held the gate, correction was
cheap; when the gate slipped past the human, correction became archaeology.**

## 5. Was there a productivity gain?

Honest answer: **it depends on which half of the system you look at, and the stakeholder's
suspicion is justified for the half that matters most.**

- **Where the feedback loop closed in software, gains were enormous.** A PySide6 operator cockpit
  in ~a day; a seven-phase architecture migration in a day with zero breakage; a 73k-LOC test
  suite; exhaustive audits that found sibling bugs no one asked about (the `RF` channel-0
  landmine); a wire-protocol hard-break executed in one sprint. No solo human ships this in six
  weeks.
- **Where the loop required physical truth, the AI's advantage inverted.** Iteration speed
  without a matching verification channel doesn't just fail to help — it *manufactures* backlog:
  layers of sim-validated work whose defects compound and are repaid later at archaeology prices
  (the wedge: ~4 weeks; `twist=0` on hardware: shipped in 023, found in 032; `get_id()` never
  worked: found in 036). A human writing the firmware alone would have been slower per line but
  would have been *forced* to keep the hardware in the loop continuously — an accidental cadence
  match that this process lost. For the physics-coupled core, net productivity was likely near
  zero, possibly negative once the stakeholder's own debugging hours are priced in.

So the reconciliation with other projects' "enormous gains" is simple: those applications are
usually ones where **the test double is the deployment target** (web apps, data pipelines, CLIs).
Here, the deployment target was a robot with silicon errata, a lossy radio, and carpet — and the
process spent five weeks discovering that its proxy for reality was fiction, one incident at a
time.

## 6. What worked — keep these

1. **Canary-gated, phased structural migration** (038–045): byte-exact golden frames, config
   field-pins, ratcheting grep gates, verbatim moves, scaffolding built for planned demolition.
   The cleanest week of the project.
2. **Parity-gate-before-replace** (050): the gate did its job — it *changed the plan* mid-sprint
   when the library proved shallower than assumed.
3. **Stakeholder-locked decisions written into artifacts** ("Decisions (locked): …", the (v,ω)
   ruling, "do not 'fix' by disabling sensors") — cheap to write, repeatedly prevented
   relitigation and AI escape-hatches.
4. **External adversarial review as a defect pump**: all three (June-11, Fable round-2,
   2026-07-01 full-codebase) found 12–16 real cross-cutting defects each — a bug class the
   per-ticket loop structurally misses.
5. **Evidence-first issue writing** (the 062+ era): issues with confirmed file:line mechanisms
   before ticketing, corrected misdiagnoses recorded in-place, controlled experiments with
   numbered arms.
6. **Design review before code** (047's Q1–Q5): five questions, five recorded answers, clean
   execution. The cheapest sprint-quality insurance in the whole history.

## 7. Recommendations for the next project

Ordered by expected leverage.

**1. Two-state "done": nothing is *done* until verified against reality.**
Give every ticket/sprint two completion states: `sim-done` and `verified`. A sprint may close
`sim-done`, but the process must track the verification debt explicitly, and *merges that delete
fallback paths* (060's legacy deletion) require `verified`, full stop. Make it mechanical: the
process tooling should refuse to move a ticket to done with unchecked acceptance boxes — the
single most repeated micro-failure in this history (10+ sprints).

**2. Cap unvalidated depth (WIP limit on reality debt).**
Pick N (2–3). No more than N sprints of hardware-touching change may stack before a mandatory
verification session. The June-28 blitz (6 sprints, sim-only) and the 23-hour cutover would both
have tripped this and been cheap to verify incrementally; instead, both were repaid as
archaeology. Schedule the human's bench time *as sprint infrastructure*, not as a courtesy
afterward — the human is the rate-limiting instrument and should be scheduled like one.

**3. Build the honest test double first, and treat divergence as a P1 defect in the double.**
Sprints 040 + 058 + 066's sim-fidelity work, done in week 1, would have prevented the majority of
the regression catalog. Concretely: single ground-truth plant; sensors as *observation models*
that can disagree; error models on every channel, on by default in the CI profile; a periodic
regression-fit against hardware traces (066's "tunable to behave identically"). And adopt the
rule the project learned three times: when sim and hardware disagree, **the sim is broken until
proven otherwise** — file it against the sim, not just the robot.

**4. Test invariants, not just tickets.**
Every externally-reviewed defect batch was a *cross-cutting invariant* violation: queries must be
pure (would have prevented R5's two worst bugs with one property test), every begin*() cancels
its predecessor, every config key has a live consumer (kills R8 as a class), every EVT has a
firing test, replies echo on the arrival channel, no silent success (`OK` must mean the consumer
saw it). Write these as property/sweep tests in week 1 and run them in CI. This is exactly the
work AI is best at generating — it just has to be asked.

**5. Hold oracle rebaselines to a higher standard than normal merges.**
Both silent behavioral losses (outlier recovery in 060, `ERR range` in 051–053) slipped through
*while the golden oracles were being legitimately regenerated*. Rule: a rebaseline PR must carry
a behavioral diff (what changed, why each delta is intended) reviewed by the human — the 060
"not an autonomous rubber-stamp" rule, but enforced every time, including for test-tolerance
loosening (059's quiet 2°→5°).

**6. Claims require receipts; knowledge has a shelf life.**
The wedge "RESOLVED" doc misdirected a later investigation while contradicting evidence already
in the repo. Require: every fix's closing claim cites the observation that proves it *on the real
system* (log line, TLM trace, video); knowledge docs carry `verified-on: <date, hardware>`
frontmatter and a status that decays to `unverified` when the subsystem changes; and when new
evidence contradicts a doc, updating the doc is part of the fix's definition of done. Also write
reflections far more often — one reflection in six weeks against dozens of correction events
means the cheapest learning loop was almost unused.

**7. Ration the human deliberately; automate the rest.**
List the decisions only the human can make — safety semantics, oracle acceptance, deletions of
working code, architecture bets, hardware sign-off — and make those gates *blocking* (no
auto-approve for them, ever). Everything else (version bumps: 33% of commit history; ticket
bookkeeping) should be invisible automation. The evidence says human attention at choke points
was worth ~100× human attention spread thin across ceremony checkboxes.

**8. Schedule adversarial review; don't wait for a crisis.**
A fresh-context review every ~10 sprints (or before any merge that deletes a fallback path).
All three reviews in this project were reactive — each triggered by an incident — yet each found
a dozen latent defects that were *already present* at the previous review-worthy moment.

**9. Prefer supersede over partial-fix for structural mistakes (the 048 rule).**
"Don't do the partial refactor then immediately redo it" was right, and the same logic earlier
would have saved arcs R4–R6 several intermediate forms. When a structure is wrong, budget the
full correction or explicitly park it — the intermediate half-states are where fragments get
dropped.

**10. Keep the ceremony that produces provenance; cut the ceremony that produces checkboxes.**
The sprint/issue/knowledge paper trail is the reason this project could root-cause its own
regressions ("lost in the 060 cutover," "arrived with 051") and the reason this post-mortem could
be written. Keep artifact-per-decision. Drop any gate the process won't actually enforce —
an unchecked box in a done ticket is worse than no box, because it launders unverified work as
reviewed.

## 8. The one-paragraph version

This project produced a genuinely large system fast, and its two planned migrations show the
human+AI process at its best. But roughly half its sprints repaid debt created by the other half,
and the debt had one dominant source: **"done" was defined by a validation surface (sim, mocks,
string-literal tests, unchecked checkboxes) that reality kept vetoing** — and the veto channel
(the stakeholder at the bench) was the scarcest, least-scheduled resource in the loop. The AI
amplified both sides: it built the proxy world and passed its tests at superhuman speed, and it
also performed superb archaeology when the proxy failed — but it could not, by itself, close the
loop against physics. The fix for the next project is not a smarter AI or more ceremony; it is
an economic one: **cap the amount of unverified work in flight, spend the human only and always
at the reality gates, and make the test double's honesty the first deliverable rather than the
sixty-sixth sprint's.**

---

*Evidence: [quantitative-evidence.md](quantitative-evidence.md) · per-era extracts in
[evidence/](evidence/) (001-009, 010-019, 020-029, 030-037, 038-045, 046-054, 055-061, 062-066,
backlog-knowledge-docs) · [addendum-incubation.md](addendum-incubation.md) (H7: the incubation
hypothesis).*
