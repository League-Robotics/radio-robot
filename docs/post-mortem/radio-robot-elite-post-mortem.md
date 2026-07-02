# Radio-Robot-Elite Post-Mortem: How This Project Evolved, and What to Do Differently

**Date:** 2026-07-02 · **Scope:** sprints 001–066, 1,255 commits (2026-05-20 → 2026-07-02), all
CLASI artifacts (sprints, tickets, issues, reflections), the knowledge base, and code-review
documents. **Purpose:** a classic blameless post-mortem focused on the human–AI collaboration —
where progress was linear, where it looped, and what to change so the next project of this class
gets to its goals without the churn this one paid.

**Method.** Ten parallel readers extracted structured evidence from every sprint directory, the
issue backlog, reflections, and knowledge docs — nine over this repo, one over the sibling
AprilCam repo (`../AprilTags`, ~29 sprints across two CLASI generations); the git logs of both
repos were mined independently for cadence, churn, and commit-mix; hypotheses were then tested
against both. Raw extracts are collected in **Appendix C**, **Appendix B**, and **Appendix D** —
every claim below is traceable to a quoted artifact.

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
Second, this repo was **one workstream in a portfolio**. AprilCam — the camera/perception system
that serves as the robot's ground-truth pose source — *predates* this repo by eight months (first
commits September 2025, with TypeScript and C ancestors before that) and was under active
development in the same window: ~320 of its 650 commits land between mid-May and June 27,
interleaved with this repo's quiet days (see the revised burst–gap table in Appendix A). The FRC
Elite Architecture design work ran alongside as well, and drove the largest "regression" (the
055–061 message cutover) — a design that did not exist at project start. Both deserve grace as
planned evolution, not failure; both also charged real costs to this repo's ledger, examined as
H8 in §4 and in Appendix D.

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

One amendment, forced by the AprilCam record (H8, Appendix D): this cannot be read as an
*absence* of truth instruments. The bench had `velocity_chart` from week two, and the playfield
camera was wired in as the robot's precision position reference from the very start of the main
build — sprint 012 added a "camera-truth verify script" on **Jun 2** (nine days before the wild
spin), with `goto_tag.py`, camera-validated calibration, `nav/camera_goto.py`, and the Playfield
module following within days; the AI agent itself had live `aprilcam` MCP tools in its own
sessions. The instruments existed and were honest; what they lacked was *structural standing*.
Camera truth was session-scoped (it ran when a human set up the playfield — three dates in the
field log), advisory (no sprint close, merge, or CI run ever required its verdict), and slower
than generation (the loop producing eight sprints a day ran nights and sim-days where the camera
doesn't reach). **An honest instrument loses to generation speed when it isn't in the merge gate
and doesn't run while the human sleeps.** The gap was never missing truth — it was truth with no
authority.

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
which is where structural insight comes from." Verdict: supported — examined in detail in
Appendix A.**
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

**H8 — "Some of this project's churn was really the cost of building AprilCam and the FRC
architecture alongside it" (the stakeholder's grace hypothesis). Verdict: partially supported —
with a twist that strengthens the central finding rather than excusing it.**
Tested against the AprilCam record (full comparison in Appendix D), the grace is real on three
counts. *Attention division:* confirmed — the burst–gap overlay (Appendix A) shows this repo's
"quiet" periods were mostly AprilCam bursts; one human attention pool served two AI-speed
workstreams plus the FRC design effort. *The truth instrument was under construction:* confirmed
— the robot's designated reality channel was itself wrong in ways its own display could not
show: a swallowed TypeError meant world coordinates were silently `None` on calibrated fields
(AprilCam sprint 004); elevated robot tags carried >7 mm of uncorrected parallax until sprint
008; the repo held **three disagreeing playfield geometries** at once, one with "nonsense"
corner coordinates (sprint 012); and the June 13–16 ENU/y-flip convention storm — discovered
because "the robot drives to the wrong edge" — plus the +43 mm off-center-tag incident that
masqueraded as a robot OTOS regression, mean a real fraction of robot-side debugging was
actually chasing instrument defects. *Interface churn:* partially — the five tour-script
variants (037), the Playfield module rework (036), and the `read_cam_pose`/`t.yaw` gotchas are
the cost of building against a moving perception API. **The twist:** AprilCam's own history
*replicates* this report's central finding instead of excusing it. It looped the same way —
roughly half its second-generation sprints are rework, an entire 14-sprint first generation was
dismantled, and it ran the identical authority-consolidation ladder (sole camera owner → path
authority → geometry SSOT → sole vision authority). But its loops closed in **days, not weeks**,
because its default validation surface — a human watching annotated live video — is the
production artifact itself… *except* for exactly the output the robot consumed: numeric frame
semantics, which the viewer renders invisibly and which were flushed out only by the consumer.
Both projects paid in the same place: the gap between what their feedback surface made legible
and what their consumers actually consumed.

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
afterward — the human is the rate-limiting instrument and should be scheduled like one. And
automate the part of the gate that doesn't need the human at all: the playfield camera plus a
canned drive sequence (`playfield_camera_run.py` — which, when finally run, verified turn
closure to 1.8° and square return to 2.1 cm) is an **automatable hardware-acceptance station**
that could have gated merges nightly with nobody present. It existed for weeks and was never
wired into any gate.

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

*Evidence for the report above: Appendix B (quantitative evidence) and Appendix C (per-era
evidence extracts) follow. Appendix A (the incubation hypothesis, H7) and Appendix D (the
AprilCam comparison, H8) are follow-up analyses raised after this report was first delivered.*

---

# Appendix A: The Incubation Hypothesis (H7)

Follow-up to the main report above, examining a stakeholder hypothesis raised after the initial
report was delivered.

## The hypothesis

> If a human writes a two-week project, every ~10% of it is followed by a shower, a dinner, a
> night's sleep — the diffuse-mode moments where you realize "this isn't going to work" or "the
> reason I can't understand this code is that its structure is wrong." AI compresses the project
> to a day: no time for shower thoughts. The human also never *reads* the code — once an agent is
> kicked off, the human disengages entirely — so the reading-driven realizations never happen
> either. Perhaps the limiting factor is simply human review. This would also explain why web
> applications show huge AI productivity gains: they are legible, the human is the user, and
> testing happens incidentally through use. Robot code is below the surface; the human never gets
> around to reviewing it.

Decomposed into three testable mechanisms:

- **H7a — Calendar compression:** insight needs elapsed time between engagement episodes;
  AI-speed removes the elapsed time.
- **H7b — Delegation kills reading:** structural insight comes from the *effort of understanding
  code*; delegation removes the reading, so structure-level defects go unseen.
- **H7c — Legibility gates engagement:** humans engage (and therefore test, and therefore have
  insights) in proportion to how visible and usable the system's surface is. Web apps are
  self-legible; robot internals are not.

## Natural experiments in the record

The corpus contains several events that function as direct tests.

**1. The overnight reversal (H7a, direct positive).** Sprint 016 deleted the `Robot` facade in
favor of `AppContext` at **Jun 8, 15:55** (`016-006: delete Robot.h/Robot.cpp`). At **Jun 9,
06:47** — the first commit of the next morning — the rename was reversed (`Refactor AppContext to
Robot`). The structure survived; the naming decision did not survive one night of human sleep.
This is a literal shower-thought artifact in the git log: the mechanism works, and it worked
*because* there happened to be a sleep boundary between the AI's decision and the merge becoming
load-bearing.

**2. The May pause (H7a, positive — but revised by the AprilCam record).** After sprints 001–005
(May 20–23), this repo paused eight days. The first act on return was not sprint 006's tooling —
it was a formal re-initiation (Jun 1: "Add feature specification and use cases") followed within
days by sprint 007, an itemized stakeholder correction of everything structural the AI had built:
"MicroBit is wrong-placed… CommandProcessor does far too much… Config is duplicated and can
diverge… the main loop is hidden." An earlier draft of this analysis read the pause as pure
incubation. The AprilCam git log corrects that: **the "pause" contained 78 AprilCam commits
(May 24–29)** — the human wasn't showering, they were building the camera. What survives the
correction is narrower but still real: time away *from this codebase* preceded the structural
diagnosis. Displacement onto another project apparently serves the same diffuse-processing
function as rest — but it also means the incubation channel was being *shared* across the
portfolio, not merely under-scheduled within this repo.

**3. The burst–gap rhythm (H7a, systemic — with the AprilCam overlay).** Commits per day show
the project's true shape — AI bursts separated by near-zero days. But overlaying the AprilCam
repo's commits on the same calendar reveals what the "gaps" actually were:

| period | this repo (commits/day) | AprilCam (total) | what it was |
|---|---|---|---|
| May 23–31 | 0 | **78** | the "May pause" — an AprilCam burst |
| Jun 2–5 | 37–100 | ~11 | churn burst (sprints 010–015) |
| Jun 6–7 | 2–6 | **31** | "gap" — bench days + AprilCam (29 commits Jun 7, the day the wedge was "eliminated") |
| Jun 8–13 | 25–96 | ~31 | burst (016–037, incl. the field crisis) — both repos running hot |
| **Jun 14–18** | **1–7** | **27** | "gap" — FRC Elite design work **+ AprilCam** |
| Jun 19 | 96 | 27 | Phase 0→F migration, all 8 sprints in one day |
| Jun 20 | 23 | 0 | field day (field-log: FAIL, PASS-with-SKIPs, FAIL) |
| **Jun 24–27** | **2–7** | 12 | the one mostly-genuine gap — field FAILs Jun 27; 048 supersede decision forms; AprilCam's **final commits (Jun 27)** |
| Jun 28 | 143 | 0 | the blitz (047–053, 20+ tickets in an afternoon) |
| Jun 29–30 | 21–126 | 0 | overnight replatform (055–061) |
| Jul 1–2 | 47–87 | 0 | full-codebase review + archaeology sprints |

Two revisions to the original reading. First, most "gaps" were not rest — they were **the other
project**: a single human attention pool time-sliced across two AI-speed workstreams (three,
counting the FRC design work). The incubation channel wasn't merely unscheduled; it was
oversubscribed. Second, the core claim survives and sharpens: each period away from this repo
still emitted its best planning artifacts (Jun 14–18 → the Phase 0→F master issue and its canary
discipline; Jun 24–27 → the "supersede 048" decision and the 048–053 roadmap; Jul 1 →
CR-01..15), and each burst then **added 10–30 tickets of new surface that no gap ever processed**
before the next burst built on top of it. Note also the sequence at the portfolio level:
AprilCam reached its own "daemon is the sole vision authority" consolidation (its sprint 015) and
went quiet on Jun 27 — and this repo's biggest bursts (Jun 28–30) began the following day.
Incubation was not absent — it was operating on stale state, always one burst behind, in
whichever repo the human had most recently left.

**4. Sprints executed while the human slept (H7a/H7b, negative case).** Phases 1–3 of the
message architecture — 16 tickets, the core of the replatform — ran **Jun 29 23:46 → Jun 30
03:06**. The human's sleep that night coincided exactly with the largest structural change of the
project; there was no possibility of contemporaneous reading, and the parity fallback, the
relaxed 2°→5° tolerance, and the deferred bench gates were all accepted by the process during
those hours. The two latent defects that batch shipped (the Planner boot-time config copy; the
lost outlier recovery) are precisely structure-level, reading-catchable defects.

**5. Slow reads were the highest-yield defect events in the project (H7b, strong positive).**
Only three deliberate whole-system reads occurred in six weeks, all reactive:

| event | trigger | yield |
|---|---|---|
| 2026-06-11 sim2real review | field crisis ("wild spin") | ~20 findings (D1–D12, A1–A8) → sprints 024–029 |
| 2026-06-12 Fable round-2 | stakeholder-commissioned | 16 correctness findings → sprint 030 |
| 2026-07-01 full-codebase review | stakeholder-requested | 15 findings (CR-01..15) → sprints 065–066 |

Three reads ≈ fifty confirmed defects ≈ **a third of the project's entire defect archaeology,
found at a desk without touching hardware**. Each read also found the *class*, not just the
instance (query-purity violations, cancel-contract non-uniformity, config-consumption gaps).
Nothing else in the process — not the 73k-LOC test suite, not the sim, not per-ticket review —
came close per unit of human time. The cognitive channel wasn't low-yield; it was unscheduled.

**6. The TestGUI effect (H7c, strong positive — refined: instruments vs. cockpits).** An earlier
draft of this section claimed the system had no legible surface until sprint 062. That is wrong,
and the correction makes the finding sharper. The project built *diagnostic instruments* early
and continuously: `tests/bench/velocity_chart.py` (strip charts + phase plot for the wheel-PID
loop) landed in sprint 014 (Jun 5), `sensors_chart.py` in 015, and by mid-June the bench held a
whole family (`world_goto_chart`, `plot_square`, `arc_calibration.ipynb`, `smoke_ritual`,
`echo_rate`). And these instruments *worked*: the velocity strip charts are what exposed the
"velocity notches" root-caused within a day to the S-watchdog uint32 underflow (Jun 5–6), the
motor "throb," and the cross-coupling noise amplification that was disabled the same day it
shipped. Where an instrument existed and was pointed at a subsystem, that subsystem got
continuously corrected — the inner PID loop, watched by `velocity_chart` from week two, is one of
the few layers with *no* entry in the regression catalog.

The distinction that survives is between an **instrument** and a **cockpit**. An instrument is
aimed deliberately: you must already suspect a subsystem to run its chart, so instruments
harvest defects from *hypotheses the human already has*. A cockpit is inhabited: the TestGUI put
four pose estimates on a playfield under a keyboard-driven robot, and the human's ordinary use
generated observations nobody was looking for — which is why 063 grew from 3 tickets to 11
mid-sprint while five weeks of bench charts had never produced that kind of unsolicited defect
stream. The layers that accumulated silent debt (config propagation, dispatch authority,
watchdog semantics, pose-authority plumbing) were exactly the layers no instrument watched and
no surface made ambient. Two corollaries: coverage of the *instrument park*, not just the test
suite, predicts where debt accumulates; and the instruments themselves need production-grade
trust — `velocity_chart` froze mid-diagnosis in 015, the bench-033 harness reset the board on
open and mis-correlated reads, and `echo_rate` had a reader bug that overstated drop rates. An
untrusted instrument is worse than none: three separate incidents in this record are the
measuring tool framing the system under test.

A second correction goes deeper (details in Appendix D): the project also had something much
closer to a *cockpit* than a bench chart for nearly its whole life. AprilCam's live view is an
inhabited overhead surface — annotated video of the playfield in world-cm coordinates, overlays
the robot pushes at 10 Hz, the human literally driving the robot underneath it — and the robot
repo consumed it from sprint 012 onward (Jun 2: "camera-truth verify script"; then `goto_tag`,
`camera_goto`, the Playfield module, `playfield_camera_run.py`). So the honest question is not
"why was there no cockpit until 062" but **"why did an existing cockpit not fire the H7c
channel?"** Three structural differences from the TestGUI, none of them habitability:

1. **Session-scoped vs ambient.** The camera view ran when a human scheduled a playfield
   session — three dates in the field log — while the TestGUI ran on the developer's desk during
   ordinary work, *including in sim mode*. One lived in the validation session; the other lived
   in the development loop, which is where the code was actually being generated.
2. **Advisory vs load-bearing.** Nothing gated on the camera's verdict. Sprints closed sim-green
   whether or not the playfield had been powered on that week.
3. **Single-truth vs comparative display.** The live view showed where the robot *is*. The
   TestGUI overlaid where the camera, the encoders, the OTOS, and the EKF *each believed* it
   was — and disagreement between estimates is the defect signal (the Jul 1–2 issue stream is
   almost entirely disagreements between those four tracks).

So the taxonomy that survives this second correction: a feedback surface pays in proportion to
how continuously it runs, whether anything *depends* on it, and whether it renders
**disagreement** rather than mere state. AprilCam's view was honest, inhabited, and optional;
the TestGUI was the same view made ambient, comparative, and part of the daily loop — and only
then did the defect stream switch on.

## Verdict

**Supported, with one refinement and one boundary.**

The refinement: it is not that incubation never happened — the gaps happened, and they produced
the project's best thinking. But the AprilCam overlay (experiment 3) shows most gaps were not
rest: they were the human's attention time-slicing to a second AI-speed project. The failure
mode is a **rate mismatch, compounded by oversubscription**: two workstreams generated unread
surface at AI speed while one human's absorbed attention alternated between them, so insight
always applied to the system as of the *previous* gap. By the time the Jun 24–27 gap produced the mecanum supersede decision,
the Jun 19 burst's eight sprints had never been read; by the time Jul 1's review processed the
replatform, it was already merged with its fallback paths deleted. The shower thought arrived —
about last week's code.

The boundary: incubation is not a universal solvent. The wedge did not yield to thought; it
yielded to a five-arm controlled stand experiment (Jul 2). Physics-coupled defects need
instrumented contact, not rumination — that is the main report's H2 and it stands independently.
H7 governs a *different* defect class: the structural ones (boot-time config copies, three
navigation stacks, queues never wired to sims, layering inversions) — exactly the class the three
slow reads harvested, and exactly the class the stakeholder says they'd have caught while writing
the code themselves. On the evidence, they're right: every one of those defects was findable by
reading, and each was in fact found by reading — eventually, reactively, after it had cost weeks.

H6 in the main report said the human is the scarce resource. H7 sharpens it: the scarce resource
is not human *approval bandwidth* (checkboxes were plentiful and worthless) but human **absorbed
attention** — reading with effort, operating the system, and the diffuse processing that follows.
Checkboxes can be delegated to auto-approve; absorption cannot.

## What to do about it (extends the main report's recommendations)

**R11 — Put a sleep boundary in front of irreversibility.** Any merge that deletes a fallback
path, rebaselines an oracle, or locks an architecture decision waits overnight — not for more CI,
but because the record shows the human's overnight pass reverses real decisions (016) and its
absence ships regressions (the 23-hour replatform). Let the AI keep working during the wait:
generating review aids, adversarial tests, alternative framings — anything but stacking further
dependent layers.

**R12 — Schedule the reading; make the AI the tour guide.** A recurring guided read — every N
sprints or before any burst >10 tickets — where the AI prepares the data-flow narrative,
invariant map, and "here's what changed and what now depends on it," and the human reads code,
not summaries. Three reactive reads yielded ~50 defects; the process never once scheduled one
proactively. This is the cheapest defect pump the project demonstrated, and it directly
manufactures the engagement H7b says delegation destroys.

**R13 — Build the cockpit first; instruments are necessary but not sufficient.** The TestGUI
should have been sprint ~5, not sprint 62. The project *did* build bench instruments early
(`velocity_chart` in sprint 014, `sensors_chart` in 015 — see experiment 6), and the subsystems
they watched stayed comparatively healthy; the gap was the absence of a surface the human would
*inhabit* during ordinary use, which is what generates unsolicited discovery. So the fuller
rule: for any AI project whose target isn't self-legible, the first deliverable is the cockpit —
dashboard, live viewer, driving console, whatever turns the human into a user — backed by an
instrument park whose coverage is tracked like test coverage (an unwatched subsystem is where
the next silent regression lives), and whose tools are validated like production code (three
incidents here were the instrument framing the system under test). And the surface must be
**wired into the gate**: a cockpit nothing depends on is what AprilCam's live view was for five
weeks — present, honest, inhabited during sessions, and consulted by no merge, no sprint close,
no CI run (see the H2 amendment). Ambient, comparative, and gating — a feedback surface needs
all three, or it is decoration.

**R14 — Size bursts to human absorption, not AI throughput.** The main report capped unvalidated
depth (recommendation 2); extend it to *unread* depth: a burst should not exceed what its human
will actually absorb before the next burst — practically, one day's diff presented with a reading
guide, not seven sprints overnight. If the human can't absorb it this week, the correct response
is a smaller burst, not a bigger summary.

**R15 — Treat "I can't follow this" as a defect signal.** The stakeholder's observation — the
difficulty of understanding code is often information about the code — held every time it was
tested here (the god components, the hand-mirrored sim loop, the triplicated go-to). Make it
operational: during guided reads, anything the human (or a fresh-context AI) cannot explain after
one honest attempt gets an issue filed against the *structure*, not against the documentation.

**A note on substitutes.** Fresh-context adversarial AI reviews (Fable round-2) demonstrably
capture part of the reading channel — a fresh model with no sunk context found 16 bugs the
resident process missed, which is functionally an artificial second-reader. Scheduled fresh-eyes
reviews can and should shoulder volume the human can't. But the record also shows their limit:
no AI review flagged "the stakeholder will hate this naming" or "supersede the partial fix" —
judgments that came only from the person who holds the intent, after time away. The substitute
scales the channel; it does not replace the incumbent.

## Reconciling the web-app asymmetry

The hypothesis explains the stakeholder's cross-project observation economically. In a web app,
the human's engagement is *incidental to use*: they click through the thing because they want to
see it, and validation, reading motivation, and shower thoughts all ride along for free — the
cost of a unit of human truth is near zero. In robotics (and any below-the-surface system:
firmware, pipelines, infra), every unit of human truth must be *scheduled* — a bench session, a
review, a stand experiment — and so under deadline pressure it is deferred, and the AI's
generation rate fills the vacuum with unverified, unread work. The productivity gap between the
two domains is not mainly about code difficulty; it is the price of human engagement per unit of
ground truth. Which yields the compact rule for choosing and running AI projects:

> **AI productivity tracks the legibility of the system's truth to its human. Where truth is a
> click away, AI compounds; where truth must be scheduled, AI outruns it — unless the process
> spends its first sprints making truth cheap.**

(Appendix D tests this rule against AprilCam — the portfolio's own control group — and sharpens
it: legibility must cover what the *consumer* consumes, not what the surface renders.)

---

# Appendix B: Quantitative Evidence

Data extracted from the git history and CLASI artifact tree on 2026-07-02.
All numbers computed from `git log` on branch
`sprint/065-...` (1,255 commits, 2026-05-20 → 2026-07-02).

## Timeline and cadence

- **Project span:** 2026-05-20 → 2026-07-02 — approximately 6 calendar weeks,
  with a pause 2026-05-23 → 2026-05-31.
- **Sprints:** 64 done + 2 active = 66 sprints. **372 tickets** total
  (~5.6 tickets/sprint).
- **Sprint duration is hours, not days.** From commit scopes:
  - Sprints 025–035 (11 sprints): all closed 2026-06-11 → 2026-06-12 (2 days).
  - Sprints 038–045 (the entire Phase 0→F architecture refactor, 8 sprints):
    all committed on **one day**, 2026-06-19.
  - Sprints 047–053 (7 sprints): all on 2026-06-28.
  - Sprints 055–061 (the message-architecture cutover, 7 sprints):
    2026-06-29 → 2026-06-30 (2 days).
- **Two epochs:** sprints 001–005 (May 20–23) built a first CODAL C++ firmware
  (~2,900 LOC); the project was then formally re-initiated on 2026-06-01
  ("Add feature specification and use cases for radio-robot-c") and sprints
  006+ proceeded under the full CLASI process.

## Commit mix

| type | count | share |
|---|---|---|
| chore | 638 | 50.8% |
| feat | 259 | 20.6% |
| untyped/other | 246 | 19.6% |
| fix | 44 | 3.5% |
| refactor | 30 | 2.4% |
| docs | 22 | 1.8% |
| test | 16 | 1.3% |

- **419 of 1,255 commits (33.4%) are `chore: bump version`** — pure process
  overhead, one for every ~2 substantive commits.
- The low raw `fix:` share is misleading: under the ticket process, fix work is
  labeled `feat(NNN-MMM)`. Sprint *names* are the better classifier (below).

## Commits per week (excluding version bumps)

| week | commits | feat | fix | refactor |
|---|---|---|---|---|
| 2026-W20 (May 18) | 25 | 17 | 1 | 0 |
| 2026-W22 (Jun 1) | 177 | 86 | 14 | 4 |
| 2026-W23 (Jun 8) | 231 | 61 | 17 | 8 |
| 2026-W24 (Jun 15) | 109 | 18 | 1 | 17 |
| 2026-W25 (Jun 22) | 112 | 11 | 0 | 0 |
| 2026-W26 (Jun 29) | 182 | 66 | 11 | 1 |

## Code size trajectory (LOC)

| date | firmware (`source/`) | host (`host/`) | tests |
|---|---|---|---|
| 2026-05-21 | 2,933 | 0 | 0 |
| 2026-06-05 | 7,355 | 20,368 | 9,357 |
| 2026-06-12 | 17,159 | 20,668 | 34,102 |
| 2026-06-19 | 19,816 | 17,383 | 51,066 |
| 2026-06-28 | 21,217 | 17,519 | 56,104 |
| 2026-07-02 | 24,890 | 24,536 | 73,506 |

- Final system ≈ **123k LOC**, of which tests are **60%**.
- Host LOC *shrank* June 12→19 (consolidation era) then grew again with the
  TestGUI (062–063).

## Churn hotspots (times touched, excluding process/docs/config artifacts)

| file | commits touching it |
|---|---|
| `source/types/Protocol.h` | **300 (24% of all commits)** |
| `source/robot/Robot.cpp` | 109 |
| `source/robot/Robot.h` | 61 |
| `source/app/CommandProcessor.cpp` | 59 |
| `source/main.cpp` | 43 |
| `source/types/Config.h` | 40 |
| `source/control/MotorController.cpp` | 38 |
| `source/control/Odometry.h` | 33 |
| `source/control/LoopScheduler.cpp` | 33 |

(`host/pyproject.toml` at 283 and `config/dotconfig.yaml` at 235 are
version-bump/process churn.)

## Rework-signal sprint names

24 of 66 sprint names (36%) contain fix / consolidate / eliminate / cutover /
replace / collapse / harden / abandon / reliability / debug / diagnosis /
cleanup. This **undercounts** structural rework: sprints 016, 018–020, 026,
029, 034, 036, 038–045, 048, 055–061 are rework by content with neutral names.

## Notable event sequences

- **Encoder/I2C wedge:** deferred out of sprint 014 (2026-06-05, "wedge
  deferred to follow-up issue") → sprint 015 diagnosis/fix → 2026-06-07
  `fix: eliminate encoder wedge — IRQ-guard I2C transactions (nRF52 TWIM
  errata)` → **still being hardened in sprint 064 on 2026-07-02** (wedge
  triggers, IRQguard query bug, read failure, outlier-filter recovery), with a
  new "boundary-latch flavor" documented 2026-07-01. Recurrence arc ≈ 4 weeks.
- **Cross-coupling reversal inside one day** (2026-06-05): `feat[015]:
  velocity EMA filter + outlier rejection + cross-wheel ratio coupling`
  followed the same day by `fix[015]: disable cross-coupling by default
  (amplified velocity noise into wheel-fighting)`.
- **Architecture re-foundations** (each restructuring what previous sprints
  built): 007 (firmware architecture foundation, after 001–005), 014 (abandon
  fibers — runtime-model reversal), 016–020 (AppContext / BVC / MotionCommand /
  CommandProcessor / HAL overhaul), 025–029 (one dispatch path, navigation
  ownership), 034–037 (consolidation), 038–045 (Phase 0→F), 047–050 (state
  object, PID replacement, EKF replacement), 055–061 (message-architecture
  cutover). ≈ 7–8 distinct restructuring waves in 6 weeks.
- Only 7 textual `Revert`/`Reapply` commits — reversals happened at the
  *sprint* granularity (new sprint to undo a design), not the commit level.

---

# Appendix C: Evidence Extracts (Per-Era Reader Reports)

Nine parallel reader agents each extracted structured evidence from a slice of the project's
sprint directories, issues, tickets, and knowledge base, quoting verbatim from the artifacts.
This appendix is their raw output, in chronological order.

## C.1 Sprints 001–009

(Compiled by a reader agent from sprint.md, issues/, tickets/, architecture-update.md in each sprint directory. Quotes verbatim from artifacts.)

### Sprint 001 — HAL Layer and Project Skeleton
- **Goal**: Stand up the C++ firmware from a placeholder `main.cpp`: eight HAL drivers, type headers, Robot skeleton, boot announcement, `HELLO` response. A port of an existing TypeScript firmware ("plan-c-port-of-radio-robot-firmware" issue).
- **Type**: NEW-CAPABILITY
- **Ticket count**: 6. No issues/ dir.
- **Rework evidence**: None inside the sprint — but nearly every structure it created was later redone: `NezhaV2` (renamed/split in 008), `GripperServo` (renamed in 008), `Announcer` (deleted in 009), Robot owning `MicroBit` and the hidden `run()` loop (reversed in 007). Sprint 001's own architecture note — "Static instances in Robot... controls initialization sequence" — is precisely what 007's issue calls "wrong-placed."
- **Human-AI friction**: Ticket 006 is a whole ticket dedicated to "run python build.py and fix all compile errors," pre-listing expected CODAL API mismatches — the plan anticipated the AI writing code against imagined APIs and budgeted a fix-it pass.
- **Notable**: "Hardware-in-the-loop only — the CODAL framework does not support unit testing off-device" — no automated test safety net for any firmware sprint in this era.

### Sprint 002 — Control Layer and Core Motion Commands
- **Goal**: Make the robot driveable: MotorController (simple PI+FF), Odometry, CommandProcessor with drive-mode state machine, S-watchdog, motion/calibration K commands, wire-compatible with legacy Python host.
- **Type**: NEW-CAPABILITY
- **Rework evidence**: Planned-in-advance rework: "simple PI + feed-forward — ratio PID comes in sprint 5... Sprint 5 replaces the body only" — a deliberate throwaway control loop. The wire protocol it painstakingly matched ("must match TypeScript exactly") was hard-deleted in sprint 009. The K-command family (13 setters) replaced by `SET`/`GET` in 009. The CommandProcessor built here became 007's "God component."
- **Human-AI friction**: `architecture-update.md` is an **unfilled template** — sprint closed with its required architecture artifact blank. Process gate not enforced.

### Sprint 003 — Full Sensor and OTOS Command Set
- **Goal**: Feature parity with the TypeScript firmware for all 30+ commands.
- **Type**: NEW-CAPABILITY
- **Rework evidence**: Knowingly shipped a command destined for deletion; the gripper `G` was removed one sprint later, leaving the gripper **uncontrollable** until 009 restored it as `GRIP`.
- **Human-AI friction**: `architecture-update.md` again an unfilled template.
- **Notable**: All handlers piled into CommandProcessor ("No new files or classes in this sprint") — feeding the god-component problem 007 had to unwind.

### Sprint 004 — Ratio PID Motor Control and G Go-To Command
- **Goal**: Replace the sprint-002 PI+FF tick with a cumulative-distance ratio PID (ported from confirmed-working TypeScript), plus a two-phase `G` go-to command.
- **Type**: REWORK-OR-REFACTOR (planned) + NEW-CAPABILITY
- **Rework evidence**: Premise is redoing sprint 002. Also destroys sprint 003 functionality: "this is a deliberate scope trade-off — the gripper G command is sacrificed for the go-to G command" — un-sacrificed in 009.
- **Human-AI friction**: **Hardware acceptance criteria left unchecked in a done ticket** (ticket 004: build/deploy `[x]` but nine of eleven physical tests `[ ]`, status `done`). Issue spec's "What Not To Do" section reads as guardrails against anticipated AI implementation mistakes.
- **Notable**: Sprint-numbering churn: 004's own architecture notes refer to its content as "sprint 5" — sprints 004/005 were swapped from the original plan.

### Sprint 005 — Navigation Layer
- **Goal**: On-device navigation — PoseProvider/PathFollower interfaces, PurePursuit and Stanley, a `NAV` waypoint command. "a key design goal of the C++ rewrite: offload path following to the firmware".
- **Type**: NEW-CAPABILITY — **never executed**
- **Ticket count**: **0**. Tickets table empty; frontmatter reads `status: open` though the directory sits in `done/`. Only sprint.md exists.
- **Rework evidence**: Abandonment. Sprint 006: "Sprint 005 (Navigation Layer) — left untouched". Capability delivered the *opposite* way: sprint 009 copied the host's `robot_radio` package (incl. `nav/`, `kinematics/`) — navigation stayed host-side.
- **Human-AI friction**: A fully-written 143-line sprint plan silently shelved and archived as-if-done. No document records the decision. Major architectural pivot ("offload path following to firmware" reversed) with no written rationale.

### Sprint 006 — mbdeploy Package
- **Goal**: Consolidate three loose deploy scripts + ad-hoc device registry into a pipx-installable `mbdeploy` package with relay-flash protection.
- **Type**: PROCESS-OR-TOOLING (+ rework of sprint-001-era scripts)
- **Human-AI friction**: The relay-flash hazard ("a real hazard") implies near-miss incidents. Decisions section headed "Decisions (from the user):" — heavy stakeholder steering.
- **Notable**: First sprint with real unit tests and use-cases wired in; process compliance improved markedly here.

### Sprint 007 — Firmware Architecture Foundation
- **Goal**: Restructure the firmware built in 001–004: MicroBit out of Robot, visible main loop, DriveController extracted, unified RobotConfig, thin CommandProcessor, fix async-reply channel bug.
- **Type**: REWORK-OR-REFACTOR + BUG-FIX-RECOVERY ("keystone sprint")
- **Rework evidence**: Redoes sprints 001–004 structure. Issue catalog: "**`MicroBit` is wrong-placed.**... **`CommandProcessor` does far too much.**... **Config is duplicated and can diverge.**... **The main loop is hidden**... its `tick()` replies are hardwired to **serial** even when the command arrived over **radio**". Architecture-update names the CommandProcessor a "**God component**".
- **Human-AI friction**: Issue opens "the stakeholder wants it restructured" and records "**Stakeholder decisions (locked)**" — human explicitly overrode the structure previous sprints produced. Verification-deferral again: "Bench gate" box unchecked in a done ticket. Issue frontmatter still `status: in-progress` — bookkeeping drift.
- **Notable**: The async-reply bug shipped in 002 lived through five sprints. Decisions made here were themselves later reversed: Robot-facade model undone by 016; CommandProcessor refactored again in 019.

### Sprint 008 — Motor/HAL Layer: Vendor Coverage, Chip Velocity, Cleanup
- **Type**: REWORK-OR-REFACTOR + NEW-CAPABILITY
- **Rework evidence**: Corrects sprint-001 HAL design: "'Nezha' is the whole controller board, not a motor"; hardcoded signs should be per-motor config; "GripperServo → Servo... generic hobby-servo driver, not gripper-specific". The 0x47 velocity register "was only ever a `return 0` stub in the old TypeScript" — the port faithfully carried over a hole.
- **Human-AI friction**: FIXME markers scattered then stripped into a tracked issue. Distrust of hardware behavior explicit: "**Do not assume the laps→mm scale**... pin it empirically".
- **Notable**: Vendored the advisory `pxt-nezha2` driver in-repo so audits stop re-deriving the I2C protocol — early knowledge-capture pattern. Encoder/I2C read path touched here resurfaces as sprint 015.

### Sprint 009 — Protocol v2 and Host Controller Migration
- **Goal**: Hard-break rewrite of the entire wire protocol + migration of the Python host controller into the repo.
- **Type**: REWORK-OR-REFACTOR + NEW-CAPABILITY — largest sprint of the era
- **Rework evidence**: Discards the protocol 002–004 built to exactly match TypeScript: "We are taking a **hard break** — no backward compatibility... There is a lot of flux right now, so a clean break is cheaper than dual-parsing". Sprint 001's `Announcer` deleted. `GRIP` restores gripper control sacrificed in 004. All 24 K-commands (13 added five sprints earlier) replaced.
- **Human-AI friction**: "**Decisions locked with the stakeholder**" format recurs — suggesting prior sessions relitigated settled choices. Bench-deferral in a done ticket recurs. Cross-plan inconsistency the AI had produced flagged in the issue.
- **Notable**: Pivots the project's center of gravity to the host (`host/robot_radio/` arrives here). The copied-verbatim `nav/`, `kinematics/`, `controllers/` packages and the rewritten CommandProcessor get reworked repeatedly later (013, 017–020).

### Batch synthesis
Fast greenfield port (001–004) followed immediately by systematic un-doing of that port's structural choices (006–009), with sprint 005 abandoned in between. Two planned-rework moves were healthy; the larger pattern is unplanned: 007's issue is an itemized stakeholder correction of AI-built structure; 008 renames/re-splits the sprint-001 HAL; 009 hard-deletes the wire protocol 002–004 had matched "exactly". Process signals: architecture updates left blank in done sprints; sprint 005 archived open; recurring pattern of hardware/bench acceptance criteria left `[ ]` inside tickets marked `done` (004-004, 007-004, 009-008). Seeds of later rework: 007's Robot-facade (reversed in 016), CommandProcessor (redone 019–020), fiber main loop (redone 014), encoder I2C path (015, 064), wholesale-copied host nav/kinematics (rewritten 013, 017–018).

## C.2 Sprints 010–019

(Compiled by a reader agent. Quotes verbatim from artifacts.)

**Timeline: the entire batch ran in ~8 days (June 2–9); 016+017+018 executed the same day (June 8) — peak cadence three sprints/day.**

### Highlights per sprint
- **010**: fixes sprint 008's readSpeed — "its mm/s conversion is wrong and carries a bogus `RobotConfig::lapsToMmScale` (default 1980, ~11× off)" — an AI-invented calibration constant shipped and caught only when the layer above consumed it. Retires the sprint-004 RatioPidController as inner loop.
- **011**: replaces 004-era go-to ("no acceleration/deceleration profile... lives in the wrong place"). Its ad-hoc `_vRamped` trapezoid was itself called out and removed in 017/018 — capability rebuilt within ~6 days.
- **012**: entirely a correction pass over 007–011: "bench testing revealed the robot is not yet student-ready: TLM pose= reports raw OTOS LSB (looks ~5x wrong)... Compiled defaults are wrong: trackwidth 120 (should be 126)." Locked stakeholder directives against AI escape-hatches: "do not 'fix' by disabling sensors"; "Do NOT demote the chip." Closed messy: "T011 bench verification carried forward to Sprint 013."
- **013**: port of the prior-repo (pre-AI) host library to v2 — "the stakeholder wants the proven prior-repo library." Opens by cleaning 012's process wreckage ("11 tickets in unknown state"). Mid-sprint bench found motor "throb" → unplanned ticket 010: **two-fiber PID isolation with busy-wait I2C**. New standing rule: "henceforth no robot-interaction change ships without tests." Chip 0x47 readSpeed — fixed in 010, fixed again in 012 — **dropped entirely** here ("Throb fixed. 0x47 disabled at policy level").
- **014**: **direct reversal of 013's fibers one day later** ("single cooperative main loop — abandon fibers"). And 014's own headline split-phase I2C design was **reverted in-sprint**: "Reverted in practice — per-tick atomic read (8 ms busy-wait) restored closed-loop velocity." Bench ticket 009 has every acceptance criterion unchecked in the done file. A stale `static` debug block left in firmware caused the phantom "first drive works, second does nothing" bug.
- **015**: diagnosis-only wedge sprint; scope doubled mid-sprint (velocity_chart froze; firmware hard-hang from `setTxBufferSize(1024)` wrapping to 0 in a uint8 — "why the robot needed repeated power cycles throughout sprint-015 debugging"). Issue §2 lists **14 attempts across sprints 008–014**. Epistemics negotiation: "a plain mutex/lock would be a runtime no-op... We do not yet TRUST this analysis, so we will MEASURE it." Exit criterion not reached; "Sprint 016 (the fix sprint)" became the AppContext refactor instead; wedge still worked at sprint 064 a month later.
- **016**: deletes sprint 007's Robot facade ("~38 methods, ~43% pure passthroughs") for AppContext — **and the rename was reversed within ~15 hours** (git: delete Robot.h Jun 8 15:55; "Refactor AppContext to Robot" Jun 9 06:47). Structure kept, name rejected — overnight stakeholder correction.
- **017**: BVC + MotionCommand/StopCondition. Issue opens with the AI having produced **two conflicting design drafts** the stakeholder had to merge and rule on: "Per the stakeholder: we commit to yaw-rate (v,ω) control — NOT (v,radius) and NOT (v,ratio)." Durable decision.
- **018**: third rewrite of go-to motion shaping in 16 days (004 one-shot arc → 011 _vRamped → 018 MotionCommand).
- **019**: 1742-line switch → dispatch table; staged dual-mode migration ("keep the old switch path live until all commands are migrated") shows learned caution. DriveController→MotionController rename — the subsystem's THIRD name; eliminated again in 060/061 (Drive/Planner). Dispatch layer reworked again in 026 and 051.

### Batch synthesis (verbatim from reader)
A one-week, ~79-ticket burst in which the team built the robot's entire motion/estimation stack twice and its runtime model three times, with the hardware bench — not design review — as the real quality gate. Pattern: an ambitious layer ships against mocks and clean builds, then the next sprint opens with bench-discovered fallout. Signature reversal: fiber whiplash (013 unplanned → 014 abandoned → 014's own I2C redesign reverted in-sprint). Friction legible in artifacts: locked directives against AI escape-hatches, stakeholder arbitration between conflicting AI drafts, overnight rollback of the AppContext rename, AI-shipped defects (11×-off constant, uint8 wrap, stale-static debug block), process-gate erosion (DoR unchecked on 012–013, 016–019; done tickets with unchecked acceptance boxes in 013/014/015). Seeds planted: complementary fusion (rebuilt 022, replaced 050), MotionCommand/StopCondition (redone 052–053), dispatch table (026, 051), MotionController naming (060–061), and the undiagnosed wedge that shadowed the project into July.

## C.3 Sprints 020–029

(Compiled by a reader agent. Quotes verbatim from artifacts.)

### 020 — HAL Abstraction, Motion Overhaul, Command Queue (12 tickets; largest of batch)
- Motion system built in 017–019 had already forked: "two parallel code paths to the motor (legacy bypass vs BVC), duplicated keepalive watchdogs."
- **Planted two seeds of the 026 crisis**: (1) 020-011 put S/T/D/G/R/TURN→VW converters inside MotionController — the layering inversion later named A2, root cause of the double-OK defect; (2) 020-003/004 built `sim_api.cpp` **without wiring the CommandQueue** built in 020-010 — the sim/hardware dispatch split later called "the single largest reason 'it works in sim and fails on the field'". Queue and sim wrapper built in one sprint, never connected.

### 021 — Mock noise model + figure-eight demo
- Noise model got the **turn-slip sign wrong** (discovered sprint 024) and shipped **off by default** — 027: "'Passes in sim' has meant almost nothing."

### 022/023 — EKF pose + velocity fusion
- Heading fusion deferred TWICE ("OTOS heading is not fused... out of scope"); 024's D1 traces "gets turned around and drives into the boards" directly to this gap.
- Mahalanobis gate landed with no recovery path ("confidently wrong, forever" divergence trap D3).
- 023 test strategy: "Primarily offline… On-robot bench verification via rogo is optional and deferred" — three consecutive estimator sprints with essentially no hardware validation.
- 023 fixed 022-era bugs: setPose() spurious-jump; "the `OV` command comment says 'report velocity' but it actually calls `setPositionRaw()`".

### 024 — Field-safety P0 fixes (pure defect sprint, D1–D5,D7 from external review)
- Sources: `docs/code_review/2026-06-11-sim2real-architecture-review.md` and `2026-06-11-wild-spin-and-cursing-forensics.md`. Field failure: "robot goes wild and spins until I power it off".
- D2: "`rotationalSlip` (default 0.74)… is referenced in **zero firmware logic**" — calibrated, stored, registered, dead.
- D4: operator workarounds (SAFE off; a daemon streaming `+` every 150ms) "demoted the watchdog from motion-supervisor to dead-process detector".
- **Sprint failed its own field test the day it shipped**: "implemented and all 1434 host/dev tests pass, but the on-field behavior is not fixed" — same-day run "produced a full-speed spin that ended with the robot jammed into the boards. Same class of failure the sprint set out to fix" (recorded in 027's issue).
- `sTimeout=60000` overrides scattered in test fixtures were masking watchdog behavior — test scaffolding hiding the defect it should have caught.

### 025 — Trustworthy host I/O
- Headline bug corrupted ALL prior observability: "`SerialConnection.send()` calls `reset_input_buffer()` before every write, discarding in-flight TLM frames, EVT done lines, and safety_stop events… the primary cause of 'the stream keeps dying'."
- "Fixing observability before changing behavior avoids chasing ghosts in a broken stream."
- 025–029 planned together as one recovery roadmap with "Why First / Second / Third / Fourth / Last" sections.

### 026 — One dispatch path ("highest-risk sprint in the roadmap")
- "`host_tests/sim_api.cpp` never wires a CommandQueue… One OK reply. D6/D11 simply don't exist [in sim]."
- "`sim_api.cpp` **hand-mirrors** the LoopScheduler loop with a 'MUST mirror LoopScheduler.cpp exactly' comment — a divergence generator by construction."
- Verdict on the 020–023 test strategy: "**Every sim test validates a system that does not exist on hardware**."
- Issue names stakeholder frustration verbatim: "the direct cause of the repeated 'go run the actual simulator on our code' frustration."
- MotionController.cpp 1953 → ~900 lines.

### 027 — Behavioral fixes on the single path
- D6: the documented API itself taught the destructive pattern — host docstrings *recommended* the keepalive that stomps active commands; "firmware emits `EVT done TURN` as if it succeeded."
- Field-profile issue: "the sim validates a friendlier system than reality: OTOS/EKF fusion off by default, MockMotor slip = 0… 'Passes in sim' has meant almost nothing — and is the source of the recurring 'you didn't actually test it on our code' frustration."
- Operator quoted: "your program is supposed to detect problems… run it for a little bit and stop… forces the operator to lunge for the power switch."
- 027-006 finally root-caused the field-024 SNAP anomaly as benign tick-ordering after it consumed diagnostic effort across three sprints.

### 028 — Calibration and host consolidation
- "Calibration logic exists in four places… calibration outputs were calibrated, stored, and registered — but read by nothing in firmware." A7: "already cost real field-debugging sessions."
- a6 names an AI-specific failure mode: "the MCP path the main agent uses and the CLI path a human uses are not the same code… CLI/MCP drift directly causes 'works for the human, fails for the agent' reports." (left in-progress into 029)
- D10: telemetry "fights its consumers" — silent-idle by design, radio commands steal the stream, no seq numbers.

### 029 — Navigation ownership
- "The same closed-loop 'drive to a world point' capability exists in three stacks with no shared code, parameters, or pose state" + "three pose estimators with no defined authority… Every navigation bug must be hunted in three stacks."
- Host stacks existed because the firmware path was untrusted: "Until sprints 025–027 prove the firmware G path trustworthy on the field, consolidating onto it risks consolidating onto a broken target."
- Human wired in as hard gate: "If no agreement is reached, the sprint is blocked — do not proceed with implementation under ambiguity."
- Later history: a1 consolidation needed sprint 035 to fully land (029 tickets closed unimplemented).

### Batch synthesis (verbatim from reader)
Sprints 020–023 were a rapid, sim-first capability build executed almost entirely offline, planting three fatal seeds: queue and sim never connected; noise model off-by-default with wrong slip sign; heading fusion deferred twice plus a gate with no recovery path. The bill came due on 2026-06-11 ("wild spin and cursing"); 024–029 is the repayment, planned as one roadmap. Sprint 024 fixed all six confirmed root causes, passed 1,434 tests, and STILL spun full-speed into the boards the same day — forcing the lesson that observability (025), path unification (026), and test-profile realism (027) had to precede behavioral trust. Duplication was consistently a symptom of distrust: host navigators grew because firmware wasn't believed; keepalive daemons and sTimeout fixtures grew because the watchdog killed legitimate motion — each workaround then masked the defect it routed around. rotationalSlip resurfaces in project memory 2026-07-02 ("SET rotSlip silently no-ops for turns") — the config-consumption defect class outlived the a8 lint built to kill it.

## C.4 Sprints 030–037

(Compiled by a reader agent. Quotes verbatim from artifacts.)

### 030 — Fable round-2 correctness fixes N1–N16 (10 tickets)
External review (`docs/code_review/2026-06-12-Fable-correctness-review/findings.md`) found what the per-ticket sim-verified process structurally missed — cross-cutting invariant violations:
- **N2 (most damning)**: "sim wires+tests the queue path; firmware runs the immediate path… After the first safety stop/halt the firmware permanently switches to queued dispatch." Entire sim regression suite validated a code path the hardware never ran.
- N1: "every `D` teleports the fused pose backward by the prior segment's length."
- N3: "`SET tlmPeriod=100` with no prior STREAM → null fn-pointer call → HardFault"; header comment "describes a guard nothing implements."
- N4/N5: 4 of 7 begin*() entry points skipped cancel-if-active; "P1.1's own verify scenario... fails here" — an earlier sprint's acceptance scenario never actually passed.
- Silent classes: queue overflow after OK (N7), sticky validity bits (N8), `SET aDecel=-100` → NaN (N6), dead RatioPidController "constructed, SET-tunable, never run".
- Verification remained sim-only; 033 later found 030's N1 fix **incomplete** ("the snapshot still precedes the input zeroing").

### 031 — Bench OTOS debug sensor
Planted three bugs/debts that consumed 032–034: (1) `DBG OTOS BENCH` enable shipped broken (union-aliasing clobber — parser wrote `.ival` then zeroed `.fval` on the same union); (2) `DBG OTOS` used `%f` which prints NOTHING on newlib-nano, invisible in host sim ("host sim's libc has full %f"); (3) "chose the fastest path to a working feature: Robot::benchOtosTick downcasts hal to NezhaHAL*" — violating HAL-agnosticism, reworked in 034. Hardware bench execution explicitly out of scope, "deferred to post-sprint team-lead validation."

### 032 — Comprehensive bench validation
- Premise concedes the gap: "Sprint 030 and 031 delivered significant firmware changes... None of these were hardware-validated after the merge."
- Run confused by FIVE compounding bugs: wrong transport, bench-enable never engaging, `twist=` permanently zero (since 023!), D-after-TURN instant-complete, ambiguous wedge detector.
- The validation harness itself "parses TLM integers with wrong unit assumptions, producing absurd million-degree heading values and meaningless assertions."
- ALL-CAPS escalation: "STAKEHOLDER DIRECTIVE — bench testing uses the SERIAL PORT, not the radio... it is the root of most of this session's confusion" (`docs/code_review/bench-032-diagnosis.md`).
- False hardware accusation refuted: "I named the encoder before verifying with an equal-wheel test" (resolution: refuted).

### 033 — Bench-found firmware fixes
- Fixes 031 (union bug — bench mode never worked since it shipped), completes 030's N1, fixes 023 (`twist=` zero on hardware since EKF velocity fusion landed; "including any real-world OTOS dropout").
- Recorded dead-end spiral: "an earlier attempt reached for objdump and a 'nRF52 pointer-comparison' theory — both dead ends. The bug was found by reading the parser and adding one en=%d probe."
- Post-fix hardware validation succeeded (8/8) but immediately opened NEW findings (F1 float-printf) feeding 034. Fix-validate-find-new-bugs became the loop.

### 034 — Push actuator state through Hardware::tick (rework of 031)
- Issue headed "Stakeholder design direction (Eric, 2026-06-12)" — the human personally specified the target architecture.
- "The F1 integer-format fix is NOT verifiable by the host sim (host libc has full %f)" — on-hardware verification by stakeholder required.

### 035 — Pose authority consolidation (A1)
- All three issues: "Provenance: sprint 029 (navigation-ownership) ticket 002, closed unimplemented" — an earlier sprint's tickets closed without being done, resurrected here.
- "Firmware fixes from sprints 024–027... have no effect when an agent uses the host-side navigator or CLI inline controller" — months of firmware fixes bypassed by parallel host code.
- Deleted: pure_pursuit.py (216 lines), stanley.py (198), ltv.py (293); navigator.py 1349→~400 lines.
- Hard gates: "Do not begin until the stakeholder-approved design doc explicitly authorises deletion of the specific files."

### 036 — Stateful Robot object + Playfield
- Bench validation found **basic host-library functions had never worked against real hardware**: `get_id()` always returned None (reader dropped the ID reply); `refresh()` returned None (snap waited on a corr-id the reply never carries). "Both bugs exist on master." Tests mocked at layers that encoded the same wrong assumptions.
- T007 self-correction: "Corrected root-cause analysis (supersedes earlier ticket draft): dtr=False does NOT suppress/mute the relay" — first AI diagnosis wrong; knowledge note "contains a now-disproven claim."

### 037 — Consolidate tests into one tree
- "Three separate test roots force constant guessing"; circular-mean helper "duplicated in playfield_tour_camera.py, playfield_random_tour.py, and tests/playfield_tour/tour_goto.py."
- Five tour-script variants retired at once, incl. one sprint 036 had just rewritten.
- Only sprint of the eight with all Definition-of-Ready boxes checked.

### Batch synthesis (verbatim from reader)
This era is the project's reckoning with the sim-to-hardware gap; its shape is a correction loop rather than a line. Roughly half the era's engineering is rework of the era's or earlier eras' own output. Seeds visible at close: hardware verification chronically deferred to "post-sprint team-lead" checkboxes left unchecked (031, 033, 036, 037); Definition-of-Ready stakeholder-approval gates skipped in five of eight sprints; a knowledge base already carrying one "now-disproven claim"; a validation strategy that still trusts a sim whose libc, transport, and dispatch path have each been proven to diverge from the device.

## C.5 Sprints 038–045

(Compiled by a reader agent. Quotes verbatim from artifacts.)

### Arc-level context
The entire batch executes a single planning artifact: `042-.../issues/done/migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md`. Motivation: "Re-organize the firmware to adopt the *FRC Elite Architecture*... its three structural seams, its vendor/transport confinement rules, and its sim/test discipline."

The issue frames this as **rework of structure built in sprints 1–37**: "**The good news.** The codebase is already ~80% this shape... **This is a rename-by-capability + leak-sealing + seam-naming + sim-untangling job — not a rewrite.**"

Structures replaced map to earlier sprints: hal/Hardware factory (020), MockHAL/MockMotor noise model (021), EKF fusion (022–023), MotionController (017–018), the one-tree test layout (037 — re-tiered again ONE sprint later in 038). Stakeholder opportunity cost recorded: "**Decisions locked (this session):** full reorganization (all seams)... start now, **pausing the encoder-calibration mission**."

Migration fixed real latent defects in the old sim: "`sim_set_enc_l/r` lies (sets commanded rather than true travel); `BenchOtos` integrates commanded velocity while `ExactPose` uses true velocity... canonical midpoint-arc integration is triplicated" (040 sprint.md).

### Per-sprint highlights
- **038 (Phase 0)**: test tiers + three canaries (vendor-confinement grep gate, config field-pin, golden-TLM byte-exact). Re-restructures the test tree consolidated one sprint earlier (037). "The canaries are the migration's regression harness."
- **039 (Phase A)**: capability-typed device layer; renames sprint-020 HAL. Alias shims deliberately created for deletion in Phase F — planned double-touching.
- **040 (Phase B)**: PhysicsWorld replaces welded mock-sim from 021; fixed lying `sim_set_enc_l/r`. Pre-authorized escalation ladder in architecture-update (never needed). "Preserve the encoder sub-step expression verbatim... No algebraic simplification" — ULP-exact preservation to keep the golden-TLM canary meaningful.
- **041 (Phase C)**: PhysicalStateEstimate seam wraps EKF (022–023). Transition mirror kept for byte-identity, removed in F. The EKF file move quietly broke `coverage.sh` — not noticed until 045.
- **042 (Phase D)**: thin Superstructure; centralizes keepalive/SAFE/ESTOP scattered across loopTickOnce (from 017–018/024/026). Explicit anti-gold-plating guard stated three times.
- **043 (Phase E)**: subsystem wrapping, bodies verbatim.
- **044 (Phase F)**: scaffold demolition; "After this ticket the migration is complete. The codebase fully embodies the FRC Elite Architecture." REPLAY mode exited compiled-but-never-run.
- **045**: coverage to 86.2% simulatable; fixed the harness broken since Phase C. Coverage push surfaced latent defects carried verbatim through all six phases:
  - "SENSOR stop... on the QUEUE path (the sim's only mode) the stop is silently dropped" — real firmware bug found, pinned not fixed (test_sensor_stop_dropped_on_queue_path_documented).
  - "`startDriveClean` vs `startDrive`: NO live callers"; single-wheel ZOH branches "DEAD-IN-SIM".
  - "`RatioPidController.cpp` | **Dead code**... removed from live control loop by N13/030-010" — dead code migrated faithfully through all six phases.

### Batch synthesis (verbatim from reader)
Sprints 38–45 were an almost purely structural era: one master issue, seven phases, 35 tickets, essentially zero new robot behavior — contract was "structural changes only — no behavior changes... Move behavioral bodies verbatim," gated by three canaries. As execution, the phased approach demonstrably worked: every sprint's tickets/done matches its plan exactly, no reopened tickets or exceptions, pre-authorized escalation ladders never invoked, canaries held byte-exact through six phases — a strikingly clean run compared to debugging-heavy eras elsewhere. The costs were quieter: the issue's hard constraint of a per-phase "hardware bench smoke" vanishes from every sprint DoD after 038 (validation was sim-only); the coverage harness silently broke in Phase C and stayed broken until 045; REPLAY closed as an unexercised stub; verbatim-move discipline preserved dead code and a latent silently-dropped sensor-stop bug; and within ten sprints another phased migration (055–061) restructured much of this freshly-built architecture again — the "migration complete" declaration in 044-004 held roughly ten days before the next re-seaming began.

## C.6 Sprints 046–054

(Compiled by a reader agent. Quotes verbatim from artifacts.)

### 046 — Mecanum drivetrain (8 tickets)
- Planned on the premise "A second robot is now on the bench" — but both HITL tickets sit in tickets/done/ with **every hardware acceptance checkbox unchecked**; commit 71868df: "HITL mecanum calibration deferred to follow-up issue… (blocked: no mecanum robot on the bench)".
- The `#ifdef ROBOT_DRIVETRAIN_MECANUM` design "metastasized to **81+ sites across ~15 files**… The abstraction it was meant to localize (IKinematics.h, sprint 046) leaked."
- A full omnidirectional drivetrain shipped "done" **without a single motor ever spinning on real hardware**; the integration was deleted 5 days later in 048 — and mecanum was later rebuilt (togov is a live mecanum robot today): a complete build→delete→rebuild cycle.

### 047 — Robust robot state object
- Positive collaboration example: issue "intended for your review before any code is written," five design questions individually resolved ("Resolved decisions (stakeholder, 2026-06-27)").
- Motivation: the team didn't trust their own fusion — "The pre-fusion dead-reckoned pose is discarded"; "poseX is actually the EKF output, not raw dead-reckoning."

### 048 — (dir name is legacy) Eliminate ROBOT_DRIVETRAIN_MECANUM entirely
- A sprint that was itself re-scoped: original plan ("kinematics namespace alias") SUPERSEDED because it was "cleanup of the design introduced in sprint 046" that still didn't meet its goal.
- Stakeholder correction recorded: "The stakeholder wants the macro gone **completely**… 1. **Supersede sprint 048** — don't do the partial refactor then immediately redo it. 2. **Compile differential-only now.**" A fully ticketed sprint abandoned before execution.
- Even the deletion needed a redo: commit ef6b1fd "strip residual control-layer mecanum sites **missed in first pass**".

### 049 — Consolidate PID onto cmon-pid
- `RatioPidController` (sprint 004) tabled as "**Dead** — update() never called" — silently orphaned by architecture churn; 049 just buried it.
- Tooling landmines embedded in sprint.md: "Do NOT use bare `uv run pytest` — falsely reports mass failures."
- First appearance of the normalized broken-window baseline: "Known pre-existing baseline: exactly 2 failures" — waiver copy-pasted into every sprint through 053.

### 050 — Replace EKF with TinyEKF (parity-gated)
- Fact-check found TinyEKF "provides **only** the bare predict/update linear algebra… **Our EKF's hard-won robustness — χ² gating per channel, D3 gate-recovery, wedge-aware omega suppression… is exactly what TinyEKF lacks.**" The bespoke code survived because it encoded field experience no library had; only matrix arithmetic was swapped.
- Model for safe replacement: keep-old-run-new-at-parity-then-delete — the opposite of the 046 pattern.

### 051 — Declarative ArgSchema layer (9 tickets)
- **Key process failure of the batch**: validation ticket 009 checked `[x]` the spot-check "`S 99999` → `ERR range l`" — yet sprint 054's bench run on real firmware got "`ERR badarg l`". Root cause per 054: "the simulation tests… **used static string literals, not live firmware calls**, so the regression passed CI." A green validation checklist disproven by hardware five days later.
- 053's validation ticket warns: "Check for any ARM-specific compile errors not caught by the host sim build (**this has bitten the project before — sprint 051**)."
- (Also introduced, found in 064: query-mutates-state on DBG IRQGUARD and `RF` silently retuning the radio to channel 0.)

### 052/053 — Stop conditions Phase 1/2
- Debt being redone is named: "leftover scaffolding from incremental 'behavior-preservation' seams (sprints 026/042). It is exactly the mirroring we want gone."
- 053 deliberately rebaselined the golden-TLM canary ("must be reviewed — not blindly accepted") — the primary behavior-preservation instrument invalidated by design during the refactor.
- Pace: 051+052+053 (20 tickets) all executed ~13:00–16:31 on 2026-06-28; 052 in ~30 minutes; DoR "Stakeholder has approved" unchecked in closed artifacts.

### 054 — ERR range vs badarg fix
- "Found during **post-roadmap bench validation of sprints 048–053**" — five refactor sprints ran on sim-only gates; first bench contact immediately found a wire regression.

### Batch synthesis (verbatim from reader)
Of 9 sprints (51 tickets), only 046 and 052 add capability; the rest refactor, replace, or repair. The two defining failures are both validation gaps, not coding gaps. The middle sprints show the process at its best (047's reviewed design; 050's parity gate). But the June-28 blitz outran the test oracle: the era's motto of "byte-identical behavior" was enforced by oracles that 053 had to deliberately re-baseline and 054 had to substantially rewrite — refactoring safety was only as good as sim fidelity, and hardware kept issuing corrections the moment it was consulted.

## C.7 Sprints 055–061

(Compiled by a reader agent. Quotes verbatim from artifacts.)

**Timeline: the entire 7-sprint, 32-ticket program executed between 2026-06-29 23:15 and 2026-06-30 21:56 — under 23 hours. Phases 1–3 (16 tickets) ran overnight 23:33–03:09.**

### Key events
- **055**: one-ticket mechanical rename ran with full sprint ceremony (issue itself said "should go through /oop"). The "2 pre-existing failures" baseline rides along the whole batch.
- **056**: proto-as-schema codegen (libprotobuf/nanopb infeasible on 128KB no-heap target). Shipped a **known name collision** the next sprint fixed.
- **057**: Drive2/Sensors subsystems, additive. Umbrella issue "replaces three earlier 'design-only' plan issues... Design docs are not the deliverable... the phases deliver code." The `2`-suffix scaffolding names born here consume tickets in 060, 061.
- **058**: whole sprint is a stakeholder correction: 057's `test_ekf_fusion_beats_noise` "injects error only into the OTOS path; the encoder reads ground truth perfectly. This proves 'EKF discards a bad OTOS and trusts a clean encoder' — NOT genuine sensor fusion." "The stakeholder explicitly asked for error versions of BOTH." Guardrail: "If the EKF cannot beat both raws... report it rather than weakening the test." Correction cost ~25 min wall clock.
- **059**: cutover ticket hit its pre-authorized fallback: "FALLBACK APPLIED — #ifdef USE_ORDERED_TICK is in place. Default is legacy path" with 3 documented parity gaps, one "defeating the cutover." TURN parity tolerance quietly relaxed 2°→5°. Bench smoke ticket closed with every hardware checkbox unticked ("deferred to human-operated bench run (team-lead)") yet carried `completes_issue: true`.
- **060**: closes 059's gaps, flips default, deletes legacy loop. **The parity oracle was reset mid-migration**: golden-TLM regenerated, "The stakeholder reviews and accepts the diff; the new values become the canary" — parity ultimately proven against a refreshed baseline, host-side only. Issue fenced the AI: regeneration "must be a deliberate, reviewed acceptance... not an autonomous rubber-stamp"; "Bench parity on real hardware must be confirmed before legacy deletion is final" — in practice legacy was deleted (09:20) and merged (09:58) with the physical run still deferred. Retained MotionController spawned 061.
- **061**: third consecutive sprint finishing the same cutover ("The stakeholder wants no legacy code left at all"). Even 061's scrub ticket missed things → unplanned ticket 008. Stakeholder imposed bench-before-merge on the branch; ticket 007 still closed "(Physical bench run deferred to stakeholder — not a ticket-done criterion)"; merged ~3h later same evening.

### Late-discovered breakage traceable to this batch (surfaced 07-01/02 in real use)
- `set-config-not-propagated-to-planner.md`: "SET rotSlip=1.0 replies OK... but the Planner holds a boot-time private copy of RobotConfig... the consumer never sees the value" — interaction of 059-004 annotation SET routing with 061's absorb-into-Planner. Root cause of the hardware 90°-turn over-rotation (2026-07-02).
- Outlier filter "lost its recovery path in the sprint-060 cutover" (per the 07-01 full-codebase review; fixed in 064).

### Synthesis (verbatim from reader)
A single 23-hour, 32-ticket, AI-executed replatforming onto an externally sourced FRC-style design; disciplined *structural* risk management combined with systematically deferred *empirical* validation. Each "final" sprint spawned the next (059's fallback → 060; 060's retained class and shims → 061; 061's missed scrub → in-sprint 8th ticket) — roughly half the batch is rework of the batch's own output. The human's role was concentrated at exactly the right two choke points (rejecting the vacuous fusion test; requiring reviewed golden regeneration), yet the one gate the humans reserved for themselves — physical bench parity on tovez — was deferred out of every sprint's done-criteria, and the legacy path was deleted and merged before it ran; the cost surfaced 24–48 hours later as latent field defects. Sim-parity gates and checklists substituted for hardware evidence at merge time, and the regressions that mattered were precisely the ones only hardware (or config-propagation) testing could catch.

## C.8 Sprints 062–066

(Compiled by a reader agent. Quotes verbatim from artifacts.)

### 062 — TestGUI (PySide6) + baseline fixes (10 tickets)
- The two "baseline fix" tickets cleared CI failures **tolerated as known noise for ~8 sprints (054→062)**.
- Stakeholder gate on golden refresh: "do NOT rubber-stamp the snapshot."
- The GUI's drive design ("repeatedly send VW ±v 0 on a ~100 ms QTimer (doubles as keepalive)... On release → send STOP") is exactly the pattern sprint 065 later classifies as safety defects CR-04/05 — new capability shipped with latent safety debt.

### 063 — Mode-driven TestGUI (11 tickets; planned 3, grew to 11 mid-sprint)
- "Ship, operate live, file issues, extend sprint" loop: tickets 004–006 from new stakeholder requests; 007–011 from live bugs.
- **Knowledge base ignored → same bug re-solved**: ticket 002's relay probe "re-introduced the passive-banner assumption" already refuted in `.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md`; ticket 010 redid it with HELLO-classify "which is what SerialConnection already does."
- **Threading bug pattern hit twice**: "a QueuedConnection to a non-QObject callable is delivered on the worker thread (the same behavior that caused the tour/GOTO segfault)."
- Fix unmasked half-finished design: repairing frame delivery exposed the avatar TLM-vs-camera fight ("jumps all over the place").
- **Out-of-process work introduced defects the sprint absorbed**: "Introduced with the Tour feature (out-of-process work on 2026-07-01)."
- 062's `_set_origin` button was "display-only" — looked functional, sent no wire command.

### 064 — Encoder pipeline hardening (6 tickets)
- **The long wedge arc (015→033→051→060→064)**: 015 IRQ guard; 033 wedge detector + EKF gating; 051 ArgSchema migration silently broke the guard's query ("a bare DBG IRQGUARD query silently disables the guard"); 060 cutover silently dropped the outlier filter's recovery; 07-02 stand session found **two new triggers the guard never covered** and "**EVT enc_wedged fired for NONE of ~18 episodes**" — ten sprints of defenses, 0% detector recall on real episodes.
- The tooling sabotaged its own experiment: "the harness preflight queried the guard and thereby disabled it" — contaminating the stand-repro baselines.
- Audit found a worse sibling: a bare `RF` "silently retunes the radio to channel 0 and persists it to flash, breaking the link."
- Audit also found: "every D command currently fires the full hardware burst twice" — flagged as Open Question, not silently fixed.
- Human ran a controlled 5-arm stress matrix on the stand; sprint plan cites arm numbers per fix. DoR: "auto-approve session."

### 065 — Stop reliability and safety (ACTIVE; 001–005 done, 006 open)
Three defects, three provenances:
- **CR-01 (new-architecture integration defect)**: stop-clause double-booking between Planner::beginDistance and Superstructure::requestGoal → assert(false) "aborts the whole Python process hosting the sim."
- **CR-04/05 (long-standing, amplified)**: "the same 'watchdog silenced by keepalives' mechanism from the June wild-spin postmortem, now structural"; exposure amplified by 062's KeyboardDriver over a link that "drops 15–50% of lines."
- **CR-06 (regression of an older fix)**: "A 2026-06-17 change set healthy = poseOk… reopening the exact 'spin on placement' failure the original D9 gate (027-005) existed to prevent… the implementation lost the transient-vs-persistent distinction."

### 066 — Sim fidelity (PLANNED, roadmap only)
- "the sim OTOS can never disagree with the encoders except via injected noise (so EKF fusion is validated in a regime that doesn't exist on hardware)"; past regression db11b7c (433mm phantom translation on a pure spin) had "zero sim coverage" — success criterion: "a db11b7c-style regression now fails in sim."
- "Existing tests that relied on OTOS==encoders may need updating — that agreement was the bug."
- Encoder-track bug on its **third iteration**: "the original 'encoder track ignores turns' bug survives on exactly the transport (relay/playfield mode) where it matters."

### Synthesis (verbatim from reader)
This era is the bill coming due after the architecture program (055–061), paid down via a new feedback instrument (the TestGUI) plus two deliberate audit events. The moment a human operated the GUI live, defects poured out (063 grew 3→11 tickets). Rework here is overwhelmingly **regression archaeology with explicit provenance** (lost in 060 cutover; arrived with 051 ArgSchema; reopens D9/027-005; re-introduced an assumption already refuted in knowledge). The era's distinctive process moves: a full-codebase review generating a numbered CR-01..15 backlog scoping sprints 065–066 wholesale; issues written with confirmed file:line mechanisms before ticketing; auto-approve sessions replacing per-sprint sign-off while HITL validation is consistently deferred to the human; and a recurring failure shape — fixes validated in sim or on fast links that fail on hardware or the 1–2 Hz relay. The AI's leverage is exhaustive audit; the human's leverage is live operation, bench experiments, and SSOT/safety adjudication.

## C.9 Backlog, Reflections, Knowledge Base, and Docs

(Compiled by a reader agent from clasi/issues/, clasi/reflections/, .clasi/knowledge/, docs/knowledge/, docs/. Quotes verbatim.)

### Open backlog character (8 issues)
Almost none is a new feature. Three kinds: (a) fixing things already declared done, (b) closing sim-vs-hardware fidelity gaps, (c) cleanup batches from the stakeholder-requested full-codebase review (`docs/code_review/2026-07-01-full-codebase-review.md`, CR-01..CR-15).

- `set-config-not-propagated-to-planner.md` — `SET rotSlip=1.0` replies OK, reads back correctly, but Planner holds a **boot-time private config copy**; shipped calibration "only 'works' because it coincidentally equals the compiled-in default."
- `sim-error-model-runtime-settable-hardware-fit.md` — "the simulator and the real robot must be tunable to behave identically"; a "zero-error" sim **structurally over-rotates** because firmware compensates for 8% scrub the sim plant cannot produce — "the 45° case only looks right because two errors cancel."
- `sim-otos-fidelity-ground-truth-and-lever-arm.md` — sim OTOS re-integrates commanded wheel speeds instead of sampling plant truth, no lever arm: "the simulator [is] blind to the OTOS bug classes that have hurt the most on hardware."
- `tlm-three-world-poses-encoder-only-pose.md` — host-side TestGUI integrator "is a defect factory," citing three dated incidents.
- `testgui-trace-correctness-slow-tlm-and-anchor-rotation.md` — encoder-track bug fixed 2026-07-01 "survives on exactly the transport (relay/playfield mode) where it matters."
- `expose-sim-error-model-knobs-in-testgui.md` — sim error models built in 057/058 never plumbed to operator panel; "the gap is exposure, not modeling."
- `landmine-cleanups-...` — `Planner::apply()` hard-codes `now=0`; untested world-frame convention stack = "the exact 'guessed geometry' class behind past incidents."
- `small-cleanups-...` — leftovers of retired designs (probe_devices still sends the retired `>PING` protocol).

### Deferred work — the mecanum arc
`later/mecanum-robot-hitl-calibration-and-playfield-verification.md`: HITL half of 046-008 deferred ("no devices found" at close). Then `done/eliminate-ifdef-robot-drivetrain-mecanum-everywhere.md` records mecanum support metastasized into "**81+ sites across ~15 files**" of `#ifdef`; sprint 048 "**ticketed but not executed**"; stakeholder decided to **supersede 048 and delete all mecanum integration** ("don't do the partial refactor then immediately redo it"). Arc: feature built → abstraction leaked → planned partial cleanup abandoned → feature deleted; hardware validation still parked in later/.

### Reflections (exactly one exists)
`2026-06-12-consult-docs-first.md` (category: ignored-instruction, sprint 032): agent "spent roughly two hours reverse-engineering the relay's behavior from first principles" when the answer (!GO handshake) was on the documented site the stakeholder had previously pointed to. Stakeholder: "you didn't read the documentation… Please ensure you always remember where the documentation is." Root cause: unrecorded instruction — docs URL mentioned in an earlier session, never persisted. Lesson promoted into CLAUDE.md.

### Knowledge docs
- `.clasi/knowledge/2026-06-12-relay-go-data-plane-and-docs.md` — relay protocol knowledge, ~2h burned re-deriving; **bench-verified CORRECTION dated 2026-06-13** declares a central claim of the original doc "**FALSE**" — even hard-won knowledge capture needed a correction cycle one day later.
- `.clasi/knowledge/2026-07-01-heading-reset-needs-oz-not-just-si.md` — firmware has three heading/pose sources; SI resets only two. **The sim did not reproduce the bug** — in sim OZ was a no-op so the drift-back couldn't exist; sim had to be fixed to be able to exhibit the hardware bug before a regression test was possible (063-006).
- `docs/knowledge/i2c-sensor-detection-and-bus-wedge.md` (06-05) — "the day we lost to a cold-boot timing problem": wedged I2C persists across reflashes (battery-backed rail) so "the same fix was tried multiple times… and appeared to fail"; plus stale incremental builds flashing old hex while reporting success. No clean-slate discipline existed; paid for in a full day.
- `docs/knowledge/encoder-wedge-nrf52-twim-irq-load-errata.md` (06-07, "Status: RESOLVED") — five red herrings; July-1 note stamped on top: "**partially superseded… 'eliminated' overstates it**" — repo's own DefaultConfig.cpp comment (06-17) recorded the wedge persisting at 4–12% after the fix; nobody reconciled the contradiction for ten days.
- `docs/knowledge/2026-07-01-encoder-wedge-boundary-latch-flavor.md` (13.5 KB) — after the message-architecture rebase "the encoder wedge is back"; re-analysis dismantles prior belief ("The wedge never actually went to zero pre-rebase; 'we got away from it' was partly exposure/perception"). 07-02 stand session isolated **two triggers, both unaffected by the IRQ guard**; wedge detector fired for "none of ~18 observed episodes"; and a bare `DBG IRQGUARD` **query silently disables the guard** (ArgSchema regression from sprint 051) — earlier A/B debugging may have been self-sabotaged. Confident RESOLVED framing in the knowledge base actively misdirected the second investigation.
- `docs/knowledge/encoders-read-zero-i2c-bus-hang.md` — the C++ port silently dropped a retry idiom the TypeScript had — a port-fidelity gap only hardware could reveal.
- `docs/knowledge/watchdog-uint32-underflow-velocity-notches.md` (06-06) — "one-line class of bug and it masqueraded as a dozen different problems"; symptom presented on hardware timing the sim's synthetic clock never produces.
- `docs/knowledge/loop-timing-and-control-frequency.md` (06-05) — control tick 11.4 ms (8 ms vendor-mandated busy-wait), ~42 Hz net; fundamental physical constraints quantified only five weeks into firmware work.
- `docs/knowledge/field-log.md` — five entries across three dates, most checks SKIP, FAILs on 06-20 and 06-27 (RT×4 closure, G-square repeatedly failing) while ~30 sprints closed in the same window — hardware acceptance neither routine nor gating.

### Docs inventory
`docs/architecture/architecture-034.md` consolidates updates 001–034 ("Naming/structure has churned heavily since the sprint-001 skeleton"); since then **30 further per-sprint updates (035–064) accumulated unconsolidated**. Updates 038–045 timestamped hourly on a single day (2026-06-19, 08:48→14:52); 048–053 likewise on 2026-06-28 — six to eight architecture revisions per day.

`docs/code_review/` shows repeated external correctness passes, each spawning remediation sprints: 2026-06-11 modularity review (source of god-object split), wild-spin-and-cursing forensics, 2026-06-12 Fable correctness review, bench-032/033 diagnoses (bench-033 root-caused the harness itself), 2026-07-01 full-codebase review. The last concedes regressions from the project's own refactors: "the outlier filter… **lost its recovery path in the sprint-060 cutover**"; a "safety-architecture regression" (watchdog "only protects against host-process death, not host-logic failure"); and "subagent dispatch was unavailable for most of this session (harness classifier outage)."

### Synthesis (verbatim from reader)
Forward velocity was real, but a large fraction of effort went into **re-earning things already marked done**, and the recurring root cause is a **validation surface that systematically diverged from the hardware it claimed to represent**. Success signals lied outright — SET replies OK while the consumer keeps a boot-time copy; a query command silently disables the guard it queries; a stale hex flashes with a success message. The one channel that could catch this — hardware acceptance — was thin and non-gating, so bugs surfaced late, under incident pressure, at 10–100× cost. The human+AI collaboration pattern is a corrective loop rather than a preventive one: the stakeholder repeatedly intervenes with ground truth, the agent responds with high-quality forensics and process patches, and big refactors keep reopening settled ground because the tests that guarded it encoded the sim's fictions, not the robot's physics. The current backlog — nearly 100% fidelity repair — is the project's own diagnosis of the same conclusion.

---

# Appendix D: The AprilCam Comparison (H8)

Follow-up analysis. AprilCam (repo `../AprilTags`) is the camera/perception system that serves
as the robot's ground-truth pose source. It predates this repo — first commits September 2025,
with TypeScript and C ancestors before that — and was developed interleaved with it through
June 2026 by the same human, the same AI tooling, and the same CLASI process. That makes it the
closest thing this portfolio has to a **control group**: same team, same method, different
system legibility. One reader agent extracted evidence from its full artifact tree; findings
below, quotes verbatim.

But "control group" alone understates its role, and the first draft of this appendix made that
mistake. AprilCam was simultaneously **inside** the robot project's loop as its reality
instrument: a daemon + MCP server + Python client library that the robot repo consumed directly
from the first week of the main build — sprint 012's "camera-truth verify script" (Jun 2, with
the robot registered as AprilTag 100), `goto_tag.py`, camera-validated calibration tooling,
`nav/camera_goto.py` (035), the Playfield module (036), `playfield_camera_run.py` (Jun 19,
strafe leg Jun 23), live `aprilcam` MCP tools in the robot repo's own `.mcp.json` used by the AI
in-session, and the standing "read the camera + geofence before driving" operating rule. It was
an *inhabited, mm-scale, overhead analysis surface the human drove the robot under* — much
closer to a cockpit than a bench chart. That fact creates a tension with the main report that
must be resolved rather than smoothed over: **an honest, precise reality instrument was present
essentially the whole time, and the sim-vs-reality gap persisted five weeks anyway.** The
resolution is in the H2 amendment and Appendix A experiment 6: the instrument had no structural
standing — session-scoped, human-gated, absent from every merge gate — so it lost, week after
week, to a generation loop that ran while it was dark. AprilCam's presence makes the gap *more*
damning, not less: the project didn't lack truth; it declined to make truth load-bearing.

## D.1 The record

**Two generations, ~29 sprints.** The 15 sprints in `.clasi/sprints/done/` (May 14 – Jun 19) are
the *second* CLASI generation. A first generation of **14 sprints** (Mar 23 – Apr 11,
`001-project-restructure-cli-foundation` … `014-oop-refactoring-and-video-based-test-system`)
lived at `docs/clasi/` and was deleted from the tree on 2026-05-14 when the process restarted.
Much of gen-1's product — in-process MCP image tools, multi-camera compositing, an in-process
detection loop, a web server, an April "Remote-Pi Deployment MVP" — was later deleted or rebuilt
by gen-2. Commit cadence: 5 commits Sept 2025, dormant Oct–Feb, then 214 (Mar), 29 (Apr),
191 (May), 211 (Jun): the June robot-integration month is as heavy as the March bootstrap.

**AprilCam looped too — but its loops closed in days.** Roughly 7 of 15 gen-2 sprints are
primarily rework, consolidation, or bug recovery. Sprint 001's OS-pipe IPC "is deleted entirely"
by sprint 002 *two days* after it shipped (001's own verification ticket couldn't run — camera
contention, the very problem 002's daemon then solved); 002's ad-hoc msgpack protocol was
replaced by 004's gRPC *three days* later. There is no multi-sprint field-crisis arc like the
robot's wild spin; the closest events are compressed bursts (the Jun 13–16 coordinate-convention
storm — six correctness fixes in four days, landed on master outside sprint ceremony — and a
~3-day Raspberry-Pi CSI bring-up captured in `docs/knowledge/raspberry-pi-camera-setup.md`).
Each rework sprint "retired a whole class of failure rather than reopening the same wound."

**The same authority-consolidation disease, four rounds.** (1) *Camera ownership* (sprints 002,
014): "the camera is opened by whichever process gets there first… or any of the other ~7 sites
that call `cv.VideoCapture`"; finished only when 014 added grep-enforced invariants ("The daemon
is the *sole* camera owner"). (2) *Filesystem-path authority* (007): MCP and daemon each derived
`data_dir` from their own CWD — "these paths diverge silently — paths.json is written to the
wrong location." (3) *Geometry authority* (012): "Today AprilCam has **two disconnected notions
of 'playfield'** and they disagree" — the canonical 134.3×89.3 cm field vs per-camera
calibrations storing "its *own* `playfield {width: 109, height: 79.5}` plus static_markers whose
world coords are nonsense (three 'corners' on the same line)"; a third stale geometry (101×89)
was purged Jun 16; the fix invalidated every existing calibration ("The stakeholder has accepted
this consequence"). (4) *Vision-compute authority* (015): in-process MCP detection deleted
("confirmed stakeholder decision") — with a residue issue still open at project end. This is the
robot repo's R4–R6 pattern (one dispatch path, pose authority, navigation ownership) reproduced
in a different codebase: **"many writers, one truth" consolidation appears to be a signature of
how AI-built systems accrete**, not a robotics artifact.

## D.2 The truth instrument was itself wrong

The robot project's designated reality channel shipped, at various points, all of the following
— none of them visible in AprilCam's own display:

| when | defect | how found |
|---|---|---|
| ≤ May 17 | **Homography silently never loaded** — a swallowed TypeError meant "`tag.wx` and `tag.wy` are always None even when a valid calibration file exists" | code inspection during protocol work; "nothing visual flagged it" |
| ≤ May 26 | **Parallax**: robot tags ~118 mm above the field "displaced toward the camera's nadir," >7 mm error | physics reasoning during sprint 008 planning |
| May 26 – Jun 16 | **Frame-convention churn**: y-axis inverted, then un-inverted ("world +y = north — drop stale y-flip"), origin moved (tag-1 vs field-center), yaw in pixel-frame vs ENU | mostly by the *robot* misbehaving |
| Jun 13 | **The ENU flagship**: "The robot drives to the wrong edge: forward motion works but every sideways/lateral component is inverted" — positions y-up, `orientation_yaw` pixel-frame, `heading_rad` from y-down velocity. "The docs are already correct; the producer code is not." | **the robot driving wrong — not the view** |
| Jun 13 | Weeks of world positions reported "against a broken frame" (the nonsense calibration geometry above) | sprint-012 planning inspection |
| ~Jun 27+ | **Off-center tag mount**: +43 mm phantom offset that masqueraded as a robot OTOS regression (robot repo memory: "'regression' was +43mm off-center tag (fixed camera-side)"); answered structurally by mobile-tag mount-pose registration | robot-side debugging session |

This grants the stakeholder's requested grace with receipts: a real fraction of what the robot
project experienced as "the robot is wrong" was **the ruler being wrong** — and it complicates
the main report's H2 prescription. "Consult reality more often" presumes reality's readout is
trustworthy; here the readout was itself a system under construction. (It does not weaken H2's
core: the sim divergences — N2, the string-literal oracles, the agreeing-by-construction OTOS —
were all robot-repo-internal and stand regardless.)

## D.3 What the comparison isolates

AprilCam's defect economics split cleanly along one line:

- **Pixel-legible defects** (BGR/RGB swaps, overlay rendering, layout, camera contention) were
  caught same-day, usually on first manual use, because "the validation surface *is* the
  product" — a human watches the annotated live view constantly. These never accumulated.
- **Frame-semantic defects** (origins, handedness, yaw conventions, parallax, calibration
  geometry) — the *numeric* output, which is precisely what the robot consumes — were invisible
  in the view, because "the viewer round-trips its own conventions": a y-flipped world frame
  renders perfectly. Every one of them was found by code inspection or by the downstream
  consumer physically misbehaving, and they cluster in the June robot-integration window,
  landing as out-of-ceremony hotfixes.

So the two projects failed in the *same* place under *different* costumes: the robot's sim was a
proxy that agreed with itself while diverging from hardware; AprilCam's viewer was a proxy that
agreed with itself while its exported conventions diverged from the standard. **A validation
surface catches only the defects it renders. Self-consistency is not truth.** AprilCam's overall
smoother ride (loops of days, no crisis arcs, no reflections ever needed) is the legibility rule
of Appendix A working as predicted — and its one blind spot produced exactly the defect class
that leaked across the project boundary.

Process footnotes from the AprilCam record that echo this repo's: the DoR "Stakeholder has
approved the sprint plan" checkbox is unchecked in 12 of 15 closed gen-2 sprints;
`.clasi/reflections/` is empty despite 94 dispatch logs; the highest-stakes correctness work
(the convention storm) happened outside sprint ceremony entirely. The process weaknesses were
portfolio-wide, not repo-specific.

## D.4 Implications (extends the recommendations)

**R16 — Validate the instrument before it adjudicates.** Any system used as ground truth for
another gets its own acceptance regime *first*: calibration against an independent physical
reference (a tape measure beats a debate), cross-instrument consistency checks (camera vs
odometry vs ruler on a static scene), and a "who watches the ruler" test suite. The +43 mm
off-center tag cost a robot-side debugging session because the instrument was assumed correct by
default. Reality's readout is code too.

**R17 — Contract-test consumed semantics between co-evolving systems.** The frame-convention
storm survived AprilCam's own tests, its viewer, and the robot's sim — it lived in the *seam*.
When two systems co-evolve, the seam needs its own oracle: a shared, executable convention test
(known tag at known pose → asserted world_xy, yaw, in the standard frame, run in both repos'
CI), versioned interface schemas, and the rule that any convention change fails the partner's
build before it reaches the partner's field session. `read_cam_pose`'s argument-order gotcha and
the `t.yaw` confusion were the same seam leaking at the API level.

**R18 — In a portfolio, stagger the truth-critical work.** Two AI-speed workstreams sharing one
human meant the instrument and its consumer were *both* in flux during the June integration
window — divergences had no fixed point to be measured against. The cheap version of the fix:
freeze one side (declare the camera's conventions done, contract-tested, and versioned) before
opening the other side's integration sprints. AprilCam in fact reached that state on Jun 27
("sole vision authority," last commit) — the robot's smoothest instrumented era (the TestGUI
loop) began immediately after, which is the portfolio-level version of "make truth cheap first."

## D.5 The sharpened rule

Appendix A closed with: *AI productivity tracks the legibility of the system's truth to its
human.* The AprilCam comparison adds the precision that the whole portfolio's history turns on:

> **Legibility must cover what the consumer consumes, not what the surface renders. Every
> validation surface — sim, viewer, test suite, checklist — catches only the defect classes it
> makes visible; the defects that cost weeks lived exactly in each surface's blind spot. The
> practical test of a feedback surface is not "does it show the system working?" but "which
> failures would it render invisible?" — and that question, asked of the sim (scrub, silicon,
> timing), the viewer (frames, conventions), and the checklists (deferred hardware), predicts
> this portfolio's entire regression catalog.**

And the dual-role correction adds the rule's second axis, temporal rather than optical: **what a
surface renders only matters while the surface is running, and only if something depends on it.**
AprilCam rendered exactly the right thing — true world pose, millimetre-scale, live — and still
changed nothing for five weeks, because it ran in scheduled sessions while the code was generated
around the clock, and no gate ever waited for its answer. Truth that isn't in the loop is
advisory, and advisory truth arrives after the merge.
