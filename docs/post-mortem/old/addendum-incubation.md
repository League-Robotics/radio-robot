# Addendum: The Incubation Hypothesis (H7)

**Date:** 2026-07-02. Follow-up to the main [post-mortem](README.md), examining a stakeholder
hypothesis raised after the report was delivered.

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

**2. The May pause (H7a, positive).** After sprints 001–005 (May 20–23), the project paused
eight days. The first act on return was not sprint 006's tooling — it was a formal re-initiation
(Jun 1: "Add feature specification and use cases") followed within days by sprint 007, an
itemized stakeholder correction of everything structural the AI had built: "MicroBit is
wrong-placed… CommandProcessor does far too much… Config is duplicated and can diverge… the main
loop is hidden." Nothing in the record shows new external information arriving during the pause.
The most parsimonious reading: distance produced the diagnosis.

**3. The burst–gap rhythm (H7a, systemic).** Commits per day show the project's true shape —
AI bursts separated by near-zero human days:

| period | commits/day | what it was |
|---|---|---|
| Jun 2–5 | 37–100 | churn burst (sprints 010–015) |
| Jun 6–7 | 2–6 | gap — bench days (wedge "eliminated" Jun 7) |
| Jun 8–13 | 25–96 | burst (016–037, incl. the field crisis) |
| **Jun 14–18** | **1–7** | **gap** — FRC Elite design work happens here |
| Jun 19 | 96 | Phase 0→F migration, all 8 sprints in one day |
| Jun 20 | 23 | field day (field-log: FAIL, PASS-with-SKIPs, FAIL) |
| **Jun 24–27** | **2–7** | **gap** — field FAILs Jun 27; 048 supersede decision forms |
| Jun 28 | 143 | the blitz (047–053, 20+ tickets in an afternoon) |
| Jun 29–30 | 21–126 | overnight replatform (055–061) |
| Jul 1–2 | 47–87 | full-codebase review + archaeology sprints |

The pattern is consistent: **each gap emitted the project's best planning artifacts** (the
Jun 14–18 gap → the Phase 0→F master issue and its canary discipline, the cleanest arc of the
project; the Jun 24–27 gap → the "supersede 048, don't partial-fix" decision and the 048–053
roadmap; the Jul 1 review → CR-01..15 and sprints 065/066). And each burst then **added 10–30
tickets of new surface that no gap ever processed** before the next burst built on top of it.
Incubation was not absent — it was operating on stale state, always one burst behind.

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

**6. The TestGUI effect (H7c, strong positive).** For five weeks the robot's internal state had
no surface a human would voluntarily inhabit; hardware acceptance lived in deferred checklists
(the field log's "PENDING — stakeholder field test" rows). Sprint 062 built a cockpit — four pose
estimates visualized on a playfield, keyboard driving. The immediate result: sprint 063 grew from
3 planned tickets to 11 mid-sprint as live operation poured out defects, and the Jul 1–2 issue
stream (config no-op, trace correctness, three-world poses) is dominated by things *seen in the
GUI*. This is the web-app mechanism transplanted: the moment the system became legible and the
human became its user, incidental testing and insight switched on. The stakeholder's web-vs-robot
productivity asymmetry is reproduced *within this single project*, before and after the cockpit
existed.

## Verdict

**Supported, with one refinement and one boundary.**

The refinement: it is not that incubation never happened — the gaps happened, and they produced
the project's best thinking. The failure mode is a **rate mismatch**: bursts created unread,
unfelt surface faster than gaps could digest it, so human insight always applied to the system
as of the *previous* gap. By the time the Jun 24–27 gap produced the mecanum supersede decision,
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

**R13 — Build the cockpit first.** The TestGUI should have been sprint ~5, not sprint 62. For
any AI project whose target isn't self-legible, the first deliverable is the surface that makes
the human *want* to operate the system daily — dashboard, live viewer, driving console, whatever
turns the human into a user. Web apps get this for free; everything else must buy it early. Every
week without it, the H7c engagement channel is dark.

**R14 — Size bursts to human absorption, not AI throughput.** The main report capped unvalidated
depth (R2); extend it to *unread* depth: a burst should not exceed what its human will actually
absorb before the next burst — practically, one day's diff presented with a reading guide, not
seven sprints overnight. If the human can't absorb it this week, the correct response is a
smaller burst, not a bigger summary.

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
