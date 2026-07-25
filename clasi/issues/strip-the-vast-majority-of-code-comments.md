---
status: pending
---

# Strip the vast majority of code comments — target ~90% removal

## The idea (stakeholder, 2026-07-25)

> We need to get rid of the vast number of comments. There's a huge amount of
> comments all over the place, and most of them are useless. Keep only little
> comments that separate sections or direct readers to other documents.
> Seriously, ~90% of the comments in the code need to be removed.

The codebase has accumulated dense, narrative comment blocks — especially in
the firmware hot path (e.g. `src/firm/app/robot_loop.cpp` has long paragraph
comments explaining sprint history, cross-cycle staging rationale, and
step-by-step data flow inline). Most of this is noise that makes the code
harder to read, not easier.

## What to KEEP

Comments earn their place only when they do one of two things:

1. **Separate sections** — short section-marker comments that break a long
   function or file into readable chunks.
2. **Direct the reader to a document** — a one-line pointer to the authoritative
   design/knowledge doc (`docs/design/`, `src/motion/DESIGN.md`, `docs/protocol-v4.md`,
   `.clasi/knowledge/`, `docs/reference/…`) where the real explanation lives.

Everything else — restated code, sprint-number archaeology, multi-paragraph
rationale, "why we did X three sprints ago" narration — comes out. If a comment
explains *why* and that why is important, it belongs in the relevant design doc,
with a one-line pointer left behind, not inline.

## What to WATCH

- **The unit-tag convention is NOT a comment to strip.** `// [mm/s]`, `# [ms]`
  bracketed unit tags are load-bearing (`.claude/rules/coding-standards.md`) —
  they carry the units that identifiers deliberately don't. Keep every one.
- **Don't delete a rationale into oblivion.** Where a comment records genuinely
  hard-won, non-obvious knowledge (bus timing, IRQ guards, wedge/latch quirks,
  actuation latency), move it to `.clasi/knowledge/` or the subsystem design doc
  and leave a pointer — don't just erase it.
- Keep license/attribution headers and any `TODO`/`FIXME` that still tracks real
  work.

## Scope

Project source: `src/firm`, `src/host`, `src/motion`, tests, tooling. This is a
sweep-style effort — likely a mechanical sprint with batched tickets by
directory/file (see the "mechanical sprints: batch dispatch" working pattern).
Design docs and `.md` files are out of scope; this is about *code* comments.

## Suggested measure of done

A reviewer skimming any touched file sees mostly code, with occasional
section markers and doc pointers — and can no longer find paragraph-length
inline narration. Rough target: ~90% of the pre-sweep comment volume gone.
