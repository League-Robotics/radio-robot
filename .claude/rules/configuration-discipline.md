# Configuration Discipline

Stakeholder rule, 2026-08-03:

> We will bake, but we have to be able to configure everything we bake. If you
> set a configuration, there's one file for it, and that file is the one used
> for baking. To send configuration you edit that file, send that file, and the
> next rebuild bakes the same file.

Two invariants, running in opposite directions:

1. **Every value the robot uses comes from the file.** No value authored in a
   C++ constant, a host-side literal, or a call-site argument.
2. **Every value in the file reaches the robot.** Each field has a runtime path
   *and* a bake path, both reading the same file, so a pushed config and a
   rebuilt image cannot disagree.

A value in the file that nothing consumes breaks invariant 2 as badly as a
missing one. **Delete it, don't wire it.**

## The rule binds PRODUCTION BOOT, not development

Refined by the stakeholder the same day. This is the point, not a loophole:

> It doesn't mean you have to do it for development. You should be able to
> configure individual items without the file — we're going to do a sweep, so
> we should allow that. In general, if you're booting up the robot in
> production, then it better come from a file.

| context | rule |
|---|---|
| production boot | every value comes from the file, no exceptions |
| development / bench tuning | ad-hoc single-value pushes are expected and allowed |

**Read-back is what makes the development relaxation safe.** An ad-hoc push
creates exactly the invisible divergence-from-file the rule exists to prevent;
it is acceptable only because you can interrogate the robot and see it.

## What a bench script or tuning sweep owes

Sweeps push literals directly — no scratch-file round trip. In exchange:

- **read back what you pushed.** An ack is not evidence: config that acks OK and
  lands nowhere is a live failure mode in this codebase, not a hypothetical.
- **record the pushed values with the results**, so a captured dataset is
  self-describing instead of depending on session memory.
- **promote the winner into the robot JSON** before anything is baked, shipped,
  or used as a baseline for a later measurement.

## Why this exists

Configuration was defined twice — a JSON→C++ generator and a `.proto`→wire
generator — with a hand-written lint to notice drift. Adding one config field
touched 16 places across 5 languages. Twice in one sprint a routine measurement
produced a confident wrong answer because the host computed against a JSON value
the firmware had stopped using (a ~1.6x error), and it led to a recommendation
to inspect a wheel mechanically that had nothing wrong with it.

Design work implementing this rule: `clasi/issues/the-configuration-object.md`.
