---
status: pending
sprint: '064'
---

# Bare `DBG IRQGUARD` query disables the IRQ guard (ArgSchema default-fill regression, 051-008)

## Description

Verified live on tovez (fw 0.20260701.14, 2026-07-02, robot USB direct):

```
DBG IRQGUARD 1   -> OK dbg irqguard=1
DBG IRQGUARD     -> OK dbg irqguard=0   <- the QUERY disabled the guard
DBG IRQGUARD     -> OK dbg irqguard=0
DBG IRQGUARD 1   -> OK dbg irqguard=1
```

**Mechanism** (`source/commands/DebugCommands.cpp`):

```c
static const ArgDef dbgIrqguardDefs[1] = {
    { "enable", ArgKind::INT, false, 0, 0 },   // default value 0
};
static const ArgSchema dbgIrqguardSchema = { dbgIrqguardDefs, 1, /*minTokens=*/0, false, nullptr };
...
if (args.count >= 1) ctx.busDiag->setIrqGuard(args.args[0].ival != 0);
```

The ArgSchema machinery fills the missing optional token with its default
(`0`), so `args.count >= 1` is always true and the handler cannot distinguish
"no arg supplied (query)" from "explicit 0". A bare query is therefore
equivalent to `DBG IRQGUARD 0`.

The pre-051-008 hand-rolled parser (`ntok >= 3` token check, from d6d798d) was
query-safe; the regression arrived with the ArgSchema migration (commit
`f4782cf`, sprint 051 — inside the big-rebase window).

## Why it matters

The IRQ guard is the primary defense against the nRF52 TWIM encoder wedge
(d6d798d: guard OFF wedged in ~30 s under load; guard ON ran clean). Any
diagnostic query — a human poking at state, a bench preflight, a future
health-check — silently disarms it until reboot or an explicit
`DBG IRQGUARD 1`. This contaminated the 2026-07-02 stand-repro baselines
(the harness preflight queried the guard and thereby disabled it; see
docs/knowledge/2026-07-01-encoder-wedge-boundary-latch-flavor.md).

## Fix

Make the handler distinguish "no token supplied" — e.g. a per-arg
`was_defaulted` / `has` flag in ArgList, or give the report path its own
no-default schema — or drop the default entirely and treat `args.count`
correctly. Add a regression test: query must not change state.

## Follow-up audit

Sweep the other 051-008-migrated handlers whose ArgSchema uses optional
args with defaults (`ndefs >= 1, minTokens=0`) for the same
query-mutates-state pattern.
