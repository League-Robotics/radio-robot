---
status: pending
priority: high
---

# `rogo repl` raises on every motion verb, its `stop` is mislabeled a panic stop, and the MCP calibration push silently no-ops

## Description

2026-08-02 post-130 review, **F4 (CRITICAL)**. Three defects in the tools our own
rules tell people to use. Verified in the tree today.

## 1. The repl's motion verbs are dead

`.claude/rules/hardware-bench-testing.md` directs users to `rogo repl`.
`src/host/robot_radio/io/repl.py:199,225,246` call `proto.twist(...)`, deleted at
the 116-001 MOVE cutover — **AttributeError on the first motion command**.
`repl.py:339` builds an `envelope_pb2.Twist` that does not exist (field 19 is
`reserved`).

**Fix:** motion verbs move to `move_twist`/`move_wheels` with ack checks, the
shape every current bench script already uses.

## 2. The repl's `stop` is the PLANNED stop, and is documented as a panic stop

`repl.py:363`:

```
  stop                        panic-stop the drivetrain
```

`stop()` is the **planned** stop — it queues behind the in-flight Move.
Measured 2026-07-29: sent 0.5 s into a 400 mm leg, the robot drove the entire
39.8 cm leg, active flag held 5.9 s. `estop()` does it in 2.9 cm / 0.10 s.

**The repl has no estop verb at all.** A user watching a runaway types `stop`
and gets the queued one. This is the exact defect class
`.claude/rules/playfield-testing.md` exists to prevent, sitting in the
interactive surface a human reaches for under stress.

(Credit where due: the repl's Ctrl-C cleanup *does* go through the shared
`halt_now()`. Only the typed verb surface is wrong.)

**Fix:** relabel `stop` honestly and add an `estop` verb. This half is small and
should not wait for the rest.

## 3. Every MCP session runs on build-time calibration while reporting success

`io/robot_mcp.py:97` → `calibration/push.py:386-395` writes v2-era text
`SET k=v` lines at connect time. Protocol v5 has **no text command plane** —
nothing is applied — and the call returns `{"status": "ok"}`.

This is the same defect `rogo sync cal` was retired for in 124-014, alive one
file over.

**Fix:** route the push through `binary_bridge.translate_command()`, the path
the TestGUI already uses and which is live — or delete the push until it can be
done, because a silent no-op that reports success is worse than no push at all.

## Why this is A

It is not a big change, but it sits under every bench session: the interactive
tool raises, the safety verb lies, and the calibration you think you pushed was
never applied. Fixing it is a precondition for trusting any measurement taken
through these paths.

## Verification

- `rogo repl` drives on the stand: connect banner, a `move_twist` leg with
  climbing encoders, ack-ring completion observed.
- `estop` exists as a repl verb and halts within one cycle; `stop`'s help text
  says "planned stop — queues behind the active move."
- The MCP connect-time push either applies (read it back — see
  [[A-no-firmware-to-host-config-readback]]) or is gone; no path returns
  `{"status": "ok"}` for work it did not do.
- `grep -rn "proto.twist\|envelope_pb2.Twist" src/host` returns nothing.

## Related

- `docs/code_review/2026-08-02-post-130-wheels-solid-review.md` F4.
- [[A-rebuild-nezha-facade-on-the-v5-binary-surface]] — the same rot, one layer
  down; F11 there, F4 here. Sequence them together if one sprint takes both.
- `.claude/rules/playfield-testing.md` — the halting rule this violates.
