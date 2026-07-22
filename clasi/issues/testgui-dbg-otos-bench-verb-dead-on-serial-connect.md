---
status: pending
---

# TestGUI's "DBG OTOS BENCH 1" on-Serial-connect push is a dead legacy verb (no binary arm)

## Context

Found 2026-07-22 while tracing the GUI robot-select calibration push
(`_push_robot_calibration()`) for a separate stakeholder bench-fix
session (SET routing through `binary_bridge.translate_command()` +
`NezhaProtocol.set_config_binary()`, both fixed that session).

`__main__.py`'s `_on_connect()` (around line 2885) sends `DBG OTOS BENCH
1` immediately after every Serial-transport connect:

```python
if name == "Serial":
    try:
        reply = transport.command("DBG OTOS BENCH 1", read_timeout=500)
        _append_log(
            "[BENCH] bench OTOS enabled (Serial = bench mode) → "
            f"{(reply or '').strip() or '(no reply)'}"
        )
```

Per its own comment: "Serial = BENCH MODE: the robot is on the stand,
where the real OTOS optically tracks nothing yet often reads
status-clean — the EKF then fuses 'stationary at the last anchor' and
pins the fused pose while the encoders move... Swap in the bench OTOS
(an errored copy of measured wheel travel) so pose-dependent behaviour
works on the stand."

`DBG` is not intercepted anywhere in `binary_bridge.py`'s
`translate_command()` (not `SET`/`GET`/`OI`/`OL`/`OA`, not in
`_POSE_RESET_VERBS`/`_ALWAYS_UNSUPPORTED_VERBS`, doesn't start with
`DEV`) — it falls through to the `_LEGACY_TRANSLATION_AVAILABLE` guard,
which is permanently `False` (`legacy_render`/`legacy_verbs` deleted by
104-002), so every Serial connect logs `[BENCH] bench OTOS enabled
(Serial = bench mode) → ERR unavailable legacy verb translation removed
...` and the bench-OTOS swap never actually happens.

## Why not fixed this session

Unlike `SET` (this session's fix routes it to the live
`NezhaProtocol.set_config()`/`config()` binary surface) and `OI`/`OL`/
`OA` (already routed via `otos_config()`), `DBG OTOS BENCH` has **no
binary-wire arm at all** — there is no `ConfigDelta`/`Move`/`Stop` shape
this verb could translate onto; it would need a genuinely new firmware
feature (or the OTOS bench-substitution logic moved host-side), not a
routing fix. Out of scope for a bench-fix session focused on
calibration-push/completion-ack/rest-creep.

## Impact

Low-to-moderate: the EKF pose-pinning-while-encoders-move symptom this
verb was meant to work around (2026-07-03 bench-OTOS diagnosis) is
plausibly still present on the stand for any Serial-connected session
that depends on fused pose. `TLMFrame.pose` (used by this same session's
`move_accuracy_bench.py` for turn-accuracy measurement) is
encoder-odometry, NOT the fused EKF pose, so this specific gap did not
affect that script's own numbers — flagging as a distinct, narrower
concern than the general "fused pose might be wrong on the stand" one.

## Proposed fix (future session)

Either: (a) give `RobotLoop`/`App::HeadingSource` a live `ConfigDelta`
or dedicated command arm for "force bench-OTOS substitution," matching
how `109-004` gave `OI`/`OL`/`OA` a direct-patch-send home; or (b) move
the "errored copy of measured wheel travel" substitution entirely
host-side (synthesize it from encoder telemetry already on the wire,
never touching firmware) if the EKF's own pose-pinning behavior can be
worked around purely by what the host feeds downstream consumers.
Stakeholder's call on which.

## Priority

Normal — known, bounded gap; does not block calibration/completion-ack/
distance-accuracy bench work (none of which depend on fused EKF pose).
