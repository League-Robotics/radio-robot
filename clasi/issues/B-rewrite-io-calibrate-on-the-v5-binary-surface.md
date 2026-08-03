---
status: pending
---

# Rewrite `rogo calibrate distance`/`turns` on the v5 binary surface

**Source:** code review 2026-07-30, `03-host-core.md` CRITICAL §5;
`04-host-planning.md` finding 1's added scope (`calibration/linear.py`/
`angular.py` send the same retired ASCII grammar over raw serial).
**Priority:** P0 for honesty (both commands crash or silently no-op today),
P1 for the rewrite itself (calibration is stakeholder-valued capability).
**Goal served:** calibration is how we make the plant model truthful — a
calibration surface that pretends to work but sends retired verbs poisons
every downstream accuracy investigation.

## What is wrong

- `io/calibrate.py:400-420,744,851` calls `proto.zero_encoders()`,
  `proto.zero_otos()`, `proto.distance()`, `proto.read_encoders()`,
  `proto.read_otos()`, `proto.set_stream_otos()` — none exist on
  `NezhaProtocol`. `rogo calibrate distance` dies with `AttributeError` on
  its first trial.
- Its raw-text pushes (`OL±n`, `OA±n`, `TN`, `K+TW/ML/MR`) are doubly dead:
  the v5 firmware has no text-plane command parser, so even without the
  `AttributeError`s these reach no handler.
- `calibration/linear.py`/`angular.py` send the same retired ASCII verbs
  (`T`, `STOP`) over a deliberately separate raw-pyserial connection — they
  fail one layer further down, silently at the wire, which is worse.

## What to do

Rewrite each primitive against the binary surface — the mapping is
mechanical once stated:

| Retired call | v5 replacement |
|---|---|
| `proto.distance(mm, speed)` | `proto.move_wheels(speed, speed, stop_distance=mm, timeout=...)` + `wait_for_ack` |
| `proto.read_encoders()` / `read_otos()` | read `enc_left`/`enc_right` / `otos` off `read_pending_binary_tlm_frames()` |
| `OL±n` / `OA±n` text pushes | `proto.otos_config(linear_scale=...)` / `otos_config(angular_scale=...)` |
| `K+TW` / `K+ML` / `K+MR` | `proto.config(**{...})` |
| `proto.set_stream_otos(...)` | telemetry always carries `otos` when flag bit 0 is set — just read frames |
| `proto.zero_encoders()` | **does not come back.** Encoders are never reset (bus-budget rule); take a software offset: record the baseline reading at trial start and subtract. |
| `proto.zero_otos()` | decide: v5 `otos_config` reset arm if one exists, else software-offset the same way |

Example of the core trial primitive after the rewrite:

```python
def drive_measured_leg(proto, distance, speed, *, timeout=10000):  # [mm] [mm/s] [ms]
    """One calibration leg: bounded Move, encoder/OTOS deltas via software
    offsets (encoders are NEVER reset -- project convention)."""
    base = latest_frame(proto)                    # pump until enc+otos fresh
    enc0 = (base.enc_left, base.enc_right)
    corr_id = proto.move_wheels(speed, speed,
                                stop_distance=abs(distance), timeout=timeout)
    require_ok(proto.wait_for_ack(corr_id))
    done = wait_for_completion(proto, corr_id)    # completion ack on the ring
    end = latest_frame(proto)
    return (end.enc_left - enc0[0], end.enc_right - enc0[1], base, end)
```

Cleanup `finally` blocks switch to the shared `halt_now()` helper (estop
sweep issue) — no swallowed `proto.stop()`.

For `calibration/linear.py`/`angular.py`: fold onto the same rewritten
primitives, or delete if `io/calibrate.py`'s rewritten commands cover the
workflow — carrying two interactive calibration stacks is how this rotted
unnoticed. Stakeholder pick.

## Acceptance

- `rogo calibrate distance` and `rogo calibrate turns` complete a full trial
  sequence on the bench robot (stand), producing scale numbers, with no
  `AttributeError` and no text-plane sends.
- `grep -rn "zero_encoders\|zero_otos\|set_stream_otos\|read_encoders\|read_otos" src/host` returns nothing.
- `grep -rn "write_line(\"STOP\"\|\"T \"" src/host/robot_radio/calibration/`
  returns nothing (rewritten or deleted).
- Ctrl-C mid-trial halts the wheels immediately (estop path).
