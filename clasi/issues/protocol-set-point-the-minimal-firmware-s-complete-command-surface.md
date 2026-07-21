---
status: pending
sprint: '116'
---

# Protocol set point: the minimal firmware's complete command surface

## Description

The protocol document the minimal firmware speaks **when the gut sprint closes** — every command the host can send and every response it gets back. This is the set point the implementation converges to, and it revises the command model: the previously planned bare TWIST/WHEELS commands are replaced by **one MOVE command** — a velocity (twist or wheel-speed variant) + a **stop condition** (time | distance | angle, ticked on-chip by a stop-condition object) + a **replace** flag against a small motion queue. Every motion is bounded: to keep going, the host sends the next MOVE before the current one expires.

Stakeholder decisions (Eric, 2026-07-21), binding:

- **MOVE is the only motion command** (+ STOP as the immediate halt). Bare TWIST leaves the wire.
- **Queue = 1 active + 4 pending**; a fifth pending MOVE gets ERR_FULL.
- **Every MOVE carries a required `timeout` backstop** — every motion is self-bounding, so **the deadman machinery is deleted**.
- **Distance/angle stop conditions measure from encoder odometry**, baselined at command activation.

## Cause

The gut needs a written contract for its end state, and the interim command surface (TWIST with a deadman duration, plus a planned separate Wheels command) had two overlapping bounding mechanisms (duration vs deadman lease) and two motion verbs where one suffices. Unifying on bounded MOVEs makes the safety property structural — host silence always ends in stopped motors because commands expire — instead of relying on a separate watchdog module.

## Proposed fix

### Transport & framing (unchanged)

- Line-based over serial CDC (bench) or the radio relay. Binary plane: `*B<base64(protobuf)>` armored lines, both directions. Text plane: exactly two verbs.
- `HELLO` → `DEVICE:...` identity banner.
- `PING` → `OK pong t=<ms>` (robot clock; the `t=` addition activates the existing host `clock_sync.py`).

### Commands (`CommandEnvelope{corr_id, oneof cmd}`)

| Arm | Field | Payload | Effect |
|---|---|---|---|
| `config` | 6 | `ConfigDelta{drivetrain \| motor \| otos}` | Live-apply + persist a tuning patch. (`planner` arm deleted → reserve 3; `watchdog` arm deleted with the deadman → reserve 4.) |
| `stop` | 13 | `Stop{}` (zero fields — cannot be malformed) | Immediate: flush the queue, zero both motor velocity targets, ack. |
| `move` | **21** (fresh; old arc-Move's 20 stays reserved — it shipped) | `Move` below | Enqueue or replace a bounded motion. |

```proto
message MoveTwist {
  float v_x   = 1;  // [mm/s] body forward
  float v_y   = 2;  // [mm/s] accepted-and-ignored on differential (wire-forward)
  float omega = 3;  // [rad/s]
}
message MoveWheels {
  float v_left  = 1;  // [mm/s]
  float v_right = 2;  // [mm/s]
}
message Move {
  oneof velocity {              // the two variations
    MoveTwist  twist  = 1;
    MoveWheels wheels = 2;
  }
  oneof stop {                  // every motion has a stop condition
    float time     = 3;  // [ms] elapsed since activation
    float distance = 4;  // [mm] |path arc length| since activation (encoder odometry)
    float angle    = 5;  // [rad] |heading change| since activation (encoder odometry)
  }
  float  timeout = 6;  // [ms] REQUIRED safety backstop; <=0 -> ERR_BADARG.
                        // Fires when a distance/angle condition can't be reached
                        // (stalled wheels); stops motors + sets the timeout fault flag.
  bool   replace = 7;  // true: flush pending + preempt active, this MOVE starts now.
                        // false: enqueue behind the active command (ERR_FULL if 4 pending).
  uint32 id      = 8;  // echoed in this command's COMPLETION ack (enqueue ack echoes corr_id)
}
```

### Execution model (firmware contract)

- Queue: 1 active + up to 4 pending. `replace=true` flushes pending and preempts the active command immediately; `replace=false` appends (5th pending → ERR_FULL, command dropped, nothing else disturbed).
- On activation, a **stop-condition object** is created, capturing its baseline (activation time; odometry path length; odometry heading). The loop **ticks it every cycle**; when it reports stop — or `timeout` elapses — the active command ends: the next queued MOVE activates the same cycle (seamless chain), or, with an empty queue, both motor velocity targets go to zero.
- **No deadman**: every MOVE is bounded by its stop condition or timeout, so host silence always ends in motors stopped when the last command expires. STOP remains the immediate manual halt. (The `app/deadman.*` module, the TWIST `duration` semantics, and the ConfigDelta `watchdog` arm are all deleted.)
- Unconfigured device (fail-closed config gate) refuses MOVE with ERR_NOT_CONFIGURED.

### Responses

- **Every command** is acked through the telemetry frame's single ack slot (per the tightened-frame spec): `ack_corr` = the envelope's `corr_id`, `ack_err` = ErrCode (0 = OK), `flags.ack_fresh` set on that frame.
- **MOVE completion**: a second ack on the cycle the command ends — `ack_corr` = `Move.id`, `ack_err` = 0 for a met stop condition; a **timeout** ending additionally sets the `flags` move-timeout fault bit (bit 15 in the tightened frame).
- Error taxonomy (existing `ErrCode` in envelope.proto, unchanged): ERR_UNKNOWN (no such arm), ERR_BADARG (malformed — e.g. missing/nonpositive timeout, no velocity variant), ERR_RANGE (bound violated), ERR_FULL (queue), ERR_DECODE (bad wire bytes), ERR_OVERSIZE, ERR_NOT_CONFIGURED.
- **Telemetry** (the return channel, every loop iteration, ~20 ms): the tightened frame per `telemetry-frame-tightening-amendment-to-gut-s1.md` — `now/seq/mode/flags/ack_corr/ack_err/enc_left/enc_right (EncoderReading)/otos (OtosReading)/pose/twist/line/color`. The host's log of this stream is the dataset.
- `ReplyEnvelope` stays narrowed to `ok/err/tlm`; `Ack{q,rem,t}`/`Error` remain declared-only schema.

### What is deliberately NOT in this protocol

Arc/segment moves, trajectory profiles, jerk limiting, heading cascade, pose-fix injection, GET/STREAM/ECHO, plan dumps, ring dumps — all reserved wire numbers, all recoverable from the `pre-gut-motion-stack` tag. The protocol is: **bounded velocity commands in, timestamped measurements out.**

### Firmware design notes (for the implementing sprint)

- `Motion::StopCondition` (new, tiny — `src/firm/motion/stop_condition.{h,cpp}` or folded into the queue module): captures kind + threshold + baselines at activation; `bool tick(now, odom)` returns stop. ~50 lines. Distance uses accumulated |path| from `App::Odometry` (add a simple `pathLength()` accessor — the deleted executor-era accessors were different plumbing); angle uses |theta − theta at activation|, wrapped.
- `App::MoveQueue` (new, small): fixed array of 5 decoded Moves + active slot; activation/completion acks; replace/flush semantics; owned by `RobotLoop`, ticked in the dispatch block where the deadman check used to live.
- `Drive` keeps `setTwist`/`setWheels` staging — the active MOVE's velocity variant stages through it at activation; `stop()` zeroes both.
- Host: `NezhaProtocol.move_twist(v_x, v_y, omega, stop=..., timeout=..., replace=...)`, `move_wheels(...)`, `stop()`; `wait_for_ack` unchanged (single slot).

### Integration with the gut issues

- The gut issue's S2 stage becomes the **MOVE protocol cutover** (Move arm 21 + MoveQueue + StopCondition; delete `twist` arm → reserve 19, delete `app/deadman.*`, delete ConfigDelta `watchdog` arm → reserve 4). S1 keeps the existing TWIST+deadman so the robot stays drivable at the S1 gate; S2 swaps the surface.
- The telemetry amendment gains `flags` bit 15 (fault: move-timeout backstop fired).

## Verification

The implementing sprint's protocol gate, robot on stand:

1. Round-trip every command: HELLO, PING (`t=` present), CONFIG patch (persists across power-cycle), MOVE × both variants × all three stop conditions, STOP — each acked with the right corr/err.
2. Stop-condition behavior: time MOVE ends on schedule; distance MOVE ends within tolerance of the commanded distance (encoders, on the stand); angle MOVE ends within tolerance of the commanded heading change; a distance MOVE that cannot progress ends at `timeout` with the fault flag set.
3. Chaining: MOVE B (replace=false) sent while A runs → seamless handoff at A's expiry; replace=true preempts mid-motion; queue overflow → ERR_FULL; empty-queue expiry → motors stop with zero host traffic (the no-deadman contract).
4. Soak per the gut protocol (≥10 min, alternating MOVEs at 5–10 Hz: no reboot/lockup, seq monotonic, drop rate ≈ baseline).

## Related

- `gut-to-minimal-firmware-motion-stack-excision-move-protocol-minimal-telemetry.md` — the parent work; this issue defines its S2 stage's protocol contract and end-state doc.
- `telemetry-frame-tightening-amendment-to-gut-s1.md` — the return-channel frame this protocol's responses ride; gains flags bit 15 for the move-timeout fault.
- `predict-to-now-odometry-estimator-ring-capture-dump-validation-trajectory-controller.md` — future build-out; the stop-condition/queue model here is the substrate its remaining-distance controller will drive through.
