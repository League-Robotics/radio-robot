# Protocol v4 Wire Specification

**Current wire truth.** This document describes the Nezha firmware
command/telemetry protocol **as shipped by sprint 116** ("MOVE protocol
cutover", tickets 001-008) — the converged, bounded-`MOVE` command
surface the minimal single-loop firmware speaks once that sprint closed.
It supersedes [`docs/protocol-v2.md`](protocol-v2.md) and
[`docs/protocol-v3.md`](protocol-v3.md) (both kept, not deleted, as the
historical record of what shipped when — see each file's own banner).

Source of the contract this document transcribes:
[`protocol-set-point-the-minimal-firmware-s-complete-command-surface.md`](../clasi/sprints/116-move-protocol-cutover/issues/protocol-set-point-the-minimal-firmware-s-complete-command-surface.md)
(the ratified stakeholder decision, Eric, 2026-07-21). Every claim below
was cross-checked against the actual shipped source
(`src/protos/envelope.proto`, `src/protos/telemetry.proto`,
`src/firm/app/robot_loop.cpp`, `src/firm/app/move_queue.{h,cpp}`,
`src/firm/motion/stop_condition.{h,cpp}`, `src/firm/app/comms.cpp`,
`src/host/robot_radio/robot/protocol.py`) and its own test suite, not
restated from the set-point issue's proposal alone. Wherever
implementation had to pin down something the issue's own Architecture
"Open Questions" left unresolved, or otherwise diverged from the
proposal in a load-bearing way, that is called out explicitly as
**AS-BUILT** below — see §5.3, §5.4, §7.2, §7.4.

---

## 1. Overview

The robot speaks the same three-plane shape protocol v2/v3 established
— a binary command plane, a minimal text safety rump, and a telemetry
return channel — with the command surface itself converged onto **one
bounded motion command**:

| Plane | Shape | Carries |
|---|---|---|
| Binary command plane | `*B<base64(CommandEnvelope)>` (host→robot) | `move` / `config` / `stop` — exactly three `cmd` oneof arms (§3) |
| Text safety rump | Two hand-typeable verbs | `HELLO` (identity banner), `PING` (liveness) — see §2.4 |
| Telemetry return channel | `*B<base64(ReplyEnvelope{tlm: Telemetry})>` (robot→host), every cycle | The frame v2 shape (§8) — the SOLE per-command outcome path (§7) |

The defining change from protocol v3: the interim `Twist` arm (a bare
`v_x`/`omega` + a `duration` that re-armed a separate `App::Deadman`
watchdog) is gone. Every motion is now a **`Move`** — a velocity variant
(twist or per-wheel speeds) plus a **stop condition** (time, distance,
or angle, ticked on-chip against odometry) plus a **required `timeout`
backstop** — queued against a small on-chip queue (1 active + 4
pending). Every `Move` is self-bounding by construction, which is what
lets this sprint delete `App::Deadman` outright: host silence always
ends in stopped motors once the active `Move`'s stop condition or
timeout fires, with no second, independently-timed watchdog needed.

---

## 2. Transport & framing

Unchanged in every particular from v2/v3 except where noted (§2.4).

### 2.1 Line shapes

```
*B<base64(CommandEnvelope bytes)>\n     -- host -> robot
*B<base64(ReplyEnvelope bytes)>\n       -- robot -> host
HELLO\n / PING\n                        -- text plane, both directions bare
```

Line-based over serial CDC (bench, 115200 baud) or the radio relay
(`RadioTransport`) — `App::Comms::pump()` (`src/firm/app/comms.cpp`)
reads at most one line per call from serial first, falling back to
radio only if serial had nothing (`Comms::pump()`, `comms.cpp:49-53`).

### 2.2 Base64 armor

- **Standard alphabet (`+/`)**, RFC 4648 `=` padding — NOT URL-safe
  (`-_`). Both sides must agree; there is no negotiation and no version
  byte (`src/firm/messages/wire_runtime.{h,cpp}` vs. Python's
  `base64.b64encode`/`b64decode`, whose default alphabet is this same
  one).
- **Dearmor** (`Comms::decodeArmoredLine()`, `comms.cpp:78-115`):
  `line[1] != 'B'` is rejected (malformed-count increment, no reply —
  see §7.4). Trailing `\r`/`\n`/space/tab is trimmed off the base64
  payload before decode.
- **Decode**: `msg::wire::decode()` (generated,
  `src/firm/messages/wire.{h,cpp}`) walks `CommandEnvelope`'s field
  table. Unknown field numbers are skipped, not rejected (forward
  compatible with a newer schema). Malformed/truncated bytes fail the
  decode — see §7.4 for what happens next (nothing is sent back; the
  failure is counted, not acked).

### 2.3 Size budget (measured, `gen_messages.py`-computed)

Recomputed by the generator on every build; a schema change that pushes
either total over the 186-byte envelope budget fails a build-time
`static_assert`, never a silently truncated wire line
(`src/firm/messages/wire.h:43-63`, `src/firm/messages/DESIGN.md` §3):

| Envelope | Worst-case arm | Total (worst arm + non-oneof bytes) |
|---|---|---|
| `CommandEnvelope` | `config`=44B, `stop`=2B, **`move`=38B** (worst=`config`) | **50 B** |
| `ReplyEnvelope` | `ok`=19B, `err`=10B, **`tlm`=147B** (worst=`tlm`) | **153 B** |
| `TelemetrySecondary` | (own armored line, not a `ReplyEnvelope` arm) | **52 B** |

Both `CommandEnvelope` and `ReplyEnvelope` sit comfortably under the
186-byte cap (136 B and 33 B of margin respectively). Note `move` (38 B,
two nested oneofs + `id`) is a structurally bigger message than the
`Twist` arm it replaced, yet `CommandEnvelope`'s own worst-case total is
unchanged at 50 B — `config` (dominated by `DrivetrainConfigPatch`)
stays the larger arm either way
(`src/firm/messages/DESIGN.md` §3, "Envelope size is bounded...").

- **Armored buffer**: 256 bytes (`kArmoredBufSize`, `comms.h:95`) — "*B"
  (2) + base64(153) (204, rounded up) + NUL, with headroom.
- **Nesting depth cap**: 8 levels (`WireRuntime::kMaxNestingDepth`) —
  this schema's deepest actual chain
  (`CommandEnvelope → *ConfigPatch`) is far shallower.

### 2.4 Text safety rump — `HELLO` / `PING`

`Comms::pumpTransport()` (`comms.cpp:56-85`) checks these two literal
strings **before** the `*B` armor check:

| Verb | Reply | Notes |
|---|---|---|
| `HELLO` | `DEVICE:NEZHA2:robot:<name>:<serial>` | `formatBanner()`, `main.cpp:36-41` — byte-frozen, host banner parsers depend on it. `<name>` = `microbit_friendly_name()`, `<serial>` = `microbit_serial_number()`. |
| `PING` | `OK pong t=<ms>` | Liveness probe + clock-sync activation (117, SUC-056). |

**117 (predict-to-now estimator v1, SUC-056) — landed; closes the prior
AS-BUILT divergence this section used to document here.** `PING`'s reply
now carries `t=<ms>` — the firmware's own current clock time —
appended via `std::snprintf(pong, sizeof(pong), "OK pong t=%lu",
static_cast<unsigned long>(now))` (`Comms::pumpTransport()`,
`comms.cpp`). `now` is `RobotLoop::cycle()`'s already-computed
`cycleStart`, threaded through the new `Comms::pump(Cmd&, uint32_t now)`
parameter — `Comms` itself still owns no `Devices::Clock&` collaborator
of its own. This activates the host's existing NTP-style `clock_sync.py`
(min-RTT offset + skew fit, `ClockSync.ping_burst()`), whose
`_parse_pong_t()` already parsed this exact shape and tolerated its
prior absence (returning `None`). Anything else sent as plain text (not
`*B`-prefixed, not `HELLO`/`PING`) increments `malformedCount_` (§7.4)
with no reply.

Any other pre-097/pre-102 text verb (`ID`, `VER`, `HELP`, `STOP`,
`SET`/`GET`, `S`/`D`/`T`/`R`/`TURN`/`RT`/`G`/`MOVE`/`MOVER`, etc.) is
**not** part of this firmware's text plane — the single-loop rebuild
(sprints 102-107) removed the larger text rump protocol-v3.md
documents; only the two verbs above remain.

---

## 3. `CommandEnvelope` — command arms

`src/protos/envelope.proto`'s `CommandEnvelope.cmd` oneof, dispatched by
`RobotLoop::processMessage()`'s switch on `msg::CommandEnvelope::CmdKind`
(`robot_loop.cpp:389-407`):

| Arm | Field # | Payload | Handler |
|---|---|---|---|
| `config` | 6 | `ConfigDelta` (§6) | `RobotLoop::handleConfig()` |
| `stop` | 13 | `Stop{}` (zero fields — "cannot be malformed" by construction) | `RobotLoop::handleStop()` |
| `move` | **21** | `Move` (§4) | `RobotLoop::handleMove()` |

`corr_id` (field 1) is present on every `CommandEnvelope` and is echoed
back via the single ack slot (§7.1), never a per-command
`ReplyEnvelope`.

**Reserved, not reused** (`envelope.proto`'s own `reserved` list,
`CommandEnvelope`): 2, 3, 4, 5, 7-12, 14-18, 19, 20. Field 19 (`Twist`,
the 103-era bare-velocity-plus-deadman shape) and field 20 (a
sprint-109 arc-command `Move`, deleted by the S1 motion-stack excision)
are the two most relevant to this rewrite — both superseded by `move`
at the fresh number **21**, never reusing either retired number. Every
other reserved number is a pre-102 arm (`drive`/`segment`/`replace`/
`pose_fix`/`otos`/`ping`/`echo`/`get`/`stream`/`id`/`hello`/`ver`/
`help`/`plan_dump`) that shipped on real hardware before the single-loop
rebuild — kept reserved per this project's standing wire-stability
discipline, never reassigned to a new field.

`ErrCode` (used by every ack — §7.2) is declared in the same file; see
§7.3 for the taxonomy table.

---

## 4. The `Move` message

```proto
message MoveTwist {
  float v_x   = 1;  // [mm/s] body forward
  float v_y   = 2;  // [mm/s] accepted-and-ignored on this differential build (wire-forward
                    //        for a future holonomic base)
  float omega = 3;  // [rad/s]
}
message MoveWheels {
  float v_left  = 1;  // [mm/s]
  float v_right = 2;  // [mm/s]
}
message Move {
  oneof velocity {              // exactly one of the two velocity variants
    MoveTwist  twist  = 1;
    MoveWheels wheels = 2;
  }
  oneof stop {                  // exactly one stop condition
    float time     = 3;  // [ms] elapsed since activation
    float distance = 4;  // [mm] |path arc length| since activation (encoder odometry)
    float angle    = 5;  // [rad] |heading change| since activation (encoder odometry)
  }
  float  timeout = 6;  // [ms] REQUIRED safety backstop; <=0 -> ERR_BADARG.
  bool   replace = 7;  // true: flush pending + preempt active, this MOVE starts now.
                        // false: enqueue behind the active command (ERR_FULL if 4 pending).
  uint32 id      = 8;  // echoed in this command's COMPLETION ack (enqueue ack echoes corr_id)
}
```

### 4.1 Shape validation (`RobotLoop::handleMove()`, `robot_loop.cpp:198-218`)

Checked, in order, **before** the `Move` ever reaches the queue:

1. **Config-completeness gate**: if the composition root is not yet
   `configured_`, ack `ERR_NOT_CONFIGURED` immediately — no other field
   is inspected.
2. **Shape**: `velocity_kind == NONE`, or `stop_kind == NONE`, or
   `timeout <= 0.0f` → ack `ERR_BADARG`. Verified 1:1 against the
   shipped test sweep (`src/tests/sim/unit/app_robot_loop_harness.cpp`,
   "MOVE shape validation" scenario, `:987-1022`): missing velocity
   variant, missing stop variant, and non-positive `timeout` are each
   independently exercised and each acks `ERR_BADARG`.

A well-formed `Move` is handed to `App::MoveQueue::enqueue()` (§5),
whose own return code (`ERR_NONE` or `ERR_FULL`) becomes this
envelope's ack.

### 4.2 AS-BUILT: zero/negative stop-condition thresholds are accepted, not `ERR_BADARG`

The set-point issue's own Architecture "Open Question 1" left this
genuinely open: *"is a zero-magnitude `distance`/`angle` threshold valid
... or should it be rejected the same way a non-positive `timeout`
is?"*, recommending (not mandating) mirroring `timeout`'s `>0` rejection
rule.

**What actually shipped** (`Motion::StopCondition`'s constructor,
`src/firm/motion/stop_condition.cpp:26-33`, doc-comment "PINNED HERE" at
`stop_condition.h:79-93`): `handleMove()` does **not** reject a
non-positive `distance`/`angle`/`time` stop value at the wire level —
only `timeout <= 0` is rejected there (§4.1). A non-positive (or NaN)
`distance`/`angle`/`time` threshold instead reaches `StopCondition`,
which clamps it to `0` (`clampPositive()`,
`stop_condition.cpp:15`) and reports `StopConditionMet` on the *very
first* `tick()` call after activation — a "stop immediately" idiom,
applied **uniformly across all three kinds** (not a `Time`-only
carve-out). A non-positive `timeout` is clamped the identical way and
reports `TimedOut` on the first tick — unless the kind-specific
condition also fires that same tick, in which case the tie-break below
still applies. Verified by
`src/tests/sim/unit/motion_stop_condition_harness.cpp` scenarios 6/6b
(`:170-233`): zero/negative/NaN threshold and zero/negative timeout each
clamp to 0, and a threshold-AND-timeout-both-zero case still resolves
via the tie-break, never a rejection.

### 4.3 Tie-break

When both the kind-specific stop condition and `timeout` are met on the
same `tick()` call, `StopConditionMet` is reported, never `TimedOut` —
the kind-specific result always wins
(`stop_condition.cpp:51-55`, verified by harness scenario 5,
`:150-167`). Consequence: a well-formed `Move` (threshold reachable
before `timeout`) always ends via `StopConditionMet`; `TimedOut` is only
ever reported on a tick where the kind-specific condition did *not* also
fire.

---

## 5. Execution model

### 5.1 Queue: 1 active + 4 pending (`App::MoveQueue`, `src/firm/app/move_queue.{h,cpp}`)

`MoveQueue::enqueue(move, corrId)` (`move_queue.cpp:51-78`):

| `move.replace` | Queue state | Effect |
|---|---|---|
| `true` | any | Flushes every pending slot (no ack for any of them — §7.4), preempts the active `Move` if any, activates `move` in this SAME call. Never `ERR_FULL`. |
| `false` | empty (no active `Move`) | Activates `move` immediately — identical activation path to `replace=true`, nothing to flush/preempt. |
| `false` | a `Move` is active, <4 pending | Appends behind the active `Move`. |
| `false` | a `Move` is active, 4 already pending | Rejected `ERR_FULL` — **provably a complete no-op**: the `ERR_FULL` check runs before any state mutation, so the existing active `Move` and all 4 pending `Move`s are byte-for-byte unchanged (verified by `app_move_queue_harness.cpp`'s ERR_FULL scenario, `:543-587`, which re-reads every slot after the rejected 5th enqueue). |

Activation (`MoveQueue::activate()`, `move_queue.cpp:10-49`) stages the
velocity variant through `App::Drive` (`setTwist()`/`setWheels()` — §5.5)
and captures the `Motion::StopCondition` baseline (activation clock time,
`Odometry::pathLength()`, `Odometry::theta()`) at that exact moment.

### 5.2 Per-cycle tick (`MoveQueue::tick()`, `move_queue.cpp:80-106`)

Called unconditionally, every loop cycle (`robot_loop.cpp:507`,
`~50 Hz` / 20 ms — the same schedule position the deleted
`deadman_.expired()` check used to occupy). A no-op if no `Move` is
active. Otherwise reconstructs a fresh `Motion::StopCondition` from the
active slot's stored baseline+kind+threshold+timeout and calls its
`tick()`:

- **`Continue`**: no-op, the active `Move` keeps running.
- **`StopConditionMet` or `TimedOut`**: the active `Move` ends this
  cycle. If a `Move` is pending, the next one activates **the same
  call** (seamless hand-off — no intervening cycle with a zero/stopped
  commanded velocity, verified by harness scenario 5,
  `app_move_queue_harness.cpp:452-493`). If the queue is now empty,
  `Drive::stop()` is called (§5.5) — both wheel velocity targets go to
  zero.

### 5.3 AS-BUILT: no completion ack for a flushed-while-pending `Move`

The set-point issue's Architecture "Open Question 2" also left this
open: what happens to the completion ack for a `Move` that was enqueued
(and acked at enqueue time), then flushed by a later `replace=true`
before it ever activated? The issue recommended (not mandated) "no
completion ack for a flushed-while-pending `Move`."

**What shipped**: `enqueue(replace=true)` sets `pendingCount_ = 0`
directly (`move_queue.cpp:56`) with no per-slot ack of any kind, and
`MoveQueue::flush()` (used by `STOP`, §5.6) likewise has a `void`
return — structurally incapable of reporting a completion for anything
it drains. **Only an activated-then-ended `Move` (via `tick()`'s
`StopConditionMet`/`TimedOut` path, §5.2) ever produces a completion
ack.** Verified explicitly by `app_move_queue_harness.cpp` scenarios 6
(`:495-541`, "the flushed pending Move never activates ... B never
appears — it was flushed, not merely deprioritized") and 8
(`:589-641`, `flush()`'s own "NO completion ack ... structurally, no
completion is ever reported for a flush()").

### 5.4 No deadman — the structural safety property (SUC-053)

`App::Deadman` and `app/deadman.{h,cpp}` no longer exist anywhere in the
tree (both test harnesses deleted alongside it). The "host went silent,
stop the robot" property it used to provide is now an **emergent
consequence** of `MoveQueue::tick()` running unconditionally every
cycle and draining to `Drive::stop()` on an empty queue (§5.2) — not a
second, independently-timed watchdog running alongside the queue. A
`Move`'s own stop condition or its required `timeout` is the only bound
on how long it runs; once the last queued `Move` ends with nothing
pending, motors stay at zero indefinitely with **zero further host
traffic required**.

### 5.5 Velocity staging (`App::Drive`, `src/firm/app/drive.{h,cpp}`)

Two independent, last-wins staging paths — `setTwist(v_x, v_y, omega)`
(via `BodyKinematics::inverse()`) and `setWheels(v_left, v_right)`
(staged directly, never round-tripped through a twist). `MoveWheels` is
**not** translated into an equivalent twist even though one exists on
this differential base — a deliberate choice (sprint 116
architecture-update.md Decision 3) anticipating a future non-differential
base where a wheel-speed pair would not correspond to any single body
twist. `Drive::stop()` zeroes both staging paths' targets regardless of
which was last active.

### 5.6 `STOP` (`RobotLoop::handleStop()`, `robot_loop.cpp:378-382`)

`drive_.stop()` (immediate zero) **and** `moveQueue_.flush()` (drains
every pending slot + ends the active `Move`, no per-`Move` ack — §5.3),
then acks the `STOP` command itself (`ERR_NONE`) via the envelope's own
`corr_id`. `Stop{}` carries zero fields, so it cannot itself be
malformed.

### 5.7 Config-completeness gate

An unconfigured composition root (real firmware satisfies this
immediately at boot; only a not-yet-`markConfigured()` `SimHarness`
composition root can observe the gate) refuses `move` with
`ERR_NOT_CONFIGURED` (§4.1). `stop`/`config` are ungated — unchanged
asymmetry from before this sprint.

---

## 6. `ConfigDelta` arm

`ConfigDelta.patch` oneof (`src/protos/config.proto` +
`envelope.proto`), dispatched by `RobotLoop::handleConfig()`
(`robot_loop.cpp:234-287`):

| Patch | Field # | Runtime application | Persisted? |
|---|---|---|---|
| `drivetrain` | 1 | **Declared only** — acks `ERR_UNIMPLEMENTED` unconditionally (`handleConfig()`'s `patch_kind != MOTOR` branch, after the `otos` special case) | — |
| `motor` | 2 | **Live** — `applyMotorConfigPatch()` merges present `kp`/`ki`/`kff`/`i_max`/`kaw` onto BOTH bound motors; `travel_calib` onto the addressed `side` only | Yes — merged into `persistedTuning_`, flash-written on change (114-004) |
| `otos` | 5 | **Live** — `applyOtosPatch()` calls `Otos::setLinearScalar()`/`setAngularScalar()`/`setOffset()`/`init()` directly | Yes (except `init`, a one-shot trigger, never persisted) |

**Reserved, not reused**: field 3 (`PlannerConfigPatch`, deleted with
the S1 motion-stack excision) and field 4 (`watchdog` — the pre-116
`StreamingDriveWatchdog` window `uint32 sTimeout`, deleted alongside
`App::Deadman`; every `Move` is self-bounding now, so there is nothing
left for a separate watchdog window to configure).
`ConfigTarget.CONFIG_WATCHDOG` (`config.proto`) stays declared, unused
— same precedent `CONFIG_PLANNER` already set.

A `ConfigDelta` arriving mid-`Move` never disturbs the active `Move`'s
staged velocity or `StopCondition` baseline —
`handleConfig()` only ever touches `motorL_`/`motorR_`/`otos_`/
`persistedTuning_`, with no reference to `drive_`/`moveQueue_`/`odom_`
at all (verified by `app_robot_loop_harness.cpp`'s SUC-055 scenario,
`:1161-1219`).

---

## 7. Responses

### 7.1 The single ack slot — the ONLY per-command outcome path

There is no per-command `ReplyEnvelope`. Every command's outcome rides
`Telemetry.ack_corr`/`ack_err` (telemetry.proto, §8) inside the next
`Telemetry` push — `App::Telemetry::ack(corrId, errCode)` marks the ack
"fresh" so the very next primary-frame emission sets `flags` bit 5
(`ack_fresh`) and carries it.

**AS-BUILT**: `ReplyEnvelope`'s `ok` (`Ack`) and `err` (`Error`) oneof
arms are declared in the schema but have **zero live producers** in the
current firmware — `Comms::sendReply()` is called from exactly one call
site (`App::Telemetry::emitPrimary()`, `telemetry.cpp:113-115`),
always with `body_kind = TLM`. A wire sniffer will never observe a
`ReplyEnvelope{ok: ...}` or `ReplyEnvelope{err: ...}` frame from this
firmware — only `ReplyEnvelope{tlm: Telemetry}`. `Ack`/`Error` remain
declared-only schema (same posture `envelope.proto`'s own doc comment
states), available to a future ticket that wants a synchronous same-line
ack for a command whose result cannot wait a telemetry cycle.

### 7.2 Two kinds of ack ride the same slot

1. **Enqueue/command ack** — `ack_corr = CommandEnvelope.corr_id`,
   `ack_err` = the `ErrCode` from dispatch (§7.3). Sent for every `move`/
   `config`/`stop` that reaches a handler (i.e. every command that was
   successfully decoded — §7.4 covers what happens to one that wasn't).
2. **MOVE completion ack** — `ack_corr = Move.id` (NOT the enqueue
   envelope's `corr_id`), sent on the exact cycle the active `Move` ends
   (§5.2).

Because it is the same single slot, an enqueue ack and a completion ack
landing in the same primary period overwrite each other
(stakeholder-accepted "ack-depth-1" tradeoff, unchanged from the frame
v2 rewrite — `wait_for_ack()`'s timeout+retry covers the rare
collision).

### 7.3 AS-BUILT: the completion ack's `ack_err` is always 0 — timeout is signaled by the flags bit, not `ack_err`

The set-point issue's Responses section reads (arguably ambiguously):
*"a second ack on the cycle the command ends ... `ack_err` = 0 for a met
stop condition; a timeout ending additionally sets the `flags`
move-timeout fault bit."*

**What shipped** (`robot_loop.cpp:513-519`):

```cpp
tlm_.setFlag(kFlagFaultMoveTimeout, moveTimedOut);
if (moveResult.completed) {
  tlm_.ack(moveResult.completion.moveId, 0);   // ack_err is ALWAYS 0 here
}
```

The completion ack's `ack_err` is **unconditionally 0**, regardless of
whether the `Move` ended via its stop condition or via `timeout`. The
host distinguishes the two outcomes **only** via `flags` bit 15
(`kFlagFaultMoveTimeout`) on the same frame that carries the completion
ack — never via a nonzero `ack_err`. `TLMFrame.fault_move_timeout`
(`protocol.py`) is the host-side read of that bit.

### 7.4 A malformed/undecodable frame gets no reply at all

`Comms`'s dearmor path (armor error, base64 error, or
`msg::wire::decode()` failure) **never replies synchronously** — it
increments `malformedCount_` and returns, leaving `Cmd.status` at
`kNone` (`comms.cpp:78-115`, `decodeArmoredLine()`'s own doc comment:
"NEVER replies"). `RobotLoop::processMessage()`'s switch on
`CmdKind::NONE` dispatches to no handler and sends no ack of any kind
for that line. The only observable effect is
`kFlagFaultCommsMalformed` (bit 9) going high on a subsequent telemetry
frame (`comms.malformedCount() > 0`, `robot_loop.cpp:167`). Practically:
`ERR_DECODE` is a schema-declared `ErrCode` value with **no live wire
producer** — no ack frame this firmware sends will ever carry it.

### 7.5 Error taxonomy (`envelope.proto`'s `ErrCode`)

| Code | Value | Meaning | Live producer today? |
|---|---|---|---|
| `ERR_NONE` | 0 | OK | Yes — every successful `move`/`stop`/`config(motor\|otos)` |
| `ERR_UNKNOWN` | 1 | No such oneof arm / unknown target | **No** — nothing in `src/firm/app/` constructs this; a `CommandEnvelope` with no recognized `cmd` arm simply gets no ack at all (§7.4-adjacent: `CmdKind::NONE` dispatches nowhere) |
| `ERR_BADARG` | 2 | Malformed argument | Yes — `handleMove()`'s shape check (§4.1: missing velocity/stop variant, non-positive `timeout`) |
| `ERR_RANGE` | 3 | A `(min)`/`(max)`/`(abs_max)` bound violated | **No** — no `CommandEnvelope`-reachable field in the current schema declares one of these bound options (they exist only on outbound `Telemetry` fields, for wire-size estimation, not runtime enforcement — `src/firm/messages/DESIGN.md` §3) |
| `ERR_FULL` | 4 | Destination queue full | Yes — `MoveQueue::enqueue()`, a 5th pending `Move` (§5.1) |
| `ERR_DECODE` | 5 | Malformed wire bytes | **No live wire producer** — see §7.4; the code exists in the schema but a decode failure is silently counted, never acked |
| `ERR_UNIMPLEMENTED` | 6 | Declared-only arm, no live consumer | Yes — `handleConfig()`'s `drivetrain` patch branch (§6) |
| `ERR_OVERSIZE` | 7 | Encoded reply would exceed the envelope cap | **No** — `Comms::sendReply()`'s `n == 0` branch is documented "unreachable in practice" (the buffer is sized from the same generated constants `encode()` is budgeted against) and silently drops the frame rather than emitting this code |
| `ERR_NOT_CONFIGURED` | 8 | Composition root refused `move` — config-completeness gate not yet satisfied | Yes — `handleMove()`'s gate (§4.1, §5.7) |

`Error.field` (the `field`-specific companion in the `Error` message)
names which `CommandEnvelope` field failed validation — moot in
practice today since `Error` itself has no live producer (§7.1); this
column is preserved for the schema's own future use.

---

## 8. Telemetry frame v2 reference

Rides `ReplyEnvelope{corr_id=0, tlm: Telemetry}` (unsolicited, `corr_id`
always 0 for this arm), emitted **every loop cycle** — primary period ==
cycle period, ~50 Hz / 20 ms
(`App::Telemetry::kPrimaryPeriod`, unchanged by sprint 116). "The frame
is the dataset": with no on-chip measurement ring and no dump command,
a timestamped frame every iteration is the entire dataset-construction
path — the host's log of this stream (`tlm_log.py`, §9) reconstructs
any time window.

### 8.1 `Telemetry` fields

| Field | # | Type | Always present? |
|---|---|---|---|
| `now` | 1 | `uint32` [ms] robot clock at frame assembly | always |
| `seq` | 2 | `uint32` | always — increments once per SENT primary frame |
| `mode` | 3 | `DriveMode` (`IDLE`/`STREAMING`/`TIMED`/`DISTANCE`/`GO_TO`/`VELOCITY`) | always — `VELOCITY` iff `moveQueue_.active()`, else `IDLE` (`robot_loop.cpp:145`) |
| `flags` | 4 | `uint32` bit-string — see §8.2 | always |
| `ack_corr` | 5 | `uint32` — valid iff `flags` bit 5 | when fresh |
| `ack_err` | 6 | `uint32` (`ErrCode`) — valid iff `flags` bit 5 | when fresh |
| `enc_left` / `enc_right` | 7 / 8 | `EncoderReading{position[mm], velocity[mm/s], time[ms]}` | always |
| `otos` | 9 | `OtosReading{x,y[mm], heading[rad], v_x,v_y[mm/s], omega[rad/s], time[ms]}` | valid iff `flags` bit 0 |
| `pose` | 10 | `Pose2D{x,y[mm], h[rad]}` — encoder-odometry integrated pose | always |
| `twist` | 11 | `BodyTwist3{v_x[mm/s], v_y, omega[rad/s]}` — fused from both wheels' measured velocities | always (`v_y` always 0 on this differential build) |
| `line` | 12 | `uint32`, 4 packed 1-byte channels (ch1 low byte) | valid iff `flags` bit 13 |
| `color` | 13 | `uint32`, packed RGBC (R low byte) | valid iff `flags` bit 14 |

### 8.2 `flags` bit table

| Bit | Constant | Meaning |
|---|---|---|
| 0 | `kFlagOtosPresent` | `OtosReading` fresh this frame |
| 1 | `kFlagOtosConnected` | live OTOS bus health |
| 2 | `kFlagActive` | motion in progress (`moveQueue_.active()`) |
| 3 | `kFlagConnLeft` | left motor bus connectivity |
| 4 | `kFlagConnRight` | right motor bus connectivity |
| 5 | `kFlagAckFresh` | `ack_corr`/`ack_err` are a NEW ack this frame (Telemetry-internal) |
| 6 | `kFlagFaultI2CSafetyNet` | I2C clearance safety-net trip — boot-time one-shot, NOT actionable if only set at boot |
| 7 | `kFlagFaultWedgeLatch` | wedge-latch detected |
| 8 | `kFlagFaultI2CNak` | I2C NAK/timeout (declared, not yet wired to a live aggregate) |
| 9 | `kFlagFaultCommsMalformed` | malformed/undecodable inbound frame seen (§7.4) |
| 10 | `kFlagEventDeadmanExpired` | **orphaned** — its producer (`App::Deadman`) is deleted; this bit can no longer go high. Left declared, not repurposed. |
| 11 | `kFlagEventBootReady` | boot-ready transition |
| 12 | `kFlagEventConfigApplied` | declared, not yet wired |
| 13 | `kFlagLinePresent` | line word fresh this frame |
| 14 | `kFlagColorPresent` | color word fresh this frame |
| 15 | `kFlagFaultMoveTimeout` | a MOVE ended via its `timeout` backstop this cycle (§7.3) — sprint 116's own bit, first live caller |
| 16-31 | — | reserved |

### 8.3 Measured sizes

`gen_messages.py`-measured (`src/protos/telemetry.proto`'s own header
comment, unchanged by sprint 116 — this sprint only touched
`envelope.proto`'s `CommandEnvelope`/`ConfigDelta` oneofs):

- **`Telemetry`, standalone** (no enclosing tag/length prefix): **144 B**.
- **Wrapped as `ReplyEnvelope.body`'s `tlm` arm** (how a primary frame
  actually goes on the wire): 147 B arm contribution (144 B payload +
  1 B tag + 2 B length varint, since 144 ≥ 128 needs a 2-byte length) +
  6 B non-oneof `ReplyEnvelope` overhead = **153 B total**, 33 B margin
  under the 186-byte envelope budget (matches §2.3's table).

### 8.4 `TelemetrySecondary` — unchanged, out of this sprint's scope

The slower (~5 Hz) diagnostic frame (`cmd_vel`/`acc_*`/`glitch_*`/
`ts_*`) rides its own independently-armored `*B` line, own
`msg::wire::encode()` overload — untouched by the MOVE cutover. 52 B
worst case (§2.3). See `telemetry.proto`'s own header comment for its
field list; not repeated here.

---

## 9. Host API examples (`src/host/robot_radio/robot/protocol.py`)

`NezhaProtocol` is fire-and-poll for every command — each call writes
the envelope and returns the assigned `corr_id` immediately (never
blocks for a reply that will not come); pair with `wait_for_ack()` to
confirm the outcome.

```python
from robot_radio.io.serial_conn import SerialConnection
from robot_radio.robot.protocol import NezhaProtocol

conn = SerialConnection(port="/dev/cu.usbmodem2121102")
conn.connect()
proto = NezhaProtocol(conn)

# Bounded twist MOVE: drive forward at 150 mm/s for up to 2000 ms of
# elapsed time (TIME stop condition), 5000 ms timeout backstop,
# replace=True (the default) preempts anything already running.
corr = proto.move_twist(150.0, 0.0, 0.0, stop_time=2000.0, timeout=5000.0)
ack = proto.wait_for_ack(corr, timeout=500)
assert ack is not None and ack.ok

# Bounded wheels MOVE: both wheels at 100 mm/s until 300 mm of path has
# been traveled (DISTANCE stop condition), enqueued behind whatever is
# currently active (replace=False).
proto.move_wheels(100.0, 100.0, stop_distance=300.0, timeout=4000.0,
                   replace=False)

# Panic stop.
proto.stop()

# Live-tune a motor gain (persisted across power-cycle, §6).
proto.config(**{"pid.kp": 0.02})

# Live-apply an OTOS calibration offset (also persisted, §6).
proto.otos_config(offset_x=-51.5, offset_y=0.0)

# Drain telemetry (frame v2, §8) as parsed TLMFrame objects.
frames = proto.read_pending_binary_tlm_frames()
for f in frames:
    if f.ack_fresh and f.ack is not None:
        print(f.ack.corr_id, f.ack.ok, f.ack.err_code)
    if f.fault_move_timeout:
        print("a MOVE just timed out")
```

`stop_time`/`stop_distance`/`stop_angle` are mutually exclusive
keyword-only args on both `move_twist()`/`move_wheels()` — exactly one
must be given (`ValueError`, no wire traffic, otherwise), mirroring
`Move.stop`'s own oneof.

**`tlm_log.py`** (`src/tests/bench/tlm_log.py`) is the bench dataset
logger — drains `read_pending_binary_tlm_frames()` to a flat CSV, one
row per frame (`frame_to_row()`), including every `flags`-derived
boolean (`flag_fault_move_timeout`, `flag_active`, …) and every
`EncoderReading`/`OtosReading` sub-field:

```bash
uv run python src/tests/bench/tlm_log.py --port /dev/cu.usbmodem2121102 --duration 60
```

---

## 10. Worked wire examples

Generated directly from this repo's own `envelope_pb2` bindings (the
same codec `protocol.py` builds on) — not hand-computed, so these bytes
are exactly what a real client sends/decodes.

**1. `MoveTwist` + TIME stop** — `corr_id=7`,
`move{twist{v_x=150, v_y=0, omega=0}, time=2000, timeout=5000, replace=true, id=42}`:

```
raw (26 B): 08 07 aa 01 15 0a 05 0d 00 00 16 43 1d 00 00 fa 44 35 00 40 9c 45 38 01 40 2a
wire line: *BCAeqARUKBQ0AABZDHQAA+kQ1AECcRTgBQCo=
```

**2. `MoveWheels` + DISTANCE stop** — `corr_id=8`,
`move{wheels{v_left=100, v_right=100}, distance=300, timeout=4000, replace=false, id=43}`:

```
raw (29 B): 08 08 aa 01 18 12 0a 0d 00 00 c8 42 15 00 00 c8 42 25 00 00 96 43 35 00 00 7a 45 40 2b
wire line: *BCAiqARgSCg0AAMhCFQAAyEIlAACWQzUAAHpFQCs=
```

**3. `STOP`** — `corr_id=9`, `stop{}`:

```
raw (4 B): 08 09 6a 00
wire line: *BCAlqAA==
```

**4. `CONFIG` (`MotorConfigPatch`)** — `corr_id=10`,
`config{motor{side=LEFT, kp=0.02, ki=0.001}}`:

```
raw (16 B): 08 0a 32 0c 12 0a 1d 0a d7 a3 3c 25 6f 12 83 3a
wire line: *BCAoyDBIKHQrXozwlbxKDOg==
```

(Field-tag sanity check on example 1: byte `aa 01` is the varint
encoding of `21<<3 | 2 = 170` — the length-delimited tag for
`CommandEnvelope.move`, field **21**, confirming §3's field-number
table against the actual bytes on the wire.)

---

## 11. Deliberately NOT in this protocol

Arc/segment moves, trajectory profiles, jerk limiting, heading cascade,
pose-fix injection, `GET`/`STREAM`/`ECHO`, plan dumps, ring dumps — all
reserved wire numbers, all recoverable from the `pre-gut-motion-stack`
tag if ever needed. The protocol is: **bounded velocity commands in,
timestamped measurements out.** There is also, as of this sprint, no
live text-plane `STOP`/`ID`/`VER`/`HELP`/`SET`/`GET` — see §2.4; those
belong to protocol v2/v3's larger text rump, not this one.

---

## 12. Verification

The bench protocol gate this document's contract is verified against
lives in ticket 010 of this sprint
(`clasi/sprints/116-move-protocol-cutover/tickets/010-full-sweep-and-bench-gate.md`)
and, once run, its bench checklist
(`docs/bench-checklists/sprint-116-move-protocol.md`, following the
sprint-114/115 precedent in that directory) or a full sim dry-run if
hardware was unavailable at execution time. See
[`.claude/rules/hardware-bench-testing.md`](../.claude/rules/hardware-bench-testing.md)
for the general stand-testing procedure this project follows.

---

## Appendix: superseded documents

- [`docs/protocol-v2.md`](protocol-v2.md) — the original text-only
  protocol (pre-097). Superseded by v3 for motion/config/telemetry, and
  now by this document for everything still live in v3.
- [`docs/protocol-v3.md`](protocol-v3.md) — the post-097 binary-envelope
  + text-rump + `rogo` proxy protocol, frozen at sprint 097 and already
  partially stale against the 102-107 single-loop rebuild by the time
  this document was written (its own banner says so). Its command table
  (§3), `Twist`-arm-era `CommandEnvelope`, ack-ring telemetry shape, and
  `rogo` proxy description are all superseded by §3/§7/§8 above and the
  fact that `host/robot_radio/io/proxy.py`/`legacy_verbs.py` (the proxy
  itself) no longer exist in this tree.
