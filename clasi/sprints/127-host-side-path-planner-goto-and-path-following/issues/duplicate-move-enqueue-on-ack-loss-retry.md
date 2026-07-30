---
status: in-progress
sprint: '127'
tickets:
- 127-002
---

# Duplicate move enqueue when an enqueue ack is lost and the host retries

## Description

A host that retries an unacked `MOVE` enqueue causes the firmware to execute
that move **twice**. The retry carries a fresh `corr_id` but the **same
`Move.id`**, and firmware has no idempotency check, so a lost *ack* (as
opposed to a lost *command*) turns one intended move into two executed ones.

Observed on hardware 2026-07-28, on the **direct USB link** (not the radio
relay), running `src/tests/bench/planner_square_tour.py` against tovez:

- **Run 1** — `(retry 1 for move 9005)`. A ~3 s runaway at turn 3: left wheel
  measured +410 mm/s held for three seconds while the right sat at −100 mm/s;
  encoder heading spiked from 270° to 1030° before decaying back. Reported
  path 2505 mm against a 2000 mm target (+505 mm); the glitch's own mean wheel
  speed × duration accounts for ~465 mm of that, i.e. essentially all of it.
- **Run 2** — `(retry 1 for move 9006)`. The robot executed **five** 90° turns
  instead of four: heading 450.3° against a 360° target, and the wheel
  differential dR−dL = 1006 mm against the five-turn theoretical
  4+1 turns × 2 × (trackwidth/2) × (π/2) = 1005 mm. Straights were
  unaffected and accurate (path 2010 mm vs 2000 mm target, +0.5%).

Both runs retried, so the failure is not rare in practice. The drive layer
itself is healthy in both runs — legs track commanded speed to within ~1%
and per-turn geometry matches theory. One duplicated command is what wrecks
an otherwise good tour.

## Cause

`src/tests/bench/planner_square_tour.py` (lines ~89-99) retries an enqueue
whose ack has not arrived within `ENQUEUE_RETRY_S = 1.2 s`, up to
`ENQUEUE_RETRIES_MAX = 6` times, re-sending via
`proto.move_twist(**spec)` — a **fresh `corr_id`, the same `Move.id`**. Its
own comment names this hazard and accepts it:

> A retry after a lost *ack* (rather than a lost command) would double-enqueue
> that move -- accepted as rare-by-construction: the ack ring rides EVERY
> telemetry frame, so an ack only vanishes if all frames carrying it drop,
> while a command is one single line.

That reasoning does not hold as observed: retries fired in 2 of 2 runs over
USB. Whatever the per-frame loss rate, the assumption that an enqueue ack is
effectively never lost is empirically false on this link.

Firmware side, `RobotLoop::handleMove()`
(`src/firm/app/robot_loop.cpp`, ~line 130-188) validates `velocity_kind`,
`stop_kind` and `timeout`, converts the wire `msg::Move` into a
`Motion::Move`, then calls `planner_.move(m, move.replace)` and acks. It
never inspects `Move.id` against anything previously accepted, so the second
copy is enqueued like any new move. `Motion::Planner::move()`
(`src/motion/planner/planner.h`) is likewise id-agnostic — it treats
`Move.id` purely as a label to echo in the completion ack
(`activeMoveId()`).

## Proposed fix

Make `MOVE` enqueue **idempotent on `Move.id`** at the wire boundary in
`RobotLoop::handleMove()`. This is a protocol-level retry concern, not a
motion concern: `Motion::Planner` must stay id-agnostic and must not learn
about transport retries.

### Behaviour

When an arriving `MOVE` carries a non-zero `Move.id` that this session has
already accepted, do **not** enqueue it again. Ack it with `err == 0` against
the *new* `corr_id`, exactly as if it had just been accepted.

Acking success (rather than a new error code) is deliberate and required: the
host's retry loop is waiting for an ack to clear `inflight`, and the move
genuinely *is* enqueued/executing/complete. Returning an error would make the
host treat a correctly-executed move as failed and abort the tour.

### `Move.id == 0` must be exempt

`move_id` defaults to `0` in
`src/host/robot_radio/robot/protocol.py` (`move_twist()`,
`move_wheels()`, and friends all take `move_id: int = 0`), meaning "unset /
don't care". Many bench scripts never set it. Deduping on id 0 would silently
drop every default-id move after the first — a far worse regression than the
bug being fixed. **Treat id 0 as "always accept, never record."**

### Suggested shape

A small fixed-size ring of recently accepted ids on `App::RobotLoop`, checked
before `planner_.move()` and appended after a successful accept:

```cpp
// robot_loop.h, private
static constexpr int kAcceptedMoveIdCount = 16;   // >> the 5-deep move queue
uint32_t acceptedMoveIds_[kAcceptedMoveIdCount] = {};  // 0 = empty slot
int acceptedMoveIdNext_ = 0;

bool alreadyAccepted(uint32_t id) const;   // false for id == 0
void recordAccepted(uint32_t id);          // no-op for id == 0
```

The window must **outlive completion**: the lost-ack case includes a move
that already ran to completion before the retry lands, so ids cannot be
evicted when the move leaves the queue. 16 slots comfortably covers the
8-move tour and any realistic retry window while staying well inside the
firmware's RAM budget (currently ~98% used — keep the array `uint32_t`, do
not widen it).

Sizing note: with a ring this small, a long session will eventually wrap and
could re-accept a very old duplicate id. That is acceptable — the retry
window is seconds, and hosts assign ids monotonically.

Insert in `handleMove()` after the existing `velocity_kind`/`stop_kind`/
`timeout` validation and after the twist `badShape` check (so a malformed
duplicate still gets `ERR_BADARG`, not a false success), and immediately
before `drive_.estop()`:

```cpp
if (alreadyAccepted(move.id)) {
  tlm_.ack(env.corr_id, 0);   // idempotent: already enqueued/run
  return;
}
```

Note this early return **must** skip `drive_.estop()` as well — re-estopping
Drive on a duplicate would disturb a planner move already in flight.

Record the id only on a successful accept:

```cpp
const bool accepted = planner_.move(m, move.replace);
if (accepted) recordAccepted(move.id);
tlm_.ack(env.corr_id, accepted ? 0 : static_cast<uint32_t>(msg::ErrCode::ERR_FULL));
```

An `ERR_FULL` rejection must NOT be recorded — the host is entitled to retry
that move for real once the queue drains.

### `replace=true` interaction

A duplicate carrying `replace=true` must also be suppressed. Re-running a
replace would clear the queue and restart the move mid-flight, which is
precisely the run-1 runaway signature (a fresh move activating on top of one
already executing). The id check sits before any `replace` handling, so this
falls out of the ordering above.

## Verification

- **Sim:** a `src/tests/sim/` case that enqueues the same non-zero `Move.id`
  twice and asserts the planner queue depth rises by exactly one, that both
  commands ack with `err == 0`, and that a *different* id still enqueues.
- **Sim:** two moves both with `Move.id == 0` must BOTH enqueue (the
  exemption above).
- **Sim:** a duplicate id arriving after the first has completed is still
  suppressed (window outlives completion).
- **Sim:** an `ERR_FULL`-rejected move's id is not recorded — re-sending it
  after the queue drains enqueues normally.
- **Hardware:** `uv run python src/tests/bench/planner_square_tour.py --port
  <robot port>` completes 8/8 with heading within a few degrees of 360° and
  **exactly four** turns (check dR−dL ≈ 804 mm, the four-turn value for
  trackwidth 128, NOT 1005 mm). Run it several times so at least one run
  contains a logged `(retry N for move …)` — that is the case being fixed,
  and a run without a retry proves nothing.

## Related

- `src/tests/bench/planner_square_tour.py` — the reproducer; its lines ~89-99
  document the hazard as accepted-but-known.
- `docs/protocol-v5.md` — the `MOVE` command plane and the bounded ack ring
  that carries enqueue and completion acks.
- **Not the same issue:** run 2 also showed a separate, unexplained stall —
  move 9007 alone took ~32 s (total run 81.3 s vs run 1's 35.6 s). Dedup does
  not obviously explain that, and it needs its own investigation.
- Wheel-speed calibration was recalibrated the same day (plant gain had
  dropped ~10%: left 876→784, right 833→762 mm/s per duty); with the new
  constants the tour's straights measure +0.5%, so leg accuracy is NOT a
  contributor to the failures above.
