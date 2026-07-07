---
id: '001'
title: Command-plane queue primitives (Rt::Mailbox, Rt::WorkQueue)
status: done
use-cases:
- SUC-001
- SUC-002
- SUC-006
depends-on: []
github-issue: ''
issue: plan-file-a-design-issue-blackboard-architecture-state-objects-command-queues.md
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Command-plane queue primitives (Rt::Mailbox, Rt::WorkQueue)

## Description

Implement the two command-plane primitives specified in
`architecture-update.md`'s Reference code ("The two command-plane
primitives"): `Rt::Mailbox<T>` (capacity 1, latest-wins — for absolute
setpoints) and `Rt::WorkQueue<T,N>` (FIFO, capacity N, `pop`/`peek` — for
deltas/commands that must all apply in order). These are the foundation
every later ticket in this sprint builds on (the Blackboard, the
Configurator, the CommandRouter, and every subsystem's adapted `tick()`
signature all consume these two templates). This ticket touches no
blackboard, no subsystem, and no command-family code — it is a pure,
dependency-free pair of templates, greenfield (Grounding confirmed no
blackboard/queue-shaped code exists anywhere in `source/` today).

## Acceptance Criteria

- [x] `Rt::Mailbox<T>`: `post()` overwrites any unread value; `empty()`
      accurately reflects fill state; `take()` returns the latest posted
      value and clears the full flag; a `take()` on an empty mailbox is
      well-defined (returns a default-constructed `T`, stays empty).
- [x] `Rt::WorkQueue<T,N>`: `post()` appends in FIFO order and returns
      `false` when at capacity N (never silently overwrites or drops
      without signaling full); `take()` pops front in FIFO order; `peek(i)`
      is non-destructive and matches `take()`'s eventual order; `size()`/
      `empty()` are accurate after any sequence of `post`/`take`.
- [x] Both templates compile with zero dependencies beyond `<cstdint>` (no
      `MicroBit.h`, no `I2CBus`, no `msg::` types required to compile the
      template itself) — instantiable with any trivially-copyable payload,
      verified by instantiating with at least one plain POD test type and
      at least one real `msg::` type (e.g. `msg::MotorCommand`) in the test
      harness.
- [x] No heap allocation — `WorkQueue` uses a fixed-size array member,
      `Mailbox` a single value member — consistent with the project's
      no-heap-in-hot-path constraint (`docs/architecture/architecture-034.md`
      §11).

## Implementation Plan

**Approach.** New header-only file `source/runtime/queue.h`, namespace
`Rt`, built directly from `architecture-update.md`'s Reference code block
(verbatim starting point). Internal representation (e.g. `WorkQueue`'s
ring-buffer index wraparound) may be refined during implementation as long
as the public `post`/`empty`/`take`/`peek`/`size` contract above holds.

**Files to create:**
- `source/runtime/queue.h` (this ticket creates the `source/runtime/`
  directory — it does not exist yet, per `architecture-update.md`'s
  Grounding section).

**Files to modify:** none.

**Testing plan:**
- New `tests/sim/unit/runtime_queue_harness.cpp` — a hand-rolled,
  dependency-free host harness (no CMake, no ARM toolchain), following the
  pattern in `tests/sim/unit/drivetrain_harness.cpp`/
  `motor_policy_harness.cpp` (PASS/FAIL prints, nonzero exit on failure).
  Covers: `Mailbox` overwrite/take/empty semantics; `WorkQueue` FIFO order,
  full-queue `post()` rejection, `peek()` non-destructive iteration,
  `size()` accounting under interleaved post/take.
- New `tests/sim/unit/test_runtime_queue.py` driving the harness via
  subprocess, matching `test_drivetrain.py`'s pattern.
- **Verification command**: `uv run pytest tests/sim/unit/test_runtime_queue.py`

**Documentation updates:** None required — internal runtime primitive, no
wire-visible behavior; `architecture-update.md` already documents the
design.
