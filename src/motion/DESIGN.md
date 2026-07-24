# Motion (`src/motion`) — Motion-Control Library (extraction in progress)

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-24 · **Status:** in-flux

---

## 1. Purpose

`src/motion/` is the motion-control half of sprint 122's two-layer split
(see sprint 122's `sprint.md`, Architecture): a SIBLING tree to `src/firm`
(not a child — Design Rationale Decision 3), holding the twist
decomposition/queueing/shaping/estimation/odometry logic that is still
under active development (goal-exact tours, same-axis carry, heading
hold), separated from the hardware-facing base (`src/firm`) the
stakeholder intends to eventually freeze and move to its own repository
(`git subtree split`). Not yet in `.clasi/config.yaml`'s validated
`sources:` list (stays `[src/firm, src/host]`, stakeholder-locked,
Decision 3's own consequence) — this directory is real, current
documentation, unvalidated by the mechanical design-doc checker, the same
treatment `src/sim`/`src/protos`/`src/scripts` already get.

**This is a STUB.** Ticket 122-001 (this ticket) only moves the three
already-pure leaves (`StopCondition`, `VelocityShaper`, `BodyKinematics`)
and stands up the boundary header; ticket 122-002 moves `MoveQueue`,
`StateEstimator`, `Odometry`, and the twist half of `Drive` in behind the
boundary. Ticket 122-004 is the one that reconciles this file into its
real, final form (full Constraints/Design/Interfaces sections for the
WHOLE tree, once everything that belongs here has actually landed) — until
then, treat `src/firm/motion/DESIGN.md` and `src/firm/kinematics/
DESIGN.md` (both still physically present, each carrying a forwarding
note to this file) as the authoritative math/rationale reference for the
three modules that have moved so far.

## 2. Orientation (as of ticket 122-001)

| File | Role |
|---|---|
| `stop_condition.{h,cpp}` | `Motion::StopCondition` — per-Move stop-condition comparison (Time/Distance/Angle + timeout backstop). Moved verbatim from `src/firm/motion/` — zero behavior change (see that directory's own DESIGN.md for full derivation). |
| `velocity_shaper.{h,cpp}` | `Motion::VelocityShaper` — the decel-into-the-goal jerk-limited speed shaper. Moved verbatim from `src/firm/motion/` — zero behavior change (full derivation: same source). |
| `body_kinematics.{h,cpp}` | `BodyKinematics` — stateless differential-drive twist/wheel-speed maps and curvature-preserving saturation. Moved verbatim from `src/firm/kinematics/` (that directory is retired — folded flat into `src/motion`, no nested `kinematics/` subdirectory here). |
| `wheel_sink.h` | `Motion::WheelSink` — the ONE new boundary header this ticket adds: an abstract wheel-velocity command sink (`setWheels(v_left, v_right)`/`stop()`), the per-wheel state struct (`WheelState`) Motion reads, and the plain config struct (`WheelSinkConfig`) Motion is constructed with. **Not yet load-bearing** — no code includes it yet; ticket 002's `Motion::MoveQueue` is its first consumer. No concrete implementation lives here (that's the base's job, `src/firm/app`, ticket 002). |
| `CMakeLists.txt` | The standalone `motion_tests` build (Design Rationale Decision 4) — plain CMake, no `libfirmware_host`, no ctypes, no Python. Builds a static `motion` library from all three moved `.cpp` files plus two `ctest`-registered executables reusing the existing `src/tests/sim/unit/motion_{stop_condition,velocity_shaper}_harness.cpp` scenario coverage, and a `motion_tests` custom target that builds + runs both via `ctest --output-on-failure`. See that file's own header comment for exact build/run commands. |

## 3. Constraints and Invariants (the part ticket 122 locks down; full
   reconciliation deferred to ticket 004)

- **`src/motion` imports NOTHING from `src/firm` except `messages/`
  headers.** Verified by grep (`grep -rn '#include "' src/motion | grep -v
  'messages/'` shows no `src/firm`-only path) — `body_kinematics.h`'s own
  `#include "messages/common.h"` is the one, explicitly allowed exception
  (the array-form `BodyKinematics` overloads take `msg::BodyTwist3`).
  `stop_condition.{h,cpp}`/`velocity_shaper.{h,cpp}` have zero `src/firm`
  dependency at all, same as before the move.
- **The boundary is a velocity sink, never a duty sink** (sprint 122
  Design Rationale Decision 1, stakeholder-locked 2026-07-24) —
  `Motion::WheelSink::setWheels(v_left, v_right)`/`stop()`, not a duty
  target. The duty-sink rewrite (folding sprint 2's PID-placement
  decision in) is explicitly deferred, not a discretionary call this
  ticket reopened.
- **Zero behavior change, zero wire change.** Ticket 122-001 is a pure
  mechanical move of three already-pure modules — every existing test
  passes unchanged; see sprint 122's Test Strategy for the recorded
  pre-/post-extraction baseline diff.
- **Qualified `#include "motion/...h"` paths resolve unchanged** in every
  `src/firm` caller that already said e.g. `#include "motion/
  stop_condition.h"` (`src/firm/app/move_queue.h`) — both the ARM build
  (root `CMakeLists.txt`) and the sim build (`src/sim/CMakeLists.txt`) add
  `src/` itself (the parent of both `src/firm` and `src/motion`) as an
  include root, so the directory literally being named `motion` at its
  new top-level location keeps every existing qualified include text
  valid with zero call-site churn. The one path that DID change:
  `#include "kinematics/body_kinematics.h"` → `#include
  "motion/body_kinematics.h"` (the `kinematics/` directory is retired,
  folded flat into `src/motion` rather than nested as `src/motion/
  kinematics/`).

## 4. Open Questions

Same three sprint.md leaves open for ticket 002 to resolve (not
re-litigated here): the exact boundary-interface/class naming (`Motion::
WheelSink` is a suggestion, not a requirement), whether `App::Drive` keeps
its name once it only implements the wheel-target sink, and whether the
existing `motion_*_harness.cpp` pytest wrappers are eventually retired in
favor of this directory's own `ctest` target or kept as thin subprocess
wrappers alongside it (both currently coexist, unchanged, pointed at their
new location).
