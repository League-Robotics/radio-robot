// pilot.h -- App::Pilot: bridges Motion::Executor into RobotLoop's own
// cycle. Sprint 109 ticket 003's own "loop glue" module (sprint.md's
// Architecture section, Step 3 responsibility group 3): decides WHEN
// Executor solves/samples happen, never HOW (that is entirely Motion::
// Executor/JerkTrajectory's job) or WHAT the result means on the wire
// (that is RobotLoop::handleMove()'s job).
//
// Boundary: inside -- calling Executor::plan()/tick()/enqueue()/flush()/
// popEvent() at the right cycle points and staging the sampled twist onto
// Drive; outside -- decoding a wire Move into a Motion::Cmd (RobotLoop/
// Comms's job, via Motion::fromMove()), and turning a CompletionEvent into
// a wire ack (RobotLoop's job, via Telemetry's ack ring).
//
// Cycle placement (src/firm/DESIGN.md Sec 3/sprint.md's own table):
// `plan()` is called from the `kPace` budget block (all Ruckig solves
// happen there, <=1/cycle -- Executor::plan()'s own contract);
// `tick(now)` is called from the motorR settle block, AFTER
// processMessage()/the deadman check and BEFORE `drive_.tick()`, so a
// same-cycle enqueue/flush is reflected in this cycle's own staged twist.
// `tick()` only stages a twist (via Drive::setTwist()) while
// `state() != Motion::State::kIdle` -- while IDLE it does nothing at all,
// deliberately, so a same-cycle raw TWIST (handleTwist()'s own
// Drive::setTwist() call, which always also calls flush() first -- see
// this file's own flush() doc comment) is never immediately clobbered by
// a stale Executor sample.
//
// No bus traffic, no sleeps, no Clock/Sleeper dependency of its own --
// `tick(now)` takes the loop's own `now` (matching Telemetry::emit(now)'s
// existing pattern) and derives its own internal dt from consecutive
// calls, rather than reaching for a Devices::Clock the way Deadman does;
// Pilot has no need to read "now" for any purpose beyond that one delta.
#pragma once

#include <cstdint>

#include "app/drive.h"
#include "motion/cmd.h"
#include "motion/executor.h"

namespace App {

class Pilot {
 public:
  Pilot(Motion::Executor& executor, Drive& drive) : executor_(executor), drive_(drive) {}

  // enqueue -- forwards to Executor::enqueue(); see executor.h for the
  // classification/outcome contract. RobotLoop::handleMove() is the only
  // caller, and is the one that turns the outcome into a wire ack.
  Motion::EnqueueOutcome enqueue(const Motion::Cmd& cmd) { return executor_.enqueue(cmd); }

  // flush -- TWIST/STOP preemption. RobotLoop::handleTwist()/handleStop()
  // both call this BEFORE (or alongside) their own existing
  // Drive::setTwist()/stop() call, so this cycle's tick() (which runs
  // after processMessage() in the schedule) sees state()==kIdle and does
  // not restage a twist over the raw command's own.
  void flush() { executor_.flush(); }

  // plan -- see this file's own cycle-placement doc comment. At most one
  // JerkTrajectory solve (Executor::plan()'s own contract).
  void plan() { executor_.plan(); }

  // tick -- samples Executor::tick(dt) and, while running (state() !=
  // kIdle), stages the result onto Drive via setTwist(). dt is derived
  // from consecutive `now` values ([ms], the loop's own markTime()); the
  // very first call after construction has no prior `now` to diff against
  // and contributes dt=0 (a single zero-length sample -- harmless,
  // JerkTrajectory::sample(0) just returns the seed state).
  void tick(uint32_t now);  // [ms]

  // popEvent -- drains one pending completion event. RobotLoop drains all
  // pending events each cycle (bounded: the ring holds at most
  // Motion::kEventRingDepth) into Telemetry's ack ring.
  bool popEvent(Motion::CompletionEvent* out) { return executor_.popEvent(out); }

  uint8_t queueDepth() const { return executor_.queueDepth(); }
  uint32_t activeId() const { return executor_.activeId(); }
  Motion::State state() const { return executor_.state(); }

 private:
  Motion::Executor& executor_;
  Drive& drive_;

  bool hasLastTick_ = false;
  uint32_t lastTick_ = 0;  // [ms]
};

}  // namespace App
