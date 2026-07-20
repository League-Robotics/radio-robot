// bench_test_config.h -- TestSupport: test-tree-only stand-in
// PlannerConfig/MotorConfig values for src/tests/sim/** harnesses that need
// SOME reasonable, real (nonzero) configuration to exercise jerk-limited
// motion, wheel PID tracking, etc. against TestSim::SimHarness (114-001,
// Decision 3, sprint.md).
//
// Origin: TestSim::SimHarness used to bake these SAME values in unconditionally
// (its own now-deleted private makeExecutorConfig()/makeMotorConfig() --
// src/sim/sim_harness.h), so every freshly-constructed harness was always
// already configured. Ticket 114-001 built a configuration-completeness gate
// (App::RobotLoop::isConfigured()/markConfigured()) that makes "unconfigured"
// a real, refusable state -- SimHarness itself must not carry a hardcoded
// behavioral default anymore (the config-as-truth ethic: a value like
// vel_kp=0.003 baked here, silently diverging from data/robots/*.json's own
// vel_kp=0.002, is exactly the class of bug this sprint exists to close).
// These values move here, UNCHANGED (byte-for-byte, including every
// explanatory comment), so every pre-existing sim test harness that relied
// on them keeps its exact prior behavior via one added line:
// `TestSupport::configureSimForBenchTest(sim);` right after construction.
//
// This header is explicitly TEST-TREE-ONLY -- it lives under
// src/tests/sim/support/, never under src/sim/ or src/firm/, so it can never
// be mistaken for (or accidentally reached from) production composition-root
// code. A real robot's actual configuration always comes from
// data/robots/*.json via gen_boot_config.py (main.cpp) or
// configure_from_robot() (the sim's own host-side JSON-driven path) -- never
// from this file.
#pragma once

#include <cstdint>

#include "devices/device_config.h"
#include "messages/planner.h"

namespace TestSim {
class SimHarness;
}  // namespace TestSim

namespace TestSupport {

// benchTestPlannerConfig -- byte-for-byte the deleted
// TestSim::SimHarness::makeExecutorConfig() body (see that function's own
// former doc comment, preserved verbatim below): a non-zero msg::PlannerConfig
// for Motion::Executor's own configure() call (109-003). This harness has no
// boot_config.cpp to read a real per-robot value from (main.cpp's own
// Config::defaultPlannerConfig()); these are reasonable stand-in values
// (matching data/robots/tovez.json's own order of magnitude) sufficient for a
// TIMED-mode ramp/hold/ramp-down to exercise real jerk-limited motion in a
// sim test -- NOT bench-tuned, and not meant to be (no bench/sim test in this
// ticket asserts a SPECIFIC numeric gain, only jerk-boundedness/no-instant-
// step/queue-mechanics).
msg::PlannerConfig benchTestPlannerConfig();

// benchTestMotorConfig -- byte-for-byte the deleted
// TestSim::SimHarness::makeMotorConfig(uint32_t port) body (see that
// function's own former doc comment, preserved verbatim below): see
// sim_api.cpp's own (now also relocated/superseded) makeMotorConfig() for the
// byte-for-byte derivation of every field set here -- unchanged tuning, just
// relocated. A large proportional gain (kp) plus a wide slew rate lets an
// injected twist saturate the PID quickly and reach full duty in one write;
// the harness's own SimPlant then integrates whatever duty actually lands on
// the wire, live, so there is no predictor to keep in sync with this tuning
// the way SimApi's DutyPredictor had to be. port: 1 = left, 2 = right (same
// convention as every other port-keyed call in this codebase).
Devices::MotorConfig benchTestMotorConfig(uint32_t port);

// configureSimForBenchTest -- convenience wrapper: pushes
// benchTestPlannerConfig() via sim.configurePlanner(), then
// benchTestMotorConfig(1)/benchTestMotorConfig(2) via sim.configureMotor()
// for both ports. This is the ONE call every pre-existing (and any new)
// src/tests/sim/** harness adds right after constructing a bare
// TestSim::SimHarness and before its first injectTwist()/injectMove()/
// step()/boot() call, to restore byte-for-byte the same "always already
// configured" behavior SimHarness's own constructor used to provide
// unconditionally -- now explicit, test-tree-only, and opt-in.
void configureSimForBenchTest(TestSim::SimHarness& sim);

}  // namespace TestSupport
