// bench_test_config.h -- TestSupport: test-tree-only stand-in MotorConfig
// values for src/tests/sim/** harnesses that need SOME reasonable, real
// (nonzero) configuration to exercise wheel PID tracking etc. against
// TestSim::SimHarness (114-001, Decision 3, sprint.md).
//
// 115-006 (gut S1 sim lockstep): benchTestPlannerConfig() DELETED --
// msg::PlannerConfig and SimHarness::configurePlanner() no longer exist
// (Motion::Executor/App::Pilot/App::HeadingSource were deleted by 115-002's
// motion-stack excision). configureSimForBenchTest() below now pushes only
// the two benchTestMotorConfig() calls -- there is no planner half left to
// push.
//
// Origin: TestSim::SimHarness used to bake these SAME values in unconditionally
// (its own now-deleted private makeExecutorConfig()/makeMotorConfig() --
// src/sim/sim_harness.h), so every freshly-constructed harness was always
// already configured. Ticket 114-001 built a configuration-completeness gate
// (App::RobotLoop::isConfigured()/markConfigured()) that makes "unconfigured"
// a real, refusable state -- SimHarness itself must not carry a hardcoded
// behavioral default anymore. These values move here (114-006 aligned the
// one field that had drifted -- velGains.kp -- to data/robots/*.json's own
// shipped value; see bench_test_config.cpp's own comment at that field),
// so every pre-existing sim test harness that relied on them keeps its
// exact prior behavior via one added line:
// `TestSupport::configureSimForBenchTest(sim);` right after construction.
// The config-as-truth ethic this guards against: a bench-test-only value
// baked here that silently diverges from data/robots/*.json's own shipped
// value for the SAME field is exactly the class of bug this sprint exists
// to close -- keep this file's values aligned with the robot JSON's own
// numbers for any field the JSON also carries (fields with no JSON
// equivalent, e.g. slewRate/wheelTravelCalib's sim-specific derivation,
// are unaffected and stay bench-tuned).
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

namespace TestSim {
class SimHarness;
}  // namespace TestSim

namespace TestSupport {

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
// benchTestMotorConfig(1)/benchTestMotorConfig(2) via sim.configureMotor()
// for both ports (115-006: the benchTestPlannerConfig()/configurePlanner()
// half is gone -- see this file's own header). This is the ONE call every
// pre-existing (and any new) src/tests/sim/** harness adds right after
// constructing a bare TestSim::SimHarness and before its first
// injectTwist()/step()/boot() call, to restore byte-for-byte the same
// "always already configured" behavior SimHarness's own constructor used to
// provide unconditionally -- now explicit, test-tree-only, and opt-in.
void configureSimForBenchTest(TestSim::SimHarness& sim);

}  // namespace TestSupport
