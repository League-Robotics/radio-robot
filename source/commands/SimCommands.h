#pragma once
#include "CommandTypes.h"

// Forward declaration — keeps the header-graph shallow (mirrors DebugCommands.h).
class SimHardware;

// ---------------------------------------------------------------------------
// SimCommands — sim-build-only Commandable that owns the SIMSET/SIMGET verb
// pair (069-003).
//
// SIMSET/SIMGET give the simulator's plant/error parameters (PhysicsWorld,
// SimOdometer) the same runtime-settable, wire-native mechanism SET/GET
// already gives RobotConfig -- WITHOUT adding any sim-only field to
// RobotConfig/ConfigRegistry.cpp, which are compiled into BOTH the ARM
// firmware target and the sim library (see architecture-update.md Design
// Rationale Decision 1).
//
// SimCommands is a second instance of the SAME optional, separately-owned
// Commandable* extension point DebugCommands already proved
// (Robot::buildCommandTable's `DebugCommands* dbg = nullptr` parameter):
// the ARM build never constructs a SimCommands, passes sim=nullptr (the
// default), and never #includes this header — see SystemCommands.cpp's
// `#ifdef HOST_BUILD`-guarded include/dispatch and Robot.h's forward
// declaration of `class SimCommands;`.
//
// kSimRegistry[] (SimCommands.cpp) dispatches through NAMED SETTER/GETTER
// FUNCTION POINTERS, not offsetof -- PhysicsWorld/SimOdometer are
// encapsulated classes with invariants, not POD structs, so the registry
// calls their existing setters/getters directly (no snapshot, no
// staleness; see architecture-update.md Design Rationale Decision 3).
// ---------------------------------------------------------------------------
class SimCommands : public Commandable {
public:
    explicit SimCommands(SimHardware& hal);

    virtual std::vector<CommandDescriptor> getCommands() const override;

private:
    SimHardware& _hal;
};
