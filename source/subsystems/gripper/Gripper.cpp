// Gripper — Phase E (043-003) optional-servo subsystem seam + GripperIONull
// null-object.  All method bodies are trivial no-ops defined inline in the
// header (periodic()/updateInputs() are no-ops; the ctor stores the ref).  This
// translation unit exists so the file is present in the documented Phase E file
// set (architecture-update.md "Files Created") and is picked up by the recursive
// source globs (sim GLOB_RECURSE + firmware RECURSIVE_FIND_FILE), giving the
// subsystem a stable compilation unit for Phase F to grow into.
//
// Behavior-preservation: this file emits no code that runs in the tick path; the
// gripper is command-driven via ServoController.  golden-TLM stays byte-exact.
#include "Gripper.h"
