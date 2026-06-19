---
id: '002'
title: Strip Commandable from Odometry; create OtosCommands app-layer handler set
status: done
use-cases:
- SUC-002
- SUC-003
depends-on:
- 041-001
github-issue: ''
issue: migrate-radio-robot-c-to-the-frc-elite-architecture-c-codal-adaptation.md
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Strip Commandable from Odometry; create OtosCommands app-layer handler set

## Description

Remove `Commandable` inheritance from `Odometry` and move the seven OTOS-tuning
verb handlers (`OI/OZ/OR/OV/OL/OA/OP`) verbatim to a new `source/app/OtosCommands`
class. Update `Robot::buildCommandTable` to aggregate `OtosCommands::getCommands()`
in place of `odometry.getCommands()`.

After this ticket:
- `Odometry.h` no longer includes `CommandTypes.h` — the `source/state/` layer is
  fully dependency-clean.
- The seven verbs work identically; `test_estimator_command_paths.py` passes green.
- The vendor-confinement baseline is updated to cover `source/state/`.

### Step-by-step plan

**1. Create source/app/OtosCommands.h**

```cpp
#pragma once
#include "CommandTypes.h"
#include "IOdometer.h"
#include "RobotState.h"

// Context bundle for OtosCommands handlers.
// Verbatim from Odometry.cpp's OdomCtx — only the struct name changes.
struct OtosCtx {
    IOdometer*           otos;
    const HardwareState* hwState;
};

class OtosCommands : public Commandable {
public:
    OtosCommands();

    virtual std::vector<CommandDescriptor> getCommands() const override;

    // Bind the IOdometer device and cached HardwareState pointer.
    // Call from Robot constructor after otos and state.inputs are live.
    void setCtx(IOdometer* otos, const HardwareState* hwState);

private:
    mutable OtosCtx _ctx;
};
```

**2. Create source/app/OtosCommands.cpp**

Move the following VERBATIM from `source/control/Odometry.cpp`:
- All seven static parse functions: `parseOI`, `parseOZ`, `parseOR`, `parseOP`,
  `parseOV`, `parseOL`, `parseOA`.
- All seven static handler functions: `handleOI`, `handleOZ`, `handleOR`,
  `handleOP`, `handleOV`, `handleOL`, `handleOA`.
- The `getCommands()` body.

Replace `OdomCtx*` casts in handlers with `OtosCtx*` casts.
Replace references to `c->odo` (which doesn't exist in `OtosCtx`) — the handlers
in `Odometry.cpp` use `c->otos` and `c->hwState` only, so no `odo` reference
needs porting. (Confirm: grep for `c->odo` in the seven handlers — it is only
used by the old `correct()` method which is NOT one of the seven command handlers.)

Constructor and `setCtx`:
```cpp
OtosCommands::OtosCommands() : _ctx{nullptr, nullptr} {}

void OtosCommands::setCtx(IOdometer* otos, const HardwareState* hwState) {
    _ctx.otos    = otos;
    _ctx.hwState = hwState;
}
```

`getCommands()` body: verbatim from `Odometry::getCommands()` but using
`&_ctx` as the context pointer and the `OtosCtx*` cast in handlers.

**3. Strip Commandable from Odometry**

In `source/control/Odometry.h`:
- Remove `public Commandable` from the class declaration.
- Remove `virtual std::vector<CommandDescriptor> getCommands() const override;`
  declaration.
- Remove the `OdomCtx` struct definition (it moved to `OtosCommands.h`).
- Remove `mutable OdomCtx _odomCtx;` private member.
- Remove `#include "CommandTypes.h"` (no longer needed).
- Remove `#include "IOtosSensor.h"` include if it was only needed for `OdomCtx`.
  (Check: `IOdometer*` type in `OdomCtx` was the reason; `Odometry.h` still needs
  `IOdometer` for `setCtx` — keep that include if `setCtx` is still declared here,
  or remove `setCtx` from `Odometry` and leave it only on `PhysicalStateEstimate`.)

  The cleanest approach: `setCtx` stays on `Odometry` (it's called from
  `PhysicalStateEstimate::setCtx`); `Odometry.h` keeps `#include "IOdometer.h"` or
  the alias shim `#include "IOtosSensor.h"`. `CommandTypes.h` is removed.

In `source/control/Odometry.cpp`:
- Remove all seven parse functions, all seven handler functions, and `getCommands()`.
- Remove `#include "CommandProcessor.h"` (no longer needed if it was only for
  `replyOK`/`replyErr` in the handlers).
- Remove `#include <cstdlib>` / `<cstring>` if only needed for parse atoi/strcmp.
  (Check each include against remaining code; only remove those with zero remaining uses.)

**4. Update Robot.h**

Add `#include "OtosCommands.h"` (or forward-declare if possible).
Add `OtosCommands _otosCommands;` as a value member after `haltController`.

Member declaration order: `OtosCommands` has no dependencies on other `Robot`
members (it is wired post-construction via `setCtx`), so it can be declared last.

**5. Update Robot.cpp constructor**

After the existing `odometry.setCtx(&otos, &state.inputs)` call, add:
```cpp
_otosCommands.setCtx(&otos, &state.inputs);
```

**6. Update Robot::buildCommandTable**

In `source/robot/SystemCommands.cpp` (or wherever `buildCommandTable` is defined):
Replace the line that calls `odometry.getCommands()` with `_otosCommands.getCommands()`.

Grep: `getCommands` in `SystemCommands.cpp` or `Robot.cpp` to find the aggregation
site. It will look something like:
```cpp
// OLD:
auto odoC = robot->odometry.getCommands();
// NEW:
auto odoC = robot->_otosCommands.getCommands();
```

**7. Update vendor-confinement baseline**

After verifying the suite is green, update `tests/_infra/vendor_baseline.txt`:
- Remove any `source/control/EKF.*` entries (EKF moved in T1).
- Add `source/state/` to the scope in `test_vendor_confinement.py` (the grep
  should now check `source/state/` for forbidden tokens; `PhysicalStateEstimate`
  and `Odometry` in `state/` must be clean).
- Commit the updated baseline alongside this ticket.

**8. Verify**

```
python3 build.py --fw-only   # ARM gate
git checkout -- source/robot/DefaultConfig.cpp
uv run --with pytest python -m pytest -q
```

## Acceptance Criteria

- [x] `Odometry` no longer inherits from `Commandable`; `getCommands()` is gone; `OdomCtx` is gone from `Odometry.h`; `CommandTypes.h` is not included from `Odometry.h`.
- [x] `source/app/OtosCommands.{h,cpp}` exist; handlers are verbatim from `Odometry.cpp`.
- [x] All seven verbs (`OI/OZ/OR/OV/OL/OA/OP`) respond correctly; `test_estimator_command_paths.py` passes green.
- [x] `Robot` owns `_otosCommands` value member; `buildCommandTable` aggregates it.
- [x] `source/state/PhysicalStateEstimate.h` transitive include set: no `CommandTypes.h`, no `Commandable`.
- [x] Vendor-confinement grep gate passes with `source/state/` in scope; baseline updated and committed.
- [x] `python3 build.py --fw-only` → 0 errors.
- [x] `uv run --with pytest python -m pytest -q` → ≥ 1997 passed, 0 errors.
- [x] Golden-TLM canary byte-exact; field-pin diff empty.

## Testing

- **Existing tests to run**: full simulation tier — `uv run --with pytest python -m pytest -q`
- **Key fences**: `test_estimator_command_paths.py` (OI/OZ/OR/OV/OL/OA/OP verb behavior), `test_vendor_confinement.py` (baseline now covers source/state/)
- **New tests to write**: none — `test_estimator_command_paths.py` already covers all seven verbs end-to-end
- **Verification command**: `uv run --with pytest python -m pytest -q` (≥ 1997 passed, 0 errors)
