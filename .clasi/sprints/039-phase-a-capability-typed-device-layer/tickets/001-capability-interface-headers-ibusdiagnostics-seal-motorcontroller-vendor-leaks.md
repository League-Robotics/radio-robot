---
id: "001"
title: "Capability interface headers + IBusDiagnostics + seal MotorController vendor leaks"
status: open
use-cases:
  - SUC-039-001
  - SUC-039-002
depends-on: []
github-issue: ""
issue: ""
completes_issue: false
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# T1 — Capability interface headers + IBusDiagnostics + seal MotorController vendor leaks

## Description

Introduce the seven `source/io/capability/` interface headers as the canonical device
interface layer. Simultaneously introduce the `IBusDiagnostics` interface plus a
`MotorBusDiagnostics` adapter, and use them to remove `#include "MicroBit.h"` and the
`I2CBus` forward declaration from `MotorController.h`. This ticket is purely additive
plus two small edits in `MotorController.*` — no existing consumers are modified; alias
shims in the old `hal/` headers ensure everything continues to compile.

After this ticket: `source/io/capability/` exists with seven headers; `MotorController.h`
has no MicroBit or I2CBus reference; vendor-confinement baseline shrinks by the
`MotorController` entries; both builds (host + `build.py` if ARM toolchain present)
are green; simulation tier is green.

**Host-verifiable:** Yes. The host build includes `MotorController.h` (and would fail
to compile if `MicroBit.h` is pulled in under `HOST_BUILD`).

## Approach

### Step 1 — Create `source/io/capability/` directory and value types

Create `source/io/capability/Pose2D.h` (or include the value types inline in
`IOdometer.h`):

```cpp
// source/io/capability/Pose2D.h
#pragma once
#include <stdint.h>
struct Pose2D    { float x, y, h; };           // mm, mm, rad
struct BodyTwist { float v_mmps, omega_rads; };  // mm/s, rad/s
struct BodyAccel { float ax_mmps2, ay_mmps2; };  // mm/s^2
```

### Step 2 — Create the seven capability headers

**`IVelocityMotor.h`** — drive-wheel capability:
```cpp
// source/io/capability/IVelocityMotor.h
#pragma once
#include <stdint.h>
struct IPositionMotor;

class IVelocityMotor {
public:
    virtual ~IVelocityMotor() = default;
    virtual void   begin() {}
    virtual void   tick(uint32_t now_ms) {}             // split-phase advance (T2 populates)
    virtual void   setOutput(int8_t pct) = 0;           // signed %, +forward
    virtual float  positionMm() const = 0;              // cumulative mm since resetPosition
    virtual float  velocityMmps() const = 0;            // last-tick wheel speed
    virtual void   resetPosition() = 0;                 // zero accumulator
    virtual void   setNeutralMode() {}                  // coast to stop (default no-op)
    // RTTI-free secondary capability — returns non-null if impl supports position moves
    virtual IPositionMotor* asPositionMotor() { return nullptr; }
    // Atomic snapshot outside the control tick (~8 ms). Default: returns positionMm().
    virtual float  positionMmAtomic() const { return positionMm(); }
};
```

**`IPositionMotor.h`** — position-move capability (Servo + Motor position moves):
```cpp
// source/io/capability/IPositionMotor.h
#pragma once
#include <stdint.h>
class IPositionMotor {
public:
    virtual ~IPositionMotor() = default;
    virtual void     setAngleDeg(uint16_t deg, uint8_t mode) = 0;
    virtual uint16_t currentAngleDeg() const = 0;
};
```

**`IOdometer.h`** — odometry capability (full IOtosSensor API, RobotConfig-sealed):
```cpp
// source/io/capability/IOdometer.h
#pragma once
#include <stdint.h>
#include "Pose2D.h"

// Forward-declare Sensor base (keeps Sensor.h out of the capability layer)
struct Sensor;

class IOdometer {
public:
    virtual ~IOdometer() = default;
    virtual bool is_initialized() const = 0;
    virtual bool readTransformed(Pose2D& poseOut, float headingRad = 0.0f) const = 0;
    virtual bool readVelocityTransformed(BodyTwist& velOut, float headingRad = 0.0f) const = 0;
    virtual bool readStatus(uint8_t& out) const = 0;
    virtual bool lastReadOk() const = 0;
    virtual BodyAccel readAccelTransformed() const = 0;
    virtual void init() = 0;
    virtual void calibrateImu(uint8_t samples) = 0;
    virtual void resetTracking() = 0;
    virtual void getPositionRaw(int16_t& x, int16_t& y, int16_t& h) const = 0;
    virtual void setPositionRaw(int16_t x, int16_t y, int16_t h) = 0;
    virtual void setWorldPose(float x_mm, float y_mm, float h_rad) {}
    virtual int8_t getLinearScalar() const = 0;
    virtual void   setLinearScalar(int8_t val) = 0;
    virtual int8_t getAngularScalar() const = 0;
    virtual void   setAngularScalar(int8_t val) = 0;
};
```

**`IBusDiagnostics.h`** — bus health capability (new):
```cpp
// source/io/capability/IBusDiagnostics.h
#pragma once
#include <stdint.h>
class IBusDiagnostics {
public:
    virtual ~IBusDiagnostics() = default;
    virtual uint32_t errorCount() const = 0;
    virtual uint32_t reentryViolations() const = 0;
    virtual uint32_t lastError() const = 0;
};
```

**`ILineSensor.h`**, **`IColorSensor.h`**, **`IPortIO.h`** — copy the existing
`source/hal/ILineSensor.h`, `IColorSensor.h`, `IPortIO.h` content into
`source/io/capability/`. These interfaces are already clean; no signature changes needed.

### Step 3 — Alias shims in source/hal/

Update each old `hal/` interface header to become a shim (add at the TOP, before
existing content, leaving existing content in place so it remains a valid header):

```cpp
// source/hal/IMotor.h — add at top, before class definition:
#include "io/capability/IVelocityMotor.h"
using IMotor = IVelocityMotor;
// ... (keep existing class definition for the transition period;
//      it becomes dead code since using IMotor = IVelocityMotor now)
```

Wait — the correct approach is: REPLACE the class definition in `IMotor.h` with the
alias, and have it include the capability header. The old class body is superseded.

```cpp
// source/hal/IMotor.h (new content — shim only)
#pragma once
#include "io/capability/IVelocityMotor.h"
using IMotor = IVelocityMotor;
```

Similarly:
- `source/hal/IServo.h` → includes `IPositionMotor.h`; `using IServo = IPositionMotor;`
- `source/hal/IOtosSensor.h` → includes `IOdometer.h` and defines type aliases:
  `using IOtosSensor = IOdometer; using OtosPose = Pose2D; using OtosVelocity = BodyTwist; using OtosAccel = BodyAccel;`

**CAUTION:** `IOtosSensor.h` currently defines the `OtosPose`, `OtosVelocity`, `OtosAccel`
structs and the `IOtosSensor` class. After the shim:
- `Pose2D` / `BodyTwist` / `BodyAccel` are defined in `Pose2D.h` (included via `IOdometer.h`).
- `OtosPose` / `OtosVelocity` / `OtosAccel` are aliases in the shim.
- Any code that declares `OtosPose p;` continues to compile (alias → `Pose2D`).
- Any code that includes `IOtosSensor.h` and uses `IOtosSensor` compiles (alias → `IOdometer`).

**`ILineSensor.h`**, **`IColorSensor.h`**, **`IPortIO.h`** in `hal/` become shims that
include the capability versions. Since the interface content is identical, these can
just be replaced with includes + an empty body (or left as-is and the capability headers
are forward-copies that callers can include directly once they are ready).

### Step 4 — `MotorBusDiagnostics` adapter (source/io/real/ or source/hal/ for now)

During T1–T4 the files are still under `source/hal/` (the directory rename is T5).
Create the adapter in `source/hal/`:

**`source/hal/MotorBusDiagnostics.h`**:
```cpp
#pragma once
#include "io/capability/IBusDiagnostics.h"
class I2CBus;
class MotorBusDiagnostics : public IBusDiagnostics {
public:
    explicit MotorBusDiagnostics(I2CBus& bus);
    uint32_t errorCount() const override;
    uint32_t reentryViolations() const override;
    uint32_t lastError() const override;
private:
    I2CBus& _bus;
};
```

**`source/hal/MotorBusDiagnostics.cpp`**:
```cpp
#include "MotorBusDiagnostics.h"
#include "I2CBus.h"
MotorBusDiagnostics::MotorBusDiagnostics(I2CBus& bus) : _bus(bus) {}
uint32_t MotorBusDiagnostics::errorCount() const { return _bus.errorCount(); }
uint32_t MotorBusDiagnostics::reentryViolations() const { return _bus.reentryViolations(); }
uint32_t MotorBusDiagnostics::lastError() const { return _bus.lastError(); }
```

Check `I2CBus.h` for the actual method names — adapt if the I2CBus API uses different
names. The getter values reported in `EVT enc_wedged` must be byte-identical to what
was reported before.

### Step 5 — Update NezhaHAL

Add `MotorBusDiagnostics _busDiag;` as a value member in `NezhaHAL` (constructed from
`_bus`). Add `IBusDiagnostics& busDiagnostics()` accessor to the public interface.

### Step 6 — Update MotorController

In `MotorController.h`:
- Remove `#ifndef HOST_BUILD` / `#include "MicroBit.h"` / `#endif` (lines 2-4).
- Remove `class I2CBus;` forward declaration.
- Change `void setI2CBus(I2CBus* bus)` → `void setBusDiagnostics(IBusDiagnostics* diag)`.
- Change `I2CBus* _i2cBus;` → `IBusDiagnostics* _busDiag;`.
- Add `#include "io/capability/IBusDiagnostics.h"` at top.

In `MotorController.cpp`:
- Remove `#include "I2CBus.h"`.
- Replace all accesses to `_i2cBus->errorCount()` / `_i2cBus->reentryViolations()` etc.
  with `_busDiag->errorCount()` etc.
- Guard with null check: `if (_busDiag)`.

### Step 7 — Update main.cpp

Replace:
```cpp
robot.motorController.setI2CBus(&hardware.bus());
```
with:
```cpp
robot.motorController.setBusDiagnostics(&hardware.busDiagnostics());
```

### Step 8 — Update vendor-confinement baseline

Remove from `tests/_infra/vendor_baseline.txt` the entries for `MotorController.h` and
`MotorController.cpp`. Run the vendor-confinement test to confirm zero new hits appear.

## Files to Create

- `source/io/capability/Pose2D.h` — `Pose2D`, `BodyTwist`, `BodyAccel` structs
- `source/io/capability/IVelocityMotor.h`
- `source/io/capability/IPositionMotor.h`
- `source/io/capability/IOdometer.h`
- `source/io/capability/IBusDiagnostics.h`
- `source/io/capability/ILineSensor.h` (copy of hal version)
- `source/io/capability/IColorSensor.h` (copy of hal version)
- `source/io/capability/IPortIO.h` (copy of hal version)
- `source/hal/MotorBusDiagnostics.h`
- `source/hal/MotorBusDiagnostics.cpp`

## Files to Modify

- `source/hal/IMotor.h` — replace with alias shim
- `source/hal/IServo.h` — replace with alias shim
- `source/hal/IOtosSensor.h` — replace with alias shim + type aliases
- `source/hal/ILineSensor.h` — replace with shim (include capability version)
- `source/hal/IColorSensor.h` — replace with shim (include capability version)
- `source/hal/IPortIO.h` — replace with shim (include capability version)
- `source/hal/NezhaHAL.h` — add `MotorBusDiagnostics _busDiag;` member + `busDiagnostics()` accessor
- `source/hal/NezhaHAL.cpp` — construct `_busDiag(_bus)` in initializer list
- `source/control/MotorController.h` — remove MicroBit include + I2CBus forward decl; add IBusDiagnostics
- `source/control/MotorController.cpp` — remove I2CBus include; use IBusDiagnostics
- `source/main.cpp` — `setI2CBus` → `setBusDiagnostics`
- `tests/_infra/sim/CMakeLists.txt` — add `source/hal/MotorBusDiagnostics.cpp` to source list (it's not globbed automatically since it's a new file in hal/)
- `tests/_infra/vendor_baseline.txt` — remove MotorController.* entries

## Acceptance Criteria

- [ ] `source/io/capability/` directory exists with exactly 8 headers (7 capability + `Pose2D.h`).
- [ ] `source/io/capability/IBusDiagnostics.h` defines `errorCount()`, `reentryViolations()`, `lastError()`.
- [ ] `source/hal/MotorBusDiagnostics.{h,cpp}` implement `IBusDiagnostics` by forwarding to `I2CBus`.
- [ ] `MotorController.h` contains no `#include "MicroBit.h"` and no `I2CBus` forward declaration.
- [ ] `MotorController.cpp` contains no `#include "I2CBus.h"`.
- [ ] `NezhaHAL` exposes `IBusDiagnostics& busDiagnostics()`.
- [ ] `main.cpp` calls `motorController.setBusDiagnostics(...)`.
- [ ] `tests/_infra/vendor_baseline.txt` no longer contains `MotorController.h` or `MotorController.cpp` entries.
- [ ] Vendor-confinement canary passes (run `uv run --with pytest python -m pytest tests/simulation/unit/test_vendor_confinement.py -q`).
- [ ] Simulation tier green: `uv run --with pytest python -m pytest -q` — all tests pass, count >= 1957.
- [ ] **Host build only** — no ARM toolchain required for this ticket.

## Testing Plan

- Run `uv run --with pytest python -m pytest -q` (full simulation tier).
- Run `uv run --with pytest python -m pytest tests/simulation/unit/test_vendor_confinement.py -v` to confirm baseline shrinkage.
- Run `uv run --with pytest python -m pytest tests/simulation/unit/test_default_config_pin.py -v` to confirm config unchanged.
- If ARM toolchain is present: `python3 build.py` must succeed for both REAL and SIM builds.
- **ARM-only files changed in this ticket:** `NezhaHAL.h/.cpp`, `main.cpp` — verify textually that constructor calls and method bindings are consistent with the new interface.
