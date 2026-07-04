# Coding Standards

## Units in Identifiers

### Core rule

Identifier names describe the **kind** of quantity a variable holds
(`speed`, `position`, `deadline`), never the **unit** it is measured in.
Units live in a trailing comment, not in the name. This is a durable,
cross-cutting convention — it applies to every identifier renamed by
sprint 071 (C++, `source/`) and sprint 076 (Python, `host/`), and to
any new identifier added after either sprint closes.

The rule covers **every identifier the project controls: fields,
properties, methods, functions, AND parameters.** A parameter named for
its unit is the same violation as a field named for its unit:

```cpp
// WRONG — the parameter names are units, not quantities
void setTwist(float v_mmps, float omega_radps);
void setVelocity(float mm_per_s);

// RIGHT — quantity names; units in the bracketed comment tag
void setTwist(float v_x, float v_y, float omega);  // [mm/s] [mm/s] [rad/s]
void setVelocity(float velocity);                  // [mm/s] signed
```

### Naming the quantity

Name what the value *is*, precisely:

- **`speed`** is a directionless magnitude. **`velocity`** is directed.
  A body twist is never a bare `v` with no direction — it has components
  (`v_x`, `v_y`, `omega`), because a drivetrain may be holonomic (forward
  *and* sideways). If a value truly has no direction, call it `speed`.
- Positions are **`x`** and **`y`** — never `position_1_mm`.
- Frame and axis subscripts are *semantic* qualifiers, not units, and are
  encouraged: `x_b` (x in the body frame), `velocity_b` (body-frame
  velocity).

Why: a unit suffix embedded in a name (`tgtMms`, `read_ms`) drifts
silently the moment the underlying representation changes (e.g. a field
that used to store centidegrees switches to degrees) — nothing forces the
name to be revisited. A separate, greppable comment tag catches every
declaration of a given unit in one shot, independent of how creatively
each declaration was named.

### C++ convention

**Format:** a leading, bracketed unit tag as the *first token* of the
declaration's trailing (or block) comment.

```cpp
// Before
float tgtMms[kWheelCount] = {};  // all-wheel speed targets, mm/s

// After
float tgtSpeed[kWheelCount] = {};  // [mm/s] all-wheel speed targets
```

### Python convention

The same tag format applies to Python, using `#` instead of `//`. Sprint
076 applied this convention across `host/`.

```python
# Before
def send(self, cmd: str, read_ms: int = 500) -> dict: ...

# After (sprint 076)
def send(self, cmd: str, read_timeout: int = 500) -> dict:  # [ms]
    ...
```

### Unit vocabulary

Reuse whatever unit text the surrounding prose already uses elsewhere in
the file (`mm`, `mm/s`, `mm/s²` or `mm/s^2`, `deg`, `deg/s`, `deg/s²`,
`ms`, `us`, `%`, `Hz`, `rad`, `rad/s`, `rad²/s`, `mm²/s`) — do not invent a
second vocabulary. Grep for the unit already used in the block comment
above the field being renamed and match its spelling.

### Compound and derived units

Compound units are written as a single bracketed tag exactly as they
appear in the vocabulary above, e.g. `// [mm/s]`, `// [mm/s²]` (or
`// [mm/s^2]`), `// [deg/s]`, `// [rad²/s]`, `// [mm²/s]`.

Derived-unit names — identifiers whose old name encoded a *ratio* or
*rate* between two units — are renamed to what the quantity *is*, with
the unit moved into the comment, not simply stripped of a trailing
suffix. For example:

```cpp
// Before
float mmPerDegL;  // wheel linear travel per motor-shaft degree of rotation, left
float mmPerDegR;  // wheel linear travel per motor-shaft degree of rotation, right

// After
float wheelTravelCalibL;  // [mm/deg] wheel linear travel per motor-shaft degree of rotation
float wheelTravelCalibR;  // [mm/deg] wheel linear travel per motor-shaft degree of rotation
```

Simply stripping `Deg` from `mmPerDegL` would leave `mmPerL`, which still
embeds `mm` and reads worse, not better — the rule is to name the
quantity, not to truncate the old name. The mecanum siblings
(`mmPerDegFR/FL/BR/BL` → `wheelTravelCalibFR/FL/BR/BL`) follow the same
pattern.

### Dimensionless fields carry no tag

Dimensionless, boolean, and enum fields never had a unit suffix and get
no tag — there is nothing to disambiguate. Examples: `rotationalSlip`,
`kFF`, `velKp`, `odomUpsideDown`, `drivetrain`.

### Ambiguity-resolution rule

Where stripping a unit suffix would collide two previously-distinguished
names — for example a `Mm`-suffixed float position vs. a raw-ticks
integer counterpart that would otherwise both become `position` — choose
a descriptive replacement for the *kind* of quantity rather than a bare
strip that produces a collision or an ambiguous bare word. In that
example, prefer `positionLinear` (the mm-scaled float) vs. `positionTicks`
(the raw-ticks integer) over a bare strip of either name.

### Grep-ability

The tag's fixed leading position means every declaration of a given unit
can be found independent of identifier spelling:

```
grep -rn "// \[mm/s\]" source/
```

and, for `host/` (sprint 076):

```
grep -rn "# \[ms\]" host/
```

This is the convention's whole purpose: a reviewer or future maintainer
can enumerate every quantity of a given unit without knowing in advance
how each one was named.

### Wire/serialized identifiers are excluded

This convention governs **identifier names in source code only**. It
does not apply to, and does not rename, any wire or serialized key
string, including:

- `SET`/`GET`/`SIMSET`/`SIMGET` wire key strings (e.g. the first argument
  of a `ConfigRegistry.cpp` `CFG_*` row or a `SimCommands.cpp`
  `kSimRegistry[]` row).
- `TLM`/`SNAP` field-name tokens (e.g. `enc=`, `pose=`, `otos=`, `vel=`,
  `twist=`, `line=`, `color=`, `ekf_rej=`, `wedge=`, `encpose=`).
- JSON config keys in `data/robots/*.json` and the
  `host/robot_radio/config/robot_config.py` pydantic field names that
  mirror them 1:1.

These strings are serialized/persisted or cross a wire boundary; renaming
one is a protocol or data-format change, not a code-readability change,
and is out of scope for this convention regardless of whether the string
happens to also look like a unit-suffixed identifier. They stay stable
even when the internal C++ (or, later, Python) identifier next to them is
renamed under this convention.

Also excluded, for the same reason (the text is wire-visible even though
it lives in a plain string literal, not a wire-key table): the `"usage: HALT
POS <x_mm> <y_mm> <radius_mm>"` error-reply string in
`source/commands/SystemCommands.cpp`'s `HALT POS` handler — it is emitted
verbatim to the client on a bad-argument `ERR`, so changing the placeholder
text inside it is a wire-format change like any other reply string.

### External/vendor function names are excluded

`system_timer_current_time_us()` (declared by the CODAL/microbit vendor SDK,
called throughout `source/hal/real/Motor.cpp`, `source/com/SerialPort.cpp`,
`source/com/I2CBus.cpp`, and `source/robot/WedgeTest.cpp`) is a vendor
library function, not a project identifier — it is not declared anywhere in
`source/`, and renaming it is not possible without forking the vendor SDK.
It is excluded the same way `extern "C"` ctypes-boundary function names are:
the convention governs names this project controls, not names imposed by an
external interface. Every *call site's own local variable* that stores or
derives from its return value (e.g. a cached "now" timestamp or a deadline)
has been renamed per the normal convention — only the vendor function's own
name is excluded.

## Naming Case (CamelCase — Google's case rules overridden)

The project follows the Google C++ Style Guide — vendored with inline
override banners at `docs/reference/google-cppguide.html` — EXCEPT its
naming-case rules, which are replaced by this stakeholder-set rule
(2026-07-04):

> Use CamelCase. **Capitalize the first letter, including all letters in
> an acronym, in a class, struct, protocol, or namespace name.**
> **Lower-case the first letter, including all letters in an acronym, in
> a variable or function name.**

Explicitly: **we are not capitalizing function names** — functions and
methods are never PascalCase. Variable and function names always start
lowercase.

```cpp
namespace Hal {                       // namespace: UpperCamelCase

class Motor {                         // type: UpperCamelCase
 public:
  void configure(const msg::MotorConfig& config);
  void apply(const msg::MotorCommand& command);
  void tick(uint32_t now);            // [ms] lowerCamelCase functions
  void setVelocity(float velocity);   // [mm/s] signed
  float velocity() const;             // [mm/s] signed
 private:
  float lastPosition_;                // [mm] member: trailing underscore
};

}  // namespace Hal
```

Details:
- Acronyms follow the case of the position: `HTTPServer` (type),
  `httpRequest` (variable/function).
- Class data members keep Google's trailing underscore (`lastPosition_`).
- Mathematical subscripts keep their underscore (`v_x`, `x_b`) — they are
  notation, not word separation.
- Filenames stay snake_case; `kConstant` constants stay.
- Legacy code (`source_old`, verbatim copies) is already lowerCamelCase
  for functions and stays as-is until touched. Generated code
  (`source/messages/*.h`) is exempt.
