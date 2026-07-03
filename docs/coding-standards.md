# Coding Standards

## Units in Identifiers

### Core rule

Identifier names describe the **kind** of quantity a variable holds
(`speed`, `position`, `deadline`), never the **unit** it is measured in.
Units live in a trailing comment, not in the name. This is a durable,
cross-cutting convention — it applies to every identifier renamed by
sprint 071 (C++, `source/`) and, in the future, sprint 072 (Python,
`host/`), and to any new identifier added after either sprint closes.

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

### Python convention (forward reference — sprint 072)

The same tag format applies to Python, using `#` instead of `//`. This
convention is documented here for forward reference only — it is **not**
applied to any `host/` file by sprint 071 or by this document's own
creation; sprint 072 is the sprint that will apply it to `host/`.

```python
# Before
def send(self, cmd: str, read_ms: int = 500) -> dict: ...

# After (sprint 072 — not applied to any host/ file by this sprint)
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

and, once sprint 072 applies the convention to `host/`:

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
