# Naming rules (stakeholder-mandated — apply to ALL new/edited code and examples)

1. **No units in ANY identifier** — method, function, field, property, or
   parameter. `speed`, never `speed_mms`; `setVelocity(float velocity)`, never
   `setVelocity(float mm_per_s)`. Units go in a leading bracketed tag as the
   first token of the trailing comment: `// [mm/s]` (Python: `# [ms]`). Full
   convention and exclusions (wire keys, vendor names): `.claude/rules/coding-standards.md`.

2. **Name the quantity, precisely.** `speed` = directionless magnitude;
   `velocity` = directed. A twist always has components (`v_x`, `v_y`, `omega`) —
   drivetrains may be holonomic; never a bare directionless `v` for a twist.
   Positions are `x`, `y` (never `position_1_mm`). Frame/axis subscripts are
   semantic, not units, and are fine: `x_b` (body frame), `velocity_b`.

3. **CamelCase, Google's case rules overridden.** The project follows the Google
   C++ Style Guide (condensed, with the overrides applied inline, at
   `docs/reference/google-cppguide-condensed.md` — the operative reference)
   EXCEPT naming case:
   - **Capitalize the first letter, including all letters in an acronym, in a
     class, struct, protocol, or namespace name**: `Motor`, `HTTPServer`,
     `namespace Hal`.
   - **Lower-case the first letter, including all letters in an acronym, in a
     variable or function name**: `tick()`, `setVelocity()`, `httpRequest`,
     `leftObs`. Function/method names NEVER start with an uppercase letter —
     we are explicitly not using Google's PascalCase functions.
   - Class data members keep the trailing underscore (`lastPosition_`).
     Mathematical subscripts keep their underscore (`v_x`, `x_b`) — notation,
     not word separation. Filenames stay snake_case. Generated
     `source/messages/*` files are never hand-edited, but the generator
     (`scripts/gen_messages.py`) must emit conforming API — its trivial
     `get_*` accessors are slated for removal after sprint 077
     (`clasi/issues/remove-generated-get-accessors.md`).

4. **Edge (command-out) types are named by their endpoints**:
   `<Producer>To<Consumer><Payload>`, payload ∈ {Command, Statement} — e.g.
   `DrivetrainToMotorCommand` for what the Drivetrain sends its wheel Motors
   (payload=Command: a parsed `msg::*Command`), or
   `CommunicatorToCommandProcessorStatement` for the raw wire line the
   Communicator hands the processor (payload=Statement: one unparsed wire
   line — verb, args, kv pairs, correlation id — before it becomes a
   command). Never name an edge type by mechanism or moment (`…Tick`,
   `…Output`, `…Batch`): the name must say what it is and who it is between.
   Long is fine; ambiguous is not.

5. When you encounter a violating name — proposed, inherited, or in your own
   draft/example code — rename toward these rules. Never propagate a violation,
   including in plans, docs, and example snippets.
