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

4. **Vocabulary: `command` (wire-inbound) vs `message` (internal).** Things
   arriving over the radio/serial channel are **commands**; internal typed
   representations are **messages** (`msg::*`). There is no third
   "statement" category — that term was removed sprint-wide (2026-07-07,
   reversing sprint 079's "statements rename"; see sprint 088's
   `architecture-update.md` Decision 1).

   **Edge (command-out) types are named by their endpoints**:
   `<Producer>To<Consumer><Payload>`, payload ∈ {Command} — e.g.
   `Hal::DrivetrainToHardwareCommand` for what the Drivetrain sends its wheel
   Motors (a parsed `msg::*Command`), or
   `Subsystems::CommunicatorToCommandProcessorCommand` for the raw wire line
   the Communicator hands the processor (one unparsed wire line — verb,
   args, kv pairs, correlation id — before it is dispatched). Never name an
   edge type by mechanism or moment (`…Tick`, `…Output`, `…Batch`): the name
   must say what it is and who it is between. Long is fine; ambiguous is
   not.

   **The "Command" overload is deliberate, not a naming bug.** "Command" now
   names two different shapes depending on which edge you're looking at:
   `Subsystems::CommunicatorToCommandProcessorCommand` (a raw, unparsed wire
   line — `char line[256]` + a return-channel field) vs. every other
   pre-existing `...Command`-suffixed edge (e.g.
   `Hal::DrivetrainToHardwareCommand`, `DrivetrainToMotorCommand`), which
   carries a *parsed* `msg::*Command` struct. This overload was weighed
   explicitly (architecture-update.md Decision 1) against renaming the
   wire-inbound edge to something overload-free (e.g. `...Line`/`...Raw...`)
   and against renaming every pre-existing internal `...Command` edge to
   `...Message` for full consistency — both rejected: the former would put
   the wire-inbound edge at odds with the surrounding vocabulary
   (`CommandProcessor`, `CommandRouter`, `CommandDescriptor`, "command
   table"), the latter roughly doubles the rename for a consistency gain
   that blocks no acceptance criterion. Do **not** rename a pre-existing
   `...Command`-suffixed internal edge type or any `msg::*Command` message
   type to chase full `Command`/`Message` consistency — that is a
   deliberately deferred future issue (architecture-update.md Step 7, Item
   4), not a defect. The raw-vs-parsed distinction is carried structurally
   (a raw wire line has a `char line[]` buffer and a doc comment saying so;
   a parsed message has typed fields), not lexically.

5. When you encounter a violating name — proposed, inherited, or in your own
   draft/example code — rename toward these rules. Never propagate a violation,
   including in plans, docs, and example snippets.
