# Where the firmware-tree design doc went

`src/firm` is a declared design-doc-set source root
(`.clasi/config.yaml`'s `sources:`), so a `DESIGN.md` sitting directly
inside it — rather than inside one of its subsystem children
(`app/`, `com/`, `config/`, `devices/`, `kinematics/`, `messages/`,
`types/`) — has no home to validate against and would be flagged as an
orphaned doc. The firmware-tree overview (purpose, the single-loop
architecture, the directory map, cross-cutting constraints, dependency
diagram) now lives in the system-level design doc:
[`docs/design/design.md`](../../docs/design/design.md).

Each subsystem's own design doc is still co-located exactly where you'd
expect: `src/firm/app/DESIGN.md`, `src/firm/com/DESIGN.md`,
`src/firm/config/DESIGN.md`, `src/firm/devices/DESIGN.md`,
`src/firm/kinematics/DESIGN.md`, `src/firm/messages/DESIGN.md`,
`src/firm/types/DESIGN.md`.
