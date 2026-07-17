---
root: ../DESIGN.md
---

# types/ — Protocol Constants and Version Plumbing

**Owner:** Eric Busboom · **Last reviewed:** 2026-07-16 · **Status:** vestigial (see §6)

---

## 1. Purpose

`types/` holds two unrelated things that predate the single-loop rebuild and
never got a real home: text-protocol tag constants left over from the
pre-binary-cutover command set, and the firmware-version generation seam
(`FIRMWARE_VERSION` / `PROTO_VERSION`, fed by `version_generated.h`). It is
not a subsystem in the sense the rest of `src/firm` uses the word — it is a
leftover grab-bag flagged in the root doc's §3/§6 as needing an audit. This
doc records that audit's findings.

## 2. Orientation

`protocol.h` is a single header, included by nothing in the live tree (see
§6). It declares, in order: six `PROTO_TAG_*` reply-tag string constants,
`PROTO_VERSION`, the `FIRMWARE_VERSION` string (sourced from the generated,
git-ignored `version_generated.h`, falling back to `"0.0.0-dev"` when that
file is absent), the `ReplyFn`/`ReplyCtx` reply-sink types, and the `KVPair`
key=value token struct. `version_generated.h` is emitted by
`scripts/gen_version.py` (run from `build.py`'s codegen step, alongside
`gen_messages.py`) and is never hand-edited — see the root doc's
"generated files are never hand-edited" invariant.

## 3. Constraints and Invariants

- **`version_generated.h` is generated, git-ignored, and never hand-edited.**
  `scripts/gen_version.py` overwrites it at every build from `pyproject.toml`.
  Edits here are silently destroyed by the next build.
- **The `__has_include` fallback must stay.** `protocol.h` guards its include
  of `version_generated.h` with `#if __has_include(...)` and defines
  `FIRMWARE_VERSION_STR "0.0.0-dev"` if it's missing. This keeps clangd (which
  doesn't run codegen) and any ad-hoc codegen-less compile building. Deleting
  the fallback because "the generator always runs" breaks IDE tooling.
- **Wire tag strings are frozen.** `PROTO_TAG_*` and the `DEVICE:NEZHA2:...`
  banner format are wire surface, exempt from the naming-convention rename
  sweep (`.claude/rules/coding-standards.md`) — see also §6 on whether these
  particular constants still matter.

## 4. Design

**Version generation pipeline.** The firmware needs to report a build
version over the wire without a hand-edited constant silently drifting (this
happened: `FIRMWARE_VERSION` sat at `0.20260704.6` while `pyproject.toml`
advanced past it). `gen_version.py` reads the canonical version out of the
root `pyproject.toml` and writes it as `#define FIRMWARE_VERSION_STR "..."`
into `version_generated.h`, which `protocol.h` includes. The file is only
rewritten when its content changes, to avoid needless rebuilds. Because
codegen doesn't run under clangd or a bare compile, `protocol.h`'s
`__has_include` guard falls back to a literal `"0.0.0-dev"` so those builds
still succeed — the fallback string is a marker, not a real version, and
should never appear in a wire reply from a real build.

**Everything else in the file is inert today** (see §6) — there is no
control flow to describe.

## 5. Interfaces

### Exposes
- **`PROTO_TAG_OK/ERR/EVT/TLM/CFG/ID`, `PROTO_VERSION`, `FIRMWARE_VERSION`,
  `ReplyFn`/`ReplyCtx`, `KVPair`:** declared, header-only, no current
  callers in `src/firm` or `src/sim` (verified by repo-wide grep — see §6).
  Any future consumer would take these as-is; no contract beyond the C++
  types themselves.

### Consumes
- **`pyproject.toml` (via `scripts/gen_version.py`):** canonical version
  string, at build time — see root doc's "Build-time generators".

## 6. Open Questions / Known Limitations

- **`protocol.h` is currently included by nothing.** A repo-wide grep
  (`grep -rn "PROTO_TAG_\|ReplyCtx\|ReplyFn\|FIRMWARE_VERSION\|PROTO_VERSION\|KVPair" src --include='*.cpp' --include='*.h'`)
  finds real consumers only in `src/archive/source_old/` (the pre-rebuild
  tree, deleted from the live build in sprints 102–107) — `Protocol.h`,
  `CommandTypes.h`, `Superstructure.h`, `CommandProcessor.*`,
  `MotionCommands.cpp`, etc. Nothing under `src/firm/app`, `src/firm/com`,
  `src/firm/devices`, `src/firm/messages`, `src/firm/config`, or `src/sim`
  includes `types/protocol.h` at all. `main.cpp`'s banner
  (`DEVICE:NEZHA2:robot:<name>:<serial>`) is hand-formatted from name and
  serial only; it does not use `FIRMWARE_VERSION` or `PROTO_VERSION`.
  `App::Comms::pumpTransport` answers `HELLO`/`PING` with the literal
  strings `"OK pong"` and the caller-supplied banner, not `PROTO_TAG_OK`.
- **`PROTO_TAG_*` predate the binary cutover.** They belong to the old
  text-tag reply format (`OK`/`ERR`/`EVT`/`TLM`/`CFG`/`ID` as a leading
  token). The current wire protocol is the binary-armored envelope codec
  (`msg::ReplyEnvelope` with an ok/err/tlm discriminant) — see root doc §4,
  "Command plane." These constants have no counterpart need in that scheme.
- **`ReplyFn`/`ReplyCtx`/`KVPair` are artifacts of the deleted
  dispatch-table architecture** (per-command handlers taking a reply
  sink + parsed kv-pairs), not the current single-loop design where
  `App::Comms::sendReply()` takes a typed `msg::ReplyEnvelope` directly and
  there is no generic kv-pair command parser.
- **Recommendation (not actioned here):** this ticket is documentation-only
  and changes no code. A follow-up cleanup ticket should decide whether to
  delete the unused declarations (`PROTO_TAG_*`, `ReplyFn`, `ReplyCtx`,
  `KVPair`) outright, keeping only the version-generation machinery
  (`PROTO_VERSION`, `FIRMWARE_VERSION`, the `__has_include` fallback) which
  is the one piece with a real, if currently unwired, purpose. File as a
  `clasi/issues/` item rather than deciding it inline.
