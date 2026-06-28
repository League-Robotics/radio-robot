---
id: '001'
title: Vendor cmon-pid as float-adapted header into libraries/cmon-pid/
status: done
use-cases:
- SUC-001
depends-on: []
github-issue: ''
issue: consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md
completes_issue: false
---

# Vendor cmon-pid as float-adapted header into libraries/cmon-pid/

## Description

Create `libraries/cmon-pid/` containing a float-adapted copy of the upstream
cmon-pid header (https://github.com/corraid/cmon-pid, BSD-2-Clause). The
upstream library hard-codes `double` throughout; the Cortex-M4F only has a
single-precision FPU so `double` is soft-emulated. Mechanically convert every
`double` occurrence in the header to `float` (no logic change). Copy the
upstream LICENSE file unchanged.

This establishes the vendoring infrastructure that Sprint 050 (TinyEKF) will
reuse. The header must satisfy the hard constraints: no heap, no STL, no
exceptions, no RTTI, compiles in both the ARM firmware build and the host sim
build.

## Acceptance Criteria

- [x] `libraries/cmon-pid/cmon-pid.h` exists with a preamble comment documenting
      the upstream source URL, commit/version, license, and the `double`->`float`
      adaptation rationale.
- [x] `libraries/cmon-pid/LICENSE` exists (BSD-2-Clause, verbatim from upstream).
- [x] The vendored header contains zero `double` occurrences (grep confirms).
- [x] The vendored header contains no `#include <...>` STL headers, no `new`,
      no `malloc`, no virtual functions, no exceptions.
- [x] The header compiles cleanly in a minimal host-build smoke test (included
      by a `.cpp` file that instantiates `cmon_pid::ParallelPid` and
      `cmon_pid::backcalculation_t`).

## Implementation Plan

### Approach

1. Fetch the upstream `cmon-pid.h` from https://github.com/corraid/cmon-pid
   (or vendor the pinned commit used for the feasibility review).
2. Create `libraries/cmon-pid/` directory.
3. Copy `cmon-pid.h` into the directory; prepend a preamble comment block:

   ```
   // VENDORED: cmon-pid — https://github.com/corraid/cmon-pid
   // Upstream commit: <hash>  License: BSD-2-Clause
   // Adaptation: all 'double' replaced with 'float' for Cortex-M4F FPU
   // compatibility. No other changes. To apply an upstream patch, diff
   // against the original and carry forward the double->float substitution.
   ```

4. Run a sed/replace over the header to convert all `double` to `float`. Be
   careful to only replace the type keyword, not partial matches (e.g., search
   for `\bdouble\b`).
5. Copy the upstream LICENSE file into `libraries/cmon-pid/LICENSE`.
6. Verify: `grep -c '\bdouble\b' libraries/cmon-pid/cmon-pid.h` must return 0.

### Files to create

- `libraries/cmon-pid/cmon-pid.h` (float-adapted, with preamble comment)
- `libraries/cmon-pid/LICENSE` (verbatim BSD-2-Clause)

### Files to modify

None in this ticket. Build-path wiring is covered by ticket 002.

### Testing plan

This ticket does not touch any source files. The confinement test does not scan
`libraries/`, so no baseline update is needed.

Smoke-verify the header is syntactically correct by confirming it compiles when
included from a minimal `.cpp` in the host sim build after ticket 002 wires the
include path. Full functional validation is in ticket 005.

Canonical test command (no new failures expected at this stage; the header is
not yet included by any source file):

```
uv run --with pytest python -m pytest tests/simulation -q
```

Expected: same result as baseline (exactly 2 pre-existing failures unchanged).

### Documentation

No doc changes needed. The preamble comment in the vendored header is the
authoritative provenance record.
