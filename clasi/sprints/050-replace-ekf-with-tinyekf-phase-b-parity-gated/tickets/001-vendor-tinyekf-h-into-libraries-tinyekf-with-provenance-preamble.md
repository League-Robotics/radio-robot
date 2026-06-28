---
id: '001'
title: Vendor tinyekf.h into libraries/tinyekf with provenance preamble
status: open
use-cases: ["SUC-001"]
depends-on: []
github-issue: ''
issue: consolidate-control-code-onto-vendored-libraries-cmon-pid-tinyekf.md
completes_issue: false
---

# Vendor tinyekf.h into libraries/tinyekf with provenance preamble

## Description

Copy `tinyekf.h` and `LICENSE` from the local upstream clone into
`libraries/tinyekf/`, adding a provenance preamble to the header. This follows
the exact same vendoring pattern used for `libraries/cmon-pid/` in Sprint 049.

**Local source path (do NOT fetch from the internet):**
```
/private/tmp/claude-501/-Volumes-Proj-proj-RobotProjects-radio-robot-elite/5339da20-86f6-428a-81b5-3407e531454c/scratchpad/tinyekf/src/tinyekf.h
```

The upstream metadata to embed in the provenance preamble:
- Repository: https://github.com/simondlevy/TinyEKF
- Commit: `ba0d2a90e22f2f4e4120a91cb05ca2ca6a8e8da3`
- License: MIT
- Adaptation note: `EKF_N` and `EKF_M` must be `#define`d by the consumer before
  `#include`; do not add these defines inside tinyekf.h.

Prepend this preamble block at the very top of the output file, before any
original content:

```cpp
// VENDORED: TinyEKF — https://github.com/simondlevy/TinyEKF
// Upstream commit: ba0d2a90e22f2f4e4120a91cb05ca2ca6a8e8da3
// License: MIT
// Adaptation: none. EKF_N and EKF_M must be #defined by the consumer
//   before #include <tinyekf.h>. This project uses EKF_N=5, EKF_M=2.
//   No other changes. To apply an upstream patch, diff against the original
//   and verify EKF_N/EKF_M contracts are preserved.
```

No CMake changes in this ticket. That is ticket 002.

## Acceptance Criteria

- [ ] `libraries/tinyekf/tinyekf.h` exists and begins with the provenance preamble.
- [ ] `libraries/tinyekf/LICENSE` exists (MIT license text from the upstream clone).
- [ ] The body of `tinyekf.h` after the preamble is byte-for-byte identical to the upstream file.
- [ ] No CMakeLists.txt or subdirectories in `libraries/tinyekf/` — header-only vendor, same structure as `libraries/cmon-pid/`.
- [ ] `uv run --with pytest python -m pytest tests/simulation -q` shows no new failures beyond the 2 pre-existing baseline (config-schema tests).

## Implementation Plan

### Approach

Pure file-copy plus preamble prepend. No build-system or source-code changes.

### Files to create

- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/libraries/tinyekf/tinyekf.h`
- `/Volumes/Proj/proj/RobotProjects/radio-robot-elite/libraries/tinyekf/LICENSE`

### Steps

1. Create `libraries/tinyekf/` directory.
2. Read the upstream `tinyekf.h` from the local scratchpad path above.
3. Write `libraries/tinyekf/tinyekf.h` with the provenance preamble prepended, followed by the original content verbatim.
4. Copy the LICENSE file from the same upstream clone directory.
5. Run the test suite to confirm no regressions (the new files are not yet included by anything; build is unchanged).

### Testing plan

**Verification command:** `uv run --with pytest python -m pytest tests/simulation -q`

Expected result: 2 failures (pre-existing config-schema tests), 0 new failures.

No new tests are required for this ticket. The files are not yet used by any build target.

### Documentation updates

None required.
