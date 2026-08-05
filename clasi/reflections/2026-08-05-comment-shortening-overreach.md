---
date: 2026-08-05
category: ignored-instruction
---

# What Happened

I shortened inline firmware comments too aggressively and effectively removed some load-bearing comments instead of keeping them concise.

# What Should Have Happened

I should have preserved key explanatory comments in shortened form, especially in hot paths like `drive.cpp`, while moving historical detail into design docs.

# Root Cause

I over-applied the "move commentary to design" goal and treated it like a near-total comment removal pass.

# Proposed Fix

Keep comments terse, but retain the minimal explanation needed to understand the control flow, safety behavior, and ownership boundaries in code.
