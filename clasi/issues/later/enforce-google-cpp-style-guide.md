---
status: pending
---

# Enforce the Google C++ Style Guide across the codebase

The project has adopted the
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
as its C++ coding standard. This is now recorded in the agent
instructions (`CLAUDE.md` and `AGENTS.md`), so all agents writing or
reviewing C++ are bound to it going forward.

## Scope

1. **Audit existing C++ code for conformance.** Sweep `source/`,
   `wedgelab/`, and C++ test code for violations of the guide —
   naming, formatting, header layout (include order, guards),
   and idiom — and bring nonconforming code into line.
2. **Keep it enforced.** New and touched C++ code must conform to the
   guide. Reviews (code-review skill, programmer agents) should check
   conformance as part of their quality pass. When editing code that
   predates this rule, bring the touched code into conformance rather
   than matching the old style.

## Notes

- Consider adding tooling support (e.g. a `.clang-format` based on the
  Google style, and/or `cpplint`) so conformance is checkable rather
  than judgment-based.
- The audit portion is mechanical and repo-wide; it should be planned
  as its own sprint work rather than done opportunistically.
