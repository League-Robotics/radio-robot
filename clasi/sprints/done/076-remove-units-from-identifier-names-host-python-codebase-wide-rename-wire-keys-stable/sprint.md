---
id: '076'
title: "Remove units from identifier names \u2014 host Python (codebase-wide rename,\
  \ wire keys stable)"
status: done
branch: sprint/076-remove-units-from-identifier-names-host-python-codebase-wide-rename-wire-keys-stable
use-cases: []
issues:
- remove-units-from-identifier-names-host-python.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 076: Remove units from identifier names — host Python (codebase-wide rename, wire keys stable)

## Goals

(Describe what this sprint aims to accomplish.)

## Problem

(What problem does this sprint address?)

## Solution

(High-level description of the approach.)

## Success Criteria

(How will we know the sprint succeeded?)

## Scope

### In Scope

(List what is included in this sprint.)

### Out of Scope

(List what is explicitly excluded.)

## Test Strategy

(Describe the overall testing approach for this sprint: what types of tests,
what areas need coverage, any integration or system-level testing needed.)

## Architecture Notes

(Key design decisions and constraints.)

## GitHub Issues

(GitHub issues linked to this sprint's tickets. Format: `owner/repo#N`.)

## Definition of Ready

Before tickets can be created, all of the following must be true:

- [ ] Sprint planning documents are complete (sprint.md, use cases, architecture)
- [ ] Architecture review passed
- [ ] Stakeholder has approved the sprint plan

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Transport primitives: rename unit-suffixed parameters in serial and ctypes-sim transport | — |
| 002 | Wire-protocol adapter: rename unit-suffixed parameters in NezhaProtocol/TLMFrame parsing | 001 |
| 003 | Robot object model: rename unit-suffixed identifiers across the Robot/Nezha/Cutebot family | 002 |
| 004 | Sensor modules: rename unit-suffixed identifiers in host-side sensor reading and classification | 002 |
| 005 | Calibration modules: rename unit-suffixed identifiers in calibration workflows | 002 |
| 006 | Navigation modules: rename unit-suffixed identifiers in go-to and path-approach math | 002 |
| 007 | TestGUI core and transport: rename unit-suffixed identifiers in app entry point, transport bridge, and command dispatch | 002 |
| 008 | TestGUI panels and recording: rename unit-suffixed identifiers in sim-error, traces, canvas, and recording glue | 007 |
| 009 | rogo CLI: rename unit-suffixed identifiers in the console-script entry point | 003, 004, 005, 006 |
| 010 | Calibration CLI and MCP surface: rename unit-suffixed identifiers in the calibration wizard and agent-facing tools | 003, 005, 007 |
| 011 | Final sweep: certify zero residual unit-suffixed identifiers, update out-of-package callers, and close the docs status line | 006, 009, 010 |

Tickets execute serially in the order listed. Ticket numbering maps to
`architecture-update.md`'s Step 3 planned-ticket labels as follows: 001→001,
002→002, 003→003, 004→004, 005→005, 006→006, 007→007a, 008→007b, 009→008a,
010→008b, 011→009 (the architecture doc's `a`/`b` suffixes are absorbed into
this sprint's sequential ticket IDs).

**Dependency note on ticket 006**: filed with a dependency on 002. The
architecture doc's Step 4a mermaid diagram omits an explicit `002→006`
edge, but Step 5's "Why" prose states plainly that "tickets 004/005/006 are
mutually independent (each depends only on 002 ...)", consistent with
`usecases.md` SUC-005's Main Flow ("issues drive commands through the
already-renamed ... protocol layer"). This ticketing pass resolved that
internal inconsistency conservatively in favor of the dependency (per
Decision 2's per-ticket keyword-argument-convergence obligation), rather
than reopening architecture review for a non-structural documentation gap.
