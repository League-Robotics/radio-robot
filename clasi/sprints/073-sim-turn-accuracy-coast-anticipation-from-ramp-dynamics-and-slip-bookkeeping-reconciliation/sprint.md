---
id: '073'
title: 'Sim turn accuracy: coast anticipation from ramp dynamics and slip bookkeeping
  reconciliation'
status: planning-docs
branch: sprint/073-sim-turn-accuracy-coast-anticipation-from-ramp-dynamics-and-slip-bookkeeping-reconciliation
use-cases: []
issues:
- sim-turn-undershoot.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 073: Sim turn accuracy: coast anticipation from ramp dynamics and slip bookkeeping reconciliation

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
| 001 | RT coast anticipation from ramp dynamics | none |
| 002 | Sim plant scrub reconciliation | none |
| 003 | TestGUI default-profile reconciliation | 002 |
| 004 | Regression sweep + Tour-1 xfail removal | 001, 002, 003 |

Tickets execute serially in the order listed.
