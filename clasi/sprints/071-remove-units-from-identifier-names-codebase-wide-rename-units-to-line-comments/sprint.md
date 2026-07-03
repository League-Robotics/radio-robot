---
id: '071'
title: Remove units from identifier names (codebase-wide rename; units to line comments)
status: planning-docs
branch: sprint/071-remove-units-from-identifier-names-codebase-wide-rename-units-to-line-comments
use-cases: []
issues:
- remove-units-from-identifier-names.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 071: Remove units from identifier names (codebase-wide rename; units to line comments)

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
| 001 | Coding-standards convention doc: document unit-comment convention | — |
| 002 | RobotConfig field renames: strip unit suffixes from calibration/timing/geometry fields | 001 |
| 003 | DesiredState/OutputState field renames: strip unit suffixes from commanded and actuator-output state | 001 |
| 004 | Proto-generated message field renames: drivetrain/motor/planner proto and codegen | 002 |
| 005 | Estimation, motion, and goal-closure identifier sweep | 002, 003, 004 |
| 006 | Real-hardware HAL identifier sweep: Motor, OtosSensor, NezhaHAL | 002 |
| 007 | Sim-library identifier sweep: PhysicsWorld, SimOdometer, SimSetters, SimCommands | 002 |
| 008 | Final sweep, docs update, and sprint closure verification | 005, 006, 007 |

Tickets execute serially in the order listed.
