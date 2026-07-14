---
id: '003'
title: 'Sensor exercise notebook: line-sensor 0-7 count + ch1 index, color sensor,
  OTOS X/Y (drum) + heading (servo)'
status: done
use-cases: []
depends-on: []
github-issue: ''
issue: ''
completes_issue: true
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sensor exercise notebook: line-sensor 0-7 count + ch1 index, color sensor, OTOS X/Y (drum) + heading (servo)

## Description

A **new notebook** exercising the rig's sensors by turning **motor 1** (the drum):

- **Line sensor**: recover the 3-bit binary count on **ch2/ch3/ch4 (0..7)** and
  the **ch1 index** (black once per revolution at count 0). One full 360°
  revolution = **8 counts**. Show the count sequence and the index pulse.
- **Color sensor**: read the painted-wheel colors cycling past as motor 1 turns;
  show distinguishable colors.
- **OTOS**: servo at neutral → drum motion reads mostly on OTOS **X**; servo
  ~+90° → mostly **Y**; a servo sweep moves OTOS **heading**. Show all three.

Depends on ticket 001 (device surface + servo).

## Acceptance Criteria

- [ ] Notebook recovers the line-sensor 0..7 count + ch1 index correctly as
      motor 1 turns; 8 counts/revolution confirmed.
- [ ] Color sensor shows distinguishable colors cycling with motor 1.
- [ ] OTOS X (neutral servo), Y (servo +90°) from the drum, and OTOS heading
      from a servo sweep are all demonstrated.

## Testing

- **HITL (rig)**: execute the notebook over USB; verify count/index/color/OTOS.
